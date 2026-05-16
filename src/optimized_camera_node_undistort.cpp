#include "ros_compat.h"
#include "image_converter.h"
#include <jetson-utils/videoSource.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <chrono>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

// Global variables
videoSource* stream = NULL;
imageConverter* image_cvt = NULL;

std::shared_ptr<image_transport::ImageTransport> image_transport_;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
image_transport::Publisher image_pub;

// Thread management
std::thread* capture_thread = NULL;
bool should_exit = false;
std::mutex frame_mutex;
std::condition_variable frame_cv;

// Pre-allocated image message pool
std::vector<sensor_msgs::msg::Image> image_pool;
size_t current_msg_index = 0;
int pool_size = 3;

// Reconnection parameters
std::string resource_str;
videoOptions video_options;
int reconnect_delay_ms = 2000;
int max_consecutive_failures = 5;

// Published-output (resized) size
int output_width = 0;
int output_height = 0;

// Publish rate cap. 0 (or negative) = unlimited (publish every captured frame).
// When > 0, frames captured faster than this rate are dropped before resize/convert/undistort/publish.
double publish_rate = 0.0;
std::chrono::steady_clock::time_point last_publish_time;

// --- Undistortion state ---
std::string calibration_file;       // YAML produced by scripts/calibration_npz_to_yaml.py
double undistort_alpha = 0.0;       // 0 = crop, 1 = keep full FoV
cv::Mat camera_matrix;              // calibration K (at calibration resolution)
cv::Mat dist_coeffs;                // distortion coefficients
int calib_width = 0;                // resolution K was estimated at
int calib_height = 0;

cv::Mat undistort_map1;             // CV_16SC2 remap LUTs
cv::Mat undistort_map2;
cv::Mat new_camera_matrix;          // K for the published, undistorted image
sensor_msgs::msg::CameraInfo cached_camera_info;

// Reads K, dist, image_width, image_height from a cv::FileStorage YAML.
bool loadCalibration(const std::string& path)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("Could not open calibration file: %s", path.c_str());
        return false;
    }

    fs["K"]            >> camera_matrix;
    fs["dist"]         >> dist_coeffs;
    fs["image_width"]  >> calib_width;
    fs["image_height"] >> calib_height;
    fs.release();

    if (camera_matrix.empty() || dist_coeffs.empty() ||
        calib_width <= 0 || calib_height <= 0)
    {
        ROS_ERROR("Calibration file is missing K / dist / image_size: %s", path.c_str());
        return false;
    }

    camera_matrix.convertTo(camera_matrix, CV_64F);
    dist_coeffs.convertTo(dist_coeffs, CV_64F);
    dist_coeffs = dist_coeffs.reshape(1, 1);

    ROS_INFO("Loaded calibration from %s (calibrated at %dx%d)",
             path.c_str(), calib_width, calib_height);
    return true;
}

// K must be scaled when the published image size differs from the
// resolution the calibration was estimated at. Distortion coefficients
// are dimensionless and do not change.
void buildUndistortMaps(int published_width, int published_height)
{
    const double sx = static_cast<double>(published_width)  / calib_width;
    const double sy = static_cast<double>(published_height) / calib_height;

    cv::Mat K_scaled = camera_matrix.clone();
    K_scaled.at<double>(0, 0) *= sx;  // fx
    K_scaled.at<double>(1, 1) *= sy;  // fy
    K_scaled.at<double>(0, 2) *= sx;  // cx
    K_scaled.at<double>(1, 2) *= sy;  // cy

    const cv::Size size(published_width, published_height);
    new_camera_matrix = cv::getOptimalNewCameraMatrix(
        K_scaled, dist_coeffs, size, undistort_alpha, size);

    cv::initUndistortRectifyMap(
        K_scaled, dist_coeffs, cv::Mat(),
        new_camera_matrix, size, CV_16SC2,
        undistort_map1, undistort_map2);

    cached_camera_info.width = published_width;
    cached_camera_info.height = published_height;
    cached_camera_info.distortion_model = "plumb_bob";
    cached_camera_info.d.assign(5, 0.0);  // image is already undistorted

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            cached_camera_info.k[r * 3 + c] = new_camera_matrix.at<double>(r, c);
        }
    }

    cached_camera_info.r = {1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0};
    cached_camera_info.p = {
        new_camera_matrix.at<double>(0, 0), 0.0,
            new_camera_matrix.at<double>(0, 2), 0.0,
        0.0, new_camera_matrix.at<double>(1, 1),
            new_camera_matrix.at<double>(1, 2), 0.0,
        0.0, 0.0, 1.0, 0.0
    };

    ROS_INFO("Undistort maps and CameraInfo cached for published size %dx%d (alpha=%.2f)",
             published_width, published_height, undistort_alpha);
}

// Initialize or reinitialize the video stream
bool initializeStream()
{
    if (stream) {
        delete stream;
        stream = NULL;
    }

    ROS_INFO("Attempting to open video source: %s", resource_str.c_str());
    stream = videoSource::Create(resource_str.c_str(), video_options);

    if (!stream) {
        ROS_ERROR("Failed to create video source");
        return false;
    }

    if (!stream->Open()) {
        ROS_ERROR("Failed to open video stream");
        delete stream;
        stream = NULL;
        return false;
    }

    ROS_INFO("Video stream opened successfully");
    return true;
}

// acquire, undistort, and publish a frame
bool acquireFrame()
{
    if (!stream || !stream->IsStreaming()) {
        return false;
    }

    imageConverter::PixelType* nextFrame = NULL;

    if (!stream->Capture(&nextFrame, 100)) {
        return false;
    }

    // Rate cap: drop the frame *before* the expensive convert/resize/undistort/publish path.
    // We still call Capture() above so the GStreamer pipeline doesn't back up.
    if (publish_rate > 0.0) {
        const auto now = std::chrono::steady_clock::now();
        const auto min_interval = std::chrono::duration<double>(1.0 / publish_rate);
        if (now - last_publish_time < min_interval) {
            return true;   // frame consumed but intentionally skipped
        }
        last_publish_time = now;
    }

    sensor_msgs::msg::Image& msg = image_pool[current_msg_index];
    current_msg_index = (current_msg_index + 1) % image_pool.size();

    int width = stream->GetWidth();
    int height = stream->GetHeight();

    if (!image_cvt->Resize(width, height, imageConverter::ROSOutputFormat)) {
        ROS_ERROR("failed to resize camera image converter");
        return false;
    }

    if (!image_cvt->Convert(msg, imageConverter::ROSOutputFormat, nextFrame)) {
        ROS_ERROR("failed to convert video stream frame to sensor_msgs::Image");
        return false;
    }

    if (stream->GetLastTimestamp() != 0) {
        msg.header.stamp = rclcpp::Time(stream->GetLastTimestamp());
    } else {
        msg.header.stamp = ROS_TIME_NOW();
    }
    msg.header.frame_id = "camera";

    // CPU path from here: resize (if requested), then undistort.
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    if (output_width > 0 && output_height > 0 &&
        (frame.cols != output_width || frame.rows != output_height))
    {
        cv::Mat resized;
        cv::resize(frame, resized,
                   cv::Size(output_width, output_height),
                   0, 0, cv::INTER_AREA);
        frame = resized;
    }

    cv::Mat undistorted;
    cv::remap(frame, undistorted, undistort_map1, undistort_map2, cv::INTER_LINEAR);

    msg = *cv_bridge::CvImage(msg.header, "bgr8", undistorted).toImageMsg();
    image_pub.publish(std::make_shared<sensor_msgs::msg::Image>(msg));

    sensor_msgs::msg::CameraInfo info_msg = cached_camera_info;
    info_msg.header = msg.header;
    camera_info_pub->publish(info_msg);

    return true;
}

void captureThreadFunc()
{
    ROS_INFO("Capture thread started");

    #ifdef __linux__
    struct sched_param param;
    param.sched_priority = 90;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        ROS_INFO("Failed to set high priority for capture thread.");
    }
    #endif

    int consecutive_failures = 0;
    bool stream_initialized = false;

    while (!should_exit) {
        if (!stream || !stream->IsStreaming()) {
            if (stream_initialized) {
                ROS_ERROR("Stream lost, attempting to reconnect...");
                stream_initialized = false;
            }

            if (initializeStream()) {
                ROS_INFO("Successfully reconnected to stream");
                consecutive_failures = 0;
                stream_initialized = true;
            } else {
                consecutive_failures++;
                if (consecutive_failures >= max_consecutive_failures) {
                    ROS_ERROR("Failed to reconnect after %d attempts, waiting longer...", consecutive_failures);
                    std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms * 3));
                    consecutive_failures = 0;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms));
                }
                continue;
            }
        }

        if (!acquireFrame()) {
            if (stream && stream->IsStreaming()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } else {
                ROS_ERROR("Frame acquisition failed, stream may be closed");
                stream_initialized = false;
            }
        } else {
            consecutive_failures = 0;
        }
    }

    ROS_INFO("Capture thread exiting");
}

int main(int argc, char **argv)
{
    ROS_CREATE_NODE("optimized_video_source_undistort");

    std::string codec_str;
    std::string flip_str;

    int video_width = video_options.width;
    int video_height = video_options.height;
    int latency = video_options.latency;
    int sensor_mode = -1;

    ROS_DECLARE_PARAMETER("resource", resource_str);
    ROS_DECLARE_PARAMETER("codec", codec_str);
    ROS_DECLARE_PARAMETER("width", video_width);
    ROS_DECLARE_PARAMETER("height", video_height);
    ROS_DECLARE_PARAMETER("framerate", video_options.frameRate);
    ROS_DECLARE_PARAMETER("loop", video_options.loop);
    ROS_DECLARE_PARAMETER("flip", flip_str);
    ROS_DECLARE_PARAMETER("latency", latency);
    ROS_DECLARE_PARAMETER("pool_size", pool_size);
    ROS_DECLARE_PARAMETER("reconnect_delay_ms", reconnect_delay_ms);
    ROS_DECLARE_PARAMETER("max_consecutive_failures", max_consecutive_failures);
    ROS_DECLARE_PARAMETER("sensor_mode", sensor_mode);
    ROS_DECLARE_PARAMETER("output_width", output_width);
    ROS_DECLARE_PARAMETER("output_height", output_height);
    ROS_DECLARE_PARAMETER("calibration_file", calibration_file);
    ROS_DECLARE_PARAMETER("undistort_alpha", undistort_alpha);
    ROS_DECLARE_PARAMETER("publish_rate", publish_rate);     // Hz cap, 0 = unlimited

    ROS_GET_PARAMETER("resource", resource_str);
    ROS_GET_PARAMETER("codec", codec_str);
    ROS_GET_PARAMETER("width", video_width);
    ROS_GET_PARAMETER("height", video_height);
    ROS_GET_PARAMETER("framerate", video_options.frameRate);
    ROS_GET_PARAMETER("loop", video_options.loop);
    ROS_GET_PARAMETER("flip", flip_str);
    ROS_GET_PARAMETER("latency", latency);
    ROS_GET_PARAMETER("pool_size", pool_size);
    ROS_GET_PARAMETER("reconnect_delay_ms", reconnect_delay_ms);
    ROS_GET_PARAMETER("max_consecutive_failures", max_consecutive_failures);
    ROS_GET_PARAMETER("sensor_mode", sensor_mode);
    ROS_GET_PARAMETER("output_width", output_width);
    ROS_GET_PARAMETER("output_height", output_height);
    ROS_GET_PARAMETER("calibration_file", calibration_file);
    ROS_GET_PARAMETER("undistort_alpha", undistort_alpha);
    ROS_GET_PARAMETER("publish_rate", publish_rate);

    if (resource_str.empty()) {
        ROS_ERROR("resource param wasn't set - please set the node's resource parameter");
        return 0;
    }

    if (calibration_file.empty()) {
        ROS_ERROR("calibration_file param wasn't set - convert the .npz with scripts/calibration_npz_to_yaml.py first");
        return 0;
    }

    if (!loadCalibration(calibration_file)) {
        return 0;
    }

    if (output_width <= 0 || output_height <= 0) {
        ROS_ERROR("output_width and output_height must be set for cached undistort maps and CameraInfo");
        return 0;
    }

    if (!codec_str.empty())
        video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

    if (!flip_str.empty())
        video_options.flipMethod = videoOptions::FlipMethodFromStr(flip_str.c_str());

    video_options.width = video_width;
    video_options.height = video_height;
    video_options.latency = latency;
    video_options.zeroCopy = true;
    video_options.sensorMode = sensor_mode;

    buildUndistortMaps(output_width, output_height);

    ROS_INFO("Capture dimensions: %dx%d at %.1f FPS",
             video_width, video_height, video_options.frameRate);
    if (output_width > 0 && output_height > 0)
        ROS_INFO("Published (output) dimensions: %dx%d", output_width, output_height);
    else
        ROS_INFO("Published (output) dimensions: same as capture");

    if (publish_rate > 0.0)
        ROS_INFO("Publish rate cap: %.1f Hz", publish_rate);
    else
        ROS_INFO("Publish rate cap: unlimited");

    image_cvt = new imageConverter();
    if (!image_cvt) {
        ROS_ERROR("Failed to create imageConverter");
        return 0;
    }

    rclcpp::QoS qos_settings(rclcpp::KeepLast(1));
    qos_settings.best_effort();

    image_transport_ = std::make_shared<image_transport::ImageTransport>(node);
    image_pub = image_transport_->advertise("image_raw", 1);
    camera_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(
        "image_raw/camera_info", qos_settings);

    image_pool.resize(pool_size);
    for (auto& msg : image_pool) {
        msg.header.frame_id = "camera";
    }

    should_exit = false;
    capture_thread = new std::thread(captureThreadFunc);

    while (ROS_OK()) {
        ROS_SPIN_ONCE();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    should_exit = true;
    if (capture_thread) {
        if (capture_thread->joinable())
            capture_thread->join();
        delete capture_thread;
    }

    delete stream;
    delete image_cvt;

    return 0;
}
