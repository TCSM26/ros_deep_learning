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
int reconnect_delay_ms = 2000;  // 2 seconds between reconnection attempts
int max_consecutive_failures = 5;

// NEW: published-output (resized) size
int output_width = 0;   // 0 = no resize, publish at capture resolution
int output_height = 0;

// Publish rate cap. 0 (or negative) = unlimited (publish every captured frame).
// When > 0, frames captured faster than this rate are dropped before resize/convert/publish.
double publish_rate = 0.0;
std::chrono::steady_clock::time_point last_publish_time;

// Camera calibration state. The raw image is still distorted, so CameraInfo
// publishes the original distortion coefficients with K scaled to output size.
std::string calibration_file;       // YAML produced by scripts/calibration_npz_to_yaml.py
std::string calibration_model = "plumb_bob";
cv::Mat camera_matrix;              // calibration K (at calibration resolution)
cv::Mat dist_coeffs;                // distortion coefficients
int calib_width = 0;                // resolution K was estimated at
int calib_height = 0;
bool calibration_loaded = false;
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
    fs["model"]        >> calibration_model;
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
    if (calibration_model.empty()) {
        calibration_model = "plumb_bob";
    }
    calibration_loaded = true;

    ROS_INFO("Loaded %s calibration from %s (calibrated at %dx%d)",
             calibration_model.c_str(), path.c_str(), calib_width, calib_height);
    return true;
}

// Build CameraInfo once for the distorted/raw image being published.
void buildCameraInfo(int published_width, int published_height)
{
    if (!calibration_loaded) {
        return;
    }

    const double sx = static_cast<double>(published_width)  / calib_width;
    const double sy = static_cast<double>(published_height) / calib_height;

    cv::Mat K_scaled = camera_matrix.clone();
    K_scaled.at<double>(0, 0) *= sx;  // fx
    K_scaled.at<double>(1, 1) *= sy;  // fy
    K_scaled.at<double>(0, 2) *= sx;  // cx
    K_scaled.at<double>(1, 2) *= sy;  // cy

    cached_camera_info.width = published_width;
    cached_camera_info.height = published_height;
    cached_camera_info.distortion_model =
        (calibration_model == "fisheye") ? "equidistant" : calibration_model;
    cached_camera_info.d.resize(static_cast<size_t>(dist_coeffs.total()));
    const double* dist_data = dist_coeffs.ptr<double>(0);
    for (size_t i = 0; i < cached_camera_info.d.size(); ++i) {
        cached_camera_info.d[i] = dist_data[i];
    }

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            cached_camera_info.k[r * 3 + c] = K_scaled.at<double>(r, c);
        }
    }

    cached_camera_info.r = {1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0};
    cached_camera_info.p = {
        K_scaled.at<double>(0, 0), 0.0,
            K_scaled.at<double>(0, 2), 0.0,
        0.0, K_scaled.at<double>(1, 1),
            K_scaled.at<double>(1, 2), 0.0,
        0.0, 0.0, 1.0, 0.0
    };

    ROS_INFO("CameraInfo cached for published size %dx%d",
             published_width, published_height);
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

// acquire and publish camera frame - optimized version with error handling
bool acquireFrame()
{
    if (!stream || !stream->IsStreaming()) {
        return false;
    }

    imageConverter::PixelType* nextFrame = NULL;

    if (!stream->Capture(&nextFrame, 100)) {
        return false;
    }

    // Rate cap: drop the frame *before* the expensive convert/resize/publish path.
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

    // NEW: optional CPU resize to published-output size.
    // Skipped when output dims are 0 or already match the captured frame.
    if (output_width > 0 && output_height > 0 &&
        (msg.width  != static_cast<uint32_t>(output_width) ||
         msg.height != static_cast<uint32_t>(output_height)))
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat resized;
        cv::resize(frame, resized,
                   cv::Size(output_width, output_height),
                   0, 0, cv::INTER_AREA);
        msg = *cv_bridge::CvImage(msg.header, "bgr8", resized).toImageMsg();
    }

    image_pub.publish(std::make_shared<sensor_msgs::msg::Image>(msg));

    sensor_msgs::msg::CameraInfo info_msg = cached_camera_info;
    info_msg.header = msg.header;
    camera_info_pub->publish(info_msg);

    return true;
}

// Thread function for continuous frame capture with reconnection logic
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
    bool stream_initialized = false;   // CHANGED: was 'true', which produced a misleading "Stream lost" log on startup

    while (!should_exit) {
        // Check if we need to reconnect
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

        // Try to acquire frame
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
    ROS_CREATE_NODE("optimized_video_source");

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
    ROS_DECLARE_PARAMETER("output_width", output_width);     // NEW
    ROS_DECLARE_PARAMETER("output_height", output_height);   // NEW
    ROS_DECLARE_PARAMETER("publish_rate", publish_rate);     // Hz cap, 0 = unlimited
    ROS_DECLARE_PARAMETER("calibration_file", calibration_file);

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
    ROS_GET_PARAMETER("output_width", output_width);         // NEW
    ROS_GET_PARAMETER("output_height", output_height);       // NEW
    ROS_GET_PARAMETER("publish_rate", publish_rate);
    ROS_GET_PARAMETER("calibration_file", calibration_file);

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
        ROS_ERROR("output_width and output_height must be set for cached CameraInfo");
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

    buildCameraInfo(output_width, output_height);

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

    // Best-effort, small queue = lowest latency (drop stale frames rather than buffer them)
    rclcpp::QoS qos_settings(rclcpp::KeepLast(1));
    qos_settings.best_effort();

    image_transport_ = std::make_shared<image_transport::ImageTransport>(node);
    image_pub = image_transport_->advertise("/image_raw", 1);
    camera_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/image_raw/camera_info", qos_settings);

    image_pool.resize(pool_size);
    for (auto& msg : image_pool) {
        msg.header.frame_id = "camera";   // CHANGED: was "map", which is a TF nav frame, not a camera frame
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
