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

    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header = msg.header;
    info_msg.width = msg.width;
    info_msg.height = msg.height;
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

    if (resource_str.empty()) {
        ROS_ERROR("resource param wasn't set - please set the node's resource parameter");
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

    ROS_INFO("Capture dimensions: %dx%d at %.1f FPS",
             video_width, video_height, video_options.frameRate);
    if (output_width > 0 && output_height > 0)
        ROS_INFO("Published (output) dimensions: %dx%d", output_width, output_height);
    else
        ROS_INFO("Published (output) dimensions: same as capture");

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