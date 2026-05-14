#include "ros_compat.h"
#include "image_converter.h"
#include <jetson-utils/videoSource.h>

#include <thread>
#include <vector>
#include <chrono>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

// Global state
videoSource* stream = NULL;
imageConverter* image_cvt = NULL;

std::shared_ptr<image_transport::ImageTransport> image_transport_;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
image_transport::Publisher image_pub;

std::thread* capture_thread = NULL;
bool should_exit = false;

// Message pool
std::vector<sensor_msgs::msg::Image> image_pool;
size_t current_msg_index = 0;
int pool_size = 3;

// Stream / reconnect params
std::string resource_str;
videoOptions video_options;
int reconnect_delay_ms = 2000;
int max_consecutive_failures = 5;

// Output resolution
int output_width = 640;
int output_height = 360;

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

bool acquireFrame()
{
    if (!stream || !stream->IsStreaming())
        return false;

    imageConverter::PixelType* nextFrame = NULL;

    if (!stream->Capture(&nextFrame, 100))
        return false;

    sensor_msgs::msg::Image& msg = image_pool[current_msg_index];
    current_msg_index = (current_msg_index + 1) % image_pool.size();

    const int width = stream->GetWidth();
    const int height = stream->GetHeight();

    if (!image_cvt->Resize(width, height, imageConverter::ROSOutputFormat)) {
        ROS_ERROR("failed to resize camera image converter");
        return false;
    }

    if (!image_cvt->Convert(msg, imageConverter::ROSOutputFormat, nextFrame)) {
        ROS_ERROR("failed to convert video stream frame to sensor_msgs::Image");
        return false;
    }

    if (stream->GetLastTimestamp() != 0)
        msg.header.stamp = rclcpp::Time(stream->GetLastTimestamp());
    else
        msg.header.stamp = ROS_TIME_NOW();

    msg.header.frame_id = "camera";

    // Only resize when needed
    if (msg.width != static_cast<uint32_t>(output_width) ||
        msg.height != static_cast<uint32_t>(output_height))
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(output_width, output_height), 0, 0, cv::INTER_AREA);
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
    ROS_CREATE_NODE("optimized_video_source");

#ifdef __linux__
    system("(command -v nvpmodel && sudo nvpmodel -m 0) > /dev/null 2>&1 || true");
    system("(command -v jetson_clocks && sudo jetson_clocks) > /dev/null 2>&1 || true");
#endif

    std::string codec_str;
    std::string flip_str;

    // Better defaults for your use-case:
    // native-ish 16:9 capture + smaller published output
    int video_width = 1280;
    int video_height = 720;
    int latency = video_options.latency;
    video_options.frameRate = 30.0f;

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
    ROS_DECLARE_PARAMETER("output_width", output_width);
    ROS_DECLARE_PARAMETER("output_height", output_height);

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
    ROS_GET_PARAMETER("output_width", output_width);
    ROS_GET_PARAMETER("output_height", output_height);

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

    ROS_INFO("Opening video source: %s", resource_str.c_str());
    ROS_INFO("Capture dimensions: %dx%d at %.1f FPS", video_width, video_height, video_options.frameRate);
    ROS_INFO("Output (published) dimensions: %dx%d", output_width, output_height);

    image_cvt = new imageConverter();
    if (!image_cvt) {
        ROS_ERROR("Failed to create imageConverter");
        return 0;
    }

    // Lowest-latency ROS settings for live camera over Wi-Fi
    rclcpp::QoS qos_settings(rclcpp::KeepLast(1));
    qos_settings.best_effort();

    image_transport_ = std::make_shared<image_transport::ImageTransport>(node);
    image_pub = image_transport_->advertise("/image_raw", 1);
    camera_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/image_raw/camera_info", qos_settings);

    image_pool.resize(pool_size);
    for (auto& msg : image_pool)
        msg.header.frame_id = "camera";

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
