#include "ros_compat.h"
#include "image_converter.h"
#include <gst/gst.h>

#include <jetson-utils/videoSource.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <chrono>

// Global variables	
videoSource* stream = NULL;
imageConverter* image_cvt = NULL;
Publisher<sensor_msgs::Image> image_pub = NULL;

/*#ifdef ROS2*/
/*Publisher<sensor_msgs::msg::CameraInfo> camera_info_pub = NULL;*/
/*#else*/
/*Publisher<sensor_msgs::CameraInfo> camera_info_pub = NULL;*/
/*#endif*/

// Thread management
std::thread* capture_thread = NULL;
bool should_exit = false;
std::mutex frame_mutex;
std::condition_variable frame_cv;

// Pre-allocated image message pool
std::vector<sensor_msgs::Image> image_pool;
size_t current_msg_index = 0;
int pool_size = 4;

// ROI configuration
bool use_roi = false;
int roi_x = 0;
int roi_y = 0;
int roi_width = 640;
int roi_height = 480;


// acquire and publish camera frame - optimized version
bool acquireFrame()
{
    imageConverter::PixelType* nextFrame = NULL;

    // get the latest frame with timeout
    if (!stream->Capture(&nextFrame, 100)) {
        return false;
    }

    // Get image from pre-allocated pool
    sensor_msgs::Image& msg = image_pool[current_msg_index];
    current_msg_index = (current_msg_index + 1) % image_pool.size();

    // Apply ROI if enabled - this would need specific handling in the converter
    int width = stream->GetWidth();
    int height = stream->GetHeight();
    
    // assure correct image size
    if (!image_cvt->Resize(width, height, imageConverter::ROSOutputFormat)) {
        ROS_ERROR("failed to resize camera image converter");
        return false;
    }

    // populate the message
    if (!image_cvt->Convert(msg, imageConverter::ROSOutputFormat, nextFrame)) {
        ROS_ERROR("failed to convert video stream frame to sensor_msgs::Image");
        return false;
    }

    // populate timestamp in header field
    msg.header.stamp = ROS_TIME_NOW();

    // publish the image
    image_pub->publish(msg);
    
    // publish camera info if available
    /*#ifdef ROS2*/
    /*auto info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();*/
    /*info_msg->header = msg.header;*/
    /*info_msg->width = width;*/
    /*info_msg->height = height;*/
    /*camera_info_pub->publish(*info_msg);*/
    /*#else*/
    /*sensor_msgs::CameraInfo info_msg;*/
    /*info_msg.header = msg.header;*/
    /*info_msg.width = width;*/
    /*info_msg.height = height;*/
    /*camera_info_pub->publish(info_msg);*/
    /*#endif*/
    
    return true;
}

// Thread function for continuous frame capture
void captureThreadFunc()
{
    ROS_INFO("Capture thread started");
    
    // Set thread priority if running with sufficient privileges
    #ifdef __linux__
    struct sched_param param;
    param.sched_priority = 90; // High real-time priority
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        ROS_INFO("Failed to set high priority for capture thread. Consider running with sudo or setting capabilities.");
    }
    #endif
    
    while (!should_exit) {
        // Capture and publish frame
        if (!acquireFrame()) {
            // If capture failed but stream is still open, wait a bit
            if (stream->IsStreaming()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } else {
                ROS_INFO("Stream is closed or reached EOS");
                break;
            }
        }
    }
    
    ROS_INFO("Capture thread exiting");
}

// node main loop
int main(int argc, char **argv)
{
    gst_init(&argc, &argv);

    const char* pipeline_str =
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
        "nvv4l2h264enc insert-sps-pps=true idrinterval=30 bitrate=4000000 ! "
        "h264parse ! "
        "rtph264pay config-interval=1 pt=96 ! "
        "udpsink host=192.168.1.100 port=5000";

    GError* err = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_str, &err);

    if (!pipeline) {
        g_printerr("Failed to create pipeline: %s\n", err->message);
        return -1;
    }

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_print("Streaming started...\n");

    // Run until interrupted
    GMainLoop* loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(loop);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}
