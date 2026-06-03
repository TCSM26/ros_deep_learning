// optimized_camera_node_undistort_qr.cpp
//
// Based on optimized_camera_node_undistort.cpp. Preserves the exact capture /
// undistort / downsize / publish pipeline of that node and ADDS local QR
// detection, decoding and relative-pose estimation that runs on the original
// full-sized camera frame (before downsizing), in a dedicated worker thread so
// it never stalls image publishing.
//
// QR detection is controlled at runtime through the /qr_enable topic
// (std_msgs/Int32): publish 1 to enable, 0 to disable. The default state is set
// by the qr_enabled_default parameter (disabled unless overridden).

#include "ros_compat.h"
#include "image_converter.h"
#include <jetson-utils/videoSource.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <queue>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
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
std::string calibration_model = "plumb_bob";
double undistort_alpha = 0.0;       // 0 = crop, 1 = keep full FoV
cv::Mat camera_matrix;              // calibration K (at calibration resolution)
cv::Mat dist_coeffs;                // distortion coefficients
int calib_width = 0;                // resolution K was estimated at
int calib_height = 0;

cv::Mat undistort_map1;             // CV_16SC2 remap LUTs
cv::Mat undistort_map2;
cv::Mat new_camera_matrix;          // K for the published, undistorted image
sensor_msgs::msg::CameraInfo cached_camera_info;

// --- QR detection state ---------------------------------------------------
// Default-disabled. Toggled live via the /qr_enable subscriber.
std::atomic<bool> qr_enabled(false);
bool qr_enabled_default = false;

std::string qr_enable_topic  = "/qr_enable";
std::string qr_data_topic    = "/qr/data";
std::string qr_pose_topic    = "/qr/pose";
std::string qr_corners_topic = "/qr/corners_downsized";

double qr_size_m = 0.10;              // physical QR side length, metres
int qr_process_every_n_frames = 6;   // run detection on 1 of every N captured frames
long qr_frame_counter = 0;           // touched only by the capture thread

// Resolution the QR calibration matrix (K) actually corresponds to. The shipped
// calibration was estimated on the DOWNSIZED image, so its fx/fy/cx/cy are valid
// only at this resolution -- they must NOT be used as-is with full-size QR
// corners. These default to the size read from the calibration YAML, but can be
// overridden with the calibration_width / calibration_height parameters when the
// YAML does not record the true calibration resolution.
int calibration_width = 0;
int calibration_height = 0;

// Camera intrinsics for the *full-sized raw* frame the QR detector runs on.
// QR detection uses the raw (still-distorted) full-resolution image, so it must
// use a full-resolution camera matrix and the real distortion coefficients.
// SAFETY: image points (QR corners) and camera intrinsics passed to solvePnP
// must correspond to the SAME image resolution. qr_camera_matrix below is K
// scaled from the calibration resolution up to the full frame size so it matches
// the full-size corners; the downsized calibration K is never used for full-size
// pose estimation.
cv::Mat qr_camera_matrix;            // calibration K scaled to the full frame size
cv::Mat qr_dist_coeffs;              // distortion coefficients (unchanged)
int qr_full_width = 0;               // full-frame size qr_camera_matrix was built for
int qr_full_height = 0;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_data_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr qr_pose_pub;
rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr qr_corners_pub;
rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr qr_enable_sub;

cv::QRCodeDetector qr_detector;

// QR worker thread + latest-frame hand-off slot. The capture thread clones the
// newest full-sized frame into qr_pending_frame and notifies the worker; the
// worker always processes the most recent frame (older un-processed frames are
// simply dropped). Cloning means capture and detection never touch the same
// pixel buffer, so no extra locking is needed during detection.
std::thread* qr_thread = NULL;
std::mutex qr_mutex;
std::condition_variable qr_cv;
cv::Mat qr_pending_frame;
bool qr_has_pending = false;
bool qr_worker_exit = false;

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

    ROS_INFO("Loaded %s calibration from %s (calibrated at %dx%d)",
             calibration_model.c_str(), path.c_str(), calib_width, calib_height);
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
    if (calibration_model == "fisheye" || calibration_model == "equidistant") {
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
            K_scaled, dist_coeffs, size, cv::Matx33d::eye(),
            new_camera_matrix, undistort_alpha, size);

        cv::fisheye::initUndistortRectifyMap(
            K_scaled, dist_coeffs, cv::Matx33d::eye(),
            new_camera_matrix, size, CV_16SC2,
            undistort_map1, undistort_map2);
    } else {
        new_camera_matrix = cv::getOptimalNewCameraMatrix(
            K_scaled, dist_coeffs, size, undistort_alpha, size);

        cv::initUndistortRectifyMap(
            K_scaled, dist_coeffs, cv::Mat(),
            new_camera_matrix, size, CV_16SC2,
            undistort_map1, undistort_map2);
    }

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

// --- QR helpers -----------------------------------------------------------

// Build the camera matrix for the full-sized raw frame QR detection runs on.
//
// The calibration K was estimated on the (downsized) calibration image of size
// calibration_width x calibration_height, so fx,fy,cx,cy are only valid at that
// resolution. QR corners, however, come from the full-size frame
// (full_w x full_h). To make the intrinsics correspond to the SAME resolution as
// the image points handed to solvePnP, scale K up to the full frame:
//
//   scale_x = full_width  / calibration_width
//   scale_y = full_height / calibration_height
//   fx_full = fx_calib * scale_x,  cx_full = cx_calib * scale_x
//   fy_full = fy_calib * scale_y,  cy_full = cy_calib * scale_y
//
// Distortion coefficients are intensive (dimensionless, normalized) quantities;
// they are unchanged by a pure resize, so they are kept as-is. This assumes the
// image was only RESIZED, not cropped. If the full frame were a crop of the
// calibration field of view, cx/cy would need a translation, not just a scale.
//
// SAFETY: never call solvePnP with full-size corners and the un-scaled downsized
// calibration matrix -- image points and intrinsics must be at one resolution.
void buildQrCameraMatrix(int full_w, int full_h)
{
    const double sx = static_cast<double>(full_w) / calibration_width;
    const double sy = static_cast<double>(full_h) / calibration_height;

    qr_camera_matrix = camera_matrix.clone();
    qr_camera_matrix.at<double>(0, 0) *= sx;  // fx_full = fx_calib * scale_x
    qr_camera_matrix.at<double>(1, 1) *= sy;  // fy_full = fy_calib * scale_y
    qr_camera_matrix.at<double>(0, 2) *= sx;  // cx_full = cx_calib * scale_x
    qr_camera_matrix.at<double>(1, 2) *= sy;  // cy_full = cy_calib * scale_y

    qr_dist_coeffs = dist_coeffs.clone();     // unchanged: resize, not crop
    qr_full_width  = full_w;
    qr_full_height = full_h;

    ROS_INFO("QR camera matrix scaled from calib %dx%d to full frame %dx%d "
             "(scale_x=%.4f, scale_y=%.4f, raw image with distortion)",
             calibration_width, calibration_height, full_w, full_h, sx, sy);
}

// Object points of a QR centred on its own face, matching the convention used
// by the Python qr_detection package (corner order: TL, TR, BR, BL).
std::vector<cv::Point3f> qrObjectPoints(double size)
{
    const float h = static_cast<float>(0.5 * size);
    return {
        cv::Point3f(-h,  h, 0.0f),
        cv::Point3f( h,  h, 0.0f),
        cv::Point3f( h, -h, 0.0f),
        cv::Point3f(-h, -h, 0.0f),
    };
}

// Re-express an OpenCV optical-frame rotation/translation in the ROS
// camera-link convention (matches transforms.optical_to_link in the Python
// package): camera_link is +X forward, +Y left, +Z up.
static const cv::Matx33d R_LINK_FROM_OPTICAL(
    0.0,  0.0, 1.0,
   -1.0,  0.0, 0.0,
    0.0, -1.0, 0.0);

// Convert a rotation matrix to a quaternion (x, y, z, w).
void matrixToQuat(const cv::Matx33d& R, double& qx, double& qy, double& qz, double& qw)
{
    const double trace = R(0, 0) + R(1, 1) + R(2, 2);
    if (trace > 0.0) {
        double s = std::sqrt(trace + 1.0) * 2.0;
        qw = 0.25 * s;
        qx = (R(2, 1) - R(1, 2)) / s;
        qy = (R(0, 2) - R(2, 0)) / s;
        qz = (R(1, 0) - R(0, 1)) / s;
    } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
        double s = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
        qw = (R(2, 1) - R(1, 2)) / s;
        qx = 0.25 * s;
        qy = (R(0, 1) + R(1, 0)) / s;
        qz = (R(0, 2) + R(2, 0)) / s;
    } else if (R(1, 1) > R(2, 2)) {
        double s = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;
        qw = (R(0, 2) - R(2, 0)) / s;
        qx = (R(0, 1) + R(1, 0)) / s;
        qy = 0.25 * s;
        qz = (R(1, 2) + R(2, 1)) / s;
    } else {
        double s = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;
        qw = (R(1, 0) - R(0, 1)) / s;
        qx = (R(0, 2) + R(2, 0)) / s;
        qy = (R(1, 2) + R(2, 1)) / s;
        qz = 0.25 * s;
    }
}

// Run QR detection / decoding / pose estimation on one full-sized raw frame and
// publish the first valid detection. Called only from the QR worker thread.
void processQrFrame(const cv::Mat& frame, const rclcpp::Time& stamp)
{
    // Safe check: skip empty frames.
    if (frame.empty()) {
        return;
    }

    // Safe check: require valid calibration (and a valid calibration reference
    // resolution to scale FROM) before attempting pose estimation.
    if (camera_matrix.empty() || dist_coeffs.empty() ||
        calibration_width <= 0 || calibration_height <= 0) {
        ROS_ERROR("QR: missing/invalid camera calibration, skipping pose estimation");
        return;
    }

    // (Re)build the full-resolution camera matrix if the frame size changed.
    if (qr_camera_matrix.empty() ||
        qr_full_width != frame.cols || qr_full_height != frame.rows) {
        buildQrCameraMatrix(frame.cols, frame.rows);
    }

    // --- QR detection + decoding (OpenCV, multi-code capable) ---
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<std::string> decoded_info;
    std::vector<cv::Point2f> points;
    bool ok = false;
    try {
        ok = qr_detector.detectAndDecodeMulti(gray, decoded_info, points);
    } catch (const cv::Exception& e) {
        ROS_ERROR("QR: detectAndDecodeMulti threw: %s", e.what());
        return;
    }

    // Safe check: invalid / insufficient corner count (need 4 per QR).
    if (!ok || points.size() < 4 || (points.size() % 4) != 0) {
        return;
    }

    const size_t num_codes = points.size() / 4;
    const std::vector<cv::Point3f> object_points = qrObjectPoints(qr_size_m);

    // Scale factors to map full-resolution corner pixels onto the downsized,
    // published image so the polygon can be overlaid on it on the PC side.
    const double scale_x = static_cast<double>(output_width)  / qr_full_width;
    const double scale_y = static_cast<double>(output_height) / qr_full_height;

#if (CV_VERSION_MAJOR >= 4)
    const int pnp_flag = cv::SOLVEPNP_IPPE_SQUARE;  // ideal for a planar square
#else
    const int pnp_flag = cv::SOLVEPNP_ITERATIVE;
#endif

    // Process detections; publish the first one that yields a valid pose.
    for (size_t i = 0; i < num_codes; ++i) {
        std::vector<cv::Point2f> corners(
            points.begin() + i * 4, points.begin() + i * 4 + 4);

        // --- solvePnP relative-pose estimation ---
        // Raw full-sized image -> full-resolution K and real distortion coeffs.
        cv::Mat rvec, tvec;
        bool pnp_ok = false;
        try {
            pnp_ok = cv::solvePnP(object_points, corners,
                                  qr_camera_matrix, qr_dist_coeffs,
                                  rvec, tvec, false, pnp_flag);
        } catch (const cv::Exception& e) {
            ROS_ERROR("QR: solvePnP threw: %s", e.what());
            pnp_ok = false;
        }
        // Safe check: skip detections where pose estimation failed.
        if (!pnp_ok) {
            continue;
        }

        // Optical-frame pose (OpenCV convention) -> camera_link convention.
        cv::Mat Rm;
        cv::Rodrigues(rvec, Rm);
        Rm.convertTo(Rm, CV_64F);
        tvec.convertTo(tvec, CV_64F);

        cv::Matx33d R_opt;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                R_opt(r, c) = Rm.at<double>(r, c);
        cv::Vec3d t_opt(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

        const cv::Matx33d R_link = R_LINK_FROM_OPTICAL * R_opt * R_LINK_FROM_OPTICAL.t();
        const cv::Vec3d  t_link = R_LINK_FROM_OPTICAL * t_opt;

        double qx, qy, qz, qw;
        matrixToQuat(R_link, qx, qy, qz, qw);

        const std::string data = (i < decoded_info.size()) ? decoded_info[i] : std::string();

        // --- Publish decoded text ---
        std_msgs::msg::String data_msg;
        data_msg.data = data;
        qr_data_pub->publish(data_msg);

        // --- Publish relative pose (camera -> QR, camera_link frame) ---
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "camera";
        pose_msg.pose.position.x = t_link[0];
        pose_msg.pose.position.y = t_link[1];
        pose_msg.pose.position.z = t_link[2];
        pose_msg.pose.orientation.x = qx;
        pose_msg.pose.orientation.y = qy;
        pose_msg.pose.orientation.z = qz;
        pose_msg.pose.orientation.w = qw;
        qr_pose_pub->publish(pose_msg);

        // --- Publish corners scaled to the downsized image proportions ---
        // scaled_x = full_x * downsized_width / full_width
        // scaled_y = full_y * downsized_height / full_height
        geometry_msgs::msg::PolygonStamped poly_msg;
        poly_msg.header.stamp = stamp;
        poly_msg.header.frame_id = "camera";
        for (const auto& c : corners) {
            geometry_msgs::msg::Point32 p;
            p.x = static_cast<float>(c.x * scale_x);
            p.y = static_cast<float>(c.y * scale_y);
            p.z = 0.0f;
            poly_msg.polygon.points.push_back(p);
        }
        qr_corners_pub->publish(poly_msg);

        // First valid QR published; stop here (single-code contract).
        return;
    }
}

// QR worker thread: waits for a freshly-captured full-sized frame and runs the
// detection pipeline on it, completely off the capture/publish path.
void qrThreadFunc()
{
    ROS_INFO("QR worker thread started");
    while (true) {
        cv::Mat local;
        {
            std::unique_lock<std::mutex> lk(qr_mutex);
            qr_cv.wait(lk, [] { return qr_has_pending || qr_worker_exit; });
            if (qr_worker_exit) {
                break;
            }
            local = qr_pending_frame;   // shares the capture-thread clone
            qr_has_pending = false;
        }
        processQrFrame(local, ROS_TIME_NOW());
    }
    ROS_INFO("QR worker thread exiting");
}

// QR enable/disable callback. 1 -> enable detection + publishing, 0 -> disable.
void qrEnableCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    const bool enable = (msg->data == 1);
    const bool prev = qr_enabled.exchange(enable);
    if (prev != enable) {
        ROS_INFO("QR detection %s via %s", enable ? "ENABLED" : "DISABLED",
                 qr_enable_topic.c_str());
    }
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

    // --- QR hand-off ---------------------------------------------------------
    // QR detection uses the ORIGINAL full-sized raw frame, captured here *before*
    // the downsize/undistort below. Only when enabled, and only on 1 of every N
    // frames, do we clone it to the worker. When disabled the extra cost is just
    // the counter increment and an atomic load (near-zero).
    if (qr_enabled.load()) {
        if ((qr_frame_counter % qr_process_every_n_frames) == 0) {
            {
                std::lock_guard<std::mutex> lk(qr_mutex);
                qr_pending_frame = frame.clone();   // fresh buffer; worker owns prior clone
                qr_has_pending = true;
            }
            qr_cv.notify_one();
        }
        ++qr_frame_counter;
    } else {
        qr_frame_counter = 0;   // reset so re-enabling starts fresh
    }

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
    ROS_CREATE_NODE("optimized_video_source_undistort_qr");

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

    // QR parameters
    ROS_DECLARE_PARAMETER("qr_enable_topic", qr_enable_topic);
    ROS_DECLARE_PARAMETER("qr_data_topic", qr_data_topic);
    ROS_DECLARE_PARAMETER("qr_pose_topic", qr_pose_topic);
    ROS_DECLARE_PARAMETER("qr_corners_topic", qr_corners_topic);
    ROS_DECLARE_PARAMETER("qr_size_m", qr_size_m);
    ROS_DECLARE_PARAMETER("qr_process_every_n_frames", qr_process_every_n_frames);
    ROS_DECLARE_PARAMETER("qr_enabled_default", qr_enabled_default);
    // Resolution the QR calibration matrix corresponds to (the downsized image it
    // was calibrated on). 0 = fall back to the size stored in the calibration YAML.
    ROS_DECLARE_PARAMETER("calibration_width", calibration_width);
    ROS_DECLARE_PARAMETER("calibration_height", calibration_height);

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

    ROS_GET_PARAMETER("qr_enable_topic", qr_enable_topic);
    ROS_GET_PARAMETER("qr_data_topic", qr_data_topic);
    ROS_GET_PARAMETER("qr_pose_topic", qr_pose_topic);
    ROS_GET_PARAMETER("qr_corners_topic", qr_corners_topic);
    ROS_GET_PARAMETER("qr_size_m", qr_size_m);
    ROS_GET_PARAMETER("qr_process_every_n_frames", qr_process_every_n_frames);
    ROS_GET_PARAMETER("qr_enabled_default", qr_enabled_default);
    ROS_GET_PARAMETER("calibration_width", calibration_width);
    ROS_GET_PARAMETER("calibration_height", calibration_height);

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

    if (qr_process_every_n_frames < 1) {
        ROS_INFO("qr_process_every_n_frames < 1, clamping to 1");
        qr_process_every_n_frames = 1;
    }

    // Resolve the QR calibration reference resolution. If the launch file does
    // not override it, fall back to the size recorded in the calibration YAML.
    // This is the resolution the calibration K is valid at and is the reference
    // we scale FROM when building the full-resolution QR camera matrix.
    if (calibration_width <= 0)  calibration_width  = calib_width;
    if (calibration_height <= 0) calibration_height = calib_height;
    if (calibration_width <= 0 || calibration_height <= 0) {
        ROS_ERROR("calibration_width/height invalid and not present in the YAML; "
                  "set calibration_width and calibration_height params");
        return 0;
    }
    ROS_INFO("QR calibration reference resolution: %dx%d (K scaled to full frame for solvePnP)",
             calibration_width, calibration_height);

    // Apply default QR enable state from the launch parameter.
    qr_enabled.store(qr_enabled_default);

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

    ROS_INFO("QR: default %s, size=%.3f m, every %d frames, enable topic=%s",
             qr_enabled_default ? "ENABLED" : "disabled",
             qr_size_m, qr_process_every_n_frames, qr_enable_topic.c_str());

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

    // QR publishers / subscriber.
    qr_data_pub    = node->create_publisher<std_msgs::msg::String>(qr_data_topic, 10);
    qr_pose_pub    = node->create_publisher<geometry_msgs::msg::PoseStamped>(qr_pose_topic, 10);
    qr_corners_pub = node->create_publisher<geometry_msgs::msg::PolygonStamped>(qr_corners_topic, 10);
    qr_enable_sub  = node->create_subscription<std_msgs::msg::Int32>(
        qr_enable_topic, 10, qrEnableCallback);

    image_pool.resize(pool_size);
    for (auto& msg : image_pool) {
        msg.header.frame_id = "camera";
    }

    should_exit = false;
    capture_thread = new std::thread(captureThreadFunc);

    qr_worker_exit = false;
    qr_thread = new std::thread(qrThreadFunc);

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

    // Stop QR worker.
    {
        std::lock_guard<std::mutex> lk(qr_mutex);
        qr_worker_exit = true;
    }
    qr_cv.notify_one();
    if (qr_thread) {
        if (qr_thread->joinable())
            qr_thread->join();
        delete qr_thread;
    }

    delete stream;
    delete image_cvt;

    return 0;
}
