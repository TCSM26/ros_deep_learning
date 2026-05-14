// src/color_detector_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <rmw/qos_profiles.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <functional>

using ImageMsg  = sensor_msgs::msg::Image;
using ImagePtr  = ImageMsg::ConstSharedPtr;
using StringMsg = std_msgs::msg::String;

class ColorFSM : public rclcpp::Node
{
public:
  ColorFSM()
  : Node("computer_sem"),
    detect_every_(1),
    frame_count_(0),
    fps_accum_(0.0),
    fps_count_(0),
    last_flag_("none"),
    last_center_{0,0}
  {
    processed_pub_ = image_transport::create_publisher(
      this, "/image_processed");

    color_flag_pub_ = create_publisher<StringMsg>("color_flag", 10);

    image_sub_ = image_transport::create_subscription(
      this,
      "/image_raw",
      std::bind(&ColorFSM::image_callback, this, std::placeholders::_1),
      "compressed",
      rmw_qos_profile_sensor_data
    );

    last_frame_time_ = std::chrono::steady_clock::now();
    last_fps_time_   = last_frame_time_;

    RCLCPP_INFO(get_logger(),
                "Nodo iniciado: detect every %d frames",
                detect_every_);
  }

private:
  void image_callback(const ImagePtr & msg)
  {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_frame_time_).count();
    last_frame_time_ = now;
    double fps = dt > 0.0 ? 1.0/dt : 0.0;
    fps_accum_ += fps;
    fps_count_++;
    if (std::chrono::duration<double>(now - last_fps_time_).count() >= 1.0) {
      double avg = fps_accum_ / fps_count_;
      RCLCPP_INFO(get_logger(),
                  "Avg FPS: %.1f over %d frames",
                  avg, fps_count_);
      fps_accum_    = 0;
      fps_count_    = 0;
      last_fps_time_= now;
    }

    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "Imagen vacía");
      return;
    }

    frame_count_ = (frame_count_ + 1) % detect_every_;
    if (frame_count_ == 0) {
      cv::Mat hsv;
      cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

      cv::Point center;
      std::vector<cv::Point> contour;
      std::string new_flag = "none";
      bool found = false;

      if (detect_color(hsv, red1_lo, red1_hi, center, contour) ||
          detect_color(hsv, red2_lo, red2_hi, center, contour))
      {
        new_flag = "red";
        found = true;
      }
      else if (detect_color(hsv, yellow_lo, yellow_hi, center, contour)) {
        new_flag = "yellow";
        found = true;
      }
      else if (detect_color(hsv, green_lo, green_hi, center, contour)) {
        new_flag = "green";
        found = true;
      }

      StringMsg out; out.data = new_flag;
      color_flag_pub_->publish(out);

      if (found) {
        last_flag_    = new_flag;
        last_center_  = center;
        last_contour_ = std::move(contour);
      }
    }

    if (last_flag_ != "none" && !last_contour_.empty()) {
      draw_result(frame, last_flag_, last_center_, last_contour_);
    }

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1) << fps << " FPS";
    cv::putText(frame, ss.str(), cv::Point(10,30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(255,255,255), 2);

    auto out_img = cv_bridge::CvImage(
      msg->header, "bgr8", frame
    ).toImageMsg();
    processed_pub_.publish(out_img);
  }

  bool detect_color(const cv::Mat & hsv,
                    const cv::Scalar & lo,
                    const cv::Scalar & hi,
                    cv::Point & out,
                    std::vector<cv::Point> & best)
  {
    cv::Mat mask;
    cv::inRange(hsv, lo, hi, mask);
    auto k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  k);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k);

    std::vector<std::vector<cv::Point>> ctrs;
    cv::findContours(mask, ctrs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto & c : ctrs) {
      double area = cv::contourArea(c);
      if (area < 100) continue;
      cv::Point2f p; float r;
      cv::minEnclosingCircle(c, p, r);
      double circ = area / (M_PI * r * r);
      if (circ < 0.7) continue;
      auto m = cv::moments(c);
      if (m.m00 == 0) continue;
      out = { int(m.m10/m.m00), int(m.m01/m.m00) };
      best = c;
      return true;
    }
    return false;
  }

  void draw_result(cv::Mat & frame,
                   const std::string & color,
                   const cv::Point & c,
                   const std::vector<cv::Point> & contour)
  {
    cv::Scalar col =
      (color == "green"  ? cv::Scalar(0,255,0)   :
       color == "yellow" ? cv::Scalar(0,255,255) :
                            cv::Scalar(0,0,255));
    cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour},
                     -1, col, 2);
    cv::circle(frame, c, 8, cv::Scalar(255,255,255), -1);
    cv::putText(frame, color, c + cv::Point(10,-10),
                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255,255,255), 2);
  }

  image_transport::Subscriber            image_sub_;
  image_transport::Publisher             processed_pub_;
  rclcpp::Publisher<StringMsg>::SharedPtr color_flag_pub_;

  std::chrono::steady_clock::time_point last_frame_time_, last_fps_time_;
  double                                 fps_accum_;
  int                                    fps_count_;
  const int                              detect_every_;
  int                                    frame_count_;

  std::string                  last_flag_;
  cv::Point                    last_center_;
  std::vector<cv::Point>       last_contour_;

  // RANGOS HSV DEFINITIVOS Y ROBUSTOS
  const cv::Scalar red1_lo{0,100,100},    red1_hi{10,255,255};
  const cv::Scalar red2_lo{160,100,100},  red2_hi{179,255,255};
  const cv::Scalar yellow_lo{18, 80, 80}, yellow_hi{35, 255, 255};  // ✔️ nuevo rango robusto
  const cv::Scalar green_lo{40,70,70},    green_hi{85,255,255};
};

int main(int argc, char ** argv)
{
  // Antes de arrancar:
  //   export COMPRESSED_IMAGE_TRANSPORT_USE_NVJPEGDEC=1
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColorFSM>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}