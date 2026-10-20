#include "psyche_obstacle_avoidance/obstacle_detector.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <limits>

namespace psyche_obstacle_avoidance
{

ObstacleDetector::ObstacleDetector()
: rclcpp::Node("obstacle_detector")
{
  // ---- Parameters ----
  // NOTE: check your depthai-ros launch output (`ros2 topic list`) and
  // confirm this matches your actual depth topic name.
  depth_topic_ = declare_parameter<std::string>("depth_topic", "/oak/stereo/image_raw");
  num_sectors_ = declare_parameter<int>("num_sectors", 5);
  min_valid_range_m_ = declare_parameter<double>("min_valid_range_m", 0.15);
  max_valid_range_m_ = declare_parameter<double>("max_valid_range_m", 6.0);
  obstacle_trigger_m_ = declare_parameter<double>("obstacle_trigger_m", 1.0);
  row_band_start_frac_ = declare_parameter<double>("row_band_start_frac", 0.35);
  row_band_end_frac_ = declare_parameter<double>("row_band_end_frac", 0.85);
  depth_is_millimeters_ = declare_parameter<bool>("depth_is_millimeters", true);

  sector_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/obstacle_sectors", 10);
  obstacle_flag_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/obstacle_detected", 10);

  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ObstacleDetector::depth_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "obstacle_detector started. Listening on '%s', %d sectors, trigger at %.2f m",
    depth_topic_.c_str(), num_sectors_, obstacle_trigger_m_);
}

void ObstacleDetector::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // depthai-ros typically publishes 16UC1 depth in millimeters.
    cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
    return;
  }

  const cv::Mat & depth = cv_ptr->image;
  if (depth.empty()) {
    return;
  }

  const int height = depth.rows;
  const int width = depth.cols;

  const int row_start = static_cast<int>(row_band_start_frac_ * height);
  const int row_end = std::clamp(
    static_cast<int>(row_band_end_frac_ * height), row_start + 1, height);

  const int sector_width = std::max(1, width / num_sectors_);

  std::vector<float> sector_min_m(num_sectors_,
    std::numeric_limits<float>::infinity());

  for (int s = 0; s < num_sectors_; ++s) {
    const int col_start = s * sector_width;
    const int col_end = (s == num_sectors_ - 1) ? width : (col_start + sector_width);

    float min_depth_m = std::numeric_limits<float>::infinity();

    for (int r = row_start; r < row_end; ++r) {
      for (int c = col_start; c < col_end; ++c) {
        float depth_m = 0.0f;

        if (msg->encoding == "16UC1" || depth_is_millimeters_) {
          uint16_t raw = depth.at<uint16_t>(r, c);
          if (raw == 0) {
            continue;  // invalid pixel, DepthAI marks unknown depth as 0
          }
          depth_m = static_cast<float>(raw) / 1000.0f;
        } else if (msg->encoding == "32FC1") {
          depth_m = depth.at<float>(r, c);
          if (!std::isfinite(depth_m) || depth_m <= 0.0f) {
            continue;
          }
        } else {
          // Unrecognized encoding: bail out early once per callback.
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Unsupported depth encoding '%s' -- update obstacle_detector for this format.",
            msg->encoding.c_str());
          return;
        }

        if (depth_m < min_valid_range_m_ || depth_m > max_valid_range_m_) {
          continue;  // treat as noise or out of relevant range
        }

        min_depth_m = std::min(min_depth_m, depth_m);
      }
    }

    sector_min_m[s] = min_depth_m;
  }

  std_msgs::msg::Float32MultiArray sector_msg;
  bool obstacle_present = false;
  for (float d : sector_min_m) {
    // Report "infinity" sectors as max_valid_range_m_ so downstream
    // consumers don't have to special-case inf.
    float clean = std::isfinite(d) ? d : static_cast<float>(max_valid_range_m_);
    sector_msg.data.push_back(clean);
    if (clean <= obstacle_trigger_m_) {
      obstacle_present = true;
    }
  }
  sector_pub_->publish(sector_msg);

  std_msgs::msg::Bool flag_msg;
  flag_msg.data = obstacle_present;
  obstacle_flag_pub_->publish(flag_msg);
}

}  // namespace psyche_obstacle_avoidance

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<psyche_obstacle_avoidance::ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
