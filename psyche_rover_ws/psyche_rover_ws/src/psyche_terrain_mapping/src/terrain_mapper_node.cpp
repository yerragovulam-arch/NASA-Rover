#include "psyche_terrain_mapping/terrain_mapper.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#include <vector>

namespace psyche_terrain_mapping
{

TerrainMapper::TerrainMapper()
: rclcpp::Node("terrain_mapper")
{
  depth_topic_ = declare_parameter<std::string>("depth_topic", "/oak/stereo/image_raw");
  camera_info_topic_ = declare_parameter<std::string>(
    "camera_info_topic", "/oak/stereo/camera_info");

  // --- Filler estimates, tune once the rover is on real terrain ---
  camera_height_m_ = declare_parameter<double>("camera_height_m", 0.3048);  // 1 ft
  camera_tilt_deg_ = declare_parameter<double>("camera_tilt_deg", 0.0);     // level

  grid_resolution_m_ = declare_parameter<double>("grid_resolution_m", 0.10);
  grid_range_m_ = declare_parameter<double>("grid_range_m", 4.0);
  grid_width_m_ = declare_parameter<double>("grid_width_m", 4.0);
  obstacle_height_thresh_m_ = declare_parameter<double>("obstacle_height_thresh_m", 0.15);
  min_valid_range_m_ = declare_parameter<double>("min_valid_range_m", 0.15);
  max_valid_range_m_ = declare_parameter<double>("max_valid_range_m", 6.0);
  pixel_stride_ = declare_parameter<int>("pixel_stride", 2);

  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/terrain_costmap", 10);

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TerrainMapper::camera_info_callback, this, std::placeholders::_1));

  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TerrainMapper::depth_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "terrain_mapper started. camera_height_m=%.3f (filler) camera_tilt_deg=%.1f (filler) "
    "-- update these once measured on the real rover",
    camera_height_m_, camera_tilt_deg_);
}

void TerrainMapper::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  // K = [fx 0 cx; 0 fy cy; 0 0 1]
  fx_ = msg->k[0];
  fy_ = msg->k[4];
  cx_ = msg->k[2];
  cy_ = msg->k[5];
  if (!have_intrinsics_ && fx_ > 0.0 && fy_ > 0.0) {
    have_intrinsics_ = true;
    RCLCPP_INFO(get_logger(), "Received camera intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
      fx_, fy_, cx_, cy_);
  }
}

void TerrainMapper::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!have_intrinsics_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Waiting for camera_info on '%s' before mapping terrain", camera_info_topic_.c_str());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
    return;
  }
  const cv::Mat & depth = cv_ptr->image;
  if (depth.empty()) {
    return;
  }

  const int grid_w = static_cast<int>(std::round(grid_width_m_ / grid_resolution_m_));
  const int grid_h = static_cast<int>(std::round(grid_range_m_ / grid_resolution_m_));

  // Track max |deviation from ground| seen per cell, and whether any point
  // landed there at all.
  std::vector<float> max_dev(static_cast<size_t>(grid_w) * grid_h, -1.0f);
  std::vector<bool> has_point(static_cast<size_t>(grid_w) * grid_h, false);

  const double tilt_rad = camera_tilt_deg_ * M_PI / 180.0;
  const double cos_t = std::cos(tilt_rad);
  const double sin_t = std::sin(tilt_rad);

  for (int v = 0; v < depth.rows; v += pixel_stride_) {
    for (int u = 0; u < depth.cols; u += pixel_stride_) {
      float depth_m = 0.0f;

      if (msg->encoding == "16UC1") {
        uint16_t raw = depth.at<uint16_t>(v, u);
        if (raw == 0) {continue;}
        depth_m = static_cast<float>(raw) / 1000.0f;
      } else if (msg->encoding == "32FC1") {
        depth_m = depth.at<float>(v, u);
        if (!std::isfinite(depth_m) || depth_m <= 0.0f) {continue;}
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Unsupported depth encoding '%s'", msg->encoding.c_str());
        return;
      }

      if (depth_m < min_valid_range_m_ || depth_m > max_valid_range_m_) {
        continue;
      }

      // Pinhole projection to camera-frame 3D point (optical convention:
      // X right, Y down, Z forward).
      const double x_cam = (u - cx_) * depth_m / fx_;
      const double y_cam = (v - cy_) * depth_m / fy_;
      const double z_cam = depth_m;

      // Rotate by camera tilt (pitch about the camera's X/right axis).
      // With camera_tilt_deg_ = 0 this is a no-op, ready for when it's
      // actually measured.
      const double y_rot = y_cam * cos_t - z_cam * sin_t;
      const double z_rot = y_cam * sin_t + z_cam * cos_t;

      // Convert to rover body frame (x forward, y left, height above ground).
      const double forward = z_rot;
      const double left = -x_cam;
      const double height_above_ground = camera_height_m_ - y_rot;

      if (forward < 0.0 || forward > grid_range_m_) {continue;}
      if (left < -grid_width_m_ / 2.0 || left > grid_width_m_ / 2.0) {continue;}

      const int gx = static_cast<int>((left + grid_width_m_ / 2.0) / grid_resolution_m_);
      const int gy = static_cast<int>(forward / grid_resolution_m_);
      if (gx < 0 || gx >= grid_w || gy < 0 || gy >= grid_h) {continue;}

      const size_t idx = static_cast<size_t>(gy) * grid_w + gx;
      has_point[idx] = true;
      max_dev[idx] = std::max(max_dev[idx], static_cast<float>(std::abs(height_above_ground)));
    }
  }

  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.stamp = msg->header.stamp;
  grid_msg.header.frame_id = "base_link";
  grid_msg.info.resolution = static_cast<float>(grid_resolution_m_);
  grid_msg.info.width = grid_w;
  grid_msg.info.height = grid_h;
  grid_msg.info.origin.position.x = 0.0;
  grid_msg.info.origin.position.y = -grid_width_m_ / 2.0;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  grid_msg.data.resize(static_cast<size_t>(grid_w) * grid_h);
  for (size_t i = 0; i < grid_msg.data.size(); ++i) {
    if (!has_point[i]) {
      grid_msg.data[i] = -1;  // unknown
    } else if (max_dev[i] > obstacle_height_thresh_m_) {
      grid_msg.data[i] = 100;  // occupied
    } else {
      grid_msg.data[i] = 0;  // free
    }
  }

  costmap_pub_->publish(grid_msg);
}

}  // namespace psyche_terrain_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<psyche_terrain_mapping::TerrainMapper>());
  rclcpp::shutdown();
  return 0;
}
