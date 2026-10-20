#ifndef PSYCHE_TERRAIN_MAPPING__TERRAIN_MAPPER_HPP_
#define PSYCHE_TERRAIN_MAPPING__TERRAIN_MAPPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>

namespace psyche_terrain_mapping
{

// Builds a 2D occupancy grid (base_link frame, +x forward, +y left) from
// the OAK-D depth stream. Each depth pixel is projected into 3D using the
// pinhole camera model and camera_info intrinsics, then converted to a
// height-above-ground using camera_height_m / camera_tilt_deg. Cells with
// points deviating too far from ground level are marked occupied.
//
// camera_height_m and camera_tilt_deg are currently rough manual estimates
// (not a calibrated extrinsic/TF) -- see README for how to tune them.
class TerrainMapper : public rclcpp::Node
{
public:
  TerrainMapper();

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  bool have_intrinsics_{false};
  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};

  // Parameters
  std::string depth_topic_;
  std::string camera_info_topic_;
  double camera_height_m_;   // filler estimate: 0.3048 (1 ft)
  double camera_tilt_deg_;   // filler estimate: 0.0 (facing straight ahead)
  double grid_resolution_m_;
  double grid_range_m_;      // forward extent
  double grid_width_m_;      // total left-right extent
  double obstacle_height_thresh_m_;
  double min_valid_range_m_;
  double max_valid_range_m_;
  int pixel_stride_;         // subsample depth image for speed
};

}  // namespace psyche_terrain_mapping

#endif  // PSYCHE_TERRAIN_MAPPING__TERRAIN_MAPPER_HPP_
