#include "psyche_terrain_mapping/path_picker.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

namespace psyche_terrain_mapping
{

PathPicker::PathPicker()
: rclcpp::Node("path_picker")
{
  max_heading_deg_ = declare_parameter<double>("max_heading_deg", 45.0);
  heading_step_deg_ = declare_parameter<double>("heading_step_deg", 5.0);
  lookahead_m_ = declare_parameter<double>("lookahead_m", 3.0);
  rover_half_width_m_ = declare_parameter<double>("rover_half_width_m", 0.35);
  step_m_ = declare_parameter<double>("step_m", 0.1);
  min_acceptable_clear_m_ = declare_parameter<double>("min_acceptable_clear_m", 0.5);
  unknown_allowance_m_ = declare_parameter<double>("unknown_allowance_m", 0.3);

  heading_pub_ = create_publisher<std_msgs::msg::Float32>("/planned_heading", 10);
  path_found_pub_ = create_publisher<std_msgs::msg::Bool>("/path_found", 10);

  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/terrain_costmap", 10,
    std::bind(&PathPicker::costmap_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "path_picker started. Scanning +/-%.0f deg in %.0f deg steps, lookahead %.1fm",
    max_heading_deg_, heading_step_deg_, lookahead_m_);
}

float PathPicker::clear_distance_for_heading(
  const nav_msgs::msg::OccupancyGrid & grid, double heading_rad) const
{
  const double cos_h = std::cos(heading_rad);
  const double sin_h = std::sin(heading_rad);
  const double res = grid.info.resolution;
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);

  for (double d = step_m_; d <= lookahead_m_; d += step_m_) {
    const double cx = d * cos_h;       // forward distance along heading
    const double cy = d * sin_h;       // lateral offset along heading

    // Check a swath perpendicular to the heading, +/- rover half width,
    // so we're checking rover footprint, not a zero-width ray.
    for (double lat = -rover_half_width_m_; lat <= rover_half_width_m_; lat += res) {
      // Perpendicular offset from the heading direction.
      const double px = cx - lat * sin_h;
      const double py = cy + lat * cos_h;

      const int gx = static_cast<int>((py - origin_y) / res);
      const int gy = static_cast<int>((px - origin_x) / res);

      if (gx < 0 || gx >= w || gy < 0 || gy >= h) {
        // Out of mapped area -- treat as unknown, subject to same
        // near-field allowance as explicit unknown cells below.
        if (d > unknown_allowance_m_) {
          return static_cast<float>(d);
        }
        continue;
      }

      const int8_t cell = grid.data[static_cast<size_t>(gy) * w + gx];
      if (cell >= 50) {
        // Occupied -- blocked here.
        return static_cast<float>(d);
      }
      if (cell < 0 && d > unknown_allowance_m_) {
        // Unknown beyond the near-field allowance -- don't drive blind.
        return static_cast<float>(d);
      }
    }
  }

  return static_cast<float>(lookahead_m_);  // fully clear to lookahead
}

void PathPicker::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  double best_heading_rad = 0.0;
  float best_clear = -1.0f;

  const double max_heading_rad = max_heading_deg_ * M_PI / 180.0;
  const double step_rad = heading_step_deg_ * M_PI / 180.0;

  for (double heading = -max_heading_rad; heading <= max_heading_rad; heading += step_rad) {
    const float clear = clear_distance_for_heading(*msg, heading);

    // Prefer greater clear distance; on a tie, prefer the heading closer
    // to straight ahead.
    if (clear > best_clear + 1e-3f ||
      (std::abs(clear - best_clear) <= 1e-3f && std::abs(heading) < std::abs(best_heading_rad)))
    {
      best_clear = clear;
      best_heading_rad = heading;
    }
  }

  std_msgs::msg::Float32 heading_msg;
  heading_msg.data = static_cast<float>(best_heading_rad);
  heading_pub_->publish(heading_msg);

  std_msgs::msg::Bool found_msg;
  found_msg.data = best_clear >= min_acceptable_clear_m_;
  path_found_pub_->publish(found_msg);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "Best heading: %.1f deg, clear %.2fm, path_found=%s",
    best_heading_rad * 180.0 / M_PI, best_clear, found_msg.data ? "true" : "false");
}

}  // namespace psyche_terrain_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<psyche_terrain_mapping::PathPicker>());
  rclcpp::shutdown();
  return 0;
}
