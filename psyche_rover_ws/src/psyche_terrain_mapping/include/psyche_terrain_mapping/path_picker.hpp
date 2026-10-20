#ifndef PSYCHE_TERRAIN_MAPPING__PATH_PICKER_HPP_
#define PSYCHE_TERRAIN_MAPPING__PATH_PICKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

namespace psyche_terrain_mapping
{

// Simple gap-based heading planner. For a fan of candidate headings
// (-max_heading_deg .. +max_heading_deg), walks outward along that heading
// over the occupancy grid and measures how far the rover could go before
// hitting an occupied or (beyond a small allowance) unknown cell, checking
// a rover-width swath at each step rather than a single ray. Picks the
// heading with the greatest clear distance, tie-breaking toward straight
// ahead.
//
// This is not a full path planner (no cost-to-goal, no multi-step search)
// -- it is a reactive "which direction has room" chooser, re-run every time
// a new costmap arrives.
class PathPicker : public rclcpp::Node
{
public:
  PathPicker();

private:
  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  float clear_distance_for_heading(
    const nav_msgs::msg::OccupancyGrid & grid, double heading_rad) const;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_found_pub_;

  // Parameters
  double max_heading_deg_;
  double heading_step_deg_;
  double lookahead_m_;
  double rover_half_width_m_;
  double step_m_;
  double min_acceptable_clear_m_;
  double unknown_allowance_m_;  // near-field distance where unknown cells are tolerated
};

}  // namespace psyche_terrain_mapping

#endif  // PSYCHE_TERRAIN_MAPPING__PATH_PICKER_HPP_
