#ifndef PSYCHE_OBSTACLE_AVOIDANCE__REACTIVE_AVOIDANCE_HPP_
#define PSYCHE_OBSTACLE_AVOIDANCE__REACTIVE_AVOIDANCE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace psyche_obstacle_avoidance
{

// Simple reactive steering: drive forward when clear, steer away from
// whichever side is blocked, stop/back off if the center is too close.
// This is meant as a working baseline -- swap in Nav2 or a costmap-based
// planner later without touching obstacle_detector.
class ReactiveAvoidance : public rclcpp::Node
{
public:
  ReactiveAvoidance();

private:
  void sector_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void nav_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void heading_callback(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sector_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Gated by the rover state machine -- only drive while in TRAVERSE_PATH.
  // Defaults to false: the rover does not move until the state machine
  // explicitly says it's safe to.
  bool nav_enabled_{false};

  // Heading (radians, 0 = straight ahead, positive = left) from
  // psyche_terrain_mapping's path_picker_node. Followed when the immediate
  // sectors are clear; overridden by close-range sector avoidance below
  // that regardless, as a fast-reacting safety net.
  double planned_heading_rad_{0.0};
  double heading_kp_{1.0};

  // Parameters
  double safe_distance_m_;     // if closest reading in front is under this, react
  double critical_distance_m_; // if under this, stop/reverse instead of just turning
  double linear_speed_;
  double angular_speed_;
  std::string cmd_vel_topic_;
};

}  // namespace psyche_obstacle_avoidance

#endif  // PSYCHE_OBSTACLE_AVOIDANCE__REACTIVE_AVOIDANCE_HPP_
