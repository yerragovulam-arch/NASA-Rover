#include "psyche_obstacle_avoidance/reactive_avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace psyche_obstacle_avoidance
{

ReactiveAvoidance::ReactiveAvoidance()
: rclcpp::Node("reactive_avoidance")
{
  safe_distance_m_ = declare_parameter<double>("safe_distance_m", 1.0);
  critical_distance_m_ = declare_parameter<double>("critical_distance_m", 0.4);
  linear_speed_ = declare_parameter<double>("linear_speed", 0.08);
  angular_speed_ = declare_parameter<double>("angular_speed", 0.4);
  reverse_speed_ = declare_parameter<double>("reverse_speed", 0.04);
  heading_kp_ = declare_parameter<double>("heading_kp", 1.0);
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  sector_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/obstacle_sectors", 10,
    std::bind(&ReactiveAvoidance::sector_callback, this, std::placeholders::_1));

  nav_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/nav_enabled", 10,
    std::bind(&ReactiveAvoidance::nav_enabled_callback, this, std::placeholders::_1));

  heading_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/planned_heading", 10,
    std::bind(&ReactiveAvoidance::heading_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "reactive_avoidance started. safe_distance=%.2fm critical_distance=%.2fm",
    safe_distance_m_, critical_distance_m_);
}

void ReactiveAvoidance::nav_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  nav_enabled_ = msg->data;
  if (!nav_enabled_) {
    // Immediately stop -- don't wait for the next sector callback.
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
  }
}

void ReactiveAvoidance::heading_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  planned_heading_rad_ = msg->data;
}

void ReactiveAvoidance::sector_callback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (!nav_enabled_) {
    // State machine hasn't told us it's safe to drive yet (e.g. still
    // mapping/planning). Stay stopped and ignore sector data.
    return;
  }

  const auto & sectors = msg->data;
  if (sectors.empty()) {
    return;
  }

  // Sectors are laid out left-to-right (see obstacle_detector).
  const size_t n = sectors.size();
  const size_t mid = n / 2;

  // "Center" sectors are what's directly ahead; split remaining left/right
  // for a simple left-vs-right clearance comparison.
  float center_min = sectors[mid];
  float left_min = std::numeric_limits<float>::infinity();
  float right_min = std::numeric_limits<float>::infinity();

  for (size_t i = 0; i < n; ++i) {
    if (i < mid) {
      left_min = std::min(left_min, sectors[i]);
    } else if (i > mid) {
      right_min = std::min(right_min, sectors[i]);
    }
  }

  geometry_msgs::msg::Twist cmd;

  if (center_min <= critical_distance_m_) {
    // Too close dead ahead -- back off slightly while turning away. A
    // hexapod can also pivot in place, so the turn does most of the work;
    // the gait controller turns this into a reversed tripod cycle.
    cmd.linear.x = -reverse_speed_;
    cmd.angular.z = (left_min > right_min) ? angular_speed_ : -angular_speed_;
  } else if (center_min <= safe_distance_m_) {
    // Obstacle ahead but not critical -- turn toward the side with more
    // clearance while easing off forward speed.
    cmd.linear.x = linear_speed_ * 0.3;
    cmd.angular.z = (left_min > right_min) ? angular_speed_ : -angular_speed_;
  } else {
    // Immediate sectors are clear -- steer toward the heading picked by
    // path_picker (0 = straight ahead) rather than always going straight.
    // Close-range checks above still override this every cycle, so a
    // sudden obstacle gets reacted to immediately rather than waiting on
    // the next full remap.
    double heading = std::clamp(planned_heading_rad_, -angular_speed_ / heading_kp_,
      angular_speed_ / heading_kp_);
    cmd.angular.z = heading_kp_ * heading;
    cmd.linear.x = linear_speed_ * std::cos(planned_heading_rad_);
  }

  cmd_vel_pub_->publish(cmd);
}

}  // namespace psyche_obstacle_avoidance

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<psyche_obstacle_avoidance::ReactiveAvoidance>());
  rclcpp::shutdown();
  return 0;
}
