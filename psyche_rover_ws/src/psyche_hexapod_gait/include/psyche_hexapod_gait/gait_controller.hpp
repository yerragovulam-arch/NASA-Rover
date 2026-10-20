#ifndef PSYCHE_HEXAPOD_GAIT__GAIT_CONTROLLER_HPP_
#define PSYCHE_HEXAPOD_GAIT__GAIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "psyche_hexapod_gait/gait_generator.hpp"

namespace psyche_hexapod_gait
{

// Turns /cmd_vel (body velocity) into 18 joint position targets (6 legs x
// coxa/femur/tibia) using a tripod gait + analytic IK.
//
// Posture is gated by /stand_request (from the state machine). Walking is
// additionally gated by /nav_enabled, same as the old wheeled drive: when
// nav is disabled the robot holds a standing pose.
//
// Output: sensor_msgs/JointState on /joint_commands (positions only). Feed
// this to your servo bridge (Dynamixel / PCA9685 / ros2_control, etc).
class GaitController : public rclcpp::Node
{
public:
  GaitController();

private:
  enum class Posture {SITTING, STANDING_UP, STANDING, SITTING_DOWN};

  void tick();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void nav_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void stand_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void publish_joints();
  void publish_status();
  static const char * to_string(Posture p);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stand_request_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr posture_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gait_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  GaitParams params_;
  std::unique_ptr<GaitGenerator> gait_;

  double control_rate_hz_{50.0};
  double stand_time_s_{2.0};
  double cmd_timeout_s_{0.5};
  std::vector<double> joint_signs_;
  std::vector<double> joint_offsets_;
  std::vector<std::string> joint_names_;

  Posture posture_{Posture::SITTING};
  double posture_blend_{0.0};  // 0 = sitting, 1 = standing
  bool stand_requested_{false};
  bool nav_enabled_{false};

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  int status_divider_{0};
};

}  // namespace psyche_hexapod_gait

#endif  // PSYCHE_HEXAPOD_GAIT__GAIT_CONTROLLER_HPP_
