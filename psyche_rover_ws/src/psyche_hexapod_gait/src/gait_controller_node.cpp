#include "psyche_hexapod_gait/gait_controller.hpp"

#include <algorithm>
#include <cmath>

namespace psyche_hexapod_gait
{

const char * GaitController::to_string(Posture p)
{
  switch (p) {
    case Posture::SITTING: return "SITTING";
    case Posture::STANDING_UP: return "STANDING_UP";
    case Posture::STANDING: return "STANDING";
    case Posture::SITTING_DOWN: return "SITTING_DOWN";
  }
  return "UNKNOWN";
}

GaitController::GaitController()
: rclcpp::Node("gait_controller")
{
  // --- Leg geometry (PLACEHOLDERS -- measure your real hexapod) ---
  params_.geometry.coxa_len = declare_parameter<double>("coxa_len_m", 0.050);
  params_.geometry.femur_len = declare_parameter<double>("femur_len_m", 0.080);
  params_.geometry.tibia_len = declare_parameter<double>("tibia_len_m", 0.130);
  params_.body_half_len = declare_parameter<double>("body_half_len_m", 0.100);
  params_.body_half_width = declare_parameter<double>("body_half_width_m", 0.060);

  // --- Gait ---
  params_.stand_height = declare_parameter<double>("stand_height_m", 0.120);
  params_.sit_height = declare_parameter<double>("sit_height_m", 0.035);
  params_.neutral_reach = declare_parameter<double>("neutral_reach_m", 0.160);
  params_.step_height = declare_parameter<double>("step_height_m", 0.040);
  params_.cycle_time = declare_parameter<double>("cycle_time_s", 1.2);
  params_.duty = declare_parameter<double>("duty", 0.5);
  params_.max_half_stride = declare_parameter<double>("max_half_stride_m", 0.040);
  params_.max_linear = declare_parameter<double>("max_linear_mps", 0.08);
  params_.max_angular = declare_parameter<double>("max_angular_rps", 0.50);
  params_.linear_accel = declare_parameter<double>("linear_accel", 0.20);
  params_.angular_accel = declare_parameter<double>("angular_accel", 1.0);

  control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
  stand_time_s_ = declare_parameter<double>("stand_time_s", 2.0);
  cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.5);

  // Servo calibration: per-joint direction and zero offset, in the order
  // leg0_coxa, leg0_femur, leg0_tibia, leg1_coxa, ... (18 values).
  joint_signs_ = declare_parameter<std::vector<double>>(
    "joint_signs", std::vector<double>(3 * kNumLegs, 1.0));
  joint_offsets_ = declare_parameter<std::vector<double>>(
    "joint_offsets_rad", std::vector<double>(3 * kNumLegs, 0.0));
  if (joint_signs_.size() != 3 * kNumLegs) {
    RCLCPP_WARN(get_logger(), "joint_signs must have 18 entries; using all +1");
    joint_signs_.assign(3 * kNumLegs, 1.0);
  }
  if (joint_offsets_.size() != 3 * kNumLegs) {
    RCLCPP_WARN(get_logger(), "joint_offsets_rad must have 18 entries; using all 0");
    joint_offsets_.assign(3 * kNumLegs, 0.0);
  }

  for (int i = 0; i < kNumLegs; ++i) {
    joint_names_.push_back(std::string(leg_name(i)) + "_coxa");
    joint_names_.push_back(std::string(leg_name(i)) + "_femur");
    joint_names_.push_back(std::string(leg_name(i)) + "_tibia");
  }

  gait_ = std::make_unique<GaitGenerator>(params_);
  gait_->set_body_height(params_.sit_height);
  last_cmd_time_ = now();

  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);
  posture_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/posture_ready", 10);
  gait_state_pub_ = create_publisher<std_msgs::msg::String>("/gait_state", 10);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&GaitController::cmd_vel_callback, this, std::placeholders::_1));
  nav_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/nav_enabled", 10,
    std::bind(&GaitController::nav_enabled_callback, this, std::placeholders::_1));
  stand_request_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/stand_request", 10,
    std::bind(&GaitController::stand_request_callback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GaitController::tick, this));

  RCLCPP_INFO(get_logger(),
    "gait_controller started (tripod gait, %.0f Hz). Waiting for /stand_request.",
    control_rate_hz_);
}

void GaitController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_ = *msg;
  last_cmd_time_ = now();
}

void GaitController::nav_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  nav_enabled_ = msg->data;
}

void GaitController::stand_request_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  stand_requested_ = msg->data;
}

void GaitController::tick()
{
  const double dt = 1.0 / control_rate_hz_;

  // --- Posture state machine: sit <-> stand ---
  switch (posture_) {
    case Posture::SITTING:
      if (stand_requested_) {posture_ = Posture::STANDING_UP;}
      break;
    case Posture::STANDING_UP:
      posture_blend_ = std::min(1.0, posture_blend_ + dt / stand_time_s_);
      if (!stand_requested_) {posture_ = Posture::SITTING_DOWN;}
      else if (posture_blend_ >= 1.0) {posture_ = Posture::STANDING;}
      break;
    case Posture::STANDING:
      if (!stand_requested_ && !gait_->is_moving()) {posture_ = Posture::SITTING_DOWN;}
      break;
    case Posture::SITTING_DOWN:
      posture_blend_ = std::max(0.0, posture_blend_ - dt / stand_time_s_);
      if (stand_requested_) {posture_ = Posture::STANDING_UP;}
      else if (posture_blend_ <= 0.0) {posture_ = Posture::SITTING;}
      break;
  }

  // Smoothstep blend between sit and stand heights.
  const double b = posture_blend_ * posture_blend_ * (3.0 - 2.0 * posture_blend_);
  gait_->set_body_height(
    params_.sit_height + b * (params_.stand_height - params_.sit_height));

  // --- Velocity command: only walk when fully standing, nav enabled, cmd fresh ---
  double vx = 0.0, vy = 0.0, wz = 0.0;
  const bool fresh = (now() - last_cmd_time_).seconds() < cmd_timeout_s_;
  if (posture_ == Posture::STANDING && nav_enabled_ && fresh) {
    vx = std::clamp(last_cmd_.linear.x, -params_.max_linear, params_.max_linear);
    vy = std::clamp(last_cmd_.linear.y, -params_.max_linear, params_.max_linear);
    wz = std::clamp(last_cmd_.angular.z, -params_.max_angular, params_.max_angular);
  }
  gait_->set_target(vx, vy, wz);
  gait_->step(dt);

  publish_joints();

  if (++status_divider_ >= static_cast<int>(control_rate_hz_ / 10.0)) {
    status_divider_ = 0;
    publish_status();
  }
}

void GaitController::publish_joints()
{
  sensor_msgs::msg::JointState js;
  js.header.stamp = now();
  js.name = joint_names_;
  js.position.resize(3 * kNumLegs);

  int unreachable = 0;
  for (int i = 0; i < kNumLegs; ++i) {
    const Vec3 foot_leg = body_to_leg(gait_->mounts()[i], gait_->foot_body(i));
    const JointAngles q = leg_ik(params_.geometry, foot_leg);
    if (!q.reachable) {++unreachable;}
    const double raw[3] = {q.coxa, q.femur, q.tibia};
    for (int j = 0; j < 3; ++j) {
      const size_t k = static_cast<size_t>(3 * i + j);
      js.position[k] = joint_signs_[k] * raw[j] + joint_offsets_[k];
    }
  }
  if (unreachable > 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "%d leg target(s) outside reach -- check leg lengths / stand_height / neutral_reach",
      unreachable);
  }
  joint_pub_->publish(js);
}

void GaitController::publish_status()
{
  std_msgs::msg::Bool ready;
  ready.data = (posture_ == Posture::STANDING);
  posture_ready_pub_->publish(ready);

  std_msgs::msg::String s;
  s.data = to_string(posture_);
  if (posture_ == Posture::STANDING && gait_->is_moving()) {s.data = "WALKING";}
  gait_state_pub_->publish(s);
}

}  // namespace psyche_hexapod_gait

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<psyche_hexapod_gait::GaitController>());
  rclcpp::shutdown();
  return 0;
}
