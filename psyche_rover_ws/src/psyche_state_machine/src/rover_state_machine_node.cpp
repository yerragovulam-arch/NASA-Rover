#include "psyche_state_machine/rover_state_machine.hpp"

namespace psyche_state_machine
{

std::string to_string(RoverState state)
{
  switch (state) {
    case RoverState::START: return "START";
    case RoverState::PERCEIVE: return "PERCEIVE";
    case RoverState::STAND_UP: return "STAND_UP";
    case RoverState::MAP_TERRAIN: return "MAP_TERRAIN";
    case RoverState::EVALUATE_TRAVERSABILITY: return "EVALUATE_TRAVERSABILITY";
    case RoverState::PICK_PATH: return "PICK_PATH";
    case RoverState::TRAVERSE_PATH: return "TRAVERSE_PATH";
  }
  return "UNKNOWN";
}

RoverStateMachine::RoverStateMachine()
: rclcpp::Node("rover_state_machine")
{
  map_settle_s_ = declare_parameter<double>("map_settle_s", 1.2);
  map_state_entered_ = now();

  state_pub_ = create_publisher<std_msgs::msg::String>("/rover_state", 10);
  stand_request_pub_ = create_publisher<std_msgs::msg::Bool>("/stand_request", 10);
  nav_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/nav_enabled", 10);

  obstacle_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/obstacle_detected", 10,
    std::bind(&RoverStateMachine::obstacle_callback, this, std::placeholders::_1));

  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/terrain_costmap", 10,
    std::bind(&RoverStateMachine::costmap_callback, this, std::placeholders::_1));

  path_found_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/path_found", 10,
    std::bind(&RoverStateMachine::path_found_callback, this, std::placeholders::_1));

  posture_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/posture_ready", 10,
    std::bind(&RoverStateMachine::posture_ready_callback, this, std::placeholders::_1));

  // 5 Hz is plenty -- this loop drives state transitions, not control.
  timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&RoverStateMachine::tick, this));

  RCLCPP_INFO(get_logger(), "rover_state_machine started in state START");
}

void RoverStateMachine::obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  obstacle_detected_ = msg->data;
}

void RoverStateMachine::posture_ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  posture_ready_ = msg->data;
}

void RoverStateMachine::publish_stand_request(bool stand)
{
  std_msgs::msg::Bool msg;
  msg.data = stand;
  stand_request_pub_->publish(msg);
}

void RoverStateMachine::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr /*msg*/)
{
  // We only need to know a fresh map arrived, not its contents -- path_picker
  // consumes the actual data. Maps that arrive while the body is still
  // settling after the legs stopped are discarded (camera is on a swaying
  // legged body, unlike a wheeled chassis).
  if (state_ == RoverState::MAP_TERRAIN &&
    (now() - map_state_entered_).seconds() >= map_settle_s_)
  {
    map_ready_ = true;
  }
}

void RoverStateMachine::path_found_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (state_ == RoverState::PICK_PATH) {
    path_ready_ = true;
    path_found_ = msg->data;
  }
}

void RoverStateMachine::transition_to(RoverState new_state)
{
  if (new_state != state_) {
    RCLCPP_INFO(get_logger(), "%s -> %s",
      to_string(state_).c_str(), to_string(new_state).c_str());
  }
  if (new_state == RoverState::MAP_TERRAIN && state_ != RoverState::MAP_TERRAIN) {
    map_state_entered_ = now();
  }
  state_ = new_state;

  std_msgs::msg::String msg;
  msg.data = to_string(state_);
  state_pub_->publish(msg);
}

void RoverStateMachine::publish_nav_enabled(bool enabled)
{
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  nav_enabled_pub_->publish(msg);
}

void RoverStateMachine::tick()
{
  switch (state_) {
    case RoverState::START: {
      transition_to(RoverState::PERCEIVE);
      break;
    }

    case RoverState::PERCEIVE: {
      // STUB: real version should confirm the OAK-D / depthai-ros driver is
      // actually publishing (e.g. check for a recent message on the depth
      // topic) before moving on. For now we just log and proceed.
      RCLCPP_INFO(get_logger(), "PERCEIVE (stub): assuming OAK-D stream is live");
      transition_to(RoverState::STAND_UP);
      break;
    }

    case RoverState::STAND_UP: {
      // Real: ask psyche_hexapod_gait to raise the body and wait for it to
      // report fully standing. Re-sent every tick so a gait controller that
      // boots late still gets the request. Nothing downstream (mapping,
      // driving) runs until the legs are actually bearing weight.
      publish_stand_request(true);
      if (posture_ready_) {
        transition_to(RoverState::MAP_TERRAIN);
      } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
          "STAND_UP: waiting for /posture_ready from gait controller");
      }
      break;
    }

    case RoverState::MAP_TERRAIN: {
      // Keep the stand request alive (a late-booting or restarted gait
      // controller would otherwise sit down).
      publish_stand_request(true);
      // Real: wait for a fresh /terrain_costmap from terrain_mapper_node.
      if (map_ready_) {
        transition_to(RoverState::EVALUATE_TRAVERSABILITY);
      } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
          "MAP_TERRAIN: waiting for /terrain_costmap");
      }
      break;
    }

    case RoverState::EVALUATE_TRAVERSABILITY: {
      // Placeholder pass-through -- see class comment in the header. The
      // actual per-heading traversability check happens inside
      // path_picker_node when we reach PICK_PATH.
      map_ready_ = false;  // consumed; next MAP_TERRAIN entry needs a fresh one
      transition_to(RoverState::PICK_PATH);
      path_ready_ = false;
      path_found_ = false;
      break;
    }

    case RoverState::PICK_PATH: {
      publish_stand_request(true);
      // Real: wait for a /path_found result from path_picker_node.
      if (path_ready_) {
        if (path_found_) {
          transition_to(RoverState::TRAVERSE_PATH);
          publish_nav_enabled(true);
        } else {
          RCLCPP_WARN(get_logger(),
            "No clear heading found -- re-mapping to retry");
          path_ready_ = false;
          transition_to(RoverState::MAP_TERRAIN);
        }
      } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
          "PICK_PATH: waiting for /path_found");
      }
      break;
    }

    case RoverState::TRAVERSE_PATH: {
      publish_stand_request(true);
      if (obstacle_detected_) {
        RCLCPP_INFO(get_logger(),
          "Obstacle detected during traverse -- halting gait and re-mapping");
        publish_nav_enabled(false);
        map_ready_ = false;
        transition_to(RoverState::MAP_TERRAIN);
      }
      // Otherwise: stay here. reactive_avoidance_node is doing the actual
      // driving/steering toward /planned_heading while nav_enabled is true.
      break;
    }
  }
}

}  // namespace psyche_state_machine

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<psyche_state_machine::RoverStateMachine>());
  rclcpp::shutdown();
  return 0;
}
