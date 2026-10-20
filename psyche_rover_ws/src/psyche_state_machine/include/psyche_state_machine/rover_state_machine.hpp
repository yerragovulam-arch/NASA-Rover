#ifndef PSYCHE_STATE_MACHINE__ROVER_STATE_MACHINE_HPP_
#define PSYCHE_STATE_MACHINE__ROVER_STATE_MACHINE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>

namespace psyche_state_machine
{

enum class RoverState
{
  START,
  PERCEIVE,
  STAND_UP,
  MAP_TERRAIN,
  EVALUATE_TRAVERSABILITY,
  PICK_PATH,
  TRAVERSE_PATH,
};

std::string to_string(RoverState state);

// Top-level orchestration for the six-legged rover. Owns no perception or planning
// logic itself -- MAP_TERRAIN waits for a fresh /terrain_costmap from
// psyche_terrain_mapping's terrain_mapper_node, and PICK_PATH waits for a
// /path_found result from path_picker_node. EVALUATE_TRAVERSABILITY remains
// a placeholder: the real per-heading traversability check currently lives
// inside path_picker_node's clear_distance_for_heading(), so this state is
// a pass-through for now, kept in case you want a separate coarse
// traversability pass later (e.g. an overall "is this area worth mapping"
// check before committing to path picking).
class RoverStateMachine : public rclcpp::Node
{
public:
  RoverStateMachine();

private:
  void tick();
  void obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void path_found_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void posture_ready_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void publish_stand_request(bool stand);
  void transition_to(RoverState new_state);
  void publish_nav_enabled(bool enabled);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr path_found_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr posture_ready_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stand_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_enabled_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  RoverState state_{RoverState::START};
  bool obstacle_detected_{false};

  // Hexapod: true once psyche_hexapod_gait reports it is fully standing.
  bool posture_ready_{false};

  // Hexapod: after the legs stop, the body (and the camera on it) is still
  // rocking for a moment. MAP_TERRAIN ignores costmaps until this much time
  // has passed since entering the state, so we never plan off a map taken
  // mid-sway. Parameter `map_settle_s`.
  double map_settle_s_{1.2};
  rclcpp::Time map_state_entered_;

  // Set true when a costmap arrives while we're in MAP_TERRAIN; reset each
  // time we (re-)enter that state so we always wait for a *fresh* map
  // rather than reusing a stale one from before the last obstacle event.
  bool map_ready_{false};

  // Set true when path_picker reports a result while we're in PICK_PATH;
  // reset each time we (re-)enter that state.
  bool path_ready_{false};
  bool path_found_{false};
};

}  // namespace psyche_state_machine

#endif  // PSYCHE_STATE_MACHINE__ROVER_STATE_MACHINE_HPP_
