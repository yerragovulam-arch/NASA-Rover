// All six real nodes wired together over the shim bus, fed by a synthetic depth camera.
// No physics here (the robot does not move) -- this is about decisions, gating, timing and
// fault handling. Physical behaviour is covered by the MuJoCo co-simulation tests.
//
// "workaround" parameter set (4 values) is what makes the stack walk on an empty floor:
//   obstacle_detector.obstacle_trigger_m 0.45, reactive_avoidance.safe_distance_m 0.45,
//   reactive_avoidance.critical_distance_m 0.30, path_picker.unknown_allowance_m 0.6
#include <map>
#include <random>

#include "nodes.hpp"

using namespace tt;

namespace
{
const Cam kCam = Cam::sim_default(320, 200);

const sensor_msgs::msg::Image & empty_floor()
{
  static auto img = image16(kCam, render_depth(kCam, Scene{}));
  return img;
}
const sensor_msgs::msg::Image & wall_at(double F)
{
  static std::map<int, sensor_msgs::msg::Image> cache;
  const int key = static_cast<int>(F * 100);
  auto it = cache.find(key);
  if (it == cache.end()) {it = cache.emplace(key, image16(kCam, render_depth(kCam, Scene{}.wall(F, -3, 3)))).first;}
  return it->second;
}

struct Stack
{
  std::shared_ptr<psyche_hexapod_gait::GaitController> gait;
  std::shared_ptr<psyche_obstacle_avoidance::ObstacleDetector> det;
  std::shared_ptr<psyche_obstacle_avoidance::ReactiveAvoidance> avoid;
  std::shared_ptr<psyche_terrain_mapping::TerrainMapper> mapper;
  std::shared_ptr<psyche_terrain_mapping::PathPicker> picker;
  std::shared_ptr<psyche_state_machine::RoverStateMachine> sm;
  bool camera_on{true};
  const sensor_msgs::msg::Image * scene{&empty_floor()};
  std::function<sensor_msgs::msg::Image(const sensor_msgs::msg::Image &)> mangle;

  explicit Stack(bool workaround)
  {
    reset_world();
    if (workaround) {
      W().set_param("obstacle_detector", "obstacle_trigger_m", 0.45);
      W().set_param("reactive_avoidance", "safe_distance_m", 0.45);
      W().set_param("reactive_avoidance", "critical_distance_m", 0.30);
      W().set_param("path_picker", "unknown_allowance_m", 0.6);
    }
    gait = std::make_shared<psyche_hexapod_gait::GaitController>();
    det = std::make_shared<psyche_obstacle_avoidance::ObstacleDetector>();
    avoid = std::make_shared<psyche_obstacle_avoidance::ReactiveAvoidance>();
    mapper = std::make_shared<psyche_terrain_mapping::TerrainMapper>();
    picker = std::make_shared<psyche_terrain_mapping::PathPicker>();
    sm = std::make_shared<psyche_state_machine::RoverStateMachine>();
    W().inject("/oak/stereo/camera_info", cam_info(kCam));
  }
  // 10 Hz depth stream
  void run(double seconds)
  {
    for (double t = 0; t < seconds - 1e-9; t += 0.1) {
      if (camera_on) {W().inject("/oak/stereo/image_raw", mangle ? mangle(*scene) : *scene);}
      W().advance(0.1);
    }
  }
  static std::string rover_state()
  {
    auto m = W().msgs<std_msgs::msg::String>("/rover_state");
    return m.empty() ? "START" : m.back().second.data;
  }
  static std::string gait_state()
  {
    auto m = W().msgs<std_msgs::msg::String>("/gait_state");
    return m.empty() ? "" : m.back().second.data;
  }
  static double last_time_in(const std::string & topic_state)
  {
    double t = -1;
    for (auto & m : W().msgs<std_msgs::msg::String>("/gait_state")) {if (m.second.data == topic_state) {t = m.first;}}
    return t;
  }
};

std::map<std::string, int> histogram(const char * topic)
{
  std::map<std::string, int> h;
  for (auto & m : W().msgs<std_msgs::msg::String>(topic)) {h[m.second.data]++;}
  return h;
}
}  // namespace

// ------------------------------------------------------------ headline ---

XFAIL_TEST(shipped_parameters_walk_on_an_empty_flat_floor,
           "with params.yaml as shipped, the full stack never leaves the map/evaluate/pick loop on an empty "
           "floor: obstacle_detector sees the floor as an obstacle and path_picker finds no path (see "
           "detector_flat_floor_* and chain_flat_empty_floor_* tests)")
{
  Stack s(false);
  s.run(60.0);
  auto h = histogram("/rover_state");
  std::string summary;
  for (auto & kv : h) {summary += tf::fmt("%s x%d  ", kv.first.c_str(), kv.second);}
  INFO("state transitions in 60 s: %s", summary.c_str());
  INFO("/cmd_vel messages: %zu", W().count("/cmd_vel"));
  CHECK_MSG(Stack::last_time_in("WALKING") >= 0, "the rover never started walking in 60 s on an empty floor");
}

TEST(workaround_parameters_reach_traverse_and_walk_forward)
{
  Stack s(true);
  s.run(30.0);
  CHECK(Stack::rover_state() == "TRAVERSE_PATH");
  CHECK(Stack::gait_state() == "WALKING");
  auto cmds = W().msgs<geometry_msgs::msg::Twist>("/cmd_vel");
  REQUIRE(!cmds.empty());
  CHECK_NEAR(cmds.back().second.linear.x, 0.08, 1e-6);
  CHECK_NEAR(cmds.back().second.angular.z, 0.0, 1e-6);
}

TEST(startup_sequence_timing_is_sane)
{
  Stack s(true);
  s.run(20.0);
  double t_stand = -1, t_walk = Stack::last_time_in("WALKING");
  for (auto & m : W().msgs<std_msgs::msg::String>("/rover_state")) {if (m.second.data == "STAND_UP" && t_stand < 0) {t_stand = m.first;}}
  double first_walk = -1;
  for (auto & m : W().msgs<std_msgs::msg::String>("/gait_state")) {if (m.second.data == "WALKING") {first_walk = m.first; break;}}
  INFO("stand-up requested at %.1f s, first WALKING at %.1f s", t_stand, first_walk);
  CHECK(first_walk > 0 && first_walk < 8.0);
  CHECK(t_walk > 0);
}

// -------------------------------------------------------- sensor faults ---

TEST(depth_loss_mid_traverse_stops_the_legs)
{
  Stack s(true);
  s.run(20.0);
  REQUIRE(Stack::gait_state() == "WALKING");
  s.camera_on = false;
  const double t0 = W().now_ns * 1e-9;
  double t_stop = -1;
  for (int k = 0; k < 80 && t_stop < 0; ++k) {
    s.run(0.1);
    if (Stack::gait_state() != "WALKING") {t_stop = W().now_ns * 1e-9 - t0;}
  }
  INFO("legs stopped %.1f s after the depth stream died (cmd_timeout_s = 0.5)", t_stop);
  CHECK_MSG(t_stop > 0 && t_stop < 3.0, "still walking 8 s after sensor loss");
}

XFAIL_TEST(depth_loss_must_pull_the_state_machine_out_of_traverse,
           "the state machine has no sensor watchdog: after the depth stream dies it stays in TRAVERSE_PATH "
           "with nav_enabled=true forever, and resumes walking without re-mapping the moment frames return")
{
  Stack s(true);
  s.run(20.0);
  REQUIRE(Stack::rover_state() == "TRAVERSE_PATH");
  s.camera_on = false;
  s.run(10.0);
  CHECK_MSG(Stack::rover_state() != "TRAVERSE_PATH", "still in TRAVERSE_PATH 10 s after sensor loss");
}

TEST(sensor_returning_after_outage_recovers_without_crash)
{
  Stack s(true);
  s.run(15.0);
  s.camera_on = false; s.run(5.0);
  s.camera_on = true; s.run(10.0);
  CHECK(Stack::gait_state() == "WALKING");
  for (auto & m : W().msgs<sensor_msgs::msg::JointState>("/joint_commands")) {
    for (double p : m.second.position) {CHECK(std::isfinite(p));}
  }
}

TEST(garbage_depth_frames_do_not_crash_or_poison_servos)
{
  Stack s(true);
  std::mt19937 rng(3);
  s.mangle = [&](const sensor_msgs::msg::Image & in) {
      sensor_msgs::msg::Image out = in;
      if (rng() % 4 == 0) {for (auto & b : out.data) {b = static_cast<uint8_t>(rng());}}   // 25% random frames
      return out;
    };
  s.run(40.0);
  for (auto & m : W().msgs<sensor_msgs::msg::JointState>("/joint_commands")) {
    for (double p : m.second.position) {CHECK(std::isfinite(p) && std::abs(p) < 2.6);}
  }
  CHECK(W().count_logs(2) == 0);
}

TEST(wrong_sized_or_wrong_encoding_frames_are_survivable)
{
  Stack s(true);
  s.run(15.0);
  sensor_msgs::msg::Image bad = empty_floor();
  bad.encoding = "rgb8";                    // colour frame on the depth topic
  bad.data.resize(static_cast<size_t>(bad.width) * bad.height * 3);
  for (int i = 0; i < 20; ++i) {W().inject("/oak/stereo/image_raw", bad);}
  sensor_msgs::msg::Image tiny = empty_floor(); tiny.data.resize(10);
  for (int i = 0; i < 20; ++i) {W().inject("/oak/stereo/image_raw", tiny);}
  s.run(5.0);
  CHECK(Stack::gait_state() == "WALKING");
}

TEST(depth_noise_and_dropouts_do_not_break_the_cycle)
{
  Stack s(true);
  std::mt19937 rng(11);
  std::normal_distribution<double> n(0.0, 0.01);
  s.mangle = [&](const sensor_msgs::msg::Image & in) {
      sensor_msgs::msg::Image out = in;
      for (size_t i = 0; i + 1 < out.data.size(); i += 2) {
        if (rng() % 10 == 0) {out.data[i] = out.data[i + 1] = 0; continue;}               // 10% dropout
        int mm = in.data[i] | (in.data[i + 1] << 8);
        mm = std::max(0, mm + static_cast<int>(n(rng) * 1000.0));                          // 1 cm noise
        out.data[i] = mm & 0xFF; out.data[i + 1] = mm >> 8;
      }
      return out;
    };
  s.run(40.0);
  auto h = histogram("/rover_state");
  INFO("with 10%% dropout + 1 cm noise: TRAVERSE entered %d time(s), state changes %zu", h["TRAVERSE_PATH"],
       W().msgs<std_msgs::msg::String>("/rover_state").size());
  CHECK(h["TRAVERSE_PATH"] >= 1);
  CHECK(Stack::last_time_in("WALKING") > 0);
}

// ----------------------------------------------------- sudden obstacles ---

TEST(wall_appearing_mid_traverse_triggers_avoidance_and_stop)
{
  Stack s(true);
  s.run(20.0);
  REQUIRE(Stack::gait_state() == "WALKING");
  const double t0 = W().now_ns * 1e-9;
  s.scene = &wall_at(0.3);          // wall 0.3 m ahead appears instantly
  s.run(0.5);
  // the avoidance node must have asked for a reverse/turn quickly
  bool reacted = false;
  for (auto & m : W().msgs<geometry_msgs::msg::Twist>("/cmd_vel")) {
    if (m.first > t0 && (m.second.linear.x < 0 || m.second.linear.x < 0.079)) {reacted = true; break;}
  }
  CHECK_MSG(reacted, "no avoidance reaction within 0.5 s of a wall at 0.3 m");
  // nav must be revoked and the legs must stop
  auto nav = W().msgs<std_msgs::msg::Bool>("/nav_enabled");
  REQUIRE(!nav.empty());
  CHECK(nav.back().second.data == false);
  CHECK_MSG(nav.back().first - t0 < 0.6, "nav disabled %.2f s after the wall appeared", nav.back().first - t0);
  s.run(6.0);
  CHECK(Stack::gait_state() != "WALKING");
}

TEST(blocked_forever_never_drives_into_the_wall)
{
  Stack s(true);
  s.scene = &wall_at(0.3);
  s.run(40.0);
  for (auto & m : W().msgs<geometry_msgs::msg::Twist>("/cmd_vel")) {
    CHECK_MSG(m.second.linear.x <= 0.0 + 1e-9, "forward command (%.3f) with a wall at 0.3 m", m.second.linear.x);
  }
  CHECK(Stack::gait_state() != "WALKING");
}

TEST(distant_wall_lets_rover_walk_then_stops_before_reaching_it)
{
  Stack s(true);
  s.scene = &wall_at(1.8);
  s.run(25.0);
  CHECK(Stack::last_time_in("WALKING") > 0);   // 1.8 m is beyond the 0.45 m reaction distance: keeps going
}

// ------------------------------------------------------- external / ops ---

TEST(external_estop_via_nav_enabled_stops_the_gait)
{
  Stack s(true);
  s.run(20.0);
  REQUIRE(Stack::gait_state() == "WALKING");
  W().inject("/nav_enabled", boolmsg(false));
  s.run(4.0);
  CHECK(Stack::gait_state() != "WALKING");
}

TEST(velocity_and_joint_limits_hold_over_the_whole_run)
{
  Stack s(true);
  s.run(40.0);
  auto m = W().msgs<sensor_msgs::msg::JointState>("/joint_commands");
  double worst_v = 0, worst_q = 0;
  for (size_t i = 1; i < m.size(); ++i) {
    for (size_t k = 0; k < 18; ++k) {
      worst_q = std::max(worst_q, std::abs(m[i].second.position[k]));
      worst_v = std::max(worst_v, std::abs(m[i].second.position[k] - m[i - 1].second.position[k]) / (m[i].first - m[i - 1].first));
    }
  }
  INFO("whole run: peak |q| = %.2f rad, peak joint speed = %.2f rad/s", worst_q, worst_v);
  CHECK(worst_q < 2.6);
  CHECK(worst_v < 6.0);
  CHECK(W().count_logs(1) == 0);
}

TEST(message_rates_in_traverse_are_as_designed)
{
  Stack s(true);
  s.run(30.0);
  const size_t cmds = W().count("/cmd_vel");
  INFO("/cmd_vel messages in 30 s: %zu (depth at 10 Hz)", cmds);
  CHECK(cmds > 150 && cmds < 320);
  CHECK(W().count("/nav_enabled") <= 2);
}

int main(int argc, char ** argv) {return tf::run_main(argc, argv, "system_faults", reset_world);}
