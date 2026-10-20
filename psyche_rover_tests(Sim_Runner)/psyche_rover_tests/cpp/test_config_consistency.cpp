// Cross-package consistency of the shipped params.yaml files, loaded through the shim.
// Catches the class of bug where each package is fine alone but the numbers disagree.
#include "nodes.hpp"

using namespace tt;
using namespace psyche_hexapod_gait;

namespace
{
const char * kGait = "psyche_hexapod_gait/config/params.yaml";
const char * kAvoid = "psyche_obstacle_avoidance/config/params.yaml";
const char * kTerrain = "psyche_terrain_mapping/config/params.yaml";
const char * kGazeboOverride = "psyche_hexapod_sim/config/sim_overrides_gazebo.yaml";

double P(const char * node, const char * name)
{
  auto it = W().params.find(std::string(node) + "/" + name);
  if (it == W().params.end()) {
    tf::fail(__FILE__, __LINE__, std::string("missing parameter ") + node + "/" + name);
    throw tf::Abort();
  }
  return it->second.d;
}

void load_all()
{
  reset_world();
  W().load_params_file(ws_path(kGait));
  W().load_params_file(ws_path(kAvoid));
  W().load_params_file(ws_path(kTerrain));
}

GaitParams gait_params_from_yaml()
{
  GaitParams p;
  p.geometry = {P("gait_controller", "coxa_len_m"), P("gait_controller", "femur_len_m"), P("gait_controller", "tibia_len_m")};
  p.body_half_len = P("gait_controller", "body_half_len_m");
  p.body_half_width = P("gait_controller", "body_half_width_m");
  p.stand_height = P("gait_controller", "stand_height_m");
  p.sit_height = P("gait_controller", "sit_height_m");
  p.neutral_reach = P("gait_controller", "neutral_reach_m");
  p.step_height = P("gait_controller", "step_height_m");
  p.cycle_time = P("gait_controller", "cycle_time_s");
  p.duty = P("gait_controller", "duty");
  p.max_half_stride = P("gait_controller", "max_half_stride_m");
  p.max_linear = P("gait_controller", "max_linear_mps");
  p.max_angular = P("gait_controller", "max_angular_rps");
  p.linear_accel = P("gait_controller", "linear_accel");
  p.angular_accel = P("gait_controller", "angular_accel");
  return p;
}
}  // namespace

TEST(yaml_files_load_and_have_expected_nodes)
{
  load_all();
  for (const char * k : {"gait_controller/stand_height_m", "obstacle_detector/num_sectors",
      "reactive_avoidance/linear_speed", "terrain_mapper/camera_height_m", "path_picker/lookahead_m"}) {
    CHECK_MSG(W().params.count(k), "missing %s", k);
  }
}

TEST(calibration_vectors_have_18_unit_entries)
{
  load_all();
  auto & s = W().params["gait_controller/joint_signs"].v;
  auto & o = W().params["gait_controller/joint_offsets_rad"].v;
  CHECK(s.size() == 18 && o.size() == 18);
  for (double v : s) {CHECK(std::abs(std::abs(v) - 1.0) < 1e-9);}
  for (double v : o) {CHECK(std::abs(v) < 1.0);}
}

TEST(avoidance_speeds_do_not_exceed_gait_limits)
{
  load_all();
  CHECK_MSG(P("reactive_avoidance", "linear_speed") <= P("gait_controller", "max_linear_mps") + 1e-9,
            "avoidance linear %.3f > gait %.3f", P("reactive_avoidance", "linear_speed"), P("gait_controller", "max_linear_mps"));
  CHECK(P("reactive_avoidance", "angular_speed") <= P("gait_controller", "max_angular_rps") + 1e-9);
  CHECK(P("reactive_avoidance", "reverse_speed") <= P("gait_controller", "max_linear_mps") + 1e-9);
}

XFAIL_TEST(obstacle_threshold_is_below_what_the_legs_can_clear,
           "terrain_mapper marks >5 cm as obstacle but step_height_m is only 4 cm: a 4-5 cm obstacle is "
           "'free' to the planner yet taller than the foot lift (and the lift has no safety margin)")
{
  load_all();
  const double thresh = P("terrain_mapper", "obstacle_height_thresh_m"), step = P("gait_controller", "step_height_m");
  INFO("obstacle threshold %.0f mm vs foot lift %.0f mm", thresh * 1000, step * 1000);
  CHECK_MSG(thresh <= step, "threshold %.3f m > step height %.3f m", thresh, step);
}

TEST(rover_half_width_covers_the_real_footprint)
{
  load_all();
  GaitParams p = gait_params_from_yaml();
  const double foot_r = 0.008;   // sim foot radius
  double worst_y = 0, worst_x = 0;
  struct S {double vx, vy, wz;};
  for (S s : {S{0.08, 0, 0}, S{0, 0.08, 0}, S{0, -0.08, 0}, S{0, 0, 0.5}, S{0, 0, -0.5}, S{0.05, 0.05, 0.3}, S{1, 1, 3}}) {
    GaitGenerator g(p); g.set_body_height(p.stand_height); g.set_target(s.vx, s.vy, s.wz);
    for (int k = 0; k < 400; ++k) {
      g.step(0.02);
      for (int i = 0; i < kNumLegs; ++i) {
        Vec3 f = g.foot_body(i);
        worst_y = std::max(worst_y, std::abs(f.y) + foot_r);
        worst_x = std::max(worst_x, std::abs(f.x) + foot_r);
      }
    }
  }
  const double body_hw = p.body_half_width + 0.05;   // sim body box
  INFO("footprint half-width %.3f m (feet), body %.3f m, half-length %.3f m; picker uses %.3f m",
       worst_y, body_hw, worst_x, P("path_picker", "rover_half_width_m"));
  CHECK_MSG(P("path_picker", "rover_half_width_m") >= worst_y - 1e-9,
            "rover_half_width_m %.3f < foot reach %.3f", P("path_picker", "rover_half_width_m"), worst_y);
}

TEST(camera_height_matches_stand_height_plus_mount)
{
  load_all();
  // sim mounts the camera 0.05 m above the body centre (cam_h 0.17 with stand 0.12)
  const double expect = P("gait_controller", "stand_height_m") + 0.05;
  CHECK_NEAR(P("terrain_mapper", "camera_height_m"), expect, 0.011);
}

TEST(planner_geometry_is_self_consistent)
{
  load_all();
  CHECK(P("path_picker", "lookahead_m") <= P("terrain_mapper", "grid_range_m"));
  CHECK(P("path_picker", "step_m") <= P("terrain_mapper", "grid_resolution_m") + 1e-9);   // never skip a cell
  CHECK(P("path_picker", "min_acceptable_clear_m") <= P("path_picker", "lookahead_m"));
  CHECK_NEAR(P("path_picker", "heading_step_deg") * 12, P("path_picker", "max_heading_deg"), 1e-9);  // lands exactly on +/-max
  const double reach_lat = P("path_picker", "lookahead_m") * std::sin(P("path_picker", "max_heading_deg") * M_PI / 180) +
    P("path_picker", "rover_half_width_m");
  INFO("widest heading needs %.2f m lateral coverage; costmap half-width is %.2f m (wide headings are clipped)",
       reach_lat, P("terrain_mapper", "grid_width_m") / 2);
}

TEST(detector_trigger_matches_avoidance_safe_distance)
{
  load_all();
  CHECK_NEAR(P("obstacle_detector", "obstacle_trigger_m"), P("reactive_avoidance", "safe_distance_m"), 1e-9);
  CHECK(P("reactive_avoidance", "critical_distance_m") < P("reactive_avoidance", "safe_distance_m"));
  CHECK(P("obstacle_detector", "min_valid_range_m") <= P("reactive_avoidance", "critical_distance_m"));
}

TEST(velocity_command_timeout_has_margin_over_sensor_period)
{
  load_all();
  const double depth_period = 1.0 / 10.0;   // sim publishes depth at 10 Hz (slowest producer of /cmd_vel)
  CHECK_MSG(P("gait_controller", "cmd_timeout_s") >= 3 * depth_period,
            "cmd_timeout %.2f s leaves <3 sensor periods of margin", P("gait_controller", "cmd_timeout_s"));
}

TEST(gait_yaml_reaches_every_target_for_all_gaits)
{
  load_all();
  GaitParams p = gait_params_from_yaml();
  struct S {double vx, vy, wz;};
  for (S s : {S{0.08, 0, 0}, S{-0.08, 0, 0}, S{0, 0.08, 0}, S{0, 0, 0.5}, S{0.05, 0, 0.3}, S{1, 1, 3}}) {
    GaitGenerator g(p); g.set_body_height(p.stand_height); g.set_target(s.vx, s.vy, s.wz);
    for (int k = 0; k < 300; ++k) {
      g.step(0.02);
      for (int i = 0; i < kNumLegs; ++i) {
        CHECK(leg_ik(p.geometry, body_to_leg(g.mounts()[i], g.foot_body(i))).reachable);
      }
    }
  }
}

TEST(gait_max_linear_is_achievable_within_stride_clamp)
{
  load_all();
  GaitParams p = gait_params_from_yaml();
  const double needed = p.max_linear * p.duty * p.cycle_time / 2.0;
  INFO("max_linear needs half-stride %.1f mm; clamp is %.1f mm", needed * 1000, p.max_half_stride * 1000);
  CHECK(needed <= p.max_half_stride + 1e-9);
}

TEST(gazebo_override_makes_32fc1_depth_work_in_detector)
{
  load_all();
  W().load_params_file(ws_path(kGazeboOverride));
  CHECK(W().params["obstacle_detector/depth_is_millimeters"].kind == rclcpp::PVal::BOOL);
  auto det = std::make_shared<psyche_obstacle_avoidance::ObstacleDetector>();
  const Cam cam = Cam::sim_default(320, 200);
  W().inject("/oak/stereo/image_raw", image32(cam, std::vector<float>(static_cast<size_t>(cam.w) * cam.h, 3.0f)));
  W().spin();
  auto s = W().msgs<std_msgs::msg::Float32MultiArray>("/obstacle_sectors");
  REQUIRE(!s.empty());
  for (float v : s.back().second.data) {CHECK_NEAR(v, 3.0, 0.01);}
}

int main(int argc, char ** argv) {return tf::run_main(argc, argv, "config_consistency", reset_world);}
