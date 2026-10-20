// gait_controller_node (real source via shim): posture machine, gating by /stand_request,
// /nav_enabled and /cmd_vel freshness, calibration, fault inputs, timing.
#include <limits>

#include "nodes.hpp"

using namespace tt;
using psyche_hexapod_gait::GaitController;
using sensor_msgs::msg::JointState;

namespace
{
constexpr double kMaxJointVel = 6.0;  // URDF velocity limit

struct GC
{
  std::shared_ptr<GaitController> node;
  GC() {node = std::make_shared<GaitController>();}
  void run(double s) {W().advance(s);}
  void stand(bool on) {W().inject("/stand_request", boolmsg(on)); W().spin();}
  void nav(bool on) {W().inject("/nav_enabled", boolmsg(on)); W().spin();}
  // publish /cmd_vel every 0.1 s (fresh) for `s` seconds
  void drive(double s, double vx, double vy = 0, double wz = 0)
  {
    for (double t = 0; t < s - 1e-9; t += 0.1) {W().inject("/cmd_vel", twist(vx, vy, wz)); W().advance(0.1);}
  }
  static std::string state()
  {
    auto m = W().msgs<std_msgs::msg::String>("/gait_state");
    return m.empty() ? "" : m.back().second.data;
  }
  static bool ready()
  {
    auto m = W().msgs<std_msgs::msg::Bool>("/posture_ready");
    return !m.empty() && m.back().second.data;
  }
  static JointState js()
  {
    auto m = W().msgs<JointState>("/joint_commands");
    return m.empty() ? JointState{} : m.back().second;
  }
  void standing_idle()
  {
    stand(true); run(2.5);
  }
  void walking(double vx = 0.06, double vy = 0, double wz = 0)
  {
    standing_idle(); nav(true); drive(3.0, vx, vy, wz);
  }
};

bool all_finite(const JointState & j)
{
  for (double p : j.position) {if (!std::isfinite(p)) {return false;}}
  return j.position.size() == 18;
}

double max_joint_speed(double t0 = 0.0, double t1 = 1e9)
{
  auto m = W().msgs<JointState>("/joint_commands");
  double worst = 0;
  for (size_t i = 1; i < m.size(); ++i) {
    if (m[i].first < t0 || m[i].first > t1) {continue;}
    const double dt = m[i].first - m[i - 1].first;
    if (dt <= 0) {continue;}
    for (size_t k = 0; k < 18; ++k) {
      worst = std::max(worst, std::abs(m[i].second.position[k] - m[i - 1].second.position[k]) / dt);
    }
  }
  return worst;
}

double joints_moved(const JointState & a, const JointState & b)
{
  double d = 0;
  for (size_t k = 0; k < 18; ++k) {d = std::max(d, std::abs(a.position[k] - b.position[k]));}
  return d;
}
}  // namespace

TEST(boots_sitting_with_sit_pose_and_not_ready)
{
  GC g; g.run(0.5);
  CHECK(GC::state() == "SITTING");
  CHECK(!GC::ready());
  JointState j = GC::js();
  REQUIRE(j.position.size() == 18);
  CHECK(all_finite(j));
  CHECK_NEAR(j.position[1], 1.109, 0.01);   // femur raised in the folded pose
  CHECK_NEAR(j.position[2], -2.071, 0.01);  // tibia folded
}

TEST(joint_names_are_18_unique_in_leg_order)
{
  GC g; g.run(0.1);
  JointState j = GC::js();
  REQUIRE(j.name.size() == 18);
  const char * legs[6] = {"leg_0_front_right", "leg_1_front_left", "leg_2_mid_right",
    "leg_3_mid_left", "leg_4_rear_right", "leg_5_rear_left"};
  const char * jn[3] = {"coxa", "femur", "tibia"};
  for (int i = 0; i < 6; ++i) {
    for (int k = 0; k < 3; ++k) {CHECK(j.name[3 * i + k] == std::string(legs[i]) + "_" + jn[k]);}
  }
}

TEST(stand_up_takes_stand_time_and_ready_comes_last)
{
  GC g; g.stand(true);
  g.run(1.0);
  CHECK(GC::state() == "STANDING_UP");
  CHECK(!GC::ready());
  g.run(1.5);
  CHECK(GC::state() == "STANDING");
  CHECK(GC::ready());
  // posture_ready must never have been true before the blend finished (2.0 s)
  for (auto & m : W().msgs<std_msgs::msg::Bool>("/posture_ready")) {
    if (m.second.data) {CHECK_MSG(m.first >= 1.99, "posture_ready true at t=%.2f", m.first);}
  }
}

TEST(no_walking_during_stand_up_or_without_nav)
{
  GC g; g.stand(true);
  g.drive(1.5, 0.06);                  // nav not enabled, still rising
  for (auto & m : W().msgs<std_msgs::msg::String>("/gait_state")) {CHECK(m.second.data != "WALKING");}
  g.run(1.5);                          // standing now
  g.drive(2.0, 0.06);                  // nav still off
  for (auto & m : W().msgs<std_msgs::msg::String>("/gait_state")) {CHECK(m.second.data != "WALKING");}
  JointState a = GC::js(); g.drive(1.0, 0.06); JointState b = GC::js();
  CHECK_MSG(joints_moved(a, b) < 1e-9, "legs moved with nav disabled (%.4f)", joints_moved(a, b));
}

TEST(nav_enable_with_fresh_cmd_starts_walking)
{
  GC g; g.standing_idle(); g.nav(true);
  g.drive(2.0, 0.06);
  CHECK(GC::state() == "WALKING");
}

TEST(cmd_vel_timeout_stops_walking_and_recentres_legs)
{
  GC g; g.walking();
  REQUIRE(GC::state() == "WALKING");
  const double t_stop = W().now_ns * 1e-9;       // last cmd was just published
  g.run(0.2);
  double t_idle = -1;
  for (int k = 0; k < 400 && t_idle < 0; ++k) {
    g.run(0.05);
    if (GC::state() == "STANDING") {t_idle = W().now_ns * 1e-9 - t_stop;}
  }
  INFO("legs idle %.2f s after the last /cmd_vel (cmd_timeout_s=0.5)", t_idle);
  CHECK_MSG(t_idle > 0 && t_idle < 3.0, "did not stop within 3 s (%.2f)", t_idle);
  JointState a = GC::js(); g.run(1.0); JointState b = GC::js();
  CHECK(joints_moved(a, b) < 1e-9);
}

TEST(nav_disable_while_walking_stops_gait_cleanly)
{
  GC g; g.walking();
  g.nav(false);
  g.drive(3.0, 0.06);
  CHECK(GC::state() == "STANDING");
  CHECK(max_joint_speed() < kMaxJointVel);
}

TEST(commands_are_clamped_to_gait_limits)
{
  GC g; g.standing_idle(); g.nav(true);
  g.drive(4.0, 5.0, 5.0, 9.0);       // absurd
  JointState j = GC::js();
  CHECK(all_finite(j));
  CHECK(max_joint_speed() < kMaxJointVel);
  CHECK_MSG(W().count_logs(1) == 0, "unreachable-target warnings under overspeed command");
}

XFAIL_TEST(nan_cmd_vel_must_not_reach_the_servos,
           "std::clamp(NaN, lo, hi) returns NaN, so a NaN /cmd_vel poisons every joint command")
{
  GC g; g.standing_idle(); g.nav(true);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  g.drive(2.0, nan, 0.0, 0.0);
  CHECK_MSG(all_finite(GC::js()), "NaN joint positions published to /joint_commands");
}

TEST(inf_cmd_vel_is_clamped_to_limits)
{
  GC g; g.standing_idle(); g.nav(true);
  const double inf = std::numeric_limits<double>::infinity();
  g.drive(3.0, inf, -inf, inf);
  CHECK(all_finite(GC::js()));
  CHECK(max_joint_speed() < kMaxJointVel);
}

TEST(sits_down_when_stand_request_withdrawn)
{
  GC g; g.standing_idle();
  g.stand(false);
  g.run(0.5);
  CHECK(GC::state() == "SITTING_DOWN");
  CHECK(!GC::ready());
  g.run(2.5);
  CHECK(GC::state() == "SITTING");
  JointState j = GC::js();
  CHECK_NEAR(j.position[2], -2.071, 0.01);
}

TEST(will_not_sit_down_while_still_walking)
{
  GC g; g.walking();
  g.stand(false);
  g.drive(2.0, 0.06);                    // nav still on, command still fresh -> keeps walking
  CHECK(GC::state() == "WALKING");
  g.nav(false);
  g.run(4.0);                            // gait drains, then it may sit
  CHECK(GC::state() == "SITTING_DOWN" || GC::state() == "SITTING");
}

TEST(stand_request_reversal_mid_rise_is_smooth)
{
  GC g; g.stand(true); g.run(1.0);
  g.stand(false); g.run(0.1); g.stand(true); g.run(0.1); g.stand(false); g.run(3.0);
  CHECK(max_joint_speed() < kMaxJointVel);
  CHECK(GC::state() == "SITTING");
}

TEST(full_mission_profile_is_smooth_finite_and_reachable)
{
  GC g;
  g.stand(true); g.run(2.5);
  g.nav(true);
  g.drive(4.0, 0.08); g.drive(3.0, 0.0, 0.0, 0.5); g.drive(3.0, -0.06, 0.04, -0.3);
  g.nav(false); g.run(3.0);
  g.stand(false); g.run(3.0);
  for (auto & m : W().msgs<JointState>("/joint_commands")) {
    for (double p : m.second.position) {CHECK(std::isfinite(p));}
  }
  INFO("peak joint speed over the mission: %.2f rad/s", max_joint_speed());
  CHECK(max_joint_speed() < kMaxJointVel);
  CHECK_MSG(W().count_logs(1) == 0, "%zu warnings during a normal mission", W().count_logs(1));
}

TEST(publish_rates_are_50hz_and_10hz)
{
  GC g; g.run(10.0);
  const size_t nj = W().count("/joint_commands"), np = W().count("/posture_ready");
  CHECK_MSG(nj >= 495 && nj <= 505, "joint_commands: %zu in 10 s", nj);
  CHECK_MSG(np >= 98 && np <= 102, "posture_ready: %zu in 10 s (expect 10 Hz)", np);
  auto m = W().msgs<JointState>("/joint_commands");
  double gap = 0;
  for (size_t i = 1; i < m.size(); ++i) {gap = std::max(gap, m[i].first - m[i - 1].first);}
  CHECK(gap < 0.0201);
}

TEST(calibration_signs_and_offsets_are_applied_per_joint)
{
  // Baseline run
  std::vector<double> base;
  {GC g; g.standing_idle(); base = GC::js().position;}
  reset_world();
  std::vector<double> signs(18, 1.0), offs(18, 0.0);
  for (int i = 0; i < 18; ++i) {
    if ((i / 3) % 2 == 1) {signs[i] = -1.0;}       // mirrored left side
    offs[i] = 0.01 * (i + 1);
  }
  W().set_param_vec("gait_controller", "joint_signs", signs);
  W().set_param_vec("gait_controller", "joint_offsets_rad", offs);
  GC g; g.standing_idle();
  auto cal = GC::js().position;
  REQUIRE(cal.size() == 18 && base.size() == 18);
  for (int k = 0; k < 18; ++k) {CHECK_NEAR(cal[k], signs[k] * base[k] + offs[k], 1e-9);}
}

TEST(wrong_length_calibration_falls_back_with_warning)
{
  std::vector<double> base;
  {GC g; g.standing_idle(); base = GC::js().position;}
  reset_world();
  W().set_param_vec("gait_controller", "joint_signs", {1, -1, 1});
  W().set_param_vec("gait_controller", "joint_offsets_rad", std::vector<double>(7, 0.5));
  GC g; g.standing_idle();
  auto cal = GC::js().position;
  for (int k = 0; k < 18; ++k) {CHECK_NEAR(cal[k], base[k], 1e-9);}
  CHECK(W().count_logs(1) >= 2);
}

TEST(infeasible_geometry_warns_but_stays_finite)
{
  reset_world();
  W().set_param("gait_controller", "stand_height_m", 0.30);   // legs cannot reach the ground
  GC g; g.standing_idle();
  CHECK(all_finite(GC::js()));
  CHECK(W().count_logs(1, "outside reach") > 0);
}

TEST(restand_after_sitting_works)
{
  GC g; g.standing_idle(); g.stand(false); g.run(3.0);
  CHECK(GC::state() == "SITTING");
  g.stand(true); g.run(2.5);
  CHECK(GC::state() == "STANDING" && GC::ready());
}

TEST(stand_request_alone_without_nav_never_walks_even_with_cmd_spam)
{
  GC g; g.standing_idle();
  for (int i = 0; i < 100; ++i) {W().inject("/cmd_vel", twist(0.08, 0.08, 0.5)); g.run(0.02);}
  CHECK(GC::state() == "STANDING");
}

int main(int argc, char ** argv) {return tf::run_main(argc, argv, "gait_controller", reset_world);}
