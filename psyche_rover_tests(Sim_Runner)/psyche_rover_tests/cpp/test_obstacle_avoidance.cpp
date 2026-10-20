// obstacle_detector_node + reactive_avoidance_node (real sources via shim).
#include <limits>

#include "nodes.hpp"

using namespace tt;
using psyche_obstacle_avoidance::ObstacleDetector;
using psyche_obstacle_avoidance::ReactiveAvoidance;

namespace
{
const Cam kCam = Cam::sim_default(320, 200);

struct Det
{
  std::shared_ptr<ObstacleDetector> node;
  std::vector<float> sectors;
  bool flag{false};
  bool published{false};
  Det() {node = std::make_shared<ObstacleDetector>();}
  void feed(const sensor_msgs::msg::Image & img)
  {
    W().clear_log();
    W().inject("/oak/stereo/image_raw", img);
    W().spin();
    auto s = W().msgs<std_msgs::msg::Float32MultiArray>("/obstacle_sectors");
    auto f = W().msgs<std_msgs::msg::Bool>("/obstacle_detected");
    published = !s.empty() && !f.empty();
    if (published) {sectors = s.back().second.data; flag = f.back().second.data;}
  }
};

// Depth image: constant `bg` everywhere, with rectangle [c0,c1) x [r0,r1) set to `fg`.
std::vector<float> plane(float bg) {return std::vector<float>(static_cast<size_t>(kCam.w) * kCam.h, bg);}
void paint(std::vector<float> & d, int c0, int c1, int r0, int r1, float v)
{
  for (int r = r0; r < r1; ++r) {for (int c = c0; c < c1; ++c) {d[static_cast<size_t>(r) * kCam.w + c] = v;}}
}

struct Avoid
{
  std::shared_ptr<ReactiveAvoidance> node;
  Avoid() {node = std::make_shared<ReactiveAvoidance>();}
  void nav(bool on) {W().inject("/nav_enabled", boolmsg(on)); W().spin();}
  void heading(double rad) {std_msgs::msg::Float32 m; m.data = static_cast<float>(rad); W().inject("/planned_heading", m); W().spin();}
  // returns last cmd published since the call (zero twist + false if none)
  bool cmd(std::vector<float> s, geometry_msgs::msg::Twist & out)
  {
    const size_t before = W().count("/cmd_vel");
    W().inject("/obstacle_sectors", sectors(std::move(s)));
    W().spin();
    auto c = W().msgs<geometry_msgs::msg::Twist>("/cmd_vel");
    if (c.size() == before) {return false;}
    out = c.back().second;
    return true;
  }
  static std_msgs::msg::Float32MultiArray sectors(std::vector<float> v) {return tt::sectors(std::move(v));}
};
}  // namespace

// =========================================================== detector ===

TEST(detector_sector_order_is_left_to_right)
{
  Det d;
  auto img = plane(3.0f);
  paint(img, 0, 40, 70, 160, 0.8f);                       // far left columns, inside the row band
  d.feed(image16(kCam, img));
  REQUIRE(d.published && d.sectors.size() == 5);
  CHECK_NEAR(d.sectors[0], 0.8, 0.002);
  for (int i = 1; i < 5; ++i) {CHECK_NEAR(d.sectors[i], 3.0, 0.002);}
  CHECK(d.flag);
}

TEST(detector_clear_scene_reports_no_obstacle)
{
  Det d;
  d.feed(image16(kCam, plane(3.0f)));
  REQUIRE(d.published);
  CHECK(!d.flag);
  for (float s : d.sectors) {CHECK_NEAR(s, 3.0, 0.002);}
}

TEST(detector_trigger_distance_boundary)
{
  struct C {float m; bool flag;};
  for (C c : {C{0.5f, true}, C{0.99f, true}, C{1.0f, true}, C{1.02f, false}, C{2.0f, false}}) {
    Det d;
    d.feed(image16(kCam, plane(c.m)));
    REQUIRE(d.published);
    CHECK_MSG(d.flag == c.flag, "plane at %.2f m: flag=%d expected %d", c.m, d.flag, c.flag);
  }
}

TEST(detector_ignores_invalid_depth_and_reports_max_range)
{
  Det d;
  for (float bad : {0.0f, 0.10f, 7.5f}) {   // no-data, nearer than 0.15 m, farther than 6 m
    d.feed(image16(kCam, plane(bad)));
    REQUIRE(d.published);
    CHECK_MSG(!d.flag, "invalid value %.2f m triggered an obstacle", bad);
    for (float s : d.sectors) {CHECK_NEAR(s, 6.0, 1e-4);}   // "infinity" reported as max_valid_range_m
  }
}

TEST(detector_only_scans_the_configured_row_band)
{
  // Band is rows 35%..85% of 200 = 70..170.
  Det d;
  auto above = plane(3.0f); paint(above, 0, 320, 0, 60, 0.5f);       // sky / far-ceiling rows
  d.feed(image16(kCam, above));
  CHECK(!d.flag);
  auto below = plane(3.0f); paint(below, 0, 320, 175, 200, 0.5f);    // bottom rows (floor at feet)
  d.feed(image16(kCam, below));
  CHECK(!d.flag);
  auto inside = plane(3.0f); paint(inside, 0, 320, 100, 102, 0.5f);
  d.feed(image16(kCam, inside));
  CHECK(d.flag);
}

TEST(detector_sector_count_parameter_and_ragged_width)
{
  for (int n : {1, 3, 7}) {
    reset_world();
    W().set_param("obstacle_detector", "num_sectors", static_cast<double>(n));
    Det d;
    d.feed(image16(kCam, plane(3.0f)));
    REQUIRE(d.published);
    CHECK_MSG(static_cast<int>(d.sectors.size()) == n, "num_sectors=%d gave %zu", n, d.sectors.size());
  }
  // width 323 with 5 sectors: the last sector absorbs the remainder columns
  reset_world();
  Cam odd = Cam::sim_default(323, 200);
  Det d;
  std::vector<float> img(static_cast<size_t>(odd.w) * odd.h, 3.0f);
  for (int r = 100; r < 110; ++r) {img[static_cast<size_t>(r) * odd.w + 322] = 0.7f;}
  d.feed(image16(odd, img));
  REQUIRE(d.published);
  CHECK_NEAR(d.sectors[4], 0.7, 0.002);
}

TEST(detector_32fc1_works_when_millimeter_flag_is_off)
{
  reset_world();
  W().set_param("obstacle_detector", "depth_is_millimeters", false);
  Det d;
  auto img = plane(3.0f); paint(img, 130, 190, 90, 150, 0.8f);
  d.feed(image32(kCam, img));
  REQUIRE(d.published);
  CHECK_NEAR(d.sectors[2], 0.8, 0.002);
  CHECK(d.flag);
}

XFAIL_TEST(detector_32fc1_works_with_default_parameters,
           "depth_is_millimeters defaults to true and then takes the uint16 branch for ANY encoding, "
           "so a 32FC1 (metres) image such as Gazebo's is read as garbage unless the override file is used")
{
  Det d;
  d.feed(image32(kCam, plane(3.0f)));
  REQUIRE(d.published);
  for (float s : d.sectors) {CHECK_MSG(std::abs(s - 3.0f) < 0.01f, "32FC1 3.0 m plane read as %.3f m", s);}
}

XFAIL_TEST(detector_flat_floor_must_not_look_like_an_obstacle,
           "rows 35-85% of a level camera 0.17 m above the floor see the floor itself from ~0.5 m, "
           "inside obstacle_trigger_m=1.0 m: an empty flat room raises /obstacle_detected permanently")
{
  Det d;
  Scene floor;  // nothing on it
  d.feed(image16(kCam, render_depth(kCam, floor)));
  REQUIRE(d.published);
  float lo = 99; for (float s : d.sectors) {lo = std::min(lo, s);}
  INFO("empty floor: nearest 'obstacle' %.2f m (trigger %.1f m)", lo, 1.0);
  CHECK_MSG(!d.flag, "empty flat floor raised obstacle_detected (nearest %.2f m)", lo);
}

TEST(detector_flat_floor_clean_if_trigger_is_inside_floor_distance)
{
  // Workaround evidence: a trigger below the nearest visible floor (about 0.52 m) stays quiet.
  reset_world();
  W().set_param("obstacle_detector", "obstacle_trigger_m", 0.45);
  Det d;
  d.feed(image16(kCam, render_depth(kCam, Scene{})));
  REQUIRE(d.published);
  CHECK(!d.flag);
}

XFAIL_TEST(detector_single_speckle_pixel_must_not_trigger,
           "no outlier filtering: one stray 0.5 m pixel (stereo speckle, specular glint) in a clear scene "
           "sets /obstacle_detected")
{
  Det d;
  auto img = plane(3.0f);
  img[static_cast<size_t>(120) * kCam.w + 160] = 0.5f;
  d.feed(image16(kCam, img));
  REQUIRE(d.published);
  CHECK_MSG(!d.flag, "a single bad pixel raised obstacle_detected");
}

XFAIL_TEST(detector_zero_sectors_does_not_crash,
           "num_sectors=0 divides by zero (width / num_sectors_)")
{
  const bool ok = finishes_within(3, [] {
    reset_world();
    W().set_param("obstacle_detector", "num_sectors", 0.0);
    Det d;
    d.feed(image16(kCam, plane(3.0f)));
  });
  CHECK_MSG(ok, "obstacle_detector crashed with num_sectors=0");
}

TEST(detector_rejects_bad_images_without_publishing)
{
  Det d;
  sensor_msgs::msg::Image empty; empty.encoding = "16UC1";
  d.feed(empty);
  CHECK(!d.published);
  auto trunc = image16(kCam, plane(3.0f));
  trunc.data.resize(100);
  d.feed(trunc);
  CHECK(!d.published);
}

TEST(detector_measures_distance_accurately)
{
  for (float m : {0.3f, 0.75f, 1.234f, 2.5f, 5.9f}) {
    Det d;
    d.feed(image16(kCam, plane(m)));
    REQUIRE(d.published);
    CHECK_NEAR(d.sectors[2], m, 0.0011);
  }
}

// ======================================================= reactive avoidance ===

TEST(avoid_silent_until_nav_enabled)
{
  Avoid a; geometry_msgs::msg::Twist t;
  CHECK(!a.cmd({3, 3, 3, 3, 3}, t));        // never enabled -> stays quiet
  a.nav(true);
  CHECK(a.cmd({3, 3, 3, 3, 3}, t));
}

TEST(avoid_nav_disable_publishes_immediate_stop)
{
  Avoid a; a.nav(true);
  geometry_msgs::msg::Twist t;
  a.cmd({3, 3, 3, 3, 3}, t);
  CHECK(t.linear.x > 0);
  const size_t before = W().count("/cmd_vel");
  a.nav(false);
  auto c = W().msgs<geometry_msgs::msg::Twist>("/cmd_vel");
  REQUIRE(c.size() == before + 1);
  CHECK(c.back().second.linear.x == 0.0 && c.back().second.angular.z == 0.0);
  CHECK(!a.cmd({3, 3, 3, 3, 3}, t));        // and ignores sectors again
}

TEST(avoid_clear_path_drives_forward_at_configured_speed)
{
  Avoid a; a.nav(true); geometry_msgs::msg::Twist t;
  REQUIRE(a.cmd({3, 3, 3, 3, 3}, t));
  CHECK_NEAR(t.linear.x, 0.08, 1e-9);
  CHECK_NEAR(t.angular.z, 0.0, 1e-9);
}

TEST(avoid_follows_planned_heading_when_clear)
{
  Avoid a; a.nav(true); geometry_msgs::msg::Twist t;
  a.heading(0.3);
  REQUIRE(a.cmd({3, 3, 3, 3, 3}, t));
  CHECK_NEAR(t.angular.z, 0.3, 1e-6);
  CHECK_NEAR(t.linear.x, 0.08 * std::cos(0.3), 1e-6);
  a.heading(-1.0);                                  // beyond angular_speed: clamped
  REQUIRE(a.cmd({3, 3, 3, 3, 3}, t));
  CHECK_NEAR(t.angular.z, -0.4, 1e-6);
}

TEST(avoid_slows_and_turns_toward_clearer_side_when_blocked_ahead)
{
  Avoid a; a.nav(true); geometry_msgs::msg::Twist t;
  REQUIRE(a.cmd({3, 3, 0.8f, 0.5f, 0.5f}, t));    // left clearer
  CHECK_NEAR(t.linear.x, 0.08 * 0.3, 1e-9);
  CHECK_NEAR(t.angular.z, 0.4, 1e-9);
  REQUIRE(a.cmd({0.5f, 0.5f, 0.8f, 3, 3}, t));    // right clearer
  CHECK_NEAR(t.angular.z, -0.4, 1e-9);
}

TEST(avoid_critical_distance_backs_off_while_turning_away)
{
  Avoid a; a.nav(true); geometry_msgs::msg::Twist t;
  REQUIRE(a.cmd({3, 3, 0.3f, 0.6f, 0.6f}, t));
  CHECK_NEAR(t.linear.x, -0.04, 1e-9);
  CHECK_NEAR(t.angular.z, 0.4, 1e-9);
  REQUIRE(a.cmd({0.6f, 0.6f, 0.3f, 3, 3}, t));
  CHECK_NEAR(t.angular.z, -0.4, 1e-9);
}

TEST(avoid_distance_thresholds_boundaries)
{
  Avoid a; a.nav(true); geometry_msgs::msg::Twist t;
  REQUIRE(a.cmd({3, 3, 0.399f, 3, 3}, t)); CHECK(t.linear.x < 0);          // just inside critical -> reverse
  REQUIRE(a.cmd({3, 3, 0.41f, 3, 3}, t));  CHECK(t.linear.x > 0);          // just outside -> slow turn, no reverse
  REQUIRE(a.cmd({3, 3, 1.0f, 3, 3}, t));   CHECK(t.linear.x > 0 && t.linear.x < 0.05);  // == safe -> slow turn
  REQUIRE(a.cmd({3, 3, 1.01f, 3, 3}, t));  CHECK_NEAR(t.linear.x, 0.08, 1e-9);
}

TEST(avoid_planned_heading_never_overrides_close_obstacle)
{
  Avoid a; a.nav(true); geometry_msgs::msg::Twist t;
  a.heading(0.5);                                    // planner says go left
  REQUIRE(a.cmd({0.6f, 0.6f, 0.7f, 3, 3}, t));      // but left is blocked, right is clear
  CHECK_MSG(t.angular.z < 0, "turned into the blocked side (angular.z=%.2f)", t.angular.z);
}

TEST(avoid_tie_between_sides_is_deterministic)
{
  Avoid a; a.nav(true); geometry_msgs::msg::Twist t1, t2;
  REQUIRE(a.cmd({0.6f, 0.6f, 0.7f, 0.6f, 0.6f}, t1));
  REQUIRE(a.cmd({0.6f, 0.6f, 0.7f, 0.6f, 0.6f}, t2));
  INFO("equal clearance both sides -> angular.z %.1f (negative = right-turn bias)", t1.angular.z);
  CHECK_NEAR(t1.angular.z, t2.angular.z, 1e-12);
  CHECK(std::abs(t1.angular.z) > 0.1);
}

TEST(avoid_output_stays_within_gait_limits_fuzz)
{
  Avoid a; a.nav(true);
  unsigned s = 99;
  auto rnd = [&]() {s = s * 1664525u + 1013904223u; return (s >> 8) / double(1 << 24);};
  for (int i = 0; i < 5000; ++i) {
    a.heading((rnd() - 0.5) * 2.0 * M_PI / 3.0);   // planner range +/-60 deg
    std::vector<float> sec(5);
    for (auto & v : sec) {v = static_cast<float>(0.1 + rnd() * 5.9);}
    geometry_msgs::msg::Twist t;
    REQUIRE(a.cmd(sec, t));
    CHECK_MSG(std::abs(t.linear.x) <= 0.08 + 1e-9 && std::abs(t.angular.z) <= 0.5, "cmd (%.3f, %.3f) outside gait limits", t.linear.x, t.angular.z);
    CHECK(std::isfinite(t.linear.x) && std::isfinite(t.angular.z));
  }
}

TEST(avoid_handles_short_or_empty_sector_arrays)
{
  Avoid a; a.nav(true); geometry_msgs::msg::Twist t;
  CHECK(!a.cmd({}, t));
  for (size_t n : {1u, 2u, 3u, 9u}) {
    std::vector<float> v(n, 0.7f);
    CHECK_MSG(a.cmd(v, t), "no command for %zu sectors", n);
  }
}

XFAIL_TEST(avoid_reacts_to_obstacles_in_footprint_beside_the_centre_sector,
           "only the centre sector (+/-7 deg, about +/-0.13 m at 1 m) is checked, but the footprint is "
           "+/-0.30 m: an obstacle 0.35 m ahead in the adjacent sector is ignored and the rover drives on")
{
  Avoid a; a.nav(true); geometry_msgs::msg::Twist t;
  for (auto s : {std::vector<float>{3, 0.35f, 3, 3, 3}, std::vector<float>{3, 3, 3, 0.35f, 3}}) {
    REQUIRE(a.cmd(s, t));
    CHECK_MSG(t.linear.x < 0.08 - 1e-9 || std::abs(t.angular.z) > 0.0,
              "full speed straight ahead with an obstacle at 0.35 m inside the footprint");
  }
}

XFAIL_TEST(avoid_nan_sector_must_not_mean_clear,
           "NaN compares false against every threshold, so a NaN centre sector falls through to the "
           "'path clear, full speed' branch")
{
  Avoid a; a.nav(true); geometry_msgs::msg::Twist t;
  REQUIRE(a.cmd({3, 3, std::numeric_limits<float>::quiet_NaN(), 3, 3}, t));
  CHECK_MSG(t.linear.x < 0.08 - 1e-9, "drove at full speed on a NaN sector");
}

XFAIL_TEST(avoid_heading_beyond_90deg_must_not_reverse,
           "linear.x = linear_speed * cos(planned_heading) is not clamped, so a heading >90 deg "
           "(stale or garbage /planned_heading) commands reverse while 'clear'")
{
  Avoid a; a.nav(true); geometry_msgs::msg::Twist t;
  a.heading(2.0);
  REQUIRE(a.cmd({3, 3, 3, 3, 3}, t));
  CHECK_MSG(t.linear.x >= 0.0, "reversing (%.3f) on a clear path", t.linear.x);
}

TEST(avoid_after_reenable_still_ignores_until_sector_arrives)
{
  Avoid a; a.nav(true); a.nav(false); a.nav(true);
  geometry_msgs::msg::Twist t;
  const size_t before = W().count("/cmd_vel");
  // re-enable alone must not publish a drive command (only the stop on disable)
  CHECK(W().count("/cmd_vel") == before);
  CHECK(a.cmd({3, 3, 3, 3, 3}, t));
}

int main(int argc, char ** argv) {return tf::run_main(argc, argv, "obstacle_avoidance", reset_world);}
