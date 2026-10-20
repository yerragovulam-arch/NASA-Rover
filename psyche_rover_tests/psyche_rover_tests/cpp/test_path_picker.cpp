// path_picker_node (real source via shim): heading choice, gap sizes, blocked / unknown
// handling, symmetry, parameter boundaries, malformed costmaps, and the full
// mapper -> picker -> reactive_avoidance chain on synthetic scenes.
#include <random>

#include "nodes.hpp"

using namespace tt;
using psyche_terrain_mapping::PathPicker;
using psyche_terrain_mapping::TerrainMapper;
using psyche_obstacle_avoidance::ReactiveAvoidance;

namespace
{
constexpr double kDeg = M_PI / 180.0;

struct Pick {double heading_deg; bool found; bool published;};

// Publish a costmap to a fresh path_picker and read its answer.
Pick pick(const Grid & g, bool keep_world = false)
{
  if (!keep_world) {reset_world();}
  auto node = std::make_shared<PathPicker>();
  W().clear_log();
  W().inject("/terrain_costmap", g);
  W().spin();
  auto h = W().msgs<std_msgs::msg::Float32>("/planned_heading");
  auto f = W().msgs<std_msgs::msg::Bool>("/path_found");
  if (h.empty() || f.empty()) {return {0, false, false};}
  return {h.back().second.data / kDeg, f.back().second.data, true};
}

Grid free_grid() {return make_grid(40, 40, 0.1, 0);}

// Paint a clear corridor (half-width 0.4 m) along heading theta; everything else occupied.
Grid corridor(double theta_deg, double len = 2.6)
{
  Grid g = make_grid(40, 40, 0.1, 100);
  const double c = std::cos(theta_deg * kDeg), s = std::sin(theta_deg * kDeg);
  for (double d = 0.0; d <= len; d += 0.025) {
    for (double lat = -0.45; lat <= 0.45; lat += 0.025) {
      if (auto * p = cell(g, d * c - lat * s, d * s + lat * c)) {*p = 0;}
    }
  }
  return g;
}

}  // namespace

TEST(open_ground_goes_straight_and_reports_found)
{
  Pick p = pick(free_grid());
  REQUIRE(p.published);
  CHECK_NEAR(p.heading_deg, 0.0, 1e-3);
  CHECK(p.found);
}

TEST(everything_unknown_is_not_a_path_and_defaults_straight)
{
  Pick p = pick(make_grid(40, 40, 0.1, -1));
  REQUIRE(p.published);
  CHECK(!p.found);
  CHECK_NEAR(p.heading_deg, 0.0, 1e-3);
}

TEST(everything_occupied_is_not_a_path)
{
  Pick p = pick(make_grid(40, 40, 0.1, 100));
  CHECK(p.published && !p.found);
}

TEST(wall_distance_vs_min_acceptable_clearance)
{
  for (double F : {0.22, 0.32}) {  // clearly closer than min_acceptable_clear_m (0.5)
    Grid g = free_grid(); fill_rect(g, F, F + 0.1, -2.0, 2.0, 100);
    Pick p = pick(g);
    CHECK_MSG(!p.found, "wall at %.2f m should not count as a path", F);
  }
  for (double F : {0.65, 1.0, 2.0}) {
    Grid g = free_grid(); fill_rect(g, F, F + 0.1, -2.0, 2.0, 100);
    Pick p = pick(g);
    CHECK_MSG(p.found, "wall at %.2f m leaves room to move", F);
  }
}

TEST(obstacle_on_left_steers_right_and_vice_versa)
{
  Grid gl = free_grid(); fill_rect(gl, 0.8, 1.3, -0.2, 2.0, 100);   // blocks centre + left
  Grid gr = free_grid(); fill_rect(gr, 0.8, 1.3, -2.0, 0.2, 100);   // blocks centre + right
  Pick pl = pick(gl), pr = pick(gr);
  INFO("blocked-left -> heading %.0f deg, blocked-right -> heading %.0f deg", pl.heading_deg, pr.heading_deg);
  CHECK_MSG(pl.heading_deg < -5.0, "should steer right (negative), got %.1f", pl.heading_deg);
  CHECK_MSG(pr.heading_deg > 5.0, "should steer left (positive), got %.1f", pr.heading_deg);
  CHECK(pl.found && pr.found);
  CHECK_NEAR(pl.heading_deg, -pr.heading_deg, 5.1);  // mirror images within one heading step
}

TEST(corridor_heading_is_recovered_across_the_fan)
{
  // Only a swath-wide corridor at angle theta is free: the picker must find it, on both sides.
  for (double th : {-60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0}) {
    Pick p = pick(corridor(th));
    REQUIRE(p.published);
    CHECK_MSG(std::abs(p.heading_deg - th) <= 5.1, "corridor at %.0f deg -> picked %.1f deg", th, p.heading_deg);
    CHECK_MSG(p.found, "corridor at %.0f deg not reported as found", th);
  }
}

TEST(heading_never_exceeds_configured_fan)
{
  std::mt19937 rng(5);
  std::uniform_int_distribution<int> coin(0, 9);
  for (int n = 0; n < 150; ++n) {
    Grid g = make_grid(40, 40, 0.1, 0);
    for (auto & c : g.data) {const int r = coin(rng); c = r < 2 ? 100 : (r < 3 ? -1 : 0);}
    Pick p = pick(g);
    REQUIRE(p.published);
    CHECK_MSG(std::isfinite(p.heading_deg) && std::abs(p.heading_deg) <= 60.0 + 1e-3, "heading %.2f out of fan", p.heading_deg);
  }
  // a 30 deg fan setting is respected
  reset_world(); W().set_param("path_picker", "max_heading_deg", 30.0);
  Pick p = pick(corridor(60.0), true);
  CHECK_MSG(std::abs(p.heading_deg) <= 30.0 + 1e-3, "30 deg fan violated: %.1f", p.heading_deg);
}

TEST(gap_between_obstacles_vs_footprint_width)
{
  // Two long obstacle blocks leave a gap of width g centred on the rover. The footprint is
  // 0.6 m wide (half_width 0.3): wider gaps pass straight through, narrower ones do not.
  struct C {double gap; bool straight;};
  for (C c : {C{0.40, false}, C{0.50, false}, C{0.80, true}, C{1.00, true}, C{1.50, true}}) {
    Grid g = free_grid();
    fill_rect(g, 0.35, 3.9, c.gap / 2, 2.0, 100);
    fill_rect(g, 0.35, 3.9, -2.0, -c.gap / 2, 100);
    Pick p = pick(g);
    INFO("gap %.2f m -> heading %.0f deg found=%d", c.gap, p.heading_deg, p.found);
    if (c.straight) {CHECK_MSG(std::abs(p.heading_deg) < 1e-3 && p.found, "gap %.2f should pass straight", c.gap);}
    else {CHECK_MSG(!(p.found && std::abs(p.heading_deg) < 1e-3),
                    "gap %.2f m is narrower than the rover: must not drive straight into it", c.gap);}
  }
}

TEST(gap_exactly_footprint_width_is_characterised)
{
  Grid g = free_grid();
  fill_rect(g, 0.35, 3.9, 0.3, 2.0, 100);
  fill_rect(g, 0.35, 3.9, -2.0, -0.3, 100);
  Pick p = pick(g);
  INFO("0.60 m gap (= footprint) -> heading %.0f deg found=%d", p.heading_deg, p.found);
  CHECK(p.published);
}

XFAIL_TEST(swath_sampling_is_left_right_symmetric,
           "swath loop `for (lat=-w; lat<=w; lat+=res)` loses the +half_width sample to float rounding "
           "(0.1 steps end at +0.2): footprint is -0.30..+0.20 m, obstacles on the LEFT edge are missed")
{
  // One obstacle cell fully INSIDE the 0.3 m half-width, 0.2-0.3 m to the left vs right.
  Grid gl = free_grid(), gr = free_grid();
  fill_rect(gl, 0.8, 0.9, 0.21, 0.29, 100);
  fill_rect(gr, 0.8, 0.9, -0.29, -0.21, 100);
  Pick pl = pick(gl), pr = pick(gr);
  INFO("obstacle 0.2-0.3 m left of path -> heading %.0f deg; same on right -> heading %.0f deg",
       pl.heading_deg, pr.heading_deg);
  CHECK_MSG(std::abs(pl.heading_deg) > 1e-3, "left-side obstacle inside the footprint was not seen");
  CHECK_MSG(std::abs(pr.heading_deg) > 1e-3, "right-side obstacle inside the footprint was not seen");
}

TEST(symmetric_blockage_picks_a_side_consistently)
{
  Grid g = free_grid(); fill_rect(g, 1.0, 1.4, -0.35, 0.35, 100);
  Pick a = pick(g), b = pick(g);
  INFO("symmetric blockage: heading %.0f deg (right-turn bias if negative)", a.heading_deg);
  CHECK(std::abs(a.heading_deg) > 1.0 && a.found);
  CHECK_NEAR(a.heading_deg, b.heading_deg, 1e-9);
}

TEST(unknown_cells_beyond_allowance_are_not_driven_into)
{
  // Free for 0.8 m then unknown: acceptable clearance.   Free for 0.3 m only: not.
  Grid ga = make_grid(40, 40, 0.1, -1); fill_rect(ga, 0.0, 0.9, -2.0, 2.0, 0);
  Grid gb = make_grid(40, 40, 0.1, -1); fill_rect(gb, 0.0, 0.3, -2.0, 2.0, 0);
  CHECK(pick(ga).found);
  CHECK(!pick(gb).found);
}

TEST(near_field_unknown_is_tolerated)
{
  // The camera cannot see the floor right under it, so the nearest cells are always unknown.
  Grid g = free_grid(); fill_rect(g, 0.0, 0.2, -2.0, 2.0, -1);   // two nearest rows unknown
  Pick p = pick(g);
  CHECK(p.found && std::abs(p.heading_deg) < 1e-3);
}

TEST(small_map_edge_counts_as_unknown)
{
  Grid g = make_grid(10, 10, 0.1, 0);  // only 1 m x 1 m mapped
  Pick p = pick(g);
  CHECK(p.found && std::abs(p.heading_deg) < 1e-3);
}

TEST(resolution_change_gives_same_decision)
{
  Grid coarse = free_grid(); fill_rect(coarse, 1.0, 1.4, -0.2, 2.0, 100);
  Grid fine = make_grid(80, 80, 0.05, 0); fill_rect(fine, 1.0, 1.4, -0.2, 2.0, 100);
  Pick a = pick(coarse), b = pick(fine);
  INFO("0.10 m grid -> %.0f deg, 0.05 m grid -> %.0f deg", a.heading_deg, b.heading_deg);
  CHECK(a.heading_deg < 0 && b.heading_deg < 0);
  CHECK_NEAR(a.heading_deg, b.heading_deg, 15.0);
}

TEST(lookahead_and_clearance_parameters_take_effect)
{
  reset_world();
  W().set_param("path_picker", "min_acceptable_clear_m", 1.5);
  Grid g = free_grid(); fill_rect(g, 1.0, 1.1, -2.0, 2.0, 100);   // wall at 1.0 m
  CHECK(!pick(g, true).found);                                      // 1.0 < 1.5 now unacceptable
}

XFAIL_TEST(zero_resolution_costmap_does_not_hang,
           "clear_distance_for_heading() loops `lat += res`; resolution 0 (or negative) is an infinite loop")
{
  const bool ok = finishes_within(3, [] {
    Grid g = free_grid(); g.info.resolution = 0.0f;
    pick(g);
  });
  CHECK_MSG(ok, "path_picker hung on a costmap with resolution 0");
}

TEST(degenerate_costmaps_do_not_crash)
{
  for (auto dims : {std::pair<int, int>{0, 0}, {1, 1}, {1, 40}, {40, 1}, {400, 400}}) {
    const bool ok = finishes_within(10, [&] {
      Grid g = make_grid(dims.first, dims.second, 0.1, 0);
      pick(g);
    });
    CHECK_MSG(ok, "crash/hang on %dx%d costmap", dims.first, dims.second);
  }
}

TEST(costmap_with_nonzero_origin_is_handled)
{
  Grid g = free_grid();
  g.info.origin.position.x = -1.0;  // map extends 1 m behind the rover
  fill_rect(g, -1.0, 0.0, -2.0, 2.0, 100);   // junk behind the rover must be ignored? (cells at F<0)
  Pick p = pick(g);
  INFO("origin.x=-1: found=%d heading=%.0f", p.found, p.heading_deg);
  CHECK(p.published);
}

// -------------------------------------------------- full perception chain ---

namespace
{
struct Chain
{
  std::shared_ptr<TerrainMapper> mapper;
  std::shared_ptr<PathPicker> picker;
  std::shared_ptr<ReactiveAvoidance> avoid;
  Cam cam;
  // allowance < 0 keeps the shipped unknown_allowance_m (0.3).
  explicit Chain(const Scene & s, double allowance = -1.0) : cam(Cam::sim_default(320, 200))
  {
    reset_world();
    if (allowance >= 0) {W().set_param("path_picker", "unknown_allowance_m", allowance);}
    mapper = std::make_shared<TerrainMapper>();
    picker = std::make_shared<PathPicker>();
    avoid = std::make_shared<ReactiveAvoidance>();
    W().inject("/oak/stereo/camera_info", cam_info(cam));
    W().spin();
    W().inject("/oak/stereo/image_raw", image16(cam, render_depth(cam, s)));
    W().spin();
  }
  Pick result() const
  {
    auto h = W().msgs<std_msgs::msg::Float32>("/planned_heading");
    auto f = W().msgs<std_msgs::msg::Bool>("/path_found");
    if (h.empty() || f.empty()) {return {0, false, false};}
    return {h.back().second.data / kDeg, f.back().second.data, true};
  }
  Grid grid() const {return W().msgs<Grid>("/terrain_costmap").back().second;}
};
}  // namespace

XFAIL_TEST(chain_flat_empty_floor_yields_a_path_with_shipped_params,
           "terrain_mapper cannot see the floor nearer than ~0.4 m (camera 0.17 m high, level) but "
           "path_picker only tolerates 0.3 m of unknown: an empty flat floor reports path_found=false")
{
  Chain c{Scene{}};
  Pick p = c.result();
  REQUIRE(p.published);
  Grid g = c.grid();
  // How far out is the +/-0.3 m footprint corridor fully mapped?
  double first_full = -1;
  for (double F = 0.05; F < 2.0 && first_full < 0; F += 0.1) {
    bool all = true;
    for (double L = -0.3; L <= 0.2; L += 0.1) {all = all && at(g, F, L) >= 0;}
    if (all) {first_full = F;}
  }
  INFO("flat floor: corridor fully mapped from %.1f m onward; picker allows unknown only up to 0.3 m", first_full);
  INFO("picker answer: heading %.1f deg, path_found=%d", p.heading_deg, p.found);
  CHECK_MSG(p.found, "no path reported on an empty flat floor");
}

TEST(chain_flat_floor_with_wider_unknown_allowance_works)
{
  // Shows the fix: with unknown_allowance_m ~0.6 the same scene gives a straight path.
  Chain c{Scene{}, 0.6};
  Pick p = c.result();
  REQUIRE(p.published);
  CHECK(p.found);
  CHECK_NEAR(p.heading_deg, 0.0, 1e-3);
}

XFAIL_TEST(chain_obstacle_beside_path_steers_around_it,
           "mapper costmap is a camera wedge (+/-37 deg) with unknown stripes; picker treats unknown as "
           "blocked, so free space beside an obstacle looks worse (~0.7 m) than driving straight at the "
           "known 12 cm block (~1.1 m) and it keeps heading 0")
{
  for (int side : {+1, -1}) {   // +1: block on the left, -1: block on the right
    Scene s;
    if (side > 0) {s.box(1.0, 1.5, -0.1, 1.2, 0.12);} else {s.box(1.0, 1.5, -1.2, 0.1, 0.12);}
    Chain c{s, 0.6};
    Pick p = c.result();
    REQUIRE(p.published);
    INFO("block on %s: heading %.1f deg found=%d", side > 0 ? "left" : "right", p.heading_deg, p.found);
    CHECK_MSG(p.heading_deg * side < -5.0, "should steer away from the %s block, got %.1f deg",
              side > 0 ? "left" : "right", p.heading_deg);
  }
}

TEST(chain_heading_sign_convention_matches_avoidance)
{
  // Whatever the picker says, reactive_avoidance must turn the same way (positive = left).
  for (double deg : {-40.0, -15.0, 15.0, 40.0}) {
    reset_world();
    auto av = std::make_shared<ReactiveAvoidance>();
    W().inject("/nav_enabled", boolmsg(true));
    std_msgs::msg::Float32 h; h.data = static_cast<float>(deg * kDeg);
    W().inject("/planned_heading", h);
    W().inject("/obstacle_sectors", sectors({3, 3, 3, 3, 3}));
    W().spin();
    auto cmd = W().msgs<geometry_msgs::msg::Twist>("/cmd_vel");
    REQUIRE(!cmd.empty());
    CHECK_MSG(cmd.back().second.angular.z * deg > 0.0, "heading %.0f deg -> angular.z %.2f", deg, cmd.back().second.angular.z);
    CHECK(cmd.back().second.linear.x > 0.0);
  }
}

TEST(chain_close_wall_is_not_a_path_far_wall_is)
{
  Pick pn;
  {
    Chain near_wall{Scene{}.wall(0.35, -3.0, 3.0), 0.6};   // each Chain resets the shared world
    pn = near_wall.result();
  }
  REQUIRE(pn.published);
  CHECK_MSG(!pn.found, "wall at 0.35 m must not be a path (heading %.1f)", pn.heading_deg);
  Chain far_wall{Scene{}.wall(1.6, -3.0, 3.0), 0.6};
  Pick pf = far_wall.result();
  CHECK_MSG(pf.found, "wall at 1.6 m leaves room to walk (heading %.1f)", pf.heading_deg);
}

TEST(chain_low_obstacle_is_walked_over_not_avoided)
{
  Chain c{Scene{}.box(1.0, 1.5, -0.6, 0.6, 0.03), 0.6};   // 3 cm: steppable
  Pick p = c.result();
  REQUIRE(p.published);
  CHECK(p.found);
  CHECK_NEAR(p.heading_deg, 0.0, 1e-3);
}

int main(int argc, char ** argv) {return tf::run_main(argc, argv, "path_picker", reset_world);}
