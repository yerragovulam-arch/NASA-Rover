// terrain_mapper_node (the real source, run through the ROS shim) fed with synthetic
// depth images of known scenes. Cases: flat floor, steps around the 5 cm threshold,
// walls, drop-offs, slopes, calibration sensitivity, invalid depth, encodings.
#include <limits>
#include "nodes.hpp"

using namespace tt;
using psyche_terrain_mapping::TerrainMapper;

namespace
{
struct Rig
{
  std::shared_ptr<TerrainMapper> node;
  Cam cam;
  explicit Rig(const Cam & c = Cam::sim_default(), bool send_info = true) : cam(c)
  {
    node = std::make_shared<TerrainMapper>();
    if (send_info) {W().inject("/oak/stereo/camera_info", cam_info(cam)); W().spin();}
  }
  // Returns the last published grid (or nullptr).
  const Grid * feed(const sensor_msgs::msg::Image & img)
  {
    const size_t before = W().count("/terrain_costmap");
    W().inject("/oak/stereo/image_raw", img);
    W().spin();
    store_ = W().msgs<Grid>("/terrain_costmap");
    if (store_.size() == before) {return nullptr;}
    return &store_.back().second;
  }
  const Grid * run(const Scene & sc, bool f32 = false)
  {
    auto d = render_depth(cam, sc);
    return feed(f32 ? image32(cam, d) : image16(cam, d));
  }
private:
  std::vector<std::pair<double, Grid>> store_;
};

void set_mapper(const char * k, double v) {W().set_param("terrain_mapper", k, v);}
}  // namespace

TEST(grid_metadata_and_value_domain)
{
  Rig r;
  const Grid * g = r.run(Scene{});
  REQUIRE(g != nullptr);
  CHECK(g->info.width == 40 && g->info.height == 40);
  CHECK_NEAR(g->info.resolution, 0.1, 1e-6);
  CHECK_NEAR(g->info.origin.position.y, -2.0, 1e-9);
  CHECK_NEAR(g->info.origin.position.x, 0.0, 1e-9);
  CHECK(g->header.frame_id == "base_link");
  CHECK(g->data.size() == 1600);
  for (int8_t v : g->data) {CHECK(v == -1 || v == 0 || v == 100);}
}

TEST(flat_floor_has_no_obstacles)
{
  Rig r;
  const Grid * g = r.run(Scene{});
  REQUIRE(g != nullptr);
  const int occ = count_occupied(*g), freec = count_value(*g, 0);
  INFO("flat floor: %d free cells, %d occupied cells", freec, occ);
  CHECK_MSG(occ == 0, "%d phantom obstacles on a flat floor", occ);
  CHECK(freec > 100);
  // Mapped floor starts where the lowest image row sees it: ~0.37 m ahead for this camera.
  CHECK(count_value(*g, 0, 0.0, 0.25) == 0);
}

TEST(step_below_threshold_is_free_above_is_occupied)
{
  for (double h : {0.02, 0.04}) {
    Rig r;
    const Grid * g = r.run(Scene{}.box(1.5, 2.1, -0.6, 0.6, h));
    REQUIRE(g != nullptr);
    CHECK_MSG(count_occupied(*g) == 0, "%.0f cm step wrongly marked occupied", h * 100);
  }
  for (double h : {0.06, 0.08, 0.12}) {
    Rig r;
    const Grid * g = r.run(Scene{}.box(1.5, 2.1, -0.6, 0.6, h));
    REQUIRE(g != nullptr);
    const int occ = count_occupied(*g, 1.4, 2.2, -0.7, 0.7);
    CHECK_MSG(occ >= 4, "%.0f cm step only %d occupied cells", h * 100, occ);
    CHECK_MSG(count_occupied(*g, 0, 1.3) == 0, "%.0f cm step caused obstacles before it", h * 100);
  }
}

TEST(step_exactly_at_threshold_is_characterised)
{
  // 5.0 cm sits exactly on obstacle_height_thresh_m; document which way it falls and
  // that sub-threshold margin (4.5 cm) is safe.
  Rig r1, r2;
  const Grid * g50 = r1.run(Scene{}.box(1.5, 2.1, -0.6, 0.6, 0.050));
  const Grid * g45 = r2.run(Scene{}.box(1.5, 2.1, -0.6, 0.6, 0.045));
  REQUIRE(g50 && g45);
  INFO("5.0 cm step -> %d occupied cells; 4.5 cm step -> %d", count_occupied(*g50), count_occupied(*g45));
  CHECK(count_occupied(*g45) == 0);
}

TEST(obstacle_side_matches_scene_side)
{
  Rig r;
  const Grid * g = r.run(Scene{}.box(1.5, 2.0, 0.3, 0.9, 0.12));  // on the LEFT
  REQUIRE(g != nullptr);
  CHECK(count_occupied(*g, 0, 4, 0.2, 1.1) >= 3);
  CHECK(count_occupied(*g, 0, 4, -2.0, 0.1) == 0);
}

TEST(wall_blocks_and_occludes_what_is_behind)
{
  Rig r;
  const Grid * g = r.run(Scene{}.wall(1.0, -1.5, 1.5));
  REQUIRE(g != nullptr);
  CHECK(count_occupied(*g, 0.9, 1.3, -1.0, 1.0) >= 8);
  CHECK(count_value(*g, 0, 1.3, 4.0, -1.0, 1.0) == 0);  // occluded -> unknown, never "free"
  CHECK(count_value(*g, -1, 1.3, 4.0, -0.5, 0.5) > 20);
}

TEST(dropoff_is_flagged_but_shallow_dip_is_not)
{
  {
    Rig r;
    const Grid * g = r.run(Scene{}.box(1.5, 2.5, -0.6, 0.6, -0.12));
    REQUIRE(g != nullptr);
    CHECK_MSG(count_occupied(*g, 1.4, 2.6, -0.7, 0.7) >= 3, "12 cm drop-off not detected");
  }
  {
    Rig r;
    const Grid * g = r.run(Scene{}.box(1.5, 2.5, -0.6, 0.6, -0.03));
    REQUIRE(g != nullptr);
    CHECK(count_occupied(*g) == 0);
  }
}

XFAIL_TEST(gentle_5deg_slope_is_traversable,
           "mapper compares against a flat reference plane: any ground that rises >5 cm over its "
           "range (about 1.4 deg at 4 m) is marked occupied, so a 5 deg ramp becomes a wall")
{
  Scene s; s.slope_fwd_deg = 5.0;
  Rig r;
  const Grid * g = r.run(s);
  REQUIRE(g != nullptr);
  CHECK_MSG(count_occupied(*g) == 0, "%d cells of a walkable 5 deg ramp marked occupied", count_occupied(*g));
}

TEST(very_gentle_slope_near_field_is_ok)
{
  Scene s; s.slope_fwd_deg = 1.0;  // 1.7 cm per metre
  Rig r;
  const Grid * g = r.run(s);
  REQUIRE(g != nullptr);
  CHECK(count_occupied(*g, 0, 2.5) == 0);
  INFO("1 deg slope: first phantom obstacles beyond ~2.9 m (%d cells beyond)", count_occupied(*g, 2.9));
}

TEST(camera_height_calibration_sensitivity)
{
  // camera_height_m is a hand-measured filler. Wrong by more than the 5 cm threshold and the
  // whole floor becomes an obstacle. These cases pin down the tolerance.
  struct C {double err; bool phantom;};
  for (C c : {C{0.0, false}, C{0.02, false}, C{-0.03, false}, C{0.06, true}, C{-0.07, true}}) {
    reset_world();
    set_mapper("camera_height_m", 0.17 + c.err);
    Rig r;  // physical camera stays at 0.17
    const Grid * g = r.run(Scene{});
    REQUIRE(g != nullptr);
    const int occ = count_occupied(*g), freec = count_value(*g, 0);
    INFO("camera_height error %+.0f cm -> %d occupied / %d free cells", c.err * 100, occ, freec);
    if (c.phantom) {CHECK_MSG(occ > 0 && freec == 0, "expected the floor to turn into obstacles");}
    else {CHECK_MSG(occ == 0, "%d phantom obstacles with only %.0f cm height error", occ, c.err * 100);}
  }
}

TEST(camera_tilt_calibration_sensitivity)
{
  // Same idea for tilt: a body that pitches while walking tilts the camera, which the mapper
  // cannot know. Small error is fine near the robot; a few degrees create phantom obstacles.
  struct C {double tilt_deg; bool phantom_in_2m;};
  for (C c : {C{0.3, false}, C{1.0, false}, C{4.0, true}, C{-4.0, true}}) {
    reset_world();
    Cam cam = Cam::sim_default();
    cam.tilt_deg = c.tilt_deg;      // physical camera is tilted...
    Rig r(cam);                     // ...mapper assumes level (camera_tilt_deg = 0)
    const Grid * g = r.run(Scene{});
    REQUIRE(g != nullptr);
    const int occ = count_occupied(*g, 0, 2.0);
    INFO("unmodelled camera tilt %+.1f deg -> %d phantom obstacle cells within 2 m", c.tilt_deg, occ);
    if (c.phantom_in_2m) {CHECK(occ > 0);} else {CHECK_MSG(occ == 0, "tilt %.1f deg gave %d phantom obstacles", c.tilt_deg, occ);}
  }
}

TEST(correct_tilt_parameter_cancels_a_tilted_camera)
{
  reset_world();
  set_mapper("camera_tilt_deg", 6.0);
  Cam cam = Cam::sim_default();
  cam.tilt_deg = 6.0;
  Rig r(cam);
  const Grid * g = r.run(Scene{}.box(2.0, 2.6, -0.5, 0.5, 0.10));
  REQUIRE(g != nullptr);
  CHECK_MSG(count_occupied(*g, 0, 1.9) == 0, "phantom obstacles despite correct tilt parameter");
  CHECK(count_occupied(*g, 1.9, 2.7, -0.6, 0.6) >= 3);
}

TEST(invalid_depth_values_are_ignored)
{
  Rig r;
  const size_t n = static_cast<size_t>(r.cam.w) * r.cam.h;
  {  // all zeros (DepthAI "no data")
    const Grid * g = r.feed(image16(r.cam, std::vector<float>(n, 0.0f)));
    REQUIRE(g != nullptr);
    CHECK(count_value(*g, -1) == 1600);
  }
  const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
  for (float bad : {nan, inf, -1.0f, 0.0f, 0.10f, 9.0f, 50.0f}) {  // NaN, inf, negative, <0.15 m, >6 m
    const Grid * g = r.feed(image32(r.cam, std::vector<float>(n, bad)));
    REQUIRE(g != nullptr);
    CHECK_MSG(count_value(*g, -1) == 1600, "depth value %g was not ignored", bad);
  }
  {  // 65535 mm saturated pixels
    sensor_msgs::msg::Image im = image16(r.cam, std::vector<float>(n, 1.0f));
    for (size_t i = 0; i < n; ++i) {im.data[2 * i] = 0xFF; im.data[2 * i + 1] = 0xFF;}
    const Grid * g = r.feed(im);
    REQUIRE(g != nullptr);
    CHECK(count_value(*g, -1) == 1600);
  }
}

TEST(sparse_dropouts_do_not_create_obstacles)
{
  Rig r;
  auto d = render_depth(r.cam, Scene{});
  unsigned s = 12345;
  for (auto & v : d) {s = s * 1664525u + 1013904223u; if ((s >> 24) < 26) {v = 0.0f;}}  // ~10% holes
  const Grid * g = r.feed(image16(r.cam, d));
  REQUIRE(g != nullptr);
  CHECK(count_occupied(*g) == 0);
  CHECK(count_value(*g, 0) > 80);
}

TEST(encodings_16uc1_and_32fc1_agree)
{
  Rig r;
  Scene sc; sc.box(1.5, 2.0, -0.4, 0.4, 0.10);
  auto d = render_depth(r.cam, sc);
  const Grid g16 = *r.feed(image16(r.cam, d));
  const Grid g32 = *r.feed(image32(r.cam, d));
  int diff = 0;
  for (size_t i = 0; i < g16.data.size(); ++i) {diff += g16.data[i] != g32.data[i];}
  INFO("cells differing between 16UC1 and 32FC1: %d / 1600", diff);
  CHECK(diff <= 6);
}

TEST(unsupported_encoding_is_rejected_with_warning)
{
  Rig r;
  sensor_msgs::msg::Image im = image16(r.cam, std::vector<float>(r.cam.w * r.cam.h, 1.0f));
  im.encoding = "bgr8"; im.data.resize(static_cast<size_t>(r.cam.w) * r.cam.h * 3);
  W().clear_log();
  const Grid * g = r.feed(im);
  CHECK(g == nullptr);
  CHECK(W().count_logs(1) + W().count_logs(2) > 0);
}

TEST(waits_for_camera_info)
{
  Rig r(Cam::sim_default(), /*send_info=*/false);
  CHECK(r.run(Scene{}) == nullptr);                    // no intrinsics yet -> nothing published
  sensor_msgs::msg::CameraInfo bad = cam_info(r.cam);
  bad.k[0] = 0.0;                                       // fx = 0 is not usable
  W().inject("/oak/stereo/camera_info", bad); W().spin();
  CHECK(r.run(Scene{}) == nullptr);
  W().inject("/oak/stereo/camera_info", cam_info(r.cam)); W().spin();
  CHECK(r.run(Scene{}) != nullptr);
}

TEST(empty_or_truncated_images_do_not_crash_or_publish)
{
  Rig r;
  sensor_msgs::msg::Image empty;
  empty.encoding = "16UC1";
  CHECK(r.feed(empty) == nullptr);
  sensor_msgs::msg::Image trunc = image16(r.cam, std::vector<float>(r.cam.w * r.cam.h, 1.0f));
  trunc.data.resize(trunc.data.size() / 2);
  CHECK(r.feed(trunc) == nullptr);
}

TEST(deterministic_for_identical_frames)
{
  Rig r;
  Scene sc; sc.box(1.5, 2.0, -0.4, 0.4, 0.10);
  auto img = image16(r.cam, render_depth(r.cam, sc));
  const Grid a = *r.feed(img);
  const Grid b = *r.feed(img);
  CHECK(a.data == b.data);
}

TEST(grid_resolution_and_extent_parameters_are_honoured)
{
  reset_world();
  set_mapper("grid_resolution_m", 0.05);
  set_mapper("grid_range_m", 3.0);
  set_mapper("grid_width_m", 2.0);
  Rig r;
  const Grid * g = r.run(Scene{}.box(1.5, 2.0, -0.3, 0.3, 0.12));
  REQUIRE(g != nullptr);
  CHECK(g->info.width == 40 && g->info.height == 60);
  CHECK_NEAR(g->info.origin.position.y, -1.0, 1e-9);
  CHECK(count_occupied(*g, 1.4, 2.1, -0.4, 0.4) > 5);
}

TEST(min_range_cut_hides_things_closer_than_015m)
{
  // An obstacle nearer than min_valid_range_m (0.15 m) is invisible by design: assert the
  // boundary so nobody lowers it without noticing.
  Rig r;
  const size_t n = static_cast<size_t>(r.cam.w) * r.cam.h;
  const Grid * g = r.feed(image32(r.cam, std::vector<float>(n, 0.14f)));
  REQUIRE(g != nullptr);
  CHECK(count_value(*g, -1) == 1600);
  g = r.feed(image32(r.cam, std::vector<float>(n, 0.20f)));
  REQUIRE(g != nullptr);
  CHECK(count_value(*g, -1) < 1600);
}

int main(int argc, char ** argv) {return tf::run_main(argc, argv, "terrain_mapper", reset_world);}
