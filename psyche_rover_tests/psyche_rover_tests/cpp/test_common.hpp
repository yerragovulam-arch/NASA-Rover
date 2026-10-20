// Shared helpers for the shim-based node tests: synthetic depth camera,
// scene description, message builders, costmap inspection.
#pragma once
#include <sys/wait.h>
#include <unistd.h>

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "shim_msgs.hpp"
#include "test_framework.hpp"

namespace tt
{

inline rclcpp::World & W() {return rclcpp::world();}
inline void reset_world() {rclcpp::world().reset();}

// Locate a workspace-relative file (tests run from anywhere).
inline std::string ws_path(const std::string & rel)
{
#ifdef PSYCHE_SRC_DIR
  return std::string(PSYCHE_SRC_DIR) + "/" + rel;
#else
  return "../" + rel;
#endif
}

// ---------------------------------------------------------------- camera ---
// Matches psyche_hexapod_sim/mujoco_sim_node.py: square pixels, vertical FOV 50 deg,
// level camera, 0.17 m above ground when standing.
struct Cam
{
  int w{160}, h{100};
  double fx{0}, fy{0}, cx{0}, cy{0};
  double height{0.17};
  double tilt_deg{0.0};  // same convention as terrain_mapper's camera_tilt_deg

  static Cam sim_default(int w = 160, int h = 100, double fovy_deg = 50.0)
  {
    Cam c;
    c.w = w; c.h = h;
    c.fy = (h / 2.0) / std::tan(fovy_deg * M_PI / 360.0);
    c.fx = c.fy;
    c.cx = w / 2.0; c.cy = h / 2.0;
    return c;
  }
};

// Ground surface seen from the rover: flat ground plus boxes / pits / ramps.
// Coordinates: F = forward (m), L = left (m); heights above the nominal ground.
struct Region {double F0, F1, L0, L1, height;};
struct Scene
{
  std::vector<Region> regions;
  double slope_fwd_deg{0.0};   // ground rises ahead by tan(slope)*F
  double slope_side_deg{0.0};  // ground rises to the left by tan(slope)*L
  Scene & box(double F0, double F1, double L0, double L1, double h)
  {
    regions.push_back({F0, F1, L0, L1, h}); return *this;
  }
  // A wall is just a very tall box.
  Scene & wall(double F0, double L0, double L1, double thickness = 0.05)
  {
    return box(F0, F0 + thickness, L0, L1, 2.0);
  }
  double ground(double F, double L) const
  {
    double g = std::tan(slope_fwd_deg * M_PI / 180.0) * F + std::tan(slope_side_deg * M_PI / 180.0) * L;
    for (const auto & r : regions) {
      if (F >= r.F0 && F <= r.F1 && L >= r.L0 && L <= r.L1) {g = r.height;}
    }
    return g;
  }
};

// z-depth image in metres, 0 = no return. Ray-marches the scene with
// bisection refinement. Uses the terrain_mapper's camera/tilt convention
// so mapper(frame) round-trips to the scene when its parameters match.
inline std::vector<float> render_depth(const Cam & cam, const Scene & sc, double max_range = 8.0)
{
  std::vector<float> out(static_cast<size_t>(cam.w) * cam.h, 0.0f);
  const double t_rad = cam.tilt_deg * M_PI / 180.0;
  const double c = std::cos(t_rad), s = std::sin(t_rad);
  for (int v = 0; v < cam.h; ++v) {
    for (int u = 0; u < cam.w; ++u) {
      const double xc = (u - cam.cx) / cam.fx, yc = (v - cam.cy) / cam.fy;
      const double dF = yc * s + c, dL = -xc, dZ = s - yc * c;
      auto below = [&](double t) {
          return cam.height + t * dZ <= sc.ground(t * dF, t * dL);
        };
      double t = 0.05;
      bool hit = false;
      double prev = t;
      while (t <= max_range) {
        if (below(t)) {hit = true; break;}
        prev = t;
        t += std::max(0.004, 0.003 * t);
      }
      if (!hit) {continue;}
      double lo = prev, hi = t;
      for (int i = 0; i < 24; ++i) {
        const double mid = 0.5 * (lo + hi);
        (below(mid) ? hi : lo) = mid;
      }
      out[static_cast<size_t>(v) * cam.w + u] = static_cast<float>(0.5 * (lo + hi));
    }
  }
  return out;
}

inline sensor_msgs::msg::Image image16(const Cam & cam, const std::vector<float> & m)
{
  sensor_msgs::msg::Image im;
  im.header.frame_id = "camera_link";
  im.width = cam.w; im.height = cam.h; im.encoding = "16UC1"; im.step = cam.w * 2;
  im.data.resize(m.size() * 2);
  for (size_t i = 0; i < m.size(); ++i) {
    double mm = std::round(m[i] * 1000.0);
    uint16_t v = (m[i] <= 0.0f || !std::isfinite(m[i])) ? 0 : static_cast<uint16_t>(std::min(mm, 65535.0));
    im.data[2 * i] = v & 0xFF; im.data[2 * i + 1] = v >> 8;
  }
  return im;
}

inline sensor_msgs::msg::Image image32(const Cam & cam, const std::vector<float> & m)
{
  sensor_msgs::msg::Image im;
  im.header.frame_id = "camera_link";
  im.width = cam.w; im.height = cam.h; im.encoding = "32FC1"; im.step = cam.w * 4;
  im.data.resize(m.size() * 4);
  for (size_t i = 0; i < m.size(); ++i) {
    float f = m[i];
    std::memcpy(&im.data[4 * i], &f, 4);
  }
  return im;
}

inline sensor_msgs::msg::CameraInfo cam_info(const Cam & cam)
{
  sensor_msgs::msg::CameraInfo ci;
  ci.width = cam.w; ci.height = cam.h;
  ci.k = {cam.fx, 0, cam.cx, 0, cam.fy, cam.cy, 0, 0, 1};
  return ci;
}

// ------------------------------------------------------------- costmaps ---
using Grid = nav_msgs::msg::OccupancyGrid;

// Costmap in the mapper's layout: row = forward, col = left.
inline Grid make_grid(int w_cells, int h_cells, double res, int8_t fill = 0)
{
  Grid g;
  g.header.frame_id = "base_link";
  g.info.resolution = static_cast<float>(res);
  g.info.width = w_cells; g.info.height = h_cells;
  g.info.origin.position.x = 0.0;
  g.info.origin.position.y = -w_cells * res / 2.0;
  g.data.assign(static_cast<size_t>(w_cells) * h_cells, fill);
  return g;
}
inline int8_t * cell(Grid & g, double F, double L)
{
  const int gx = static_cast<int>((L - g.info.origin.position.y) / g.info.resolution);
  const int gy = static_cast<int>((F - g.info.origin.position.x) / g.info.resolution);
  if (gx < 0 || gx >= static_cast<int>(g.info.width) || gy < 0 || gy >= static_cast<int>(g.info.height)) {
    return nullptr;
  }
  return &g.data[static_cast<size_t>(gy) * g.info.width + gx];
}
inline int8_t at(const Grid & g, double F, double L)
{
  return *cell(const_cast<Grid &>(g), F, L);
}
inline void fill_rect(Grid & g, double F0, double F1, double L0, double L1, int8_t v)
{
  const double r = g.info.resolution;
  for (double F = F0; F < F1; F += r / 2) {
    for (double L = L0; L < L1; L += r / 2) {
      if (auto * c = cell(g, F, L)) {*c = v;}
    }
  }
}
inline int count_value(const Grid & g, int8_t v, double F0 = 0, double F1 = 1e9, double L0 = -1e9, double L1 = 1e9)
{
  int n = 0;
  for (int gy = 0; gy < static_cast<int>(g.info.height); ++gy) {
    for (int gx = 0; gx < static_cast<int>(g.info.width); ++gx) {
      const double F = (gy + 0.5) * g.info.resolution + g.info.origin.position.x;
      const double L = (gx + 0.5) * g.info.resolution + g.info.origin.position.y;
      if (F < F0 || F > F1 || L < L0 || L > L1) {continue;}
      if (g.data[static_cast<size_t>(gy) * g.info.width + gx] == v) {++n;}
    }
  }
  return n;
}
inline int count_occupied(const Grid & g, double F0 = 0, double F1 = 1e9, double L0 = -1e9, double L1 = 1e9)
{
  int n = 0;
  for (int gy = 0; gy < static_cast<int>(g.info.height); ++gy) {
    for (int gx = 0; gx < static_cast<int>(g.info.width); ++gx) {
      const double F = (gy + 0.5) * g.info.resolution + g.info.origin.position.x;
      const double L = (gx + 0.5) * g.info.resolution + g.info.origin.position.y;
      if (F < F0 || F > F1 || L < L0 || L > L1) {continue;}
      if (g.data[static_cast<size_t>(gy) * g.info.width + gx] >= 50) {++n;}
    }
  }
  return n;
}

// Run fn in a child process; false if it crashes or does not finish within `seconds`.
template<class F>
bool finishes_within(int seconds, F fn)
{
  fflush(stdout);
  pid_t pid = fork();
  if (pid == 0) {alarm(seconds); fn(); _exit(0);}
  int st = 0;
  waitpid(pid, &st, 0);
  return WIFEXITED(st) && WEXITSTATUS(st) == 0;
}

// ------------------------------------------------------------- messaging ---
template<class M>
const M * last_msg(const std::string & topic, std::vector<std::pair<double, M>> & store)
{
  store = W().msgs<M>(topic);
  return store.empty() ? nullptr : &store.back().second;
}

inline std_msgs::msg::Bool boolmsg(bool b)
{
  std_msgs::msg::Bool m; m.data = b; return m;
}
inline geometry_msgs::msg::Twist twist(double vx, double vy, double wz)
{
  geometry_msgs::msg::Twist t; t.linear.x = vx; t.linear.y = vy; t.angular.z = wz; return t;
}
inline std_msgs::msg::Float32MultiArray sectors(std::vector<float> v)
{
  std_msgs::msg::Float32MultiArray m; m.data = std::move(v); return m;
}

}  // namespace tt
