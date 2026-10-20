// Co-simulation host: runs the REAL Psyche nodes (through the ROS shim) in one process
// and talks to a Python physics simulator over stdin/stdout. Python owns time:
// it sends "T <ns>" and gets back the joint command and a status line.
//
//   autonomy_host --nodes all|gait|gait,det,... [--param node.name=value ...] [--yaml file]
//
// Protocol (one command per line; DEPTH is followed by w*h*2 raw bytes):
//   INFO w h fx fy cx cy      camera_info
//   DEPTH w h                 16UC1 depth image (mm)
//   PUBB topic 0|1            publish std_msgs/Bool
//   PUBT vx vy wz             publish geometry_msgs/Twist on /cmd_vel
//   PUBF topic value          publish std_msgs/Float32
//   T t_ns                    advance simulated time, then reply:
//                               S rover gait nav cmd_vx cmd_wz obstacle path_found heading warn err
//                               J q0 ... q17        (only after the first /joint_commands)
//                               OK
//   Q                         quit
#include <cstdio>
#include <iostream>
#include <sstream>

#include "nodes.hpp"

using namespace tt;

namespace
{
struct Monitor : rclcpp::Node
{
  std::string rover{"-"}, gait{"-"};
  int nav{-1}, obstacle{-1}, path_found{-1};
  double cmd_vx{0}, cmd_wz{0}, heading{0};
  std::vector<double> q;
  std::vector<std::shared_ptr<void>> keep;

  Monitor() : rclcpp::Node("monitor")
  {
    keep.push_back(create_subscription<sensor_msgs::msg::JointState>("/joint_commands", 10,
      [this](std::shared_ptr<sensor_msgs::msg::JointState> m) {q = m->position;}));
    keep.push_back(create_subscription<std_msgs::msg::String>("/rover_state", 10,
      [this](std::shared_ptr<std_msgs::msg::String> m) {rover = m->data;}));
    keep.push_back(create_subscription<std_msgs::msg::String>("/gait_state", 10,
      [this](std::shared_ptr<std_msgs::msg::String> m) {gait = m->data;}));
    keep.push_back(create_subscription<std_msgs::msg::Bool>("/nav_enabled", 10,
      [this](std::shared_ptr<std_msgs::msg::Bool> m) {nav = m->data;}));
    keep.push_back(create_subscription<std_msgs::msg::Bool>("/obstacle_detected", 10,
      [this](std::shared_ptr<std_msgs::msg::Bool> m) {obstacle = m->data;}));
    keep.push_back(create_subscription<std_msgs::msg::Bool>("/path_found", 10,
      [this](std::shared_ptr<std_msgs::msg::Bool> m) {path_found = m->data;}));
    keep.push_back(create_subscription<std_msgs::msg::Float32>("/planned_heading", 10,
      [this](std::shared_ptr<std_msgs::msg::Float32> m) {heading = m->data;}));
    keep.push_back(create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
      [this](std::shared_ptr<geometry_msgs::msg::Twist> m) {cmd_vx = m->linear.x; cmd_wz = m->angular.z;}));
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  reset_world();
  W().record = false;   // long runs: do not keep every message
  std::string nodes = "all";
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--nodes" && i + 1 < argc) {nodes = argv[++i];}
    else if (a == "--yaml" && i + 1 < argc) {W().load_params_file(argv[++i]);}
    else if (a == "--param" && i + 1 < argc) {
      std::string kv = argv[++i];
      const size_t eq = kv.find('=');
      const size_t dot = kv.find('.');
      if (eq == std::string::npos || dot == std::string::npos) {std::fprintf(stderr, "bad --param %s\n", kv.c_str()); return 2;}
      const std::string node = kv.substr(0, dot), name = kv.substr(dot + 1, eq - dot - 1), val = kv.substr(eq + 1);
      if (val == "true" || val == "false") {W().set_param(node, name, val == "true");}
      else {W().set_param(node, name, std::stod(val));}
    }
  }
  auto has = [&](const char * n) {return nodes == "all" || nodes.find(n) != std::string::npos;};
  std::shared_ptr<void> k1, k2, k3, k4, k5, k6;
  if (has("gait")) {k1 = std::make_shared<psyche_hexapod_gait::GaitController>();}
  if (has("det")) {k2 = std::make_shared<psyche_obstacle_avoidance::ObstacleDetector>();}
  if (has("avoid")) {k3 = std::make_shared<psyche_obstacle_avoidance::ReactiveAvoidance>();}
  if (has("mapper")) {k4 = std::make_shared<psyche_terrain_mapping::TerrainMapper>();}
  if (has("picker")) {k5 = std::make_shared<psyche_terrain_mapping::PathPicker>();}
  if (has("sm")) {k6 = std::make_shared<psyche_state_machine::RoverStateMachine>();}
  Monitor mon;

  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream is(line);
    std::string cmd;
    is >> cmd;
    if (cmd == "Q") {break;}
    if (cmd == "INFO") {
      Cam c; is >> c.w >> c.h >> c.fx >> c.fy >> c.cx >> c.cy;
      W().inject("/oak/stereo/camera_info", cam_info(c));
      W().spin();
    } else if (cmd == "DEPTH") {
      int w, h; is >> w >> h;
      sensor_msgs::msg::Image im;
      im.width = w; im.height = h; im.encoding = "16UC1"; im.step = w * 2;
      im.header.stamp = rclcpp::Time{W().now_ns};
      im.data.resize(static_cast<size_t>(w) * h * 2);
      std::cin.read(reinterpret_cast<char *>(im.data.data()), static_cast<std::streamsize>(im.data.size()));
      W().inject("/oak/stereo/image_raw", im);
      W().spin();
    } else if (cmd == "PUBB") {
      std::string t; int v; is >> t >> v;
      W().inject(t, boolmsg(v != 0)); W().spin();
    } else if (cmd == "PUBT") {
      double a, b, c; is >> a >> b >> c;
      W().inject("/cmd_vel", twist(a, b, c)); W().spin();
    } else if (cmd == "PUBF") {
      std::string t; double v; is >> t >> v;
      std_msgs::msg::Float32 m; m.data = static_cast<float>(v);
      W().inject(t, m); W().spin();
    } else if (cmd == "T") {
      long long t; is >> t;
      W().advance_to_ns(t);
      std::printf("S %s %s %d %.5f %.5f %d %d %.5f %zu %zu\n", mon.rover.c_str(), mon.gait.c_str(), mon.nav,
                  mon.cmd_vx, mon.cmd_wz, mon.obstacle, mon.path_found, mon.heading,
                  W().count_logs(1), W().count_logs(2));
      if (!mon.q.empty()) {
        std::printf("J");
        for (double q : mon.q) {std::printf(" %.9g", q);}
        std::printf("\n");
      }
      std::printf("OK\n");
      std::fflush(stdout);
    }
  }
  return 0;
}
