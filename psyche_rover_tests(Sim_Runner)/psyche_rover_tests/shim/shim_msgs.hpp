// All message types used by the Psyche nodes, as plain structs.
#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#define SHIM_MSG(T) using SharedPtr = std::shared_ptr<T>; using ConstSharedPtr = std::shared_ptr<const T>;

namespace std_msgs::msg {
struct Header { rclcpp::Time stamp; std::string frame_id; };
struct Bool { bool data{false}; SHIM_MSG(Bool) };
struct String { std::string data; SHIM_MSG(String) };
struct Float32 { float data{0.f}; SHIM_MSG(Float32) };
struct Float32MultiArray { std::vector<float> data; SHIM_MSG(Float32MultiArray) };
}  // namespace std_msgs::msg

namespace geometry_msgs::msg {
struct Vector3 { double x{0}, y{0}, z{0}; };
struct Point { double x{0}, y{0}, z{0}; };
struct Quaternion { double x{0}, y{0}, z{0}, w{1}; };
struct Pose { Point position; Quaternion orientation; };
struct Twist { Vector3 linear, angular; SHIM_MSG(Twist) };
}  // namespace geometry_msgs::msg

namespace sensor_msgs::msg {
struct Image {
  std_msgs::msg::Header header;
  uint32_t height{0}, width{0};
  std::string encoding;
  uint8_t is_bigendian{0};
  uint32_t step{0};
  std::vector<uint8_t> data;
  SHIM_MSG(Image)
};
struct CameraInfo {
  std_msgs::msg::Header header;
  uint32_t height{0}, width{0};
  std::string distortion_model;
  std::vector<double> d;
  std::array<double, 9> k{};
  std::array<double, 9> r{};
  std::array<double, 12> p{};
  SHIM_MSG(CameraInfo)
};
struct JointState {
  std_msgs::msg::Header header;
  std::vector<std::string> name;
  std::vector<double> position, velocity, effort;
  SHIM_MSG(JointState)
};
}  // namespace sensor_msgs::msg

namespace nav_msgs::msg {
struct MapMetaData {
  rclcpp::Time map_load_time;
  float resolution{0.f};
  uint32_t width{0}, height{0};
  geometry_msgs::msg::Pose origin;
};
struct OccupancyGrid {
  std_msgs::msg::Header header;
  MapMetaData info;
  std::vector<int8_t> data;
  SHIM_MSG(OccupancyGrid)
};
}  // namespace nav_msgs::msg
