cmake_minimum_required(VERSION 3.8)
project(psyche_hexapod_gait)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(gait_controller_node src/gait_controller_node.cpp)
ament_target_dependencies(gait_controller_node
  rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS
  gait_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # ROS-free tests of the kinematics and gait generator (plain executables,
  # exit nonzero on failure).
  enable_testing()
  add_executable(test_kinematics test/test_kinematics.cpp)
  add_executable(test_gait test/test_gait.cpp)
  add_test(NAME kinematics COMMAND test_kinematics)
  add_test(NAME gait COMMAND test_gait)
endif()

ament_package()
