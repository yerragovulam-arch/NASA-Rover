cmake_minimum_required(VERSION 3.8)
project(psyche_obstacle_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
ament_target_dependencies(obstacle_detector_node
  rclcpp sensor_msgs std_msgs cv_bridge image_transport OpenCV)

add_executable(reactive_avoidance_node src/reactive_avoidance_node.cpp)
ament_target_dependencies(reactive_avoidance_node
  rclcpp std_msgs geometry_msgs)

install(TARGETS
  obstacle_detector_node
  reactive_avoidance_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
