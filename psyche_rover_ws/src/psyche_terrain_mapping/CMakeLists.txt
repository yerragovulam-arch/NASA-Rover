cmake_minimum_required(VERSION 3.8)
project(psyche_terrain_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(terrain_mapper_node src/terrain_mapper_node.cpp)
ament_target_dependencies(terrain_mapper_node
  rclcpp sensor_msgs nav_msgs cv_bridge image_transport OpenCV)

add_executable(path_picker_node src/path_picker_node.cpp)
ament_target_dependencies(path_picker_node
  rclcpp nav_msgs std_msgs)

install(TARGETS
  terrain_mapper_node
  path_picker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
