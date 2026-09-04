cmake_minimum_required(VERSION 3.8)
project(psyche_state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_executable(rover_state_machine_node src/rover_state_machine_node.cpp)
ament_target_dependencies(rover_state_machine_node rclcpp std_msgs nav_msgs)

install(TARGETS
  rover_state_machine_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
