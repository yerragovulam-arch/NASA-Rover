cmake_minimum_required(VERSION 3.8)
project(psyche_rover_tests)

# NOTE: this file was NOT compiled in the environment where the suite was written (no cmake/ament
# available). The authoritative runner is run_all_tests.py; this only wires the same tests into
# `colcon test`. It shells out to the runner, so there is one source of truth.
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME psyche_cpp_and_static
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/run_all_tests.py --quick
            --report ${CMAKE_CURRENT_BINARY_DIR}/test_report)
  add_test(NAME psyche_mujoco_sim
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/run_all_tests.py --only sim
            --report ${CMAKE_CURRENT_BINARY_DIR}/test_report_sim)
  set_tests_properties(psyche_mujoco_sim PROPERTIES TIMEOUT 1800)
endif()

ament_package()
