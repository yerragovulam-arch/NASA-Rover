// Pulls in the real node class declarations (their .cpp files are compiled separately,
// against the ROS shim, by run_all_tests.py).
#pragma once
#include "psyche_hexapod_gait/gait_controller.hpp"
#include "psyche_obstacle_avoidance/obstacle_detector.hpp"
#include "psyche_obstacle_avoidance/reactive_avoidance.hpp"
#include "psyche_state_machine/rover_state_machine.hpp"
#include "psyche_terrain_mapping/path_picker.hpp"
#include "psyche_terrain_mapping/terrain_mapper.hpp"
#include "test_common.hpp"
