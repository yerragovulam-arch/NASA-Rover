# Psyche test report

Generated 2026-10-20 07:39:00

| suite | kind | pass | fail | known issues (xfail) | xpass | skipped |
|---|---|---:|---:|---:|---:|---:|
| test_kinematics | cpp | 1 | 0 | 0 | 0 | 0 |
| test_gait | cpp | 1 | 0 | 0 | 0 | 0 |
| test_gait_extended | cpp | 17 | 0 | 3 | 0 | 0 |
| test_terrain_mapper | cpp | 20 | 0 | 1 | 0 | 0 |
| test_path_picker | cpp | 21 | 0 | 4 | 0 | 0 |
| test_obstacle_avoidance | cpp | 22 | 0 | 7 | 0 | 0 |
| test_gait_controller | cpp | 19 | 0 | 1 | 0 | 0 |
| test_state_machine | cpp | 16 | 0 | 4 | 0 | 0 |
| test_config_consistency | cpp | 11 | 0 | 1 | 0 | 0 |
| test_system_faults | cpp | 13 | 0 | 2 | 0 | 0 |
| test_static | static | 15 | 0 | 0 | 0 | 0 |
| test_sim_gait | sim | 31 | 0 | 5 | 0 | 2 |
| test_sim_autonomy | sim | 11 | 0 | 8 | 0 | 1 |
| **total** | | **198** | **0** | **36** | **0** | **3** |

## Failures / unexpected passes (0)


## Known issues - tests that currently fail by design (36)

Each is a real defect or limitation found by the suite, kept as an expected failure (XFAIL). When one is fixed the test reports XPASS and the run fails until the marker is removed.

- **test_gait_extended::ik_nan_input_must_be_flagged_unreachable** - leg_ik(NaN) returns reachable=true with NaN angles (comparisons with NaN are false)
- **test_gait_extended::gait_generator_rejects_nan_command** - GaitGenerator::set_target(NaN) poisons every foot target with NaN
- **test_gait_extended::reach_formula_from_readme_is_sufficient** - README reach formula only checks MAX reach; low stand_height + short neutral_reach puts the foot inside the minimum-reach hole |femur-tibia|
    - 67 combos passed the README formula, 3 still unreachable (first: stand_height=0.04 neutral_reach=0.10 (forward))
- **test_terrain_mapper::gentle_5deg_slope_is_traversable** - mapper compares against a flat reference plane: any ground that rises >5 cm over its range (about 1.4 deg at 4 m) is marked occupied, so a 5 deg ramp becomes a wall
- **test_path_picker::swath_sampling_is_left_right_symmetric** - swath loop `for (lat=-w; lat<=w; lat+=res)` loses the +half_width sample to float rounding (0.1 steps end at +0.2): footprint is -0.30..+0.20 m, obstacles on the LEFT edge are missed
    - obstacle 0.2-0.3 m left of path -> heading 0 deg; same on right -> heading 10 deg
- **test_path_picker::zero_resolution_costmap_does_not_hang** - clear_distance_for_heading() loops `lat += res`; resolution 0 (or negative) is an infinite loop
- **test_path_picker::chain_flat_empty_floor_yields_a_path_with_shipped_params** - terrain_mapper cannot see the floor nearer than ~0.4 m (camera 0.17 m high, level) but path_picker only tolerates 0.3 m of unknown: an empty flat floor reports path_found=false
    - flat floor: corridor fully mapped from 0.4 m onward; picker allows unknown only up to 0.3 m
    - picker answer: heading 0.0 deg, path_found=0
- **test_path_picker::chain_obstacle_beside_path_steers_around_it** - mapper costmap is a camera wedge (+/-37 deg) with unknown stripes; picker treats unknown as blocked, so free space beside an obstacle looks worse (~0.7 m) than driving straight at the known 12 cm block (~1.1 m) and it keeps heading 0
    - block on left: heading 0.0 deg found=1
    - block on right: heading 25.0 deg found=1
- **test_obstacle_avoidance::detector_32fc1_works_with_default_parameters** - depth_is_millimeters defaults to true and then takes the uint16 branch for ANY encoding, so a 32FC1 (metres) image such as Gazebo's is read as garbage unless the override file is used
- **test_obstacle_avoidance::detector_flat_floor_must_not_look_like_an_obstacle** - rows 35-85% of a level camera 0.17 m above the floor see the floor itself from ~0.5 m, inside obstacle_trigger_m=1.0 m: an empty flat room raises /obstacle_detected permanently
    - empty floor: nearest 'obstacle' 0.53 m (trigger 1.0 m)
- **test_obstacle_avoidance::detector_single_speckle_pixel_must_not_trigger** - no outlier filtering: one stray 0.5 m pixel (stereo speckle, specular glint) in a clear scene sets /obstacle_detected
- **test_obstacle_avoidance::detector_zero_sectors_does_not_crash** - num_sectors=0 divides by zero (width / num_sectors_)
- **test_obstacle_avoidance::avoid_reacts_to_obstacles_in_footprint_beside_the_centre_sector** - only the centre sector (+/-7 deg, about +/-0.13 m at 1 m) is checked, but the footprint is +/-0.30 m: an obstacle 0.35 m ahead in the adjacent sector is ignored and the rover drives on
- **test_obstacle_avoidance::avoid_nan_sector_must_not_mean_clear** - NaN compares false against every threshold, so a NaN centre sector falls through to the 'path clear, full speed' branch
- **test_obstacle_avoidance::avoid_heading_beyond_90deg_must_not_reverse** - linear.x = linear_speed * cos(planned_heading) is not clamped, so a heading >90 deg (stale or garbage /planned_heading) commands reverse while 'clear'
- **test_gait_controller::nan_cmd_vel_must_not_reach_the_servos** - std::clamp(NaN, lo, hi) returns NaN, so a NaN /cmd_vel poisons every joint command
- **test_state_machine::persistent_obstacle_flag_does_not_livelock_the_rover** - TRAVERSE_PATH exits on the very next tick whenever /obstacle_detected is true, even though the planner just found a clear heading; with the flag permanently high (e.g. the flat-floor false positive, or any object within 1 m) nav is enabled for ~0.2 s per cycle and the rover never walks
    - 60 s with permanent obstacle flag: nav enabled 5.8 s in total, longest stretch 0.20 s, 29 enable pulses
- **test_state_machine::perceive_waits_for_live_sensor_data** - PERCEIVE is a stub: the machine proceeds to STAND_UP and mapping with no depth stream at all
- **test_state_machine::restarted_state_machine_revokes_nav_enabled** - after a state-machine restart nobody publishes /nav_enabled=false: gait/avoidance nodes keep the stale 'true' and a robot that was mid-traverse keeps obeying commands
- **test_state_machine::late_starting_peer_learns_nav_enabled** - nav_enabled=true is published exactly once on entering TRAVERSE_PATH (stand_request, by contrast, is re-sent every tick): a reactive_avoidance/gait node that starts or restarts later stays gated forever
- **test_config_consistency::obstacle_threshold_is_below_what_the_legs_can_clear** - terrain_mapper marks >5 cm as obstacle but step_height_m is only 4 cm: a 4-5 cm obstacle is 'free' to the planner yet taller than the foot lift (and the lift has no safety margin)
    - obstacle threshold 50 mm vs foot lift 40 mm
- **test_system_faults::shipped_parameters_walk_on_an_empty_flat_floor** - with params.yaml as shipped, the full stack never leaves the map/evaluate/pick loop on an empty floor: obstacle_detector sees the floor as an obstacle and path_picker finds no path (see detector_flat_floor_* and chain_flat_empty_floor_* tests)
    - state transitions in 60 s: EVALUATE_TRAVERSABILITY x32  MAP_TERRAIN x32  PERCEIVE x1  PICK_PATH x31  STAND_UP x1  
    - /cmd_vel messages: 0
- **test_system_faults::depth_loss_must_pull_the_state_machine_out_of_traverse** - the state machine has no sensor watchdog: after the depth stream dies it stays in TRAVERSE_PATH with nav_enabled=true forever, and resumes walking without re-mapping the moment frames return
- **test_sim_gait::side_slope_drift_is_small** - open-loop gait has no heading/lateral correction: on a 10 deg side slope the robot slides downhill ~0.4 m in 26 s (27% of its forward path); nothing in the stack uses the IMU
    - 10 deg side slope: forward 1.45 m, sideways slide -0.40 m
- **test_sim_gait::asteroid_gravity_keeps_three_feet_down** - at 1.5% g (Psyche-like) the position-controlled tripod gait has no real traction: the robot hops with fewer than 3 feet down most of the time, so motion is momentum, not controlled stepping
- **test_sim_gait::step_over_4cm_wide_barrier_matches_step_height** - step_height_m is 4 cm (and params.yaml says 'step over ~step_height_m') but a 4 cm full-width barrier stops the robot dead: effective crossing limit is ~3 cm. The mapper still calls <=5 cm free
    - 4 cm barrier: reached x=0.23 m (barrier spans 0.45..0.75)
- **test_sim_gait::recovers_from_being_on_its_side_or_inverted** - no self-righting: nothing reads /imu, so a robot that lands on its side or upside down stays there with the legs flailing (z = 0.02 m)
- **test_sim_gait::servo_failure_is_detected_and_reported** - a dead servo goes unnoticed: nothing consumes /joint_states, so the stack keeps commanding forward motion (here 63% progress with a spinning/ drifting body) with no warning, nav stop or state change
    - after 30 s with a dead servo: warnings=0 errors=0 nav=1 gait=WALKING
- **test_sim_autonomy::shipped_parameters_make_progress_on_empty_floor** - with params.yaml as shipped the rover never moves on an empty flat floor: obstacle_detector reads the floor as an obstacle (/obstacle_detected permanently true) and path_picker finds no path (unknown near-field > 0.3 m). Reproduced here in physics, not just message flow
    - 60 s on an empty floor: moved 0.00 m; states {'-': 9, 'PERCEIVE': 10, 'STAND_UP': 120, 'MAP_TERRAIN': 2240, 'EVALUATE_TRAVERSABILITY': 311, 'PICK_PATH': 310}
- **test_sim_autonomy::empty_floor_path_is_straight** - planned heading jitters frame to frame (about +25 / 0 / -15 deg) because the 10 cm costmap has sparse unknown stripes and a camera wedge: on an empty floor the rover meanders (net displacement a third of path length) instead of walking straight
    - path 3.43 m, net 1.71 m, straightness 0.50
- **test_sim_autonomy::obstacle_course_is_completed** - rover approaches an obstacle to ~0.5 m and then stalls forever in the map/evaluate/pick loop: the picker cannot see free space beside it (camera wedge + unknown = blocked), so it never routes around
    - shipped 3-obstacle world: furthest x = 0.24 m after 240 s (course ends ~3.0 m)
- **test_sim_autonomy::low_steppable_box_is_walked_over** - a 3 cm box is steppable (gait test passes it) and the mapper calls it free, but obstacle_detector sees its top surface at ~0.43 m (inside the trigger) and the rover halts in front of it
    - 3 cm box at x=1.0: furthest x = 0.21 m
- **test_sim_autonomy::wall_with_no_depth_returns_is_not_driven_into** - an obstacle that returns NO depth (black, specular, transparent) is invisible: the detector skips zero pixels and the state machine ignores path_found/unknown space while in TRAVERSE_PATH, so the rover walks straight into it and keeps pushing
    - reflective wall at x=1.45: rover reached x=1.23, obstacle contact ticks: 2477
- **test_sim_autonomy::depth_speckle_does_not_freeze_the_rover** - no outlier rejection: 0.2% of pixels (about 1 in 500) returning 0.3 m - stereo speckle, specular glints, dust - keep /obstacle_detected high forever and the rover never takes a step
    - with speckle: moved 0.00 m, TRAVERSE ticks 0
- **test_sim_autonomy::autonomous_stack_climbs_a_5_degree_ramp** - 5 deg uphill slope: terrain_mapper (flat-plane reference) turns the ramp into a wall, so the autonomous stack stops although the open-loop gait climbs 5 degrees at 80% speed
    - 5 deg ramp: path 3.74 m, net 1.41 m
- **test_sim_autonomy::angled_wall_approach_does_not_touch_the_wall** - angled approach to a wall clips it: only the centre sector (+/-0.13 m at 1 m) is checked, so a leg (footprint +/-0.30 m) touches the wall before the rover reacts. Deterministic reproduction of one of the ~15% of randomised runs that do this
    - start yaw -22 deg: 370 contact ticks with the wall ({'geom9': 21, 'geom17': 85, 'geom16': 196, 'body': 68})
