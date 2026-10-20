# Wheeled rover -> six-legged (hexapod) conversion

## What changed

| Area | Before (wheeled) | Now (hexapod) |
|---|---|---|
| New package | -- | `psyche_hexapod_gait`: tripod gait + 3-DOF leg IK. Consumes `/cmd_vel`, publishes 18 joint targets on `/joint_commands`. |
| State machine | START, PERCEIVE, MAP, EVALUATE, PICK, TRAVERSE | Adds `STAND_UP` after `PERCEIVE` (waits for `/posture_ready`). Keeps `/stand_request` alive. `MAP_TERRAIN` discards maps for `map_settle_s` (1.2 s) after the gait halts. |
| Avoidance speeds | 0.3 m/s, 0.6 rad/s, reverse 0.1 | 0.08 m/s, 0.4 rad/s, new `reverse_speed` 0.04 |
| Terrain mapper | camera 0.3048 m, obstacle thresh 0.15 m | camera 0.17 m (placeholder), thresh 0.05 m (step-over limit) |
| Path picker | half-width 0.35, +/-45 deg | half-width 0.30 (footprint incl. legs), +/-60 deg (can pivot in place) |
| Launch | 5 nodes | + `gait_controller_node` in `rover_full_stack.launch.py` |

Obstacle detector, costmap projection and the gap-based path picker logic
are untouched.

## Verified here

- IK/FK round trip on 105 poses: 0 failures.
- Gait generator: forward, strafe, turn, arc and over-speed commands keep
  every foot target reachable, always have 3+ feet down, and return to
  neutral on stop.

## NOT verified (no ROS 2 / hardware in my sandbox)

- The ROS nodes were **not compiled** here. Run `colcon build` and fix
  anything I couldn't catch.
- All leg dimensions, servo signs/offsets, and `camera_height_m` are
  placeholders.
- You still need a servo bridge from `/joint_commands` to your actual
  servos.
- Gait is open-loop flat-ground: no IMU leveling or foot-contact sensing.

Build: `colcon build && source install/setup.bash &&
ros2 launch psyche_state_machine rover_full_stack.launch.py`
