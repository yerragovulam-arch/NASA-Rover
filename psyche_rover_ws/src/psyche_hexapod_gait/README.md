# psyche_hexapod_gait

Walking layer for the six-legged Psyche rover. Takes the same `/cmd_vel`
body-velocity commands the old wheeled drive used and turns them into 18
servo joint targets (6 legs x coxa / femur / tibia) using a **tripod gait**
and analytic leg inverse kinematics. Everything upstream (obstacle
detection, terrain mapping, path picking, state machine) is unchanged in
structure -- only this node knows the robot has legs.

## Leg layout

```
      front
  1 (FL)   0 (FR)
  3 (ML)   2 (MR)
  5 (RL)   4 (RR)
      rear
```

Even index = right side, odd = left. Tripod groups are `{0, 3, 4}` and
`{1, 2, 5}`: one group swings while the other holds the body up, so at
least three feet are always on the ground. Joint names follow
`leg_<i>_<front|mid|rear>_<right|left>_<coxa|femur|tibia>`
(e.g. `leg_0_front_right_coxa`).

## Node: `gait_controller_node`

| Topic | Type | Dir | Purpose |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | sub | desired body velocity. Uses `linear.x`, `linear.y` (strafe), `angular.z`. Clamped to `max_linear_mps` / `max_angular_rps`. Stale after `cmd_timeout_s`. |
| `/nav_enabled` | `std_msgs/Bool` | sub | same safety gate as before: when false the robot holds a standing pose and does not walk. |
| `/stand_request` | `std_msgs/Bool` | sub | from the state machine; true = stand up, false = sit down. |
| `/joint_commands` | `sensor_msgs/JointState` | pub | 18 position targets (rad), 50 Hz. Feed to your servo bridge. |
| `/posture_ready` | `std_msgs/Bool` | pub | true once fully standing. State machine's `STAND_UP` waits for this. |
| `/gait_state` | `std_msgs/String` | pub | `SITTING`, `STANDING_UP`, `STANDING`, `WALKING`, `SITTING_DOWN`. |

Walking only happens when **all** of these hold: fully standing, `/nav_enabled`
is true, and a `/cmd_vel` arrived within `cmd_timeout_s`. When commands stop,
the gait finishes its current half-cycle with feet back at neutral before
idling, so legs never freeze mid-swing.

## Files

- `hexapod_kinematics.hpp` -- header-only, ROS-free IK/FK, leg mounts,
  tripod groups.
- `gait_generator.hpp` -- header-only, ROS-free tripod gait (foot
  trajectories for forward, strafe and rotation, with velocity smoothing
  and stride clamping).
- `gait_controller_node.cpp` -- thin ROS 2 wrapper, posture handling,
  joint calibration.

## Tests

Plain executables, no ROS or gtest needed:

```bash
cd psyche_hexapod_gait
g++ -std=c++17 -Iinclude test/test_kinematics.cpp -o /tmp/test_kin && /tmp/test_kin
g++ -std=c++17 -Iinclude test/test_gait.cpp -o /tmp/test_gait && /tmp/test_gait
```

`test_kinematics` checks IK -> FK round trips. `test_gait` runs forward,
strafe, turn, arc and over-speed commands and checks every foot target is
reachable, 3+ feet stay down, and the legs return to neutral on stop. Both
also run under `colcon test`.

## What you MUST set for your real robot

All in `config/params.yaml`, no rebuild needed. Current values are
**placeholders**:

1. **Leg lengths** (`coxa_len_m`, `femur_len_m`, `tibia_len_m`) and body
   dimensions (`body_half_len_m`, `body_half_width_m`). If these are wrong
   you will see `leg target(s) outside reach` warnings.
2. **Servo calibration** (`joint_signs`, `joint_offsets_rad`). Mirrored
   servos (often the left side) need `-1`. Offsets set what servo angle
   corresponds to the IK's zero pose: coxa 0 = leg pointing straight out
   along its mount direction, femur 0 = upper leg horizontal, tibia 0 =
   leg fully straight, negative = knee bent.
3. **Heights/stride** (`stand_height_m`, `neutral_reach_m`,
   `step_height_m`, `max_half_stride_m`). Keep
   `femur + tibia > sqrt((neutral_reach + stride - coxa)^2 + stand_height^2)`
   or the legs can't reach.
4. **Servo bridge.** This package stops at `/joint_commands`. You need a
   node (Dynamixel SDK, PCA9685/Maestro driver, or a ros2_control hardware
   interface) that reads those positions and drives your actual servos.
   I don't know what servos/controller you're using, so that isn't included.

## Known gaps / next steps

- **Flat-ground open-loop gait.** No body pitch/roll compensation, foot
  contact sensing, or terrain-adaptive foot placement. On rough Psyche-analog
  terrain you will want IMU-based body leveling and per-leg contact
  detection (servo current or foot switches). Add this before trusting it
  off flat ground.
- **Single gait.** Tripod only (fast, stable). A slower ripple/wave gait
  (`duty` > 0.5 needs different phase offsets) would be a better fit for
  loose regolith-like terrain.
- **No power-down handling.** Sitting folds the legs to `sit_height_m`;
  make sure your servos hold torque there.
