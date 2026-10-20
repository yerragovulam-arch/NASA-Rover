# Hardware test checklist

These cannot run in a simulator. Each has a pass criterion so the result is a number, not a feeling.
Copy the table into your lab notebook and fill in the last column.

**Rules:** first power-on is always on the bench stand (body lifted, feet in the air). Keep a hand on the
power switch. Do not skip a step because the previous sim test passed.

## 1. Servo calibration (before any ground contact)

| # | Test | How | Pass | Result |
|---|---|---|---|---|
| 1.1 | Joint order | Command each of the 18 joints +0.2 rad one at a time via `/joint_commands` | The joint named in the message is the one that moves; coxa swings the leg horizontally, femur/tibia vertically | |
| 1.2 | Direction (`joint_signs`) | Same, observe direction | Right-side legs and left-side legs move as mirror images: +coxa = leg swings forward (CCW seen from above) on every leg. Fix with `joint_signs`, not by editing code | |
| 1.3 | Zero offset (`joint_offsets_rad`) | Command the *stand* pose with the body on the stand | Measured with a protractor: femur and tibia within 2 deg of the pose in `params.yaml`; all six feet at equal height within 3 mm | |
| 1.4 | Range | Command the extreme poses the gait uses (see `joint_ranges...` INFO line in the gait test output: coxa +/-0.25, femur -0.03..1.11, tibia -2.07..-1.10 rad) | No horn hits a stop; servo does not buzz at the extremes. Span per joint is at most ~65 deg (coxa ~29, femur ~65, tibia ~55), so a 180 deg servo is fine **if centred with offsets** | |
| 1.5 | Link lengths | Measure coxa/femur/tibia hinge-to-hinge | Within 1 mm of `coxa_len_m`, `femur_len_m`, `tibia_len_m` (the IK is only as good as these) | |
| 1.6 | Camera | Measure lens height above floor in the *stand* pose and camera pitch | `camera_height_m` within **1 cm** and `camera_tilt_deg` within **1 deg** (sim tests: >5 cm / >3 deg error turns the floor into obstacles) | |

## 2. Servo bridge (the missing piece: `/joint_commands` -> servos)

| # | Test | How | Pass | Result |
|---|---|---|---|---|
| 2.1 | Latency | Scope/logic analyser on a GPIO toggled when the bridge receives a message vs servo PWM edge | < 40 ms median, < 100 ms worst (sim tolerates 400 ms open loop, but the walking gait slips at high latency under load) | |
| 2.2 | Rate | `ros2 topic hz /joint_commands` at the bridge | 50 Hz +/- 2 Hz, no gaps > 40 ms over 10 min | |
| 2.3 | Watchdog | Kill the gait controller while walking | Servos hold or sit within 1 s. **Note:** nothing in the current stack does this - the bridge must | |
| 2.4 | Brown-out | Drop supply to the servo low-voltage cut-off while standing | Robot sits down in a controlled way, no twitching; MCU stays alive | |
| 2.5 | Torque limit | Measure stall current of one leg | Within the supply rating; sim says collapse happens below ~0.1x nominal torque, so check you have >5x margin at the worst temperature | |

## 3. Standing / walking on the bench then the floor

| # | Test | How | Pass | Result |
|---|---|---|---|---|
| 3.1 | Stand-up | `/stand_request` true on the floor | Reaches `STANDING` in ~2 s; all six feet loaded; body level within 2 deg (IMU) | |
| 3.2 | Sit-down | `/stand_request` false | Controlled, no drop | |
| 3.3 | Dead reckoning forward | 2 m straight line, 10 repeats, `/cmd_vel` 0.06 m/s | Distance 1.7-2.1 m (sim: 93%), lateral drift < 10 cm, heading change < 5 deg | |
| 3.4 | Dead reckoning turn | 360 deg commanded at 0.4 rad/s | Rotates 270-370 deg (sim yaw gain is only 0.80) | |
| 3.5 | Step height | Full-width barrier 2, 3, 4 cm | Crosses 3 cm (sim limit); 4 cm is the claimed `step_height_m` - record what really happens | |
| 3.6 | Slope | Ramp 5, 10, 15 deg | Climbs at the sim's speed ratios (80/65/49%) +/- 20 pts without sliding | |
| 3.7 | Surface | Carpet, tile, loose gravel, sand | No slip-induced turn > 15 deg in 2 m | |
| 3.8 | Push | 5 N lateral push at hip height | Recovers within 3 s, no foot lifts off with fewer than 3 feet down | |

## 4. Endurance and thermal

| # | Test | How | Pass | Result |
|---|---|---|---|---|
| 4.1 | 30-minute walk | Treadmill or loop, 0.06 m/s | No fault; speed in the last 5 min within 3% of the first 5; servo case temperature < 60 C (or datasheet limit - 15 C) | |
| 4.2 | Battery | Log voltage during 4.1 | Predicted run time >= 2x the longest mission | |
| 4.3 | Cold soak | Fridge/freezer 30 min at the lowest expected temperature, then stand | Stands in < 4 s, no collapse (sim: torque can drop to ~0.15x before walking fails) | |
| 4.4 | Repeatability | Same 3 m course with 2 obstacles, 20 runs | >= 18/20 reach the end without human touch. Record every failure's state-machine state | |

## 5. Perception on the real camera

| # | Test | How | Pass | Result |
|---|---|---|---|---|
| 5.1 | Flat floor | Empty floor, robot standing | `/obstacle_detected` stays **false** for 60 s (sim shows this fails with shipped params) | |
| 5.2 | Speckle | Point a laser/phone torch at the floor, shiny tile | No flicker of `/obstacle_detected` with nothing in the way | |
| 5.3 | Lighting | Direct low-angle sun, shadow edge, dim room | Depth valid pixels > 60% in the floor region; nothing walks into a wall | |
| 5.4 | Reflective / black / glass | Black foam board, mirror, glass door in the path | Rover **stops**. (Sim: a wall that returns no depth is walked into - expect the same until fixed) | |
| 5.5 | Min range | Wall at 0.15, 0.20, 0.30 m | Detected, not clipped by `min_valid_range_m` | |
| 5.6 | Depth encoding | `ros2 topic echo --once /oak/stereo/image_raw` | Encoding is `16UC1` (mm). If `32FC1`, set `depth_is_millimeters:=false` | |
