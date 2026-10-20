# psyche_obstacle_avoidance

Depth-based obstacle detection and reactive avoidance for the Psyche rover.
ROS2 Humble, C++, targeting Jetson Orin with an OAK-D Lite.

## What's here

- **`obstacle_detector_node`** — subscribes to the OAK-D Lite depth image,
  splits it into vertical sectors (default 5: far-left, left, center, right,
  far-right), and finds the closest valid depth reading in each. Publishes:
  - `/obstacle_sectors` (`std_msgs/Float32MultiArray`) — min distance per
    sector, left to right, in meters.
  - `/obstacle_detected` (`std_msgs/Bool`) — true if any sector is inside
    `obstacle_trigger_m`.
- **`reactive_avoidance_node`** — publishes body-velocity `/cmd_vel`
  (forward / strafe-free / yaw), which `psyche_hexapod_gait` walks with a
  tripod gait. Subscribes to `/obstacle_sectors`, drives
  straight when clear, steers toward the side with more room when something's
  ahead, stops and backs off if it's critically close. Publishes `/cmd_vel`.

This is a working baseline, not a full nav stack. It's structured so you can
swap the avoidance node for Nav2 + a costmap later without touching obstacle
detection, and so a drill node can be added as a separate package down the
line.

## Things you need to verify on your actual setup

1. **Depth topic name.** I used `/oak/stereo/image_raw` as a placeholder —
   run `ros2 topic list` while your depthai-ros driver is running and update
   `config/params.yaml` (`depth_topic`) to match. Common depthai-ros topics
   look like `/oak/stereo/image_raw` or `/stereo/depth`.
2. **Depth encoding.** DepthAI usually publishes `16UC1` depth in
   millimeters, which is what this code assumes by default. It also handles
   `32FC1` (meters) if your driver outputs that instead — it detects this
   from the image encoding automatically.
3. **`cmd_vel` consumer** — on the hexapod this is
   `psyche_hexapod_gait`'s `gait_controller_node`. Speeds in `params.yaml`
   (`linear_speed` 0.08 m/s, `angular_speed` 0.4 rad/s, `reverse_speed`
   0.04 m/s) were scaled down from the wheeled values to what a hexapod
   gait can do; keep them at or below the gait's `max_linear_mps` /
   `max_angular_rps`.
4. **You'll need depthai-ros installed and publishing the depth stream** —
   this package only consumes it, it doesn't launch the camera driver itself.
   Add that to your top-level launch file once you know the package name
   you're using (`depthai_ros_driver` is the common one).

## Build

```bash
cd ~/your_ros2_ws/src
# copy this folder in here
cd ~/your_ros2_ws
colcon build --packages-select psyche_obstacle_avoidance
source install/setup.bash
```

## Run

```bash
ros2 launch psyche_obstacle_avoidance rover_avoidance.launch.py
```

Or run nodes individually for debugging:

```bash
ros2 run psyche_obstacle_avoidance obstacle_detector_node --ros-args --params-file src/psyche_obstacle_avoidance/config/params.yaml
ros2 run psyche_obstacle_avoidance reactive_avoidance_node --ros-args --params-file src/psyche_obstacle_avoidance/config/params.yaml
```

Sanity check sector output before trusting the avoidance behavior:

```bash
ros2 topic echo /obstacle_sectors
```

## Tuning

All in `config/params.yaml`, no rebuild needed:

- `obstacle_trigger_m` / `safe_distance_m` / `critical_distance_m` — how
  close something needs to be before the rover reacts.
- `row_band_start_frac` / `row_band_end_frac` — which vertical slice of the
  depth image counts as "rover height" obstacles (avoids flagging sky or
  ground-directly-in-front-of-camera as obstacles). Tune these against real
  Psyche-analog terrain — you may need a narrower band if the camera pitch
  picks up a lot of near-ground rock texture as false positives.
- `num_sectors` — more sectors = finer steering resolution, at the cost of
  noisier per-sector minimums on rough terrain.

## Known gaps / next steps

- No filtering/temporal smoothing on sector distances yet — a single bad
  frame can trigger a reaction. Worth adding a short moving average if you
  see jittery behavior on rocky terrain.
- No IMU/odometry fusion — this is purely reactive off the current depth
  frame, no memory of terrain it just passed.
- No integration point for a goal/waypoint yet — right now "clear path"
  just means "go straight." You'll want a goal-heading input merged into
  `reactive_avoidance_node`'s logic once you're navigating to actual
  waypoints instead of just wandering obstacle-free.
