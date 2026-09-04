# psyche_terrain_mapping

Turns the OAK-D depth stream into an occupancy grid, and picks a heading
across that grid. Two nodes:

## terrain_mapper_node

Subscribes to depth (`sensor_msgs/Image`) and camera info
(`sensor_msgs/CameraInfo`). For each depth pixel:

1. Projects it into 3D using the pinhole model and the camera's intrinsics
   (`fx, fy, cx, cy` from `camera_info.k`).
2. Rotates that point by `camera_tilt_deg` (pitch about the camera's right
   axis) to account for a downward-tilted mount.
3. Converts to rover body frame: `forward`, `left`, and
   `height_above_ground = camera_height_m - (rotated vertical offset)`.
4. Bins it into a 2D grid cell. Each cell tracks the largest
   `|height_above_ground|` seen in it.

Publishes `nav_msgs/OccupancyGrid` on `/terrain_costmap`
(`frame_id = "base_link"`, +x forward, +y left). Cell values: `0` = free
(everything landing there stayed within `obstacle_height_thresh_m` of
ground level), `100` = occupied, `-1` = unknown (no depth data reached that
cell).

### About `camera_height_m` and `camera_tilt_deg`

**These are currently filler estimates: 0.3048 m (1 ft) and 0 deg, set
because that's what you had on hand.** They are not a calibrated extrinsic
or a TF transform -- just two numbers plugged into the projection math.

To tune them for real: measure the OAK-D's actual mount height off the
ground and its mount angle (0 = perfectly level, positive = tilted down).
Update `config/params.yaml`. If it's off, you'll see it directly in
`/terrain_costmap` -- flat ground getting marked occupied means the values
are wrong (usually tilt).

For a more permanent setup, this would normally be a static TF
(`camera_link` -> `base_link`) rather than two manually-set parameters --
worth switching to once the mount is finalized and you're not still
adjusting it.

## path_picker_node

Subscribes to `/terrain_costmap`. For headings from `-max_heading_deg` to
`+max_heading_deg` in `heading_step_deg` steps:

- Walks outward along that heading in `step_m` increments up to
  `lookahead_m`.
- At each distance, checks a swath `+/- rover_half_width_m` perpendicular
  to the heading (not just a single ray -- this accounts for the rover's
  actual footprint).
- Returns the distance at which it first hits an occupied cell, or an
  unknown cell beyond `unknown_allowance_m` (a small near-field allowance
  since cells right under/near the rover are often unmapped).

Picks the heading with the greatest clear distance (ties go to whichever is
closer to straight ahead). Publishes:

- `/planned_heading` (`std_msgs/Float32`, radians, `0` = straight ahead,
  positive = left) -- consumed directly by `reactive_avoidance_node` in
  `psyche_obstacle_avoidance`.
- `/path_found` (`std_msgs/Bool`) -- `true` if the best heading's clear
  distance meets `min_acceptable_clear_m`. Consumed by the state machine to
  decide whether to proceed to `TRAVERSE_PATH` or go back and re-map.

**What this is not**: a full path planner. No cost-to-goal, no multi-step
search, no memory between cycles -- it's a "which direction has the most
room right now" chooser, re-run every time a new costmap arrives. That's
consistent with the drive-until-obstacle behavior described for this
project; if you later want the rover working toward a specific target
(rather than just picking open ground), this is the piece that would need
real path planning instead.

## rover_half_width_m

Set to `0.35` as a placeholder in `config/params.yaml` -- update this to
your actual rover's half-width so the swath check reflects real footprint.

## Build & run

Part of the same workspace as `psyche_obstacle_avoidance` and
`psyche_state_machine`. Bring up the whole stack via:

```bash
ros2 launch psyche_state_machine rover_full_stack.launch.py
```

Sanity-check the map and planner independently:

```bash
ros2 topic echo /terrain_costmap --field info   # confirm it's publishing, check dims
ros2 topic echo /planned_heading
ros2 topic echo /path_found
```
