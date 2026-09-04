# psyche_state_machine

Top-level rover behavior state machine. Orchestrates the existing
`psyche_obstacle_avoidance` package rather than duplicating any of its logic.

## States

```
START -> PERCEIVE -> MAP_TERRAIN -> EVALUATE_TRAVERSABILITY -> PICK_PATH -> TRAVERSE_PATH
                            ^                                                    |
                            |____________________ obstacle detected _____________|
```

`TRAVERSE_PATH` never exits on its own -- it just keeps driving until
`/obstacle_detected` goes true, at which point it stops the rover and loops
back to `MAP_TERRAIN` to deal with what it just found. This never terminates,
by design.

## What's real vs. stubbed

**Real:**
- The state transitions themselves and the timer loop driving them (5 Hz).
- The `/obstacle_detected` hookup to `psyche_obstacle_avoidance`.
- `MAP_TERRAIN` waits for an actual fresh `/terrain_costmap` from
  `psyche_terrain_mapping`'s `terrain_mapper_node` (real depth-to-occupancy-grid
  projection, not a stub).
- `PICK_PATH` waits for an actual `/path_found` result from `path_picker_node`
  (real gap-based heading search over the costmap). If no clear heading is
  found, it loops back to `MAP_TERRAIN` to retry rather than driving blind.
- The `/nav_enabled` safety gate: `reactive_avoidance_node` will not publish
  any drive commands until this state machine says `true`. It defaults to
  `false` on boot, so the rover does not move until it's actually reached
  `TRAVERSE_PATH`.
- `reactive_avoidance_node` now steers toward `/planned_heading` when clear,
  keeping its original close-range emergency dodging as a safety override on
  top (so a sudden obstacle gets reacted to immediately, not just on the next
  remap cycle).

**Stubbed (log + transition, no real algorithm yet):**
- `PERCEIVE` — should confirm the OAK-D depth stream is actually live before
  moving on. Currently just assumes it and proceeds.
- `EVALUATE_TRAVERSABILITY` — currently a pass-through. The real per-heading
  traversability check now lives inside `path_picker_node`'s
  `clear_distance_for_heading()`. This state is kept as a placeholder in case
  you want a separate coarser check later (e.g. "is this general area worth
  mapping at all" before committing to path picking).

See `psyche_terrain_mapping/README.md` for the terrain mapping and path
picking details, including the filler `camera_height_m` / `camera_tilt_deg`
values that need real measurements once you're on the actual rover.

## Not in scope yet

- **Drill logic.** You mentioned wanting the rover to stop and deploy a drill
  at some point later -- that'll likely be a new state (e.g. `DEPLOY_DRILL`)
  branching off `TRAVERSE_PATH` or `PICK_PATH` once you have criteria for
  when/where to drill. Not built here.

## Topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/obstacle_detected` | `std_msgs/Bool` | sub | from `obstacle_detector_node` |
| `/rover_state` | `std_msgs/String` | pub | current state name, for debugging (`ros2 topic echo /rover_state`) |
| `/nav_enabled` | `std_msgs/Bool` | pub | gates `reactive_avoidance_node`'s driving |

## Build & run

This expects to live alongside `psyche_obstacle_avoidance` in the same
workspace (`psyche_rover_ws/src/`):

```bash
cd ~/psyche_rover_ws
colcon build
source install/setup.bash
ros2 launch psyche_state_machine rover_full_stack.launch.py
```

Watch state transitions live:

```bash
ros2 topic echo /rover_state
```
