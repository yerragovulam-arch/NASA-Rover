# psyche_rover_tests

Automated tests for the Psyche hexapod, from pure maths up to the full autonomy stack walking in MuJoCo.
**It runs your real, unmodified node sources** (`gait_controller`, `terrain_mapper`, `path_picker`,
`obstacle_detector`, `reactive_avoidance`, `rover_state_machine`) - no ROS 2 installation needed.

```
python3 run_all_tests.py            # everything except the slow soaks         (~10 min)
python3 run_all_tests.py --quick    # C++ + static, no simulator               (~1 min after a 45 s build)
python3 run_all_tests.py --long     # + 30-min endurance, 20-run repeatability (+~15 min)
python3 run_all_tests.py --only sim --filter slope
```

To watch a scenario in the MuJoCo window: `python3 watch_sim.py walk` (see the script header for scenes; `mjpython` on macOS).

Requirements: `g++` (C++17), `pip install mujoco numpy pyyaml`. Results go to `test_report.md/.json`.

**Snapshot (shipped code, no edits):** 198 pass, 0 fail, **36 known issues**, 3 slow tests skipped by default.

## What "known issue" means

A test marked `XFAIL` documents a real defect or limitation: it is *expected to fail today* and its reason is
printed. The run stays green, but if someone fixes the bug the test turns into `XPASS` and the run fails until the
marker is removed, so fixes are noticed and cannot silently regress. `FAIL` always means something broke.

## How it works

| Piece | What it does |
|---|---|
| `shim/` | A ~300-line stand-in for `rclcpp`, message types and `cv_bridge`: simulated clock, FIFO message bus, wall timers, parameters (defaults, overrides, or your `params.yaml`), log capture. The node `.cpp` files compile against it unchanged. |
| `cpp/test_*.cpp` | Unit/node/system tests. Synthetic depth images come from a ray-marched scene renderer (`test_common.hpp`) that uses the mapper's own camera/tilt convention. |
| `cpp/autonomy_host.cpp` | Runs all six real nodes in one process and talks to Python over a pipe (Python owns the clock). |
| `simtests/sim_env.py` | MuJoCo model built from *your* `generate_models.py` plus variants (slope, friction, soft ground, gravity, torque derating, extra obstacles), a **ray-cast depth camera** (`mj_multiRay`, no OpenGL), servo fault injection (limp / stuck / latency). |
| `simtests/test_static.py` | URDF vs MJCF vs SDF parity, joint-name order across 5 files, generated files up to date, YAML vs code defaults, manifests, launch files. |
| `docs/HARDWARE_CHECKLIST.md` | The tests that cannot be simulated, each with a numeric pass criterion. |

## Coverage

* **Gait/kinematics (20, plus the 2 original executables):** joint limits and 180-degree-servo span, joint speed incl. command discontinuities, IK edges and 200k-point fuzz, reach formula, stride clamp, 200 s no-drift soak, static stability margin, body velocity recovered from stance feet vs command, left/right mirror symmetry.
* **Terrain mapper (21):** flat floor, steps at 2-12 cm incl. the 5 cm threshold, walls and occlusion, drop-offs, slopes, camera height/tilt calibration sensitivity, invalid/NaN/saturated depth, encodings, missing camera info.
* **Path picker (25):** heading recovery across the +/-60 fan, gap widths vs footprint, unknown/occupied handling, symmetry, parameter boundaries, malformed costmaps (incl. hang detection in a child process), and the real mapper -> picker -> avoidance chain.
* **Obstacle detector + avoidance (29):** sector order, trigger boundaries, row band, invalid pixels, flat-floor and speckle behaviour, decision table, limits fuzz, NaN/short arrays.
* **Gait controller (20), state machine (20), config consistency (12), full-stack fault injection (15):** posture machine, gating, timeouts, calibration, restart and late-join behaviour, sensor loss, garbage frames, sudden walls, e-stop.
* **MuJoCo open loop (36):** stand/sit, distance/yaw/strafe accuracy, level body, >=3 feet down, slopes 5-25 deg up/down/side, friction 0.15-1.2, soft ground, 1/6 g and 1.5% g, cold-servo torque, command latency, step-over 2-12 cm, push recovery, tilted drops, limp/stuck servos, 120 s and 30 min endurance, repeatability.
* **MuJoCo closed loop (18):** whole stack on flat floor, walls, boxes, depth dropouts/noise/shadows/speckle/reflective surface, sensor loss, e-stop, slope, low gravity, repeatability.

## Findings (the 36 known issues), most important first

**Stack-level: the shipped parameters cannot walk on an empty floor** (`shipped_parameters_*`, confirmed in physics)

1. `obstacle_detector` scans image rows 35-85%; a level camera 0.17 m up sees the floor itself from ~0.52 m, inside `obstacle_trigger_m=1.0`, so `/obstacle_detected` is permanently true.
2. `terrain_mapper` cannot see floor nearer than ~0.4 m but `path_picker.unknown_allowance_m` is 0.3 m, so an empty floor reports `path_found=false`.
3. Four parameter changes make it walk (used by the "workaround" tests): detector trigger 0.45, avoidance safe 0.45 / critical 0.30, picker allowance 0.6.
4. Even then: the planned heading jitters frame to frame (sparse 10 cm costmap, +/-37 deg camera wedge) so it meanders; it never routes *around* an obstacle (it stops ~0.5 m before it and stalls); a 3 cm steppable box also stops it.

**Safety**

5. Wall/object that returns **no depth** (black, specular, glass) is walked into and pushed against.
6. A NaN `/cmd_vel` (or NaN sector) gives NaN joint commands / full-speed-ahead (`std::clamp` and comparisons with NaN).
7. State machine: `/obstacle_detected` high exits TRAVERSE on the next tick (livelock); `nav_enabled=true` is sent once (late/restarted nodes stay gated) and never revoked on restart; no sensor watchdog; PERCEIVE is a stub.
8. Avoidance only checks the centre sector (+/-7 deg, about +/-0.13 m at 1 m) though the footprint is +/-0.30 m.
9. No servo-fault detection (a dead mid-leg servo = 38% progress and a 78 deg spin, silently); no self-righting; open-loop gait slides 0.4 m downhill on a 10 deg side slope.

**Planner/mapper**

10. Picker swath loop `lat += res` loses the `+half_width` sample to float rounding: footprint is -0.30..+0.20 m, left-edge obstacles missed. `resolution=0` hangs it.
11. Mapper compares against a flat plane: a 5 deg ramp becomes a wall (open-loop gait climbs it at 80%).
12. Detector: 32FC1 images are read as millimetres unless the Gazebo override is loaded; one speckle pixel at 0.3 m freezes the rover; `num_sectors=0` divides by zero.

**Physical limits worth knowing (passing tests, with numbers in their INFO lines)**

* Step-over: crosses 3 cm, **blocked at 4 cm** although `step_height_m=0.04` and the mapper allows up to 5 cm (`obstacle_height_thresh_m` > `step_height_m`).
* Walking gain 0.93, yaw gain 0.80 (dead-reckoning scale factors); climbs 20 deg at 30% speed, stalls at 25.
* At 1.5% g (Psyche-like) the robot is airborne ~67% of the time: it stays upright but motion is momentum, not stepping.
* Servo torque can drop to ~0.15x before walking fails; command latency up to 400 ms is tolerated open loop.
* README reach formula ignores the minimum-reach limit `|femur-tibia|` (fails at low stand height + short reach).

## Honest limitations

* **Not run under real ROS 2 / colcon** (none available where this was written). The shim reproduces only what these nodes use; behaviour that depends on QoS, executors, threading or real timer jitter is untested. `CMakeLists.txt` here was never compiled - `run_all_tests.py` is the verified entry point.
* **Gazebo is never run.** Gazebo-vs-MuJoCo parity is checked statically (geometry, masses, limits, FOV, friction, obstacles, topics, depth format), not dynamically.
* MuJoCo depth is ray-cast (exact z-depth, no sensor noise, no stereo artifacts); noise/dropouts/speckle are injected by the tests. Camera mount offset (0.13 m ahead of body centre) is in the sim but ignored by the mapper, as in the repo.
* Thresholds were set from measured behaviour of the shipped parameters with margin. If you retune the robot, expect to update them (they are named constants at the top of each test). Servo joint limits are assumptions (`kLimitRad`, `kServoTravelRad`).
* Contact detection uses MuJoCo touch sensors scaled by gravity; "obstacle hit" means a non-foot robot geom touching an obstacle geom.

## Adding a test

C++: add `TEST(name) { CHECK(...); }` (or `XFAIL_TEST(name, "reason")`) to a `cpp/test_*.cpp`; new files go in `CPP_TESTS` in `run_all_tests.py`.
Python: `@test` / `@xfail("reason")` / `@slow` in `simtests/test_*.py`; use `sim_run(...)` or `auto(...)` helpers.
When fixing a known issue, run the suite: its test reports `XPASS`; change `XFAIL_TEST`/`@xfail` to `TEST`/`@test`.
