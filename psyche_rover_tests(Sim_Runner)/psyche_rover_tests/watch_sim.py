#!/usr/bin/env python3
"""Watch a test scenario in the MuJoCo viewer (real gait/autonomy nodes driving the robot).

    python3 watch_sim.py walk                 # stand up, walk forward
    python3 watch_sim.py turn | strafe        # in-place turn / sidestep
    python3 watch_sim.py slope15              # climb a 15 degree ramp
    python3 watch_sim.py step3 | step4        # 3 cm / 4 cm full-width barrier
    python3 watch_sim.py autonomy             # whole stack, empty floor, SHIPPED params (it will not move!)
    python3 watch_sim.py autonomy-fixed       # whole stack with the 4-parameter workaround
    python3 watch_sim.py wall                 # whole stack (workaround) approaching a wall
    python3 watch_sim.py asteroid             # open-loop walk at 1.5% gravity
Options: --seconds N (default 60)  --speed X (1.0 = real time, 0 = as fast as possible)
         --dry-run   run headless without a window (used to check the script works)
macOS: start with `mjpython watch_sim.py ...` instead of python3 (MuJoCo requirement).
Close the window to stop.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "simtests"))
from sim_env import CTRL_DT, Host, Sim, build_xml, openloop_script, run_loop  # noqa: E402

WORKAROUND = {"obstacle_detector.obstacle_trigger_m": 0.45, "reactive_avoidance.safe_distance_m": 0.45,
              "reactive_avoidance.critical_distance_m": 0.30, "path_picker.unknown_allowance_m": 0.6}
WALL = dict(boxes=[(1.5, 0.0, 0.05, 1.0, 0.12)])
# name -> (xml kwargs, open-loop cmd or None for autonomy, host params)
SCENES = {
    "walk": (dict(), (0.06, 0, 0), {}),
    "turn": (dict(), (0, 0, 0.4), {}),
    "strafe": (dict(), (0, 0.05, 0), {}),
    "slope15": (dict(slope_deg=15), (0.06, 0, 0), {}),
    "step3": (dict(boxes=[(0.6, 0.0, 0.15, 1.0, 0.03)]), (0.06, 0, 0), {}),
    "step4": (dict(boxes=[(0.6, 0.0, 0.15, 1.0, 0.04)]), (0.06, 0, 0), {}),
    "asteroid": (dict(gravity=9.81 * 0.015), (0.06, 0, 0), {}),
    "autonomy": (dict(), None, {}),
    "autonomy-fixed": (dict(), None, WORKAROUND),
    "wall": (WALL, None, WORKAROUND),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", choices=sorted(SCENES))
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    xml_kw, cmd, params = SCENES[a.scene]
    sim = Sim(build_xml(**xml_kw), depth_wh=(320, 200))
    host = Host("gait" if cmd else "all", params=params)
    viewer = None
    if not a.dry_run:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(sim.m, sim.d)
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 1.6, -20, 135
    t_wall = time.time()

    def on_tick(t, sim, host, st):
        if viewer is None:
            return
        if not viewer.is_running():
            raise KeyboardInterrupt
        viewer.cam.lookat[:] = sim.d.qpos[:3]          # follow the robot
        viewer.sync()
        if a.speed > 0:
            lag = t / a.speed - (time.time() - t_wall)
            if lag > 0:
                time.sleep(lag)
        if int(round(t / CTRL_DT)) % 100 == 0:
            x, y, z, yaw, p, r = sim.pose()
            print(f"t={t:5.1f}s  x={x:+.2f} y={y:+.2f} z={z:.3f}  state={st.get('rover')}/{st.get('gait')}", flush=True)

    try:
        tr = run_loop(sim, host, a.seconds, script=openloop_script(cmd=cmd) if cmd else None,
                      depth_hz=0.0 if cmd else 10.0, on_tick=on_tick, stop_if_fell=False)
        x, y, z, *_ = sim.pose()
        print(f"finished: x={x:+.2f} y={y:+.2f} z={z:.3f} after {tr.t[-1]:.0f} s")
    except KeyboardInterrupt:
        pass
    finally:
        host.close()
        if viewer is not None:
            viewer.close()


if __name__ == "__main__":
    main()
