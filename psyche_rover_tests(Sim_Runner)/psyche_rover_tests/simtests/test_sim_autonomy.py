"""Closed-loop co-simulation: all six REAL nodes (via the shim) <-> MuJoCo with a ray-cast
depth camera (320x200, 50 deg vertical FOV, like mujoco_sim_node.py).

SHIPPED  = parameters exactly as in the repo's params.yaml / code defaults.
WORKAROUND = four parameter changes that stop the empty-floor lock-up (see the XFAIL tests):
  obstacle_detector.obstacle_trigger_m 0.45   reactive_avoidance.safe_distance_m 0.45
  reactive_avoidance.critical_distance_m 0.30 path_picker.unknown_allowance_m 0.6
"""
import collections
import math

import numpy as np

import mujoco
from harness import chk, info, require, run, test, xfail, slow
from sim_env import Host, Sim, build_xml, fell, max_tilt_deg, run_loop

WORKAROUND = {
    "obstacle_detector.obstacle_trigger_m": 0.45,
    "reactive_avoidance.safe_distance_m": 0.45,
    "reactive_avoidance.critical_distance_m": 0.30,
    "path_picker.unknown_allowance_m": 0.6,
}
WALL_1_5 = dict(boxes=[(1.5, 0.0, 0.05, 1.0, 0.12)])           # 12 cm tall, 2 m wide, face at x=1.45
SHIPPED_WORLD = dict(obstacles=[(1.0, 0.0, 0.03), (1.8, -0.4, 0.08), (2.5, 0.35, 0.12)])


def auto(secs, params=None, xml=None, depth_fn=None, on_tick=None, spawn=None, nodes="all", depth_hz=10.0):
    sim = Sim(build_xml(**(xml or {})), depth_wh=(320, 200))
    if spawn:
        orig = sim.spawn
        sim.spawn = lambda q, **kw: orig(q, **{**spawn, **kw})
    host = Host(nodes, params=params or {})
    try:
        fn = depth_fn(sim) if depth_fn else None
        tr = run_loop(sim, host, secs, depth_hz=depth_hz, depth_fn=fn, on_tick=on_tick)
    finally:
        host.close()
    return sim, tr


def path_length(tr, t0=0.0):
    p = tr.pos[tr.idx(t0):, :2]
    return float(np.sum(np.hypot(*np.diff(p, axis=0).T))) if len(p) > 1 else 0.0


def net_displacement(tr, t0=0.0):
    p = tr.pos[tr.idx(t0):, :2]
    return float(np.hypot(*(p[-1] - p[0]))) if len(p) > 1 else 0.0


def body_hits(tr):
    c = collections.Counter(g for h in tr.hits for g in h)
    return sum(c.values()), dict(c)


def first_time(tr, field, value):
    for t, v in zip(tr.t, getattr(tr, field)):
        if v == value:
            return float(t)
    return None


def speed_at(tr, t, w=0.5):
    i0, i1 = tr.idx(t - w), tr.idx(t)
    if i1 - i0 < 2:
        return 0.0
    return float(np.hypot(*(tr.pos[i1, :2] - tr.pos[i0, :2])) / (tr.t[i1] - tr.t[i0]))


# --------------------------------------------------------------- headline ---
@xfail("with params.yaml as shipped the rover never moves on an empty flat floor: obstacle_detector reads the "
       "floor as an obstacle (/obstacle_detected permanently true) and path_picker finds no path "
       "(unknown near-field > 0.3 m). Reproduced here in physics, not just message flow")
def shipped_parameters_make_progress_on_empty_floor():
    sim, tr = auto(60.0)
    c = collections.Counter(tr.rover)
    info(f"60 s on an empty floor: moved {net_displacement(tr):.2f} m; states {dict(c)}")
    chk(net_displacement(tr) > 1.0, f"rover moved only {net_displacement(tr):.2f} m in 60 s")


@test
def workaround_parameters_walk_without_falling():
    sim, tr = auto(60.0, WORKAROUND)
    c = collections.Counter(tr.rover)
    info(f"path length {path_length(tr):.2f} m, net {net_displacement(tr):.2f} m, "
         f"{c['TRAVERSE_PATH']} of {len(tr.t)} ticks in TRAVERSE_PATH, tilt {max_tilt_deg(tr, 5, 60):.1f} deg")
    chk(not fell(sim), "fell over")
    chk(path_length(tr) > 2.5, f"only {path_length(tr):.2f} m walked")
    chk(max_tilt_deg(tr, 5, 60) < 3.0, "body tilt while walking")
    chk(tr.err[-1] == 0 and tr.warn[-1] == 0, f"log warnings/errors: {tr.warn[-1]}/{tr.err[-1]}")


@test
def startup_reaches_walking_quickly():
    sim, tr = auto(20.0, WORKAROUND)
    t = first_time(tr, "gait", "WALKING")
    info(f"first WALKING at {t} s")
    chk(t is not None and t < 8.0, f"walking after {t} s")
    chk(first_time(tr, "gait", "STANDING") is not None and first_time(tr, "gait", "STANDING") < 4.0, "stand-up too slow")


@xfail("planned heading jitters frame to frame (about +25 / 0 / -15 deg) because the 10 cm costmap has sparse "
       "unknown stripes and a camera wedge: on an empty floor the rover meanders (net displacement a third of "
       "path length) instead of walking straight")
def empty_floor_path_is_straight():
    sim, tr = auto(60.0, WORKAROUND)
    p, n = path_length(tr, 8.0), net_displacement(tr, 8.0)
    info(f"path {p:.2f} m, net {n:.2f} m, straightness {n / max(p, 1e-9):.2f}")
    chk(n / max(p, 1e-9) > 0.8, f"straightness {n / max(p, 1e-9):.2f}")


# ---------------------------------------------------------------- obstacles ---
@test
def never_touches_a_tall_wall_with_good_depth():
    sim, tr = auto(90.0, WORKAROUND, xml=WALL_1_5)
    hits, names = body_hits(tr)
    info(f"wall face at x=1.45: rover reached x={tr.pos[:, 0].max():.2f}, contacts with obstacle: {hits}")
    chk(not fell(sim), "fell")
    chk(hits == 0, f"touched the wall: {names}")
    chk(tr.pos[:, 0].max() < 1.25, "got within one leg-length of the wall")


@test
def tall_box_in_path_is_not_hit_and_rover_stays_upright():
    sim, tr = auto(90.0, WORKAROUND, xml=dict(obstacles=[(1.0, 0.0, 0.12)]))
    hits, names = body_hits(tr)
    chk(not fell(sim) and hits == 0, f"hits {names}")


@xfail("rover approaches an obstacle to ~0.5 m and then stalls forever in the map/evaluate/pick loop: the picker "
       "cannot see free space beside it (camera wedge + unknown = blocked), so it never routes around")
def obstacle_course_is_completed():
    sim, tr = auto(240.0, WORKAROUND, xml=SHIPPED_WORLD)
    info(f"shipped 3-obstacle world: furthest x = {tr.pos[:, 0].max():.2f} m after 240 s (course ends ~3.0 m)")
    chk(tr.pos[:, 0].max() > 3.2, f"stopped at x={tr.pos[:, 0].max():.2f} m")


@xfail("a 3 cm box is steppable (gait test passes it) and the mapper calls it free, but obstacle_detector sees "
       "its top surface at ~0.43 m (inside the trigger) and the rover halts in front of it")
def low_steppable_box_is_walked_over():
    sim, tr = auto(90.0, WORKAROUND, xml=dict(obstacles=[(1.0, 0.0, 0.03)]))
    info(f"3 cm box at x=1.0: furthest x = {tr.pos[:, 0].max():.2f} m")
    chk(tr.pos[:, 0].max() > 1.4, f"stopped at x={tr.pos[:, 0].max():.2f} m")


# ------------------------------------------------------------- sensor faults ---
@test
def depth_stream_loss_stops_the_robot_quickly():
    sim, tr = auto(50.0, WORKAROUND, depth_fn=lambda s: (lambda img, t: None if t > 30 else img))
    v_before, v_after = speed_at(tr, 29.5), speed_at(tr, 33.0)
    info(f"speed before loss {v_before:.3f} m/s, 3 s after {v_after:.3f} m/s")
    chk(v_before > 0.04, "was not walking before the outage")
    chk(v_after < 0.005, f"still moving at {v_after:.3f} m/s 3 s after sensor loss")


@test
def external_estop_stops_the_robot():
    def on_tick(t, sim, host, st):
        if abs(t - 30.0) < 1e-6:
            host.pub_bool("/nav_enabled", False)
    sim, tr = auto(45.0, WORKAROUND, on_tick=on_tick)
    v_before, v_after = speed_at(tr, 29.5), speed_at(tr, 33.5)
    info(f"speed before e-stop {v_before:.3f} m/s, 3.5 s after {v_after:.3f} m/s")
    chk(v_before > 0.04, "was not walking before the e-stop")
    chk(v_after < 0.005, f"still moving at {v_after:.3f} m/s")


@test
def dropouts_and_noise_do_not_cause_falls_or_collisions():
    def depth_fn(sim):
        rng = np.random.default_rng(5)

        def f(img, t):
            img = img.astype(np.int32)
            img[rng.random(img.shape) < 0.10] = 0
            img = np.where(img > 0, img + rng.normal(0, 10, img.shape).astype(np.int32), 0)
            return np.clip(img, 0, 65535).astype(np.uint16)
        return f
    sim, tr = auto(60.0, WORKAROUND, xml=WALL_1_5, depth_fn=depth_fn)
    hits, names = body_hits(tr)
    info(f"10% dropout + 1 cm noise with a wall ahead: path {path_length(tr):.2f} m, hits {hits}")
    chk(not fell(sim) and hits == 0, f"fell or hit: {names}")


@test
def shadow_patches_do_not_cause_collisions():
    def depth_fn(sim):
        rng = np.random.default_rng(2)

        def f(img, t):
            img = img.copy()
            for _ in range(12):                       # ~30% of the frame lost to 'shadow' patches
                r, c = rng.integers(0, 170), rng.integers(0, 290)
                img[r:r + 30, c:c + 30] = 0
            return img
        return f
    sim, tr = auto(90.0, WORKAROUND, xml=WALL_1_5, depth_fn=depth_fn)
    hits, names = body_hits(tr)
    info(f"30% patchy depth loss with a wall ahead: rover x_max {tr.pos[:, 0].max():.2f}, hits {hits}")
    chk(not fell(sim) and hits == 0, f"fell or hit: {names}")


@xfail("an obstacle that returns NO depth (black, specular, transparent) is invisible: the detector skips "
       "zero pixels and the state machine ignores path_found/unknown space while in TRAVERSE_PATH, so the "
       "rover walks straight into it and keeps pushing")
def wall_with_no_depth_returns_is_not_driven_into():
    def depth_fn(sim):
        names = [mujoco.mj_id2name(sim.m, mujoco.mjtObj.mjOBJ_GEOM, i) or "" for i in range(sim.m.ngeom)]
        wall = [i for i, n in enumerate(names) if n.startswith("box_")]

        def f(img, t):
            out = img.copy()
            out[np.isin(sim._gid.reshape(img.shape), wall)] = 0
            return out
        return f
    sim, tr = auto(90.0, WORKAROUND, xml=WALL_1_5, depth_fn=depth_fn)
    hits, names = body_hits(tr)
    info(f"reflective wall at x=1.45: rover reached x={tr.pos[:, 0].max():.2f}, obstacle contact ticks: {hits}")
    chk(hits == 0, f"collided ({hits} contact ticks): {names}")


@xfail("no outlier rejection: 0.2% of pixels (about 1 in 500) returning 0.3 m - stereo speckle, specular glints, "
       "dust - keep /obstacle_detected high forever and the rover never takes a step")
def depth_speckle_does_not_freeze_the_rover():
    def depth_fn(sim):
        rng = np.random.default_rng(1)

        def f(img, t):
            img = img.copy()
            img[rng.random(img.shape) < 0.002] = 300
            return img
        return f
    sim, tr = auto(60.0, WORKAROUND, depth_fn=depth_fn)
    info(f"with speckle: moved {net_displacement(tr):.2f} m, TRAVERSE ticks {sum(1 for r in tr.rover if r == 'TRAVERSE_PATH')}")
    chk(net_displacement(tr) > 1.0, f"rover moved only {net_displacement(tr):.2f} m")


# -------------------------------------------------------- environment faults ---
@test
def slope_and_low_friction_do_not_topple_the_autonomous_stack():
    for name, xml in (("5 deg slope", dict(slope_deg=5)), ("mu 0.3", dict(mu=0.3)), ("soft ground", dict(floor_solref="0.05 0.5"))):
        sim, tr = auto(40.0, WORKAROUND, xml=xml)
        info(f"{name}: path {path_length(tr):.2f} m, tilt {max_tilt_deg(tr, 5, 40):.1f} deg")
        chk(not fell(sim), f"{name}: fell")


@xfail("5 deg uphill slope: terrain_mapper (flat-plane reference) turns the ramp into a wall, so the autonomous "
       "stack stops although the open-loop gait climbs 5 degrees at 80% speed")
def autonomous_stack_climbs_a_5_degree_ramp():
    sim, tr = auto(60.0, WORKAROUND, xml=dict(slope_deg=5))
    info(f"5 deg ramp: path {path_length(tr):.2f} m, net {net_displacement(tr):.2f} m")
    chk(net_displacement(tr) > 1.5, f"net displacement {net_displacement(tr):.2f} m")


@test
def low_gravity_closed_loop_stays_upright():
    for name, g in (("1/6 g", 9.81 / 6), ("1.5% g", 9.81 * 0.015)):
        sim, tr = auto(40.0, WORKAROUND, xml=dict(gravity=g))
        info(f"{name}: path {path_length(tr):.2f} m, peak height {tr.pos[:, 2].max():.3f} m, tilt {max_tilt_deg(tr, 5, 40):.1f} deg")
        chk(not fell(sim) and tr.pos[:, 2].max() < 0.2, f"{name}: fell or launched")


# ------------------------------------------------------------- repeatability ---
def _one_run(k, secs=40.0):
    """Randomised approach to the 2 m wall: start heading +/-0.5 rad, 3% depth dropouts (seeded)."""
    rng = np.random.default_rng(100 + k)

    def depth_fn(sim):
        def f(img, t):
            out = img.copy()
            out[rng.random(img.shape) < 0.03] = 0
            return out
        return f
    yaw = float(rng.uniform(-0.5, 0.5))
    sim, tr = auto(secs, WORKAROUND, xml=WALL_1_5, depth_fn=depth_fn, spawn=dict(yaw=yaw))
    hits, names = body_hits(tr)
    return sim, tr, yaw, hits, names


def _repeat(n, strict_contact):
    safe, lens, contacts = 0, [], 0
    for k in range(n):
        sim, tr, yaw, hits, names = _one_run(k)
        lens.append(path_length(tr))
        contacts += hits > 0
        ok = (not fell(sim)) and path_length(tr) > 0.5          # wall at 1.5 m caps the path at ~1-2 m
        chk(ok, f"run {k} (start yaw {math.degrees(yaw):+.0f} deg): fell={fell(sim)} path={path_length(tr):.2f}")
        if strict_contact:
            chk(hits == 0, f"run {k} (start yaw {math.degrees(yaw):+.0f} deg): touched the wall ({hits} ticks, {names})")
        safe += ok and hits == 0
    info(f"{safe}/{n} runs without fall AND without wall contact; {contacts}/{n} touched the wall; "
         f"path lengths {min(lens):.1f}-{max(lens):.1f} m")


@xfail("angled approach to a wall clips it: only the centre sector (+/-0.13 m at 1 m) is checked, so a leg "
       "(footprint +/-0.30 m) touches the wall before the rover reacts. Deterministic reproduction of one of the "
       "~15% of randomised runs that do this")
def angled_wall_approach_does_not_touch_the_wall():
    sim, tr, yaw, hits, names = _one_run(12)
    info(f"start yaw {math.degrees(yaw):+.0f} deg: {hits} contact ticks with the wall ({names})")
    chk(not fell(sim), "fell")
    chk(hits == 0, f"touched the wall for {hits} ticks")


@test
def repeatability_closed_loop_4_runs_never_fall():
    _repeat(4, strict_contact=False)


@xfail("randomised wall approaches (start heading +/-0.5 rad, 3% dropouts): about 3 in 20 runs touch the wall", slow=True)
def repeatability_closed_loop_20_runs():
    _repeat(20, strict_contact=True)


if __name__ == "__main__":
    raise SystemExit(run("sim_autonomy"))
