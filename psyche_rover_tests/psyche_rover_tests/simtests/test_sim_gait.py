"""Open-loop MuJoCo tests: the REAL gait_controller node (via the shim) drives the simulated
hexapod. /cmd_vel is streamed by the test, so these isolate legs, terrain and faults from
perception. Thresholds come from measured behaviour of the shipped parameters with margin."""
import math

import numpy as np

from harness import chk, info, near, require, run, test, xfail, slow
from sim_env import (Host, Sim, build_xml, contact_stats, fell, forward_progress, max_tilt_deg,
                     openloop_script, run_loop)

WALK = 4.0     # seconds: stand-up (2 s) + settle before walking is measured


def sim_run(cmd=(0.06, 0.0, 0.0), secs=30.0, xml=None, sim_kw=None, setup=None, on_tick=None,
            spawn=None, script=None, stop_if_fell=True):
    sim = Sim(build_xml(**(xml or {})), **(sim_kw or {}))
    if setup:
        setup(sim)
    if spawn:
        orig = sim.spawn
        sim.spawn = lambda q, **kw: orig(q, **{**spawn, **kw})
    host = Host("gait")
    try:
        tr = run_loop(sim, host, secs, script=script or openloop_script(cmd=cmd), on_tick=on_tick,
                      stop_if_fell=stop_if_fell)
    finally:
        host.close()
    return sim, tr


def ratio(tr, cmd_speed, t0=WALK):
    f, l, y = forward_progress(tr, t0, tr.t[-1])
    return f / (cmd_speed * (tr.t[-1] - t0)), f, l, y


# ------------------------------------------------------------ posture ---
@test
def stand_up_reaches_height_level_and_all_feet_down():
    sim, tr = sim_run(cmd=(0, 0, 0), secs=6.0)
    x, y, z, yaw, pitch, roll = sim.pose()
    info(f"standing z={z:.3f} m, tilt={max_tilt_deg(tr, 3.5, 6.0):.2f} deg")
    chk(0.115 <= z <= 0.130, f"standing height {z:.3f} m not near 0.12")
    chk(max_tilt_deg(tr, 3.5, 6.0) < 2.0, "not level after standing")
    chk(contact_stats(tr, 3.5, 6.0, min_feet=6)[0] > 0.99, "not all six feet on the ground when standing")


@test
def sit_down_is_stable_and_low():
    def script(t, host):
        if abs(t - 0.02) < 1e-9:
            host.pub_bool("/stand_request", True)
        if abs(t - 6.0) < 1e-9:
            host.pub_bool("/stand_request", False)
    sim, tr = sim_run(secs=12.0, script=script)
    x, y, z, *_ = sim.pose()
    info(f"seated z={z:.3f} m")
    chk(z < 0.05, f"body still {z:.3f} m high after sit-down")
    chk(max_tilt_deg(tr, 7, 12) < 1.0, "tilted after sitting")
    chk(tr.gait[-1] == "SITTING", f"gait state {tr.gait[-1]}")


@test
def nav_disabled_robot_stays_put_despite_cmd_spam():
    def script(t, host):
        if abs(t - 0.02) < 1e-9:
            host.pub_bool("/stand_request", True)
        if t > 3.0 and int(round(t / 0.02)) % 5 == 0:
            host.pub_twist(0.08, 0.08, 0.5)
    sim, tr = sim_run(secs=20.0, script=script)
    moved = float(np.hypot(*(tr.pos[-1, :2] - tr.pos[tr.idx(4.0), :2])))
    chk(moved < 0.01, f"moved {moved * 100:.1f} cm with nav disabled")


# ---------------------------------------------------------- locomotion ---
@test
def walk_forward_distance_and_drift():
    sim, tr = sim_run((0.06, 0, 0), 34.0)
    r, f, l, y = ratio(tr, 0.06)
    info(f"{f:.3f} m of {0.06 * 30:.2f} m commanded ({100 * r:.0f}%), lateral {l * 100:+.1f} cm, yaw {math.degrees(y):+.1f} deg")
    chk(0.85 <= r <= 1.05, f"distance ratio {r:.2f}")
    chk(abs(l) < 0.05, f"lateral drift {l:.3f} m")
    chk(abs(math.degrees(y)) < 5.0, f"yaw drift {math.degrees(y):.1f} deg")


@test
def walk_reverse_distance_and_drift():
    sim, tr = sim_run((-0.06, 0, 0), 34.0)
    r, f, l, y = ratio(tr, -0.06)
    info(f"reverse: {100 * r:.0f}% of command, lateral {l * 100:+.1f} cm")
    chk(0.85 <= r <= 1.05, f"reverse ratio {r:.2f}")
    chk(abs(l) < 0.05 and abs(math.degrees(y)) < 5.0, "reverse drifts")


@test
def speed_is_linear_in_command():
    for v in (0.02, 0.04, 0.06, 0.08):
        sim, tr = sim_run((v, 0, 0), 24.0)
        r, f, l, y = ratio(tr, v)
        info(f"cmd {v:.2f} m/s -> {100 * r:.0f}%")
        chk(0.80 <= r <= 1.08, f"cmd {v}: ratio {r:.2f}")


@test
def turn_in_place_rate_matches_command():
    sim, tr = sim_run((0, 0, 0.4), 30.0)
    f, l, y = forward_progress(tr, WALK, tr.t[-1])
    rate = y / (tr.t[-1] - WALK)
    drift = float(np.hypot(*(tr.pos[-1, :2] - tr.pos[tr.idx(WALK), :2])))
    info(f"turn rate {rate:.3f} rad/s (cmd 0.40) = yaw gain {rate / 0.4:.2f} (forward gain is ~0.93); "
         f"position drift {drift * 100:.1f} cm")
    chk(0.28 <= rate <= 0.44, f"turn rate {rate:.3f}")
    chk(drift < 0.10, f"position drift while turning {drift:.3f} m")
    sim, tr = sim_run((0, 0, -0.4), 30.0)
    f, l, y = forward_progress(tr, WALK, tr.t[-1])
    chk(y < 0 and abs(y) / (tr.t[-1] - WALK) > 0.28, "right turn wrong sign/rate")


@test
def strafe_matches_command_both_ways():
    for v in (0.05, -0.05):
        sim, tr = sim_run((0, v, 0), 30.0)
        f, l, y = forward_progress(tr, WALK, tr.t[-1])
        r = l / (v * (tr.t[-1] - WALK))
        info(f"strafe cmd {v:+.2f}: {100 * r:.0f}%, forward drift {f * 100:+.1f} cm, yaw {math.degrees(y):+.1f} deg")
        chk(0.85 <= r <= 1.08, f"strafe {v}: ratio {r:.2f}")
        chk(abs(f) < 0.08 and abs(math.degrees(y)) < 5.0, "strafe drifts")


@test
def arc_motion_follows_commanded_curvature():
    v, w = 0.05, 0.25
    sim, tr = sim_run((v, 0, w), 40.0)
    f, l, y = forward_progress(tr, WALK, tr.t[-1])
    rate = y / (tr.t[-1] - WALK)
    info(f"arc yaw rate {rate:.3f} rad/s (cmd {w})")
    chk(0.17 <= rate <= 0.28, f"arc yaw rate {rate:.3f}")
    # radius = v/w = 0.2 m: robot stays near a circle -> far from a straight line
    span = float(np.ptp(tr.pos[tr.idx(WALK):, 0]))
    info(f"arc radius target 0.20 m, x-span {span:.2f} m")
    chk(span < 0.6, f"arc is not tight (x span {span:.2f} m)")


@test
def body_stays_level_across_all_gaits():
    for name, cmd in (("forward", (0.08, 0, 0)), ("turn", (0, 0, 0.5)), ("strafe", (0, 0.08, 0)), ("arc", (0.05, 0, 0.3))):
        sim, tr = sim_run(cmd, 24.0)
        tilt = max_tilt_deg(tr, WALK, tr.t[-1])
        zr = float(np.ptp(tr.pos[tr.idx(WALK):, 2]))
        info(f"{name}: max tilt {tilt:.2f} deg, height ripple {zr * 1000:.1f} mm")
        chk(tilt < 3.0, f"{name}: tilt {tilt:.1f} deg")
        chk(zr < 0.012, f"{name}: height ripple {zr * 1000:.0f} mm")


@test
def never_fewer_than_three_feet_down_nominal_ground():
    for name, cmd in (("forward", (0.08, 0, 0)), ("turn", (0, 0, 0.5)), ("strafe", (0, 0.08, 0))):
        sim, tr = sim_run(cmd, 24.0)
        frac, lo = contact_stats(tr, WALK, tr.t[-1], 3)
        chk(lo >= 3, f"{name}: only {lo} feet down at worst")


# ------------------------------------------------------------- terrain ---
@test
def slope_climbing_table():
    floors = {5: 0.65, 10: 0.50, 15: 0.35, 20: 0.20}
    for deg, floor in floors.items():
        sim, tr = sim_run((0.06, 0, 0), 30.0, xml=dict(slope_deg=deg))
        r, f, l, y = ratio(tr, 0.06)
        info(f"{deg:2d} deg uphill: {100 * r:3.0f}% of commanded speed, max tilt {max_tilt_deg(tr, WALK, 30):.1f} deg")
        chk(not fell(sim), f"{deg} deg: fell over")
        chk(r >= floor, f"{deg} deg: progress {100 * r:.0f}% < {100 * floor:.0f}%")
        chk(abs(l) < 0.05, f"{deg} deg: lateral drift {l:.2f} m")


@test
def steep_slope_stalls_without_tipping():
    sim, tr = sim_run((0.06, 0, 0), 30.0, xml=dict(slope_deg=25))
    r, *_ = ratio(tr, 0.06)
    info(f"25 deg uphill: {100 * r:.0f}% progress (limit of climbing ability)")
    chk(not fell(sim), "tipped over on a 25 degree slope")
    chk(r < 0.5, "unexpectedly easy: has the model changed?")


@test
def downhill_slope_is_controlled():
    sim, tr = sim_run((0.06, 0, 0), 30.0, xml=dict(slope_deg=-15))
    r, f, l, y = ratio(tr, 0.06)
    info(f"15 deg downhill: {100 * r:.0f}% of commanded speed")
    chk(not fell(sim), "fell on a downhill slope")
    chk(r < 1.5, f"ran away downhill ({100 * r:.0f}% of commanded speed)")


@xfail("open-loop gait has no heading/lateral correction: on a 10 deg side slope the robot slides "
       "downhill ~0.4 m in 26 s (27% of its forward path); nothing in the stack uses the IMU")
def side_slope_drift_is_small():
    sim, tr = sim_run((0.06, 0, 0), 30.0, xml=dict(side_slope_deg=10))
    r, f, l, y = ratio(tr, 0.06)
    info(f"10 deg side slope: forward {f:.2f} m, sideways slide {l:+.2f} m")
    chk(abs(l) < 0.10 * abs(f), f"slid {abs(l):.2f} m sideways over {f:.2f} m forward")


@test
def side_slope_does_not_tip_up_to_20_degrees():
    for deg in (10, 20):
        sim, tr = sim_run((0.06, 0, 0), 30.0, xml=dict(side_slope_deg=deg))
        chk(not fell(sim), f"{deg} deg side slope: tipped over")
        chk(max_tilt_deg(tr, WALK, 30) < deg + 3, f"{deg} deg side slope: body tilt beyond the slope")


@test
def friction_sweep():
    table = {1.2: 0.85, 0.8: 0.75, 0.5: 0.65, 0.3: 0.55, 0.15: 0.42}
    for mu, floor in table.items():
        sim, tr = sim_run((0.06, 0, 0), 30.0, xml=dict(mu=mu))
        r, f, l, y = ratio(tr, 0.06)
        frac, lo = contact_stats(tr, WALK, 30, 3)
        info(f"mu={mu:4.2f}: {100 * r:3.0f}% progress, >=3 feet {100 * frac:.0f}% of the time (worst {lo})")
        chk(not fell(sim), f"mu {mu}: fell")
        chk(r >= floor, f"mu {mu}: progress {100 * r:.0f}% < {100 * floor:.0f}%")
        chk(abs(math.degrees(y)) < 10, f"mu {mu}: yaw drift {math.degrees(y):.0f} deg")


@test
def soft_ground_still_walks():
    for sr in ("0.05 0.5", "0.1 0.3"):
        sim, tr = sim_run((0.06, 0, 0), 30.0, xml=dict(floor_solref=sr))
        r, *_ = ratio(tr, 0.06)
        info(f"soft contact solref={sr}: {100 * r:.0f}%, z={tr.pos[-1, 2]:.3f}")
        chk(not fell(sim) and r > 0.8, f"solref {sr}: ratio {r:.2f}")
        chk(tr.pos[-1, 2] > 0.10, "body sank into soft ground")


@test
def lunar_gravity_walks_without_launching():
    sim, tr = sim_run((0.06, 0, 0), 30.0, xml=dict(gravity=9.81 / 6))
    r, *_ = ratio(tr, 0.06)
    zmax = float(tr.pos[:, 2].max())
    info(f"1/6 g: {100 * r:.0f}% progress, peak body height {zmax:.3f} m")
    chk(not fell(sim) and r > 0.7, f"ratio {r:.2f}")
    chk(zmax < 0.16, f"body bounced to {zmax:.3f} m")
    chk(max_tilt_deg(tr, WALK, 30) < 5, "tilting in low gravity")


@test
def asteroid_gravity_stays_attached_and_upright():
    sim, tr = sim_run((0.06, 0, 0), 30.0, xml=dict(gravity=9.81 * 0.015))
    r, *_ = ratio(tr, 0.06)
    zmax = float(tr.pos[:, 2].max())
    frac, _ = contact_stats(tr, WALK, 30, 3)
    n = tr.contacts[tr.idx(WALK):].sum(axis=1)
    info(f"1.5% g: progress {100 * r:.0f}%, peak z {zmax:.3f} m, >=3 feet {100 * frac:.0f}% of the time, "
         f"airborne (0 feet) {100 * float(np.mean(n == 0)):.0f}% of the time")
    chk(not fell(sim), "tumbled")
    chk(zmax < 0.20, f"launched to {zmax:.2f} m")
    chk(max_tilt_deg(tr, WALK, 30) < 10, "tilting")


@xfail("at 1.5% g (Psyche-like) the position-controlled tripod gait has no real traction: the robot "
       "hops with fewer than 3 feet down most of the time, so motion is momentum, not controlled stepping")
def asteroid_gravity_keeps_three_feet_down():
    sim, tr = sim_run((0.06, 0, 0), 30.0, xml=dict(gravity=9.81 * 0.015))
    frac, lo = contact_stats(tr, WALK, 30, 3)
    chk(frac > 0.9, f">=3 feet down only {100 * frac:.0f}% of the time")


@test
def cold_servo_torque_derating():
    for ts, floor in ((0.5, 0.85), (0.25, 0.85)):
        sim, tr = sim_run((0.06, 0, 0), 30.0, xml=dict(torque_scale=ts))
        r, *_ = ratio(tr, 0.06)
        info(f"servo torque x{ts}: {100 * r:.0f}% progress, z={tr.pos[-1, 2]:.3f}")
        chk(not fell(sim) and r >= floor, f"torque x{ts}: ratio {r:.2f}")
    sim, tr = sim_run((0.06, 0, 0), 12.0, xml=dict(torque_scale=0.08), stop_if_fell=False)
    info(f"servo torque x0.08 (0.24 N*m): body height {tr.pos[-1, 2]:.3f} m -> robot collapses (limit is between x0.08 and x0.15)")
    chk(tr.pos[-1, 2] < 0.06, "expected collapse below ~x0.1 torque: model changed?")


@test
def servo_command_latency_tolerance():
    for lat in (0.1, 0.2, 0.4):
        sim, tr = sim_run((0.06, 0, 0), 30.0, sim_kw=dict(latency_s=lat))
        r, f, l, y = ratio(tr, 0.06)
        info(f"command latency {lat * 1000:.0f} ms: {100 * r:.0f}% progress")
        chk(not fell(sim) and r > 0.85 and abs(l) < 0.08, f"latency {lat}: ratio {r:.2f}, lateral {l:.2f}")


# ------------------------------------------------------------- obstacles ---
@test
def step_over_3cm_wide_barrier():
    sim, tr = sim_run((0.06, 0, 0), 60.0, xml=dict(boxes=[(0.6, 0.0, 0.15, 1.0, 0.03)]))
    xmax = float(tr.pos[:, 0].max())
    tilt = max_tilt_deg(tr, WALK, tr.t[-1])
    info(f"3 cm barrier (2 m wide): reached x={xmax:.2f} m, max tilt {tilt:.1f} deg")
    chk(xmax > 1.0, f"stopped at x={xmax:.2f} m")
    chk(not fell(sim) and tilt < 10, f"tilt {tilt:.1f} deg crossing")


@xfail("step_height_m is 4 cm (and params.yaml says 'step over ~step_height_m') but a 4 cm full-width "
       "barrier stops the robot dead: effective crossing limit is ~3 cm. The mapper still calls <=5 cm free")
def step_over_4cm_wide_barrier_matches_step_height():
    sim, tr = sim_run((0.06, 0, 0), 60.0, xml=dict(boxes=[(0.6, 0.0, 0.15, 1.0, 0.04)]))
    xmax = float(tr.pos[:, 0].max())
    info(f"4 cm barrier: reached x={xmax:.2f} m (barrier spans 0.45..0.75)")
    chk(xmax > 1.0, f"blocked at x={xmax:.2f} m by a 4 cm barrier")


@test
def tall_barriers_block_without_falling_or_climbing():
    for h in (0.06, 0.08, 0.12):
        sim, tr = sim_run((0.06, 0, 0), 40.0, xml=dict(boxes=[(0.6, 0.0, 0.15, 1.0, h)]))
        xmax = float(tr.pos[:, 0].max())
        info(f"{h * 100:.0f} cm barrier: stopped at x={xmax:.2f} m")
        chk(xmax < 0.45, f"{h * 100:.0f} cm barrier crossed (x={xmax:.2f})")
        chk(not fell(sim) and max_tilt_deg(tr, WALK, tr.t[-1]) < 8, f"{h * 100:.0f} cm barrier: unstable")


@test
def narrow_obstacles_are_straddled_not_hit():
    # the shipped 30 cm boxes: legs span 55 cm, so up to 8 cm boxes pass between the feet
    for h in (0.03, 0.08):
        sim, tr = sim_run((0.06, 0, 0), 60.0, xml=dict(obstacles=[(0.6, 0.0, h)]))
        chk(tr.pos[:, 0].max() > 1.2, f"{h * 100:.0f} cm centred box not passed")
    sim, tr = sim_run((0.06, 0, 0), 40.0, xml=dict(obstacles=[(0.6, 0.0, 0.12)]))
    chk(tr.pos[:, 0].max() < 0.45, "12 cm box (body clearance ~10 cm) should block")


# --------------------------------------------------------------- disturbance ---
def _push(force, axis, t0=8.0, dur=0.2):
    def f(t, sim, host, st):
        sim.d.xfrc_applied[sim.base, :] = 0
        if t0 <= t < t0 + dur:
            sim.d.xfrc_applied[sim.base, axis] = force
    return f


@test
def push_recovery_standing():
    for F, axis, name in ((30, 1, "30 N sideways"), (60, 1, "60 N sideways"), (20, 0, "20 N forward")):
        sim, tr = sim_run((0, 0, 0), 20.0, on_tick=_push(F, axis))
        during = max_tilt_deg(tr, 8, 10)
        after = max_tilt_deg(tr, 13, 20)
        info(f"{name}: peak tilt {during:.1f} deg, tilt after 5 s {after:.2f} deg")
        chk(not fell(sim) and after < 1.0, f"{name}: did not recover")


@test
def push_recovery_while_walking():
    sim, tr = sim_run((0.06, 0, 0), 25.0, on_tick=_push(40, 1, t0=10.0))
    chk(not fell(sim), "fell after a 40 N shove while walking")
    chk(max_tilt_deg(tr, 14, 25) < 3.0, "still tilted 4 s after the shove")
    chk(contact_stats(tr, 12, 25, 3)[1] >= 3, "lost support after the shove")


@test
def recovers_from_dropped_tilted_pose():
    for roll, pitch in ((0.4, 0.0), (0.0, 0.4), (0.8, 0.3)):
        sim, tr = sim_run((0, 0, 0), 15.0, spawn=dict(z=0.2, roll=roll, pitch=pitch), stop_if_fell=False)
        x, y, z, yaw, p, r = sim.pose()
        chk(abs(r) < math.radians(2) and abs(p) < math.radians(2) and z > 0.11,
            f"roll {math.degrees(roll):.0f}/pitch {math.degrees(pitch):.0f}: ended roll {math.degrees(r):.0f} pitch {math.degrees(p):.0f} z {z:.3f}")


@xfail("no self-righting: nothing reads /imu, so a robot that lands on its side or upside down "
       "stays there with the legs flailing (z = 0.02 m)")
def recovers_from_being_on_its_side_or_inverted():
    for roll in (math.radians(110), math.pi):
        sim, tr = sim_run((0, 0, 0), 20.0, spawn=dict(z=0.2, roll=roll), stop_if_fell=False)
        x, y, z, yaw, p, r = sim.pose()
        chk(abs(r) < math.radians(10) and z > 0.10, f"started at roll {math.degrees(roll):.0f}: ended roll {math.degrees(r):.0f}, z {z:.3f}")


# -------------------------------------------------------------- servo faults ---
@test
def limp_leg_does_not_topple_the_robot():
    rows = []
    for leg in range(6):
        sim, tr = sim_run((0.06, 0, 0), 30.0, setup=lambda s, L=leg: s.limp(L), stop_if_fell=False)
        r, f, l, y = ratio(tr, 0.06)
        tilt = max_tilt_deg(tr, WALK, 30)
        rows.append((leg, r, math.degrees(y), tilt, fell(sim)))
        info(f"leg {leg} unpowered: {100 * r:3.0f}% progress, yaw {math.degrees(y):+6.1f} deg, tilt {tilt:4.1f} deg")
        chk(not fell(sim), f"leg {leg} limp: robot fell over")
        chk(tilt < 20, f"leg {leg} limp: tilt {tilt:.0f} deg")
        chk(sim.finite(), f"leg {leg} limp: non-finite state")


@test
def stuck_servo_does_not_topple_the_robot():
    cases = (("leg1 femur stuck up", 1, 1, 0.5), ("leg2 tibia stuck folded", 2, 2, -2.0), ("leg4 coxa stuck out", 4, 0, 0.4))
    for name, leg, joint, ang in cases:
        sim, tr = sim_run((0.06, 0, 0), 30.0, setup=lambda s, a=(leg, joint, ang): s.stick(*a), stop_if_fell=False)
        r, f, l, y = ratio(tr, 0.06)
        info(f"{name}: {100 * r:3.0f}% progress, yaw {math.degrees(y):+6.1f} deg, tilt {max_tilt_deg(tr, WALK, 30):.1f} deg")
        chk(not fell(sim), f"{name}: fell over")


@xfail("a dead servo goes unnoticed: nothing consumes /joint_states, so the stack keeps commanding "
       "forward motion (here 63% progress with a spinning/ drifting body) with no warning, nav stop or state change")
def servo_failure_is_detected_and_reported():
    sim = Sim(build_xml())
    host = Host("gait")
    sim.limp(3)   # mid-left leg
    tr = run_loop(sim, host, 30.0, script=openloop_script(cmd=(0.06, 0, 0)))
    st = host.status
    host.close()
    info(f"after 30 s with a dead servo: warnings={st['warn']} errors={st['err']} nav={st['nav']} gait={st['gait']}")
    chk(st["warn"] + st["err"] > 0 or st["nav"] == 0 or st["gait"] != "WALKING", "no fault indication of any kind")


# ----------------------------------------------------- endurance / repeatability ---
@test
def endurance_120s_no_drift_or_degradation():
    sim, tr = sim_run((0.06, 0, 0.0), 124.0)
    _endurance_checks(sim, tr, 0.06, 4.0)


@slow
def endurance_30min():
    sim, tr = sim_run((0.06, 0, 0.0), 1804.0)
    _endurance_checks(sim, tr, 0.06, 4.0)


def _endurance_checks(sim, tr, v, t0):
    n = len(tr.t)
    mid = tr.idx((t0 + tr.t[-1]) / 2)
    f1, l1, y1 = forward_progress(tr, t0, tr.t[mid])
    f2, l2, y2 = forward_progress(tr, tr.t[mid], tr.t[-1])
    h1, h2 = tr.t[mid] - t0, tr.t[-1] - tr.t[mid]
    s1, s2 = f1 / h1, f2 / h2
    eff = np.array(sim.torque_log)
    e1, e2 = eff[tr.idx(t0):mid].mean(), eff[mid:].mean()
    info(f"{tr.t[-1]:.0f} s: speed first half {s1:.4f} m/s, second half {s2:.4f} m/s; "
         f"mean actuator effort {e1:.2f} -> {e2:.2f} N*m; max tilt {max_tilt_deg(tr, t0, tr.t[-1]):.2f} deg; "
         f"net lateral {l1 + l2:+.2f} m, yaw {math.degrees(y1 + y2):+.1f} deg")
    chk(not fell(sim) and sim.finite(), "fell or went non-finite")
    chk(abs(s2 - s1) / s1 < 0.03, "walking speed changed over time")
    chk(abs(e2 - e1) / max(e1, 1e-9) < 0.05, "actuator effort drifted (thermal proxy)")
    chk(max_tilt_deg(tr, t0, tr.t[-1]) < 3.0, "tilt grew")
    chk(abs(l1 + l2) < 0.02 * (f1 + f2) + 0.05, "heading drift grew with time")
    chk(tr.err[-1] == 0 and tr.warn[-1] == 0, f"errors/warnings during the run ({tr.warn[-1]}/{tr.err[-1]})")


@test
def repeatability_across_start_poses():
    rng = np.random.default_rng(42)
    n = 5
    _repeat(n, rng)


@slow
def repeatability_20_runs():
    _repeat(20, np.random.default_rng(7))


def _repeat(n, rng):
    ratios, laterals = [], []
    for k in range(n):
        yaw = float(rng.uniform(-math.pi, math.pi))
        x0, y0 = [float(v) for v in rng.uniform(-0.5, 0.5, 2)]
        sim, tr = sim_run((0.06, 0, 0), 24.0, spawn=dict(yaw=yaw, x=x0, y=y0))
        r, f, l, y = ratio(tr, 0.06)
        ratios.append(r)
        laterals.append(l)
        chk(not fell(sim), f"run {k}: fell")
    ratios = np.array(ratios)
    cv = ratios.std() / ratios.mean()
    info(f"{n} runs, random start pose/heading: distance ratio {ratios.mean():.3f} +/- {ratios.std():.4f} "
         f"(CV {100 * cv:.2f}%), worst lateral drift {max(abs(v) for v in laterals) * 100:.1f} cm")
    chk(cv < 0.03, f"distance varies {100 * cv:.1f}% between identical runs")
    chk(ratios.min() > 0.85, f"worst run only {100 * ratios.min():.0f}%")


if __name__ == "__main__":
    raise SystemExit(run("sim_gait"))
