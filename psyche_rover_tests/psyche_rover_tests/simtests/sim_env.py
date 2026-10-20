"""MuJoCo test environment for the Psyche hexapod.

* build_xml()   - regenerates the MJCF from psyche_hexapod_sim/scripts/generate_models.py
                  and applies scenario variants (slope, friction, soft ground, gravity,
                  torque derating, obstacles, extra geometry).
* Sim           - physics wrapper: spawn, step, pose, foot contacts, ray-cast depth camera
                  (no OpenGL needed), fault injection (limp / stuck servo, command latency).
* Host          - client for the C++ autonomy_host (the real ROS nodes behind the shim).
* run_loop()    - the co-simulation loop (50 Hz control, physics at 500 Hz).
"""
import argparse
import collections
import math
import os
import subprocess
import sys

import numpy as np

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC = os.path.dirname(PKG)
sys.path.insert(0, os.path.join(SRC, "psyche_hexapod_sim", "scripts"))
import generate_models as gm  # noqa: E402

HOST_BIN = os.environ.get("PSYCHE_HOST_BIN", os.path.join(PKG, "build", "autonomy_host"))
CTRL_DT = 0.02          # gait controller period (50 Hz)
PHYS_PER_CTRL = 10      # 10 x 2 ms
SPAWN_Z = 0.065         # same as mujoco_sim_node.py spawn_height_m
LEGS = ["leg_0_front_right", "leg_1_front_left", "leg_2_mid_right",
        "leg_3_mid_left", "leg_4_rear_right", "leg_5_rear_left"]


# ------------------------------------------------------------------ model ---
def _args(**over):
    a = argparse.Namespace(coxa=0.05, femur=0.08, tibia=0.13, hl=0.10, hw=0.06, stand=0.12,
                           cam_h=0.17, fovy=50.0, body_mass=1.2, link_mass=0.03, foot_r=0.008,
                           kp=12.0, kv=0.4, damping=0.05, effort=3.0, vel=6.0,
                           controllers_yaml="x", obstacles=[])
    for k, v in over.items():
        setattr(a, k, v)
    return a


def build_xml(obstacles=(), mu=None, floor_solref=None, slope_deg=0.0, side_slope_deg=0.0,
              gravity=9.81, torque_scale=1.0, boxes=(), **gen_over):
    """obstacles: (x, y, height) 30x30 cm boxes like the shipped world.
    boxes: (x, y, hx, hy, height) arbitrary footprint. slope_deg>0 = uphill in +x."""
    a = _args(obstacles=list(obstacles), **gen_over)
    xml = gm.gen_mjcf(a)
    if mu is not None:
        xml = xml.replace('friction="1.2 0.02 0.001"', f'friction="{mu} 0.02 0.001"')
    floor_old = '<geom name="floor" type="plane" size="30 30 0.1" material="grid"/>'
    assert floor_old in xml
    attrs = ""
    if floor_solref:
        attrs += f' solref="{floor_solref}"'
    if slope_deg or side_slope_deg:
        attrs += f' euler="{math.radians(side_slope_deg):.6f} {-math.radians(slope_deg):.6f} 0"'
    xml = xml.replace(floor_old, floor_old.replace("/>", attrs + "/>"))
    xml = xml.replace('gravity="0 0 -9.81"', f'gravity="0 0 {-gravity}"')
    if torque_scale != 1.0:
        f = a.effort * torque_scale
        xml = xml.replace(f'forcerange="-{a.effort} {a.effort}"', f'forcerange="-{f} {f}"')
    extra = ""
    for k, (x, y, hx, hy, h) in enumerate(boxes):
        extra += (f'    <geom name="box_{k}" type="box" pos="{x} {y} {h / 2}" '
                  f'size="{hx} {hy} {h / 2}" rgba="0.7 0.2 0.2 1"/>\n')
    if extra:
        xml = xml.replace('    <body name="base_link"', extra + '    <body name="base_link"', 1)
    return xml


# -------------------------------------------------------------------- sim ---
class Sim:
    def __init__(self, xml, depth_wh=(160, 100), latency_s=0.0):
        self.m = mujoco.MjModel.from_xml_string(xml)
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.act_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.act_names]
        self.qadr = [int(m.jnt_qposadr[j]) for j in jid]
        self.touch = [int(m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, f"{n}_touch")])
                      for n in LEGS]
        self.cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "oak")
        self.base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.fovy = float(m.cam_fovy[self.cam])
        self.w, self.h = depth_wh
        self._prep_rays()
        self.started = False
        self.stuck = {}                  # actuator index -> fixed target angle
        self.latency_ticks = int(round(latency_s / CTRL_DT))
        self._queue = collections.deque()
        self.torque_log = []
        self.obstacle_boxes = []         # (x, y, hx, hy, h) for collision accounting

    # -- spawn exactly as mujoco_sim_node.py: pose = first command, body at SPAWN_Z
    def spawn(self, q, x=0.0, y=0.0, yaw=0.0, z=SPAWN_Z, roll=0.0, pitch=0.0):
        d = self.d
        for k, a in enumerate(self.qadr):
            d.qpos[a] = q[k]
        d.qpos[0], d.qpos[1], d.qpos[2] = x, y, z
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        d.qpos[3:7] = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                       cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
        mujoco.mj_forward(self.m, d)
        self.started = True

    def limp(self, leg, joint=None):
        """Remove servo power from a leg (all three joints, or one: 0/1/2)."""
        for j in ([0, 1, 2] if joint is None else [joint]):
            i = 3 * leg + j
            self.m.actuator_gainprm[i, 0] = 0.0
            self.m.actuator_biasprm[i, 1] = 0.0
            self.m.actuator_biasprm[i, 2] = 0.0

    def stick(self, leg, joint, angle):
        self.stuck[3 * leg + joint] = angle

    def step(self, q_cmd):
        q = np.array(q_cmd, dtype=float)
        if self.latency_ticks:
            self._queue.append(q)
            q = self._queue.popleft() if len(self._queue) > self.latency_ticks else self._queue[0]
        for i, a in self.stuck.items():
            q[i] = a
        self.d.ctrl[:] = q
        eff = 0.0
        for _ in range(PHYS_PER_CTRL):
            mujoco.mj_step(self.m, self.d)
        eff = float(np.sum(np.abs(self.d.actuator_force)))
        self.torque_log.append(eff)

    # -- telemetry
    def pose(self):
        p = self.d.qpos
        w, x, y, z = p[3:7]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        return p[0], p[1], p[2], yaw, pitch, roll

    def contacts(self, thresh=None):
        # default threshold = 1% of the weight carried by 3 feet, so it scales with gravity
        if thresh is None:
            thresh = 0.01 * float(np.sum(self.m.body_mass)) * abs(float(self.m.opt.gravity[2])) / 3.0
        return [self.d.sensordata[a] > thresh for a in self.touch]

    def obstacle_hits(self, prefixes=("obstacle_", "box_")):
        """Names of robot geoms currently touching any obstacle geom (feet included)."""
        hit = set()
        m, d = self.m, self.d
        for i in range(d.ncon):
            c = d.contact[i]
            n1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1) or ""
            n2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2) or ""
            for a, b in ((n1, n2), (n2, n1)):
                if a.startswith(prefixes):
                    hit.add(b or f"geom{c.geom2 if a == n1 else c.geom1}")
        return hit

    def finite(self):
        return bool(np.all(np.isfinite(self.d.qpos)) and np.all(np.isfinite(self.d.qvel)))

    # -- ray-cast depth camera (z-depth, mm, uint16; 0 = no return)
    def _prep_rays(self):
        w, h = self.w, self.h
        f = (h / 2.0) / math.tan(math.radians(self.fovy) / 2.0)
        self.fx = self.fy = f
        self.cx, self.cy = w / 2.0, h / 2.0
        u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
        self._dc = np.stack([(u - self.cx) / f, -(v - self.cy) / f, -np.ones_like(u)], -1).reshape(-1, 3)
        self._dcn = np.linalg.norm(self._dc, axis=1)
        n = self._dc.shape[0]
        self._gid = np.zeros(n, dtype=np.int32)
        self._dist = np.zeros(n)
        self._nrm = np.zeros(n * 3)

    def depth_mm(self, far=8.0):
        mat = self.d.cam_xmat[self.cam].reshape(3, 3)
        dw = self._dc @ mat.T
        dw = dw / np.linalg.norm(dw, axis=1)[:, None]
        mujoco.mj_multiRay(self.m, self.d, self.d.cam_xpos[self.cam].copy(), dw.ravel(), None, 1,
                           self.base, self._gid, self._dist, self._nrm, dw.shape[0], 1e10)
        z = self._dist / self._dcn
        z[(self._dist < 0) | (z >= far - 0.05)] = 0.0
        return np.clip(np.round(z * 1000.0), 0, 65535).astype(np.uint16).reshape(self.h, self.w)


# ------------------------------------------------------------------- host ---
class Host:
    def __init__(self, nodes="all", params=None, yamls=()):
        if not os.path.exists(HOST_BIN):
            raise RuntimeError(f"autonomy_host not built ({HOST_BIN}); run run_all_tests.py first")
        cmd = [HOST_BIN, "--nodes", nodes]
        for y in yamls:
            cmd += ["--yaml", y]
        for k, v in (params or {}).items():
            cmd += ["--param", f"{k}={str(v).lower() if isinstance(v, bool) else v}"]
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.status = {}
        self.q = None
        self.t_ns = 0

    def _send(self, s):
        self.p.stdin.write(s.encode() + b"\n")

    def info(self, sim):
        self._send(f"INFO {sim.w} {sim.h} {sim.fx} {sim.fy} {sim.cx} {sim.cy}")

    def depth(self, img):
        self._send(f"DEPTH {img.shape[1]} {img.shape[0]}")
        self.p.stdin.write(np.ascontiguousarray(img, dtype="<u2").tobytes())

    def pub_bool(self, topic, v):
        self._send(f"PUBB {topic} {int(v)}")

    def pub_twist(self, vx, vy, wz):
        self._send(f"PUBT {vx} {vy} {wz}")

    def tick(self, t_s):
        self.t_ns = int(round(t_s * 1e9))
        self._send(f"T {self.t_ns}")
        self.p.stdin.flush()
        while True:
            line = self.p.stdout.readline().decode().strip()
            if not line:
                raise RuntimeError("autonomy_host died")
            if line == "OK":
                break
            if line[0] == "S":
                f = line.split()
                self.status = dict(rover=f[1], gait=f[2], nav=int(f[3]), cmd_vx=float(f[4]),
                                   cmd_wz=float(f[5]), obstacle=int(f[6]), path_found=int(f[7]),
                                   heading=float(f[8]), warn=int(f[9]), err=int(f[10]))
            elif line[0] == "J":
                self.q = np.array([float(x) for x in line.split()[1:]])
        return self.status, self.q

    def close(self):
        try:
            self._send("Q")
            self.p.stdin.flush()
        except Exception:
            pass
        self.p.wait(timeout=5)


# ------------------------------------------------------------------ trace ---
class Trace:
    def __init__(self):
        self.t, self.pos, self.rpy, self.contacts, self.rover, self.gait = [], [], [], [], [], []
        self.nav, self.obstacle, self.warn, self.err, self.cmd_vx = [], [], [], [], []
        self.hits = []

    def add(self, t, sim, st):
        x, y, z, yaw, pitch, roll = sim.pose()
        self.t.append(t)
        self.pos.append((x, y, z))
        self.rpy.append((roll, pitch, yaw))
        self.contacts.append(sim.contacts())
        self.rover.append(st.get("rover", "-"))
        self.gait.append(st.get("gait", "-"))
        self.nav.append(st.get("nav", -1))
        self.obstacle.append(st.get("obstacle", -1))
        self.warn.append(st.get("warn", 0))
        self.err.append(st.get("err", 0))
        self.cmd_vx.append(st.get("cmd_vx", 0.0))
        h = sim.obstacle_hits()
        self.hits.append(sorted(g for g in h if not g.endswith('_foot')))

    def arr(self):
        self.t = np.array(self.t)
        self.pos = np.array(self.pos)
        self.rpy = np.array(self.rpy)
        self.contacts = np.array(self.contacts)
        return self

    def idx(self, t):
        return int(np.searchsorted(self.t, t))


def fell(sim):
    x, y, z, yaw, pitch, roll = sim.pose()
    return (abs(roll) > math.radians(60) or abs(pitch) > math.radians(60) or not sim.finite())


# ------------------------------------------------------------- co-sim loop ---
def run_loop(sim, host, seconds, *, script=None, depth_hz=0.0, depth_fn=None, on_tick=None,
             stop_if_fell=True):
    """Advance sim + autonomy host together.
    script(t, host)   called every control tick (publish stand/nav/cmd_vel for open-loop tests)
    depth_hz          >0: send ray-cast depth frames to the host (closed-loop tests)
    depth_fn(img, t)  optional hook to corrupt / drop frames (return None to drop)
    on_tick(t, sim, host, status)  optional per-tick hook (fault injection)"""
    tr = Trace()
    host.info(sim)
    n = int(round(seconds / CTRL_DT))
    next_depth = 0.0
    for k in range(1, n + 1):
        t = k * CTRL_DT
        if script:
            script(t, host)
        if depth_hz > 0 and sim.started and t >= next_depth - 1e-9:
            img = sim.depth_mm()
            if depth_fn:
                img = depth_fn(img, t)
            if img is not None:
                host.depth(img)
            next_depth += 1.0 / depth_hz
        st, q = host.tick(t)
        if q is None:
            continue
        if not sim.started:
            sim.spawn(q)
        sim.step(q)
        if on_tick:
            on_tick(t, sim, host, st)
        tr.add(t, sim, st)
        if stop_if_fell and fell(sim):
            break
    return tr.arr()


def openloop_script(stand_at=0.0, nav_at=3.5, cmd=(0.0, 0.0, 0.0), until=1e9, cmd_period=0.1):
    """Stand, then enable nav and stream /cmd_vel like reactive_avoidance would."""
    state = {"last": -1.0}

    def f(t, host):
        if t >= stand_at and state["last"] < 0:
            host.pub_bool("/stand_request", True)
        if t >= nav_at and state["last"] < 0:
            host.pub_bool("/nav_enabled", True)
        if t >= nav_at and t < until and t - state["last"] >= cmd_period - 1e-9:
            host.pub_twist(*cmd)
            state["last"] = t
        elif state["last"] < 0 and t >= nav_at:
            state["last"] = t
    return f


# ----------------------------------------------------------------- metrics ---
def forward_progress(tr, t0, t1):
    """Displacement along the initial heading, lateral drift, yaw change, over [t0, t1]."""
    i0, i1 = tr.idx(t0), min(tr.idx(t1), len(tr.t) - 1)
    yaw0 = tr.rpy[i0, 2]
    d = tr.pos[i1, :2] - tr.pos[i0, :2]
    fwd = d[0] * math.cos(yaw0) + d[1] * math.sin(yaw0)
    lat = -d[0] * math.sin(yaw0) + d[1] * math.cos(yaw0)
    yaw = np.unwrap(tr.rpy[:, 2])
    dyaw = float(yaw[i1] - yaw[i0])          # unwrapped: turn-in-place can exceed 180 deg
    return fwd, lat, dyaw


def max_tilt_deg(tr, t0, t1):
    i0, i1 = tr.idx(t0), tr.idx(t1)
    seg = np.abs(tr.rpy[i0:i1, :2])
    return float(np.degrees(seg.max())) if seg.size else 0.0


def contact_stats(tr, t0, t1, min_feet=3):
    i0, i1 = tr.idx(t0), tr.idx(t1)
    n = tr.contacts[i0:i1].sum(axis=1)
    return float(np.mean(n >= min_feet)) if len(n) else 1.0, int(n.min()) if len(n) else 6
