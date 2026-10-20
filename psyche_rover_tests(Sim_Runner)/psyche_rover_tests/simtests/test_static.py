"""Static checks that need no simulator: the generated models agree with each other and with the
gait package, YAML files agree with the code, manifests list what the sources include, launch files
point at things that exist. These catch the 'works on my machine until the first colcon build' class
of problems, and the Gazebo-vs-MuJoCo parity that cannot be tested dynamically without Gazebo."""
import glob
import math
import os
import re
import xml.etree.ElementTree as ET

import yaml

from harness import chk, info, require, run, test, xfail
from sim_env import SRC, gm, _args

SIM = os.path.join(SRC, "psyche_hexapod_sim")
GAIT = os.path.join(SRC, "psyche_hexapod_gait")
PACKAGES = ["psyche_hexapod_gait", "psyche_obstacle_avoidance", "psyche_terrain_mapping",
            "psyche_state_machine", "psyche_hexapod_sim"]


def read(path):
    with open(path) as f:
        return f.read()


def urdf():
    return ET.parse(os.path.join(SIM, "urdf", "psyche_hexapod.urdf")).getroot()


def mjcf():
    return ET.parse(os.path.join(SIM, "mjcf", "psyche_hexapod.xml")).getroot()


def fl(s):
    return [float(v) for v in s.split()]


# ---------------------------------------------------------------- names ---
def gait_joint_names():
    txt = read(os.path.join(GAIT, "include", "psyche_hexapod_gait", "hexapod_kinematics.hpp"))
    legs = re.findall(r'"(leg_\d_\w+)"', txt)
    return [f"{l}_{j}" for l in legs for j in ("coxa", "femur", "tibia")]


@test
def joint_names_agree_everywhere():
    want = gait_joint_names()
    require(len(want) == 18, f"found {len(want)} names in hexapod_kinematics.hpp")
    urdf_j = [j.get("name") for j in urdf().iter("joint") if j.get("type") == "revolute"]
    mj_a = [a.get("name") for a in mjcf().find("actuator").iter("position")]
    ctrl = yaml.safe_load(read(os.path.join(SIM, "config", "controllers.yaml")))["leg_position_controller"]["ros__parameters"]["joints"]
    gen = [f"{n}_{j}" for n in gm.LEG_NAMES for j in gm.JOINTS]
    chk(set(urdf_j) == set(want) and len(urdf_j) == 18, "URDF joint names differ from gait_controller")
    chk(mj_a == want, "MJCF actuator order differs from gait_controller joint order (sim indexes by name, "
        "but check_gait_in_mujoco.py indexes by position)")
    chk(ctrl == want, "controllers.yaml joint order differs (JointGroupPositionController is order-sensitive)")
    chk(gen == want, "generate_models.py leg/joint naming differs")


@test
def leg_mount_geometry_matches_gait_package():
    mounts_gen = gm.mounts(0.10, 0.06)
    txt = read(os.path.join(GAIT, "include", "psyche_hexapod_gait", "hexapod_kinematics.hpp"))
    chk("body_half_len = 0.100" in txt and "body_half_width = 0.060" in txt, "default mount dims changed in the header")
    # the header's mount table, evaluated numerically
    hl, hw, p = 0.10, 0.06, math.pi
    cpp = [(hl, -hw, -p / 4), (hl, hw, p / 4), (0, -hw * 1.4, -p / 2), (0, hw * 1.4, p / 2),
           (-hl, -hw, -3 * p / 4), (-hl, hw, 3 * p / 4)]
    for a, b in zip(mounts_gen, cpp):
        chk(all(abs(x - y) < 1e-9 for x, y in zip(a, b)), f"mount mismatch {a} vs {b}")


# --------------------------------------------------------------- parity ---
@test
def urdf_and_mjcf_describe_the_same_robot():
    u, m = urdf(), mjcf()
    # joints: origin, axis, limit, damping
    mj_bodies = {b.get("name"): b for b in m.iter("body")}
    mj_default = m.find("default/joint")
    for j in u.iter("joint"):
        if j.get("type") != "revolute":
            continue
        name = j.get("name")
        link = name + "_link"
        body = mj_bodies[link]
        ux = fl(j.find("origin").get("xyz"))
        uy = fl(j.find("origin").get("rpy") or "0 0 0")
        mx = fl(body.get("pos"))
        chk(all(abs(a - b) < 1e-4 for a, b in zip(ux, mx)), f"{name}: origin {ux} vs {mx}")
        if name.endswith("coxa"):
            chk(abs(uy[2] - fl(body.get("euler"))[2]) < 1e-4, f"{name}: mount yaw differs")
        ax_u = fl(j.find("axis").get("xyz"))
        ax_m = fl(body.find("joint").get("axis"))
        chk(ax_u == ax_m, f"{name}: axis {ax_u} vs {ax_m}")
        lim = j.find("limit")
        rng = fl(mj_default.get("range"))
        chk(abs(float(lim.get("lower")) - rng[0]) < 1e-9 and abs(float(lim.get("upper")) - rng[1]) < 1e-9, f"{name}: joint limits differ")
        chk(abs(float(j.find("dynamics").get("damping")) - float(mj_default.get("damping"))) < 1e-9, f"{name}: damping differs")
    # effort limit vs actuator force range
    eff_u = {float(j.find("limit").get("effort")) for j in u.iter("joint") if j.get("type") == "revolute"}
    fr = fl(m.find("default/position").get("forcerange"))
    chk(eff_u == {fr[1]} and fr[0] == -fr[1], f"effort limits differ: URDF {eff_u} vs MJCF {fr}")


@test
def link_lengths_masses_and_feet_agree():
    u, m = urdf(), mjcf()
    for leg in gm.LEG_NAMES:
        for part, length in (("coxa", 0.05), ("femur", 0.08), ("tibia", 0.13)):
            link = [l for l in u.iter("link") if l.get("name") == f"{leg}_{part}_link"][0]
            cyl = link.find("collision/geometry/cylinder")
            chk(abs(float(cyl.get("length")) - length) < 1e-6, f"URDF {leg} {part} length")
            mass_u = float(link.find("inertial/mass").get("value"))
            body = [b for b in m.iter("body") if b.get("name") == f"{leg}_{part}_link"][0]
            cap = body.find("geom")
            ft = fl(cap.get("fromto"))
            chk(abs(ft[3] - length) < 1e-6, f"MJCF {leg} {part} length {ft[3]} vs {length}")
            chk(abs(float(cap.get("mass")) - mass_u) < 1e-9, f"{leg} {part}: mass differs")
        foot_u = [l for l in u.iter("link") if l.get("name") == f"{leg}_tibia_link"][0].find("collision/geometry/sphere")
        foot_m = [g for g in m.iter("geom") if g.get("name") == f"{leg}_foot"][0]
        chk(abs(float(foot_u.get("radius")) - float(foot_m.get("size"))) < 1e-9, f"{leg}: foot radius differs")
    # body
    bu = [l for l in u.iter("link") if l.get("name") == "base_link"][0]
    size_u = fl(bu.find("collision/geometry/box").get("size"))
    bm = [g for g in m.iter("geom") if g.get("name") == "body"][0]
    size_m = [2 * v for v in fl(bm.get("size"))]
    chk(all(abs(a - b) < 1e-4 for a, b in zip(size_u, size_m)), f"body box {size_u} vs {size_m}")
    chk(abs(float(bu.find("inertial/mass").get("value")) - float(bm.get("mass"))) < 1e-9, "body mass differs")


@test
def sensors_and_world_parity_between_gazebo_and_mujoco():
    u, m = urdf(), mjcf()
    cam_u = [j for j in u.iter("joint") if j.get("name") == "camera_joint"][0].find("origin").get("xyz")
    cam_m = [c for c in m.iter("camera") if c.get("name") == "oak"][0]
    chk(all(abs(a - b) < 1e-4 for a, b in zip(fl(cam_u), fl(cam_m.get("pos")))), f"camera pose {cam_u} vs {cam_m.get('pos')}")
    imu_u = [j for j in u.iter("joint") if j.get("name") == "imu_joint"][0].find("origin").get("xyz")
    imu_m = [s for s in m.iter("site") if s.get("name") == "imu"][0].get("pos")
    chk(fl(imu_u) == fl(imu_m), "IMU position differs")
    # field of view: Gazebo specifies horizontal FOV for a 320x200 image, MuJoCo vertical FOV
    cam_sdf = next(u.iter("camera"))
    hfov = float(cam_sdf.find("horizontal_fov").text)
    w, h = int(cam_sdf.find("image/width").text), int(cam_sdf.find("image/height").text)
    vfov = 2 * math.atan(math.tan(hfov / 2) * h / w)
    info(f"Gazebo hfov {math.degrees(hfov):.1f} deg -> vfov {math.degrees(vfov):.1f} deg; MuJoCo fovy {cam_m.get('fovy')} deg")
    chk(abs(math.degrees(vfov) - float(cam_m.get("fovy"))) < 1.0, "Gazebo and MuJoCo camera FOV differ by >1 deg")
    chk((w, h) == (320, 200), "depth image size differs from mujoco_sim_node default 320x200")
    near, far = float(cam_sdf.find("clip/near").text), float(cam_sdf.find("clip/far").text)
    chk(near <= 0.15 and far >= 6.0, f"Gazebo depth clip [{near}, {far}] must cover the mapper's 0.15-6 m range")
    # physics
    world = ET.parse(os.path.join(SIM, "worlds", "psyche_world.sdf")).getroot()
    step = float(world.find(".//max_step_size").text)
    chk(abs(step - float(m.find("option").get("timestep"))) < 1e-12, "physics step differs")
    chk(fl(world.find(".//gravity").text)[2] == fl(m.find("option").get("gravity"))[2], "gravity differs")
    mu_world = float(world.find(".//mu").text)
    mu_mj = float(m.find("default/geom").get("friction").split()[0])
    chk(abs(mu_world - mu_mj) < 1e-9, f"ground friction {mu_world} vs {mu_mj}")
    mu_foot = {float(x.text) for x in u.iter("mu1")}
    chk(mu_foot == {mu_mj}, f"foot friction in URDF {mu_foot} vs MJCF {mu_mj}")
    # obstacles
    obs_w = sorted((fl(mod.find("pose").text)[0], fl(mod.find("pose").text)[1], 2 * fl(mod.find("pose").text)[2])
                   for mod in world.iter("model") if mod.get("name", "").startswith("obstacle_"))
    obs_m = sorted((fl(g.get("pos"))[0], fl(g.get("pos"))[1], 2 * fl(g.get("pos"))[2]) for g in m.iter("geom")
                   if (g.get("name") or "").startswith("obstacle_"))
    chk(len(obs_w) == len(obs_m) == 3 and all(abs(a - b) < 1e-4 for p, q in zip(obs_w, obs_m) for a, b in zip(p, q)),
        f"obstacles differ: {obs_w} vs {obs_m}")


@test
def generated_files_are_up_to_date_with_the_generator():
    a = _args()
    a.obstacles = [(1.0, 0.0, 0.03), (1.8, -0.4, 0.08), (2.5, 0.35, 0.12)]
    a.controllers_yaml = "$(find psyche_hexapod_sim)/config/controllers.yaml"
    for sub, name, gen in (("urdf", "psyche_hexapod.urdf", gm.gen_urdf), ("mjcf", "psyche_hexapod.xml", gm.gen_mjcf),
                           ("worlds", "psyche_world.sdf", gm.gen_world)):
        chk(read(os.path.join(SIM, sub, name)) == gen(a), f"{sub}/{name} was hand-edited or is stale: rerun generate_models.py")


@test
def generator_defaults_match_the_gait_and_mapper_params():
    gait = yaml.safe_load(read(os.path.join(GAIT, "config", "params.yaml")))["gait_controller"]["ros__parameters"]
    tmap = yaml.safe_load(read(os.path.join(SRC, "psyche_terrain_mapping", "config", "params.yaml")))["terrain_mapper"]["ros__parameters"]
    a = _args()
    for g, k in ((a.coxa, "coxa_len_m"), (a.femur, "femur_len_m"), (a.tibia, "tibia_len_m"), (a.hl, "body_half_len_m"),
                 (a.hw, "body_half_width_m"), (a.stand, "stand_height_m")):
        chk(abs(g - gait[k]) < 1e-9, f"generator {g} vs gait {k} {gait[k]}")
    chk(abs(a.cam_h - tmap["camera_height_m"]) < 1e-9, "sim camera height differs from terrain_mapper camera_height_m")
    chk(abs(a.vel - 6.0) < 1e-9, "sim velocity limit changed (tests assume 6 rad/s)")


# ----------------------------------------------------------------- config ---
NODE_FILES = {
    "gait_controller": "psyche_hexapod_gait/src/gait_controller_node.cpp",
    "obstacle_detector": "psyche_obstacle_avoidance/src/obstacle_detector_node.cpp",
    "reactive_avoidance": "psyche_obstacle_avoidance/src/reactive_avoidance_node.cpp",
    "terrain_mapper": "psyche_terrain_mapping/src/terrain_mapper_node.cpp",
    "path_picker": "psyche_terrain_mapping/src/path_picker_node.cpp",
}
YAML_FILES = ["psyche_hexapod_gait/config/params.yaml", "psyche_obstacle_avoidance/config/params.yaml",
              "psyche_terrain_mapping/config/params.yaml"]


def declared(node):
    txt = read(os.path.join(SRC, NODE_FILES[node]))
    out = {}
    for name, default in re.findall(r'declare_parameter<(?:[^<>]|<[^<>]*>)+>\(\s*"(\w+)"\s*,\s*([^;]+?)\);', txt):
        default = default.strip()
        try:
            out[name] = float(default)
        except ValueError:
            if default in ("true", "false"):
                out[name] = default == "true"
            else:
                m = re.match(r'"([^"]*)"$', default)
                out[name] = m.group(1) if m else None
    return out


def yaml_params():
    out = {}
    for f in YAML_FILES:
        for node, body in yaml.safe_load(read(os.path.join(SRC, f))).items():
            out[node] = body["ros__parameters"]
    return out


@test
def every_yaml_key_is_declared_by_its_node():
    # an undeclared key (typo) is silently ignored by ROS 2: the default is used instead
    y = yaml_params()
    for node in NODE_FILES:
        d = declared(node)
        for k in y[node]:
            chk(k in d, f"{node}: YAML key '{k}' is not declared in the source (typo? it would be ignored)")


@test
def yaml_values_equal_code_defaults():
    y = yaml_params()
    drift = []
    for node in NODE_FILES:
        d = declared(node)
        for k, v in y[node].items():
            if k in d and isinstance(d[k], (float, bool)) and not isinstance(v, list):
                if abs(float(v) - float(d[k])) > 1e-9:
                    drift.append(f"{node}.{k}: yaml {v} vs code default {d[k]}")
            elif k in d and isinstance(d[k], str) and d[k] != v:
                drift.append(f"{node}.{k}: yaml '{v}' vs code default '{d[k]}'")
    for dline in drift:
        chk(False, dline)
    info(f"{len(drift)} YAML/code default mismatches")


@test
def every_declared_parameter_is_present_in_yaml():
    y = yaml_params()
    missing = []
    for node in NODE_FILES:
        for k in declared(node):
            if k not in y[node]:
                missing.append(f"{node}.{k}")
    info("parameters only settable in code (not in YAML): " + (", ".join(missing) or "none"))
    chk(not missing, "parameters missing from YAML (cannot be tuned from config): " + ", ".join(missing))


# --------------------------------------------------------------- manifests ---
@test
def package_manifests_cover_every_included_ros_package():
    known = {"rclcpp", "std_msgs", "geometry_msgs", "sensor_msgs", "nav_msgs", "cv_bridge", "image_transport"}
    for pkg in PACKAGES[:4]:
        root = os.path.join(SRC, pkg)
        used = set()
        for f in glob.glob(os.path.join(root, "src", "*.cpp")) + glob.glob(os.path.join(root, "include", "**", "*.hpp"), recursive=True):
            for inc in re.findall(r"#include\s*<(\w+)/", read(f)):
                if inc in known:
                    used.add(inc)
            if "opencv2" in read(f):
                used.add("OpenCV")
        pxml = read(os.path.join(root, "package.xml"))
        cmake = read(os.path.join(root, "CMakeLists.txt"))
        for dep in sorted(used):
            chk(f"<depend>{dep}</depend>" in pxml, f"{pkg}: package.xml missing <depend>{dep}</depend>")
            chk(f"find_package({dep}" in cmake, f"{pkg}: CMakeLists missing find_package({dep})")


@test
def launch_files_reference_real_executables_and_configs():
    exes = {}
    for pkg in PACKAGES:
        cm = read(os.path.join(SRC, pkg, "CMakeLists.txt"))
        exes[pkg] = set(re.findall(r"add_executable\((\w+)", cm))
        for m in re.findall(r"install\(PROGRAMS\s+([^)]+)\)", cm):
            exes[pkg] |= {os.path.basename(p) for p in m.split() if "scripts" in p}
    for lf in glob.glob(os.path.join(SRC, "*", "launch", "*.py")):
        txt = read(lf)
        pairs = re.findall(r"n\(\s*'(\w+)',\s*'(\w+)'", txt) + re.findall(r"package\s*=\s*['\"](\w+)['\"]\s*,\s*executable\s*=\s*['\"](\w+)['\"]", txt)
        for pkg, exe in pairs:
            if pkg not in exes:
                continue          # external ROS package (robot_state_publisher, ros_gz_*, ...): not ours to check
            if pkg in exes:
                chk(exe in exes[pkg] or exe + ".py" in exes[pkg], f"{os.path.basename(lf)}: {pkg} has no executable '{exe}'")
        for pkg, d, f in re.findall(r"share\('(\w+)'\),\s*'(\w+)',\s*'([\w\.]+)'", txt):
            if pkg in exes:
                chk(os.path.exists(os.path.join(SRC, pkg, d, f)), f"{os.path.basename(lf)}: missing {pkg}/{d}/{f}")


@test
def gazebo_bridge_topics_match_node_subscriptions():
    br = [f for f in glob.glob(os.path.join(SIM, "config", "*.yaml")) if "bridge" in f]
    require(br, "no bridge yaml found")
    topics = {e["ros_topic_name"] for e in yaml.safe_load(read(br[0]))}
    det = declared("obstacle_detector")["depth_topic"]
    tm = declared("terrain_mapper")
    for t in (det, tm["depth_topic"], tm["camera_info_topic"], "/clock", "/imu"):
        chk(t in topics, f"bridge does not forward {t}")


@test
def gazebo_depth_format_has_a_matching_override():
    sdf_fmt = next(urdf().iter("camera")).find("image/format").text
    ov = yaml.safe_load(read(os.path.join(SIM, "config", "sim_overrides_gazebo.yaml")))
    chk(sdf_fmt == "R_FLOAT32", f"Gazebo depth format changed to {sdf_fmt}")
    chk(ov["obstacle_detector"]["ros__parameters"]["depth_is_millimeters"] is False,
        "Gazebo publishes metres (R_FLOAT32) but the override does not switch obstacle_detector to float")


@test
def tests_are_registered_with_cmake_for_colcon_test():
    cm = read(os.path.join(GAIT, "CMakeLists.txt"))
    for t in ("test_kinematics", "test_gait"):
        chk(f"add_test(NAME" in cm and t in cm, f"{t} not registered")
    for pkg in ("psyche_terrain_mapping", "psyche_obstacle_avoidance", "psyche_state_machine"):
        cmk = read(os.path.join(SRC, pkg, "CMakeLists.txt"))
        chk("add_test" not in cmk, f"{pkg} registers tests itself (those now live in psyche_rover_tests)")


if __name__ == "__main__":
    raise SystemExit(run("static"))
