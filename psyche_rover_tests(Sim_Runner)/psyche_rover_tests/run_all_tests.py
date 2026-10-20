#!/usr/bin/env python3
"""Build and run every Psyche test: C++ (shim), static, and MuJoCo simulation.

    python3 run_all_tests.py                 # everything except the slow soaks (~10 min)
    python3 run_all_tests.py --quick         # C++ + static only (~1 min, no simulator)
    python3 run_all_tests.py --long          # + 30-minute endurance and 20-run repeatability
    python3 run_all_tests.py --only cpp,static
    python3 run_all_tests.py --filter slope  # name substring, forwarded to every suite

Needs: g++ (C++17), python3 with numpy, mujoco, pyyaml (pip install mujoco numpy pyyaml).
No ROS 2 installation is required: the real node sources are compiled against shim/.
Exit code is non-zero if any test FAILS or a known issue unexpectedly PASSES (XPASS).
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import time

PKG = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(PKG)
BUILD = os.path.join(PKG, "build")
CXX = os.environ.get("CXX", "g++")

NODES = [("terrain_mapper_node", "psyche_terrain_mapping"), ("path_picker_node", "psyche_terrain_mapping"),
         ("obstacle_detector_node", "psyche_obstacle_avoidance"), ("reactive_avoidance_node", "psyche_obstacle_avoidance"),
         ("rover_state_machine_node", "psyche_state_machine"), ("gait_controller_node", "psyche_hexapod_gait")]
INCLUDES = ["shim", "../psyche_terrain_mapping/include", "../psyche_obstacle_avoidance/include",
            "../psyche_state_machine/include", "../psyche_hexapod_gait/include", "cpp"]
# ROS-free tests that need no shim and no node objects
PURE_TESTS = ["psyche_hexapod_gait/test/test_kinematics.cpp", "psyche_hexapod_gait/test/test_gait.cpp"]
CPP_TESTS = ["test_gait_extended", "test_terrain_mapper", "test_path_picker", "test_obstacle_avoidance",
             "test_gait_controller", "test_state_machine", "test_config_consistency", "test_system_faults"]
SIM_TESTS = ["test_sim_gait", "test_sim_autonomy"]
STATIC_TESTS = ["test_static"]


def newest(paths):
    return max((os.path.getmtime(p) for p in paths if os.path.exists(p)), default=0)


def all_sources():
    pats = [f"{PKG}/shim/**/*", f"{PKG}/cpp/*", f"{SRC}/psyche_*/include/**/*.hpp", f"{SRC}/psyche_*/src/*.cpp"]
    files = []
    for p in pats:
        files += [f for f in glob.glob(p, recursive=True) if os.path.isfile(f)]
    return files


def run_cmd(cmd, **kw):
    return subprocess.run(cmd, **kw)


def build(verbose=False):
    os.makedirs(os.path.join(BUILD, "nodes"), exist_ok=True)
    inc = [f"-I{os.path.join(PKG, i)}" for i in INCLUDES]
    flags = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-Wno-unused-parameter", f'-DPSYCHE_SRC_DIR="{SRC}"']
    stamp = newest(all_sources())
    ok = True

    def compile_(out, cmd):
        nonlocal ok
        if os.path.exists(out) and os.path.getmtime(out) >= stamp:
            return
        t = time.time()
        r = run_cmd(cmd + ["-o", out], capture_output=True, text=True)
        if r.returncode != 0:
            ok = False
            print(f"BUILD FAILED: {os.path.basename(out)}\n{r.stderr[-3000:]}")
        elif verbose:
            print(f"  built {os.path.basename(out)} ({time.time() - t:.0f}s)")

    for n, pkg in NODES:
        compile_(os.path.join(BUILD, "nodes", n + ".o"),
                 [CXX, *flags, "-c", f"-Dmain={n}_unused_main", *inc, f"{SRC}/{pkg}/src/{n}.cpp"])
    objs = sorted(glob.glob(os.path.join(BUILD, "nodes", "*.o")))
    for t in PURE_TESTS:
        name = os.path.basename(t)[:-4]
        compile_(os.path.join(BUILD, name), [CXX, *flags, f"-I{SRC}/psyche_hexapod_gait/include", f"{SRC}/{t}"])
    for t in CPP_TESTS + ["autonomy_host"]:
        if not ok:
            break
        compile_(os.path.join(BUILD, t), [CXX, *flags, *inc, f"{PKG}/cpp/{t}.cpp", *objs])
    return ok


def run_suite(cmd, jpath, env=None):
    r = subprocess.run(cmd, env=env)
    data = {"suite": os.path.basename(jpath), "results": []}
    if os.path.exists(jpath):
        with open(jpath) as f:
            data = json.load(f)
    else:   # plain pass/fail executable (the original gait tests): exit code is the result
        data["results"].append(dict(name=os.path.basename(jpath)[:-5], status="PASS" if r.returncode == 0 else "FAIL",
                                    xfail="", detail="non-zero exit code" if r.returncode else "", infos=[]))
        with open(jpath, "w") as f:
            json.dump(data, f)
    return data, r.returncode


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true", help="C++ + static only (no simulator)")
    ap.add_argument("--long", action="store_true", help="include 30-min endurance / 20-run repeatability")
    ap.add_argument("--only", default="cpp,static,sim", help="comma list of: cpp,static,sim")
    ap.add_argument("--filter", default="", help="test-name substring")
    ap.add_argument("--report", default=os.path.join(PKG, "test_report"), help="path prefix for .md/.json report")
    ap.add_argument("-v", action="store_true")
    a = ap.parse_args()
    groups = set(a.only.split(","))
    if a.quick:
        groups.discard("sim")

    print("== building ==")
    t0 = time.time()
    if not build(a.v):
        return 2
    print(f"build ok ({time.time() - t0:.0f}s)")

    suites = []
    jdir = os.path.join(BUILD, "results")
    os.makedirs(jdir, exist_ok=True)
    extra = ["--filter", a.filter] if a.filter else []
    env = dict(os.environ, PSYCHE_HOST_BIN=os.path.join(BUILD, "autonomy_host"))

    if "cpp" in groups:
        print("\n== C++ tests (real node sources on the ROS shim) ==")
        for t in [os.path.basename(p)[:-4] for p in PURE_TESTS] + CPP_TESTS:
            j = os.path.join(jdir, t + ".json")
            suites.append((t, "cpp", *run_suite([os.path.join(BUILD, t), "--json", j] + extra, j)))
    if "static" in groups:
        print("\n== static consistency tests ==")
        for t in STATIC_TESTS:
            j = os.path.join(jdir, t + ".json")
            suites.append((t, "static", *run_suite([sys.executable, "-u", os.path.join(PKG, "simtests", t + ".py"), "--json", j] + extra, j, env)))
    if "sim" in groups:
        print("\n== MuJoCo simulation tests ==")
        for t in SIM_TESTS:
            j = os.path.join(jdir, t + ".json")
            cmd = [sys.executable, "-u", os.path.join(PKG, "simtests", t + ".py"), "--json", j] + extra
            if a.long:
                cmd.append("--long")
            suites.append((t, "sim", *run_suite(cmd, j, env)))

    # ---------------- summary ----------------
    tot = dict(PASS=0, FAIL=0, XFAIL=0, XPASS=0, SKIP=0)
    lines = ["# Psyche test report", "", f"Generated {time.strftime('%Y-%m-%d %H:%M:%S')}", "",
             "| suite | kind | pass | fail | known issues (xfail) | xpass | skipped |", "|---|---|---:|---:|---:|---:|---:|"]
    fails, known = [], []
    for name, kind, data, rc in suites:
        c = dict(PASS=0, FAIL=0, XFAIL=0, XPASS=0, SKIP=0)
        for r in data["results"]:
            c[r["status"]] += 1
            if r["status"] in ("FAIL", "XPASS"):
                fails.append((name, r))
            if r["status"] == "XFAIL":
                known.append((name, r))
        for k in tot:
            tot[k] += c[k]
        lines.append(f"| {name} | {kind} | {c['PASS']} | {c['FAIL']} | {c['XFAIL']} | {c['XPASS']} | {c['SKIP']} |")
    lines.append(f"| **total** | | **{tot['PASS']}** | **{tot['FAIL']}** | **{tot['XFAIL']}** | **{tot['XPASS']}** | **{tot['SKIP']}** |")
    lines += ["", f"## Failures / unexpected passes ({len(fails)})", ""]
    for name, r in fails:
        lines.append(f"- **{name}::{r['name']}** [{r['status']}] {r['detail'].strip().splitlines()[0] if r['detail'].strip() else r['xfail']}")
    lines += ["", f"## Known issues - tests that currently fail by design ({len(known)})", "",
              "Each is a real defect or limitation found by the suite, kept as an expected failure (XFAIL). When one is fixed "
              "the test reports XPASS and the run fails until the marker is removed.", ""]
    for name, r in known:
        lines.append(f"- **{name}::{r['name']}** - {r['xfail']}")
        for i in r.get("infos", []):
            lines.append(f"    - {i}")
    with open(a.report + ".md", "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(a.report + ".json", "w") as f:
        json.dump(dict(totals=tot, suites=[dict(name=n, kind=k, results=d["results"]) for n, k, d, _ in suites]), f, indent=1)

    print("\n" + "=" * 70)
    print(f"TOTAL: {tot['PASS']} pass, {tot['FAIL']} fail, {tot['XFAIL']} known issues (xfail), "
          f"{tot['XPASS']} xpass, {tot['SKIP']} skipped")
    for name, r in fails:
        print(f"  !! {name}::{r['name']} [{r['status']}]")
    print(f"report: {a.report}.md")
    return 1 if (tot["FAIL"] or tot["XPASS"]) else 0


if __name__ == "__main__":
    sys.exit(main())
