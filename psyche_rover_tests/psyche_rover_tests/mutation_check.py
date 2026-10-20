#!/usr/bin/env python3
"""Does the suite actually catch bugs? Inject plausible faults into a COPY of the source (your
workspace is never modified) and confirm that the expected test turns red.

    python3 mutation_check.py              # all mutations (~1 min each: rebuild + one filtered test)
    python3 mutation_check.py -k picker    # only mutations whose name contains 'picker'

A mutation is 'killed' if at least one test whose name contains the expected substring FAILs.
A 'survivor' means a gap in the suite (or an equivalent mutation) - investigate it.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

PKG = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(PKG)

# (name, file relative to src/, old text, new text, test-name substring that must FAIL)
MUTATIONS = [
    ("gait: left-front mount yaw sign flipped", "psyche_hexapod_gait/include/psyche_hexapod_gait/hexapod_kinematics.hpp",
     "{+body_half_len, +body_half_width, +pi / 4.0},", "{+body_half_len, +body_half_width, -pi / 4.0},",
     "left_right_mirror"),
    ("picker: published heading sign inverted", "psyche_terrain_mapping/src/path_picker_node.cpp",
     "heading_msg.data = static_cast<float>(best_heading_rad);", "heading_msg.data = -static_cast<float>(best_heading_rad);",
     "obstacle_on_left_steers_right"),
    ("state machine: map settle window removed", "psyche_state_machine/src/rover_state_machine_node.cpp",
     ">= map_settle_s_)", ">= -1.0)", "costmaps_during_settle_window_are_discarded"),
    ("state machine: obstacle no longer leaves TRAVERSE", "psyche_state_machine/src/rover_state_machine_node.cpp",
     "if (obstacle_detected_) {", "if (false && obstacle_detected_) {", "obstacle_during_traverse_disables_nav"),
    ("avoidance: dodge direction swapped", "psyche_obstacle_avoidance/src/reactive_avoidance_node.cpp",
     "(left_min > right_min) ? angular_speed_ : -angular_speed_", "(left_min > right_min) ? -angular_speed_ : angular_speed_",
     "avoid_slows_and_turns"),
    ("mapper: height above ground sign wrong", "psyche_terrain_mapping/src/terrain_mapper_node.cpp",
     "camera_height_m_ - y_rot", "camera_height_m_ + y_rot", "flat_floor_has_no_obstacles"),
    ("gait controller: /cmd_vel timeout disabled", "psyche_hexapod_gait/src/gait_controller_node.cpp",
     "< cmd_timeout_s_;", "< 1e9;", "cmd_vel_timeout_stops_walking"),
    ("gait controller: declares STANDING before the stand-up blend finishes", "psyche_hexapod_gait/src/gait_controller_node.cpp",
     "else if (posture_blend_ >= 1.0) {posture_ = Posture::STANDING;}", "else if (posture_blend_ >= 0.2) {posture_ = Posture::STANDING;}",
     "stand_up_takes_stand_time"),
    ("detector: obstacle trigger comparison inverted", "psyche_obstacle_avoidance/src/obstacle_detector_node.cpp",
     "if (clean <= obstacle_trigger_m_) {", "if (clean > obstacle_trigger_m_) {", "detector_trigger_distance_boundary"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", default="")
    a = ap.parse_args()
    killed, survived = [], []
    for name, rel, old, new, expect in MUTATIONS:
        if a.k not in name:
            continue
        tmp = tempfile.mkdtemp(prefix="psyche_mut_")
        try:
            dst = os.path.join(tmp, "src")
            shutil.copytree(SRC, dst, ignore=shutil.ignore_patterns("build", "__pycache__", "test_report*", ".git"))
            path = os.path.join(dst, rel)
            text = open(path).read()
            if old not in text:
                print(f"[SKIP ] {name}: pattern not found in {rel} (source changed?)")
                continue
            open(path, "w").write(text.replace(old, new))
            r = subprocess.run([sys.executable, os.path.join(dst, "psyche_rover_tests", "run_all_tests.py"),
                                "--only", "cpp", "--filter", expect, "--report", os.path.join(tmp, "rep")],
                               capture_output=True, text=True)
            if "BUILD FAILED" in r.stdout:
                print(f"[SKIP ] {name}: mutant did not compile")
                continue
            failed = False
            for jf in os.listdir(os.path.join(dst, "psyche_rover_tests", "build", "results")):
                with open(os.path.join(dst, "psyche_rover_tests", "build", "results", jf)) as f:
                    for t in json.load(f)["results"]:
                        if t["status"] == "FAIL" and expect in t["name"]:
                            failed = True
            (killed if failed else survived).append(name)
            print(f"[{'KILLED' if failed else 'SURVIVED'}] {name}  (expected '{expect}' to fail)")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    print(f"\n{len(killed)} killed, {len(survived)} survived")
    return 1 if survived else 0


if __name__ == "__main__":
    sys.exit(main())
