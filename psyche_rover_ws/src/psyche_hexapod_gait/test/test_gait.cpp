// Standalone gait sanity test (no ROS). Checks, for forward, strafe, and
// rotation commands, that (1) every foot target is reachable by the legs,
// (2) tripod support always has 3 feet down, and (3) the gait comes back to
// neutral when the command stops.
//   g++ -std=c++17 -Iinclude test/test_gait.cpp -o /tmp/test_gait && /tmp/test_gait
#include <cstdio>
#include "psyche_hexapod_gait/gait_generator.hpp"

using namespace psyche_hexapod_gait;

static int run(const char * name, double vx, double vy, double wz)
{
  GaitParams p;
  GaitGenerator g(p);
  g.set_body_height(p.stand_height);
  g.set_target(vx, vy, wz);
  int bad = 0;
  const double dt = 0.02;
  for (int k = 0; k < 600; ++k) {
    if (k == 400) {g.set_target(0, 0, 0);}
    g.step(dt);
    int down = 0;
    for (int i = 0; i < kNumLegs; ++i) {
      Vec3 f = g.foot_body(i);
      if (f.z <= -p.stand_height + 1e-6) {++down;}
      JointAngles q = leg_ik(p.geometry, body_to_leg(g.mounts()[i], f));
      if (!q.reachable) {
        if (bad < 3) {std::printf("  %s: leg %d unreachable at step %d\n", name, i, k);}
        ++bad;
      }
    }
    if (down < 3) {++bad; std::printf("  %s: only %d feet down at step %d\n", name, down, k);}
  }
  // After stopping, feet should be at neutral.
  for (int i = 0; i < kNumLegs; ++i) {
    Vec3 f = g.foot_body(i), n = g.neutral_foot(i);
    if (std::hypot(std::hypot(f.x - n.x, f.y - n.y), f.z - n.z) > 1e-3) {
      std::printf("  %s: leg %d not back at neutral\n", name, i);
      ++bad;
    }
  }
  std::printf("%-8s failures: %d (idle=%s)\n", name, bad, g.is_moving() ? "no" : "yes");
  return bad;
}

int main()
{
  int bad = 0;
  bad += run("forward", 0.08, 0, 0);
  bad += run("strafe", 0, 0.08, 0);
  bad += run("turn", 0, 0, 0.5);
  bad += run("arc", 0.05, 0, 0.3);
  bad += run("overspeed", 0.5, 0.5, 2.0);
  return bad == 0 ? 0 : 1;
}
