// Standalone (no ROS / gtest) sanity test: IK -> FK round trip and
// mount frame transforms. Build:
//   g++ -std=c++17 -Iinclude test/test_kinematics.cpp -o /tmp/test_kin && /tmp/test_kin
#include <cstdio>
#include "psyche_hexapod_gait/hexapod_kinematics.hpp"

using namespace psyche_hexapod_gait;

int main()
{
  LegGeometry g;
  int bad = 0, checked = 0;
  for (double x = 0.08; x <= 0.26; x += 0.02) {
    for (double y = -0.10; y <= 0.10; y += 0.05) {
      for (double z = -0.20; z <= -0.04; z += 0.04) {
        Vec3 f{x, y, z};
        JointAngles q = leg_ik(g, f);
        if (!q.reachable) {continue;}
        ++checked;
        Vec3 r = leg_fk(g, q);
        double e = std::hypot(std::hypot(r.x - f.x, r.y - f.y), r.z - f.z);
        if (e > 1e-6) {
          ++bad;
          std::printf("IK/FK mismatch %.2e at (%.2f %.2f %.2f)\n", e, x, y, z);
        }
      }
    }
  }
  auto mounts = default_mounts();
  for (int i = 0; i < kNumLegs; ++i) {
    Vec3 p{0.3, -0.1, -0.15};
    Vec3 q = leg_to_body(mounts[i], body_to_leg(mounts[i], p));
    if (std::abs(q.x - p.x) + std::abs(q.y - p.y) + std::abs(q.z - p.z) > 1e-9) {
      ++bad;
      std::printf("frame round trip failed for leg %d\n", i);
    }
  }
  std::printf("checked %d poses, failures: %d\n", checked, bad);
  return bad == 0 ? 0 : 1;
}
