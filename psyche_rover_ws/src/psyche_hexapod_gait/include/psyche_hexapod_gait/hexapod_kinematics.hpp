#ifndef PSYCHE_HEXAPOD_GAIT__HEXAPOD_KINEMATICS_HPP_
#define PSYCHE_HEXAPOD_GAIT__HEXAPOD_KINEMATICS_HPP_

#include <algorithm>
#include <array>
#include <cmath>

namespace psyche_hexapod_gait
{

// Header-only and ROS-free so it can be unit-tested on its own.
//
// Body frame: +x forward, +y left, +z up, origin at body center.
//
// Leg numbering (top view, front of robot at top):
//
//     1 front-left     0 front-right
//     3 mid-left       2 mid-right
//     5 rear-left      4 rear-right
//
// Even index = right side, odd index = left side. Tripod groups are
// {0, 3, 4} and {1, 2, 5}: each group has one leg per side and forms a
// stable triangle while the other group swings.

constexpr int kNumLegs = 6;

struct LegGeometry
{
  double coxa_len{0.050};   // hip yaw axis -> femur joint (m)   PLACEHOLDER
  double femur_len{0.080};  // femur joint -> knee (m)           PLACEHOLDER
  double tibia_len{0.130};  // knee -> foot tip (m)              PLACEHOLDER
};

struct LegMount
{
  double x;    // coxa joint position in body frame (m)
  double y;
  double yaw;  // direction the leg points when coxa angle = 0 (rad)
};

struct JointAngles
{
  double coxa{0.0};
  double femur{0.0};
  double tibia{0.0};
  bool reachable{true};
};

struct Vec3
{
  double x{0.0}, y{0.0}, z{0.0};
};

// Hexagonal body: front/rear legs angled 45 deg out, middle legs straight
// out sideways. Dimensions are placeholders.
inline std::array<LegMount, kNumLegs> default_mounts(
  double body_half_len = 0.100, double body_half_width = 0.060)
{
  const double pi = M_PI;
  return {{
    {+body_half_len, -body_half_width, -pi / 4.0},          // 0 front-right
    {+body_half_len, +body_half_width, +pi / 4.0},          // 1 front-left
    {0.0, -body_half_width * 1.4, -pi / 2.0},               // 2 mid-right
    {0.0, +body_half_width * 1.4, +pi / 2.0},               // 3 mid-left
    {-body_half_len, -body_half_width, -3.0 * pi / 4.0},    // 4 rear-right
    {-body_half_len, +body_half_width, +3.0 * pi / 4.0},    // 5 rear-left
  }};
}

inline const char * leg_name(int leg)
{
  static const char * names[kNumLegs] = {
    "leg_0_front_right", "leg_1_front_left", "leg_2_mid_right",
    "leg_3_mid_left", "leg_4_rear_right", "leg_5_rear_left"};
  return names[leg];
}

// Tripod group membership: 0 or 1.
inline int tripod_group(int leg)
{
  switch (leg) {
    case 0: case 3: case 4: return 0;
    default: return 1;
  }
}

// Inverse kinematics for one 3-DOF leg. `foot` is the desired foot position
// in the leg frame: origin at the coxa joint, x along the leg's outward
// direction at coxa = 0, y to its left, z up.
//
// Angle conventions (radians):
//   coxa  : yaw about z; positive swings the foot toward +y of the leg frame
//   femur : pitch of upper leg from horizontal; positive = knee above horizontal
//   tibia : knee bend relative to the femur; 0 = straight leg, negative = foot
//           folded below the knee (normal walking pose)
inline JointAngles leg_ik(const LegGeometry & g, const Vec3 & foot)
{
  JointAngles out;

  out.coxa = std::atan2(foot.y, foot.x);

  // Reduce to the vertical plane containing the leg.
  const double horiz = std::hypot(foot.x, foot.y) - g.coxa_len;
  const double z = foot.z;
  double d = std::hypot(horiz, z);  // femur joint -> foot distance

  const double max_reach = g.femur_len + g.tibia_len;
  const double min_reach = std::abs(g.femur_len - g.tibia_len);
  if (d > max_reach - 1e-6 || d < min_reach + 1e-6) {
    out.reachable = false;
    d = std::clamp(d, min_reach + 1e-6, max_reach - 1e-6);
  }

  // Law of cosines for the knee.
  const double cos_knee =
    (g.femur_len * g.femur_len + g.tibia_len * g.tibia_len - d * d) /
    (2.0 * g.femur_len * g.tibia_len);
  const double knee_interior = std::acos(std::clamp(cos_knee, -1.0, 1.0));
  out.tibia = knee_interior - M_PI;  // interior angle pi (straight) -> 0

  // Femur angle = direction to foot, raised by the triangle's base angle
  // (knee-up configuration).
  const double cos_off =
    (g.femur_len * g.femur_len + d * d - g.tibia_len * g.tibia_len) /
    (2.0 * g.femur_len * d);
  const double offset = std::acos(std::clamp(cos_off, -1.0, 1.0));
  out.femur = std::atan2(z, horiz) + offset;

  return out;
}

// Forward kinematics (inverse of leg_ik). Used by the unit test.
inline Vec3 leg_fk(const LegGeometry & g, const JointAngles & q)
{
  // Knee position in the leg's vertical plane.
  const double kx = g.coxa_len + g.femur_len * std::cos(q.femur);
  const double kz = g.femur_len * std::sin(q.femur);
  // Tibia direction = femur direction rotated by the (negative) knee bend.
  const double tib_dir = q.femur + q.tibia;
  const double fx = kx + g.tibia_len * std::cos(tib_dir);
  const double fz = kz + g.tibia_len * std::sin(tib_dir);
  return {fx * std::cos(q.coxa), fx * std::sin(q.coxa), fz};
}

// Body-frame point -> leg frame for a given mount.
inline Vec3 body_to_leg(const LegMount & m, const Vec3 & p)
{
  const double dx = p.x - m.x;
  const double dy = p.y - m.y;
  const double c = std::cos(-m.yaw);
  const double s = std::sin(-m.yaw);
  return {dx * c - dy * s, dx * s + dy * c, p.z};
}

// Leg-frame point -> body frame.
inline Vec3 leg_to_body(const LegMount & m, const Vec3 & p)
{
  const double c = std::cos(m.yaw);
  const double s = std::sin(m.yaw);
  return {m.x + p.x * c - p.y * s, m.y + p.x * s + p.y * c, p.z};
}

}  // namespace psyche_hexapod_gait

#endif  // PSYCHE_HEXAPOD_GAIT__HEXAPOD_KINEMATICS_HPP_
