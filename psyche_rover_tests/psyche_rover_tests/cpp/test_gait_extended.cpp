// Extended gait + kinematics tests. Header-only code under test, no ROS, no shim.
//
// Servo limits are ASSUMPTIONS -- edit kLimit* when you pick real servos.
#include <array>
#include <cmath>
#include <random>
#include <vector>

#include "psyche_hexapod_gait/gait_generator.hpp"
#include "test_framework.hpp"

using namespace psyche_hexapod_gait;

namespace
{
// Joint range of the simulated robot (URDF/MJCF use +/-2.6 rad) and the servo
// velocity limit used by the URDF (6 rad/s).
constexpr double kLimitRad = 2.6;
constexpr double kMaxJointVel = 6.0;
constexpr double kServoTravelRad = M_PI;  // a typical 180-degree hobby servo

struct Scenario {const char * name; double vx, vy, wz;};
const Scenario kScenarios[] = {
  {"forward", 0.08, 0, 0}, {"reverse", -0.08, 0, 0}, {"strafe_l", 0, 0.08, 0},
  {"strafe_r", 0, -0.08, 0}, {"turn_l", 0, 0, 0.5}, {"turn_r", 0, 0, -0.5},
  {"arc", 0.05, 0, 0.3}, {"diag", 0.06, 0.06, 0.0}, {"overspeed", 0.5, 0.5, 2.0}};

using Q18 = std::array<double, 18>;

Q18 joints(const GaitGenerator & g)
{
  Q18 q{};
  for (int i = 0; i < kNumLegs; ++i) {
    JointAngles a = leg_ik(g.params().geometry, body_to_leg(g.mounts()[i], g.foot_body(i)));
    q[3 * i] = a.coxa; q[3 * i + 1] = a.femur; q[3 * i + 2] = a.tibia;
  }
  return q;
}

struct Range {double lo[3]{9, 9, 9}, hi[3]{-9, -9, -9};};

void accumulate(Range & r, const Q18 & q)
{
  for (int i = 0; i < kNumLegs; ++i) {
    for (int j = 0; j < 3; ++j) {
      r.lo[j] = std::min(r.lo[j], q[3 * i + j]);
      r.hi[j] = std::max(r.hi[j], q[3 * i + j]);
    }
  }
}

bool all_reachable(const GaitGenerator & g)
{
  for (int i = 0; i < kNumLegs; ++i) {
    if (!leg_ik(g.params().geometry, body_to_leg(g.mounts()[i], g.foot_body(i))).reachable) {return false;}
  }
  return true;
}

bool feet_finite(const GaitGenerator & g)
{
  for (int i = 0; i < kNumLegs; ++i) {
    Vec3 f = g.foot_body(i);
    if (!std::isfinite(f.x) || !std::isfinite(f.y) || !std::isfinite(f.z)) {return false;}
  }
  return true;
}
}  // namespace

// ------------------------------------------------------------ joint limits ---

TEST(joint_ranges_within_model_limits_all_gaits)
{
  GaitParams p;
  Range total;
  for (const auto & s : kScenarios) {
    GaitGenerator g(p);
    g.set_body_height(p.stand_height);
    g.set_target(s.vx, s.vy, s.wz);
    for (int k = 0; k < 1500; ++k) {
      g.step(0.02);
      Q18 q = joints(g);
      accumulate(total, q);
      for (double a : q) {
        if (std::abs(a) > kLimitRad) {
          CHECK_MSG(false, "%s: joint angle %.3f exceeds +/-%.2f at step %d", s.name, a, kLimitRad, k);
          k = 1500; break;
        }
      }
    }
  }
  // sit pose too (it is the extreme femur/tibia pose)
  GaitGenerator g(p);
  g.set_body_height(p.sit_height);
  g.step(0.02);
  accumulate(total, joints(g));
  const char * jn[3] = {"coxa", "femur", "tibia"};
  for (int j = 0; j < 3; ++j) {
    INFO("%s range [%.3f, %.3f] rad (min margin to +/-%.1f limit: %.2f)", jn[j], total.lo[j], total.hi[j],
         kLimitRad, kLimitRad - std::max(std::abs(total.lo[j]), std::abs(total.hi[j])));
  }
}

TEST(joint_span_fits_180deg_servo_after_centering)
{
  // Even a 180-degree servo works if joint_offsets_rad centres the range.
  GaitParams p;
  Range r;
  for (const auto & s : kScenarios) {
    GaitGenerator g(p);
    g.set_body_height(p.stand_height);
    g.set_target(s.vx, s.vy, s.wz);
    for (int k = 0; k < 1500; ++k) {g.step(0.02); accumulate(r, joints(g));}
  }
  for (double h : {p.sit_height, p.stand_height}) {
    GaitGenerator g(p); g.set_body_height(h); g.step(0.02); accumulate(r, joints(g));
  }
  const char * jn[3] = {"coxa", "femur", "tibia"};
  for (int j = 0; j < 3; ++j) {
    const double span = r.hi[j] - r.lo[j];
    CHECK_MSG(span < kServoTravelRad - 0.2, "%s needs %.2f rad of travel; a 180 deg servo has %.2f",
              jn[j], span, kServoTravelRad);
    INFO("%s: needs %.0f deg travel, centre of range %.3f rad (candidate joint_offset)", jn[j],
         span * 180 / M_PI, 0.5 * (r.hi[j] + r.lo[j]));
  }
}

TEST(joint_velocity_bounded_steady_gaits)
{
  GaitParams p;
  double worst = 0;
  for (const auto & s : kScenarios) {
    GaitGenerator g(p);
    g.set_body_height(p.stand_height);
    g.set_target(s.vx, s.vy, s.wz);
    g.step(0.02);
    Q18 prev = joints(g);
    for (int k = 0; k < 1500; ++k) {
      if (k == 1000) {g.set_target(0, 0, 0);}
      g.step(0.02);
      Q18 q = joints(g);
      for (int i = 0; i < 18; ++i) {worst = std::max(worst, std::abs(q[i] - prev[i]) / 0.02);}
      prev = q;
    }
  }
  INFO("peak joint speed over all gaits: %.2f rad/s (limit %.1f)", worst, kMaxJointVel);
  CHECK(worst < kMaxJointVel);
}

TEST(joint_velocity_bounded_at_command_discontinuities)
{
  // Abrupt reversals / mode switches must not make a servo snap.
  struct Seq {const char * n; Scenario a, b;};
  const Seq seqs[] = {
    {"fwd->rev", {"", 0.08, 0, 0}, {"", -0.08, 0, 0}},
    {"turnL->turnR", {"", 0, 0, 0.5}, {"", 0, 0, -0.5}},
    {"fwd->strafe", {"", 0.08, 0, 0}, {"", 0, 0.08, 0}},
    {"strafe->turn", {"", 0, 0.08, 0}, {"", 0, 0, 0.5}},
    {"fwd->overspeed_rev", {"", 0.08, 0, 0}, {"", -1.0, 1.0, -3.0}}};
  GaitParams p;
  for (const auto & sq : seqs) {
    GaitGenerator g(p);
    g.set_body_height(p.stand_height);
    g.set_target(sq.a.vx, sq.a.vy, sq.a.wz);
    for (int k = 0; k < 400; ++k) {g.step(0.02);}
    Q18 prev = joints(g);
    g.set_target(sq.b.vx, sq.b.vy, sq.b.wz);
    double worst = 0, foot_jump = 0;
    std::array<Vec3, 6> pf;
    for (int i = 0; i < 6; ++i) {pf[i] = g.foot_body(i);}
    for (int k = 0; k < 300; ++k) {
      g.step(0.02);
      Q18 q = joints(g);
      for (int i = 0; i < 18; ++i) {worst = std::max(worst, std::abs(q[i] - prev[i]) / 0.02);}
      for (int i = 0; i < 6; ++i) {
        Vec3 f = g.foot_body(i);
        foot_jump = std::max(foot_jump, std::sqrt((f.x - pf[i].x) * (f.x - pf[i].x) +
                                                  (f.y - pf[i].y) * (f.y - pf[i].y) + (f.z - pf[i].z) * (f.z - pf[i].z)));
        pf[i] = f;
      }
      prev = q;
      CHECK_MSG(all_reachable(g), "%s: unreachable foot target after switch (step %d)", sq.n, k);
    }
    INFO("%s: peak joint speed %.2f rad/s, largest foot jump %.1f mm/step", sq.n, worst, foot_jump * 1000);
    CHECK_MSG(worst < kMaxJointVel, "%s: joint speed %.2f rad/s", sq.n, worst);
    CHECK_MSG(foot_jump < 0.012, "%s: foot teleported %.1f mm in one 20 ms step", sq.n, foot_jump * 1000);
  }
}

// --------------------------------------------------------- IK edge cases ---

TEST(ik_reach_boundaries)
{
  LegGeometry g;
  const double maxr = g.femur_len + g.tibia_len, minr = std::abs(g.femur_len - g.tibia_len);
  // Just inside the envelope: reachable and exact.
  for (double frac : {0.0005, 0.002, 0.01}) {
    Vec3 f{g.coxa_len + (maxr - frac), 0, 0};
    JointAngles q = leg_ik(g, f);
    CHECK_MSG(q.reachable, "foot %.4f m inside max reach should be reachable", frac);
    Vec3 r = leg_fk(g, q);
    CHECK_NEAR(r.x, f.x, 1e-6);
  }
  // Just outside: flagged unreachable, angles finite.
  for (double over : {0.002, 0.05, 1.0}) {
    JointAngles q = leg_ik(g, {g.coxa_len + maxr + over, 0, 0});
    CHECK_MSG(!q.reachable, "beyond max reach by %.3f must be unreachable", over);
    CHECK(std::isfinite(q.coxa) && std::isfinite(q.femur) && std::isfinite(q.tibia));
  }
  // Inside the minimum-reach hole (femur folded onto tibia).
  JointAngles q = leg_ik(g, {g.coxa_len + minr * 0.5, 0, 0});
  CHECK(!q.reachable);
  CHECK(std::isfinite(q.femur) && std::isfinite(q.tibia));
}

TEST(ik_foot_under_or_behind_hip_is_finite)
{
  LegGeometry g;
  for (Vec3 f : {Vec3{g.coxa_len, 0, -0.12}, Vec3{0, 0, -0.12}, Vec3{-0.03, 0, -0.10}, Vec3{0, 0.1, -0.12}}) {
    JointAngles q = leg_ik(g, f);
    CHECK(std::isfinite(q.coxa) && std::isfinite(q.femur) && std::isfinite(q.tibia));
    if (q.reachable) {
      Vec3 r = leg_fk(g, q);
      CHECK_NEAR(std::hypot(std::hypot(r.x - f.x, r.y - f.y), r.z - f.z), 0.0, 1e-6);
    }
  }
}

TEST(ik_fuzz_never_nan_and_flag_matches_fk)
{
  LegGeometry g;
  std::mt19937 rng(1234);
  std::uniform_real_distribution<double> u(-0.5, 0.5);
  int nan = 0, mismatch = 0, reach = 0;
  for (int i = 0; i < 200000; ++i) {
    Vec3 f{u(rng), u(rng), u(rng)};
    JointAngles q = leg_ik(g, f);
    if (!std::isfinite(q.coxa) || !std::isfinite(q.femur) || !std::isfinite(q.tibia)) {++nan; continue;}
    if (q.reachable) {
      ++reach;
      Vec3 r = leg_fk(g, q);
      if (std::hypot(std::hypot(r.x - f.x, r.y - f.y), r.z - f.z) > 1e-6) {++mismatch;}
    }
  }
  INFO("%d of 200000 random targets reachable", reach);
  CHECK_MSG(nan == 0, "%d NaN results", nan);
  CHECK_MSG(mismatch == 0, "%d reachable-flagged targets failed FK round trip", mismatch);
}

XFAIL_TEST(ik_nan_input_must_be_flagged_unreachable,
           "leg_ik(NaN) returns reachable=true with NaN angles (comparisons with NaN are false)")
{
  LegGeometry g;
  const double nan = std::nan("");
  JointAngles q = leg_ik(g, {nan, 0.0, -0.1});
  CHECK(!q.reachable || std::isfinite(q.femur));
}

XFAIL_TEST(gait_generator_rejects_nan_command,
           "GaitGenerator::set_target(NaN) poisons every foot target with NaN")
{
  GaitParams p;
  GaitGenerator g(p);
  g.set_body_height(p.stand_height);
  g.set_target(std::nan(""), 0, 0);
  for (int k = 0; k < 100; ++k) {g.step(0.02);}
  CHECK(feet_finite(g));
}

// ------------------------------------------------------------- gait maths ---

TEST(stride_clamp_respected_all_legs)
{
  GaitParams p;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> uv(-1.0, 1.0), uw(-4.0, 4.0);
  double worst = 0;
  for (int trial = 0; trial < 60; ++trial) {
    GaitGenerator g(p);
    g.set_body_height(p.stand_height);
    g.set_target(uv(rng), uv(rng), uw(rng));
    for (int k = 0; k < 400; ++k) {
      g.step(0.02);
      for (int i = 0; i < kNumLegs; ++i) {
        Vec3 f = g.foot_body(i), n = g.neutral_foot(i);
        worst = std::max(worst, std::hypot(f.x - n.x, f.y - n.y));
      }
    }
  }
  INFO("largest foot excursion from neutral: %.1f mm (clamp %.1f mm)", worst * 1000, p.max_half_stride * 1000);
  CHECK(worst <= p.max_half_stride + 1e-6);
}

namespace
{
// Returns number of (height, reach) combos judged OK by the formula and how many of those
// still produced an unreachable foot target somewhere in the gait.
void reach_formula_sweep(bool include_min_reach, int & ok, int & violations, std::string & first)
{
  ok = 0; violations = 0;
  for (double h = 0.04; h <= 0.19; h += 0.01) {
    for (double nr = 0.10; nr <= 0.24; nr += 0.02) {
      GaitParams p; p.stand_height = h; p.neutral_reach = nr;
      const double f = p.geometry.femur_len, t = p.geometry.tibia_len, c = p.geometry.coxa_len;
      // README: femur + tibia > sqrt((neutral_reach + stride - coxa)^2 + stand_height^2)
      bool pass = (f + t) > std::sqrt(std::pow(nr + p.max_half_stride - c, 2) + h * h);
      if (include_min_reach) {
        // ...and the foot must stay outside the folded-leg hole at the inward end of the stride,
        // where the swing foot is also lifted by step_height (smallest vertical distance).
        const double zmin = std::max(0.0, h - p.step_height);
        pass = pass && std::abs(f - t) < std::sqrt(std::pow(nr - p.max_half_stride - c, 2) + zmin * zmin);
      }
      if (!pass) {continue;}
      ++ok;
      bool bad = false;
      for (const auto & s : kScenarios) {
        GaitGenerator g(p); g.set_body_height(h); g.set_target(s.vx, s.vy, s.wz);
        for (int k = 0; k < 300 && !bad; ++k) {g.step(0.02); bad = !all_reachable(g);}
        if (bad) {
          ++violations;
          if (first.empty()) {
            first = tf::fmt("stand_height=%.2f neutral_reach=%.2f (%s)", h, nr, s.name);
          }
          break;
        }
      }
    }
  }
}
}  // namespace

XFAIL_TEST(reach_formula_from_readme_is_sufficient,
           "README reach formula only checks MAX reach; low stand_height + short neutral_reach "
           "puts the foot inside the minimum-reach hole |femur-tibia|")
{
  int ok, bad; std::string first;
  reach_formula_sweep(false, ok, bad, first);
  INFO("%d combos passed the README formula, %d still unreachable (first: %s)", ok, bad, first.c_str());
  CHECK_MSG(bad == 0, "formula passed but gait unreachable at %s", first.c_str());
}

TEST(reach_envelope_check_with_min_and_max_is_sound)
{
  int ok, bad; std::string first;
  reach_formula_sweep(true, ok, bad, first);
  INFO("%d combos passed max+min envelope check, %d unreachable", ok, bad);
  CHECK(ok > 10);
  CHECK_MSG(bad == 0, "envelope check passed but gait unreachable at %s", first.c_str());
}

TEST(random_leg_dimensions_reachability_flag_is_honest)
{
  // Random (plausible) robots: whenever the gait claims every target reachable,
  // FK must agree; whenever it flags unreachable, the target really is outside.
  std::mt19937 rng(99);
  std::uniform_real_distribution<double> uc(0.03, 0.08), uf(0.05, 0.12), ut(0.08, 0.18),
    uh(0.05, 0.16), un(0.10, 0.22);
  int robots_ok = 0, robots_bad = 0;
  for (int n = 0; n < 300; ++n) {
    GaitParams p;
    p.geometry = {uc(rng), uf(rng), ut(rng)};
    p.stand_height = uh(rng); p.neutral_reach = un(rng);
    GaitGenerator g(p); g.set_body_height(p.stand_height); g.set_target(0.08, 0, 0.3);
    bool any_unreach = false;
    for (int k = 0; k < 120; ++k) {
      g.step(0.02);
      for (int i = 0; i < kNumLegs; ++i) {
        Vec3 f = body_to_leg(g.mounts()[i], g.foot_body(i));
        JointAngles q = leg_ik(p.geometry, f);
        if (q.reachable) {
          Vec3 r = leg_fk(p.geometry, q);
          CHECK_MSG(std::hypot(std::hypot(r.x - f.x, r.y - f.y), r.z - f.z) < 1e-6, "reachable but FK mismatch");
        } else {
          any_unreach = true;
          const double horiz = std::hypot(f.x, f.y) - p.geometry.coxa_len;
          const double d = std::hypot(horiz, f.z);
          CHECK_MSG(d >= p.geometry.femur_len + p.geometry.tibia_len - 1e-5 ||
                    d <= std::abs(p.geometry.femur_len - p.geometry.tibia_len) + 1e-5,
                    "flagged unreachable but d=%.4f is inside envelope", d);
        }
      }
    }
    (any_unreach ? robots_bad : robots_ok)++;
  }
  INFO("%d random robots fully reachable, %d with unreachable targets (flagged correctly)", robots_ok, robots_bad);
  CHECK(robots_ok > 0);
}

TEST(long_run_no_drift_no_nan)
{
  // 10,000 control ticks (200 s). After the velocity ramp the gait is periodic
  // with period cycle_time; positions one hour of cycles later must match.
  GaitParams p;
  GaitGenerator g(p);
  g.set_body_height(p.stand_height);
  g.set_target(0.07, 0.01, 0.1);
  const int per_cycle = static_cast<int>(std::lround(p.cycle_time / 0.02));
  std::array<Vec3, 6> ref{};
  const int k_ref = 3000, k_end = k_ref + per_cycle * 110;  // 110 cycles later, 9,600 ticks total
  for (int k = 0; k <= k_end; ++k) {
    g.step(0.02);
    CHECK_MSG(feet_finite(g), "non-finite foot at tick %d", k);
    CHECK_MSG(g.phase() >= 0.0 && g.phase() < 1.0, "phase %.6f out of range at tick %d", g.phase(), k);
    if (k == k_ref) {for (int i = 0; i < 6; ++i) {ref[i] = g.foot_body(i);}}
  }
  double worst = 0;
  for (int i = 0; i < 6; ++i) {
    Vec3 f = g.foot_body(i);
    worst = std::max(worst, std::hypot(std::hypot(f.x - ref[i].x, f.y - ref[i].y), f.z - ref[i].z));
  }
  INFO("foot position error after 110 further cycles: %.2e m", worst);
  CHECK(worst < 1e-6);
}

TEST(static_stability_margin_every_step)
{
  // Centre of mass (body centre) must stay inside the support polygon of the
  // feet on the ground, with margin, for every gait.
  GaitParams p;
  double worst = 1e9;
  const char * worst_name = "";
  for (const auto & s : kScenarios) {
    GaitGenerator g(p);
    g.set_body_height(p.stand_height);
    g.set_target(s.vx, s.vy, s.wz);
    for (int k = 0; k < 600; ++k) {
      g.step(0.02);
      std::vector<Vec3> down;
      for (int i = 0; i < kNumLegs; ++i) {
        if (g.foot_body(i).z <= -p.stand_height + 1e-6) {down.push_back(g.foot_body(i));}
      }
      REQUIRE(down.size() >= 3);
      // Distance from origin to the hull edge nearest it, via "all pairs" half-plane test
      // on the convex hull (<= 6 points -> brute force).
      double margin = 1e9;
      for (size_t a = 0; a < down.size(); ++a) {
        for (size_t b = 0; b < down.size(); ++b) {
          if (a == b) {continue;}
          const double ex = down[b].x - down[a].x, ey = down[b].y - down[a].y;
          bool is_edge = true;
          for (size_t c = 0; c < down.size(); ++c) {
            if (c == a || c == b) {continue;}
            if (ex * (down[c].y - down[a].y) - ey * (down[c].x - down[a].x) < -1e-12) {is_edge = false; break;}
          }
          if (!is_edge) {continue;}
          const double len = std::hypot(ex, ey);
          if (len < 1e-9) {continue;}
          const double d = (ex * (0 - down[a].y) - ey * (0 - down[a].x)) / len;  // signed, >0 inside (CCW hull edge)
          margin = std::min(margin, d);
        }
      }
      if (margin < worst) {worst = margin; worst_name = s.name;}
    }
  }
  INFO("smallest static stability margin: %.1f mm (%s)", worst * 1000, worst_name);
  CHECK_MSG(worst > 0.02, "stability margin only %.1f mm", worst * 1000);
}

TEST(body_velocity_recovered_from_stance_feet_matches_command)
{
  // Planted feet don't move in the world, so the body's motion is the negative of the
  // stance-foot motion. Least-squares fit (vx, vy, wz) and compare with the (clamped)
  // command. This checks the gait kinematics independent of any simulator.
  GaitParams p;
  const Scenario cases[] = {{"forward", 0.08, 0, 0}, {"reverse", -0.06, 0, 0}, {"strafe_l", 0, 0.06, 0},
    {"turn_l", 0, 0, 0.4}, {"turn_r", 0, 0, -0.4}, {"arc", 0.05, 0, 0.25}};
  for (const auto & c : cases) {
    GaitGenerator g(p);
    g.set_body_height(p.stand_height);
    g.set_target(c.vx, c.vy, c.wz);
    for (int k = 0; k < 400; ++k) {g.step(0.02);}  // settle ramp
    std::array<Vec3, 6> prev;
    for (int i = 0; i < 6; ++i) {prev[i] = g.foot_body(i);}
    double sx = 0, sy = 0, sw = 0; int n = 0;
    for (int k = 0; k < 360; ++k) {  // 6 full cycles
      g.step(0.02);
      // Normal equations for foot displacement d_i = -(v + w x r_i) dt  (small angle)
      double A[3][3] = {{0}}, b[3] = {0};
      int m = 0;
      for (int i = 0; i < 6; ++i) {
        Vec3 f = g.foot_body(i);
        const bool stance = f.z <= -p.stand_height + 1e-9 && prev[i].z <= -p.stand_height + 1e-9;
        if (stance) {
          const double dx = -(f.x - prev[i].x) / 0.02, dy = -(f.y - prev[i].y) / 0.02;
          // dx = vx - wz*y,  dy = vy + wz*x   (feet at positions f)
          const double rows[2][4] = {{1, 0, -f.y, dx}, {0, 1, f.x, dy}};
          for (auto & r : rows) {
            for (int a = 0; a < 3; ++a) {
              for (int bb = 0; bb < 3; ++bb) {A[a][bb] += r[a] * r[bb];}
              b[a] += r[a] * r[3];
            }
          }
          ++m;
        }
        prev[i] = f;
      }
      if (m < 3) {continue;}
      // solve 3x3 by Cramer's rule
      auto det = [](double M[3][3]) {
          return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
                 M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                 M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
        };
      const double D = det(A);
      if (std::abs(D) < 1e-12) {continue;}
      double sol[3];
      for (int col = 0; col < 3; ++col) {
        double Mc[3][3];
        for (int r = 0; r < 3; ++r) {for (int cc = 0; cc < 3; ++cc) {Mc[r][cc] = (cc == col) ? b[r] : A[r][cc];}}
        sol[col] = det(Mc) / D;
      }
      sx += sol[0]; sy += sol[1]; sw += sol[2]; ++n;
    }
    REQUIRE(n > 100);
    sx /= n; sy /= n; sw /= n;
    INFO("%-9s cmd (%.3f,%.3f,%.2f) -> recovered (%.4f,%.4f,%.3f)", c.name, c.vx, c.vy, c.wz, sx, sy, sw);
    CHECK_NEAR(sx, c.vx, 0.004 + 0.03 * std::abs(c.vx));
    CHECK_NEAR(sy, c.vy, 0.004 + 0.03 * std::abs(c.vy));
    CHECK_NEAR(sw, c.wz, 0.03 + 0.05 * std::abs(c.wz));
  }
}

TEST(left_right_mirror_symmetry_forward_gait)
{
  // Leg 0 (front-right) must be the mirror image of leg 1 (front-left) half a cycle
  // later, and the same for the other pairs. Catches sign errors in mounts/gait.
  GaitParams p;
  GaitGenerator g(p);
  g.set_body_height(p.stand_height);
  g.set_target(0.07, 0, 0);
  const int half = static_cast<int>(std::lround(p.cycle_time / 0.02 / 2.0));
  std::vector<std::array<Vec3, 6>> seq;
  for (int k = 0; k < 800; ++k) {
    g.step(0.02);
    std::array<Vec3, 6> f;
    for (int i = 0; i < 6; ++i) {f[i] = g.foot_body(i);}
    seq.push_back(f);
  }
  const int pairs[3][2] = {{0, 1}, {2, 3}, {4, 5}};
  for (auto & pr : pairs) {
    double worst = 0;
    for (int k = 300; k < 600; ++k) {
      Vec3 a = seq[k][pr[0]], b = seq[k + half][pr[1]];
      worst = std::max(worst, std::hypot(std::hypot(a.x - b.x, a.y + b.y), a.z - b.z));
    }
    CHECK_MSG(worst < 1e-6, "legs %d/%d not mirror images (err %.2e m)", pr[0], pr[1], worst);
  }
}

TEST(tripod_groups_are_balanced)
{
  int count[2] = {0, 0}, right[2] = {0, 0};
  auto m = default_mounts();
  for (int i = 0; i < kNumLegs; ++i) {
    const int gp = tripod_group(i);
    ++count[gp];
    if (m[i].y < 0) {++right[gp];}
  }
  CHECK(count[0] == 3 && count[1] == 3);
  // each group: 2 legs on one side, 1 on the other (classic tripod), never 3 on one side
  for (int gp = 0; gp < 2; ++gp) {CHECK(right[gp] == 1 || right[gp] == 2);}
  // and every group's feet at neutral enclose the body centre
  GaitParams p;
  GaitGenerator g(p);
  for (int gp = 0; gp < 2; ++gp) {
    std::vector<Vec3> pts;
    for (int i = 0; i < kNumLegs; ++i) {if (tripod_group(i) == gp) {pts.push_back(g.neutral_foot(i));}}
    auto cross = [&](Vec3 a, Vec3 b) {return a.x * b.y - a.y * b.x;};
    const double s0 = cross({pts[1].x - pts[0].x, pts[1].y - pts[0].y, 0}, {-pts[0].x, -pts[0].y, 0});
    const double s1 = cross({pts[2].x - pts[1].x, pts[2].y - pts[1].y, 0}, {-pts[1].x, -pts[1].y, 0});
    const double s2 = cross({pts[0].x - pts[2].x, pts[0].y - pts[2].y, 0}, {-pts[2].x, -pts[2].y, 0});
    CHECK_MSG((s0 > 0 && s1 > 0 && s2 > 0) || (s0 < 0 && s1 < 0 && s2 < 0), "group %d triangle misses the body centre", gp);
  }
}

TEST(sit_to_stand_neutral_pose_reachable_at_every_height)
{
  GaitParams p;
  for (double h = p.sit_height; h <= p.stand_height + 1e-9; h += 0.001) {
    GaitGenerator g(p);
    g.set_body_height(h);
    g.step(0.02);
    CHECK_MSG(all_reachable(g), "neutral pose unreachable at body height %.3f", h);
  }
}

TEST(returns_to_neutral_and_idles_after_every_scenario)
{
  GaitParams p;
  for (const auto & s : kScenarios) {
    GaitGenerator g(p);
    g.set_body_height(p.stand_height);
    g.set_target(s.vx, s.vy, s.wz);
    for (int k = 0; k < 300; ++k) {g.step(0.02);}
    g.set_target(0, 0, 0);
    int ticks = 0;
    while (g.is_moving() && ticks < 1000) {g.step(0.02); ++ticks;}
    CHECK_MSG(!g.is_moving(), "%s: still moving 20 s after stop", s.name);
    INFO("%-9s idle %.2f s after stop", s.name, ticks * 0.02);
    CHECK_MSG(ticks * 0.02 < 6.0, "%s: took %.1f s to settle", s.name, ticks * 0.02);
    for (int i = 0; i < kNumLegs; ++i) {
      Vec3 f = g.foot_body(i), n = g.neutral_foot(i);
      CHECK(std::hypot(std::hypot(f.x - n.x, f.y - n.y), f.z - n.z) < 1e-3);
    }
  }
}

int main(int argc, char ** argv) {return tf::run_main(argc, argv, "gait_extended");}
