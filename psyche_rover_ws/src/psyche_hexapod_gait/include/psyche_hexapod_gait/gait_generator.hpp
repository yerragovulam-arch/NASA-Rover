#ifndef PSYCHE_HEXAPOD_GAIT__GAIT_GENERATOR_HPP_
#define PSYCHE_HEXAPOD_GAIT__GAIT_GENERATOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>

#include "psyche_hexapod_gait/hexapod_kinematics.hpp"

namespace psyche_hexapod_gait
{

struct GaitParams
{
  LegGeometry geometry;
  double body_half_len{0.100};
  double body_half_width{0.060};

  double stand_height{0.120};       // body height above ground when standing (m)
  double sit_height{0.035};         // body height when sitting / powered down (m)
  double neutral_reach{0.160};      // foot distance from coxa axis, along the leg (m)
  double step_height{0.040};        // swing-foot lift (m)
  double cycle_time{1.2};           // s per full tripod cycle
  double duty{0.5};                 // fraction of cycle a leg is on the ground (tripod = 0.5)
  double max_half_stride{0.040};    // clamp on foot travel either side of neutral (m)
  double max_linear{0.08};          // m/s  (what a stride of max_half_stride gives)
  double max_angular{0.50};         // rad/s
  double linear_accel{0.20};        // m/s^2 smoothing on commanded velocity
  double angular_accel{1.0};        // rad/s^2
};

// Pure (ROS-free) tripod gait generator. Call set_target() with the desired
// body velocity, then step(dt) at a fixed rate and read foot_body() for each
// leg's foot position in the body frame.
class GaitGenerator
{
public:
  explicit GaitGenerator(const GaitParams & p)
  : p_(p), mounts_(default_mounts(p.body_half_len, p.body_half_width))
  {
    for (int i = 0; i < kNumLegs; ++i) {
      neutral_[i] = neutral_foot(i);
    }
  }

  void set_target(double vx, double vy, double wz)
  {
    // Scale so no leg's stride exceeds the clamp, preserving direction.
    tvx_ = vx; tvy_ = vy; twz_ = wz;
    const double s = stride_scale(vx, vy, wz);
    tvx_ *= s; tvy_ *= s; twz_ *= s;
  }

  void set_body_height(double h) {body_height_ = h;}

  bool is_moving() const {return moving_;}
  double phase() const {return phase_;}

  void step(double dt)
  {
    // Smooth commanded velocity.
    vx_ = approach(vx_, tvx_, p_.linear_accel * dt);
    vy_ = approach(vy_, tvy_, p_.linear_accel * dt);
    wz_ = approach(wz_, twz_, p_.angular_accel * dt);

    const bool want_motion =
      std::abs(tvx_) + std::abs(tvy_) + std::abs(twz_) > 1e-4;
    const bool still_ramping =
      std::abs(vx_) + std::abs(vy_) + std::abs(wz_) > 1e-3;

    if (want_motion || still_ramping) {
      moving_ = true;
    }

    if (moving_) {
      const double dphase = dt / p_.cycle_time;
      const double prev = phase_;
      phase_ += dphase;
      // Detect crossing of a tripod swap boundary (0 or 0.5).
      const bool crossed_half = prev < 0.5 && phase_ >= 0.5;
      const bool crossed_full = phase_ >= 1.0;
      if (crossed_full) {phase_ -= 1.0;}
      if (!want_motion && !still_ramping && (crossed_half || crossed_full)) {
        // Velocity has decayed to ~0 and we're at a swap boundary: all
        // feet are on the ground at neutral. Safe to idle.
        moving_ = false;
        phase_ = 0.0;
      }
    }

    for (int i = 0; i < kNumLegs; ++i) {
      foot_[i] = compute_foot(i);
    }
  }

  // Foot tip in body frame (m).
  Vec3 foot_body(int leg) const {return foot_[leg];}
  const std::array<LegMount, kNumLegs> & mounts() const {return mounts_;}
  const GaitParams & params() const {return p_;}

  // Neutral foot position (at current body height) in body frame.
  Vec3 neutral_foot(int leg) const
  {
    Vec3 f{p_.neutral_reach, 0.0, 0.0};
    Vec3 b = leg_to_body(mounts_[leg], f);
    b.z = -body_height_;
    return b;
  }

private:
  static double approach(double cur, double tgt, double max_delta)
  {
    return cur + std::clamp(tgt - cur, -max_delta, max_delta);
  }

  // Half-stride vector for a leg: foot travel relative to neutral at
  // stance start. A = (v + w x r) * Ts / 2.
  void half_stride(int leg, double vx, double vy, double wz, double & ax, double & ay) const
  {
    const double rx = neutral_[leg].x;
    const double ry = neutral_[leg].y;
    const double Ts = p_.duty * p_.cycle_time;
    ax = (vx - wz * ry) * Ts * 0.5;
    ay = (vy + wz * rx) * Ts * 0.5;
  }

  double stride_scale(double vx, double vy, double wz) const
  {
    double worst = 0.0;
    for (int i = 0; i < kNumLegs; ++i) {
      double ax, ay;
      half_stride(i, vx, vy, wz, ax, ay);
      worst = std::max(worst, std::hypot(ax, ay));
    }
    if (worst <= p_.max_half_stride || worst < 1e-9) {return 1.0;}
    return p_.max_half_stride / worst;
  }

  Vec3 compute_foot(int leg) const
  {
    // Tripod: group 1 runs half a cycle behind group 0.
    double ph = phase_ + (tripod_group(leg) == 0 ? 0.0 : 0.5);
    ph -= std::floor(ph);

    // Swing for the first (1 - duty) of the cycle, then stance.
    const double swing_frac = 1.0 - p_.duty;
    double h, lift;
    if (ph < swing_frac) {
      const double s = ph / swing_frac;
      const double u = 0.5 - 0.5 * std::cos(M_PI * s);  // eased 0..1
      h = -1.0 + 2.0 * u;                                 // back -> front
      lift = moving_ ? p_.step_height * std::sin(M_PI * s) : 0.0;
    } else {
      const double s = (ph - swing_frac) / p_.duty;
      h = 1.0 - 2.0 * s;                                  // front -> back
      lift = 0.0;
    }

    double ax, ay;
    half_stride(leg, vx_, vy_, wz_, ax, ay);

    const Vec3 n = neutral_foot(leg);
    return {n.x + h * ax, n.y + h * ay, n.z + lift};
  }

  GaitParams p_;
  std::array<LegMount, kNumLegs> mounts_;
  std::array<Vec3, kNumLegs> neutral_;
  std::array<Vec3, kNumLegs> foot_{};

  double body_height_{0.120};
  double phase_{0.0};
  bool moving_{false};
  double vx_{0}, vy_{0}, wz_{0};
  double tvx_{0}, tvy_{0}, twz_{0};
};

}  // namespace psyche_hexapod_gait

#endif  // PSYCHE_HEXAPOD_GAIT__GAIT_GENERATOR_HPP_
