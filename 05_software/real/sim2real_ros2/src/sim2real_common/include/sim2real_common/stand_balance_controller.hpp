#pragma once

#include <array>
#include <vector>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sim2real_common
{

class StandBalanceController
{
public:
  StandBalanceController(double control_dt = 0.02)
  : control_dt_(control_dt)
  {
    profile_h_ = {0.157f, 0.248f, 0.311f, 0.366f, 0.411f, 0.448f};
    profile_hip_ = {1.5f, 1.2f, 1.0f, 0.8f, 0.6f, 0.4f};
    profile_knee_ = {-2.5f, -2.1f, -1.8f, -1.5f, -1.2f, -0.9f};
    reset();
  }

  void reset()
  {
    stable_time_ = 0.0f;
  }

  std::array<float, 16> computeTarget(
    const std::array<float, 3>& projected_gravity,
    const std::array<float, 3>& imu_gyro,
    const std::array<float, 3>& cmd)
  {
    float hip_base = 0.9f;
    float knee_base = -1.8f;
    estimateBaseLegPose(hip_base, knee_base);

    float roll = 0.0f;
    float pitch = 0.0f;
    estimateRollPitch(projected_gravity, roll, pitch);

    float roll_rate = imu_gyro[0];
    float pitch_rate = imu_gyro[1];

    float roll_corr = -kp_roll_ * roll - kd_roll_rate_ * roll_rate;

    float lateral_lean = lateral_lean_gain_ * cmd[1];

    std::array<float, 16> target{};
    for (int leg_idx = 0; leg_idx < 4; ++leg_idx) {
      float side = (leg_idx == 0 || leg_idx == 2) ? 1.0f : -1.0f;

      target[leg_idx * 3 + 0] = std::clamp(side * roll_corr + lateral_lean, -hip_abduction_clip_, hip_abduction_clip_);
      target[leg_idx * 3 + 1] = std::clamp(hip_base, hip_pitch_clip_[0], hip_pitch_clip_[1]);
      target[leg_idx * 3 + 2] = std::clamp(knee_base, knee_clip_[0], knee_clip_[1]);
    }
    // wheels 0
    target[12] = target[13] = target[14] = target[15] = 0.0f;

    bool stable = (std::abs(roll * 180.0f / static_cast<float>(M_PI)) <= stable_roll_deg_) &&
                  (std::abs(pitch * 180.0f / static_cast<float>(M_PI)) <= stable_pitch_deg_) &&
                  (std::max(std::abs(roll_rate * 180.0f / static_cast<float>(M_PI)), std::abs(pitch_rate * 180.0f / static_cast<float>(M_PI))) <= stable_gyro_deg_s_);

    stable_time_ = stable ? (stable_time_ + static_cast<float>(control_dt_)) : 0.0f;

    return target;
  }

  bool isStable() const
  {
    return stable_time_ >= enter_hold_s_;
  }

private:
  void estimateRollPitch(const std::array<float, 3>& projected_gravity, float& roll, float& pitch)
  {
    float gx = projected_gravity[0];
    float gy = projected_gravity[1];
    float gz = projected_gravity[2];
    roll = std::atan2(-gy, std::max(1e-6f, -gz));
    pitch = std::atan2(gx, std::sqrt(std::max(1e-6f, gy * gy + gz * gz)));
  }

  void estimateBaseLegPose(float& hip, float& knee)
  {
    float h_clamp = std::clamp(height_, profile_h_.front(), profile_h_.back());
    hip = interpolate(h_clamp, profile_h_, profile_hip_);
    knee = interpolate(h_clamp, profile_h_, profile_knee_);
  }

  float interpolate(float x, const std::vector<float>& xp, const std::vector<float>& fp)
  {
    if (x <= xp.front()) return fp.front();
    if (x >= xp.back()) return fp.back();
    for (std::size_t i = 0; i < xp.size() - 1; ++i) {
      if (x >= xp[i] && x <= xp[i+1]) {
        float f = (x - xp[i]) / (xp[i+1] - xp[i]);
        return fp[i] + f * (fp[i+1] - fp[i]);
      }
    }
    return fp.back();
  }

  double control_dt_;
  float height_{0.33f};
  float kp_roll_{0.85f};
  float kd_roll_rate_{0.03f};
  float lateral_lean_gain_{0.0f};
  float hip_abduction_clip_{0.45f};
  std::array<float, 2> hip_pitch_clip_{-1.0f, 2.5f};
  std::array<float, 2> knee_clip_{-2.6f, -0.3f};
  float stable_roll_deg_{6.0f};
  float stable_pitch_deg_{8.0f};
  float stable_gyro_deg_s_{45.0f};
  float enter_hold_s_{1.0f};

  std::vector<float> profile_h_;
  std::vector<float> profile_hip_;
  std::vector<float> profile_knee_;

  float stable_time_{0.0f};
};

} // namespace sim2real_common
