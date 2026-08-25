#pragma once

#include <array>
#include <cmath>
#include <algorithm>

#include "sim2real_common/deployment_contract.hpp"

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
    reset();
  }

  void setNominalLegPose(float hip_pitch, float knee)
  {
    nominal_hip_pitch_ = hip_pitch;
    nominal_knee_ = knee;
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
    const float hip_base = nominal_hip_pitch_;
    const float knee_base = nominal_knee_;

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

  double control_dt_;
  float nominal_hip_pitch_{DeploymentContract::kDefaultDofPos[1]};
  float nominal_knee_{DeploymentContract::kDefaultDofPos[2]};
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

  float stable_time_{0.0f};
};

} // namespace sim2real_common
