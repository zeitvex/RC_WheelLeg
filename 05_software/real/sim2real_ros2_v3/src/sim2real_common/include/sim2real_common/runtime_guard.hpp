#pragma once

#include <array>
#include <string>
#include <vector>
#include <cmath>

namespace sim2real_common
{

enum class GuardLevel : int {
  OK = 0,
  WARN = 1,
  STOP = 2
};

struct GuardDecision {
  GuardLevel level{GuardLevel::OK};
  std::string reason;
};

class RuntimeGuard {
public:
  RuntimeGuard(
    float max_ang_vel = 12.0f,
    float max_tilt_z = -0.30f,
    float imu_age_warn_ms = 60.0f,
    float imu_age_stop_ms = 200.0f)
  : max_ang_vel_(max_ang_vel),
    max_tilt_z_(max_tilt_z),
    imu_age_warn_ms_(imu_age_warn_ms),
    imu_age_stop_ms_(imu_age_stop_ms)
  {}

  GuardDecision check(
    const std::array<float, 3>& imu_gyro,
    const std::array<float, 3>& projected_gravity,
    float imu_age_ms,
    bool estop_triggered,
    const std::vector<float>& extra_vals = {})
  {
    GuardDecision decision;

    // 1) user E-stop
    if (estop_triggered) {
      decision.level = GuardLevel::STOP;
      decision.reason = "user E-stop";
      return decision;
    }

    // 2) NaN/Inf check
    for (float v : imu_gyro) {
      if (std::isnan(v) || std::isinf(v)) {
        decision.level = GuardLevel::STOP;
        decision.reason = "NaN/Inf detected in imu_gyro";
        return decision;
      }
    }
    for (float v : projected_gravity) {
      if (std::isnan(v) || std::isinf(v)) {
        decision.level = GuardLevel::STOP;
        decision.reason = "NaN/Inf detected in projected_gravity";
        return decision;
      }
    }
    for (float v : extra_vals) {
      if (std::isnan(v) || std::isinf(v)) {
        decision.level = GuardLevel::STOP;
        decision.reason = "NaN/Inf detected in checked values";
        return decision;
      }
    }

    // 3) IMU stale
    if (imu_age_ms > imu_age_stop_ms_) {
      decision.level = GuardLevel::STOP;
      decision.reason = "IMU stale " + std::to_string(imu_age_ms) + "ms";
      return decision;
    }
    bool warned_imu = (imu_age_ms > imu_age_warn_ms_);

    // 4) Tilt check
    if (projected_gravity[2] > max_tilt_z_) {
      decision.level = GuardLevel::STOP;
      decision.reason = "tilt: g_z=" + std::to_string(projected_gravity[2]);
      return decision;
    }

    // 5) Angular velocity check
    float ang_norm = std::sqrt(imu_gyro[0] * imu_gyro[0] + imu_gyro[1] * imu_gyro[1] + imu_gyro[2] * imu_gyro[2]);
    if (ang_norm > max_ang_vel_) {
      decision.level = GuardLevel::STOP;
      decision.reason = "ang_vel overflow: |w|=" + std::to_string(ang_norm);
      return decision;
    }

    if (warned_imu) {
      decision.level = GuardLevel::WARN;
      decision.reason = "IMU age " + std::to_string(imu_age_ms) + "ms";
      return decision;
    }

    decision.level = GuardLevel::OK;
    return decision;
  }

private:
  float max_ang_vel_;
  float max_tilt_z_;
  float imu_age_warn_ms_;
  float imu_age_stop_ms_;
};

} // namespace sim2real_common
