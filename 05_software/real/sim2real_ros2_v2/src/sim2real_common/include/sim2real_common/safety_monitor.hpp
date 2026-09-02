#pragma once

#include <array>
#include <string>
#include <cmath>
#include <algorithm>

namespace sim2real_common
{

enum class SafetyLevel : int {
  NORMAL = 0,
  CLIP = 1,
  BRAKE = 2,
  ESTOP = 3
};

struct SafetyDecision {
  SafetyLevel level{SafetyLevel::NORMAL};
  std::string message;
  std::array<float, 16> clipped_target{};
};

class SafetyMonitor {
public:
  SafetyMonitor(
    float max_target_offset = 0.6f,
    float max_ang_vel = 10.0f,
    float max_tilt_z = -0.3f,
    int clip_to_brake = 0,
    float hard_target_offset = 1.2f)
  : max_target_offset_(max_target_offset),
    max_ang_vel_(max_ang_vel),
    max_tilt_z_(max_tilt_z),
    clip_to_brake_(clip_to_brake),
    hard_target_offset_(hard_target_offset),
    consecutive_clips_(0)
  {}

  SafetyDecision check(
    const std::array<float, 16>& target_pose,
    const std::array<float, 16>& default_pose,
    const std::array<float, 3>& imu_gyro,
    const std::array<float, 3>& projected_gravity,
    bool estop_triggered)
  {
    SafetyDecision decision;
    decision.clipped_target = target_pose;

    if (estop_triggered) {
      decision.level = SafetyLevel::ESTOP;
      decision.message = "user E-stop";
      return decision;
    }

    // Tilt check (g_z should be ~ -1.0, if it is > max_tilt_z e.g. -0.3, it is tilted)
    if (projected_gravity[2] > max_tilt_z_) {
      decision.level = SafetyLevel::BRAKE;
      decision.message = "tilt detected: g_z=" + std::to_string(projected_gravity[2]);
      return decision;
    }

    // Angular velocity norm check
    float ang_vel_norm = std::sqrt(imu_gyro[0] * imu_gyro[0] + imu_gyro[1] * imu_gyro[1] + imu_gyro[2] * imu_gyro[2]);
    if (ang_vel_norm > max_ang_vel_) {
      decision.level = SafetyLevel::BRAKE;
      decision.message = "angular velocity overflow: |w|=" + std::to_string(ang_vel_norm);
      return decision;
    }

    // Offset check
    bool needs_clip = false;
    float max_offset = 0.0f;
    for (std::size_t i = 0; i < 12; ++i) { // check leg joint offsets from default pose
      float offset = target_pose[i] - default_pose[i];
      max_offset = std::max(max_offset, std::abs(offset));
      if (std::abs(offset) > max_target_offset_) {
        needs_clip = true;
        float clipped_val = std::clamp(offset, -max_target_offset_, max_target_offset_);
        decision.clipped_target[i] = default_pose[i] + clipped_val;
      }
    }

    if (needs_clip) {
      consecutive_clips_++;
      if (hard_target_offset_ > 0.0f && max_offset > hard_target_offset_) {
        decision.level = SafetyLevel::BRAKE;
        decision.message = "target leg offset exceeds hard limit: " + std::to_string(max_offset);
        return decision;
      }
      if (clip_to_brake_ > 0 && consecutive_clips_ >= clip_to_brake_) {
        decision.level = SafetyLevel::BRAKE;
        decision.message = "clipped " + std::to_string(consecutive_clips_) + " frames in a row";
        return decision;
      }
      decision.level = SafetyLevel::CLIP;
      decision.message = "target leg offset out of range";
      return decision;
    }

    consecutive_clips_ = 0;
    decision.level = SafetyLevel::NORMAL;
    return decision;
  }

  void reset() {
    consecutive_clips_ = 0;
  }

private:
  float max_target_offset_;
  float max_ang_vel_;
  float max_tilt_z_;
  int clip_to_brake_;
  float hard_target_offset_;
  int consecutive_clips_;
};

} // namespace sim2real_common
