#pragma once

#include <array>
#include <cstddef>

namespace sim2real_common
{

struct DeploymentContract
{
  static constexpr std::size_t kObsDim = 53;
  static constexpr std::size_t kActionDim = 16;
  static constexpr std::size_t kLegJointCount = 12;
  static constexpr std::size_t kWheelCount = 4;
  static constexpr double kPolicyHz = 50.0;
  static constexpr double kMotorHz = 200.0;
  static constexpr double kStatusHz = 10.0;

  static constexpr std::array<int, 4> kWheelIndices = {12, 13, 14, 15};
  static constexpr float kLegKp = 50.0f;
  static constexpr float kLegKd = 1.5f;
  static constexpr float kLegHoldKp = kLegKp;
  static constexpr float kLegHoldKd = kLegKd;
  static constexpr float kWheelKd = 1.0f;

  static constexpr std::array<int, 16> kCanBusMap = {
    1, 1, 1, // fl legs
    1, 1, 1, // fr legs
    2, 2, 2, // rl legs
    2, 2, 2, // rr legs
    1, 1, 2, 2 // wheels: fl, fr, rl, rr
  };

  static constexpr std::array<int, 16> kCanIdMap = {
    1, 2, 3, // fl legs
    5, 6, 7, // fr legs
    1, 2, 3, // rl legs
    5, 6, 7, // rr legs
    4, 8, 4, 8 // wheels: fl, fr, rl, rr
  };

  static constexpr std::array<float, 16> kDirectionMap = {
    -1.0f, -1.0f, -1.0f, // fl
    -1.0f,  1.0f,  1.0f, // fr
     1.0f, -1.0f, -1.0f, // rl
     1.0f,  1.0f,  1.0f, // rr
    -1.0f,  1.0f, -1.0f,  1.0f  // wheels
  };

  static constexpr std::array<float, 16> kZeroOffsetMap = {
     0.003f,  0.030f,  0.028f, // fl
     0.004f,  0.038f,  0.011f, // fr
     0.019f, -0.034f,  0.025f, // rl
    -0.001f,  0.039f,  0.018f, // rr
     0.000f,  0.000f,  0.000f,  0.000f  // wheels
  };

  static constexpr std::array<float, 16> kActionScale = {
    0.125f, 0.25f, 0.25f,
    0.125f, 0.25f, 0.25f,
    0.125f, 0.25f, 0.25f,
    0.125f, 0.25f, 0.25f,
    5.0f, 5.0f, 5.0f, 5.0f
  };

  static constexpr std::array<float, 16> kDefaultDofPos = {
    0.0f, 0.550f, -1.125f,
    0.0f, 0.550f, -1.125f,
    0.0f, 0.550f, -1.125f,
    0.0f, 0.550f, -1.125f,
    0.0f, 0.0f, 0.0f, 0.0f
  };
};

static constexpr std::array<const char *, 16> kJointLabels = {
  "fl_hip_abduction",
  "fl_hip_pitch",
  "fl_knee",
  "fr_hip_abduction",
  "fr_hip_pitch",
  "fr_knee",
  "rl_hip_abduction",
  "rl_hip_pitch",
  "rl_knee",
  "rr_hip_abduction",
  "rr_hip_pitch",
  "rr_knee",
  "fl_wheel",
  "fr_wheel",
  "rl_wheel",
  "rr_wheel"
};

}  // namespace sim2real_common
