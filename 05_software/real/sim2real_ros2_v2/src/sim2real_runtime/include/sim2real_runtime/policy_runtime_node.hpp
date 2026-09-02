#pragma once

#include <array>
#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sim2real_interfaces/msg/runtime_state.hpp"
#include "sim2real_interfaces/msg/runtime_target.hpp"
#include "sim2real_common/stand_balance_controller.hpp"
#include "sim2real_common/safety_monitor.hpp"
#include "sim2real_common/runtime_guard.hpp"

// ONNXRuntime C++ API
#include <onnxruntime_cxx_api.h>

namespace sim2real_runtime
{

class PolicyRuntimeNode : public rclcpp::Node
{
public:
  PolicyRuntimeNode();

private:
  void onState(const sim2real_interfaces::msg::RuntimeState::SharedPtr msg);
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onCmdVelStamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void applyCmdVel(float vx, float vy, float vyaw);
  void onPolicyLoop();

  std::array<float, 53> buildObservation(
    const sim2real_interfaces::msg::RuntimeState & state,
    const std::array<float, 3> & cmd,
    const std::array<float, 16> & last_actions) const;

  std::array<float, 16> runPolicy(const std::array<float, 53> & obs);
  bool isZeroCommand(const std::array<float, 3> & cmd, const std::array<float, 3> & imu_gyro) const;
  bool isCommandActive(const std::array<float, 3> & cmd) const;

  rclcpp::Publisher<sim2real_interfaces::msg::RuntimeTarget>::SharedPtr target_pub_;
  rclcpp::Subscription<sim2real_interfaces::msg::RuntimeState>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_sub_;
  rclcpp::TimerBase::SharedPtr policy_timer_;

  std::mutex mutex_;
  sim2real_interfaces::msg::RuntimeState latest_state_;
  bool has_state_{false};
  std::chrono::steady_clock::time_point last_state_recv_time_{};
  std::array<float, 3> cmd_{{0.0f, 0.0f, 0.0f}};
  std::array<float, 3> raw_cmd_{{0.0f, 0.0f, 0.0f}};
  std::array<float, 16> last_actions_{};
  std::uint32_t sequence_{0};

  // Startup State Machine
  enum class StartupState {
    BOOT_HOLD,
    STARTUP_SOFT_HOLD,
    STARTUP_TRANSITION,
    STARTUP_HOLD_AFTER,
    RUNTIME
  };
  StartupState startup_state_{StartupState::BOOT_HOLD};
  std::array<float, 16> start_pose_{};
  std::array<float, 16> startup_delta_{};
  rclcpp::Time state_start_time_{0, 0, RCL_ROS_TIME};
  double transition_time_{4.0};
  double hold_time_{1.0};
  std::unique_ptr<sim2real_common::StandBalanceController> stand_balance_;

  // ONNX Runtime members
  std::string model_path_{"policies/model_rough.onnx"};
  bool use_cuda_{false};  // enable CUDA Execution Provider on Orin Nano
  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::MemoryInfo> memory_info_;
  
  std::vector<std::string> input_names_str_;
  std::vector<std::string> output_names_str_;
  std::vector<const char*> input_names_char_;
  std::vector<const char*> output_names_char_;
  
  std::vector<std::int64_t> input_shape_;
  std::vector<std::int64_t> output_shape_;

  // Command filter and release states
  std::array<float, 3> filtered_cmd_{{0.0f, 0.0f, 0.0f}};
  float release_alpha_{0.0f};
  float command_release_s_{0.35f};
  float release_command_hold_s_{0.12f};
  float release_posture_max_err_{0.35f};
  float release_target_blend_s_{0.30f};
  float clip_obs_{100.0f};
  bool hold_zero_command_pose_{true};
  bool enable_zero_cmd_suppression_{true};
  bool require_active_command_to_release_{true};
  bool zero_cmd_use_yaw_rate_{false};
  bool runtime_released_{false};
  float release_active_time_{0.0f};
  float zero_cmd_lin_thresh_{0.05f};
  float zero_cmd_yaw_thresh_{0.05f};
  float zero_yaw_rate_thresh_{0.10f};

  // E-stop and Safety variables
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  std::atomic<bool> estop_triggered_{false};
  std::atomic<bool> safety_enabled_{true};
  std::atomic<bool> safety_triggered_{false};
  std::string safety_reason_{""};
  std::unique_ptr<sim2real_common::SafetyMonitor> safety_monitor_;
  std::unique_ptr<sim2real_common::RuntimeGuard> runtime_guard_;

  void onEstop(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace sim2real_runtime
