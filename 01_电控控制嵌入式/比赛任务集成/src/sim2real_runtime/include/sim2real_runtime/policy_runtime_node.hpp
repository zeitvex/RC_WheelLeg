#pragma once

#include <array>
#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <atomic>
#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sim2real_interfaces/msg/runtime_state.hpp"
#include "sim2real_interfaces/msg/runtime_target.hpp"
#include "sim2real_common/event_logger.hpp"
#include "sim2real_common/stand_balance_controller.hpp"
#include "sim2real_common/safety_monitor.hpp"
#include "sim2real_common/runtime_guard.hpp"

// ONNXRuntime C++ API
#include <onnxruntime_cxx_api.h>

#ifdef SIM2REAL_RUNTIME_HAS_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#endif

namespace sim2real_runtime
{

class PolicyRuntimeNode : public rclcpp::Node
{
public:
  PolicyRuntimeNode();
  ~PolicyRuntimeNode() override;

private:
  enum class InferenceBackend {
    None,
    TensorRT,
    OnnxRuntime,
  };

  enum class ModelMode {
    Rough,
    Crawl,
    Wall,
  };

  enum class CrawlBackend {
    Ik,
    Rl,
  };

  enum class ModelSwitchState {
    Idle,
    ToStand,
    StandHold,
    ToModelPose,
  };

  enum class StartupState {
    BOOT_HOLD,
    STARTUP_SOFT_HOLD,
    STARTUP_TRANSITION,
    STARTUP_HOLD_AFTER,
    RUNTIME
  };

  enum class PostureHoldMode {
    None,
    Keep,
    ReturnDefault,
  };

  void onState(const sim2real_interfaces::msg::RuntimeState::SharedPtr msg);
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onCmdVelStamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onModelSwitchCmd(const std_msgs::msg::String::SharedPtr msg);
  void onPostureCmd(const std_msgs::msg::String::SharedPtr msg);
  void applyCmdVel(float vx, float vy, float vyaw);
  void onPolicyLoop();
  bool initInferenceBackend();
  bool initTensorRt();
  bool initOnnxRuntime();
  void shutdownOnnxRuntime();
  void shutdownTensorRt();
  std::string deriveTensorRtEnginePath(const std::string & onnx_model_path) const;
  const std::array<float, 16> & defaultPoseForMode(ModelMode mode) const;
  const std::string & modelPathForMode(ModelMode mode) const;
  const std::string & modelEnginePathForMode(ModelMode mode) const;
  bool switchInferenceModel(ModelMode target_mode);
  void publishModelStatus();
  bool modeUsesInference(ModelMode mode) const;
  std::array<float, 16> computeHoldTarget(
    const sim2real_interfaces::msg::RuntimeState & state,
    const std::array<float, 3> & cmd);
  std::array<float, 16> computeIkCrawlTarget(
    const sim2real_interfaces::msg::RuntimeState & state,
    const std::array<float, 3> & cmd);
  float computeCrawlIkCommandScale(
    const sim2real_interfaces::msg::RuntimeState & state,
    const std::array<float, 16> & leg_target) const;
  float projectedGravityTiltRad(const std::array<float, 3> & projected_gravity) const;
  const char * startupStateName(StartupState state) const;
  const char * modelModeName(ModelMode mode) const;
  const char * modelSwitchStateName(ModelSwitchState state) const;
  const char * postureHoldModeName(PostureHoldMode mode) const;
  const char * inferenceBackendName() const;
  const char * crawlBackendName() const;
  void initializeDebugTrace();
  void appendDebugTrace(
    const sim2real_interfaces::msg::RuntimeState & state,
    const sim2real_interfaces::msg::RuntimeTarget & target);

  std::array<float, 53> buildObservation(
    const sim2real_interfaces::msg::RuntimeState & state,
    const std::array<float, 3> & cmd,
    const std::array<float, 16> & last_actions) const;

  std::array<float, 16> runPolicy(const std::array<float, 53> & obs);
  bool isZeroCommand(const std::array<float, 3> & cmd, const std::array<float, 3> & imu_gyro) const;
  bool isCommandActive(const std::array<float, 3> & cmd) const;
  void startPostureTransition(
    PostureHoldMode mode,
    const std::array<float, 16> & start_pose,
    const std::array<float, 16> & target_pose,
    const rclcpp::Time & now_time);

  rclcpp::Publisher<sim2real_interfaces::msg::RuntimeTarget>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr model_status_pub_;
  rclcpp::Subscription<sim2real_interfaces::msg::RuntimeState>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr model_switch_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posture_cmd_sub_;
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
  StartupState startup_state_{StartupState::BOOT_HOLD};
  std::array<float, 16> start_pose_{};
  std::array<float, 16> startup_delta_{};
  rclcpp::Time state_start_time_{0, 0, RCL_ROS_TIME};
  double transition_time_{4.0};
  double hold_time_{1.0};
  std::unique_ptr<sim2real_common::StandBalanceController> stand_balance_;

  ModelMode current_model_mode_{ModelMode::Rough};
  ModelMode requested_model_mode_{ModelMode::Rough};
  ModelMode loaded_model_mode_{ModelMode::Rough};
  CrawlBackend crawl_backend_{CrawlBackend::Ik};
  ModelSwitchState model_switch_state_{ModelSwitchState::Idle};
  bool model_switch_requested_{false};
  bool hold_active_model_pose_when_unreleased_{false};
  std::array<float, 16> rough_default_dof_pos_{};
  std::array<float, 16> crawl_default_dof_pos_{};
  std::array<float, 16> wall_default_dof_pos_{};
  std::array<float, 16> active_default_dof_pos_{};
  std::array<float, 16> safety_reference_dof_pos_{};
  std::array<float, 16> keep_pose_dof_pos_{};
  std::array<float, 16> posture_start_pose_{};
  std::array<float, 16> posture_target_pose_{};
  std::array<float, 16> posture_delta_{};
  PostureHoldMode posture_hold_mode_{PostureHoldMode::None};
  bool posture_transition_active_{false};
  rclcpp::Time posture_transition_start_time_{0, 0, RCL_ROS_TIME};
  double posture_transition_s_{0.8};
  std::array<float, 16> switch_start_pose_{};
  std::array<float, 16> switch_delta_{};
  rclcpp::Time model_switch_state_start_time_{0, 0, RCL_ROS_TIME};
  double model_switch_transition_s_{1.2};
  double model_switch_to_stand_transition_scale_{1.35};
  double model_switch_to_model_transition_scale_{1.55};
  double model_switch_min_transition_s_{0.35};
  double model_switch_stand_hold_s_{0.45};
  double model_switch_stand_max_err_{0.18};
  double model_switch_stand_max_vel_{0.8};
  double active_switch_transition_s_{1.2};

  std::string rough_model_path_{"policies/model_rough.onnx"};
  std::string rough_model_engine_path_{""};
  std::string crawl_model_path_{"policies/model_crawl.onnx"};
  std::string crawl_model_engine_path_{""};
  std::string wall_model_path_{"policies/model_wall.onnx"};
  std::string wall_model_engine_path_{""};
  float crawl_ik_wheel_linear_gain_{6.25f};
  float crawl_ik_wheel_yaw_gain_{4.0f};
  float crawl_ik_max_wheel_speed_{6.0f};
  float crawl_ik_abduction_clip_{0.45f};
  float crawl_ik_yaw_rate_kp_{0.0f};
  bool crawl_ik_imu_posture_{false};
  float crawl_ik_encoder_posture_kp_{0.0f};
  float crawl_ik_encoder_posture_max_{0.03f};
  bool crawl_ik_encoder_guard_{true};
  float crawl_ik_encoder_guard_start_{0.28f};
  float crawl_ik_encoder_guard_stop_{0.65f};
  bool crawl_ik_imu_guard_{true};
  float crawl_ik_imu_guard_start_rad_{0.20943952f};
  float crawl_ik_imu_guard_stop_rad_{0.48869219f};

  // ONNX Runtime members
  std::string model_path_{"policies/model_rough.onnx"};
  std::string model_engine_path_{""};
  bool prefer_tensorrt_{true};
  bool use_cuda_{false};  // enable CUDA Execution Provider on Orin Nano
  InferenceBackend inference_backend_{InferenceBackend::None};
  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::MemoryInfo> memory_info_;
  
  std::vector<std::string> input_names_str_;
  std::vector<std::string> output_names_str_;
  std::vector<const char*> input_names_char_;
  std::vector<const char*> output_names_char_;
  
  std::vector<std::int64_t> input_shape_;
  std::vector<std::int64_t> output_shape_;

#ifdef SIM2REAL_RUNTIME_HAS_TENSORRT
  nvinfer1::IRuntime * trt_runtime_{nullptr};
  nvinfer1::ICudaEngine * trt_engine_{nullptr};
  nvinfer1::IExecutionContext * trt_context_{nullptr};
  cudaStream_t trt_stream_{nullptr};
  void * trt_input_buffer_{nullptr};
  void * trt_output_buffer_{nullptr};
  std::string trt_input_name_;
  std::string trt_output_name_;
#endif

  // Command filter and release states
  std::array<float, 3> filtered_cmd_{{0.0f, 0.0f, 0.0f}};
  float runtime_max_vx_{1.0f};
  float runtime_max_vy_{0.3f};
  float runtime_max_yaw_rate_{1.0f};
  float release_alpha_{0.0f};
  float command_release_s_{0.35f};
  float release_command_hold_s_{0.12f};
  float release_posture_max_err_{0.35f};
  float release_target_blend_s_{0.30f};
  float model_switch_release_scale_{1.3f};
  float clip_obs_{100.0f};
  bool hold_zero_command_pose_{true};
  bool enable_zero_cmd_suppression_{true};
  bool require_active_command_to_release_{true};
  bool zero_cmd_use_yaw_rate_{false};
  bool runtime_released_{false};
  bool slow_release_after_model_switch_{false};
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
  sim2real_common::EventLogger event_logger_;
  std::string run_log_dir_;
  std::string debug_trace_path_;
  bool debug_trace_enabled_{true};
  std::uint32_t debug_trace_decimation_{1};
  std::uint32_t debug_trace_counter_{0};
  std::uint32_t protection_trigger_count_{0};
  std::uint32_t target_clip_count_{0};
  bool clip_active_logged_{false};
  std::unique_ptr<sim2real_common::SafetyMonitor> safety_monitor_;
  std::unique_ptr<sim2real_common::SafetyMonitor> model_switch_safety_monitor_;
  std::unique_ptr<sim2real_common::RuntimeGuard> runtime_guard_;

  void onEstop(const std_msgs::msg::Bool::SharedPtr msg);
  void logEvent(
    const std::string & level,
    const std::string & event,
    const std::string & message);
  void logProtectionEvent(
    const std::string & trigger,
    const std::string & reason,
    const std::string & action);
  void finalizeRunSummary();
};

}  // namespace sim2real_runtime
