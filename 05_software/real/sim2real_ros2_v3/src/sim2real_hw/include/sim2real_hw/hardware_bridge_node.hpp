#pragma once

#include <array>
#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sim2real_interfaces/msg/runtime_state.hpp"
#include "sim2real_interfaces/msg/runtime_target.hpp"
#include "sim2real_common/event_logger.hpp"
#include "sim2real_common/low_pass_filter.hpp"
#include "sim2real_common/mahony_filter.hpp"
#include "sim2real_common/safety_monitor.hpp"
#include "sim2real_common/runtime_guard.hpp"

struct can_frame;

namespace sim2real_hw
{

enum class RecoveryKind
{
  None,
  Stale,
  NoEffect
};

enum class RecoveryStage
{
  Idle,
  AwaitInitFeedback,
  AwaitEffectVerification
};

enum class ActiveModelMode
{
  Rough,
  Crawl,
  Wall
};

struct MotorConfig
{
  int bus; // 1 or 2
  int id;  // motor CAN id
  float direction;
  float offset;
};

struct MotorStateInternal
{
  float position{0.0f};
  float velocity{0.0f};
  float torque{0.0f};
  float temperature{0.0f};
  float bus_voltage{0.0f};
  float estimated_current_arms{0.0f};
  float last_command_sim{0.0f};
  std::uint32_t update_count{0};
  std::uint32_t stale_count{0};
  std::uint32_t command_active_count{0};
  std::uint32_t no_effect_count{0};
  std::uint16_t fault_code{0};
  std::uint16_t fault_detail_1{0};
  std::uint16_t fault_detail_2{0};
  // Hold-over state
  float last_valid_pos{0.0f};
  float last_valid_vel{0.0f};
  float last_valid_torque{0.0f};
  std::uint32_t prev_update_count{0};
  bool has_valid_data{false};
  bool has_bus_voltage{false};
  bool has_fault_snapshot{false};
  bool stale_reported{false};
  bool recovered_reported{false};
  bool disable_reported{false};
  bool command_effect_monitoring_active{false};
  bool no_effect_reported{false};
  bool high_temp_reported{false};
  bool high_current_reported{false};
  bool high_voltage_reported{false};
  bool low_voltage_reported{false};
  bool fault_code_reported{false};
  bool init_confirmed{false};
  std::uint32_t init_attempt_count{0};
  std::uint32_t recovery_attempt_count{0};
  std::uint32_t no_effect_recovery_attempt_count{0};
  std::chrono::steady_clock::time_point last_recovery_attempt_time_{};
  std::chrono::steady_clock::time_point last_no_effect_recovery_attempt_time_{};
  std::chrono::steady_clock::time_point last_diag_snapshot_time_{};
  std::chrono::steady_clock::time_point last_diag_request_time_{};
  std::chrono::steady_clock::time_point recovery_stage_deadline_{};
  std::uint32_t recovery_start_update_count{0};
  std::uint32_t recovery_active_attempt_number{0};
  RecoveryKind recovery_kind{RecoveryKind::None};
  RecoveryStage recovery_stage{RecoveryStage::Idle};
  std::string recovery_trigger;
  std::string last_power_event_reason;
};

class HardwareBridgeNode : public rclcpp::Node
{
public:
  HardwareBridgeNode();
  ~HardwareBridgeNode();

private:
  void onTarget(const sim2real_interfaces::msg::RuntimeTarget::SharedPtr msg);
  void onModelStatus(const std_msgs::msg::String::SharedPtr msg);
  void onReadLoop();
  void onWriteLoop();
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  bool initCan(const std::string& ifname, int& fd);
  bool sendCanFrame(int fd, std::uint32_t can_id, const std::uint8_t* data, std::uint8_t dlc);
  bool readCanFrame(int fd, void* frame, int timeout_us);

  bool enableMotor(int fd, int motor_id);
  bool disableMotor(int fd, int motor_id, bool clear_fault = false);
  bool writeParameterInt(int fd, int motor_id, std::uint16_t param_id, std::uint32_t value);
  bool setModeRaw(int fd, int motor_id, std::int8_t mode);
  bool readParameter(int fd, int motor_id, std::uint16_t param_id);
  bool writeLimit(int fd, int motor_id, std::uint16_t param_id, float limit);
  bool writeOperationFrame(int fd, int motor_id, double pos, double vel, double kp, double kd, double torque);
  bool initializeMotor(std::size_t index, const std::string & reason, int max_attempts = 3);
  bool initializeMotorsOnBus(int bus_id, const std::string & reason);
  bool waitForMotorFeedback(std::size_t index, std::chrono::milliseconds timeout);
  void processCanFrame(const struct can_frame & frame, int bus_id);
  void drainCanFrames(int fd, int bus_id, int timeout_us);
  bool isLegMotor(std::size_t index) const;
  bool isWheelMotor(std::size_t index) const;
  bool motorHasBlockingFault(std::size_t index) const;
  bool isNoEffectConditionPresent(std::size_t index) const;
  std::uint32_t noEffectCommandWarmupCycles(std::size_t index) const;
  std::uint32_t noEffectTriggerCycles(std::size_t index) const;
  std::uint32_t noEffectAttemptLimit(std::size_t index) const;
  std::uint32_t noEffectCooldownMs(std::size_t index) const;
  std::uint32_t noEffectVerifyTimeoutMs(std::size_t index) const;
  bool hasFreshNoEffectDiagnostics(std::size_t index) const;
  void requestMotorDiagnostics(std::size_t index);
  std::string classifyNoEffectSuspect(std::size_t index) const;
  std::string buildNoEffectSummary(std::size_t index) const;
  void updateMotorCommandTracking(std::size_t index, float sim_command, const std::string & target_source);
  void updateNoEffectDetection(std::size_t index);
  bool startMotorRecoverySequence(
    std::size_t index,
    const std::string & trigger,
    RecoveryKind kind,
    std::uint32_t attempt_number);
  void processMotorRecoverySequence(std::size_t index);
  void clearMotorRecoverySequence(std::size_t index);
  bool shouldAttemptMotorRecovery(std::size_t index) const;
  bool attemptMotorRecovery(std::size_t index, const std::string & trigger);
  bool shouldAttemptNoEffectRecovery(std::size_t index) const;
  bool attemptNoEffectRecovery(std::size_t index, const std::string & trigger);
  const char * jointName(std::size_t index) const;
  std::string motorTag(std::size_t index) const;
  float estimateCurrentArms(float torque_nm) const;
  std::string decodeFaultCode(std::uint16_t fault_code) const;
  std::string decodeFaultDetailRegister(std::uint16_t register_value, int register_index) const;
  std::string buildMotorFaultSummary(std::size_t index) const;
  std::string formatProtectionReason(const std::string & trigger, const std::string & reason) const;
  void logProtectionEvent(const std::string & trigger, const std::string & reason, const std::string & action);
  void logMotorPowerEvent(std::size_t index, const std::string & state, const std::string & reason);
  void logMotorDiagnosticEvent(std::size_t index, const std::string & event, const std::string & reason, const char * level = "WARN");
  void updateMotorTelemetry(std::size_t index, float pos_sim, float vel_sim, float torque_sim, float temperature_c);
  void handleParameterResponse(const struct can_frame & frame, int bus_id);
  void updateMotorDiagnostics(std::size_t index);
  void pollMotorDiagnostics();
  void finalizeRunSummary();

  rclcpp::Publisher<sim2real_interfaces::msg::RuntimeState>::SharedPtr state_pub_;
  rclcpp::Subscription<sim2real_interfaces::msg::RuntimeTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr model_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr write_timer_;
  rclcpp::CallbackGroup::SharedPtr motor_callback_group_;
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;

  std::mutex target_mutex_;
  std::array<float, 16> latest_target_{};
  std::array<float, 16> latest_raw_action_{};
  std::string latest_target_source_{"boot_hold"};
  rclcpp::Time latest_target_stamp_{0, 0, RCL_ROS_TIME};
  std::array<float, 16> rough_default_dof_pos_{};
  std::array<float, 16> crawl_default_dof_pos_{};
  std::array<float, 16> wall_default_dof_pos_{};
  std::array<float, 16> active_default_dof_pos_{};
  ActiveModelMode active_model_mode_{ActiveModelMode::Rough};
  bool model_switch_active_{false};
  std::uint32_t target_sequence_{0};
  std::uint32_t state_sequence_{0};
  double target_timeout_ms_{150.0};

  // SocketCAN file descriptors
  int can0_fd_{-1};
  int can1_fd_{-1};
  std::string can0_name_{"can0"};
  std::string can1_name_{"can1"};

  // CAN error recovery
  static constexpr int kCanErrorThreshold = 50;  // consecutive errors before reinit
  int can0_error_count_{0};
  int can1_error_count_{0};
  bool reinitCan(const std::string& ifname, int& fd, int& error_count);

  // Hold-over constants
  static constexpr std::uint32_t kHoldoverThreshold = 2;
  static constexpr std::uint32_t kMotorDropReportThreshold = 40;

  // Motor configurations and states
  std::array<MotorConfig, 16> motors_;
  std::array<MotorStateInternal, 16> motor_states_;

  // IMU state
  std::mutex imu_mutex_;
  std::array<float, 3> imu_gyro_{};
  std::array<float, 3> imu_accel_{};
  std::array<float, 3> projected_gravity_{0.0f, 0.0f, -1.0f};
  bool imu_fresh_{false};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_imu_recv_time_{};
  bool has_received_imu_{false};
  std::array<float, 3> imu_gravity_sum_{0.0f, 0.0f, 0.0f};
  std::uint32_t imu_gravity_sample_count_{0};
  static constexpr std::uint32_t kImuGravityAlignSamples = 50;

  // Odom state
  std::mutex odom_mutex_;
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  std::array<float, 3> odom_pos_{};
  std::array<float, 4> odom_quat_wxyz_{1.0f, 0.0f, 0.0f, 0.0f};
  std::array<float, 3> odom_linear_vel_{};
  std::array<float, 3> odom_angular_vel_{};
  bool odom_fresh_{false};

  // Filters and Estimators
  std::unique_ptr<sim2real_common::LowPassFilter> lpf_legs_;
  std::unique_ptr<sim2real_common::LowPassFilter> lpf_wheels_;
  std::unique_ptr<sim2real_common::MahonyFilter> mahony_filter_;
  std::unique_ptr<sim2real_common::SafetyMonitor> safety_monitor_;
  std::unique_ptr<sim2real_common::SafetyMonitor> model_switch_safety_monitor_;
  std::unique_ptr<sim2real_common::RuntimeGuard> runtime_guard_;
  std::atomic<bool> mahony_initialized_{false};
  rclcpp::Time last_read_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time startup_soft_hold_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diag_poll_time_{0, 0, RCL_ROS_TIME};
  std::size_t diag_poll_motor_index_{0};

  // Telemetry
  std::uint32_t fresh_count_{0};
  std::uint32_t holdover_count_{0};
  std::uint32_t stale_max_{0};
  std::uint32_t holdover_events_total_{0};
  std::uint32_t protection_trigger_count_{0};
  std::uint32_t motor_drop_event_count_{0};
  std::uint32_t motor_recover_event_count_{0};
  std::uint32_t motor_fault_event_count_{0};
  bool timeout_hold_logged_{false};
  bool clip_active_logged_{false};

  bool dry_run_{false};
  std::atomic<bool> estop_triggered_{false};
  std::atomic<bool> safety_enabled_{true};
  std::atomic<bool> safety_triggered_{false};
  std::string safety_reason_{""};
  sim2real_common::EventLogger event_logger_;
  std::string run_log_dir_;
  float motor_temp_warn_c_{100.0f};
  float motor_temp_fault_c_{135.0f};
  float motor_bus_overvoltage_v_{60.0f};
  float motor_bus_undervoltage_v_{12.0f};
  float motor_current_warn_arms_{10.5f};
  float motor_current_peak_arms_{14.0f};
  float motor_torque_warn_nm_{13.0f};
  float motor_torque_peak_nm_{17.0f};
  double diag_poll_period_s_{0.10};
  float wheel_no_effect_command_threshold_{1.0f};
  float wheel_no_effect_min_response_ratio_{0.20f};
  float wheel_no_effect_velocity_epsilon_{0.25f};
  float wheel_no_effect_max_temperature_c_{90.0f};
  float wheel_no_effect_min_bus_voltage_v_{18.0f};
  std::uint32_t wheel_no_effect_command_warmup_cycles_{12};
  std::uint32_t wheel_no_effect_trigger_cycles_{30};
  std::uint32_t wheel_no_effect_attempt_limit_{2};
  std::uint32_t wheel_no_effect_cooldown_ms_{1200};
  std::uint32_t wheel_recovery_verify_timeout_ms_{180};
  std::uint32_t wheel_no_effect_diag_freshness_ms_{350};
  std::uint32_t wheel_no_effect_diag_request_period_ms_{80};
  float leg_no_effect_position_error_threshold_{0.18f};
  float leg_no_effect_velocity_epsilon_{0.12f};
  float leg_no_effect_max_estimated_current_arms_{4.0f};
  float leg_no_effect_max_abs_torque_nm_{5.0f};
  float leg_no_effect_max_temperature_c_{100.0f};
  float leg_no_effect_min_bus_voltage_v_{18.0f};
  std::uint32_t leg_no_effect_command_warmup_cycles_{40};
  std::uint32_t leg_no_effect_trigger_cycles_{25};
  std::uint32_t leg_no_effect_attempt_limit_{2};
  std::uint32_t leg_no_effect_cooldown_ms_{1200};
  std::uint32_t leg_recovery_verify_timeout_ms_{220};

  void onEstop(const std_msgs::msg::Bool::SharedPtr msg);
  void logEvent(
    const std::string & level,
    const std::string & event,
    const std::string & message);
};

}  // namespace sim2real_hw
