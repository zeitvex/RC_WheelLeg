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
#include "sim2real_interfaces/msg/runtime_state.hpp"
#include "sim2real_interfaces/msg/runtime_target.hpp"
#include "sim2real_common/low_pass_filter.hpp"
#include "sim2real_common/mahony_filter.hpp"
#include "sim2real_common/safety_monitor.hpp"
#include "sim2real_common/runtime_guard.hpp"

namespace sim2real_hw
{

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
  std::uint32_t update_count{0};
  std::uint32_t stale_count{0};
  // Hold-over state
  float last_valid_pos{0.0f};
  float last_valid_vel{0.0f};
  float last_valid_torque{0.0f};
  std::uint32_t prev_update_count{0};
  bool has_valid_data{false};
};

class HardwareBridgeNode : public rclcpp::Node
{
public:
  HardwareBridgeNode();
  ~HardwareBridgeNode();

private:
  void onTarget(const sim2real_interfaces::msg::RuntimeTarget::SharedPtr msg);
  void onReadLoop();
  void onWriteLoop();
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  bool initCan(const std::string& ifname, int& fd);
  bool sendCanFrame(int fd, std::uint32_t can_id, const std::uint8_t* data, std::uint8_t dlc);
  bool readCanFrame(int fd, void* frame, int timeout_us);

  bool enableMotor(int fd, int motor_id);
  bool disableMotor(int fd, int motor_id);
  bool setModeRaw(int fd, int motor_id, std::int8_t mode);
  bool writeLimit(int fd, int motor_id, std::uint16_t param_id, float limit);
  bool writeOperationFrame(int fd, int motor_id, double pos, double vel, double kp, double kd, double torque);

  rclcpp::Publisher<sim2real_interfaces::msg::RuntimeState>::SharedPtr state_pub_;
  rclcpp::Subscription<sim2real_interfaces::msg::RuntimeTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr write_timer_;

  std::mutex target_mutex_;
  std::array<float, 16> latest_target_{};
  std::array<float, 16> latest_raw_action_{};
  std::string latest_target_source_{"boot_hold"};
  rclcpp::Time latest_target_stamp_{0, 0, RCL_ROS_TIME};
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
  std::unique_ptr<sim2real_common::RuntimeGuard> runtime_guard_;
  bool mahony_initialized_{false};
  rclcpp::Time last_read_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time startup_soft_hold_start_time_{0, 0, RCL_ROS_TIME};

  // Telemetry
  std::uint32_t fresh_count_{0};
  std::uint32_t holdover_count_{0};
  std::uint32_t stale_max_{0};
  std::uint32_t holdover_events_total_{0};

  bool dry_run_{false};
  std::atomic<bool> estop_triggered_{false};
  std::atomic<bool> safety_enabled_{true};
  std::atomic<bool> safety_triggered_{false};
  std::string safety_reason_{""};

  void onEstop(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace sim2real_hw
