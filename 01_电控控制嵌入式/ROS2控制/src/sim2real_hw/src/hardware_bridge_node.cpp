#include "sim2real_hw/hardware_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstring>
#include <sstream>
#include <string>
#include <thread>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sim2real_common/deployment_contract.hpp"

using namespace std::chrono_literals;

namespace sim2real_hw
{

// Protocol constants
const std::uint32_t COMM_ENABLE = 3;
const std::uint32_t COMM_DISABLE = 4;
const std::uint32_t COMM_WRITE_PARAMETER = 18;
const std::uint32_t COMM_READ_PARAMETER = 17;
const std::uint32_t COMM_OPERATION_CONTROL = 1;
const std::uint32_t COMM_SET_ZERO_POSITION = 6;
const std::uint16_t PARAM_MODE = 0x7005;
const std::uint16_t PARAM_VELOCITY_LIMIT = 0x7017;
const std::uint16_t PARAM_TORQUE_LIMIT = 0x700B;
const std::uint16_t PARAM_CAN_TIMEOUT = 0x7028;
const std::uint16_t PARAM_VBUS = 0x3007;
const std::uint16_t PARAM_DRV_FAULT = 0x3022;
const std::uint16_t PARAM_DRV_FAULT_DETAIL_1 = 0x3024;
const std::uint16_t PARAM_DRV_FAULT_DETAIL_2 = 0x3025;
const std::uint8_t HOST_ID = 0xFD;
constexpr int kMotorInitRetrySleepMs = 15;
constexpr int kMotorInitConfirmTimeoutMs = 120;
constexpr int kMotorRecoveryCooldownMs = 500;
constexpr std::uint32_t kMotorRecoveryTriggerStaleCount = 10;
constexpr std::uint32_t kMotorRecoveryAttemptLimit = 3;
inline void pack_u16_be(std::uint8_t* buf, std::uint16_t val)
{
  buf[0] = (val >> 8) & 0xFF;
  buf[1] = val & 0xFF;
}

inline float nearest_periodic(float val, float ref)
{
  float diff = val - ref;
  float wrapped = diff - 2.0f * static_cast<float>(M_PI) * std::floor((diff + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI)));
  return ref + wrapped;
}

std::string extractJsonStringField(const std::string & payload, const std::string & key)
{
  const std::string needle = "\"" + key + "\":\"";
  const std::size_t start = payload.find(needle);
  if (start == std::string::npos) {
    return {};
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find('"', value_start);
  if (value_end == std::string::npos) {
    return {};
  }

  return payload.substr(value_start, value_end - value_start);
}

std::string toLowerCopy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

HardwareBridgeNode::HardwareBridgeNode()
: Node("sim2real_hw_node")
{
  // 1. Declare and get parameters
  const std::string event_log_dir = declare_parameter<std::string>(
    "event_log_dir", "logs_v2_web");
  run_log_dir_ = event_log_dir;
  event_logger_.configure(event_log_dir, "sim2real_hw_events");
  target_timeout_ms_ = declare_parameter<double>("target_timeout_ms", 150.0);
  can0_name_ = declare_parameter<std::string>("can0_name", "can0");
  can1_name_ = declare_parameter<std::string>("can1_name", "can1");
  dry_run_ = declare_parameter<bool>("dry_run", true); // Default to dry-run for safety
  const std::vector<double> default_rough_pose(
    sim2real_common::DeploymentContract::kDefaultDofPos.begin(),
    sim2real_common::DeploymentContract::kDefaultDofPos.end());
  const std::vector<double> configured_rough_pose = declare_parameter<std::vector<double>>(
    "rough_default_dof_pos", default_rough_pose);
  if (configured_rough_pose.size() == rough_default_dof_pos_.size()) {
    for (std::size_t i = 0; i < rough_default_dof_pos_.size(); ++i) {
      rough_default_dof_pos_[i] = static_cast<float>(configured_rough_pose[i]);
    }
  } else {
    rough_default_dof_pos_ = sim2real_common::DeploymentContract::kDefaultDofPos;
    RCLCPP_WARN(
      get_logger(),
      "Parameter rough_default_dof_pos has %zu entries, expected 16. Falling back to deployment default pose.",
      configured_rough_pose.size());
  }
  active_default_dof_pos_ = rough_default_dof_pos_;
  const std::vector<double> default_crawl_pose = declare_parameter<std::vector<double>>(
    "crawl_default_dof_pos",
    std::vector<double>{
      0.2, 1.697, -2.650,
      -0.2, 1.697, -2.650,
      0.2, 1.697, -2.650,
      -0.2, 1.697, -2.650,
      0.0, 0.0, 0.0, 0.0
    });
  if (default_crawl_pose.size() == crawl_default_dof_pos_.size()) {
    for (std::size_t i = 0; i < crawl_default_dof_pos_.size(); ++i) {
      crawl_default_dof_pos_[i] = static_cast<float>(default_crawl_pose[i]);
    }
  } else {
    crawl_default_dof_pos_ = {
      0.2f, 1.697f, -2.650f,
      -0.2f, 1.697f, -2.650f,
      0.2f, 1.697f, -2.650f,
      -0.2f, 1.697f, -2.650f,
      0.0f, 0.0f, 0.0f, 0.0f
    };
    RCLCPP_WARN(
      get_logger(),
      "Parameter crawl_default_dof_pos has %zu entries, expected 16. Falling back to configured crawl pose.",
      default_crawl_pose.size());
  }
  const std::vector<double> default_wall_pose_param(
    rough_default_dof_pos_.begin(), rough_default_dof_pos_.end());
  const std::vector<double> default_wall_pose = declare_parameter<std::vector<double>>(
    "wall_default_dof_pos", default_wall_pose_param);
  if (default_wall_pose.size() == wall_default_dof_pos_.size()) {
    for (std::size_t i = 0; i < wall_default_dof_pos_.size(); ++i) {
      wall_default_dof_pos_[i] = static_cast<float>(default_wall_pose[i]);
    }
  } else {
    wall_default_dof_pos_ = rough_default_dof_pos_;
    RCLCPP_WARN(
      get_logger(),
      "Parameter wall_default_dof_pos has %zu entries, expected 16. Falling back to rough default pose.",
      default_wall_pose.size());
  }

  // Safety parameters
  safety_enabled_ = declare_parameter<bool>("safety_enabled", true);
  double max_target_offset = declare_parameter<double>("max_target_offset", 0.6);
  double model_switch_max_target_offset = declare_parameter<double>(
    "model_switch_max_target_offset", std::max(max_target_offset, 1.8));
  double hard_target_offset = declare_parameter<double>("hard_target_offset", 1.2);
  double max_ang_vel = declare_parameter<double>("max_ang_vel", 10.0);
  double max_tilt_z = declare_parameter<double>("max_tilt_z", -0.3);
  int clip_to_brake = declare_parameter<int>("clip_to_brake", 0);
  double imu_age_warn_ms = declare_parameter<double>("imu_age_warn_ms", 60.0);
  double imu_age_stop_ms = declare_parameter<double>("imu_age_stop_ms", 200.0);
  motor_temp_warn_c_ = static_cast<float>(declare_parameter<double>("motor_temp_warn_c", 100.0));
  motor_temp_fault_c_ = static_cast<float>(declare_parameter<double>("motor_temp_fault_c", 135.0));
  motor_bus_overvoltage_v_ = static_cast<float>(declare_parameter<double>("motor_bus_overvoltage_v", 60.0));
  motor_bus_undervoltage_v_ = static_cast<float>(declare_parameter<double>("motor_bus_undervoltage_v", 12.0));
  motor_current_warn_arms_ = static_cast<float>(declare_parameter<double>("motor_current_warn_arms", 10.5));
  motor_current_peak_arms_ = static_cast<float>(declare_parameter<double>("motor_current_peak_arms", 14.0));
  motor_torque_warn_nm_ = static_cast<float>(declare_parameter<double>("motor_torque_warn_nm", 13.0));
  motor_torque_peak_nm_ = static_cast<float>(declare_parameter<double>("motor_torque_peak_nm", 17.0));
  diag_poll_period_s_ = declare_parameter<double>("motor_diag_poll_period_s", 0.10);
  wheel_no_effect_command_threshold_ = static_cast<float>(
    declare_parameter<double>("wheel_no_effect_command_threshold", 1.0));
  wheel_no_effect_min_response_ratio_ = static_cast<float>(
    declare_parameter<double>("wheel_no_effect_min_response_ratio", 0.20));
  wheel_no_effect_velocity_epsilon_ = static_cast<float>(
    declare_parameter<double>("wheel_no_effect_velocity_epsilon", 0.25));
  wheel_no_effect_max_temperature_c_ = static_cast<float>(
    declare_parameter<double>("wheel_no_effect_max_temperature_c", 90.0));
  wheel_no_effect_min_bus_voltage_v_ = static_cast<float>(
    declare_parameter<double>("wheel_no_effect_min_bus_voltage_v", 18.0));
  wheel_no_effect_command_warmup_cycles_ = static_cast<std::uint32_t>(
    declare_parameter<int>("wheel_no_effect_command_warmup_cycles", 12));
  wheel_no_effect_trigger_cycles_ = static_cast<std::uint32_t>(
    declare_parameter<int>("wheel_no_effect_trigger_cycles", 30));
  wheel_no_effect_attempt_limit_ = static_cast<std::uint32_t>(
    declare_parameter<int>("wheel_no_effect_attempt_limit", 2));
  wheel_no_effect_cooldown_ms_ = static_cast<std::uint32_t>(
    declare_parameter<int>("wheel_no_effect_cooldown_ms", 1200));
  wheel_recovery_verify_timeout_ms_ = static_cast<std::uint32_t>(
    declare_parameter<int>("wheel_recovery_verify_timeout_ms", 180));
  wheel_no_effect_diag_freshness_ms_ = static_cast<std::uint32_t>(
    declare_parameter<int>("wheel_no_effect_diag_freshness_ms", 350));
  wheel_no_effect_diag_request_period_ms_ = static_cast<std::uint32_t>(
    declare_parameter<int>("wheel_no_effect_diag_request_period_ms", 80));
  leg_no_effect_position_error_threshold_ = static_cast<float>(
    declare_parameter<double>("leg_no_effect_position_error_threshold", 0.18));
  leg_no_effect_velocity_epsilon_ = static_cast<float>(
    declare_parameter<double>("leg_no_effect_velocity_epsilon", 0.12));
  leg_no_effect_max_estimated_current_arms_ = static_cast<float>(
    declare_parameter<double>("leg_no_effect_max_estimated_current_arms", 4.0));
  leg_no_effect_max_abs_torque_nm_ = static_cast<float>(
    declare_parameter<double>("leg_no_effect_max_abs_torque_nm", 5.0));
  leg_no_effect_max_temperature_c_ = static_cast<float>(
    declare_parameter<double>("leg_no_effect_max_temperature_c", 100.0));
  leg_no_effect_min_bus_voltage_v_ = static_cast<float>(
    declare_parameter<double>("leg_no_effect_min_bus_voltage_v", 18.0));
  leg_no_effect_command_warmup_cycles_ = static_cast<std::uint32_t>(
    declare_parameter<int>("leg_no_effect_command_warmup_cycles", 40));
  leg_no_effect_trigger_cycles_ = static_cast<std::uint32_t>(
    declare_parameter<int>("leg_no_effect_trigger_cycles", 25));
  leg_no_effect_attempt_limit_ = static_cast<std::uint32_t>(
    declare_parameter<int>("leg_no_effect_attempt_limit", 2));
  leg_no_effect_cooldown_ms_ = static_cast<std::uint32_t>(
    declare_parameter<int>("leg_no_effect_cooldown_ms", 1200));
  leg_recovery_verify_timeout_ms_ = static_cast<std::uint32_t>(
    declare_parameter<int>("leg_recovery_verify_timeout_ms", 220));

  RCLCPP_INFO(get_logger(), "Initializing hardware bridge node (Dry run: %s)", dry_run_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "Event log file: %s", event_logger_.componentLogPath().c_str());
  RCLCPP_INFO(get_logger(), "Run log directory: %s", run_log_dir_.c_str());
  logEvent("INFO", "node_start", dry_run_ ? "Hardware bridge node started in dry-run mode." : "Hardware bridge node started.");
  if (safety_enabled_) {
    RCLCPP_INFO(get_logger(), "Safety monitoring is ENABLED (tilt threshold: %f, ang_vel threshold: %f)", max_tilt_z, max_ang_vel);
  } else {
    RCLCPP_WARN(get_logger(), "Safety monitoring is DISABLED!");
  }

  // 2. Set up logical motors mapping matching contract
  // Mapping index in array matches joint ordering in kJointLabels
  for (std::size_t i = 0; i < 16; ++i) {
    motors_[i].direction = sim2real_common::DeploymentContract::kDirectionMap[i];
    motors_[i].offset = sim2real_common::DeploymentContract::kZeroOffsetMap[i];
    motors_[i].bus = sim2real_common::DeploymentContract::kCanBusMap[i];
    motors_[i].id = sim2real_common::DeploymentContract::kCanIdMap[i];
  }

  // 3. Initialize filters & safety monitors
  lpf_legs_ = std::make_unique<sim2real_common::LowPassFilter>(5.0, 0.005, 12);
  lpf_wheels_ = std::make_unique<sim2real_common::LowPassFilter>(15.0, 0.005, 4);
  mahony_filter_ = std::make_unique<sim2real_common::MahonyFilter>(2.0f, 0.0f);

  safety_monitor_ = std::make_unique<sim2real_common::SafetyMonitor>(
    static_cast<float>(max_target_offset),
    static_cast<float>(max_ang_vel),
    static_cast<float>(max_tilt_z),
    clip_to_brake,
    static_cast<float>(hard_target_offset)
  );
  model_switch_safety_monitor_ = std::make_unique<sim2real_common::SafetyMonitor>(
    static_cast<float>(model_switch_max_target_offset),
    static_cast<float>(max_ang_vel),
    static_cast<float>(max_tilt_z),
    clip_to_brake,
    static_cast<float>(hard_target_offset)
  );

  runtime_guard_ = std::make_unique<sim2real_common::RuntimeGuard>(
    static_cast<float>(max_ang_vel + 2.0), // slightly higher limit for runtime guard stop
    static_cast<float>(max_tilt_z),
    static_cast<float>(imu_age_warn_ms),
    static_cast<float>(imu_age_stop_ms)
  );

  // 4. Initialize CAN sockets if not in dry-run
  if (!dry_run_) {
    if (!initCan(can0_name_, can0_fd_) || !initCan(can1_name_, can1_fd_)) {
      RCLCPP_ERROR(get_logger(), "CAN initialization failed! Falling back to dry-run.");
      dry_run_ = true;
    }
  }

  // 5. Initialize motor target states
  latest_target_ = rough_default_dof_pos_;
  latest_raw_action_.fill(0.0f);

  // 6. Set up ROS publishers & subscriptions
  motor_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sensor_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  control_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sensor_sub_options;
  sensor_sub_options.callback_group = sensor_callback_group_;
  rclcpp::SubscriptionOptions control_sub_options;
  control_sub_options.callback_group = control_callback_group_;

  state_pub_ = create_publisher<sim2real_interfaces::msg::RuntimeState>("runtime/state", 10);
  target_sub_ = create_subscription<sim2real_interfaces::msg::RuntimeTarget>(
    "runtime/target", 10,
    std::bind(&HardwareBridgeNode::onTarget, this, std::placeholders::_1),
    control_sub_options);
  model_status_sub_ = create_subscription<std_msgs::msg::String>(
    "runtime/model_status", 10,
    std::bind(&HardwareBridgeNode::onModelStatus, this, std::placeholders::_1),
    control_sub_options);
  std::string imu_topic = declare_parameter<std::string>("imu_topic", "/odin1/imu");
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, 10,
    std::bind(&HardwareBridgeNode::onImu, this, std::placeholders::_1),
    sensor_sub_options);
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/safety/estop", 10,
    std::bind(&HardwareBridgeNode::onEstop, this, std::placeholders::_1),
    control_sub_options);

  // Odom subscription
  std::string odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    std::bind(&HardwareBridgeNode::onOdom, this, std::placeholders::_1),
    sensor_sub_options);

  // 7. Enable motors on total startup
  if (!dry_run_) {
    RCLCPP_INFO(get_logger(), "Enabling RobStride motors...");
    for (std::size_t i = 0; i < 16; ++i) {
      initializeMotor(i, "startup_init");
    }
  }

  // 8. Timers at 200Hz (5ms)
  read_timer_ = create_wall_timer(
    5ms, std::bind(&HardwareBridgeNode::onReadLoop, this), motor_callback_group_);
  write_timer_ = create_wall_timer(
    5ms, std::bind(&HardwareBridgeNode::onWriteLoop, this), motor_callback_group_);
}

HardwareBridgeNode::~HardwareBridgeNode()
{
  if (!dry_run_) {
    RCLCPP_INFO(get_logger(), "Disabling RobStride motors on shutdown...");
    for (std::size_t i = 0; i < 16; ++i) {
      int fd = (motors_[i].bus == 1) ? can0_fd_ : can1_fd_;
      logMotorPowerEvent(i, "disabled", "node shutdown requested motor disable");
      disableMotor(fd, motors_[i].id);
    }
    if (can0_fd_ >= 0) {
      if (::close(can0_fd_) < 0) {
        RCLCPP_WARN(get_logger(), "Failed to close can0 socket: %s", strerror(errno));
      }
    }
    if (can1_fd_ >= 0) {
      if (::close(can1_fd_) < 0) {
        RCLCPP_WARN(get_logger(), "Failed to close can1 socket: %s", strerror(errno));
      }
    }
  }
  logEvent("INFO", "node_stop", "Hardware bridge node stopped.");
  finalizeRunSummary();
}

void HardwareBridgeNode::logEvent(
  const std::string & level,
  const std::string & event,
  const std::string & message)
{
  event_logger_.log(level, "sim2real_hw_node", event, message);
}

const char * HardwareBridgeNode::jointName(std::size_t index) const
{
  return sim2real_common::kJointLabels[index];
}

std::string HardwareBridgeNode::motorTag(std::size_t index) const
{
  std::ostringstream oss;
  oss << jointName(index)
      << "(index=" << index
      << ",bus=" << motors_[index].bus
      << ",id=" << motors_[index].id
      << ")";
  return oss.str();
}

float HardwareBridgeNode::estimateCurrentArms(float torque_nm) const
{
  constexpr float kTorqueConstantNmPerArms = 1.22f;
  return std::abs(torque_nm) / kTorqueConstantNmPerArms;
}

std::string HardwareBridgeNode::decodeFaultCode(std::uint16_t fault_code) const
{
  if (fault_code == 0) {
    return "none";
  }

  std::vector<std::string> reasons;
  if (fault_code & (1u << 14)) reasons.emplace_back("stall_or_overload");
  if (fault_code & (1u << 7)) reasons.emplace_back("encoder_not_calibrated");
  if (fault_code & (1u << 3)) reasons.emplace_back("bus_overvoltage");
  if (fault_code & (1u << 2)) reasons.emplace_back("bus_undervoltage");
  if (fault_code & (1u << 1)) reasons.emplace_back("driver_chip_fault");
  if (fault_code & (1u << 0)) reasons.emplace_back("overtemperature");
  if (reasons.empty()) reasons.emplace_back("unknown_fault_bits");

  std::ostringstream oss;
  for (std::size_t i = 0; i < reasons.size(); ++i) {
    if (i > 0) {
      oss << '|';
    }
    oss << reasons[i];
  }
  return oss.str();
}

std::string HardwareBridgeNode::decodeFaultDetailRegister(std::uint16_t register_value, int register_index) const
{
  if (register_value == 0) {
    return "none";
  }

  std::ostringstream oss;
  if (register_index == 1) {
    oss << "driver_fault_reg1=0x" << std::hex << register_value
        << " (possible: mos_overcurrent_or_uvlo)";
  } else {
    oss << "driver_fault_reg2=0x" << std::hex << register_value
        << " (possible: gate_driver_short_or_half_bridge_damage)";
  }
  return oss.str();
}

std::string HardwareBridgeNode::buildMotorFaultSummary(std::size_t index) const
{
  const auto & state = motor_states_[index];
  std::ostringstream oss;
  oss << "motor=" << motorTag(index)
      << ", fault_code=0x" << std::hex << state.fault_code << std::dec
      << ", decoded_fault=" << decodeFaultCode(state.fault_code)
      << ", temperature_c=" << state.temperature
      << ", estimated_current_arms=" << state.estimated_current_arms;
  if (state.has_bus_voltage) {
    oss << ", bus_voltage_v=" << state.bus_voltage;
  }
  if (state.fault_detail_1 != 0) {
    oss << ", " << decodeFaultDetailRegister(state.fault_detail_1, 1);
  }
  if (state.fault_detail_2 != 0) {
    oss << ", " << decodeFaultDetailRegister(state.fault_detail_2, 2);
  }
  return oss.str();
}

std::string HardwareBridgeNode::formatProtectionReason(
  const std::string & trigger,
  const std::string & reason) const
{
  std::ostringstream oss;
  oss << "trigger=" << trigger
      << ", protection_action=safety_brake, reason=" << reason;
  return oss.str();
}

void HardwareBridgeNode::logProtectionEvent(
  const std::string & trigger,
  const std::string & reason,
  const std::string & action)
{
  protection_trigger_count_++;
  std::ostringstream oss;
  oss << "trigger=" << trigger
      << ", protection_action=" << action
      << ", reason=" << reason;
  std::size_t suspect_index = motors_.size();
  float suspect_score = -1.0f;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    float score = 0.0f;
    if (motor_states_[i].fault_code != 0 || motor_states_[i].fault_detail_1 != 0 || motor_states_[i].fault_detail_2 != 0) {
      score += 100.0f;
    }
    score += motor_states_[i].temperature;
    score += 2.0f * motor_states_[i].estimated_current_arms;
    score += std::min<float>(50.0f, static_cast<float>(motor_states_[i].stale_count));
    if (score > suspect_score) {
      suspect_score = score;
      suspect_index = i;
    }
  }
  if (suspect_index < motors_.size()) {
    oss << ", suspect_motor={" << buildMotorFaultSummary(suspect_index) << "}";
  }
  logEvent("ERROR", "protection_triggered", oss.str());
}

void HardwareBridgeNode::logMotorPowerEvent(
  std::size_t index,
  const std::string & state,
  const std::string & reason)
{
  std::ostringstream oss;
  oss << "motor=" << motorTag(index)
      << ", state=" << state
      << ", reason=" << reason;
  logEvent(state == "recovered" ? "INFO" : "WARN", "motor_power_state", oss.str());
}

void HardwareBridgeNode::logMotorDiagnosticEvent(
  std::size_t index,
  const std::string & event,
  const std::string & reason,
  const char * level)
{
  std::ostringstream oss;
  oss << "motor=" << motorTag(index) << ", " << reason;
  logEvent(level, event, oss.str());
}

void HardwareBridgeNode::finalizeRunSummary()
{
  std::ostringstream oss;
  oss << "run_dir=" << run_log_dir_
      << ", protection_trigger_count=" << protection_trigger_count_
      << ", motor_drop_event_count=" << motor_drop_event_count_
      << ", motor_recover_event_count=" << motor_recover_event_count_
      << ", motor_fault_event_count=" << motor_fault_event_count_
      << ", holdover_events_total=" << holdover_events_total_
      << ", final_safety_triggered=" << (safety_triggered_ ? "true" : "false");
  if (!safety_reason_.empty()) {
    oss << ", final_safety_reason=" << safety_reason_;
  }
  event_logger_.logSummary("sim2real_hw_node", oss.str());
}

void HardwareBridgeNode::updateMotorTelemetry(
  std::size_t index,
  float pos_sim,
  float vel_sim,
  float torque_sim,
  float temperature_c)
{
  auto & state = motor_states_[index];
  state.position = pos_sim;
  state.velocity = vel_sim;
  state.torque = torque_sim;
  state.temperature = temperature_c;
  state.estimated_current_arms = estimateCurrentArms(torque_sim);
  state.update_count++;
  state.stale_count = 0;
  state.last_valid_pos = pos_sim;
  state.last_valid_vel = vel_sim;
  state.last_valid_torque = torque_sim;
  state.has_valid_data = true;
  state.init_confirmed = true;
  state.recovery_attempt_count = 0;
}

bool HardwareBridgeNode::isWheelMotor(std::size_t index) const
{
  return index >= sim2real_common::DeploymentContract::kLegJointCount;
}

bool HardwareBridgeNode::isLegMotor(std::size_t index) const
{
  return index < sim2real_common::DeploymentContract::kLegJointCount;
}

bool HardwareBridgeNode::motorHasBlockingFault(std::size_t index) const
{
  const auto & state = motor_states_[index];
  if (state.fault_code != 0 || state.fault_detail_1 != 0 || state.fault_detail_2 != 0) {
    return true;
  }
  const float max_temperature = isWheelMotor(index)
    ? wheel_no_effect_max_temperature_c_
    : leg_no_effect_max_temperature_c_;
  if (state.temperature >= max_temperature) {
    return true;
  }
  const float min_bus_voltage = isWheelMotor(index)
    ? wheel_no_effect_min_bus_voltage_v_
    : leg_no_effect_min_bus_voltage_v_;
  if (state.has_bus_voltage && state.bus_voltage <= min_bus_voltage) {
    return true;
  }
  return false;
}

std::uint32_t HardwareBridgeNode::noEffectCommandWarmupCycles(std::size_t index) const
{
  return isWheelMotor(index) ? wheel_no_effect_command_warmup_cycles_ : leg_no_effect_command_warmup_cycles_;
}

std::uint32_t HardwareBridgeNode::noEffectTriggerCycles(std::size_t index) const
{
  return isWheelMotor(index) ? wheel_no_effect_trigger_cycles_ : leg_no_effect_trigger_cycles_;
}

std::uint32_t HardwareBridgeNode::noEffectAttemptLimit(std::size_t index) const
{
  return isWheelMotor(index) ? wheel_no_effect_attempt_limit_ : leg_no_effect_attempt_limit_;
}

std::uint32_t HardwareBridgeNode::noEffectCooldownMs(std::size_t index) const
{
  return isWheelMotor(index) ? wheel_no_effect_cooldown_ms_ : leg_no_effect_cooldown_ms_;
}

std::uint32_t HardwareBridgeNode::noEffectVerifyTimeoutMs(std::size_t index) const
{
  return isWheelMotor(index) ? wheel_recovery_verify_timeout_ms_ : leg_recovery_verify_timeout_ms_;
}

bool HardwareBridgeNode::hasFreshNoEffectDiagnostics(std::size_t index) const
{
  const auto & state = motor_states_[index];
  if (!state.has_bus_voltage || !state.has_fault_snapshot) {
    return false;
  }
  if (state.last_diag_snapshot_time_.time_since_epoch().count() == 0) {
    return false;
  }
  const auto age = std::chrono::steady_clock::now() - state.last_diag_snapshot_time_;
  return age <= std::chrono::milliseconds(wheel_no_effect_diag_freshness_ms_);
}

void HardwareBridgeNode::requestMotorDiagnostics(std::size_t index)
{
  if (dry_run_ || index >= motors_.size()) {
    return;
  }

  auto & state = motor_states_[index];
  const auto now_tp = std::chrono::steady_clock::now();
  if (state.last_diag_request_time_.time_since_epoch().count() != 0) {
    const auto since_last = now_tp - state.last_diag_request_time_;
    if (since_last < std::chrono::milliseconds(wheel_no_effect_diag_request_period_ms_)) {
      return;
    }
  }

  const int fd = motors_[index].bus == 1 ? can0_fd_ : can1_fd_;
  readParameter(fd, motors_[index].id, PARAM_VBUS);
  readParameter(fd, motors_[index].id, PARAM_DRV_FAULT);
  readParameter(fd, motors_[index].id, PARAM_DRV_FAULT_DETAIL_1);
  readParameter(fd, motors_[index].id, PARAM_DRV_FAULT_DETAIL_2);
  state.last_diag_request_time_ = now_tp;
}

std::string HardwareBridgeNode::classifyNoEffectSuspect(std::size_t index) const
{
  const auto & state = motor_states_[index];
  if (state.fault_code != 0 || state.fault_detail_1 != 0 || state.fault_detail_2 != 0) {
    return "suspect_driver_fault_or_protection";
  }
  const float min_bus_voltage = isWheelMotor(index)
    ? wheel_no_effect_min_bus_voltage_v_
    : leg_no_effect_min_bus_voltage_v_;
  if (state.has_bus_voltage && state.bus_voltage <= min_bus_voltage) {
    return "suspect_low_voltage";
  }
  const float max_temperature = isWheelMotor(index)
    ? wheel_no_effect_max_temperature_c_
    : leg_no_effect_max_temperature_c_;
  if (state.temperature >= max_temperature) {
    return "suspect_overtemp";
  }
  if (state.estimated_current_arms >= motor_current_warn_arms_ ||
      std::abs(state.torque) >= motor_torque_warn_nm_) {
    return "suspect_mechanical_stall_or_overload";
  }
  if (isLegMotor(index)) {
    return "suspect_enable_or_position_loop_drop";
  }
  return "suspect_enable_or_mode_drop";
}

std::string HardwareBridgeNode::buildNoEffectSummary(std::size_t index) const
{
  const auto & state = motor_states_[index];
  const float commanded = std::abs(state.last_command_sim);
  const float actual = std::abs(state.velocity);
  const float response_ratio = isWheelMotor(index) && commanded > 1.0e-4f ? actual / commanded : 1.0f;
  const float position_error = std::abs(state.last_command_sim - state.position);

  std::ostringstream oss;
  oss << "suspect=" << classifyNoEffectSuspect(index)
      << ", command=" << state.last_command_sim
      << ", position=" << state.position
      << ", position_error=" << position_error
      << ", feedback_vel=" << state.velocity
      << ", response_ratio=" << response_ratio
      << ", torque_nm=" << state.torque
      << ", estimated_current_arms=" << state.estimated_current_arms
      << ", temperature_c=" << state.temperature;
  if (state.has_bus_voltage) {
    oss << ", bus_voltage_v=" << state.bus_voltage;
  }
  if (state.fault_code != 0 || state.fault_detail_1 != 0 || state.fault_detail_2 != 0) {
    oss << ", fault_summary={" << buildMotorFaultSummary(index) << "}";
  }
  return oss.str();
}

void HardwareBridgeNode::updateMotorCommandTracking(
  std::size_t index,
  float sim_command,
  const std::string & target_source)
{
  auto & state = motor_states_[index];
  state.last_command_sim = sim_command;

  if (target_source == "safety_brake" || target_source == "safety_estop") {
    state.command_active_count = 0;
    state.no_effect_count = 0;
    state.no_effect_recovery_attempt_count = 0;
    state.command_effect_monitoring_active = false;
    state.no_effect_reported = false;
    return;
  }

  bool command_active = false;
  if (isWheelMotor(index)) {
    command_active = std::abs(sim_command) >= wheel_no_effect_command_threshold_;
  } else {
    command_active = std::abs(sim_command - state.position) >= leg_no_effect_position_error_threshold_;
  }

  if (command_active) {
    state.command_active_count++;
    state.command_effect_monitoring_active = true;
  } else {
    state.command_active_count = 0;
    state.no_effect_count = 0;
    state.no_effect_recovery_attempt_count = 0;
    state.command_effect_monitoring_active = false;
    state.no_effect_reported = false;
  }
}

void HardwareBridgeNode::updateNoEffectDetection(std::size_t index)
{
  auto & state = motor_states_[index];
  if (!state.has_valid_data || state.stale_count > 0) {
    state.no_effect_count = 0;
    state.command_effect_monitoring_active = false;
    state.no_effect_reported = false;
    return;
  }

  if (!state.command_effect_monitoring_active ||
      state.command_active_count < noEffectCommandWarmupCycles(index)) {
    state.no_effect_count = 0;
    state.no_effect_reported = false;
    return;
  }

  if (!hasFreshNoEffectDiagnostics(index)) {
    requestMotorDiagnostics(index);
    state.no_effect_count = 0;
    return;
  }

  if (motorHasBlockingFault(index)) {
    state.no_effect_count = 0;
    return;
  }

  if (isNoEffectConditionPresent(index)) {
    state.no_effect_count++;
    if (!state.no_effect_reported &&
        state.no_effect_count >= noEffectTriggerCycles(index)) {
      state.no_effect_reported = true;
      std::ostringstream oss;
      oss << "trigger=" << (isWheelMotor(index) ? "wheel_no_effect" : "leg_no_effect")
          << ", no_effect_count=" << state.no_effect_count
          << ", " << buildNoEffectSummary(index);
      logMotorDiagnosticEvent(index, "motor_no_effect_detected", oss.str(), "WARN");
    }
  } else {
    state.no_effect_count = 0;
    state.no_effect_recovery_attempt_count = 0;
    state.no_effect_reported = false;
  }
}

bool HardwareBridgeNode::isNoEffectConditionPresent(std::size_t index) const
{
  const auto & state = motor_states_[index];
  if (isWheelMotor(index)) {
    const float commanded = std::abs(state.last_command_sim);
    const float actual = std::abs(state.velocity);
    const float response_ratio = commanded > 1.0e-4f ? actual / commanded : 1.0f;
    return actual <= wheel_no_effect_velocity_epsilon_ ||
      response_ratio < wheel_no_effect_min_response_ratio_;
  }

  const float position_error = std::abs(state.last_command_sim - state.position);
  const float actual_velocity = std::abs(state.velocity);
  const float estimated_current = std::abs(state.estimated_current_arms);
  const float measured_torque = std::abs(state.torque);
  return position_error >= leg_no_effect_position_error_threshold_ &&
    actual_velocity <= leg_no_effect_velocity_epsilon_ &&
    estimated_current <= leg_no_effect_max_estimated_current_arms_ &&
    measured_torque <= leg_no_effect_max_abs_torque_nm_;
}

void HardwareBridgeNode::handleParameterResponse(const struct can_frame & frame, int bus_id)
{
  const std::uint32_t extra_data = (frame.can_id >> 8) & 0xFFFF;
  const int motor_id = extra_data & 0xFF;
  const std::uint16_t param_id = static_cast<std::uint16_t>((frame.data[1] << 8) | frame.data[0]);

  for (std::size_t i = 0; i < motors_.size(); ++i) {
    if (motors_[i].bus != bus_id || motors_[i].id != motor_id) {
      continue;
    }

    auto & state = motor_states_[i];
    if (param_id == PARAM_VBUS) {
      float vbus = 0.0f;
      std::memcpy(&vbus, &frame.data[4], sizeof(float));
      state.bus_voltage = vbus;
      state.has_bus_voltage = true;
      state.last_diag_snapshot_time_ = std::chrono::steady_clock::now();
    } else if (param_id == PARAM_DRV_FAULT) {
      const std::uint16_t fault = static_cast<std::uint16_t>((frame.data[5] << 8) | frame.data[4]);
      state.fault_code = fault;
      state.has_fault_snapshot = true;
      state.last_diag_snapshot_time_ = std::chrono::steady_clock::now();
    } else if (param_id == PARAM_DRV_FAULT_DETAIL_1) {
      state.fault_detail_1 = static_cast<std::uint16_t>((frame.data[5] << 8) | frame.data[4]);
      state.has_fault_snapshot = true;
      state.last_diag_snapshot_time_ = std::chrono::steady_clock::now();
    } else if (param_id == PARAM_DRV_FAULT_DETAIL_2) {
      state.fault_detail_2 = static_cast<std::uint16_t>((frame.data[5] << 8) | frame.data[4]);
      state.has_fault_snapshot = true;
      state.last_diag_snapshot_time_ = std::chrono::steady_clock::now();
    }
    break;
  }
}

bool HardwareBridgeNode::waitForMotorFeedback(std::size_t index, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const std::uint32_t prev_update_count = motor_states_[index].update_count;

  while (std::chrono::steady_clock::now() < deadline) {
    if (motor_states_[index].update_count > prev_update_count) {
      motor_states_[index].init_confirmed = true;
      return true;
    }

    if (can0_fd_ >= 0) {
      drainCanFrames(can0_fd_, 1, 1000);
    }
    if (can1_fd_ >= 0) {
      drainCanFrames(can1_fd_, 2, 1000);
    }
    std::this_thread::sleep_for(2ms);
  }

  return motor_states_[index].update_count > prev_update_count;
}

bool HardwareBridgeNode::startMotorRecoverySequence(
  std::size_t index,
  const std::string & trigger,
  RecoveryKind kind,
  std::uint32_t attempt_number)
{
  if (dry_run_ || index >= motors_.size()) {
    return false;
  }

  auto & state = motor_states_[index];
  if (state.recovery_stage != RecoveryStage::Idle) {
    return false;
  }

  const int fd = motors_[index].bus == 1 ? can0_fd_ : can1_fd_;
  if (fd < 0) {
    logMotorDiagnosticEvent(index, "motor_init_failed",
      "reason=" + trigger + ", detail=invalid_can_fd", "ERROR");
    return false;
  }

  bool step_ok = true;
  logMotorPowerEvent(index, "reset_before_enable",
    "trigger=" + trigger + ", recovery_attempt=" + std::to_string(attempt_number));
  step_ok = disableMotor(fd, motors_[index].id, true) && step_ok;
  state.fault_code = 0;
  state.fault_detail_1 = 0;
  state.fault_detail_2 = 0;
  state.fault_code_reported = false;
  std::this_thread::sleep_for(30ms);
  step_ok = setModeRaw(fd, motors_[index].id, 0) && step_ok;
  std::this_thread::sleep_for(30ms);
  step_ok = enableMotor(fd, motors_[index].id) && step_ok;
  std::this_thread::sleep_for(20ms);
  step_ok = writeLimit(fd, motors_[index].id, PARAM_VELOCITY_LIMIT, 20.0f) && step_ok;
  step_ok = writeLimit(fd, motors_[index].id, PARAM_TORQUE_LIMIT, 17.0f) && step_ok;
  step_ok = writeParameterInt(fd, motors_[index].id, PARAM_CAN_TIMEOUT, 0) && step_ok;

  if (!step_ok) {
    logMotorDiagnosticEvent(index, "motor_init_retry",
      "reason=" + trigger + ", attempt=1, sent_ok=false, feedback_confirmed=false", "ERROR");
    return false;
  }

  state.recovery_kind = kind;
  state.recovery_stage = RecoveryStage::AwaitInitFeedback;
  state.recovery_start_update_count = state.update_count;
  state.recovery_active_attempt_number = attempt_number;
  state.recovery_trigger = trigger;
  state.recovery_stage_deadline_ =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(kMotorInitConfirmTimeoutMs);
  state.init_confirmed = false;
  return true;
}

void HardwareBridgeNode::clearMotorRecoverySequence(std::size_t index)
{
  auto & state = motor_states_[index];
  state.recovery_kind = RecoveryKind::None;
  state.recovery_stage = RecoveryStage::Idle;
  state.recovery_start_update_count = 0;
  state.recovery_active_attempt_number = 0;
  state.recovery_stage_deadline_ = std::chrono::steady_clock::time_point{};
  state.recovery_trigger.clear();
}

void HardwareBridgeNode::processMotorRecoverySequence(std::size_t index)
{
  auto & state = motor_states_[index];
  if (state.recovery_stage == RecoveryStage::Idle) {
    return;
  }

  const auto now_tp = std::chrono::steady_clock::now();
  if (state.recovery_stage == RecoveryStage::AwaitInitFeedback) {
    if (state.update_count > state.recovery_start_update_count) {
      state.init_confirmed = true;
      if (state.recovery_kind == RecoveryKind::NoEffect) {
        state.recovery_stage = RecoveryStage::AwaitEffectVerification;
        state.recovery_stage_deadline_ =
          now_tp + std::chrono::milliseconds(noEffectVerifyTimeoutMs(index));
      } else {
        state.recovered_reported = true;
        state.stale_reported = false;
        logMotorPowerEvent(index, "recovered_after_reinit",
          "trigger=" + state.recovery_trigger + ", recovery_attempt=" +
          std::to_string(state.recovery_active_attempt_number));
        clearMotorRecoverySequence(index);
      }
      return;
    }

    if (now_tp >= state.recovery_stage_deadline_) {
      logMotorDiagnosticEvent(index, "motor_init_retry",
        "reason=" + state.recovery_trigger +
        ", attempt=" + std::to_string(state.recovery_active_attempt_number) +
        ", sent_ok=true, feedback_confirmed=false", "ERROR");
      clearMotorRecoverySequence(index);
    }
    return;
  }

  if (state.recovery_stage == RecoveryStage::AwaitEffectVerification) {
    if (state.stale_count == 0 && !motorHasBlockingFault(index)) {
      bool recovered = false;
      if (isWheelMotor(index)) {
        const float commanded = std::abs(state.last_command_sim);
        const float actual = std::abs(state.velocity);
        const float response_ratio = commanded > 1.0e-4f ? actual / commanded : 1.0f;
        recovered =
          commanded >= wheel_no_effect_command_threshold_ &&
          actual > wheel_no_effect_velocity_epsilon_ &&
          response_ratio >= wheel_no_effect_min_response_ratio_;
      } else {
        const float position_error = std::abs(state.last_command_sim - state.position);
        recovered = position_error < (leg_no_effect_position_error_threshold_ * 0.5f);
      }
      if (recovered) {
        state.no_effect_count = 0;
        state.no_effect_reported = false;
        state.command_active_count = 0;
        state.no_effect_recovery_attempt_count = 0;
        logMotorPowerEvent(index, "recovered_after_no_effect_reinit",
          "trigger=" + state.recovery_trigger + ", recovery_attempt=" +
          std::to_string(state.recovery_active_attempt_number));
        clearMotorRecoverySequence(index);
        return;
      }
    }

    if (now_tp >= state.recovery_stage_deadline_) {
      logMotorDiagnosticEvent(index, "motor_no_effect_recovery_failed",
        "trigger=" + state.recovery_trigger +
        ", no_effect_count=" + std::to_string(state.no_effect_count) +
        ", recovery_attempt=" + std::to_string(state.recovery_active_attempt_number) +
        ", " + buildNoEffectSummary(index), "ERROR");
      clearMotorRecoverySequence(index);
    }
  }
}

bool HardwareBridgeNode::initializeMotor(std::size_t index, const std::string & reason, int max_attempts)
{
  if (dry_run_ || index >= motors_.size()) {
    return true;
  }

  auto & state = motor_states_[index];
  const int fd = motors_[index].bus == 1 ? can0_fd_ : can1_fd_;
  if (fd < 0) {
    logMotorDiagnosticEvent(index, "motor_init_failed",
      "reason=" + reason + ", detail=invalid_can_fd", "ERROR");
    return false;
  }

  state.init_confirmed = false;
  bool success = false;
  for (int attempt = 1; attempt <= max_attempts; ++attempt) {
    state.init_attempt_count++;
    bool step_ok = true;
    logMotorPowerEvent(index, "reset_before_enable",
      "reason=" + reason + ", attempt=" + std::to_string(attempt));
    step_ok = disableMotor(fd, motors_[index].id, true) && step_ok;
    state.fault_code = 0;
    state.fault_detail_1 = 0;
    state.fault_detail_2 = 0;
    state.fault_code_reported = false;
    std::this_thread::sleep_for(30ms);
    step_ok = setModeRaw(fd, motors_[index].id, 0) && step_ok;
    std::this_thread::sleep_for(30ms);
    step_ok = enableMotor(fd, motors_[index].id) && step_ok;
    std::this_thread::sleep_for(20ms);
    step_ok = writeLimit(fd, motors_[index].id, PARAM_VELOCITY_LIMIT, 20.0f) && step_ok;
    step_ok = writeLimit(fd, motors_[index].id, PARAM_TORQUE_LIMIT, 17.0f) && step_ok;
    step_ok = writeParameterInt(fd, motors_[index].id, PARAM_CAN_TIMEOUT, 0) && step_ok;

    if (step_ok) {
      success = waitForMotorFeedback(index, std::chrono::milliseconds(kMotorInitConfirmTimeoutMs));
    }

    if (success) {
      std::ostringstream oss;
      oss << "reason=" << reason
          << ", attempt=" << attempt
          << ", init_attempt_count=" << state.init_attempt_count;
      logMotorDiagnosticEvent(index, "motor_init_confirmed", oss.str(), "INFO");
      return true;
    }

    std::ostringstream oss;
    oss << "reason=" << reason
        << ", attempt=" << attempt
        << ", sent_ok=" << (step_ok ? "true" : "false")
        << ", feedback_confirmed=" << (state.init_confirmed ? "true" : "false");
    logMotorDiagnosticEvent(index, "motor_init_retry", oss.str(), attempt == max_attempts ? "ERROR" : "WARN");
    std::this_thread::sleep_for(std::chrono::milliseconds(kMotorInitRetrySleepMs));
  }

  return false;
}

bool HardwareBridgeNode::initializeMotorsOnBus(int bus_id, const std::string & reason)
{
  bool all_ok = true;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    if (motors_[i].bus != bus_id) {
      continue;
    }
    const bool ok = initializeMotor(i, reason, 3);
    all_ok = ok && all_ok;
  }
  return all_ok;
}

void HardwareBridgeNode::processCanFrame(const struct can_frame & frame, int bus_id)
{
  if (!(frame.can_id & CAN_EFF_FLAG)) {
    return;
  }

  const std::uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
  if (comm_type == 2) {
    const std::uint32_t extra_data = (frame.can_id >> 8) & 0xFFFF;
    const int motor_id = extra_data & 0xFF;

    for (std::size_t i = 0; i < motors_.size(); ++i) {
      if (motors_[i].bus != bus_id || motors_[i].id != motor_id) {
        continue;
      }

      std::uint16_t p_u16 = (frame.data[0] << 8) | frame.data[1];
      std::uint16_t v_u16 = (frame.data[2] << 8) | frame.data[3];
      std::uint16_t t_u16 = (frame.data[4] << 8) | frame.data[5];
      std::uint16_t temp_u16 = (frame.data[6] << 8) | frame.data[7];

      double pos_raw = (static_cast<double>(p_u16) / 32767.0 - 1.0) * (4.0 * M_PI);
      double vel_raw = (static_cast<double>(v_u16) / 32767.0 - 1.0) * 44.0;
      double torque_raw = (static_cast<double>(t_u16) / 32767.0 - 1.0) * 17.0;

      float pos_sim = (static_cast<float>(pos_raw) - motors_[i].offset) / motors_[i].direction;
      float vel_sim = static_cast<float>(vel_raw) / motors_[i].direction;
      float torque_sim = static_cast<float>(torque_raw) / motors_[i].direction;

      if (i < 12) {
        pos_sim = nearest_periodic(pos_sim, rough_default_dof_pos_[i]);
      }

      updateMotorTelemetry(i, pos_sim, vel_sim, torque_sim, static_cast<float>(temp_u16) * 0.1f);
      updateMotorDiagnostics(i);
      return;
    }
  } else if (comm_type == COMM_READ_PARAMETER) {
    handleParameterResponse(frame, bus_id);
  }
}

void HardwareBridgeNode::drainCanFrames(int fd, int bus_id, int timeout_us)
{
  if (fd < 0) {
    return;
  }

  struct can_frame frame;
  while (readCanFrame(fd, &frame, timeout_us)) {
    processCanFrame(frame, bus_id);
    timeout_us = 0;
  }
}

bool HardwareBridgeNode::shouldAttemptMotorRecovery(std::size_t index) const
{
  const auto & state = motor_states_[index];
  if (state.recovery_stage != RecoveryStage::Idle) {
    return false;
  }
  if (!state.has_valid_data) {
    return false;
  }
  if (state.stale_count < kMotorRecoveryTriggerStaleCount) {
    return false;
  }
  if (state.recovery_attempt_count >= kMotorRecoveryAttemptLimit) {
    return false;
  }
  if (state.fault_code != 0 || state.fault_detail_1 != 0 || state.fault_detail_2 != 0) {
    return false;
  }

  const auto now_tp = std::chrono::steady_clock::now();
  if (state.last_recovery_attempt_time_.time_since_epoch().count() != 0) {
    const auto since_last = now_tp - state.last_recovery_attempt_time_;
    if (since_last < std::chrono::milliseconds(kMotorRecoveryCooldownMs)) {
      return false;
    }
  }
  return true;
}

bool HardwareBridgeNode::attemptMotorRecovery(std::size_t index, const std::string & trigger)
{
  if (!shouldAttemptMotorRecovery(index)) {
    return false;
  }

  auto & state = motor_states_[index];
  const std::uint32_t attempt_number = state.recovery_attempt_count + 1;

  std::ostringstream start_oss;
  start_oss << "trigger=" << trigger
            << ", stale_count=" << state.stale_count
            << ", recovery_attempt=" << attempt_number;
  logMotorDiagnosticEvent(index, "motor_recovery_attempt", start_oss.str(),
    attempt_number >= kMotorRecoveryAttemptLimit ? "ERROR" : "WARN");

  if (!startMotorRecoverySequence(index, trigger, RecoveryKind::Stale, attempt_number)) {
    return false;
  }
  state.recovery_attempt_count = attempt_number;
  state.last_recovery_attempt_time_ = std::chrono::steady_clock::now();
  return true;
}

bool HardwareBridgeNode::shouldAttemptNoEffectRecovery(std::size_t index) const
{
  const auto & state = motor_states_[index];
  if (state.recovery_stage != RecoveryStage::Idle) {
    return false;
  }
  if (!state.has_valid_data || state.stale_count > 0) {
    return false;
  }
  if (!hasFreshNoEffectDiagnostics(index)) {
    return false;
  }
  if (state.no_effect_count < noEffectTriggerCycles(index)) {
    return false;
  }
  if (motorHasBlockingFault(index)) {
    return false;
  }
  if (state.no_effect_recovery_attempt_count >= noEffectAttemptLimit(index)) {
    return false;
  }

  const auto now_tp = std::chrono::steady_clock::now();
  if (state.last_no_effect_recovery_attempt_time_.time_since_epoch().count() != 0) {
    const auto since_last = now_tp - state.last_no_effect_recovery_attempt_time_;
    if (since_last < std::chrono::milliseconds(noEffectCooldownMs(index))) {
      return false;
    }
  }
  return true;
}

bool HardwareBridgeNode::attemptNoEffectRecovery(std::size_t index, const std::string & trigger)
{
  if (!shouldAttemptNoEffectRecovery(index)) {
    return false;
  }

  auto & state = motor_states_[index];
  const std::uint32_t attempt_number = state.no_effect_recovery_attempt_count + 1;

  std::ostringstream start_oss;
  start_oss << "trigger=" << trigger
            << ", no_effect_count=" << state.no_effect_count
            << ", recovery_attempt=" << attempt_number
            << ", " << buildNoEffectSummary(index);
  logMotorDiagnosticEvent(index, "motor_no_effect_recovery_attempt", start_oss.str(),
    attempt_number >= noEffectAttemptLimit(index) ? "ERROR" : "WARN");

  if (!startMotorRecoverySequence(index, trigger, RecoveryKind::NoEffect, attempt_number)) {
    return false;
  }
  state.no_effect_recovery_attempt_count = attempt_number;
  state.last_no_effect_recovery_attempt_time_ = std::chrono::steady_clock::now();
  return true;
}

void HardwareBridgeNode::updateMotorDiagnostics(std::size_t index)
{
  auto & state = motor_states_[index];

  if (state.temperature >= motor_temp_warn_c_) {
    if (!state.high_temp_reported) {
      state.high_temp_reported = true;
      motor_fault_event_count_++;
      std::ostringstream oss;
      oss << "temperature_c=" << state.temperature
          << ", trigger=temperature_high"
          << ", protection_hint=" << (state.temperature >= motor_temp_fault_c_ ? "motor_overtemperature_fault" : "thermal_warning")
          << ", estimated_current_arms=" << state.estimated_current_arms;
      logMotorDiagnosticEvent(index, "motor_temperature_alert", oss.str(),
        state.temperature >= motor_temp_fault_c_ ? "ERROR" : "WARN");
    }
  } else {
    state.high_temp_reported = false;
  }

  if (state.estimated_current_arms >= motor_current_warn_arms_ || std::abs(state.torque) >= motor_torque_warn_nm_) {
    if (!state.high_current_reported) {
      state.high_current_reported = true;
      motor_fault_event_count_++;
      std::ostringstream oss;
      oss << "estimated_current_arms=" << state.estimated_current_arms
          << ", torque_nm=" << state.torque
          << ", trigger=" << (std::abs(state.torque) >= motor_torque_peak_nm_ || state.estimated_current_arms >= motor_current_peak_arms_
              ? "overload_peak" : "overload_warning")
          << ", protection_hint=stall_or_overload_protection";
      logMotorDiagnosticEvent(index, "motor_overload_alert", oss.str(),
        (std::abs(state.torque) >= motor_torque_peak_nm_ || state.estimated_current_arms >= motor_current_peak_arms_) ? "ERROR" : "WARN");
    }
  } else {
    state.high_current_reported = false;
  }

  if (state.has_bus_voltage) {
    if (state.bus_voltage >= motor_bus_overvoltage_v_) {
      if (!state.high_voltage_reported) {
        state.high_voltage_reported = true;
        motor_fault_event_count_++;
        logMotorDiagnosticEvent(index, "motor_bus_voltage_alert",
          "bus_voltage_v=" + std::to_string(state.bus_voltage) +
          ", trigger=bus_overvoltage, protection_hint=overvoltage_fault", "ERROR");
      }
    } else {
      state.high_voltage_reported = false;
    }

    if (state.bus_voltage <= motor_bus_undervoltage_v_) {
      if (!state.low_voltage_reported) {
        state.low_voltage_reported = true;
        motor_fault_event_count_++;
        logMotorDiagnosticEvent(index, "motor_bus_voltage_alert",
          "bus_voltage_v=" + std::to_string(state.bus_voltage) +
          ", trigger=bus_undervoltage, protection_hint=undervoltage_fault", "ERROR");
      }
    } else {
      state.low_voltage_reported = false;
    }
  }

  if (state.fault_code != 0 || state.fault_detail_1 != 0 || state.fault_detail_2 != 0) {
    if (!state.fault_code_reported) {
      state.fault_code_reported = true;
      motor_fault_event_count_++;
      logMotorDiagnosticEvent(index, "motor_fault_code", buildMotorFaultSummary(index), "ERROR");
    }
  } else {
    state.fault_code_reported = false;
  }
}

void HardwareBridgeNode::pollMotorDiagnostics()
{
  if (dry_run_ || diag_poll_period_s_ <= 0.0) {
    return;
  }

  const auto now_time = now();
  if (last_diag_poll_time_.nanoseconds() > 0 &&
      (now_time - last_diag_poll_time_).seconds() < diag_poll_period_s_) {
    return;
  }
  last_diag_poll_time_ = now_time;

  const std::size_t index = diag_poll_motor_index_ % motors_.size();
  const int fd = motors_[index].bus == 1 ? can0_fd_ : can1_fd_;
  readParameter(fd, motors_[index].id, PARAM_VBUS);
  readParameter(fd, motors_[index].id, PARAM_DRV_FAULT);
  readParameter(fd, motors_[index].id, PARAM_DRV_FAULT_DETAIL_1);
  readParameter(fd, motors_[index].id, PARAM_DRV_FAULT_DETAIL_2);
  diag_poll_motor_index_ = (diag_poll_motor_index_ + 1) % motors_.size();
}

void HardwareBridgeNode::onTarget(const sim2real_interfaces::msg::RuntimeTarget::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(target_mutex_);
  latest_target_ = msg->target;
  latest_raw_action_ = msg->raw_action;
  latest_target_source_ = msg->target_source;
  latest_target_stamp_ = rclcpp::Time(msg->stamp);
  target_sequence_ = msg->sequence;
}

void HardwareBridgeNode::onModelStatus(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string current_model = toLowerCopy(extractJsonStringField(msg->data, "current_model"));
  const std::string switch_state = toLowerCopy(extractJsonStringField(msg->data, "switch_state"));

  std::scoped_lock<std::mutex> lock(target_mutex_);
  model_switch_active_ = !switch_state.empty() && switch_state != "idle";
  if (current_model.empty()) {
    return;
  }
  ActiveModelMode next_mode = ActiveModelMode::Rough;
  if (current_model == "crawl" || current_model == "ik") {
    next_mode = ActiveModelMode::Crawl;
  } else if (current_model == "wall") {
    next_mode = ActiveModelMode::Wall;
  }
  if (next_mode == active_model_mode_) {
    return;
  }

  active_model_mode_ = next_mode;
  const char * mode_name = "rough";
  if (active_model_mode_ == ActiveModelMode::Crawl) {
    active_default_dof_pos_ = crawl_default_dof_pos_;
    mode_name = "ik/crawl";
  } else if (active_model_mode_ == ActiveModelMode::Wall) {
    active_default_dof_pos_ = wall_default_dof_pos_;
    mode_name = "wall";
  } else {
    active_default_dof_pos_ = rough_default_dof_pos_;
  }
  RCLCPP_INFO(
    get_logger(),
    "Hardware active model reference updated to %s pose.",
    mode_name);
}

void HardwareBridgeNode::onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(imu_mutex_);

  const float gyro_x = static_cast<float>(msg->angular_velocity.x);
  const float gyro_y = static_cast<float>(msg->angular_velocity.y);
  const float gyro_z = static_cast<float>(msg->angular_velocity.z);

  const float accel_x = static_cast<float>(msg->linear_acceleration.x);
  const float accel_y = static_cast<float>(msg->linear_acceleration.y);
  const float accel_z = static_cast<float>(msg->linear_acceleration.z);

  imu_gyro_ = {gyro_x, gyro_y, gyro_z};
  imu_accel_ = {accel_x, accel_y, accel_z};
  if (!mahony_initialized_.load(std::memory_order_relaxed) &&
      imu_gravity_sample_count_ < kImuGravityAlignSamples) {
    imu_gravity_sum_[0] += accel_x;
    imu_gravity_sum_[1] += accel_y;
    imu_gravity_sum_[2] += accel_z;
    imu_gravity_sample_count_++;
  }
  // Track both ROS header time and local receive time. The local steady clock
  // is used for stale detection so scheduler jitter or device timestamp quirks
  // don't falsely trip the runtime guard.
  last_imu_stamp_ = rclcpp::Time(msg->header.stamp);
  last_imu_recv_time_ = std::chrono::steady_clock::now();
  has_received_imu_ = true;
  imu_fresh_ = true;
}

void HardwareBridgeNode::onEstop(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(target_mutex_);
  const bool was_estop = estop_triggered_;
  estop_triggered_ = msg->data;
  if (estop_triggered_) {
    RCLCPP_WARN(get_logger(), "!!! E-stop received over /safety/estop !!!");
    logEvent("WARN", "estop_triggered", "E-stop received over /safety/estop.");
  } else {
    RCLCPP_INFO(get_logger(), "E-stop reset.");
    logEvent("INFO", "estop_reset", "E-stop reset via /safety/estop.");
    const bool user_estop_latch =
      safety_reason_.find("user E-stop") != std::string::npos ||
      (was_estop && safety_reason_.empty());
    if (safety_triggered_ && user_estop_latch) {
      safety_triggered_ = false;
      safety_reason_.clear();
      clip_active_logged_ = false;
      RCLCPP_INFO(get_logger(), "Cleared user E-stop safety latch.");
      logEvent("INFO", "safety_latch_reset", "Cleared user E-stop safety latch.");
    }
  }
}

void HardwareBridgeNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(odom_mutex_);
  last_odom_stamp_ = rclcpp::Time(msg->header.stamp);

  odom_pos_[0] = static_cast<float>(msg->pose.pose.position.x);
  odom_pos_[1] = static_cast<float>(msg->pose.pose.position.y);
  odom_pos_[2] = static_cast<float>(msg->pose.pose.position.z);

  odom_quat_wxyz_[0] = static_cast<float>(msg->pose.pose.orientation.w);
  odom_quat_wxyz_[1] = static_cast<float>(msg->pose.pose.orientation.x);
  odom_quat_wxyz_[2] = static_cast<float>(msg->pose.pose.orientation.y);
  odom_quat_wxyz_[3] = static_cast<float>(msg->pose.pose.orientation.z);

  odom_linear_vel_[0] = static_cast<float>(msg->twist.twist.linear.x);
  odom_linear_vel_[1] = static_cast<float>(msg->twist.twist.linear.y);
  odom_linear_vel_[2] = static_cast<float>(msg->twist.twist.linear.z);

  odom_angular_vel_[0] = static_cast<float>(msg->twist.twist.angular.x);
  odom_angular_vel_[1] = static_cast<float>(msg->twist.twist.angular.y);
  odom_angular_vel_[2] = static_cast<float>(msg->twist.twist.angular.z);

  odom_fresh_ = true;
}

void HardwareBridgeNode::onReadLoop()
{
  // 1. Process CAN messages (only if CAN is open)
  if (!dry_run_) {
    for (std::size_t i = 0; i < 16; ++i) {
      motor_states_[i].stale_count++;
    }

    drainCanFrames(can0_fd_, 1, 50);
    drainCanFrames(can1_fd_, 2, 50);

    // Hold-over: apply last valid data for stale motors
    for (std::size_t i = 0; i < 16; ++i) {
      if (motor_states_[i].stale_count >= kHoldoverThreshold && motor_states_[i].has_valid_data) {
        motor_states_[i].position = motor_states_[i].last_valid_pos;
        motor_states_[i].velocity = motor_states_[i].last_valid_vel;
        motor_states_[i].torque = motor_states_[i].last_valid_torque;
        holdover_events_total_++;
        if (!motor_states_[i].stale_reported &&
            motor_states_[i].stale_count >= kMotorDropReportThreshold) {
          motor_states_[i].stale_reported = true;
          motor_states_[i].recovered_reported = false;
          motor_states_[i].disable_reported = true;
          motor_states_[i].last_power_event_reason =
            "no status frame received, stale_count=" + std::to_string(motor_states_[i].stale_count);
          motor_drop_event_count_++;
          logMotorPowerEvent(i, "dropped_or_unresponsive", motor_states_[i].last_power_event_reason);
        }
        if (shouldAttemptMotorRecovery(i)) {
          attemptMotorRecovery(i, "stale_feedback");
        }
      } else if (motor_states_[i].stale_count == 0 && motor_states_[i].stale_reported) {
        motor_states_[i].stale_reported = false;
        if (!motor_states_[i].recovered_reported) {
          motor_states_[i].recovered_reported = true;
          motor_recover_event_count_++;
          logMotorPowerEvent(i, "recovered", "status frame reception resumed");
        }
      }

      processMotorRecoverySequence(i);

      if (motor_states_[i].stale_count == 0) {
        updateNoEffectDetection(i);
        if (shouldAttemptNoEffectRecovery(i)) {
          attemptNoEffectRecovery(i, "wheel_no_effect");
        }
      }
    }
  }

  pollMotorDiagnostics();

  // 2. Fetch IMU data & update MahonyFilter
  auto now_time = now();
  double dt = 0.005;
  if (last_read_time_.nanoseconds() > 0) {
    dt = (now_time - last_read_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) {
      dt = 0.005;
    }
  }
  last_read_time_ = now_time;

  std::array<float, 3> gyro{};
  std::array<float, 3> accel{0.0f, 0.0f, 9.81f};
  bool imu_fresh = false;
  double imu_age_ms = 0.0;
  {
    std::scoped_lock<std::mutex> lock(imu_mutex_);
    gyro = imu_gyro_;
    accel = imu_accel_;
    imu_fresh = imu_fresh_;
    imu_fresh_ = false;

    if (has_received_imu_) {
      const auto age = std::chrono::steady_clock::now() - last_imu_recv_time_;
      imu_age_ms = std::chrono::duration<double, std::milli>(age).count();
    } else if (last_imu_stamp_.nanoseconds() > 0) {
      const auto age_ns = (now_time - last_imu_stamp_).nanoseconds();
      imu_age_ms = age_ns > 0 ? static_cast<double>(age_ns) / 1.0e6 : 0.0;
    }
  }

  std::array<float, 4> quat{1.0f, 0.0f, 0.0f, 0.0f};
  const bool mahony_initialized = mahony_initialized_.load(std::memory_order_relaxed);
  if (imu_fresh || mahony_initialized) {
    if (!mahony_initialized) {
      std::array<float, 3> gravity_init = accel;
      {
        std::scoped_lock<std::mutex> lock(imu_mutex_);
        if (imu_gravity_sample_count_ >= kImuGravityAlignSamples) {
          gravity_init = {
            imu_gravity_sum_[0] / static_cast<float>(imu_gravity_sample_count_),
            imu_gravity_sum_[1] / static_cast<float>(imu_gravity_sample_count_),
            imu_gravity_sum_[2] / static_cast<float>(imu_gravity_sample_count_)
          };
        }
      }
      mahony_filter_->reset_with_accel(gravity_init);
      mahony_initialized_.store(true, std::memory_order_relaxed);
    }
    quat = mahony_filter_->update(accel, gyro, static_cast<float>(dt));
  }
  std::array<float, 3> projected_gravity = sim2real_common::get_gravity_orientation(quat);
  {
    std::scoped_lock<std::mutex> lock(imu_mutex_);
    projected_gravity_ = projected_gravity;
  }

  // Run RuntimeGuard check
  if (safety_enabled_ && !safety_triggered_) {
    std::vector<float> extra_vals;
    extra_vals.reserve(32);
    for (std::size_t i = 0; i < 16; ++i) {
      extra_vals.push_back(motor_states_[i].position);
      extra_vals.push_back(motor_states_[i].velocity);
    }
    bool estop_active = false;
    {
      std::scoped_lock<std::mutex> lock(target_mutex_);
      estop_active = estop_triggered_;
    }

    auto guard_decision = runtime_guard_->check(gyro, projected_gravity, imu_age_ms, estop_active, extra_vals);
    if (guard_decision.level == sim2real_common::GuardLevel::STOP) {
      safety_triggered_ = true;
      safety_reason_ = "Runtime Guard Stop: " + guard_decision.reason;
      RCLCPP_ERROR(get_logger(), "SAFETY TRIGGERED: %s", safety_reason_.c_str());
      logEvent("ERROR", "safety_triggered", safety_reason_);
      logProtectionEvent("runtime_guard_stop", guard_decision.reason, "safety_brake");
    } else if (guard_decision.level == sim2real_common::GuardLevel::WARN) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Safety Guard Warning: %s", guard_decision.reason.c_str());
    }
  }

  // 3. Populate RuntimeState message
  sim2real_interfaces::msg::RuntimeState msg;
  msg.stamp = now_time;
  msg.sequence = state_sequence_++;
  msg.source = dry_run_ ? "stub_hw" : "socket_can_hw";

  fresh_count_ = 0;
  holdover_count_ = 0;
  stale_max_ = 0;
  std::array<float, 16> dry_run_target_snapshot{};
  std::uint32_t dry_run_target_sequence = 0;
  if (dry_run_) {
    std::scoped_lock<std::mutex> lock(target_mutex_);
    dry_run_target_snapshot = latest_target_;
    dry_run_target_sequence = target_sequence_;
  }

  for (std::size_t i = 0; i < 16; ++i) {
    // Stale frames detection & holdover count
    if (!dry_run_) {
      if (motor_states_[i].stale_count > 0) {
        holdover_count_++;
        stale_max_ = std::max(stale_max_, motor_states_[i].stale_count);
      } else {
        fresh_count_++;
      }
    }
    
    if (dry_run_) {
      // Mock motor positions tracking target
      msg.joint_pos[i] = dry_run_target_snapshot[i];
      msg.joint_vel[i] = 0.0f;
      msg.joint_torque[i] = 0.0f;
      msg.update_counts[i] = dry_run_target_sequence;
    } else {
      msg.joint_pos[i] = motor_states_[i].position;
      msg.joint_vel[i] = motor_states_[i].velocity;
      msg.joint_torque[i] = motor_states_[i].torque;
      msg.update_counts[i] = motor_states_[i].update_count;
    }
  }

  msg.imu_gyro = gyro;
  msg.imu_accel = accel;
  msg.quat_wxyz = quat;
  msg.projected_gravity = projected_gravity;
  msg.imu_age_ms = imu_age_ms;
  msg.imu_fresh = imu_fresh || (imu_age_ms < 60.0); // Allow brief staleness

  msg.odom_age_ms = 0.0f;
  msg.odom_fresh = false;
  msg.odom_pos = {0.0f, 0.0f, 0.0f};
  msg.odom_quat_wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
  msg.odom_linear_vel = {0.0f, 0.0f, 0.0f};
  msg.odom_angular_vel = {0.0f, 0.0f, 0.0f};
  msg.odom_local_pos = {0.0f, 0.0f, 0.0f};
  msg.odom_local_yaw = 0.0f;

  // Populate odom fields from subscriber data
  {
    std::scoped_lock<std::mutex> lock(odom_mutex_);
    if (odom_fresh_) {
      double odom_age = (now_time - last_odom_stamp_).seconds() * 1000.0;
      msg.odom_age_ms = static_cast<float>(odom_age);
      msg.odom_fresh = (odom_age < 200.0);  // 200ms threshold
      msg.odom_pos = odom_pos_;
      msg.odom_quat_wxyz = odom_quat_wxyz_;
      msg.odom_linear_vel = odom_linear_vel_;
      msg.odom_angular_vel = odom_angular_vel_;
      msg.odom_local_pos = odom_pos_;
      // Compute yaw from quaternion
      float qw = odom_quat_wxyz_[0], qx = odom_quat_wxyz_[1];
      float qy = odom_quat_wxyz_[2], qz = odom_quat_wxyz_[3];
      float siny_c = 2.0f * (qw * qz + qx * qy);
      float cosy_c = 1.0f - 2.0f * (qy * qy + qz * qz);
      msg.odom_local_yaw = std::atan2(siny_c, cosy_c);
    }
  }

  msg.fresh_count = dry_run_ ? 16 : fresh_count_;
  msg.holdover_count = dry_run_ ? 0 : holdover_count_;
  msg.stale_max = dry_run_ ? 0 : stale_max_;

  state_pub_->publish(msg);
}

void HardwareBridgeNode::onWriteLoop()
{
  const auto now_time = now();
  std::array<float, 16> target{};
  std::array<float, 16> active_default_pose = rough_default_dof_pos_;
  std::string target_source;
  bool model_switch_active = false;
  rclcpp::Time latest_target_stamp{0, 0, RCL_ROS_TIME};
  double age_ms = 0.0;

  {
    std::scoped_lock<std::mutex> lock(target_mutex_);
    target = latest_target_;
    target_source = latest_target_source_;
    active_default_pose = active_default_dof_pos_;
    model_switch_active = model_switch_active_;
    latest_target_stamp = latest_target_stamp_;
    
    if (latest_target_stamp.nanoseconds() > 0) {
      const auto age_ns = (now_time - latest_target_stamp).nanoseconds();
      age_ms = age_ns > 0 ? static_cast<double>(age_ns) / 1.0e6 : 0.0;
    }
  }

  if (target_source == "model_switch_to_model_pose") {
    active_default_pose = rough_default_dof_pos_;
  }
  const bool target_is_model_switch =
    target_source.rfind("model_switch_", 0) == 0;

  // Timeout guard: fall back to the active model's reference pose if target is stale
  if (latest_target_stamp.nanoseconds() == 0 || age_ms > target_timeout_ms_) {
    target = active_default_pose;
    target_source = "timeout_hold";
    if (!timeout_hold_logged_) {
      timeout_hold_logged_ = true;
      logEvent("WARN", "target_timeout_hold",
        "runtime target stale, switched to timeout_hold, age_ms=" + std::to_string(age_ms));
    }
  } else {
    timeout_hold_logged_ = false;
  }

  // Run SafetyMonitor check on incoming target commands
  std::array<float, 3> gyro{};
  std::array<float, 3> proj_grav{};
  bool estop_active = false;
  {
    std::scoped_lock<std::mutex> lock(imu_mutex_);
    gyro = imu_gyro_;
    proj_grav = projected_gravity_;
  }
  {
    std::scoped_lock<std::mutex> lock(target_mutex_);
    estop_active = estop_triggered_;
  }

  if (safety_enabled_ && !safety_triggered_) {
    auto * active_safety_monitor =
      (model_switch_active || target_is_model_switch) && model_switch_safety_monitor_ ?
      model_switch_safety_monitor_.get() :
      safety_monitor_.get();
    auto safety_decision = active_safety_monitor->check(target, active_default_pose, gyro, proj_grav, estop_active);
    if (safety_decision.level == sim2real_common::SafetyLevel::ESTOP || safety_decision.level == sim2real_common::SafetyLevel::BRAKE) {
      safety_triggered_ = true;
      safety_reason_ = "Safety Monitor Stop: " + safety_decision.message;
      RCLCPP_ERROR(get_logger(), "SAFETY TRIGGERED: %s", safety_reason_.c_str());
      logEvent("ERROR", "safety_triggered", safety_reason_);
      logProtectionEvent("safety_monitor_stop", safety_decision.message, "safety_brake");
    } else if (safety_decision.level == sim2real_common::SafetyLevel::CLIP) {
      target = safety_decision.clipped_target;
      target_source = "safety_clip";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Safety Monitor: Joint target clipped.");
      if (!clip_active_logged_) {
        clip_active_logged_ = true;
        logEvent("WARN", "target_clipped",
          "protection=safety_clip, reason=" + safety_decision.message + ", source=" + target_source);
      }
    } else {
      clip_active_logged_ = false;
    }
  }

  // Override to safety_brake if safety is triggered locally or by estop
  if (safety_triggered_) {
    target_source = "safety_brake";
  }

  // 1. Joint LPF command filtering (200Hz, dt=0.005s)
  std::array<float, 12> legs_in{};
  std::array<float, 12> legs_out{};
  std::array<float, 4> wheels_in{};
  std::array<float, 4> wheels_out{};

  std::copy(target.begin(), target.begin() + 12, legs_in.begin());
  std::copy(target.begin() + 12, target.end(), wheels_in.begin());

  lpf_legs_->filter(legs_in.data(), legs_out.data());
  lpf_wheels_->filter(wheels_in.data(), wheels_out.data());

  std::array<float, 16> filtered_target{};
  std::copy(legs_out.begin(), legs_out.end(), filtered_target.begin());
  std::copy(wheels_out.begin(), wheels_out.end(), filtered_target.begin() + 12);

  // 2. Control execution (MIT mode write over CAN)
  if (!dry_run_) {
    for (std::size_t i = 0; i < 16; ++i) {
      // Coordinate transform: sim_to_real
      // real = sign * sim + offset
      float sim_val = filtered_target[i];
      float real_val = motors_[i].direction * sim_val + motors_[i].offset;

      int fd = (motors_[i].bus == 1) ? can0_fd_ : can1_fd_;

      if (i < 12) {
        // Leg joints: MIT position control
        // Kp & Kd depend on whether we are holding pose, running policy, or in safety damping mode
        double kp_val = sim2real_common::DeploymentContract::kLegKp;
        double kd_val = sim2real_common::DeploymentContract::kLegKd;
        
        if (target_source == "safety_brake" || target_source == "safety_estop") {
          kp_val = 0.0;
          kd_val = 2.5; // Leg damping Kd
          real_val = 0.0; // Set to zero position (sign/offset will be ignored anyway under kp=0)
        } else if (target_source == "startup_soft_hold") {
          if (startup_soft_hold_start_time_.nanoseconds() == 0) {
            startup_soft_hold_start_time_ = now_time;
          }
          double elapsed = (now_time - startup_soft_hold_start_time_).seconds();
          double kp_scale = 0.125 + (1.0 - 0.125) * std::min(1.0, elapsed / 1.0); // 1.0s ramp
          kp_val = sim2real_common::DeploymentContract::kLegKp * kp_scale;
          kd_val = sim2real_common::DeploymentContract::kLegKd;
        } else {
          startup_soft_hold_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          if (
            target_source == "timeout_hold" ||
            target_source == "boot_hold" ||
            target_source == "runtime_zero_hold" ||
            target_source == "runtime_keep_pose" ||
            target_source == "runtime_keep_return_default" ||
            target_source == "startup_hold" ||
            target_source == "model_switch_to_stand" ||
            target_source == "model_switch_stand_hold" ||
            target_source == "model_switch_to_model_pose")
          {
            kp_val = sim2real_common::DeploymentContract::kLegKp;
            kd_val = sim2real_common::DeploymentContract::kLegKd;
          }
        }
        updateMotorCommandTracking(i, sim_val, target_source);
        writeOperationFrame(fd, motors_[i].id, real_val, 0.0, kp_val, kd_val, 0.0);
      } else {
        // Wheel joints: MIT velocity control (Kp = 0, Kd = kWheelKd, velocity = target, position = 0)
        double vel_real = motors_[i].direction * sim_val; // Wheels actions are in velocity, apply sign
        double kd_val = sim2real_common::DeploymentContract::kWheelKd;
        if (target_source == "safety_brake" || target_source == "safety_estop") {
          vel_real = 0.0;
          kd_val = 2.0; // Wheel damping Kd
        }
        updateMotorCommandTracking(i, sim_val, target_source);
        writeOperationFrame(fd, motors_[i].id, 0.0, vel_real, 0.0, kd_val, 0.0);
      }
    }
  }
}

bool HardwareBridgeNode::enableMotor(int fd, int motor_id)
{
  std::uint32_t ext_id = (COMM_ENABLE << 24) | (HOST_ID << 8) | motor_id;
  std::uint8_t data[8] = {0};
  return sendCanFrame(fd, ext_id, data, 8);
}

bool HardwareBridgeNode::disableMotor(int fd, int motor_id, bool clear_fault)
{
  std::uint32_t ext_id = (COMM_DISABLE << 24) | (HOST_ID << 8) | motor_id;
  std::uint8_t data[8] = {0};
  data[0] = clear_fault ? 1 : 0;
  return sendCanFrame(fd, ext_id, data, 8);
}

bool HardwareBridgeNode::readParameter(int fd, int motor_id, std::uint16_t param_id)
{
  std::uint32_t ext_id = (COMM_READ_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
  std::uint8_t data[8] = {0};
  data[0] = param_id & 0xFF;
  data[1] = (param_id >> 8) & 0xFF;
  return sendCanFrame(fd, ext_id, data, 8);
}

bool HardwareBridgeNode::setModeRaw(int fd, int motor_id, std::int8_t mode)
{
  return writeParameterInt(fd, motor_id, PARAM_MODE, static_cast<std::uint8_t>(mode));
}

bool HardwareBridgeNode::writeLimit(int fd, int motor_id, std::uint16_t param_id, float limit)
{
  std::uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
  std::uint8_t data[8] = {0};
  data[0] = param_id & 0xFF;
  data[1] = (param_id >> 8) & 0xFF;
  std::memcpy(&data[4], &limit, sizeof(float));
  return sendCanFrame(fd, ext_id, data, 8);
}

bool HardwareBridgeNode::writeParameterInt(int fd, int motor_id, std::uint16_t param_id, std::uint32_t value)
{
  std::uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
  std::uint8_t data[8] = {0};
  data[0] = param_id & 0xFF;
  data[1] = (param_id >> 8) & 0xFF;
  std::memcpy(&data[4], &value, sizeof(std::uint32_t));
  return sendCanFrame(fd, ext_id, data, 8);
}

bool HardwareBridgeNode::writeOperationFrame(int fd, int motor_id, double pos, double vel, double kp_val, double kd_val, double torque)
{
  const double P_LIMIT = 4.0 * M_PI;
  const double V_LIMIT = 44.0;
  const double T_LIMIT = 17.0;
  const double KP_LIMIT = 500.0;
  const double KD_LIMIT = 5.0;

  double pos_clamped = std::max(-P_LIMIT, std::min(P_LIMIT, pos));
  double vel_clamped = std::max(-V_LIMIT, std::min(V_LIMIT, vel));
  double kp_clamped = std::max(0.0, std::min(KP_LIMIT, kp_val));
  double kd_clamped = std::max(0.0, std::min(KD_LIMIT, kd_val));
  double torque_clamped = std::max(-T_LIMIT, std::min(T_LIMIT, torque));

  std::uint16_t pos_u16 = static_cast<std::uint16_t>(((pos_clamped / P_LIMIT) + 1.0) * 32767.0);
  std::uint16_t vel_u16 = static_cast<std::uint16_t>(((vel_clamped / V_LIMIT) + 1.0) * 32767.0);
  std::uint16_t kp_u16 = static_cast<std::uint16_t>((kp_clamped / KP_LIMIT) * 65535.0);
  std::uint16_t kd_u16 = static_cast<std::uint16_t>((kd_clamped / KD_LIMIT) * 65535.0);
  std::uint16_t torque_u16 = static_cast<std::uint16_t>(((torque_clamped / T_LIMIT) + 1.0) * 32767.0);

  std::uint8_t data[8];
  pack_u16_be(&data[0], pos_u16);
  pack_u16_be(&data[2], vel_u16);
  pack_u16_be(&data[4], kp_u16);
  pack_u16_be(&data[6], kd_u16);

  std::uint32_t ext_id = (COMM_OPERATION_CONTROL << 24) | (torque_u16 << 8) | motor_id;
  return sendCanFrame(fd, ext_id, data, 8);
}

bool HardwareBridgeNode::initCan(const std::string& ifname, int& fd)
{
  struct sockaddr_can addr;
  struct ifreq ifr;

  if ((fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to create SocketCAN socket for %s", ifname.c_str());
    return false;
  }

  // Set non-blocking mode
  int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to set socket to non-blocking for %s", ifname.c_str());
    ::close(fd);
    fd = -1;
    return false;
  }

  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to ioctl SIOCGIFINDEX for %s", ifname.c_str());
    ::close(fd);
    fd = -1;
    return false;
  }

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to bind SocketCAN socket for %s", ifname.c_str());
    ::close(fd);
    fd = -1;
    return false;
  }

  RCLCPP_INFO(get_logger(), "Successfully bound to SocketCAN interface %s", ifname.c_str());
  return true;
}

bool HardwareBridgeNode::sendCanFrame(int fd, std::uint32_t can_id, const std::uint8_t* data, std::uint8_t dlc)
{
  if (fd < 0) return false;
  struct can_frame frame;
  frame.can_id = can_id | CAN_EFF_FLAG; // Extended frame format (29-bit CAN ID)
  frame.can_dlc = dlc;
  if (data) {
    std::memcpy(frame.data, data, dlc);
  } else {
    std::memset(frame.data, 0, 8);
  }

  ssize_t bytes_written = ::write(fd, &frame, sizeof(struct can_frame));
  if (bytes_written != sizeof(struct can_frame)) {
    int err = errno;
    // Track errors per bus for recovery logic
    if (fd == can0_fd_) {
      can0_error_count_++;
      if (can0_error_count_ >= kCanErrorThreshold) {
        RCLCPP_ERROR(get_logger(), "CAN0 write: %d consecutive errors (errno=%d: %s). Attempting reinit.",
          can0_error_count_, err, strerror(err));
        if (!reinitCan(can0_name_, can0_fd_, can0_error_count_)) {
          RCLCPP_FATAL(get_logger(), "CAN0 reinit failed! Triggering safety brake.");
          safety_triggered_ = true;
          safety_reason_ = "CAN0 bus failure - reinit failed";
          logEvent("FATAL", "safety_triggered", safety_reason_);
        }
      }
    } else if (fd == can1_fd_) {
      can1_error_count_++;
      if (can1_error_count_ >= kCanErrorThreshold) {
        RCLCPP_ERROR(get_logger(), "CAN1 write: %d consecutive errors (errno=%d: %s). Attempting reinit.",
          can1_error_count_, err, strerror(err));
        if (!reinitCan(can1_name_, can1_fd_, can1_error_count_)) {
          RCLCPP_FATAL(get_logger(), "CAN1 reinit failed! Triggering safety brake.");
          safety_triggered_ = true;
          safety_reason_ = "CAN1 bus failure - reinit failed";
          logEvent("FATAL", "safety_triggered", safety_reason_);
        }
      }
    }
    return false;
  }
  // Reset error count on success
  if (fd == can0_fd_) can0_error_count_ = 0;
  else if (fd == can1_fd_) can1_error_count_ = 0;
  return true;
}

bool HardwareBridgeNode::readCanFrame(int fd, void* frame_ptr, int timeout_us)
{
  if (fd < 0) return false;
  auto* frame = static_cast<struct can_frame*>(frame_ptr);

  if (timeout_us > 0) {
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout_us;
    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(fd, &rdfs);

    int ret = ::select(fd + 1, &rdfs, nullptr, nullptr, &tv);
    if (ret < 0) {
      int err = errno;
      if (err != EINTR) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "CAN select error (fd=%d): %s", fd, strerror(err));
      }
      return false;
    }
    if (ret == 0) {
      return false; // timeout, normal
    }
  }

  ssize_t bytes_read = ::read(fd, frame, sizeof(struct can_frame));
  if (bytes_read < 0) {
    int err = errno;
    if (err != EAGAIN && err != EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "CAN read error (fd=%d): %s", fd, strerror(err));
    }
    return false;
  }
  return (bytes_read == sizeof(struct can_frame));
}

bool HardwareBridgeNode::reinitCan(const std::string& ifname, int& fd, int& error_count)
{
  RCLCPP_WARN(get_logger(), "Attempting to reinitialize CAN interface: %s", ifname.c_str());
  logEvent("WARN", "can_reinit_attempt", "interface=" + ifname + ", consecutive_errors=" + std::to_string(error_count));
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
  bool success = initCan(ifname, fd);
  if (success) {
    error_count = 0;
    const int recovered_bus = (&fd == &can0_fd_) ? 1 : 2;
    if (!initializeMotorsOnBus(recovered_bus, "can_reinit")) {
      logEvent("ERROR", "motor_reinit_after_can_reinit_failed",
        "interface=" + ifname + ", bus=" + std::to_string(recovered_bus));
    }
    RCLCPP_INFO(get_logger(), "CAN interface %s reinitialized successfully.", ifname.c_str());
    logEvent("INFO", "can_reinit_success", "interface=" + ifname);
  } else {
    logEvent("ERROR", "can_reinit_failed", "interface=" + ifname);
  }
  return success;
}

}  // namespace sim2real_hw

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sim2real_hw::HardwareBridgeNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
