#include "sim2real_hw/hardware_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "sim2real_common/deployment_contract.hpp"

using namespace std::chrono_literals;

namespace sim2real_hw
{

// Protocol constants
const std::uint32_t COMM_ENABLE = 3;
const std::uint32_t COMM_DISABLE = 4;
const std::uint32_t COMM_WRITE_PARAMETER = 18;
const std::uint32_t COMM_OPERATION_CONTROL = 1;
const std::uint32_t COMM_SET_ZERO_POSITION = 6;
const std::uint16_t PARAM_MODE = 0x7005;
const std::uint16_t PARAM_VELOCITY_LIMIT = 0x7017;
const std::uint16_t PARAM_TORQUE_LIMIT = 0x700B;
const std::uint8_t HOST_ID = 0xFD;

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

HardwareBridgeNode::HardwareBridgeNode()
: Node("sim2real_hw_node")
{
  // 1. Declare and get parameters
  target_timeout_ms_ = declare_parameter<double>("target_timeout_ms", 150.0);
  can0_name_ = declare_parameter<std::string>("can0_name", "can0");
  can1_name_ = declare_parameter<std::string>("can1_name", "can1");
  dry_run_ = declare_parameter<bool>("dry_run", true); // Default to dry-run for safety

  // Safety parameters
  safety_enabled_ = declare_parameter<bool>("safety_enabled", true);
  double max_target_offset = declare_parameter<double>("max_target_offset", 0.6);
  double hard_target_offset = declare_parameter<double>("hard_target_offset", 1.2);
  double max_ang_vel = declare_parameter<double>("max_ang_vel", 10.0);
  double max_tilt_z = declare_parameter<double>("max_tilt_z", -0.3);
  int clip_to_brake = declare_parameter<int>("clip_to_brake", 0);
  double imu_age_warn_ms = declare_parameter<double>("imu_age_warn_ms", 60.0);
  double imu_age_stop_ms = declare_parameter<double>("imu_age_stop_ms", 200.0);

  RCLCPP_INFO(get_logger(), "Initializing hardware bridge node (Dry run: %s)", dry_run_ ? "true" : "false");
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
  latest_target_ = sim2real_common::DeploymentContract::kDefaultDofPos;
  latest_raw_action_.fill(0.0f);

  // 6. Set up ROS publishers & subscriptions
  state_pub_ = create_publisher<sim2real_interfaces::msg::RuntimeState>("runtime/state", 10);
  target_sub_ = create_subscription<sim2real_interfaces::msg::RuntimeTarget>(
    "runtime/target", 10,
    std::bind(&HardwareBridgeNode::onTarget, this, std::placeholders::_1));
  std::string imu_topic = declare_parameter<std::string>("imu_topic", "/odin1/imu");
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, 10,
    std::bind(&HardwareBridgeNode::onImu, this, std::placeholders::_1));
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/safety/estop", 10,
    std::bind(&HardwareBridgeNode::onEstop, this, std::placeholders::_1));

  // Odom subscription
  std::string odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    std::bind(&HardwareBridgeNode::onOdom, this, std::placeholders::_1));

  // 7. Enable motors on total startup
  if (!dry_run_) {
    RCLCPP_INFO(get_logger(), "Enabling RobStride motors...");
    for (std::size_t i = 0; i < 16; ++i) {
      int fd = (motors_[i].bus == 1) ? can0_fd_ : can1_fd_;
      enableMotor(fd, motors_[i].id);
      setModeRaw(fd, motors_[i].id, 0); // MIT Mode
      writeLimit(fd, motors_[i].id, PARAM_VELOCITY_LIMIT, 20.0f);
      writeLimit(fd, motors_[i].id, PARAM_TORQUE_LIMIT, 17.0f);
    }
  }

  // 8. Timers at 200Hz (5ms)
  read_timer_ = create_wall_timer(5ms, std::bind(&HardwareBridgeNode::onReadLoop, this));
  write_timer_ = create_wall_timer(5ms, std::bind(&HardwareBridgeNode::onWriteLoop, this));
}

HardwareBridgeNode::~HardwareBridgeNode()
{
  if (!dry_run_) {
    RCLCPP_INFO(get_logger(), "Disabling RobStride motors on shutdown...");
    for (std::size_t i = 0; i < 16; ++i) {
      int fd = (motors_[i].bus == 1) ? can0_fd_ : can1_fd_;
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
  if (!mahony_initialized_ && imu_gravity_sample_count_ < kImuGravityAlignSamples) {
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
  estop_triggered_ = msg->data;
  if (estop_triggered_) {
    RCLCPP_WARN(get_logger(), "!!! Physical E-stop received over /safety/estop !!!");
  } else {
    RCLCPP_INFO(get_logger(), "Physical E-stop reset.");
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

    struct can_frame frame;
    // Process can0 (bus 1)
    while (readCanFrame(can0_fd_, &frame, 50)) {
      if (!(frame.can_id & CAN_EFF_FLAG)) continue;
      std::uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
      if (comm_type == 2) { // Status Frame
        std::uint32_t extra_data = (frame.can_id >> 8) & 0xFFFF;
        int motor_id = extra_data & 0xFF;
        
        for (std::size_t i = 0; i < 16; ++i) {
          if (motors_[i].bus == 1 && motors_[i].id == motor_id) {
            std::uint16_t p_u16 = (frame.data[0] << 8) | frame.data[1];
            std::uint16_t v_u16 = (frame.data[2] << 8) | frame.data[3];
            std::uint16_t t_u16 = (frame.data[4] << 8) | frame.data[5];
            std::uint16_t temp_u16 = (frame.data[6] << 8) | frame.data[7];

            double pos_raw = (static_cast<double>(p_u16) / 32767.0 - 1.0) * (4.0 * M_PI);
            double vel_raw = (static_cast<double>(v_u16) / 32767.0 - 1.0) * 44.0;
            double torque_raw = (static_cast<double>(t_u16) / 32767.0 - 1.0) * 17.0;

            // Apply motor mapping: real_to_sim
            // real = sign * sim + offset -> sim = (real - offset) / sign
            float pos_sim = (static_cast<float>(pos_raw) - motors_[i].offset) / motors_[i].direction;
            float vel_sim = static_cast<float>(vel_raw) / motors_[i].direction;
            float torque_sim = static_cast<float>(torque_raw) / motors_[i].direction;

            if (i < 12) {
              pos_sim = nearest_periodic(pos_sim, sim2real_common::DeploymentContract::kDefaultDofPos[i]);
            }

            motor_states_[i].position = pos_sim;
            motor_states_[i].velocity = vel_sim;
            motor_states_[i].torque = torque_sim;
            motor_states_[i].temperature = static_cast<float>(temp_u16) * 0.1f;
            motor_states_[i].update_count++;
            motor_states_[i].stale_count = 0;
            // Update hold-over valid data
            motor_states_[i].last_valid_pos = pos_sim;
            motor_states_[i].last_valid_vel = vel_sim;
            motor_states_[i].last_valid_torque = torque_sim;
            motor_states_[i].has_valid_data = true;
            break;
          }
        }
      }
    }

    // Process can1 (bus 2)
    while (readCanFrame(can1_fd_, &frame, 50)) {
      if (!(frame.can_id & CAN_EFF_FLAG)) continue;
      std::uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
      if (comm_type == 2) {
        std::uint32_t extra_data = (frame.can_id >> 8) & 0xFFFF;
        int motor_id = extra_data & 0xFF;

        for (std::size_t i = 0; i < 16; ++i) {
          if (motors_[i].bus == 2 && motors_[i].id == motor_id) {
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
              pos_sim = nearest_periodic(pos_sim, sim2real_common::DeploymentContract::kDefaultDofPos[i]);
            }

            motor_states_[i].position = pos_sim;
            motor_states_[i].velocity = vel_sim;
            motor_states_[i].torque = torque_sim;
            motor_states_[i].temperature = static_cast<float>(temp_u16) * 0.1f;
            motor_states_[i].update_count++;
            motor_states_[i].stale_count = 0;
            // Update hold-over valid data
            motor_states_[i].last_valid_pos = pos_sim;
            motor_states_[i].last_valid_vel = vel_sim;
            motor_states_[i].last_valid_torque = torque_sim;
            motor_states_[i].has_valid_data = true;
            break;
          }
        }
      }
    }

    // Hold-over: apply last valid data for stale motors
    for (std::size_t i = 0; i < 16; ++i) {
      if (motor_states_[i].stale_count >= kHoldoverThreshold && motor_states_[i].has_valid_data) {
        motor_states_[i].position = motor_states_[i].last_valid_pos;
        motor_states_[i].velocity = motor_states_[i].last_valid_vel;
        motor_states_[i].torque = motor_states_[i].last_valid_torque;
        holdover_events_total_++;
      }
    }
  }

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
  if (imu_fresh || mahony_initialized_) {
    if (!mahony_initialized_) {
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
      mahony_initialized_ = true;
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
      msg.joint_pos[i] = latest_target_[i];
      msg.joint_vel[i] = 0.0f;
      msg.joint_torque[i] = 0.0f;
      msg.update_counts[i] = target_sequence_;
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
  std::string target_source;
  double age_ms = 0.0;

  {
    std::scoped_lock<std::mutex> lock(target_mutex_);
    target = latest_target_;
    target_source = latest_target_source_;
    
    if (latest_target_stamp_.nanoseconds() > 0) {
      const auto age_ns = (now_time - latest_target_stamp_).nanoseconds();
      age_ms = age_ns > 0 ? static_cast<double>(age_ns) / 1.0e6 : 0.0;
    }
  }

  // Timeout guard: default stand pose if target is stale
  if (latest_target_stamp_.nanoseconds() == 0 || age_ms > target_timeout_ms_) {
    target = sim2real_common::DeploymentContract::kDefaultDofPos;
    target_source = "timeout_hold";
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
    auto safety_decision = safety_monitor_->check(target, sim2real_common::DeploymentContract::kDefaultDofPos, gyro, proj_grav, estop_active);
    if (safety_decision.level == sim2real_common::SafetyLevel::ESTOP || safety_decision.level == sim2real_common::SafetyLevel::BRAKE) {
      safety_triggered_ = true;
      safety_reason_ = "Safety Monitor Stop: " + safety_decision.message;
      RCLCPP_ERROR(get_logger(), "SAFETY TRIGGERED: %s", safety_reason_.c_str());
    } else if (safety_decision.level == sim2real_common::SafetyLevel::CLIP) {
      target = safety_decision.clipped_target;
      target_source = "safety_clip";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Safety Monitor: Joint target clipped.");
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
          kp_val = sim2real_common::DeploymentContract::kLegHoldKp * kp_scale;
          kd_val = sim2real_common::DeploymentContract::kLegHoldKd;
        } else {
          startup_soft_hold_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          if (target_source == "timeout_hold" || target_source == "boot_hold" || target_source == "runtime_zero_hold" || target_source == "startup_hold") {
            kp_val = sim2real_common::DeploymentContract::kLegHoldKp;
            kd_val = sim2real_common::DeploymentContract::kLegHoldKd;
          }
        }
        writeOperationFrame(fd, motors_[i].id, real_val, 0.0, kp_val, kd_val, 0.0);
      } else {
        // Wheel joints: MIT velocity control (Kp = 0, Kd = kWheelKd, velocity = target, position = 0)
        double vel_real = motors_[i].direction * sim_val; // Wheels actions are in velocity, apply sign
        double kd_val = sim2real_common::DeploymentContract::kWheelKd;
        if (target_source == "safety_brake" || target_source == "safety_estop") {
          vel_real = 0.0;
          kd_val = 2.0; // Wheel damping Kd
        }
        writeOperationFrame(fd, motors_[i].id, 0.0, vel_real, 0.0, kd_val, 0.0);
      }
    }
  }
}

bool HardwareBridgeNode::enableMotor(int fd, int motor_id)
{
  std::uint32_t ext_id = (COMM_ENABLE << 24) | (HOST_ID << 8) | motor_id;
  return sendCanFrame(fd, ext_id, nullptr, 0);
}

bool HardwareBridgeNode::disableMotor(int fd, int motor_id)
{
  std::uint32_t ext_id = (COMM_DISABLE << 24) | (HOST_ID << 8) | motor_id;
  std::uint8_t data[8] = {0};
  return sendCanFrame(fd, ext_id, data, 8);
}

bool HardwareBridgeNode::setModeRaw(int fd, int motor_id, std::int8_t mode)
{
  std::uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
  std::uint8_t data[8] = {0};
  data[0] = PARAM_MODE & 0xFF;
  data[1] = (PARAM_MODE >> 8) & 0xFF;
  data[4] = static_cast<std::uint8_t>(mode);
  return sendCanFrame(fd, ext_id, data, 8);
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
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
  bool success = initCan(ifname, fd);
  if (success) {
    error_count = 0;
    RCLCPP_INFO(get_logger(), "CAN interface %s reinitialized successfully.", ifname.c_str());
  }
  return success;
}

}  // namespace sim2real_hw

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sim2real_hw::HardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
