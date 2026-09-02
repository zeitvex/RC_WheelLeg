#include "sim2real_runtime/policy_runtime_node.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

#include "sim2real_common/deployment_contract.hpp"

using namespace std::chrono_literals;

namespace sim2real_runtime
{

// Named constants for timing and command filtering
constexpr float kCmdAccelLimitXY = 0.02f;     // m/s per step (at 50Hz)
constexpr float kCmdAccelLimitYaw = 0.03f;    // rad/s per step (at 50Hz)
constexpr float kPolicyDt = 0.02f;            // policy loop period (50Hz)

PolicyRuntimeNode::PolicyRuntimeNode()
: Node("sim2real_runtime_node")
{
  // 1. Declare and get parameters
  model_path_ = declare_parameter<std::string>("model_path", "policies/model_rough.onnx");
  use_cuda_ = declare_parameter<bool>("use_cuda", false);  // enable CUDA EP on Orin Nano

  // Safety parameters
  safety_enabled_ = declare_parameter<bool>("safety_enabled", true);
  double max_target_offset = declare_parameter<double>("max_target_offset", 0.6);
  double hard_target_offset = declare_parameter<double>("hard_target_offset", 1.2);
  double max_ang_vel = declare_parameter<double>("max_ang_vel", 10.0);
  double max_tilt_z = declare_parameter<double>("max_tilt_z", -0.3);
  int clip_to_brake = declare_parameter<int>("clip_to_brake", 0);
  double imu_age_warn_ms = declare_parameter<double>("imu_age_warn_ms", 60.0);
  double imu_age_stop_ms = declare_parameter<double>("imu_age_stop_ms", 200.0);

  command_release_s_ = static_cast<float>(declare_parameter<double>("command_release_s", 0.35));
  release_command_hold_s_ = static_cast<float>(declare_parameter<double>("release_command_hold_s", 0.12));
  release_posture_max_err_ = static_cast<float>(declare_parameter<double>("release_posture_max_err", 0.35));
  release_target_blend_s_ = static_cast<float>(declare_parameter<double>("release_target_blend_s", 0.30));
  clip_obs_ = static_cast<float>(declare_parameter<double>("clip_obs", 100.0));
  hold_zero_command_pose_ = declare_parameter<bool>("hold_zero_command_pose", true);
  enable_zero_cmd_suppression_ = declare_parameter<bool>("enable_zero_cmd_suppression", true);
  require_active_command_to_release_ = declare_parameter<bool>("require_active_command_to_release", true);
  zero_cmd_use_yaw_rate_ = declare_parameter<bool>("zero_cmd_use_yaw_rate", true);
  runtime_released_ = !require_active_command_to_release_;

  RCLCPP_INFO(get_logger(), "Loading ONNX policy model from: %s", model_path_.c_str());
  
  // Initialize StandBalanceController
  stand_balance_ = std::make_unique<sim2real_common::StandBalanceController>(0.02);

  // Initialize SafetyMonitor and RuntimeGuard
  safety_monitor_ = std::make_unique<sim2real_common::SafetyMonitor>(
    static_cast<float>(max_target_offset),
    static_cast<float>(max_ang_vel),
    static_cast<float>(max_tilt_z),
    clip_to_brake,
    static_cast<float>(hard_target_offset)
  );

  runtime_guard_ = std::make_unique<sim2real_common::RuntimeGuard>(
    static_cast<float>(max_ang_vel + 2.0),
    static_cast<float>(max_tilt_z),
    static_cast<float>(imu_age_warn_ms),
    static_cast<float>(imu_age_stop_ms)
  );

  // 2. Initialize Ort C++ environment
  try {
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "sim2real_onnx_env");
    
    Ort::SessionOptions session_options;
    // single-thread ORIN optimization to prevent thread scheduling jitter
    session_options.SetIntraOpNumThreads(1);
    session_options.SetInterOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // CUDA Execution Provider (Orin Nano GPU acceleration)
    if (use_cuda_) {
      try {
        OrtCUDAProviderOptions cuda_opts{};
        cuda_opts.device_id = 0;
        // enable_cuda_graph: false for single-inference RL policy (avoids overhead)
        session_options.AppendExecutionProvider_CUDA(cuda_opts);
        RCLCPP_INFO(get_logger(), "CUDA Execution Provider enabled (device 0)");
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(),
          "CUDA EP init failed (ONNX Runtime built without CUDA?): %s. Falling back to CPU.",
          e.what());
        use_cuda_ = false;
      }
    }

    session_ = std::make_unique<Ort::Session>(*env_, model_path_.c_str(), session_options);
    memory_info_ = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU));

    // Get input/output nodes names and shapes
    Ort::AllocatorWithDefaultOptions allocator;
    
    std::size_t num_inputs = session_->GetInputCount();
    for (std::size_t i = 0; i < num_inputs; ++i) {
      auto name = session_->GetInputNameAllocated(i, allocator);
      input_names_str_.push_back(std::string(name.get()));
    }
    for (const auto& name : input_names_str_) {
      input_names_char_.push_back(name.c_str());
    }
    
    std::size_t num_outputs = session_->GetOutputCount();
    for (std::size_t i = 0; i < num_outputs; ++i) {
      auto name = session_->GetOutputNameAllocated(i, allocator);
      output_names_str_.push_back(std::string(name.get()));
    }
    for (const auto& name : output_names_str_) {
      output_names_char_.push_back(name.c_str());
    }

    auto input_type_info = session_->GetInputTypeInfo(0);
    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    input_shape_ = input_tensor_info.GetShape();
    if (input_shape_[0] < 0) {
      input_shape_[0] = 1;
    }
    
    auto output_type_info = session_->GetOutputTypeInfo(0);
    auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
    output_shape_ = output_tensor_info.GetShape();
    if (output_shape_[0] < 0) {
      output_shape_[0] = 1;
    }

    // Validate output shape matches expected action dimension
    if (output_shape_.size() < 2 || output_shape_[1] != static_cast<std::int64_t>(sim2real_common::DeploymentContract::kActionDim)) {
      RCLCPP_FATAL(get_logger(),
        "ONNX model output dimension mismatch! Expected %ld, got %ld. Wrong model?",
        static_cast<std::int64_t>(sim2real_common::DeploymentContract::kActionDim),
        output_shape_.size() >= 2 ? output_shape_[1] : -1);
      throw std::runtime_error("ONNX model output shape mismatch");
    }

    RCLCPP_INFO(get_logger(), "Successfully loaded ONNX policy model. Input shape: [%ld, %ld], Output shape: [%ld, %ld]",
      input_shape_[0], input_shape_[1], output_shape_[0], output_shape_[1]);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "Failed to load ONNX model: %s", e.what());
    throw;
  }

  // 3. Create publishers and subscriptions
  target_pub_ = create_publisher<sim2real_interfaces::msg::RuntimeTarget>("runtime/target", 10);
  state_sub_ = create_subscription<sim2real_interfaces::msg::RuntimeState>(
    "runtime/state", 10,
    std::bind(&PolicyRuntimeNode::onState, this, std::placeholders::_1));
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&PolicyRuntimeNode::onCmdVel, this, std::placeholders::_1));
  cmd_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_stamped", 10,
    std::bind(&PolicyRuntimeNode::onCmdVelStamped, this, std::placeholders::_1));
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/safety/estop", 10,
    std::bind(&PolicyRuntimeNode::onEstop, this, std::placeholders::_1));

  // 4. Timer at 50Hz (20ms)
  policy_timer_ = create_wall_timer(20ms, std::bind(&PolicyRuntimeNode::onPolicyLoop, this));
  
  last_actions_.fill(0.0f);
}

void PolicyRuntimeNode::onState(const sim2real_interfaces::msg::RuntimeState::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  latest_state_ = *msg;
  has_state_ = true;
  last_state_recv_time_ = std::chrono::steady_clock::now();
}

void PolicyRuntimeNode::applyCmdVel(float vx, float vy, float vyaw)
{
  // Velocity saturation limits (consistent with training domain)
  constexpr float kMaxLinVelX = 0.8f;   // m/s
  constexpr float kMaxLinVelY = 0.3f;   // m/s
  constexpr float kMaxAngVelZ = 0.5f;   // rad/s

  std::scoped_lock<std::mutex> lock(mutex_);
  raw_cmd_[0] = std::clamp(vx, -kMaxLinVelX, kMaxLinVelX);
  raw_cmd_[1] = std::clamp(vy, -kMaxLinVelY, kMaxLinVelY);
  raw_cmd_[2] = std::clamp(vyaw, -kMaxAngVelZ, kMaxAngVelZ);
  cmd_ = raw_cmd_;
}

void PolicyRuntimeNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  applyCmdVel(
    static_cast<float>(msg->linear.x),
    static_cast<float>(msg->linear.y),
    static_cast<float>(msg->angular.z));
}

void PolicyRuntimeNode::onCmdVelStamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  applyCmdVel(
    static_cast<float>(msg->twist.linear.x),
    static_cast<float>(msg->twist.linear.y),
    static_cast<float>(msg->twist.angular.z));
}

void PolicyRuntimeNode::onEstop(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  estop_triggered_ = msg->data;
  if (estop_triggered_) {
    RCLCPP_WARN(get_logger(), "!!! E-stop triggered via /safety/estop !!!");
  } else {
    RCLCPP_INFO(get_logger(), "E-stop reset.");
  }
}

std::array<float, 53> PolicyRuntimeNode::buildObservation(
  const sim2real_interfaces::msg::RuntimeState & state,
  const std::array<float, 3> & cmd,
  const std::array<float, 16> & last_actions) const
{
  std::array<float, 53> obs{};
  std::size_t cursor = 0;

  for (int i = 0; i < 3; ++i) {
    obs[cursor++] = state.imu_gyro[i] * 0.25f;
  }
  for (int i = 0; i < 3; ++i) {
    obs[cursor++] = state.projected_gravity[i];
  }
  for (float v : cmd) {
    obs[cursor++] = v;
  }
  for (std::size_t i = 0; i < sim2real_common::DeploymentContract::kLegJointCount; ++i) {
    obs[cursor++] = state.joint_pos[i] - sim2real_common::DeploymentContract::kDefaultDofPos[i];
  }
  for (std::size_t i = 0; i < sim2real_common::DeploymentContract::kLegJointCount; ++i) {
    obs[cursor++] = state.joint_vel[i] * 0.05f;
  }
  for (std::size_t i = 12; i < sim2real_common::DeploymentContract::kActionDim; ++i) {
    obs[cursor++] = state.joint_vel[i] * 0.05f;
  }
  for (float v : last_actions) {
    obs[cursor++] = v;
  }

  // Clip observations values to ±clip_obs_
  if (clip_obs_ > 0.0f) {
    for (float & v : obs) {
      v = std::clamp(v, -clip_obs_, clip_obs_);
    }
  }

  return obs;
}

std::array<float, 16> PolicyRuntimeNode::runPolicy(const std::array<float, 53> & obs)
{
  std::array<float, 16> action{};
  try {
    auto input_tensor = Ort::Value::CreateTensor<float>(
      *memory_info_,
      const_cast<float*>(obs.data()),
      obs.size(),
      input_shape_.data(),
      input_shape_.size()
    );

    auto output_tensor = Ort::Value::CreateTensor<float>(
      *memory_info_,
      action.data(),
      action.size(),
      output_shape_.data(),
      output_shape_.size()
    );

    session_->Run(
      Ort::RunOptions{nullptr},
      input_names_char_.data(),
      &input_tensor,
      1,
      output_names_char_.data(),
      &output_tensor,
      1
    );
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "ONNX Runtime inference exception: %s", e.what());
    action.fill(0.0f);
  }

  for (float& v : action) {
    v = std::clamp(v, -10.0f, 10.0f);
  }

  return action;
}

bool PolicyRuntimeNode::isZeroCommand(const std::array<float, 3> & cmd, const std::array<float, 3> & imu_gyro) const
{
  const float planar_cmd = std::sqrt(cmd[0] * cmd[0] + cmd[1] * cmd[1]);
  const bool cmd_is_zero = planar_cmd < zero_cmd_lin_thresh_ && std::abs(cmd[2]) < zero_cmd_yaw_thresh_;
  if (!zero_cmd_use_yaw_rate_) {
    return cmd_is_zero;
  }
  return cmd_is_zero && std::abs(imu_gyro[2]) < zero_yaw_rate_thresh_;
}

bool PolicyRuntimeNode::isCommandActive(const std::array<float, 3> & cmd) const
{
  const float planar_cmd = std::sqrt(cmd[0] * cmd[0] + cmd[1] * cmd[1]);
  return planar_cmd >= zero_cmd_lin_thresh_ || std::abs(cmd[2]) >= zero_cmd_yaw_thresh_;
}

void PolicyRuntimeNode::onPolicyLoop()
{
  sim2real_interfaces::msg::RuntimeState state;
  std::array<float, 3> cmd{};
  std::array<float, 3> raw_cmd{};
  std::array<float, 16> last_actions{};
  bool estop_active = false;
  bool safety_active = false;
  double state_age_ms = 0.0;
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    if (!has_state_) {
      return;
    }
    state = latest_state_;
    cmd = cmd_;
    raw_cmd = raw_cmd_;
    last_actions = last_actions_;
    estop_active = estop_triggered_;
    safety_active = safety_triggered_;
    if (last_state_recv_time_.time_since_epoch().count() != 0) {
      state_age_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - last_state_recv_time_).count();
    }
  }

  // 1) Run RuntimeGuard check
  if (safety_enabled_ && !safety_active) {
    std::vector<float> extra_vals;
    extra_vals.reserve(48);
    for (float v : state.joint_pos) extra_vals.push_back(v);
    for (float v : state.joint_vel) extra_vals.push_back(v);
    for (float v : last_actions) extra_vals.push_back(v);

    const float effective_imu_age_ms = static_cast<float>(std::max(
      static_cast<double>(state.imu_age_ms), state_age_ms));

    auto guard_decision = runtime_guard_->check(
      state.imu_gyro, state.projected_gravity, effective_imu_age_ms, estop_active, extra_vals);
    if (guard_decision.level == sim2real_common::GuardLevel::STOP) {
      {
        std::scoped_lock<std::mutex> lock(mutex_);
        safety_triggered_ = true;
      }
      safety_active = true;
      safety_reason_ = "Runtime Guard Stop: " + guard_decision.reason;
      RCLCPP_ERROR(get_logger(), "SAFETY STOP TRIGGERED in Policy Runtime: %s", safety_reason_.c_str());
    } else if (guard_decision.level == sim2real_common::GuardLevel::WARN) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Safety Guard Warning in Policy Runtime: %s", guard_decision.reason.c_str());
    }
  }

  if (safety_active) {
    sim2real_interfaces::msg::RuntimeTarget target;
    target.stamp = now();
    target.sequence = sequence_++;
    target.raw_command = raw_cmd;
    target.command = cmd;
    target.raw_action.fill(0.0f);
    target.scaled_action.fill(0.0f);
    target.target = sim2real_common::DeploymentContract::kDefaultDofPos;
    target.target_source = "safety_brake";
    target.target_age_ms = 0.0f;
    target_pub_->publish(target);
    return;
  }

  sim2real_interfaces::msg::RuntimeTarget target;
  target.stamp = now();
  target.sequence = sequence_++;
  target.raw_command = raw_cmd;
  target.raw_action.fill(0.0f);
  target.scaled_action.fill(0.0f);
  target.command = cmd;

  const auto now_time = rclcpp::Time(target.stamp);

  if (startup_state_ == StartupState::BOOT_HOLD) {
    // 1. Initial State Read
    start_pose_ = state.joint_pos;
    start_pose_[12] = start_pose_[13] = start_pose_[14] = start_pose_[15] = 0.0f; // Wheel starts at 0

    // 2. Shortest periodic delta to stand pose
    float max_dev = 0.0f;
    for (std::size_t i = 0; i < 12; ++i) {
      float delta = sim2real_common::DeploymentContract::kDefaultDofPos[i] - start_pose_[i];
      delta = delta - 2.0f * static_cast<float>(M_PI) * std::floor((delta + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI)));
      startup_delta_[i] = delta;
      max_dev = std::max(max_dev, std::abs(delta));
    }
    startup_delta_[12] = startup_delta_[13] = startup_delta_[14] = startup_delta_[15] = 0.0f;

    if (max_dev > 3.0f) {
      RCLCPP_WARN(get_logger(), "Measured joint dev too large (%f rad > 3.0 rad). Aborting standup transition.", max_dev);
      target.target = start_pose_;
      target.target_source = "boot_hold";
      target_pub_->publish(target);
      return;
    }

    // Adapt transition time: min 2s, max 6s, 1.5s per rad
    transition_time_ = std::clamp(max_dev * 1.5, 2.0, 6.0);
    startup_state_ = StartupState::STARTUP_SOFT_HOLD;
    state_start_time_ = now_time;
    RCLCPP_INFO(get_logger(), "Standup sequence started. Starting dev: %f rad, transition time: %f s", max_dev, transition_time_);
  }

  if (startup_state_ == StartupState::STARTUP_SOFT_HOLD) {
    double elapsed = (now_time - state_start_time_).seconds();
    target.target = start_pose_;
    target.target_source = "startup_soft_hold";

    if (elapsed >= 1.0) { // 1s soft hold
      startup_state_ = StartupState::STARTUP_TRANSITION;
      state_start_time_ = now_time;
      RCLCPP_INFO(get_logger(), "Transitioning to stand pose...");
    }
  }
  else if (startup_state_ == StartupState::STARTUP_TRANSITION) {
    double elapsed = (now_time - state_start_time_).seconds();
    double phase = std::min(1.0, elapsed / transition_time_);

    // Cosine blend interpolation
    double blend = 0.5 - 0.5 * std::cos(M_PI * phase);
    for (std::size_t i = 0; i < 16; ++i) {
      target.target[i] = start_pose_[i] + blend * startup_delta_[i];
    }
    target.target_source = "startup_hold";

    if (phase >= 1.0) {
      // Settle check
      float max_pos_err = 0.0f;
      for (std::size_t i = 0; i < 12; ++i) {
        float delta = sim2real_common::DeploymentContract::kDefaultDofPos[i] - state.joint_pos[i];
        delta = delta - 2.0f * static_cast<float>(M_PI) * std::floor((delta + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI)));
        max_pos_err = std::max(max_pos_err, std::abs(delta));
      }
      float max_vel_err = 0.0f;
      for (std::size_t i = 0; i < 12; ++i) {
        max_vel_err = std::max(max_vel_err, std::abs(state.joint_vel[i]));
      }

      if (max_pos_err <= 0.30f && max_vel_err <= 0.6f) {
        startup_state_ = StartupState::STARTUP_HOLD_AFTER;
        state_start_time_ = now_time;
        RCLCPP_INFO(get_logger(), "Pose settled. Holding for 1.0s...");
      }
    }
  }
  else if (startup_state_ == StartupState::STARTUP_HOLD_AFTER) {
    double elapsed = (now_time - state_start_time_).seconds();
    
    // Run stand balance controller during holding phase
    target.target = stand_balance_->computeTarget(state.projected_gravity, state.imu_gyro, cmd);
    target.target_source = "startup_hold";

    if (elapsed >= 1.0 && stand_balance_->isStable()) {
      startup_state_ = StartupState::RUNTIME;
      RCLCPP_INFO(get_logger(), "Standup sequence completed. Entering Policy RUNTIME mode!");
    }
  }
  else if (startup_state_ == StartupState::RUNTIME) {
    // Python template uses the command directly in policy obs/release logic.
    // Upstream cmd mux may already smooth it, so do not apply an extra runtime filter here.
    filtered_cmd_ = cmd;

    const auto target_hold = stand_balance_->computeTarget(state.projected_gravity, state.imu_gyro, std::array<float, 3>{0.0f, 0.0f, 0.0f});
    const bool zero_command = isZeroCommand(cmd, state.imu_gyro);

    if (!runtime_released_) {
      if (require_active_command_to_release_) {
        if (isCommandActive(cmd)) {
          release_active_time_ += kPolicyDt;
        } else {
          release_active_time_ = 0.0f;
        }

        float max_hold_err = 0.0f;
        for (std::size_t i = 0; i < sim2real_common::DeploymentContract::kLegJointCount; ++i) {
          max_hold_err = std::max(max_hold_err, std::abs(state.joint_pos[i] - target_hold[i]));
        }

        const bool active_ready = release_active_time_ >= release_command_hold_s_;
        const bool posture_ready = max_hold_err <= release_posture_max_err_;
        if (active_ready && posture_ready) {
          runtime_released_ = true;
        }
      } else {
        runtime_released_ = true;
      }
    }

    if (!runtime_released_ || zero_command) {
      release_alpha_ = 0.0f;
      target.runtime_released = false;
      target.release_alpha = 0.0f;
      target.zero_command = zero_command;
      target.raw_action.fill(0.0f);
      target.scaled_action.fill(0.0f);
      last_actions.fill(0.0f);
      target.target_source = "runtime_zero_hold";
      target.target = target_hold;
      if (!runtime_released_) {
        target.target_source = "runtime_hold";
      }
    } else {
      release_alpha_ = std::min(1.0f, release_alpha_ + kPolicyDt / std::max(command_release_s_, 1.0e-3f));
      target.runtime_released = (release_alpha_ >= 1.0f);
      target.release_alpha = release_alpha_;
      target.zero_command = false;
      target.command = cmd;

      auto raw = runPolicy(buildObservation(state, cmd, last_actions));
      for (float & v : raw) {
        v *= release_alpha_;
      }
      target.raw_action = raw;

      const float blend = std::min(1.0f, release_alpha_ * (command_release_s_ / std::max(release_target_blend_s_, kPolicyDt)));
      for (std::size_t i = 0; i < sim2real_common::DeploymentContract::kActionDim; ++i) {
        target.scaled_action[i] = raw[i] * sim2real_common::DeploymentContract::kActionScale[i];
        const float policy_target = target.scaled_action[i] + sim2real_common::DeploymentContract::kDefaultDofPos[i];
        target.target[i] = (1.0f - blend) * target_hold[i] + blend * policy_target;
        last_actions[i] = raw[i];
      }
      target.target_source = blend < 0.999f ? "runtime_blend" : "runtime_policy";
    }
  }

  // 2) Run SafetyMonitor check on computed target
  if (safety_enabled_) {
    auto safety_decision = safety_monitor_->check(target.target, sim2real_common::DeploymentContract::kDefaultDofPos, state.imu_gyro, state.projected_gravity, estop_active);
    if (safety_decision.level == sim2real_common::SafetyLevel::ESTOP || safety_decision.level == sim2real_common::SafetyLevel::BRAKE) {
      {
        std::scoped_lock<std::mutex> lock(mutex_);
        safety_triggered_ = true;
      }
      safety_reason_ = "Safety Monitor Stop: " + safety_decision.message;
      RCLCPP_ERROR(get_logger(), "SAFETY STOP TRIGGERED in Policy Runtime: %s", safety_reason_.c_str());
      
      // Override target to safety_brake damping pose
      target.target = sim2real_common::DeploymentContract::kDefaultDofPos;
      target.target_source = "safety_brake";
    } else if (safety_decision.level == sim2real_common::SafetyLevel::CLIP) {
      target.target = safety_decision.clipped_target;
      target.target_source = "safety_clip";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Safety Monitor: Joint target clipped in Policy Runtime.");
    }
  }

  target.target_age_ms = 0.0f;

  {
    std::scoped_lock<std::mutex> lock(mutex_);
    last_actions_ = last_actions;
  }

  target_pub_->publish(target);
}

}  // namespace sim2real_runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sim2real_runtime::PolicyRuntimeNode>());
  rclcpp::shutdown();
  return 0;
}
