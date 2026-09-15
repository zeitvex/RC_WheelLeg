#include "sim2real_runtime/policy_runtime_node.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "sim2real_common/deployment_contract.hpp"

using namespace std::chrono_literals;

namespace sim2real_runtime
{

#ifdef SIM2REAL_RUNTIME_HAS_TENSORRT
namespace
{

class TensorRtLogger final : public nvinfer1::ILogger
{
public:
  explicit TensorRtLogger(rclcpp::Logger logger)
  : logger_(std::move(logger))
  {
  }

  void log(Severity severity, const char * msg) noexcept override
  {
    if (msg == nullptr) {
      return;
    }

    switch (severity) {
      case Severity::kINTERNAL_ERROR:
      case Severity::kERROR:
        RCLCPP_ERROR(logger_, "[TensorRT] %s", msg);
        break;
      case Severity::kWARNING:
        RCLCPP_WARN(logger_, "[TensorRT] %s", msg);
        break;
      case Severity::kINFO:
        RCLCPP_INFO(logger_, "[TensorRT] %s", msg);
        break;
      default:
        RCLCPP_DEBUG(logger_, "[TensorRT] %s", msg);
        break;
    }
  }

private:
  rclcpp::Logger logger_;
};

TensorRtLogger & getTensorRtLogger(rclcpp::Logger logger)
{
  static TensorRtLogger trt_logger(logger);
  return trt_logger;
}

template <typename T>
void destroyTensorRtObject(T *& object)
{
  if (object == nullptr) {
    return;
  }
#if NV_TENSORRT_MAJOR >= 10
  delete object;
#else
  object->destroy();
#endif
  object = nullptr;
}

}  // namespace
#endif

// Named constants for timing and command filtering
constexpr float kCmdAccelLimitXY = 0.02f;     // m/s per step (at 50Hz)
constexpr float kCmdAccelLimitYaw = 0.03f;    // rad/s per step (at 50Hz)
constexpr float kPolicyDt = 0.02f;            // policy loop period (50Hz)

std::string toLowerCopy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

double rosTimeToSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

PolicyRuntimeNode::PolicyRuntimeNode()
: Node("sim2real_runtime_node")
{
  // 1. Declare and get parameters
  const std::string event_log_dir = declare_parameter<std::string>(
    "event_log_dir", "logs_v2_web");
  run_log_dir_ = event_log_dir;
  event_logger_.configure(event_log_dir, "sim2real_runtime_events");
  model_path_ = declare_parameter<std::string>("model_path", "policies/model_rough.onnx");
  model_engine_path_ = declare_parameter<std::string>("model_engine_path", "");
  prefer_tensorrt_ = declare_parameter<bool>("prefer_tensorrt", true);
  use_cuda_ = declare_parameter<bool>("use_cuda", false);  // enable CUDA EP on Orin Nano
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
  safety_reference_dof_pos_ = rough_default_dof_pos_;
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
      "Parameter crawl_default_dof_pos has %zu entries, expected 16. Falling back to training default pose.",
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
  const double keep_pose_hip_pitch = declare_parameter<double>("keep_pose_hip_pitch", 0.610);
  const double keep_pose_knee = declare_parameter<double>("keep_pose_knee", -1.250);
  const std::vector<double> default_keep_pose = {
    0.0, keep_pose_hip_pitch, keep_pose_knee,
    0.0, keep_pose_hip_pitch, keep_pose_knee,
    0.0, keep_pose_hip_pitch, keep_pose_knee,
    0.0, keep_pose_hip_pitch, keep_pose_knee,
    0.0, 0.0, 0.0, 0.0
  };
  const std::vector<double> configured_keep_pose = declare_parameter<std::vector<double>>(
    "keep_pose_dof_pos", default_keep_pose);
  if (configured_keep_pose.size() == keep_pose_dof_pos_.size()) {
    for (std::size_t i = 0; i < keep_pose_dof_pos_.size(); ++i) {
      keep_pose_dof_pos_[i] = static_cast<float>(configured_keep_pose[i]);
    }
  } else {
    for (std::size_t i = 0; i < keep_pose_dof_pos_.size(); ++i) {
      keep_pose_dof_pos_[i] = static_cast<float>(default_keep_pose[i]);
    }
    RCLCPP_WARN(
      get_logger(),
      "Parameter keep_pose_dof_pos has %zu entries, expected 16. Falling back to keep_pose_hip_pitch/keep_pose_knee.",
      configured_keep_pose.size());
  }
  posture_transition_s_ = declare_parameter<double>("keep_pose_transition_s", 0.8);
  rough_model_path_ = model_path_;
  rough_model_engine_path_ = declare_parameter<std::string>("rough_model_engine_path", model_engine_path_);
  crawl_model_path_ = declare_parameter<std::string>("crawl_model_path", "policies/model_crawl.onnx");
  crawl_model_engine_path_ = declare_parameter<std::string>("crawl_model_engine_path", "");
  wall_model_path_ = declare_parameter<std::string>("wall_model_path", "policies/model_wall.onnx");
  wall_model_engine_path_ = declare_parameter<std::string>("wall_model_engine_path", "");
  const std::string crawl_backend = toLowerCopy(
    declare_parameter<std::string>("crawl_backend", "ik"));
  crawl_backend_ = crawl_backend == "rl" ? CrawlBackend::Rl : CrawlBackend::Ik;
  crawl_ik_wheel_linear_gain_ = static_cast<float>(
    declare_parameter<double>("crawl_ik_wheel_linear_gain", 6.25));
  crawl_ik_wheel_yaw_gain_ = static_cast<float>(
    declare_parameter<double>("crawl_ik_wheel_yaw_gain", 4.0));
  crawl_ik_max_wheel_speed_ = static_cast<float>(
    declare_parameter<double>("crawl_ik_max_wheel_speed", 6.0));
  crawl_ik_abduction_clip_ = static_cast<float>(
    declare_parameter<double>("crawl_ik_abduction_clip", 0.45));
  crawl_ik_yaw_rate_kp_ = static_cast<float>(
    declare_parameter<double>("crawl_ik_yaw_rate_kp", 1.6));
  crawl_ik_imu_posture_ = declare_parameter<bool>("crawl_ik_imu_posture", false);
  crawl_ik_encoder_posture_kp_ = static_cast<float>(
    declare_parameter<double>("crawl_ik_encoder_posture_kp", 0.0));
  crawl_ik_encoder_posture_max_ = static_cast<float>(
    declare_parameter<double>("crawl_ik_encoder_posture_max", 0.03));
  crawl_ik_encoder_guard_ = declare_parameter<bool>("crawl_ik_encoder_guard", true);
  crawl_ik_encoder_guard_start_ = static_cast<float>(
    declare_parameter<double>("crawl_ik_encoder_guard_start", 0.28));
  crawl_ik_encoder_guard_stop_ = static_cast<float>(
    declare_parameter<double>("crawl_ik_encoder_guard_stop", 0.65));
  crawl_ik_imu_guard_ = declare_parameter<bool>("crawl_ik_imu_guard", true);
  const double crawl_ik_imu_guard_start_deg = declare_parameter<double>(
    "crawl_ik_imu_guard_start_deg", 12.0);
  const double crawl_ik_imu_guard_stop_deg = declare_parameter<double>(
    "crawl_ik_imu_guard_stop_deg", 28.0);
  constexpr double kPi = 3.14159265358979323846;
  crawl_ik_imu_guard_start_rad_ = static_cast<float>(
    crawl_ik_imu_guard_start_deg * kPi / 180.0);
  crawl_ik_imu_guard_stop_rad_ = static_cast<float>(
    crawl_ik_imu_guard_stop_deg * kPi / 180.0);
  model_switch_transition_s_ = declare_parameter<double>("model_switch_transition_s", 1.2);
  model_switch_to_stand_transition_scale_ = declare_parameter<double>(
    "model_switch_to_stand_transition_scale", 1.35);
  model_switch_to_model_transition_scale_ = declare_parameter<double>(
    "model_switch_to_model_transition_scale", 1.55);
  model_switch_min_transition_s_ = declare_parameter<double>("model_switch_min_transition_s", 0.35);
  model_switch_stand_hold_s_ = declare_parameter<double>("model_switch_stand_hold_s", 0.45);
  model_switch_stand_max_err_ = declare_parameter<double>("model_switch_stand_max_err", 0.18);
  model_switch_stand_max_vel_ = declare_parameter<double>("model_switch_stand_max_vel", 0.8);

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

  command_release_s_ = static_cast<float>(declare_parameter<double>("command_release_s", 0.35));
  release_command_hold_s_ = static_cast<float>(declare_parameter<double>("release_command_hold_s", 0.12));
  release_posture_max_err_ = static_cast<float>(declare_parameter<double>("release_posture_max_err", 0.35));
  release_target_blend_s_ = static_cast<float>(declare_parameter<double>("release_target_blend_s", 0.30));
  model_switch_release_scale_ = static_cast<float>(
    declare_parameter<double>("model_switch_release_scale", 1.3));
  runtime_max_vx_ = static_cast<float>(declare_parameter<double>("runtime_max_vx", 1.0));
  runtime_max_vy_ = static_cast<float>(declare_parameter<double>("runtime_max_vy", 0.3));
  runtime_max_yaw_rate_ = static_cast<float>(
    declare_parameter<double>("runtime_max_yaw_rate", 1.0));
  debug_trace_enabled_ = declare_parameter<bool>("debug_trace_enabled", true);
  const int debug_trace_decimation = declare_parameter<int>("debug_trace_decimation", 1);
  debug_trace_decimation_ = static_cast<std::uint32_t>(std::max(debug_trace_decimation, 1));
  clip_obs_ = static_cast<float>(declare_parameter<double>("clip_obs", 100.0));
  hold_zero_command_pose_ = declare_parameter<bool>("hold_zero_command_pose", true);
  enable_zero_cmd_suppression_ = declare_parameter<bool>("enable_zero_cmd_suppression", true);
  require_active_command_to_release_ = declare_parameter<bool>("require_active_command_to_release", true);
  zero_cmd_use_yaw_rate_ = declare_parameter<bool>("zero_cmd_use_yaw_rate", true);
  runtime_released_ = !require_active_command_to_release_;

  if (rough_model_engine_path_.empty()) {
    rough_model_engine_path_ = deriveTensorRtEnginePath(rough_model_path_);
  }
  if (model_engine_path_.empty()) {
    model_engine_path_ = rough_model_engine_path_;
  }
  if (crawl_model_engine_path_.empty()) {
    crawl_model_engine_path_ = deriveTensorRtEnginePath(crawl_model_path_);
  }
  if (wall_model_engine_path_.empty()) {
    wall_model_engine_path_ = deriveTensorRtEnginePath(wall_model_path_);
  }

  RCLCPP_INFO(
    get_logger(),
    "Policy inference setup: rough_model=%s, rough_engine=%s, crawl_model=%s, crawl_engine=%s, wall_model=%s, wall_engine=%s, crawl_backend=%s, prefer_tensorrt=%s",
    rough_model_path_.c_str(),
    rough_model_engine_path_.c_str(),
    crawl_model_path_.c_str(),
    crawl_model_engine_path_.c_str(),
    wall_model_path_.c_str(),
    wall_model_engine_path_.c_str(),
    crawlBackendName(),
    prefer_tensorrt_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(),
    "Event log file: %s",
    event_logger_.componentLogPath().c_str());
  RCLCPP_INFO(
    get_logger(),
    "Run log directory: %s",
    run_log_dir_.c_str());
  logEvent("INFO", "node_start", "Policy runtime node started.");
  initializeDebugTrace();
  
  // Initialize StandBalanceController
  stand_balance_ = std::make_unique<sim2real_common::StandBalanceController>(0.02);
  const float rough_hip_mean =
    (rough_default_dof_pos_[1] + rough_default_dof_pos_[4] +
    rough_default_dof_pos_[7] + rough_default_dof_pos_[10]) * 0.25f;
  const float rough_knee_mean =
    (rough_default_dof_pos_[2] + rough_default_dof_pos_[5] +
    rough_default_dof_pos_[8] + rough_default_dof_pos_[11]) * 0.25f;
  stand_balance_->setNominalLegPose(rough_hip_mean, rough_knee_mean);

  // Initialize SafetyMonitor and RuntimeGuard
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
    static_cast<float>(max_ang_vel + 2.0),
    static_cast<float>(max_tilt_z),
    static_cast<float>(imu_age_warn_ms),
    static_cast<float>(imu_age_stop_ms)
  );

  // 2. Initialize inference backend
  if (!initInferenceBackend()) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize any inference backend.");
    throw std::runtime_error("failed to initialize inference backend");
  }

  // 3. Create publishers and subscriptions
  target_pub_ = create_publisher<sim2real_interfaces::msg::RuntimeTarget>("runtime/target", 10);
  model_status_pub_ = create_publisher<std_msgs::msg::String>("runtime/model_status", 10);
  state_sub_ = create_subscription<sim2real_interfaces::msg::RuntimeState>(
    "runtime/state", 10,
    std::bind(&PolicyRuntimeNode::onState, this, std::placeholders::_1));
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&PolicyRuntimeNode::onCmdVel, this, std::placeholders::_1));
  cmd_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_stamped", 10,
    std::bind(&PolicyRuntimeNode::onCmdVelStamped, this, std::placeholders::_1));
  model_switch_sub_ = create_subscription<std_msgs::msg::String>(
    "runtime/model_cmd", 10,
    std::bind(&PolicyRuntimeNode::onModelSwitchCmd, this, std::placeholders::_1));
  posture_cmd_sub_ = create_subscription<std_msgs::msg::String>(
    "runtime/posture_cmd", 10,
    std::bind(&PolicyRuntimeNode::onPostureCmd, this, std::placeholders::_1));
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/safety/estop", 10,
    std::bind(&PolicyRuntimeNode::onEstop, this, std::placeholders::_1));

  // 4. Timer at 50Hz (20ms)
  policy_timer_ = create_wall_timer(20ms, std::bind(&PolicyRuntimeNode::onPolicyLoop, this));
  
  last_actions_.fill(0.0f);
  publishModelStatus();
}

PolicyRuntimeNode::~PolicyRuntimeNode()
{
  logEvent("INFO", "node_stop", "Policy runtime node stopped.");
  finalizeRunSummary();
  shutdownTensorRt();
  shutdownOnnxRuntime();
}

void PolicyRuntimeNode::logEvent(
  const std::string & level,
  const std::string & event,
  const std::string & message)
{
  event_logger_.log(level, "sim2real_runtime_node", event, message);
}

void PolicyRuntimeNode::initializeDebugTrace()
{
  if (!debug_trace_enabled_ || run_log_dir_.empty()) {
    return;
  }

  std::error_code ec;
  std::filesystem::create_directories(run_log_dir_, ec);
  debug_trace_path_ = (std::filesystem::path(run_log_dir_) / "runtime_debug_trace.csv").string();

  std::ofstream stream(debug_trace_path_, std::ios::trunc);
  if (!stream.is_open()) {
    RCLCPP_WARN(get_logger(), "Failed to open runtime debug trace file: %s", debug_trace_path_.c_str());
    debug_trace_enabled_ = false;
    return;
  }

  stream
    << "stamp_sec,target_seq,state_seq,state_source,startup_state,current_model,requested_model,loaded_model,"
    << "switch_state,target_source,runtime_released,release_alpha,zero_command,target_age_ms,imu_age_ms,odom_age_ms,"
    << "cmd_vx,cmd_vy,cmd_wz,raw_cmd_vx,raw_cmd_vy,raw_cmd_wz";
  for (std::size_t i = 0; i < sim2real_common::DeploymentContract::kActionDim; ++i) {
    stream << ",joint_pos_" << i;
  }
  for (std::size_t i = 0; i < sim2real_common::DeploymentContract::kActionDim; ++i) {
    stream << ",target_" << i;
  }
  stream << '\n';
}

void PolicyRuntimeNode::appendDebugTrace(
  const sim2real_interfaces::msg::RuntimeState & state,
  const sim2real_interfaces::msg::RuntimeTarget & target)
{
  if (!debug_trace_enabled_ || debug_trace_path_.empty()) {
    return;
  }

  if ((debug_trace_counter_++ % debug_trace_decimation_) != 0) {
    return;
  }

  std::ofstream stream(debug_trace_path_, std::ios::app);
  if (!stream.is_open()) {
    return;
  }

  stream
    << std::fixed << std::setprecision(6)
    << rosTimeToSeconds(target.stamp)
    << ',' << target.sequence
    << ',' << state.sequence
    << ',' << state.source
    << ',' << startupStateName(startup_state_)
    << ',' << modelModeName(current_model_mode_)
    << ',' << modelModeName(requested_model_mode_)
    << ',' << modelModeName(loaded_model_mode_)
    << ',' << modelSwitchStateName(model_switch_state_)
    << ',' << target.target_source
    << ',' << (target.runtime_released ? 1 : 0)
    << ',' << target.release_alpha
    << ',' << (target.zero_command ? 1 : 0)
    << ',' << target.target_age_ms
    << ',' << state.imu_age_ms
    << ',' << state.odom_age_ms
    << ',' << target.command[0]
    << ',' << target.command[1]
    << ',' << target.command[2]
    << ',' << target.raw_command[0]
    << ',' << target.raw_command[1]
    << ',' << target.raw_command[2];

  for (float value : state.joint_pos) {
    stream << ',' << value;
  }
  for (float value : target.target) {
    stream << ',' << value;
  }
  stream << '\n';
}

void PolicyRuntimeNode::logProtectionEvent(
  const std::string & trigger,
  const std::string & reason,
  const std::string & action)
{
  protection_trigger_count_++;
  std::ostringstream oss;
  oss << "trigger=" << trigger
      << ", protection_action=" << action
      << ", reason=" << reason;
  logEvent("ERROR", "protection_triggered", oss.str());
}

void PolicyRuntimeNode::finalizeRunSummary()
{
  std::ostringstream oss;
  oss << "run_dir=" << run_log_dir_
      << ", protection_trigger_count=" << protection_trigger_count_
      << ", target_clip_count=" << target_clip_count_
      << ", final_safety_triggered=" << (safety_triggered_ ? "true" : "false");
  if (!safety_reason_.empty()) {
    oss << ", final_safety_reason=" << safety_reason_;
  }
  event_logger_.logSummary("sim2real_runtime_node", oss.str());
}

std::string PolicyRuntimeNode::deriveTensorRtEnginePath(const std::string & onnx_model_path) const
{
  constexpr const char * kSuffix = ".onnx";
  if (onnx_model_path.size() > std::strlen(kSuffix) &&
      onnx_model_path.compare(onnx_model_path.size() - std::strlen(kSuffix), std::strlen(kSuffix), kSuffix) == 0) {
    return onnx_model_path.substr(0, onnx_model_path.size() - std::strlen(kSuffix)) + "_fp16.engine";
  }
  return onnx_model_path + ".engine";
}

bool PolicyRuntimeNode::initInferenceBackend()
{
  if (prefer_tensorrt_) {
    if (initTensorRt()) {
      inference_backend_ = InferenceBackend::TensorRT;
      return true;
    }
    RCLCPP_WARN(get_logger(), "TensorRT initialization failed. Falling back to ONNX Runtime.");
  }

  if (initOnnxRuntime()) {
    inference_backend_ = InferenceBackend::OnnxRuntime;
    return true;
  }

  inference_backend_ = InferenceBackend::None;
  return false;
}

#ifdef SIM2REAL_RUNTIME_HAS_TENSORRT
bool PolicyRuntimeNode::initTensorRt()
{
  shutdownTensorRt();
  inference_backend_ = InferenceBackend::None;

  if (model_engine_path_.empty()) {
    RCLCPP_WARN(get_logger(), "TensorRT engine path is empty.");
    return false;
  }

  std::ifstream engine_file(model_engine_path_, std::ios::binary);
  if (!engine_file) {
    RCLCPP_WARN(get_logger(), "TensorRT engine file not found: %s", model_engine_path_.c_str());
    return false;
  }

  engine_file.seekg(0, std::ios::end);
  const std::streamsize engine_size = engine_file.tellg();
  if (engine_size <= 0) {
    RCLCPP_WARN(get_logger(), "TensorRT engine file is empty: %s", model_engine_path_.c_str());
    return false;
  }
  engine_file.seekg(0, std::ios::beg);

  std::vector<char> engine_data(static_cast<std::size_t>(engine_size));
  if (!engine_file.read(engine_data.data(), engine_size)) {
    RCLCPP_WARN(get_logger(), "Failed to read TensorRT engine file: %s", model_engine_path_.c_str());
    return false;
  }

  auto & logger = getTensorRtLogger(get_logger());
  trt_runtime_ = nvinfer1::createInferRuntime(logger);
  if (trt_runtime_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Failed to create TensorRT runtime.");
    return false;
  }

  trt_engine_ = trt_runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size());
  if (trt_engine_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Failed to deserialize TensorRT engine: %s", model_engine_path_.c_str());
    shutdownTensorRt();
    return false;
  }

  trt_context_ = trt_engine_->createExecutionContext();
  if (trt_context_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Failed to create TensorRT execution context.");
    shutdownTensorRt();
    return false;
  }

  if (trt_engine_->getNbIOTensors() != 2) {
    RCLCPP_WARN(
      get_logger(),
      "Unexpected TensorRT IO tensor count: %d (expected 2).",
      trt_engine_->getNbIOTensors());
    shutdownTensorRt();
    return false;
  }

  for (int i = 0; i < trt_engine_->getNbIOTensors(); ++i) {
    const char * tensor_name = trt_engine_->getIOTensorName(i);
    if (trt_engine_->getTensorIOMode(tensor_name) == nvinfer1::TensorIOMode::kINPUT) {
      trt_input_name_ = tensor_name;
    } else {
      trt_output_name_ = tensor_name;
    }
  }

  if (trt_input_name_.empty() || trt_output_name_.empty()) {
    RCLCPP_WARN(get_logger(), "Failed to resolve TensorRT input/output tensor names.");
    shutdownTensorRt();
    return false;
  }

  const auto input_dims = trt_engine_->getTensorShape(trt_input_name_.c_str());
  const auto output_dims = trt_engine_->getTensorShape(trt_output_name_.c_str());
  if (input_dims.nbDims != 2 || output_dims.nbDims != 2) {
    RCLCPP_WARN(get_logger(), "Unexpected TensorRT tensor ranks. input=%d output=%d", input_dims.nbDims, output_dims.nbDims);
    shutdownTensorRt();
    return false;
  }

  if (input_dims.d[1] != 53 || output_dims.d[1] != static_cast<int>(sim2real_common::DeploymentContract::kActionDim)) {
      RCLCPP_WARN(
        get_logger(),
        "TensorRT engine shape mismatch. input second dim=%ld output second dim=%ld",
        static_cast<long>(input_dims.d[1]),
        static_cast<long>(output_dims.d[1]));
    shutdownTensorRt();
    return false;
  }

  input_shape_ = {1, 53};
  output_shape_ = {1, static_cast<std::int64_t>(sim2real_common::DeploymentContract::kActionDim)};

  if (cudaStreamCreate(&trt_stream_) != cudaSuccess) {
    RCLCPP_WARN(get_logger(), "Failed to create CUDA stream for TensorRT.");
    shutdownTensorRt();
    return false;
  }

  const std::size_t input_bytes = sizeof(float) * 53;
  const std::size_t output_bytes = sizeof(float) * sim2real_common::DeploymentContract::kActionDim;
  if (cudaMalloc(&trt_input_buffer_, input_bytes) != cudaSuccess ||
      cudaMalloc(&trt_output_buffer_, output_bytes) != cudaSuccess) {
    RCLCPP_WARN(get_logger(), "Failed to allocate TensorRT CUDA buffers.");
    shutdownTensorRt();
    return false;
  }

  if (!trt_context_->setInputShape(trt_input_name_.c_str(), nvinfer1::Dims2{1, 53})) {
    RCLCPP_WARN(get_logger(), "Failed to set TensorRT input shape.");
    shutdownTensorRt();
    return false;
  }

  if (!trt_context_->setTensorAddress(trt_input_name_.c_str(), trt_input_buffer_) ||
      !trt_context_->setTensorAddress(trt_output_name_.c_str(), trt_output_buffer_)) {
    RCLCPP_WARN(get_logger(), "Failed to bind TensorRT IO buffers.");
    shutdownTensorRt();
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "TensorRT engine loaded successfully from %s. Input=%s[1x53], Output=%s[1x%ld]",
    model_engine_path_.c_str(),
    trt_input_name_.c_str(),
    trt_output_name_.c_str(),
    static_cast<long>(sim2real_common::DeploymentContract::kActionDim));
  return true;
}
#else
bool PolicyRuntimeNode::initTensorRt()
{
  RCLCPP_INFO(get_logger(), "TensorRT support not compiled in; skipping TensorRT initialization.");
  return false;
}
#endif

bool PolicyRuntimeNode::initOnnxRuntime()
{
  shutdownOnnxRuntime();

  try {
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "sim2real_onnx_env");

    const auto available_providers = Ort::GetAvailableProviders();
    std::ostringstream provider_stream;
    for (std::size_t i = 0; i < available_providers.size(); ++i) {
      if (i > 0) {
        provider_stream << ", ";
      }
      provider_stream << available_providers[i];
    }
    RCLCPP_INFO(
      get_logger(),
      "ONNX Runtime available providers: [%s]",
      provider_stream.str().c_str());

    if (use_cuda_) {
      const bool has_cuda_provider = std::find(
        available_providers.begin(),
        available_providers.end(),
        "CUDAExecutionProvider") != available_providers.end();
      if (!has_cuda_provider) {
        RCLCPP_WARN(
          get_logger(),
          "Parameter use_cuda=true, but CUDAExecutionProvider is not available in the current ONNX Runtime build.");
      }
    }

    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetInterOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    if (use_cuda_) {
      try {
        OrtCUDAProviderOptions cuda_opts{};
        cuda_opts.device_id = 0;
        session_options.AppendExecutionProvider_CUDA(cuda_opts);
        RCLCPP_INFO(get_logger(), "CUDA Execution Provider enabled (device 0)");
      } catch (const std::exception& e) {
        RCLCPP_WARN(
          get_logger(),
          "CUDA EP init failed (ONNX Runtime built without CUDA?): %s. Falling back to CPU.",
          e.what());
        use_cuda_ = false;
      }
    }

    session_ = std::make_unique<Ort::Session>(*env_, model_path_.c_str(), session_options);
    memory_info_ = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU));

    Ort::AllocatorWithDefaultOptions allocator;
    input_names_str_.clear();
    output_names_str_.clear();
    input_names_char_.clear();
    output_names_char_.clear();

    const std::size_t num_inputs = session_->GetInputCount();
    for (std::size_t i = 0; i < num_inputs; ++i) {
      auto name = session_->GetInputNameAllocated(i, allocator);
      input_names_str_.push_back(std::string(name.get()));
    }
    for (const auto & name : input_names_str_) {
      input_names_char_.push_back(name.c_str());
    }

    const std::size_t num_outputs = session_->GetOutputCount();
    for (std::size_t i = 0; i < num_outputs; ++i) {
      auto name = session_->GetOutputNameAllocated(i, allocator);
      output_names_str_.push_back(std::string(name.get()));
    }
    for (const auto & name : output_names_str_) {
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

    if (output_shape_.size() < 2 || output_shape_[1] != static_cast<std::int64_t>(sim2real_common::DeploymentContract::kActionDim)) {
      throw std::runtime_error("ONNX model output shape mismatch");
    }

    RCLCPP_INFO(
      get_logger(),
      "ONNX Runtime model loaded successfully from %s. Input shape: [%ld, %ld], Output shape: [%ld, %ld]",
      model_path_.c_str(),
      input_shape_[0],
      input_shape_[1],
      output_shape_[0],
      output_shape_[1]);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to load ONNX Runtime model: %s", e.what());
    shutdownOnnxRuntime();
    return false;
  }
}

void PolicyRuntimeNode::shutdownOnnxRuntime()
{
  memory_info_.reset();
  session_.reset();
  env_.reset();
  input_names_char_.clear();
  output_names_char_.clear();
  input_names_str_.clear();
  output_names_str_.clear();
}

void PolicyRuntimeNode::shutdownTensorRt()
{
#ifdef SIM2REAL_RUNTIME_HAS_TENSORRT
  if (trt_input_buffer_ != nullptr) {
    cudaFree(trt_input_buffer_);
    trt_input_buffer_ = nullptr;
  }
  if (trt_output_buffer_ != nullptr) {
    cudaFree(trt_output_buffer_);
    trt_output_buffer_ = nullptr;
  }
  if (trt_stream_ != nullptr) {
    cudaStreamDestroy(trt_stream_);
    trt_stream_ = nullptr;
  }
  if (trt_context_ != nullptr) {
    destroyTensorRtObject(trt_context_);
  }
  if (trt_engine_ != nullptr) {
    destroyTensorRtObject(trt_engine_);
  }
  if (trt_runtime_ != nullptr) {
    destroyTensorRtObject(trt_runtime_);
  }
  trt_input_name_.clear();
  trt_output_name_.clear();
#endif
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
  std::scoped_lock<std::mutex> lock(mutex_);
  raw_cmd_[0] = std::clamp(vx, -runtime_max_vx_, runtime_max_vx_);
  raw_cmd_[1] = std::clamp(vy, -runtime_max_vy_, runtime_max_vy_);
  raw_cmd_[2] = std::clamp(vyaw, -runtime_max_yaw_rate_, runtime_max_yaw_rate_);
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

void PolicyRuntimeNode::onModelSwitchCmd(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string command = msg ? msg->data : "";
  const auto to_lower = [](std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return value;
  };

  const std::string normalized = to_lower(command);
  ModelMode target_mode = current_model_mode_;
  bool recognized = true;
  if (normalized == "toggle" || normalized == "switch") {
    target_mode = current_model_mode_ == ModelMode::Rough ? ModelMode::Crawl : ModelMode::Rough;
  } else if (normalized == "rough") {
    target_mode = ModelMode::Rough;
  } else if (normalized == "crawl" || normalized == "ik") {
    target_mode = ModelMode::Crawl;
  } else if (normalized == "wall") {
    target_mode = ModelMode::Wall;
  } else {
    recognized = false;
  }

  if (!recognized) {
    RCLCPP_WARN(get_logger(), "Ignoring unknown model switch command: %s", command.c_str());
    return;
  }

  if (startup_state_ != StartupState::RUNTIME) {
    RCLCPP_WARN(get_logger(), "Ignoring model switch command before runtime release: %s", command.c_str());
    return;
  }

  if (model_switch_state_ != ModelSwitchState::Idle) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring model switch command while another switch is active. current_state=%s",
      modelSwitchStateName(model_switch_state_));
    return;
  }

  if (posture_hold_mode_ != PostureHoldMode::None) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring model switch command while posture hold is active. posture=%s",
      postureHoldModeName(posture_hold_mode_));
    return;
  }
  if (target_mode == current_model_mode_) {
    RCLCPP_INFO(get_logger(), "Model switch requested to current model %s; ignoring.", modelModeName(target_mode));
    return;
  }

  {
    std::scoped_lock<std::mutex> lock(mutex_);
    requested_model_mode_ = target_mode;
    model_switch_requested_ = true;
  }
  RCLCPP_INFO(
    get_logger(),
    "Queued model switch from %s to %s",
    modelModeName(current_model_mode_),
    modelModeName(target_mode));
  publishModelStatus();
}

void PolicyRuntimeNode::startPostureTransition(
  PostureHoldMode mode,
  const std::array<float, 16> & start_pose,
  const std::array<float, 16> & target_pose,
  const rclcpp::Time & now_time)
{
  posture_hold_mode_ = mode;
  posture_transition_active_ = true;
  posture_transition_start_time_ = now_time;
  posture_start_pose_ = start_pose;
  posture_target_pose_ = target_pose;
  posture_start_pose_[12] = posture_start_pose_[13] = posture_start_pose_[14] = posture_start_pose_[15] = 0.0f;
  posture_target_pose_[12] = posture_target_pose_[13] = posture_target_pose_[14] = posture_target_pose_[15] = 0.0f;

  for (std::size_t i = 0; i < posture_delta_.size(); ++i) {
    float delta = posture_target_pose_[i] - posture_start_pose_[i];
    if (i < sim2real_common::DeploymentContract::kLegJointCount) {
      delta = delta - 2.0f * static_cast<float>(M_PI) * std::floor(
        (delta + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI)));
    }
    posture_delta_[i] = delta;
  }

  cmd_.fill(0.0f);
  raw_cmd_.fill(0.0f);
  filtered_cmd_.fill(0.0f);
  last_actions_.fill(0.0f);
  runtime_released_ = false;
  release_alpha_ = 0.0f;
  release_active_time_ = 0.0f;
  hold_active_model_pose_when_unreleased_ = true;
  slow_release_after_model_switch_ = true;
  safety_reference_dof_pos_ = mode == PostureHoldMode::Keep ? keep_pose_dof_pos_ : active_default_dof_pos_;
}

void PolicyRuntimeNode::onPostureCmd(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string command = msg ? msg->data : "";
  const std::string normalized = toLowerCopy(command);
  const rclcpp::Time now_time = now();

  std::scoped_lock<std::mutex> lock(mutex_);
  if (startup_state_ != StartupState::RUNTIME) {
    RCLCPP_WARN(get_logger(), "Ignoring posture command before runtime release: %s", command.c_str());
    return;
  }
  if (model_switch_state_ != ModelSwitchState::Idle) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring posture command while model switch is active. current_state=%s",
      modelSwitchStateName(model_switch_state_));
    return;
  }
  std::array<float, 16> start_pose = has_state_ ? latest_state_.joint_pos : active_default_dof_pos_;
  start_pose[12] = start_pose[13] = start_pose[14] = start_pose[15] = 0.0f;

  if (normalized == "keep") {
    if (posture_hold_mode_ == PostureHoldMode::Keep && !posture_transition_active_) {
      return;
    }
    startPostureTransition(PostureHoldMode::Keep, start_pose, keep_pose_dof_pos_, now_time);
    RCLCPP_INFO(get_logger(), "Posture hold requested: keep pose.");
  } else if (normalized == "default" || normalized == "release" || normalized == "off") {
    if (posture_hold_mode_ == PostureHoldMode::None && !posture_transition_active_) {
      return;
    }
    startPostureTransition(PostureHoldMode::ReturnDefault, start_pose, active_default_dof_pos_, now_time);
    RCLCPP_INFO(get_logger(), "Posture hold requested: return to default pose.");
  } else if (!normalized.empty()) {
    RCLCPP_WARN(get_logger(), "Ignoring unknown posture command: %s", command.c_str());
  }
}
void PolicyRuntimeNode::onEstop(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  const bool was_estop = estop_triggered_;
  estop_triggered_ = msg->data;
  if (estop_triggered_) {
    RCLCPP_WARN(get_logger(), "!!! E-stop triggered via /safety/estop !!!");
    logEvent("WARN", "estop_triggered", "E-stop triggered via /safety/estop.");
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
    obs[cursor++] = state.joint_pos[i] - active_default_dof_pos_[i];
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

  switch (inference_backend_) {
    case InferenceBackend::TensorRT:
#ifdef SIM2REAL_RUNTIME_HAS_TENSORRT
      if (cudaMemcpyAsync(
            trt_input_buffer_,
            obs.data(),
            sizeof(float) * obs.size(),
            cudaMemcpyHostToDevice,
            trt_stream_) != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "TensorRT H2D copy failed.");
        action.fill(0.0f);
        break;
      }
      if (!trt_context_->enqueueV3(trt_stream_)) {
        RCLCPP_ERROR(get_logger(), "TensorRT enqueue failed.");
        action.fill(0.0f);
        break;
      }
      if (cudaMemcpyAsync(
            action.data(),
            trt_output_buffer_,
            sizeof(float) * action.size(),
            cudaMemcpyDeviceToHost,
            trt_stream_) != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "TensorRT D2H copy failed.");
        action.fill(0.0f);
        break;
      }
      if (cudaStreamSynchronize(trt_stream_) != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "TensorRT stream synchronization failed.");
        action.fill(0.0f);
      }
#else
      action.fill(0.0f);
#endif
      break;
    case InferenceBackend::OnnxRuntime:
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
      break;
    default:
      RCLCPP_ERROR(get_logger(), "No inference backend available.");
      action.fill(0.0f);
      break;
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

bool PolicyRuntimeNode::modeUsesInference(ModelMode mode) const
{
  return mode == ModelMode::Rough ||
         mode == ModelMode::Wall ||
         (mode == ModelMode::Crawl && crawl_backend_ == CrawlBackend::Rl);
}

const std::array<float, 16> & PolicyRuntimeNode::defaultPoseForMode(ModelMode mode) const
{
  switch (mode) {
    case ModelMode::Crawl:
      return crawl_default_dof_pos_;
    case ModelMode::Wall:
      return wall_default_dof_pos_;
    case ModelMode::Rough:
    default:
      return rough_default_dof_pos_;
  }
}

const std::string & PolicyRuntimeNode::modelPathForMode(ModelMode mode) const
{
  switch (mode) {
    case ModelMode::Crawl:
      return crawl_model_path_;
    case ModelMode::Wall:
      return wall_model_path_;
    case ModelMode::Rough:
    default:
      return rough_model_path_;
  }
}

const std::string & PolicyRuntimeNode::modelEnginePathForMode(ModelMode mode) const
{
  switch (mode) {
    case ModelMode::Crawl:
      return crawl_model_engine_path_;
    case ModelMode::Wall:
      return wall_model_engine_path_;
    case ModelMode::Rough:
    default:
      return rough_model_engine_path_;
  }
}

std::array<float, 16> PolicyRuntimeNode::computeHoldTarget(
  const sim2real_interfaces::msg::RuntimeState & state,
  const std::array<float, 3> & cmd)
{
  if (current_model_mode_ == ModelMode::Crawl && crawl_backend_ == CrawlBackend::Ik) {
    auto target = crawl_default_dof_pos_;
    if (crawl_ik_imu_posture_) {
      const auto balance_target = stand_balance_->computeTarget(
        state.projected_gravity, state.imu_gyro, cmd);
      for (std::size_t leg_idx = 0; leg_idx < 4; ++leg_idx) {
        const std::size_t abduction_index = leg_idx * 3;
        target[abduction_index] = std::clamp(
          balance_target[abduction_index], -crawl_ik_abduction_clip_, crawl_ik_abduction_clip_);
      }
    } else {
      stand_balance_->computeTarget(
        state.projected_gravity, state.imu_gyro, std::array<float, 3>{0.0f, 0.0f, 0.0f});
    }
    if (crawl_ik_encoder_posture_kp_ > 0.0f) {
      for (std::size_t i = 0; i < 12; ++i) {
        const float encoder_err = target[i] - state.joint_pos[i];
        const float correction = std::clamp(
          crawl_ik_encoder_posture_kp_ * encoder_err,
          -crawl_ik_encoder_posture_max_,
          crawl_ik_encoder_posture_max_);
        target[i] += correction;
      }
    }
    target[12] = 0.0f;
    target[13] = 0.0f;
    target[14] = 0.0f;
    target[15] = 0.0f;
    return target;
  }

  // Keep the balance controller updated for stability monitoring, while holding
  // the active model's default pose for rough/wall runtime zero-command holds.
  stand_balance_->computeTarget(
    state.projected_gravity, state.imu_gyro, std::array<float, 3>{0.0f, 0.0f, 0.0f});
  return active_default_dof_pos_;
}

std::array<float, 16> PolicyRuntimeNode::computeIkCrawlTarget(
  const sim2real_interfaces::msg::RuntimeState & state,
  const std::array<float, 3> & cmd)
{
  auto target = computeHoldTarget(state, cmd);
  const float yaw_rate_cmd =
    cmd[2] + crawl_ik_yaw_rate_kp_ * (cmd[2] - state.imu_gyro[2]);
  const float command_scale = computeCrawlIkCommandScale(state, target);

  const float left_wheel = std::clamp(
    cmd[0] * crawl_ik_wheel_linear_gain_ - yaw_rate_cmd * crawl_ik_wheel_yaw_gain_,
    -crawl_ik_max_wheel_speed_, crawl_ik_max_wheel_speed_) * command_scale;
  const float right_wheel = std::clamp(
    cmd[0] * crawl_ik_wheel_linear_gain_ + yaw_rate_cmd * crawl_ik_wheel_yaw_gain_,
    -crawl_ik_max_wheel_speed_, crawl_ik_max_wheel_speed_) * command_scale;

  target[12] = left_wheel;
  target[13] = right_wheel;
  target[14] = left_wheel;
  target[15] = right_wheel;
  return target;
}

float PolicyRuntimeNode::projectedGravityTiltRad(
  const std::array<float, 3> & projected_gravity) const
{
  const float lateral = std::hypot(projected_gravity[0], projected_gravity[1]);
  const float vertical = std::max(1.0e-6f, std::abs(projected_gravity[2]));
  return std::atan2(lateral, vertical);
}

float PolicyRuntimeNode::computeCrawlIkCommandScale(
  const sim2real_interfaces::msg::RuntimeState & state,
  const std::array<float, 16> & leg_target) const
{
  float scale = 1.0f;
  if (crawl_ik_encoder_guard_) {
    float max_leg_err = 0.0f;
    for (std::size_t i = 0; i < 12; ++i) {
      max_leg_err = std::max(max_leg_err, std::abs(state.joint_pos[i] - leg_target[i]));
    }
    if (max_leg_err >= crawl_ik_encoder_guard_stop_) {
      scale = 0.0f;
    } else if (max_leg_err > crawl_ik_encoder_guard_start_) {
      const float span = std::max(
        1.0e-6f, crawl_ik_encoder_guard_stop_ - crawl_ik_encoder_guard_start_);
      scale *= 1.0f - (max_leg_err - crawl_ik_encoder_guard_start_) / span;
    }
  }

  if (crawl_ik_imu_guard_) {
    const float tilt = projectedGravityTiltRad(state.projected_gravity);
    if (tilt >= crawl_ik_imu_guard_stop_rad_) {
      scale = 0.0f;
    } else if (tilt > crawl_ik_imu_guard_start_rad_) {
      const float span = std::max(
        1.0e-6f, crawl_ik_imu_guard_stop_rad_ - crawl_ik_imu_guard_start_rad_);
      scale *= 1.0f - (tilt - crawl_ik_imu_guard_start_rad_) / span;
    }
  }

  return std::clamp(scale, 0.0f, 1.0f);
}

bool PolicyRuntimeNode::switchInferenceModel(ModelMode target_mode)
{
  const ModelMode previous_mode = current_model_mode_;
  const ModelMode previous_loaded_mode = loaded_model_mode_;
  const std::array<float, 16> previous_default_pose = active_default_dof_pos_;
  const std::string previous_model_path = model_path_;
  const std::string previous_model_engine_path = model_engine_path_;
  const bool target_uses_inference = modeUsesInference(target_mode);

  if (!target_uses_inference) {
    current_model_mode_ = target_mode;
    active_default_dof_pos_ = defaultPoseForMode(target_mode);
    RCLCPP_INFO(
      get_logger(),
      "Switched active model to %s using backend %s",
      modelModeName(current_model_mode_),
      crawlBackendName());
    publishModelStatus();
    return true;
  }

  if (loaded_model_mode_ == target_mode) {
    current_model_mode_ = target_mode;
    active_default_dof_pos_ = defaultPoseForMode(target_mode);
    RCLCPP_INFO(
      get_logger(),
      "Switched active model to %s using already loaded backend %s",
      modelModeName(current_model_mode_),
      inferenceBackendName());
    publishModelStatus();
    return true;
  }

  const std::string next_model_path = modelPathForMode(target_mode);
  const std::string next_model_engine_path = modelEnginePathForMode(target_mode);

  shutdownTensorRt();
  shutdownOnnxRuntime();

  model_path_ = next_model_path;
  model_engine_path_ = next_model_engine_path;

  if (!initInferenceBackend()) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to switch inference model to %s (model=%s engine=%s)",
      modelModeName(target_mode),
      model_path_.c_str(),
      model_engine_path_.c_str());
    shutdownTensorRt();
    shutdownOnnxRuntime();
    model_path_ = previous_model_path;
    model_engine_path_ = previous_model_engine_path;
    current_model_mode_ = previous_mode;
    loaded_model_mode_ = previous_loaded_mode;
    active_default_dof_pos_ = previous_default_pose;
    if (!initInferenceBackend()) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to restore previous inference model %s after switch failure.",
        modelModeName(previous_loaded_mode));
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Restored previous inference model %s after switch failure.",
        modelModeName(previous_loaded_mode));
    }
    publishModelStatus();
    return false;
  }

  loaded_model_mode_ = target_mode;
  current_model_mode_ = target_mode;
  active_default_dof_pos_ = defaultPoseForMode(target_mode);
  RCLCPP_INFO(
    get_logger(),
    "Switched active model to %s using backend %s",
    modelModeName(current_model_mode_),
    inferenceBackendName());
  publishModelStatus();
  return true;
}

const char * PolicyRuntimeNode::modelModeName(ModelMode mode) const
{
  switch (mode) {
    case ModelMode::Rough:
      return "rough";
    case ModelMode::Crawl:
      return crawl_backend_ == CrawlBackend::Ik ? "ik" : "crawl";
    case ModelMode::Wall:
      return "wall";
    default:
      return "unknown";
  }
}

const char * PolicyRuntimeNode::startupStateName(StartupState state) const
{
  switch (state) {
    case StartupState::BOOT_HOLD:
      return "boot_hold";
    case StartupState::STARTUP_SOFT_HOLD:
      return "startup_soft_hold";
    case StartupState::STARTUP_TRANSITION:
      return "startup_transition";
    case StartupState::STARTUP_HOLD_AFTER:
      return "startup_hold_after";
    case StartupState::RUNTIME:
      return "runtime";
    default:
      return "unknown";
  }
}

const char * PolicyRuntimeNode::modelSwitchStateName(ModelSwitchState state) const
{
  switch (state) {
    case ModelSwitchState::Idle:
      return "idle";
    case ModelSwitchState::ToStand:
      return "to_stand";
    case ModelSwitchState::StandHold:
      return "stand_hold";
    case ModelSwitchState::ToModelPose:
      return "to_model_pose";
    default:
      return "unknown";
  }
}

const char * PolicyRuntimeNode::postureHoldModeName(PostureHoldMode mode) const
{
  switch (mode) {
    case PostureHoldMode::Keep:
      return "keep";
    case PostureHoldMode::ReturnDefault:
      return "return_default";
    case PostureHoldMode::None:
    default:
      return "none";
  }
}
const char * PolicyRuntimeNode::inferenceBackendName() const
{
  switch (inference_backend_) {
    case InferenceBackend::TensorRT:
      return "tensorrt";
    case InferenceBackend::OnnxRuntime:
      return "onnxruntime";
    default:
      return "none";
  }
}

const char * PolicyRuntimeNode::crawlBackendName() const
{
  switch (crawl_backend_) {
    case CrawlBackend::Ik:
      return "ik";
    case CrawlBackend::Rl:
      return "rl";
    default:
      return "unknown";
  }
}

void PolicyRuntimeNode::publishModelStatus()
{
  if (!model_status_pub_) {
    return;
  }

  const char * active_backend =
    (current_model_mode_ == ModelMode::Crawl && crawl_backend_ == CrawlBackend::Ik) ?
    crawlBackendName() : inferenceBackendName();

  std::ostringstream stream;
  stream << "{"
         << "\"current_model\":\"" << modelModeName(current_model_mode_) << "\","
         << "\"requested_model\":\"" << modelModeName(requested_model_mode_) << "\","
         << "\"switch_state\":\"" << modelSwitchStateName(model_switch_state_) << "\","
         << "\"backend\":\"" << active_backend << "\","
         << "\"crawl_backend\":\"" << crawlBackendName() << "\","
         << "\"inference_backend\":\"" << inferenceBackendName() << "\","
         << "\"switching\":" << (model_switch_state_ != ModelSwitchState::Idle ? "true" : "false")
         << "}";
  std_msgs::msg::String msg;
  msg.data = stream.str();
  model_status_pub_->publish(msg);
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
  bool model_switch_requested = false;
  ModelMode requested_model_mode = current_model_mode_;
  PostureHoldMode posture_hold_mode = PostureHoldMode::None;
  bool posture_transition_active = false;
  std::array<float, 16> posture_start_pose{};
  std::array<float, 16> posture_target_pose{};
  std::array<float, 16> posture_delta{};
  rclcpp::Time posture_transition_start_time{0, 0, RCL_ROS_TIME};
  double posture_transition_s = posture_transition_s_;
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
    model_switch_requested = model_switch_requested_;
    requested_model_mode = requested_model_mode_;
    posture_hold_mode = posture_hold_mode_;
    posture_transition_active = posture_transition_active_;
    posture_start_pose = posture_start_pose_;
    posture_target_pose = posture_target_pose_;
    posture_delta = posture_delta_;
    posture_transition_start_time = posture_transition_start_time_;
    posture_transition_s = posture_transition_s_;
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
      logEvent("ERROR", "safety_triggered", safety_reason_);
      logProtectionEvent("runtime_guard_stop", guard_decision.reason, "safety_brake");
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
    target.target = safety_reference_dof_pos_;
    target.target_source = "safety_brake";
    target.target_age_ms = 0.0f;
    target_pub_->publish(target);
    appendDebugTrace(state, target);
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
      float delta = active_default_dof_pos_[i] - start_pose_[i];
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
      appendDebugTrace(state, target);
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
        float delta = active_default_dof_pos_[i] - state.joint_pos[i];
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
    
    // Keep balance stability tracking running, while holding the exact default
    // rough pose so startup hold matches the runtime stand posture.
    stand_balance_->computeTarget(state.projected_gravity, state.imu_gyro, cmd);
    target.target = rough_default_dof_pos_;
    target.target_source = "startup_hold";

    if (elapsed >= 1.0 && stand_balance_->isStable()) {
      startup_state_ = StartupState::RUNTIME;
      RCLCPP_INFO(get_logger(), "Standup sequence completed. Entering Policy RUNTIME mode!");
    }
  }
  else if (startup_state_ == StartupState::RUNTIME) {
    if (model_switch_requested && posture_hold_mode == PostureHoldMode::None &&
        model_switch_state_ == ModelSwitchState::Idle && requested_model_mode != current_model_mode_) {
      switch_start_pose_ = state.joint_pos;
      switch_start_pose_[12] = switch_start_pose_[13] = switch_start_pose_[14] = switch_start_pose_[15] = 0.0f;
      float max_dev = 0.0f;
      for (std::size_t i = 0; i < 12; ++i) {
        float delta = rough_default_dof_pos_[i] - switch_start_pose_[i];
        delta = delta - 2.0f * static_cast<float>(M_PI) * std::floor((delta + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI)));
        switch_delta_[i] = delta;
        max_dev = std::max(max_dev, std::abs(delta));
      }
      switch_delta_[12] = switch_delta_[13] = switch_delta_[14] = switch_delta_[15] = 0.0f;
      const double min_switch_s = std::max(0.05, model_switch_min_transition_s_);
      active_switch_transition_s_ = std::clamp(
        max_dev * 1.2, min_switch_s, std::max(min_switch_s, model_switch_transition_s_)) *
        std::max(0.1, model_switch_to_stand_transition_scale_);
      model_switch_state_ = ModelSwitchState::ToStand;
      model_switch_state_start_time_ = now_time;
      safety_reference_dof_pos_ = rough_default_dof_pos_;
      runtime_released_ = false;
      release_active_time_ = 0.0f;
      release_alpha_ = 0.0f;
      last_actions.fill(0.0f);
      hold_active_model_pose_when_unreleased_ = true;
      {
        std::scoped_lock<std::mutex> lock(mutex_);
        model_switch_requested_ = false;
      }
      RCLCPP_INFO(
        get_logger(),
        "Starting model switch transition: %s -> stand -> %s",
        modelModeName(current_model_mode_),
        modelModeName(requested_model_mode));
      publishModelStatus();
    }

    if (model_switch_state_ == ModelSwitchState::ToStand) {
      const double elapsed = (now_time - model_switch_state_start_time_).seconds();
      const double phase = std::min(1.0, elapsed / std::max(1.0e-3, active_switch_transition_s_));
      const double blend = 0.5 - 0.5 * std::cos(M_PI * phase);
      for (std::size_t i = 0; i < 16; ++i) {
        target.target[i] = switch_start_pose_[i] + static_cast<float>(blend) * switch_delta_[i];
      }
      target.target_source = "model_switch_to_stand";
      if (phase >= 1.0) {
        model_switch_state_ = ModelSwitchState::StandHold;
        model_switch_state_start_time_ = now_time;
        publishModelStatus();
      }
    } else if (model_switch_state_ == ModelSwitchState::StandHold) {
      target.target = rough_default_dof_pos_;
      target.target_source = "model_switch_stand_hold";
      const double elapsed = (now_time - model_switch_state_start_time_).seconds();
      float max_pos_err = 0.0f;
      for (std::size_t i = 0; i < sim2real_common::DeploymentContract::kLegJointCount; ++i) {
        float delta = rough_default_dof_pos_[i] - state.joint_pos[i];
        delta = delta - 2.0f * static_cast<float>(M_PI) * std::floor(
          (delta + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI)));
        max_pos_err = std::max(max_pos_err, std::abs(delta));
      }
      float max_vel_err = 0.0f;
      for (std::size_t i = 0; i < sim2real_common::DeploymentContract::kLegJointCount; ++i) {
        max_vel_err = std::max(max_vel_err, std::abs(state.joint_vel[i]));
      }
      const bool hold_elapsed = elapsed >= model_switch_stand_hold_s_;
      const bool stand_ready =
        max_pos_err <= static_cast<float>(model_switch_stand_max_err_) &&
        max_vel_err <= static_cast<float>(model_switch_stand_max_vel_);
      if (hold_elapsed && stand_ready) {
        switch_start_pose_ = state.joint_pos;
        switch_start_pose_[12] = switch_start_pose_[13] = switch_start_pose_[14] = switch_start_pose_[15] = 0.0f;
        if (switchInferenceModel(requested_model_mode)) {
          float max_dev = 0.0f;
          for (std::size_t i = 0; i < 16; ++i) {
            float delta = active_default_dof_pos_[i] - switch_start_pose_[i];
            if (i < sim2real_common::DeploymentContract::kLegJointCount) {
              delta = delta - 2.0f * static_cast<float>(M_PI) * std::floor(
                (delta + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI)));
              max_dev = std::max(max_dev, std::abs(delta));
            }
            switch_delta_[i] = delta;
          }
          const double min_switch_s = std::max(0.05, model_switch_min_transition_s_);
          active_switch_transition_s_ = std::clamp(
            max_dev * 1.2, min_switch_s, std::max(min_switch_s, model_switch_transition_s_)) *
            std::max(0.1, model_switch_to_model_transition_scale_);
          const bool has_pose_delta = std::any_of(
            switch_delta_.begin(), switch_delta_.end(), [](float value) { return std::abs(value) > 1.0e-4f; });
          model_switch_state_ = has_pose_delta ? ModelSwitchState::ToModelPose : ModelSwitchState::Idle;
          model_switch_state_start_time_ = now_time;
          if (!has_pose_delta) {
            safety_reference_dof_pos_ = active_default_dof_pos_;
            slow_release_after_model_switch_ = true;
          }
          hold_active_model_pose_when_unreleased_ = true;
          publishModelStatus();
        } else {
          float max_dev = 0.0f;
          for (std::size_t i = 0; i < 16; ++i) {
            float delta = active_default_dof_pos_[i] - switch_start_pose_[i];
            if (i < sim2real_common::DeploymentContract::kLegJointCount) {
              delta = delta - 2.0f * static_cast<float>(M_PI) * std::floor(
                (delta + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI)));
              max_dev = std::max(max_dev, std::abs(delta));
            }
            switch_delta_[i] = delta;
          }
          const double min_switch_s = std::max(0.05, model_switch_min_transition_s_);
          active_switch_transition_s_ = std::clamp(
            max_dev * 1.2, min_switch_s, std::max(min_switch_s, model_switch_transition_s_)) *
            std::max(0.1, model_switch_to_model_transition_scale_);
          model_switch_state_ = ModelSwitchState::ToModelPose;
          model_switch_state_start_time_ = now_time;
          hold_active_model_pose_when_unreleased_ = true;
          publishModelStatus();
        }
      } else if (hold_elapsed) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Model switch stand hold waiting for settle: max_pos_err=%.3f rad, max_vel_err=%.3f rad/s",
          max_pos_err, max_vel_err);
      }
    } else if (model_switch_state_ == ModelSwitchState::ToModelPose) {
      const double elapsed = (now_time - model_switch_state_start_time_).seconds();
      const double to_model_transition_s = std::max(1.0e-3, active_switch_transition_s_);
      const double phase = std::min(1.0, elapsed / to_model_transition_s);
      const double blend = 0.5 - 0.5 * std::cos(M_PI * phase);
      for (std::size_t i = 0; i < 16; ++i) {
        target.target[i] = switch_start_pose_[i] + static_cast<float>(blend) * switch_delta_[i];
      }
      target.target_source = "model_switch_to_model_pose";
      if (phase >= 1.0) {
        model_switch_state_ = ModelSwitchState::Idle;
        safety_reference_dof_pos_ = active_default_dof_pos_;
        hold_active_model_pose_when_unreleased_ = true;
        slow_release_after_model_switch_ = true;
        publishModelStatus();
      }
    } else if (posture_hold_mode != PostureHoldMode::None) {
      const double elapsed = posture_transition_active ?
        (now_time - posture_transition_start_time).seconds() : posture_transition_s;
      const double phase = posture_transition_active ?
        std::min(1.0, elapsed / std::max(1.0e-3, posture_transition_s)) : 1.0;
      const double blend = 0.5 - 0.5 * std::cos(M_PI * phase);
      for (std::size_t i = 0; i < target.target.size(); ++i) {
        target.target[i] = posture_start_pose[i] + static_cast<float>(blend) * posture_delta[i];
      }
      target.target_source = posture_hold_mode == PostureHoldMode::Keep ?
        "runtime_keep_pose" : "runtime_keep_return_default";
      target.runtime_released = false;
      target.release_alpha = 0.0f;
      target.zero_command = true;
      target.command.fill(0.0f);
      target.raw_command.fill(0.0f);
      target.raw_action.fill(0.0f);
      target.scaled_action.fill(0.0f);
      last_actions.fill(0.0f);
      release_alpha_ = 0.0f;
      runtime_released_ = false;
      release_active_time_ = 0.0f;

      if (phase >= 1.0) {
        std::scoped_lock<std::mutex> lock(mutex_);
        if (posture_hold_mode_ == posture_hold_mode) {
          posture_transition_active_ = false;
          if (posture_hold_mode == PostureHoldMode::ReturnDefault) {
            posture_hold_mode_ = PostureHoldMode::None;
            safety_reference_dof_pos_ = active_default_dof_pos_;
          } else {
            safety_reference_dof_pos_ = keep_pose_dof_pos_;
          }
        }
      }
    } else {
    // Python template uses the command directly in policy obs/release logic.
    // Upstream cmd mux may already smooth it, so do not apply an extra runtime filter here.
    filtered_cmd_ = cmd;

    const auto target_hold = computeHoldTarget(state, cmd);
    const bool zero_command = isZeroCommand(cmd, state.imu_gyro);

    if (!runtime_released_) {
      if (require_active_command_to_release_) {
        if (isCommandActive(cmd)) {
          release_active_time_ += kPolicyDt;
        } else {
          release_active_time_ = 0.0f;
        }

        const auto & unreleased_hold_target =
          hold_active_model_pose_when_unreleased_ ? active_default_dof_pos_ : target_hold;
        float max_hold_err = 0.0f;
        for (std::size_t i = 0; i < sim2real_common::DeploymentContract::kLegJointCount; ++i) {
          max_hold_err = std::max(max_hold_err, std::abs(state.joint_pos[i] - unreleased_hold_target[i]));
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
      target.target = hold_active_model_pose_when_unreleased_ ? active_default_dof_pos_ : target_hold;
      if (!runtime_released_) {
        target.target_source = "runtime_hold";
      }
    } else {
      hold_active_model_pose_when_unreleased_ = false;
      const float effective_release_scale = slow_release_after_model_switch_ ?
        std::max(1.0f, model_switch_release_scale_) : 1.0f;
      const float effective_command_release_s = command_release_s_ * effective_release_scale;
      const float effective_release_target_blend_s = release_target_blend_s_ * effective_release_scale;
      release_alpha_ = std::min(1.0f, release_alpha_ + kPolicyDt / std::max(effective_command_release_s, 1.0e-3f));
      target.runtime_released = (release_alpha_ >= 1.0f);
      target.release_alpha = release_alpha_;
      target.zero_command = false;
      target.command = cmd;

      if (current_model_mode_ == ModelMode::Crawl && crawl_backend_ == CrawlBackend::Ik) {
        target.target = computeIkCrawlTarget(state, cmd);
        target.target_source = "runtime_crawl_ik";
        target.raw_action.fill(0.0f);
        target.scaled_action.fill(0.0f);
        last_actions.fill(0.0f);
      } else {
        auto raw = runPolicy(buildObservation(state, cmd, last_actions));
        for (float & v : raw) {
          v *= release_alpha_;
        }
        target.raw_action = raw;

        const float blend = std::min(
          1.0f,
          release_alpha_ * (effective_command_release_s / std::max(effective_release_target_blend_s, kPolicyDt)));
        for (std::size_t i = 0; i < sim2real_common::DeploymentContract::kActionDim; ++i) {
          target.scaled_action[i] = raw[i] * sim2real_common::DeploymentContract::kActionScale[i];
          const float policy_target = target.scaled_action[i] + active_default_dof_pos_[i];
          target.target[i] = (1.0f - blend) * target_hold[i] + blend * policy_target;
          last_actions[i] = raw[i];
        }
        target.target_source = blend < 0.999f ? "runtime_blend" : "runtime_policy";
      }
      if (release_alpha_ >= 1.0f) {
        slow_release_after_model_switch_ = false;
      }
    }
    }
  }

  // 2) Run SafetyMonitor check on computed target
  if (safety_enabled_) {
    auto * active_safety_monitor =
      (model_switch_state_ != ModelSwitchState::Idle || posture_hold_mode != PostureHoldMode::None) && model_switch_safety_monitor_ ?
      model_switch_safety_monitor_.get() :
      safety_monitor_.get();
    auto safety_decision = active_safety_monitor->check(
      target.target, safety_reference_dof_pos_, state.imu_gyro, state.projected_gravity, estop_active);
    if (safety_decision.level == sim2real_common::SafetyLevel::ESTOP || safety_decision.level == sim2real_common::SafetyLevel::BRAKE) {
      {
        std::scoped_lock<std::mutex> lock(mutex_);
        safety_triggered_ = true;
      }
      safety_reason_ = "Safety Monitor Stop: " + safety_decision.message;
      RCLCPP_ERROR(get_logger(), "SAFETY STOP TRIGGERED in Policy Runtime: %s", safety_reason_.c_str());
      logEvent("ERROR", "safety_triggered", safety_reason_);
      logProtectionEvent("safety_monitor_stop", safety_decision.message, "safety_brake");
      
      // Override target to safety_brake damping pose
      target.target = safety_reference_dof_pos_;
      target.target_source = "safety_brake";
    } else if (safety_decision.level == sim2real_common::SafetyLevel::CLIP) {
      target.target = safety_decision.clipped_target;
      target.target_source = "safety_clip";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Safety Monitor: Joint target clipped in Policy Runtime.");
      target_clip_count_++;
      if (!clip_active_logged_) {
        clip_active_logged_ = true;
        logEvent("WARN", "target_clipped",
          "protection=safety_clip, reason=" + safety_decision.message + ", source=" + target.target_source);
      }
    } else {
      clip_active_logged_ = false;
    }
  }

  target.target_age_ms = 0.0f;

  {
    std::scoped_lock<std::mutex> lock(mutex_);
    last_actions_ = last_actions;
  }

  target_pub_->publish(target);
  appendDebugTrace(state, target);
}

}  // namespace sim2real_runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sim2real_runtime::PolicyRuntimeNode>());
  rclcpp::shutdown();
  return 0;
}
