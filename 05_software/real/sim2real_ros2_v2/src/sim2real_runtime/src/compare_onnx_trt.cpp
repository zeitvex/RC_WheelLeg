#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include "sim2real_common/deployment_contract.hpp"

namespace
{

class TensorRtLogger final : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (msg == nullptr) {
      return;
    }
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << msg << std::endl;
    }
  }
};

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

class TensorRtRunner
{
public:
  explicit TensorRtRunner(const std::string & engine_path)
  {
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file) {
      throw std::runtime_error("Failed to open TensorRT engine: " + engine_path);
    }
    engine_file.seekg(0, std::ios::end);
    const std::streamsize engine_size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);
    if (engine_size <= 0) {
      throw std::runtime_error("TensorRT engine is empty: " + engine_path);
    }

    std::vector<char> engine_data(static_cast<std::size_t>(engine_size));
    if (!engine_file.read(engine_data.data(), engine_size)) {
      throw std::runtime_error("Failed to read TensorRT engine: " + engine_path);
    }

    runtime_ = nvinfer1::createInferRuntime(logger_);
    if (runtime_ == nullptr) {
      throw std::runtime_error("Failed to create TensorRT runtime");
    }

    engine_ = runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size());
    if (engine_ == nullptr) {
      throw std::runtime_error("Failed to deserialize TensorRT engine");
    }

    context_ = engine_->createExecutionContext();
    if (context_ == nullptr) {
      throw std::runtime_error("Failed to create TensorRT execution context");
    }

    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * tensor_name = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(tensor_name) == nvinfer1::TensorIOMode::kINPUT) {
        input_name_ = tensor_name;
      } else {
        output_name_ = tensor_name;
      }
    }

    if (input_name_.empty() || output_name_.empty()) {
      throw std::runtime_error("Failed to resolve TensorRT IO tensor names");
    }

    if (!context_->setInputShape(
          input_name_.c_str(),
          nvinfer1::Dims2{1, static_cast<int>(sim2real_common::DeploymentContract::kObsDim)})) {
      throw std::runtime_error("Failed to set TensorRT input shape");
    }

    if (cudaStreamCreate(&stream_) != cudaSuccess) {
      throw std::runtime_error("Failed to create CUDA stream");
    }

    const std::size_t input_bytes = sizeof(float) * sim2real_common::DeploymentContract::kObsDim;
    const std::size_t output_bytes = sizeof(float) * sim2real_common::DeploymentContract::kActionDim;
    if (cudaMalloc(&input_buffer_, input_bytes) != cudaSuccess ||
        cudaMalloc(&output_buffer_, output_bytes) != cudaSuccess) {
      throw std::runtime_error("Failed to allocate TensorRT buffers");
    }

    if (!context_->setTensorAddress(input_name_.c_str(), input_buffer_) ||
        !context_->setTensorAddress(output_name_.c_str(), output_buffer_)) {
      throw std::runtime_error("Failed to bind TensorRT buffers");
    }
  }

  ~TensorRtRunner()
  {
    if (input_buffer_ != nullptr) {
      cudaFree(input_buffer_);
    }
    if (output_buffer_ != nullptr) {
      cudaFree(output_buffer_);
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
    destroyTensorRtObject(context_);
    destroyTensorRtObject(engine_);
    destroyTensorRtObject(runtime_);
  }

  std::array<float, sim2real_common::DeploymentContract::kActionDim> run(
    const std::array<float, sim2real_common::DeploymentContract::kObsDim> & obs)
  {
    std::array<float, sim2real_common::DeploymentContract::kActionDim> out{};
    const std::size_t input_bytes = sizeof(float) * obs.size();
    const std::size_t output_bytes = sizeof(float) * out.size();

    if (cudaMemcpyAsync(input_buffer_, obs.data(), input_bytes, cudaMemcpyHostToDevice, stream_) != cudaSuccess) {
      throw std::runtime_error("TensorRT H2D copy failed");
    }
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("TensorRT enqueue failed");
    }
    if (cudaMemcpyAsync(out.data(), output_buffer_, output_bytes, cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
      throw std::runtime_error("TensorRT D2H copy failed");
    }
    if (cudaStreamSynchronize(stream_) != cudaSuccess) {
      throw std::runtime_error("TensorRT stream sync failed");
    }
    return out;
  }

private:
  TensorRtLogger logger_;
  nvinfer1::IRuntime * runtime_{nullptr};
  nvinfer1::ICudaEngine * engine_{nullptr};
  nvinfer1::IExecutionContext * context_{nullptr};
  cudaStream_t stream_{nullptr};
  void * input_buffer_{nullptr};
  void * output_buffer_{nullptr};
  std::string input_name_;
  std::string output_name_;
};

class OnnxRunner
{
public:
  explicit OnnxRunner(const std::string & onnx_path)
  : env_(ORT_LOGGING_LEVEL_WARNING, "compare_onnx_trt")
  {
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetInterOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    session_ = std::make_unique<Ort::Session>(env_, onnx_path.c_str(), session_options);
    memory_info_ = std::make_unique<Ort::MemoryInfo>(
      Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU));

    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name = session_->GetInputNameAllocated(0, allocator);
    auto output_name = session_->GetOutputNameAllocated(0, allocator);
    input_name_ = input_name.get();
    output_name_ = output_name.get();
    input_name_ptr_ = input_name_.c_str();
    output_name_ptr_ = output_name_.c_str();

    input_shape_ = {1, static_cast<std::int64_t>(sim2real_common::DeploymentContract::kObsDim)};
    output_shape_ = {1, static_cast<std::int64_t>(sim2real_common::DeploymentContract::kActionDim)};
  }

  std::array<float, sim2real_common::DeploymentContract::kActionDim> run(
    const std::array<float, sim2real_common::DeploymentContract::kObsDim> & obs)
  {
    std::array<float, sim2real_common::DeploymentContract::kActionDim> out{};

    auto input_tensor = Ort::Value::CreateTensor<float>(
      *memory_info_,
      const_cast<float *>(obs.data()),
      obs.size(),
      input_shape_.data(),
      input_shape_.size());

    auto output_tensor = Ort::Value::CreateTensor<float>(
      *memory_info_,
      out.data(),
      out.size(),
      output_shape_.data(),
      output_shape_.size());

    session_->Run(
      Ort::RunOptions{nullptr},
      &input_name_ptr_,
      &input_tensor,
      1,
      &output_name_ptr_,
      &output_tensor,
      1);

    return out;
  }

private:
  Ort::Env env_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::MemoryInfo> memory_info_;
  std::string input_name_;
  std::string output_name_;
  const char * input_name_ptr_{nullptr};
  const char * output_name_ptr_{nullptr};
  std::array<std::int64_t, 2> input_shape_{};
  std::array<std::int64_t, 2> output_shape_{};
};

struct DiffStats
{
  double max_abs_diff{0.0};
  double mean_abs_diff{0.0};
};

DiffStats compareOutputs(
  const std::array<float, sim2real_common::DeploymentContract::kActionDim> & a,
  const std::array<float, sim2real_common::DeploymentContract::kActionDim> & b)
{
  DiffStats stats;
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double diff = std::abs(static_cast<double>(a[i]) - static_cast<double>(b[i]));
    stats.max_abs_diff = std::max(stats.max_abs_diff, diff);
    sum += diff;
  }
  stats.mean_abs_diff = sum / static_cast<double>(a.size());
  return stats;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: compare_onnx_trt <model.onnx> <model.engine> [num_samples] [seed] [max_abs_threshold]\n";
    return 2;
  }

  const std::string onnx_path = argv[1];
  const std::string engine_path = argv[2];
  const int num_samples = argc >= 4 ? std::max(1, std::atoi(argv[3])) : 32;
  const std::uint32_t seed = argc >= 5 ? static_cast<std::uint32_t>(std::strtoul(argv[4], nullptr, 10)) : 12345U;
  const double max_abs_threshold = argc >= 6 ? std::atof(argv[5]) : 1.0e-2;

  try {
    OnnxRunner onnx_runner(onnx_path);
    TensorRtRunner trt_runner(engine_path);

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

    double worst_max_abs = 0.0;
    double worst_mean_abs = 0.0;
    int worst_sample = -1;

    for (int sample_idx = 0; sample_idx < num_samples; ++sample_idx) {
      std::array<float, sim2real_common::DeploymentContract::kObsDim> obs{};
      for (float & v : obs) {
        v = dist(rng);
      }

      const auto onnx_out = onnx_runner.run(obs);
      const auto trt_out = trt_runner.run(obs);
      const auto diff = compareOutputs(onnx_out, trt_out);

      if (diff.max_abs_diff > worst_max_abs) {
        worst_max_abs = diff.max_abs_diff;
        worst_mean_abs = diff.mean_abs_diff;
        worst_sample = sample_idx;
      }

      std::cout << "sample=" << sample_idx
                << " max_abs_diff=" << diff.max_abs_diff
                << " mean_abs_diff=" << diff.mean_abs_diff
                << '\n';
    }

    std::cout << "summary worst_sample=" << worst_sample
              << " worst_max_abs_diff=" << worst_max_abs
              << " worst_mean_abs_diff=" << worst_mean_abs
              << " threshold=" << max_abs_threshold
              << '\n';

    if (worst_max_abs > max_abs_threshold) {
      std::cerr << "FAILED: TensorRT output deviates from ONNX Runtime beyond threshold.\n";
      return 1;
    }

    std::cout << "PASSED: TensorRT output matches ONNX Runtime within threshold.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "ERROR: " << e.what() << '\n';
    return 1;
  }
}
