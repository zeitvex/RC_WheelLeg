#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sim2real_common
{

class LowPassFilter
{
public:
  LowPassFilter(double cutoff_freq, double dt, std::size_t dim)
  : dim_(dim), initialized_(false)
  {
    alpha_ = static_cast<float>(1.0 - std::exp(-2.0 * M_PI * cutoff_freq * dt));
    y_prev_.resize(dim, 0.0f);
  }

  void filter(const float* x, float* y)
  {
    if (!initialized_) {
      for (std::size_t i = 0; i < dim_; ++i) {
        y_prev_[i] = x[i];
      }
      initialized_ = true;
    }
    for (std::size_t i = 0; i < dim_; ++i) {
      y[i] = alpha_ * x[i] + (1.0f - alpha_) * y_prev_[i];
      y_prev_[i] = y[i];
    }
  }

  void filter(const std::vector<float>& x, std::vector<float>& y)
  {
    filter(x.data(), y.data());
  }

  void reset()
  {
    initialized_ = false;
  }

private:
  float alpha_;
  std::size_t dim_;
  bool initialized_;
  std::vector<float> y_prev_;
};

} // namespace sim2real_common
