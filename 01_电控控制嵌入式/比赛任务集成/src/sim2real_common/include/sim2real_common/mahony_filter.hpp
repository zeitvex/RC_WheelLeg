#pragma once

#include <array>
#include <cmath>
#include <algorithm>

namespace sim2real_common
{

// Helper to calculate gravity orientation from quaternion [w, x, y, z]
inline std::array<float, 3> get_gravity_orientation(const std::array<float, 4>& quat_wxyz)
{
  float qw = quat_wxyz[0];
  float qx = quat_wxyz[1];
  float qy = quat_wxyz[2];
  float qz = quat_wxyz[3];
  
  float gx = 2.0f * (-qz * qx + qw * qy);
  float gy = -2.0f * (qz * qy + qw * qx);
  float gz = 1.0f - 2.0f * (qw * qw + qz * qz);
  return {gx, gy, gz};
}

// Helper to create quaternion from acceleration vector
inline std::array<float, 4> quat_from_accel(const std::array<float, 3>& accel)
{
  float norm_a = std::sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
  if (norm_a < 1e-9f) {
    return {1.0f, 0.0f, 0.0f, 0.0f};
  }
  
  float ax = accel[0] / norm_a;
  float ay = accel[1] / norm_a;
  float az = accel[2] / norm_a;
  
  // Ref gravity vector is [0.0, 0.0, 1.0]
  float cross_x = -ay;
  float cross_y = ax;
  float cross_z = 0.0f;
  float dot = az;
  
  if (dot < -0.999999f) {
    return {0.0f, 1.0f, 0.0f, 0.0f};
  }
  
  float s = std::sqrt((1.0f + dot) * 2.0f);
  std::array<float, 4> q = {
    s * 0.5f,
    cross_x / s,
    cross_y / s,
    cross_z / s
  };
  
  float norm_q = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (norm_q < 1e-9f) {
    return {1.0f, 0.0f, 0.0f, 0.0f};
  }
  q[0] /= norm_q;
  q[1] /= norm_q;
  q[2] /= norm_q;
  q[3] /= norm_q;
  
  return q;
}

class MahonyFilter
{
public:
  MahonyFilter(float kp = 2.0f, float ki = 0.0f)
  : kp_(kp), ki_(ki)
  {
    q_ = {1.0f, 0.0f, 0.0f, 0.0f};
    e_int_ = {0.0f, 0.0f, 0.0f};
  }

  void reset_with_accel(const std::array<float, 3>& accel)
  {
    q_ = quat_from_accel(accel);
    e_int_ = {0.0f, 0.0f, 0.0f};
  }

  std::array<float, 4> update(const std::array<float, 3>& accel, const std::array<float, 3>& gyro, float dt)
  {
    float norm_a = std::sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    std::array<float, 3> gyro_corr = gyro;
    
    if (norm_a > 1e-6f) {
      float ax = accel[0] / norm_a;
      float ay = accel[1] / norm_a;
      float az = accel[2] / norm_a;
      
      float qw = q_[0];
      float qx = q_[1];
      float qy = q_[2];
      float qz = q_[3];
      
      float vx = 2.0f * (qx * qz - qw * qy);
      float vy = 2.0f * (qw * qx + qy * qz);
      float vz = qw * qw - qx * qx - qy * qy + qz * qz;
      
      // Error = cross(a, v)
      float ex = ay * vz - az * vy;
      float ey = az * vx - ax * vz;
      float ez = ax * vy - ay * vx;
      
      if (ki_ > 0.0f) {
        e_int_[0] += ex * dt;
        e_int_[1] += ey * dt;
        e_int_[2] += ez * dt;
      } else {
        e_int_ = {0.0f, 0.0f, 0.0f};
      }
      
      gyro_corr[0] += kp_ * ex + ki_ * e_int_[0];
      gyro_corr[1] += kp_ * ey + ki_ * e_int_[1];
      gyro_corr[2] += kp_ * ez + ki_ * e_int_[2];
    }
    
    float qw = q_[0];
    float qx = q_[1];
    float qy = q_[2];
    float qz = q_[3];
    
    float q_dot_w = 0.5f * (-qx * gyro_corr[0] - qy * gyro_corr[1] - qz * gyro_corr[2]);
    float q_dot_x = 0.5f * ( qw * gyro_corr[0] + qy * gyro_corr[2] - qz * gyro_corr[1]);
    float q_dot_y = 0.5f * ( qw * gyro_corr[1] - qx * gyro_corr[2] + qz * gyro_corr[0]);
    float q_dot_z = 0.5f * ( qw * gyro_corr[2] + qx * gyro_corr[1] - qy * gyro_corr[0]);
    
    q_[0] += q_dot_w * dt;
    q_[1] += q_dot_x * dt;
    q_[2] += q_dot_y * dt;
    q_[3] += q_dot_z * dt;
    
    float norm_q = std::sqrt(q_[0]*q_[0] + q_[1]*q_[1] + q_[2]*q_[2] + q_[3]*q_[3]) + 1e-9f;
    q_[0] /= norm_q;
    q_[1] /= norm_q;
    q_[2] /= norm_q;
    q_[3] /= norm_q;
    
    return q_;
  }

  const std::array<float, 4>& get_q() const { return q_; }

private:
  float kp_;
  float ki_;
  std::array<float, 4> q_;
  std::array<float, 3> e_int_;
};

} // namespace sim2real_common
