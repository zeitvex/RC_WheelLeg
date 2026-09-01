builtin_interfaces/Time stamp
uint32 sequence

float32[16] joint_pos
float32[16] joint_vel
float32[16] joint_torque

float32[3] imu_gyro
float32[3] imu_accel
float32[4] quat_wxyz
float32[3] projected_gravity

float32 imu_age_ms
bool imu_fresh

float32 odom_age_ms
bool odom_fresh
float32[3] odom_pos
float32[4] odom_quat_wxyz
float32[3] odom_linear_vel
float32[3] odom_angular_vel
float32[3] odom_local_pos
float32 odom_local_yaw

uint32 fresh_count
uint32 holdover_count
uint32 stale_max
uint32[16] update_counts

string source
