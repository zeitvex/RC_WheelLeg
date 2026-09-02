#ifndef ODIN1_IMU_BRIDGE_H
#define ODIN1_IMU_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 输入: 无
 * 输出: odin1_imu_sample_t
 * 作用: 描述一帧 IMU 数据, 供 C/C++/Python 共享使用
 * 注意: 坐标系已从 SDK 重映射为 ROS 标准 (right-handed: x前 y左 z上)
 *       重映射规则: accel_x←SDK_accel_y, accel_y←-SDK_accel_x, accel_z←SDK_accel_z
 *                   gyro_x ←SDK_gyro_y,  gyro_y ←-SDK_gyro_x,  gyro_z ←SDK_gyro_z
 */
typedef struct odin1_imu_sample_t {
    float accel_x;  /* m/s², ROS 标准坐标系 */
    float accel_y;
    float accel_z;
    float gyro_x;   /* rad/s, ROS 标准坐标系 */
    float gyro_y;
    float gyro_z;
    uint64_t stamp_ns;
    uint64_t sequence;
} odin1_imu_sample_t;

/**
 * 输入: 无
 * 输出: odin1_odom_type_e
 * 作用: 标识里程计数据类型
 */
typedef enum {
    ODIN1_ODOM_STANDARD = 0,   /* 标准里程计, 含位置/姿态/速度/协方差 */
    ODIN1_ODOM_HIGHFREQ,        /* 高频里程计, 含位置/姿态 */
    ODIN1_ODOM_TF,              /* TF 变换(重定位后), 全局坐标系下位姿 */
} odin1_odom_type_e;

/**
 * 输入: 无
 * 输出: odin1_odom_sample_t
 * 作用: 描述一帧里程计数据, 统一容纳三种类型, 供 C/C++/Python 共享使用
 * 缩放: SDK 原始 int64 值已按 ÷1e6 转换为 double (位置:米, 姿态:四元数, 速度:m/s, rad/s)
 * 注意: 速度与协方差仅 ODIN1_ODOM_STANDARD 有效, 其他类型为零
 */
typedef struct odin1_odom_sample_t {
    odin1_odom_type_e type;
    uint64_t stamp_ns;
    double pos_x;          /* 米, odom 坐标系 */
    double pos_y;
    double pos_z;
    double orient_w;       /* 单位四元数 */
    double orient_x;
    double orient_y;
    double orient_z;
    double linear_vel_x;   /* m/s (仅 STANDARD 有效) */
    double linear_vel_y;
    double linear_vel_z;
    double angular_vel_x;  /* rad/s (仅 STANDARD 有效) */
    double angular_vel_y;
    double angular_vel_z;
    double pose_cov[36];   /* 位姿协方差 (仅 STANDARD 有效) */
    double twist_cov[36];  /* 速度协方差 (仅 STANDARD 有效) */
} odin1_odom_sample_t;

/**
 * 输入: 无
 * 输出: const char*
 * 作用: 返回当前 bridge 的版本字符串
 */
const char* odin1_imu_version(void);

/**
 * 输入: timeout_ms[int]
 * 输出: int, 0 表示成功, 非 0 表示失败
 * 作用: 初始化 SDK, 等待设备连接并开始 IMU 数据流
 */
int odin1_imu_start(int timeout_ms);

/**
 * 输入: 无
 * 输出: 无
 * 作用: 停止数据流并释放 SDK 资源
 */
void odin1_imu_stop(void);

/**
 * 输入: 无
 * 输出: int, 1 表示运行中, 0 表示未运行
 * 作用: 返回当前 bridge 是否处于运行状态
 */
int odin1_imu_is_running(void);

/**
 * 输入: timeout_ms[int]
 * 输出: int, 1 表示有数据可读, 0 表示超时, 负数表示异常
 * 作用: 阻塞等待 IMU 数据到达
 */
int odin1_imu_wait_for_data(int timeout_ms);

/**
 * 输入: out_sample[odin1_imu_sample_t*]
 * 输出: int, 1 表示成功取出一帧, 0 表示队列为空, 负数表示异常
 * 作用: 从内部队列中弹出一帧 IMU 数据
 */
int odin1_imu_pop_sample(odin1_imu_sample_t* out_sample);

/**
 * 输入: out_sample[odin1_imu_sample_t*]
 * 输出: int, 1 表示成功读取, 0 表示当前还没有数据, 负数表示异常
 * 作用: 获取最近一帧 IMU 数据, 不会从队列中删除
 */
int odin1_imu_get_latest(odin1_imu_sample_t* out_sample);

/**
 * 输入: 无
 * 输出: const char*
 * 作用: 返回最近一次错误信息
 */
const char* odin1_imu_last_error(void);

// ---------------------------------------------------------------------------
// Odom (里程计) 接口
// ---------------------------------------------------------------------------

/**
 * 输入: timeout_ms[int]
 * 输出: int, 1 表示有数据可读, 0 表示超时, 负数表示异常
 * 作用: 阻塞等待里程计数据到达（任意类型）
 */
int odin1_odom_wait_for_data(int timeout_ms);

/**
 * 输入: out_sample[odin1_odom_sample_t*]
 * 输出: int, 1 表示成功取出一帧, 0 表示队列为空, 负数表示异常
 * 作用: 从内部队列中弹出一帧里程计数据
 */
int odin1_odom_pop_sample(odin1_odom_sample_t* out_sample);

/**
 * 输入: out_sample[odin1_odom_sample_t*]
 * 输出: int, 1 表示成功读取, 0 表示当前还没有数据, 负数表示异常
 * 作用: 获取最近一帧里程计数据, 不会从队列中删除
 */
int odin1_odom_get_latest(odin1_odom_sample_t* out_sample);

/**
 * 输入: 无
 * 输出: const char*
 * 作用: 返回最近一次里程计错误信息
 */
const char* odin1_odom_last_error(void);

#ifdef __cplusplus
}
#endif

#endif
