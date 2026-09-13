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
 */
typedef struct odin1_imu_sample_t {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint64_t stamp_ns;
    uint64_t sequence;
} odin1_imu_sample_t;

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

#ifdef __cplusplus
}
#endif

#endif
