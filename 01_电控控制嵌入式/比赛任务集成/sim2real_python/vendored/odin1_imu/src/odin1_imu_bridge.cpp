#include "odin1_imu_bridge.h"

#include "lidar_api.h"
#include "lidar_api_type.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace {

constexpr const char* kBridgeVersion = "0.1.0";
constexpr std::size_t kMaxQueueSize = 1024;
constexpr int kDefaultMode = LIDAR_MODE_SLAM;

std::atomic<bool> g_running{false};
std::atomic<bool> g_sdk_initialized{false};
std::atomic<bool> g_device_connected{false};
std::atomic<bool> g_stream_started{false};

device_handle g_device = nullptr;

std::mutex g_state_mutex;
std::mutex g_queue_mutex;
std::condition_variable g_queue_cv;
std::deque<odin1_imu_sample_t> g_queue;
odin1_imu_sample_t g_latest_sample{};
bool g_has_latest_sample = false;

std::mutex g_error_mutex;
std::string g_last_error = "bridge not started";

/**
 * 输入: message[const std::string&]
 * 输出: 无
 * 作用: 线程安全地记录最近一次错误信息
 */
void set_last_error(const std::string& message) {
    std::lock_guard<std::mutex> lock(g_error_mutex);
    g_last_error = message;
}

/**
 * 输入: 无
 * 输出: 无
 * 作用: 清空内部 IMU 队列和最近一帧缓存
 */
void clear_queue_locked_state() {
    std::lock_guard<std::mutex> lock(g_queue_mutex);
    g_queue.clear();
    g_latest_sample = {};
    g_has_latest_sample = false;
}

/**
 * 输入: raw_sample[const imu_convert_data_t*]
 * 输出: odin1_imu_sample_t
 * 作用: 将 SDK IMU 结构转换为 bridge 对外结构
 */
odin1_imu_sample_t convert_sample(const imu_convert_data_t* raw_sample) {
    odin1_imu_sample_t converted{};
    if (raw_sample == nullptr) {
        return converted;
    }

    converted.accel_x = raw_sample->accel_x;
    converted.accel_y = raw_sample->accel_y;
    converted.accel_z = raw_sample->accel_z;
    converted.gyro_x = raw_sample->gyro_x;
    converted.gyro_y = raw_sample->gyro_y;
    converted.gyro_z = raw_sample->gyro_z;
    converted.stamp_ns = raw_sample->stamp;
    converted.sequence = raw_sample->sequence;
    return converted;
}

/**
 * 输入: 无
 * 输出: 无
 * 作用: 安全关闭当前设备与 SDK 资源
 */
void cleanup_device_and_sdk() {
    std::lock_guard<std::mutex> lock(g_state_mutex);

    if (g_device != nullptr) {
        try {
            if (g_stream_started.load()) {
                lidar_deactivate_stream_type(g_device, LIDAR_DT_RAW_IMU);
                lidar_stop_stream(g_device, kDefaultMode);
                g_stream_started = false;
            }

            lidar_unregister_stream_callback(g_device);
            lidar_close_device(g_device);
            lidar_destory_device(g_device);
        } catch (...) {
        }
        g_device = nullptr;
    }

    if (g_sdk_initialized.load()) {
        try {
            lidar_system_deinit();
        } catch (...) {
        }
        g_sdk_initialized = false;
    }

    g_device_connected = false;
}

/**
 * 输入: data[const lidar_data_t*], user_data[void*]
 * 输出: 无
 * 作用: 接收 SDK 回调中的 IMU 数据并写入内部缓存队列
 */
void lidar_data_callback(const lidar_data_t* data, void* user_data) {
    (void)user_data;

    if (!g_running.load() || data == nullptr) {
        return;
    }

    if (data->type != LIDAR_DT_RAW_IMU) {
        return;
    }

    if (data->stream.imageList[0].pAddr == nullptr) {
        set_last_error("sdk imu callback returned null payload");
        return;
    }

    const auto* raw_sample =
        static_cast<const imu_convert_data_t*>(data->stream.imageList[0].pAddr);
    odin1_imu_sample_t sample = convert_sample(raw_sample);

    {
        std::lock_guard<std::mutex> lock(g_queue_mutex);
        if (g_queue.size() >= kMaxQueueSize) {
            g_queue.pop_front();
        }
        g_queue.push_back(sample);
        g_latest_sample = sample;
        g_has_latest_sample = true;
    }

    g_queue_cv.notify_all();
}

/**
 * 输入: device_info[const lidar_device_info_t*], attach[bool]
 * 输出: 无
 * 作用: 响应 SDK 设备插拔事件并启动 IMU 数据流
 */
void lidar_device_callback(const lidar_device_info_t* device_info, bool attach) {
    if (!g_running.load()) {
        return;
    }

    if (!attach) {
        g_device_connected = false;
        g_stream_started = false;
        return;
    }

    if (device_info == nullptr) {
        set_last_error("sdk device callback returned null device info");
        return;
    }

    std::lock_guard<std::mutex> lock(g_state_mutex);

    if (g_device != nullptr) {
        return;
    }

    device_handle device_handle_local = nullptr;
    if (lidar_create_device(const_cast<lidar_device_info_t*>(device_info), &device_handle_local) != 0) {  // SDK接口，来源: include/lidar_api.h
        set_last_error("lidar_create_device failed");
        return;
    }

    if (lidar_open_device(device_handle_local) != 0) {  // SDK接口，来源: include/lidar_api.h
        set_last_error("lidar_open_device failed");
        lidar_destory_device(device_handle_local);  // SDK接口，来源: include/lidar_api.h
        return;
    }

    lidar_data_callback_info_t callback_info{};
    callback_info.data_callback = lidar_data_callback;
    callback_info.user_data = nullptr;
    if (lidar_register_stream_callback(device_handle_local, callback_info) != 0) {  // SDK接口，来源: include/lidar_api.h
        set_last_error("lidar_register_stream_callback failed");
        lidar_close_device(device_handle_local);  // SDK接口，来源: include/lidar_api.h
        lidar_destory_device(device_handle_local);  // SDK接口，来源: include/lidar_api.h
        return;
    }

    uint32_t dtof_subframe_odr = 0;
    if (lidar_start_stream(device_handle_local, kDefaultMode, dtof_subframe_odr) != 0) {  // SDK接口，来源: include/lidar_api.h
        (void)dtof_subframe_odr;
        set_last_error("lidar_start_stream failed");
        lidar_unregister_stream_callback(device_handle_local);  // SDK接口，来源: include/lidar_api.h
        lidar_close_device(device_handle_local);  // SDK接口，来源: include/lidar_api.h
        lidar_destory_device(device_handle_local);  // SDK接口，来源: include/lidar_api.h
        return;
    }

    if (lidar_activate_stream_type(device_handle_local, LIDAR_DT_RAW_IMU) != 0) {  // SDK接口，来源: include/lidar_api.h
        set_last_error("lidar_activate_stream_type(raw_imu) failed");
        lidar_stop_stream(device_handle_local, kDefaultMode);  // SDK接口，来源: include/lidar_api.h
        lidar_unregister_stream_callback(device_handle_local);  // SDK接口，来源: include/lidar_api.h
        lidar_close_device(device_handle_local);  // SDK接口，来源: include/lidar_api.h
        lidar_destory_device(device_handle_local);  // SDK接口，来源: include/lidar_api.h
        return;
    }

    g_device = device_handle_local;
    g_stream_started = true;
    g_device_connected = true;
    set_last_error("");
    g_queue_cv.notify_all();
}

}  // namespace

extern "C" {

/**
 * 输入: 无
 * 输出: const char*
 * 作用: 返回当前 bridge 的版本字符串
 */
const char* odin1_imu_version(void) {
    return kBridgeVersion;
}

/**
 * 输入: timeout_ms[int]
 * 输出: int, 0 表示成功, 非 0 表示失败
 * 作用: 初始化 SDK, 等待设备连接并开始 IMU 数据流
 */
int odin1_imu_start(int timeout_ms) {
    if (timeout_ms <= 0) {
        timeout_ms = 5000;
    }

    if (g_running.load()) {
        return 0;
    }

    clear_queue_locked_state();
    set_last_error("waiting for odin1 device");

    if (lidar_system_init(lidar_device_callback) != 0) {  // SDK接口，来源: include/lidar_api.h
        set_last_error("lidar_system_init failed");
        return -1;
    }

    g_sdk_initialized = true;
    g_running = true;

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (g_device_connected.load()) {
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    set_last_error("timeout waiting for odin1 imu stream");
    odin1_imu_stop();
    return -2;
}

/**
 * 输入: 无
 * 输出: 无
 * 作用: 停止数据流并释放 SDK 资源
 */
void odin1_imu_stop(void) {
    g_running = false;
    cleanup_device_and_sdk();
    clear_queue_locked_state();
    g_queue_cv.notify_all();
}

/**
 * 输入: 无
 * 输出: int, 1 表示运行中, 0 表示未运行
 * 作用: 返回当前 bridge 是否处于运行状态
 */
int odin1_imu_is_running(void) {
    return g_running.load() ? 1 : 0;
}

/**
 * 输入: timeout_ms[int]
 * 输出: int, 1 表示有数据可读, 0 表示超时, 负数表示异常
 * 作用: 阻塞等待 IMU 数据到达
 */
int odin1_imu_wait_for_data(int timeout_ms) {
    if (!g_running.load()) {
        return -1;
    }

    std::unique_lock<std::mutex> lock(g_queue_mutex);
    const bool ready = g_queue_cv.wait_for(
        lock,
        std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 1000),
        [] { return !g_queue.empty() || !g_running.load(); });

    if (!g_running.load()) {
        return -1;
    }

    return ready && !g_queue.empty() ? 1 : 0;
}

/**
 * 输入: out_sample[odin1_imu_sample_t*]
 * 输出: int, 1 表示成功取出一帧, 0 表示队列为空, 负数表示异常
 * 作用: 从内部队列中弹出一帧 IMU 数据
 */
int odin1_imu_pop_sample(odin1_imu_sample_t* out_sample) {
    if (out_sample == nullptr) {
        set_last_error("odin1_imu_pop_sample received null output pointer");
        return -1;
    }

    std::lock_guard<std::mutex> lock(g_queue_mutex);
    if (g_queue.empty()) {
        return 0;
    }

    *out_sample = g_queue.front();
    g_queue.pop_front();
    return 1;
}

/**
 * 输入: out_sample[odin1_imu_sample_t*]
 * 输出: int, 1 表示成功读取, 0 表示当前还没有数据, 负数表示异常
 * 作用: 获取最近一帧 IMU 数据, 不会从队列中删除
 */
int odin1_imu_get_latest(odin1_imu_sample_t* out_sample) {
    if (out_sample == nullptr) {
        set_last_error("odin1_imu_get_latest received null output pointer");
        return -1;
    }

    std::lock_guard<std::mutex> lock(g_queue_mutex);
    if (!g_has_latest_sample) {
        return 0;
    }

    *out_sample = g_latest_sample;
    return 1;
}

/**
 * 输入: 无
 * 输出: const char*
 * 作用: 返回最近一次错误信息
 */
const char* odin1_imu_last_error(void) {
    std::lock_guard<std::mutex> lock(g_error_mutex);
    return g_last_error.c_str();
}

}  // extern "C"
