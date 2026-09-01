# sim2real_ros2 架构说明

## 设计目标

- 保留已验证的 RL 部署契约不变
- 将低延迟循环从 Python 迁移至 C++
- 暴露标准 ROS 2 接口用于导航和系统集成
- 保持安全边界独立于策略正确性

## 各包职责

### `sim2real_interfaces`（接口消息）

定义最小化的运行时消息：

- `RuntimeState`
  硬件桥接发布的归一化运行时状态快照
- `RuntimeTarget`
  策略运行时发送至硬件桥接的最新策略目标

### `sim2real_common`（共享常量）

存储编译期常量和部署契约辅助：

- 观测维度和字段布局
- 动作维度和轮子索引
- 关节顺序和默认站姿
- 动作缩放因子和默认循环频率
- Mahony 姿态滤波器
- 站立平衡控制器
- 安全监控器（SafetyMonitor / RuntimeGuard）

### `sim2real_hw`（硬件桥接）

拥有硬件侧执行循环和安全边界：

- RobStride CAN 收发
- IMU 与 Odin 状态采集
- 电机丢帧检测与保活逻辑（holdover）
- 看门狗与阻尼刹车
- 发布 `RuntimeState`
- 订阅 `RuntimeTarget`
- 订阅 `/odom` 里程计数据

目标热路径：

- 以 `200Hz` 频率读取状态
- 应用最新安全目标
- 超时或安全违规时立即停机

### `sim2real_runtime`（策略运行时）

拥有策略侧执行：

- 订阅 `RuntimeState`
- 按当前部署契约精确构建 `53D` 观测
- 以 `50Hz` 运行 ONNXRuntime 推理
- 对 raw_action 做 `[-10, 10]` 安全裁剪
- 发布 `RuntimeTarget`
- 仲裁命令来源：estop > safety_hold > startup > navigation > web

同时包含：
- `odom_relay_node`：将 `/odin1/odometry` 中继为 `/odom`，帧名 `odin1_base_link` → `base_link`，并广播 TF

### `sim2real_nav2`（导航配置）

拥有：

- Nav2 参数文件（planner、controller、costmap、AMCL、behavior）
- Nav2 启动文件（含 AMCL、costmap 生命周期节点、pointcloud_to_laserscan）

### `sim2real_bringup`（启动管理）

拥有：

- 参数文件
- 启动组合
- 运行时模式选择
- 集成 odin_ros_driver、sim2real_nav2 的条件启动

## 迁移规则

1. 优化之前先冻结当前契约
2. 先迁移传输和循环结构，再调整控制算法
3. C++ 运行时未达到影子模式一致性前，保留 Python 运行时可用
4. 按段测量延迟：
   - 观测延迟
   - 策略推理延迟
   - 目标传输延迟
   - 执行器响应延迟

## 首个里程碑

首个里程碑不是"机器人在 ROS 2 下行走"，而是：

1. `sim2real_hw` 发布稳定的 `RuntimeState`
2. `sim2real_runtime` 从该状态构建正确的 `53D` 观测
3. `sim2real_runtime` 以 `50Hz` 发布 `RuntimeTarget`
4. `sim2real_hw` 消费最新目标并执行超时刹车
5. `cmd_vel` 可通过 ROS 2 注入而不改变策略契约

> ✅ 以上里程碑已全部完成。
