# 迁移计划

## Phase 1: 硬件核心迁移 ✅ 已完成

将当前高频热路径从 Python 迁出。

吸收的源文件：

- `sim2real/interface/motor_driver.py`
- `sim2real/interface/motor_mapping.py`
- `sim2real/interface/imu_client.py`
- `sim2real/safety/runtime_guard.py`
- `sim2real/web/session.py`

交付物：

- C++ SocketCAN 电机总线封装
- C++ 状态缓存
- target 超时保活（timeout hold）
- 阻尼刹车 / 急停通路
- 发布 `RuntimeState`

## Phase 2: 策略运行时迁移 ✅ 已完成

吸收的源文件：

- `sim2real/policy/policy_runner.py`
- `sim2real/interface/real_io.py`
- `sim2real/web/session.py`

交付物：

- 精确的 `53D` 观测构造器
- ONNXRuntime C++ 推理封装
- `50Hz` 策略定时器
- 命令平滑与来源仲裁
- raw_action `[-10, 10]` 安全裁剪
- 发布 `RuntimeTarget`

## Phase 3: ROS 2 系统集成 ✅ 已完成

参考的源项目：

- `00_ reference/odin_ros_driver`
- `00_ reference/EDULITE_A3/el_a3_ros`
- `00_ reference/rl_sar`

交付物：

- `cmd_vel` / `cmd_vel_stamped` 输入（支持 Twist 和 TwistStamped）
- `odom_relay_node`：里程计中继 + TF 广播（odom → base_link）
- 诊断话题
- rosbag/foxglove 可观测性

## Phase 4: 导航集成 ✅ 已完成

目标：

- 导航通过 ROS 2 发送身体速度指令
- RL 运行时保持为 locomotion 控制器
- 看门狗和安全边界始终在导航之下

规则：

- 导航绝不直接写电机指令
- 策略契约在重新训练前保持不变
- 任何新增历史项或里程计项必须版本化

## 当前状态

所有 4 个 Phase 已全部完成。以下为已实现的关键组件：

| 组件 | 节点 | 说明 |
|------|------|------|
| 硬件桥接 | `sim2real_hw_node` | 200Hz CAN 收发 + IMU + Mahony + 安全 |
| 策略运行时 | `sim2real_runtime_node` | 50Hz ONNX 推理 + 53D 观测 + raw_action clip |
| 里程计中继 | `odom_relay_node` | /odin1/odometry → /odom + odom→base_link TF |
| 导航栈 | Nav2 全套节点 | AMCL + costmap + DWB + Navfn + BT + lifecycle |
| 传感器驱动 | `odin_ros_driver` | IMU + 点云 + 里程计原始发布 |
| 点云转换 | `pointcloud_to_laserscan` | /odin1/cloud_slam → /scan (供 AMCL 使用) |
