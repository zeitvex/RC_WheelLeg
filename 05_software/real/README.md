# 真机控制版本演进

本目录保存 16DOF 轮足机器人从早期 Python 闭环到 ROS 2 部署的真机控制演进。

## `ik_real`

基于几何逆运动学和轨迹插值的真机控制探索，不依赖强化学习策略。主要用于验证电机接口、关节映射和姿态轨迹。

## `sim2real`

第一代 Python 策略部署栈，包含：

- 53D 观测到 16D 动作的策略运行时
- 电机映射和真机 IO
- IMU 接入
- 站立初始化与平衡
- 运行时安全检查和阻尼刹车
- Web 调试界面
- 对齐、标定和独立检查工具

部署说明见 [`sim2real/README.md`](sim2real/README.md) 与 [`sim2real/DEPLOYMENT.md`](sim2real/DEPLOYMENT.md)。

## `sim2real_v2`

Python Sim2Real v2，保留 `53D -> 16D` 策略接口，并增加电机反馈新鲜度、Odin odom 诊断、命令平滑、Web 运行时诊断和安全监控工具，对应 `v0.9.0`。

部署说明见 [`sim2real_v2/README.md`](sim2real_v2/README.md) 与 [`sim2real_v2/DEPLOYMENT.md`](sim2real_v2/DEPLOYMENT.md)。

## ROS 2/C++ 版本线

### `sim2real_ros2`（初版，`v0.10.0`）

无后缀目录固定表示 ROS 2/C++ Sim2Real 初版：将策略热路径迁移为 50 Hz C++ 推理和 200 Hz CAN 电机循环，并加入 ROS 2 消息、命令仲裁、Nav2 与统一启动结构。原始快照未随工程保存 Odin ROS 2 驱动源码，依赖边界见 [`sim2real_ros2/README.md`](sim2real_ros2/README.md)。

### `sim2real_ros2_v2`（`v0.11.0`～`v0.12.0`）

在 `v0.11.0` 中，该目录是 ROS 2 Sim2Real v2 导航原型，增加简单导航节点、PCD 交互定位、任务点/任务序列和 Web 导航调试。

`v0.11.1` 在同一路径继续演进，首次归档完整 Odin 驱动、TensorRT、多策略切换和硬件诊断，并使用 `hip=0.670`、`knee=-1.390` 的调参站姿。

`v0.12.0` 仍在同一路径上形成里程计导航联调快照：固定纯里程计模式，加入 odom fallback 的 TF 冲突保护、A_min 路线和多地图工具；默认 Rough 策略为 `model_9600`，默认站姿回到比赛站姿。当前该目录保持 `v0.12.0` 快照，阶段说明见 [`sim2real_ros2_v2/README.md`](sim2real_ros2_v2/README.md)。

### `sim2real_ros2_v3`（最终比赛版；代码快照 `v1.0.0`，规范目录 `v1.1.0`）

第三版来自原始目录 `sim2real_ros2_v2(last_not_slalom_1050)`，整理时正式命名为 `sim2real_ros2_v3`。它是 1050 分比赛最终部署，包含 `model_6800` Rough、`model_84` Wall、最终路线、完整 Odin 驱动、CAN 和触控屏。部署说明见 [`sim2real_ros2_v3/README.md`](sim2real_ros2_v3/README.md)。

## 实机记录

[![第一代 Sim2Real 真机验证](../../06_assets/images/early_sim2real_preview.jpg)](../../06_assets/videos/early_sim2real.mp4)

该视频记录了这一阶段的早期真机测试，用于对应本目录中的第一代控制与部署实现。
