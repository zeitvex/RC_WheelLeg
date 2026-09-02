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

Python Sim2Real v2，保留 `53D -> 16D` 策略接口，并增加电机反馈新鲜度、Odin odom 诊断、命令平滑、Web 运行时诊断和安全监控工具。该版本对应重排主线的 `v0.9.0`。

部署说明见 [`sim2real_v2/README.md`](sim2real_v2/README.md) 与 [`sim2real_v2/DEPLOYMENT.md`](sim2real_v2/DEPLOYMENT.md)。

## `sim2real_ros2`

ROS 2/C++ Sim2Real 初版，将策略热路径迁移为 50 Hz C++ 推理和 200 Hz CAN 电机循环，并加入 ROS 2 消息、命令仲裁、Nav2 与统一启动结构。该版本对应重排主线的 `v0.10.0`。

原始快照没有随工程保存 Odin ROS 2 驱动源码，该依赖边界见 [`sim2real_ros2/README.md`](sim2real_ros2/README.md)。

## `sim2real_ros2_v2`

ROS 2 Sim2Real v2 导航原型，在初版基础上增加简单导航节点、PCD 交互定位、任务点/任务序列和 Web 导航调试。该版本对应重排主线的 `v0.11.0`。

本阶段的两份大体积 PCD 已确定性抽样，Odin 驱动仍为外部依赖；详细边界见 [`sim2real_ros2_v2/README.md`](sim2real_ros2_v2/README.md)。

## 实机记录

[![第一代 Sim2Real 真机验证](../../06_assets/images/early_sim2real_preview.jpg)](../../06_assets/videos/early_sim2real.mp4)

该视频记录了这一阶段的早期真机测试，用于对应本目录中的第一代控制与部署实现。
