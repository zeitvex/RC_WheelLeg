# 真机控制与部署

本目录保存 16DOF 轮足机器人从早期接口验证到最终比赛 ROS 2 部署的演进。

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

## `sim2real_ros2`

`last_not_slalom_1050` 最终比赛工程的规范化归档，包含：

- ROS 2 Humble + C++ 运行时
- 50 Hz 策略推理与 200 Hz CAN 电机热路径
- Rough `model_6800`、Wall `model_84` 和 Crawl IK 模式
- Odin IMU/里程计驱动、简单导航、命令仲裁和触控屏 UI
- 比赛路线、抽样 PCD、Docker 与部署说明

`1050` 是比赛得分，不是模型编号。完整入口与缺失的 Odin 重定位地图边界见 [`sim2real_ros2/README.md`](sim2real_ros2/README.md)。

## 实机记录

[![第一代 Sim2Real 真机验证](../../06_assets/images/early_sim2real_preview.jpg)](../../06_assets/videos/early_sim2real.mp4)

该视频记录了这一阶段的早期真机测试，用于对应本目录中的第一代控制与部署实现。
