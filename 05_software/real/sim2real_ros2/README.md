# ROS 2/C++ Sim2Real 初版

本目录归档 `real/sim2real_ros2`，对应重排主线的 `v0.10.0`。这是轮腿机器人 Sim2Real 部署栈从 Python 运行时迁移到 ROS 2 + C++ 的第一版系统工程。

本工程保留当前 `sim2real` 已验证的部署契约，同时将运行时热路径迁移到 C++：

- `53D` 策略观测契约不变
- `16D` 动作契约不变
- `50Hz` 策略循环与训练对齐
- `200Hz` 电机循环为专用 C++ 热路径
- ROS 2 作为导航、TF、诊断和启动管理的系统集成层

## 工作区布局

- `src/sim2real_interfaces`
  硬件桥接与策略运行时共享的 ROS 2 消息定义。
- `src/sim2real_common`
  共享常量、部署契约辅助函数、Mahony 姿态滤波器、站立平衡控制器、安全监控。
- `src/sim2real_hw`
  面向硬件的桥接节点：RobStride CAN 收发、IMU/Odin 数据采集、看门狗、状态发布。
- `src/sim2real_runtime`
  策略运行时节点：`53D→16D` ONNX 推理、命令滤波/仲裁、目标发布。
  同时包含 `odom_relay_node`（里程计中继与 TF 广播）。
- `src/sim2real_nav2`
  ROS 2 Navigation2 (Nav2) 配置包：参数、启动文件、AMCL、costmap、planner/controller。
- `src/sim2real_bringup`
  统一启动文件与运行时参数配置。
- `src/odin_ros_driver`
  Odin 传感器 ROS 2 驱动（含 IMU、点云、里程计发布）。
- `docs`
  架构说明与迁移计划。

## 目标架构

```text
Odin / IMU / Odom  --->  sim2real_hw  --->  sim2real_runtime  --->  sim2real_hw
                              |                  |                    |
                              v                  v                    v
                         RuntimeState       RuntimeTarget          电机 CAN 指令
                              |                  |
                              +-------> 诊断 / 遥测

Nav2 / cmd_vel  ------------------------------>  sim2real_runtime
                   (经 odom_relay_node 提供 odom→base_link TF)
```

## 当前状态

已完成 Phase 0-5 的全部迁移：

1. ✅ 冻结部署契约（deployment_contract.hpp）
2. ✅ ROS 2 包结构搭建
3. ✅ 硬件热路径迁移至 C++（SocketCAN 驱动、200Hz 电机循环）
4. ✅ ONNX 策略运行时迁移至 C++（50Hz 推理循环）
5. ✅ 导航与诊断通过 ROS 2 接入（Nav2 + odom_relay + TF）

## 契约来源

迁移过程中以下文件被视为真值源：

- `sim2real/deployment_manifest.yaml`
- `sim2real/interface/motor_mapping.py`
- `sim2real/interface/real_io.py`
- `sim2real/policy/policy_runner.py`
- `sim2real/web/session.py`

## 注意事项

- 开发目标为 Linux + ROS 2 Humble，运行于 Jetson Orin / x86_64。
- Windows 仅作为编辑环境使用。
- 观测顺序、动作缩放、默认站姿、电机映射不得独立修改，
  除非训练与部署同步更新。
- 原始快照中的 `src/odin_ros_driver` 是空目录，本版本仍需要另行提供兼容的 Odin ROS 2 驱动；其源码从后续版本开始随工程归档。
- 自研 ROS 包保留原始 `Proprietary` 清单字段，公开发布前仍需统一许可证和维护者信息。
