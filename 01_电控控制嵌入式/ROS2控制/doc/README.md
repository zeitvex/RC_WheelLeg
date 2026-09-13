# ROS2 控制

这是当前 16DOF 轮足机器人唯一推荐的 ROS 2 实机运行目录。Odin1 位于系统上游，提供 IMU、里程计、点云和重定位结果；本目录将这些输入接入导航、策略、命令仲裁和电机执行链路。

## 运行链路

```text
Odin1
  -> src/odin_ros_driver
  -> /odin1/imu、/odin1/odometry、/tf、/odin1/cloud_slam
  -> 导航 / 比赛路线
  -> sim2real_runtime
  -> sim2real_hw
  -> SocketCAN
  -> 16 个电机
```

## 目录

- `src/odin_ros_driver/`：Odin1 ROS 2 驱动与配置；
- `src/sim2real_bringup/`：统一启动文件和运行参数；
- `src/sim2real_runtime/`：53 维观测、策略推理、命令仲裁和安全逻辑；
- `src/sim2real_hw/`：关节状态读取、目标下发、CAN 通信和硬件保护；
- `src/sim2real_common/`：关节顺序、动作缩放、姿态滤波和部署契约；
- `src/sim2real_nav2/`：导航参数和任务配置；
- `policies/`：运行时策略模型；
- `map/`：运行时点云地图和比赛路线；
- `screen/`：车载屏幕控制程序；
- `runros.sh`：加载环境，默认只启动 Odin1 驱动；
- `start_sim2real.sh`：启动完整 Sim2Real 控制栈。

路线编辑工具统一放在 `../../../05_规划/打点工具/`，定位和扫图工具统一放在 `../../../04_定位/doc/`。本目录中的 `map/` 和 `policies/` 是运行时资产，不要与其他目录重复复制。

注意：`src/` 内的 Python 文件是 ROS 2 节点或 ROS 2 辅助工具，必须随 ROS 2 工作区一起构建和启动；独立 Python 控制实现请使用 `../../Python控制/`，不要混用两套入口。

## 启动入口

先阅读：

- `部署与运行说明.md`
- `系统架构说明.md`
- `调试与安全说明.md`

需要单独启动 Odin1 扫图或重定位时使用 `runros.sh`；需要启动机器人控制、策略和硬件桥接时使用 `start_sim2real.sh`。Python 版本不在本目录中，统一位于 `../../Python控制/`。
