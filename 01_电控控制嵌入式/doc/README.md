# 电控、控制与嵌入式

这一部分负责把 Odin1 的传感器、里程计和重定位结果接入轮足机器人，完成 Sim2Real、硬件桥接、嵌入式参考和比赛实机运行。

## Odin1 在控制链路中的位置

```text
Odin1
  -> odin_ros_driver
  -> IMU / odometry / point cloud / map->odom
  -> 导航与比赛任务
  -> 命令仲裁与策略运行
  -> 硬件桥接
  -> CAN 电机
```

Odin1 不直接生成关节控制量，而是为控制系统提供上游状态与环境输入：

- `/odin1/imu`：进入策略观测、站立平衡和独立安全检查；
- `/odin1/odometry`：经中继后作为 `/odom`，供控制与导航使用；
- `/tf`：重定位成功后提供 `map -> odom`，统一比赛路线坐标；
- `/odin1/cloud_slam`：用于扫图、地图保存和重定位相关处理。

## 目录结构

```text
01_电控控制嵌入式/
├── doc/                  # 本分类入口和总体说明
├── ROS2控制/             # ROS 2 主线、Odin1 驱动和最终比赛运行系统
│   ├── src/              # ROS 2 功能包
│   ├── policies/         # ROS 2 运行时策略
│   ├── map/              # ROS 2 运行时地图和路线
│   ├── screen/           # 屏幕
│   └── doc/              # ROS 2 部署、架构和调试说明
├── Python控制/           # Python 控制版本集中目录
│   ├── 早期真机控制/     # IK、轨迹插值和接口验证
│   └── 最终Python部署/   # 独立 Python Sim2Real 部署方案
└── 嵌入式/               # STM32/FreeRTOS/CAN 工程
    └── Dji_A/
```

## 推荐使用顺序

1. 需要扫图或重定位时，先阅读 `04_定位/doc/`。
2. 需要编辑点云路线时，使用 `05_规划/打点工具/`。
3. 需要启动最终 ROS 2 控制系统时，只进入 `ROS2控制/`。
4. 需要使用 Python 版本时，进入 `Python控制/` 下对应版本。
5. 需要阅读 STM32/FreeRTOS 控制参考时，进入 `嵌入式/Dji_A/`。

`ROS2控制/` 中的 `src/`、地图、策略、脚本和配置必须作为一个整体使用，不要再拆到其他分类后重新拼接。Python 版本与 ROS 2 版本的依赖和启动入口彼此独立。

说明：`ROS2控制/src/` 中出现的 `.py` 文件是 ROS 2 节点或 ROS 2 工具，属于 ROS 2 工作区，不能脱离对应的 ROS 2 包单独运行。独立的 Python 控制版本统一放在 `Python控制/` 中。

说明：`ROS2控制/src/` 中出现的 `.py` 文件是 ROS 2 节点或 ROS 2 工具，属于 ROS 2 工作区，不能脱离对应的 ROS 2 包单独运行。独立的 Python 控制版本统一放在 `Python控制/` 中。

## 参考

电控、CAN、Sim2Real 和嵌入式工程的组织方式参考：

[Dichen33/RC_Legged_Control](https://github.com/Dichen33/RC_Legged_Control)

本项目仅借鉴其“最终运行主线、历史验证资料、嵌入式参考”三层组织方式，实际传感器主线仍以 Odin1 为核心。
