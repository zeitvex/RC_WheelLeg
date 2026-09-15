# 软件

本目录保存 16DOF 轮足机器人的训练、仿真和真机软件演进。

```text
05_software/
├─ train/
│  └─ rc_mjlab/       # 训练、MJCF、MuJoCo、Sim2Sim 和本地 mjlab 依赖
└─ real/
   ├─ ik_real/        # IK 轨迹与早期真机控制
   ├─ sim2real/       # 第一代 Python 策略真机部署
   ├─ sim2real_v2/    # Python Sim2Real v2
   ├─ sim2real_ros2/  # ROS 2/C++ Sim2Real 初版
   ├─ sim2real_ros2_v2/ # ROS 2 导航原型及里程计演进
   └─ sim2real_ros2_v3/ # 最终比赛 ROS 2/C++ 部署
```

当前工作树按架构大版本同时保留三个 ROS 2 目录：无后缀目录是初版，`_v2` 是第二版演进的最终里程计快照，`_v3` 是 `last_not_slalom_1050` 最终比赛部署。各目录内部的小阶段仍可通过对应 Tag 恢复。

## 数据流

```text
MJCF + mjlab task
        |
        v
   PPO 训练策略
        |
        +----> MuJoCo 姿态 / IK / MPC 调试
        |
        +----> Sim2Sim 策略验证
        |
        +----> Python Sim2Real / v2 ----> 电机 / IMU
        |
        +----> ROS 2/C++ Sim2Real -----> CAN / IMU / 导航

IK real --------------------------------> 电机
```

`rc_mjlab` 是自包含工程。训练、MJCF、MuJoCo、Sim2Sim、导航工具和策略权重通过相对路径绑定，因此保留其内部布局，没有为了目录外观拆散。第一代完整闭环见 `v0.3.0`，第一份新版 MJCF 与训练框架见 `v0.4.0`，随机化增强版见 `v0.5.0`，比赛最终训练架构见 `v0.6.0`，后期 MuJoCo 工具集见 `v0.7.0`，后期 Sim2Sim 与比赛 Rough 策略见 `v0.8.0`，完整导航打点工具见 `v0.8.1`，Python Sim2Real v2 对应 `v0.9.0`，ROS 2/C++ 初版对应 `v0.10.0`，简单导航原型对应 `v0.11.0`，完整 Odin/TensorRT 与站姿调参对应 `v0.11.1`，纯里程计导航联调对应 `v0.12.0`，1050 分比赛最终部署对应 `v1.0.0`。

详细说明见：

- [`train/README.md`](train/README.md)
- [`real/README.md`](real/README.md)
- [`../01_doc/architecture/early_software_stack.md`](../01_doc/architecture/early_software_stack.md)
