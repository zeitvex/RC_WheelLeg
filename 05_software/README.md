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
   └─ sim2real_ros2_v2/ # ROS 2 导航原型及后续演进
```

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

`rc_mjlab` 是自包含工程。训练、MJCF、MuJoCo、Sim2Sim、导航工具和策略权重通过相对路径绑定，因此保留其内部布局，没有为了目录外观拆散。第一代完整闭环见 `v0.3.0`，第一份新版 MJCF 与训练框架见 `v0.4.0`，随机化增强版见 `v0.5.0`，比赛最终训练架构见 `v0.6.0`，后期 MuJoCo 工具集见 `v0.7.0`，后期 Sim2Sim 与比赛 Rough 策略见 `v0.8.0`，完整导航打点工具见 `v0.8.1`，Python Sim2Real v2 对应 `v0.9.0`，ROS 2/C++ 初版对应 `v0.10.0`，简单导航与 PCD 导航原型对应 `v0.11.0`。

详细说明见：

- [`train/README.md`](train/README.md)
- [`real/README.md`](real/README.md)
- [`../01_doc/architecture/early_software_stack.md`](../01_doc/architecture/early_software_stack.md)
