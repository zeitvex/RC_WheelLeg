# 软件算法

这一部分保留强化学习训练与 `sim2sim` 验证。Odin1 是真实机器人侧的传感器与定位来源，训练算法和仿真验证属于 Odin1 的下游应用。

## Odin1 与算法链路

```text
Odin1 IMU / map->odom / 路点
  -> 真机观测与导航指令
  -> 53 维策略观测、vx / vy / wz
  -> 强化学习策略推理
  -> 关节目标与轮速目标
```

在 Sim2Real 中，Odin1 的 IMU 经姿态滤波后提供角速度和投影重力，直接进入策略观测；Odin1 的全局定位结果由导航模块转换为目标速度，间接进入策略；定位和 IMU 状态还用于策略切换与安全保护。

## 目录

- `训练仿真/`：MJCF/MuJoCo 模型、强化学习训练任务、策略权重和 `sim2sim` 工具。
- `训练仿真/sim2sim/`：策略回放、路线验证、IK 验证和 ONNX 导出。
- `训练仿真/doc/`：训练环境、依赖和整体说明。

真机策略运行、Odin1 IMU 接入、电机通信和比赛任务集成统一放在：

`01_电控控制嵌入式/比赛任务集成/`

## 参考

强化学习训练和仿真组织方式参考：

[wusi321/RC_Legged_Training_Simulation](https://github.com/wusi321/RC_Legged_Training_Simulation)

参考仓库用于说明训练任务、PPO、MuJoCo 和 Sim2Sim 的组织方式；本项目中 Odin1 负责把训练后的策略接入真实环境所需的 IMU、定位和导航输入。
