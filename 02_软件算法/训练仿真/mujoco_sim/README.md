# MuJoCo 独立工具集

本目录保存不依赖训练循环的 MuJoCo、姿态、IK、动力学和 MPC 分析工具。脚本通过父目录读取 `../mjcf/scene.xml` 与 `../mjcf/wheelleg.xml`，因此应从 `rc_mjlab` 工程根目录运行。

## 工具分类

| 入口 | 用途 |
| --- | --- |
| `posture_tool.py` | 基于解析运动学快速生成站立和低姿态参数表 |
| `rl_friendly_opt.py` | 按轮心位置、关节力矩和雅可比条件数筛选适合 RL 的姿态 |
| `posture_optimizer.py` | 解析计算与 MuJoCo 扫描结合的姿态优化 |
| `static_posture_optimizer.py` | 在重力和地面接触下评估静态站姿、支撑域和离地间隙 |
| `ik_diff_sweep.py` | 扫描 IK 姿态和差速轮跟踪参数，可导出 JSON |
| `run.py` | 启动完整 MuJoCo 控制、GUI 和 MPC 调试链路 |
| `robot.py`、`controller.py` | 仿真机器人接口和控制器 |
| `dynamics.py`、`mpc.py`、`mpc_controller.py` | Pinocchio 动力学与 OSQP MPC |

## 依赖

执行工程根目录的 `uv sync` 后，训练环境已经提供 NumPy、SciPy 和 MuJoCo。不同工具还需要：

- 纯解析工具：Python、NumPy。
- MuJoCo 扫描：`mujoco`、NumPy。
- 完整 MPC：`pinocchio`、`osqp`、SciPy。
- GUI：系统可用的 Tk/Tkinter。

Pinocchio 和 OSQP 没有加入训练环境锁文件，因为它们只服务于可选 MPC 工具，且 Pinocchio 的安装方式与操作系统、Conda/Python 环境有关。

## 常用命令

在 `05_software/train/rc_mjlab` 下执行：

```bash
# 不启动 MuJoCo 的快速姿态表
uv run python mujoco_sim/posture_tool.py
uv run python mujoco_sim/rl_friendly_opt.py

# 姿态扫描
uv run python mujoco_sim/posture_optimizer.py --analyze
uv run python mujoco_sim/static_posture_optimizer.py --quick

# IK 与差速轮参数快速扫描
uv run python mujoco_sim/ik_diff_sweep.py --quick

# 完整 GUI/MPC 仿真，需要可选依赖
uv run python mujoco_sim/run.py
```

## 参数边界

这是一份历史工具快照，保留当时用于分析和调参的常量：

- `config.py`、`posture_optimizer.py` 和 `static_posture_optimizer.py` 中的解析质量常量为 `12.3 kg`。
- 当前新版 MJCF 的惯性质量合计约为 `18.0377 kg`。
- `config.py` 的姿态表默认值为髋俯仰 `0.666`、膝关节 `-1.546`；比赛训练架构的 Rough 默认姿态为 `0.550/-1.125`。

MuJoCo 直接加载模型的工具会使用 MJCF 内的质量和惯性；显式读取 `ROBOT_MASS` 的解析计算和 MPC 工具仍使用历史常量。使用输出作为新版本控制参数前，应先根据目标机械状态完成质量、惯性和默认姿态复核。
