# rc_mjlab

基于 [mjlab](https://github.com/mujocolab/mjlab) 的 16DOF 串联轮足机器人强化学习、MuJoCo 验证与策略部署工程。

## 版本定位

当前目录是多个里程碑累积后的工作树，不应整体写成“对应 `v0.8.1`”：

| Tag | 本目录中的主要变化 |
| --- | --- |
| `v0.4.0` | 第一份新 MJCF 与新 mjlab 训练基线（`uni_mjlab(1)`） |
| `v0.5.0` | 随机化增强训练版本（`uni_mjlab_new`） |
| `v0.6.0` | `best` 比赛训练架构：分轴速度奖励、自适应指令课程和障碍释放课程 |
| `v0.7.0` | 补充后期 `mujoco_sim` 姿态、IK、动力学和 MPC 工具 |
| `v0.8.0` | 补充后期 Sim2Sim、路线检查和比赛 Rough 策略 `model_6800.onnx` |
| `v0.8.1` | 补充导航打点、路线迭代与抽样 PCD 工具 |

后续 ROS 2 真机版本没有把本目录重新定义为新的训练版本。要查看某个阶段的真实代码，请切换对应 Tag；当前训练主体以 `v0.6.0` 的 `best` 架构为基础，工具链累计到 `v0.8.1`。

## 目录说明

```text
rc_mjlab/
├─ src/robot/           # Robot-Flat/Rough/Crawl 任务、PPO 配置和本地 RSL-RL/HIM 代码
├─ mjcf/                # 机器人、场景与网格资源
├─ mjlab/               # 固定基准并带本地补丁的 mjlab 源码
├─ mujoco_sim/          # 姿态、IK、动力学、MPC 与 GUI 工具
├─ sim2sim/             # ONNX/PT 回放、比赛场景、IK 与路线检查
├─ tools/nav_tools/     # PCD、地图、航点和路线编辑工具
├─ model_rough.pt       # 早期 Rough 参考 checkpoint
├─ model_6800.onnx      # 比赛最终部署使用的 Rough 策略
├─ pyproject.toml       # Python 包和依赖声明
├─ uv.lock              # 历史环境的精确锁文件
└─ DEPENDENCIES.md      # 上游基准、本地补丁和可选依赖说明
```

`model_6800.onnx` 是最终部署工件，不等于训练代码版本号。训练过程中存在基模、继续训练和 checkpoint 筛选，仅凭该 ONNX 不能恢复完整训练日志。

## 已注册任务

| Task ID | 用途 | 默认训练时长 |
| --- | --- | --- |
| `Robot-Flat-v0` | 平地基础运动 | 20 s/episode |
| `Robot-Rough-v0` | 粗糙地形、台阶、随机网格、高墙和坡面 | 20 s/episode |
| `Robot-Crawl-v0` | 低杆、低姿态和匍匐任务 | 30 s/episode |

任务入口由 [`src/robot/__init__.py`](src/robot/__init__.py) 注册；环境真值见 [`src/robot/config/env_cfgs.py`](src/robot/config/env_cfgs.py)，PPO 真值见 [`src/robot/config/rl_cfg.py`](src/robot/config/rl_cfg.py)。

## 当前控制与模型参数

以下参数来自当前工作树源码，不代表所有历史 Tag：

| 项目 | 当前值 |
| --- | --- |
| MuJoCo 物理步长 | `0.005 s`（200 Hz） |
| 控制降采样 | `decimation = 4` |
| 策略周期 | `0.020 s`（50 Hz） |
| 默认并行环境 | 2048 |
| 腿部执行器 | 位置控制，`Kp=50.0`、`Kd=1.5`、力矩上限 `17 Nm` |
| 轮部执行器 | 速度控制，`Kd=1.0`、力矩上限 `17 Nm` |
| 关节速度参考常量 | `13 rad/s`；当前 Builtin actuator 构造未显式传入该常量 |
| 默认站姿 | hip pitch `0.550`、knee `-1.125`、机身高度 `0.42 m` |
| 外展关节动作缩放 | `0.125 rad` |
| 其余腿关节动作缩放 | `0.25 rad` |
| 轮速动作缩放 | `5.0 rad/s` |
| 动作延迟 | 每个环境随机 `0～2` 个控制步 |
| 低通截止频率 | 腿 `5 Hz`、轮 `15 Hz` |

README 原先写的 `0.002 s × decimation 10`、4096 环境、`Kp=40/Kd=1` 和轮部 `Kd=0.5` 均不对应当前代码，已删除。

## 观测与动作契约

Actor 单步观测为 53 维：

| 观测项 | 维度 |
| --- | ---: |
| 基座角速度 | 3 |
| 投影重力 | 3 |
| 速度/航向指令 | 3 |
| 12 个腿关节相对位置 | 12 |
| 12 个腿关节速度 | 12 |
| 4 个轮关节速度 | 4 |
| 上一步 16 维动作 | 16 |

动作共 16 维：12 个腿关节位置目标和 4 个轮关节速度目标。Critic 在 Actor 观测之外增加基座线速度、轮地接触和高度扫描等特权信息。

## Rough 当前配置摘要

`Robot-Rough-v0` 当前混合八类地形：

| 地形 | 比例 | 当前范围摘要 |
| --- | ---: | --- |
| 平地 | 15% | 8 m × 8 m |
| 正向台阶 | 5% | 阶高 `0～0.20 m` |
| 反向台阶 | 35% | 阶高 `0～0.20 m` |
| 随机网格 | 27% | 高度 `0～0.20 m` |
| 随机粗糙面 | 1% | 起伏 `0～0.06 m` |
| Perlin 噪声 | 1% | 起伏 `0～0.06 m` |
| 自定义高墙 | 15% | 高度 `0.10～0.35 m` |
| 金字塔坡面 | 1% | 坡度 `0.052～0.325` |

课程学习从平地、粗糙面、Perlin、坡面和正向台阶开始，随后按训练步数释放随机网格、反向台阶和高墙。同时对 X、Y、Yaw 三个指令轴分别做自适应范围调整。

当前 Rough 奖励使用分轴 `vx/vy/yaw` 跟踪，并启用每步总奖励不低于 0 的截断，而不是旧 README 中的 `track_lin_vel=4.5`、`track_ang_vel=2.0`。它还包含动作变化率、扭矩/功率、腿轮加速度、关节限位、镜像姿态、静止姿态、接触力和非期望碰撞等约束；精确权重以 `rough_env_cfg()` 为准。

## 当前域随机化边界

基础配置实际启用的主要随机项包括：

- 基座质心三轴偏移：各 `[-0.05, 0.05] m`；
- 碰撞几何摩擦：`[0.3, 1.0]`；
- 执行器刚度、阻尼缩放：各 `[0.9, 1.1]`，log-uniform；
- 基座附加质量：`[-1.0, 3.0] kg`；
- Rough 间歇推扰：每 `5～10 s` 设置一次 X/Y `[-0.5, 0.5] m/s` 速度扰动；
- 关节动作延迟：`0～2` 个策略步。

旧 README 中列出的 encoder bias、持续外力、关节摩擦和力矩上限随机化并非当前 Rough 配置的完整真实状态，因此不再作为“当前已启用项”陈述。历史随机化差异见 [`../../../01_doc/training_evolution.md`](../../../01_doc/training_evolution.md)。

## 环境安装与基本命令

在本目录执行：

```bash
uv sync

uv run train Robot-Flat-v0
uv run train Robot-Rough-v0
uv run train Robot-Crawl-v0

uv run play Robot-Rough-v0
```

GPU、CUDA、MuJoCo development wheel 和驱动要求见 [`DEPENDENCIES.md`](DEPENDENCIES.md)。恢复训练时需要明确 checkpoint/run 来源，不建议仅凭 README 猜测跨实验热启动参数。

## Sim2Sim 与导航工具

```bash
uv run --with-requirements sim2sim/requirements.txt python sim2sim/nav_sim2sim.py

uv run --with-requirements tools/nav_tools/requirements.txt python tools/nav_tools/nav_map_viewer.py
```

- 后期 Sim2Sim 入口和策略边界：[`sim2sim/README.md`](sim2sim/README.md)
- 导航打点与路线数据：[`tools/nav_tools/README.md`](tools/nav_tools/README.md)
- 训练版本演进：[`../../../01_doc/training_evolution.md`](../../../01_doc/training_evolution.md)
- 全项目版本历史：[`../../../01_doc/version_history.md`](../../../01_doc/version_history.md)

## 复现边界

- `uv.lock` 保存依赖解析结果，但仍需要匹配的 NVIDIA 驱动和 CUDA 环境。
- TensorRT engine 属于真机部署环境，本目录以训练代码、PT/ONNX 和仿真验证为主。
- 比赛最终真机工程位于 [`../../real/sim2real_ros2_v3`](../../real/sim2real_ros2_v3)。
- 参数若与本文冲突，以当前 Tag 中的配置源码为准；不同 Tag 之间不要直接混用奖励权重、站姿和模型。
