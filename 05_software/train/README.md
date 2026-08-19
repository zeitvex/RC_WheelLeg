# 强化学习与仿真工程

`rc_mjlab/` 保存 16DOF 轮足机器人的当前训练与 Sim2Sim 工程。历史快照由 Git Tag 保留，不在目录中复制 `old`、`new` 或 `final` 版本。

当前内容对应 `v0.6.0`，是比赛使用的最终训练代码架构。训练过程可能先获得基模，再调整奖励、课程和环境参数继续训练；模型 checkpoint 的变化不等同于软件架构变化。

## 内容

- `src/robot`：Flat、Rough、Crawl 训练任务和自定义 MDP
- `mjcf`：轮足机器人 MuJoCo 模型和网格
- `sim2sim`：策略加载、交互控制和比赛地形验证
- `mjlab`：固定版本的本地训练框架依赖
- `model_rough.pt`：本阶段 Rough 策略权重
- `pyproject.toml`、`uv.lock`：Python 环境与依赖锁定

与 `v0.3.0` 相比，本版本更新了 MJCF 质量和惯性参数，并将 mjlab 上游基准从 `00409797` 更新到 `40f8d93e`。机械 CAD 未发生变化。

与 `v0.4.0` 相比，本版本没有再次修改 MJCF 和训练框架，只调整训练环境配置：投影重力噪声从 `±0.05` 扩大到 `±0.08`，最大动作延迟从 2 步增加到 4 步，扩大摩擦、刚度和阻尼随机化，增加腿部质量随机化与连续外力/力矩扰动。

`v0.6.0` 在 `v0.5.0` 之后转向比赛任务优化：降低部分过强随机化，加入分轴速度跟踪奖励、自适应指令课程、障碍地形释放课程、楼梯横向/偏航约束以及更完整的训练诊断。详细对比见 [`../../01_doc/training_evolution.md`](../../01_doc/training_evolution.md)。

工程命令和任务说明见 [`rc_mjlab/README.md`](rc_mjlab/README.md)，本地依赖来源见 [`rc_mjlab/DEPENDENCIES.md`](rc_mjlab/DEPENDENCIES.md)。
