# 强化学习与仿真工程

`rc_mjlab/` 保存 16DOF 轮足机器人的当前训练与 Sim2Sim 工程。历史快照由 Git Tag 保留，不在目录中复制 `old`、`new` 或 `final` 版本。

当前内容对应 `v0.4.0`，是第一份完整采用新版 MJCF 和新版 mjlab 框架的训练工程。

## 内容

- `src/robot`：Flat、Rough、Crawl 训练任务和自定义 MDP
- `mjcf`：轮足机器人 MuJoCo 模型和网格
- `sim2sim`：策略加载、交互控制和比赛地形验证
- `mjlab`：固定版本的本地训练框架依赖
- `model_rough.pt`：本阶段 Rough 策略权重
- `pyproject.toml`、`uv.lock`：Python 环境与依赖锁定

与 `v0.3.0` 相比，本版本更新了 MJCF 质量和惯性参数，并将 mjlab 上游基准从 `00409797` 更新到 `40f8d93e`。机械 CAD 未发生变化。

工程命令和任务说明见 [`rc_mjlab/README.md`](rc_mjlab/README.md)，本地依赖来源见 [`rc_mjlab/DEPENDENCIES.md`](rc_mjlab/DEPENDENCIES.md)。
