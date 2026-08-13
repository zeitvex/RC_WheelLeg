# 第一代强化学习与仿真工程

`rc_mjlab/` 是 16DOF 轮足机器人的第一代自包含训练与仿真工程。

## 内容

- `src/robot`：Flat、Rough、Crawl 训练任务和自定义 MDP
- `mjcf`：轮足机器人 MuJoCo 模型和网格
- `mujoco_sim`：不依赖策略的独立 MuJoCo/MPC 调试工具
- `sim2sim`：策略加载、交互控制和比赛地形验证
- `mjlab`：固定版本的本地训练框架依赖
- `model_rough.pt`、`model_crawl.pt`：对应的早期策略权重
- `pyproject.toml`、`uv.lock`：Python 环境与依赖锁定

工程命令和任务说明见 [`rc_mjlab/README.md`](rc_mjlab/README.md)，本地依赖来源见 [`rc_mjlab/DEPENDENCIES.md`](rc_mjlab/DEPENDENCIES.md)。
