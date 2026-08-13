# 依赖说明

## Python 环境

- Python `>=3.10`
- `uv` 依赖管理
- MuJoCo development wheel
- `mjlab[cu128]`
- PyTorch CUDA 12.8 环境
- `pynput`

精确解析结果保存在 `uv.lock`。项目使用本地可编辑 `mjlab`：

```toml
[tool.uv.sources]
mjlab = { path = "mjlab", editable = true }
```

## mjlab 来源

- 上游仓库：`https://github.com/mujocolab/mjlab.git`
- 基准提交：`0040979763ab43bc1220812c9de4bc74e2631f42`
- 基准日期：`2026-04-28`
- 上游许可证：Apache-2.0，许可证文件保留在 `mjlab/LICENSE`

早期工程在该基准上保留了 3 处本地修改：

1. `mjlab/pyproject.toml`：增加清华 PyPI 镜像。
2. `mjlab/src/mjlab/envs/mdp/dr/actuator.py`：让 effort limit 随机化支持轮子使用的 velocity/motor actuator。
3. `mjlab/src/mjlab/scene/scene.py`：通过 XML 字符串加载场景，以适配当时的场景组合方式。

本次归档保留修改后的完整工作树，但不包含上游 `.git`、本地 `.venv`、缓存和生成日志。

## 基本入口

在 `05_software/train/rc_mjlab` 下执行：

```bash
uv sync
uv run train Robot-Flat-v0
uv run play Robot-Rough-v0
```

GPU、CUDA、MuJoCo development wheel 和驱动版本必须满足 `pyproject.toml` 与 `uv.lock` 的约束。
