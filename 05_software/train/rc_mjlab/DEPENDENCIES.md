# 依赖说明

## Python 环境

- Python `>=3.10,<3.14`
- `uv` 依赖管理
- MuJoCo `3.8` 系列
- `mjlab[cu128]`
- PyTorch CUDA 12.8 环境
- `pynput`

精确解析结果保存在 `uv.lock`。项目使用本地可编辑的 `mjlab`：

```toml
[tool.uv.sources]
mjlab = { path = "mjlab", editable = true }
```

## mjlab 来源

- 上游仓库：`https://github.com/mujocolab/mjlab.git`
- 基准提交：`40f8d93e31b589dccae78ba6aadfc4b74cd1e3fd`
- 基准日期：`2026-06-02`
- 上游许可证：Apache-2.0，许可证文件保留在 `mjlab/LICENSE`

本版本在该基准上保留 1 处本地修改：

1. `mjlab/src/mjlab/envs/mdp/dr/actuator.py`：为分组执行器补充名称到运行时执行器对象的解析，使 PD 增益和力矩限制随机化能够正确作用于轮腿机器人的执行器组。

本次归档保留修改后的完整工作树，但不包含上游 `.git`、本地缓存、生成日志和运行时临时文件。

## 基本入口

在 `05_software/train/rc_mjlab` 下执行：

```bash
uv sync
uv run train Robot-Flat-v0
uv run play Robot-Rough-v0
```

GPU、CUDA、MuJoCo development wheel 和驱动版本必须满足 `pyproject.toml` 与 `uv.lock` 的约束。
