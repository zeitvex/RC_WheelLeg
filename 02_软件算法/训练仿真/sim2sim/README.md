# 后期 Sim2Sim 工具

本目录保存比赛训练架构之后形成的 MuJoCo 策略验证工具。`v0.8.0` 在早期 Sim2Sim 基础上增加 ONNX 策略加载、IK 参数扫描、纯 IK 绕桩验证和批量路线检查；训练任务与 MJCF 不在本阶段修改。

## 主要入口

- `nav_sim2sim.py`：Pygame 面板与 MuJoCo 多任务导航，Rough 策略优先加载根目录的 `model_6800.onnx`。
- `sim2sim.py`：较轻量的键盘控制与策略回放入口，优先加载 `model_6800.onnx`，缺失时回退到早期 `model_rough.pt`。
- `ik_slalom_sim2sim.py`：不依赖 RL 策略的 IK、差速轮、路径跟踪和绕桩测试。
- `ik_compensation_sweep.py`：批量扫描 IK 补偿参数并输出排序结果。
- `nav_route_sim2sim_check.py`：使用 ONNX 策略批量检查内置任务或外部航点路线。
- `export_onnx.py`：将兼容的 PyTorch actor checkpoint 导出并核对为 ONNX。
- `interface/mujoco_io.py`：MuJoCo 模型、传感器和执行器接口。
- `policy/policy_runner.py`：PT/ONNX 策略加载与历史观测缓存。

## 环境

主训练环境继续由根目录的 `uv.lock` 管理。后期 Sim2Sim 新增的 Pygame 与 ONNX Runtime 单独记录在 `sim2sim/requirements.txt`，运行时叠加，避免重新解析时改变已归档的 MuJoCo nightly 版本：

```powershell
uv run --with-requirements .\sim2sim\requirements.txt python .\sim2sim\nav_sim2sim.py
```

训练工程提供 MuJoCo、NumPy、PyTorch、Matplotlib 和 `pynput`；专用 requirements 显式补充 Pygame 与 ONNX Runtime。下面其他命令同样使用 `--with-requirements .\sim2sim\requirements.txt`。

## 常用命令

```powershell
# 比赛 Rough 策略交互回放
uv run --with-requirements .\sim2sim\requirements.txt python .\sim2sim\nav_sim2sim.py

# 轻量策略回放
uv run --with-requirements .\sim2sim\requirements.txt python .\sim2sim\sim2sim.py

# 纯 IK 绕桩验证
uv run --with-requirements .\sim2sim\requirements.txt python .\sim2sim\ik_slalom_sim2sim.py --test slalom

# IK 补偿参数扫描
uv run --with-requirements .\sim2sim\requirements.txt python .\sim2sim\ik_compensation_sweep.py --top 12

# 使用内置绕桩任务做批量 Sim2Sim 路线检查
uv run --with-requirements .\sim2sim\requirements.txt python .\sim2sim\nav_route_sim2sim_check.py `
  --terrain-xml .\sim2sim\terrain\scene_terrain.xml `
  --mission slalom `
  --onnx .\model_6800.onnx

# 导出早期参考 PT 权重；也可用 --pt-path 指定其他 checkpoint
uv run --with-requirements .\sim2sim\requirements.txt python .\sim2sim\export_onnx.py
```

## 模型与边界

- `../model_6800.onnx` 是 `last_not_slalom_1050` 最终真机工程使用的比赛 Rough 策略，SHA-256 为 `3C994BDD3434AD15770A52AC0E8D229F502F00D6511CDD42C2E2C742301AEF13`。
- `../model_rough.pt` 是较早阶段的参考 checkpoint，两者不是同一版本的权重。
- Crawl 模型未在本阶段归档；需要 Crawl 策略的入口会查找 `model_crawl.onnx` 或 `model_crawl.pt`。
- `nav_route_sim2sim_check.py` 依赖 `../tools/nav_tools/route_safety_check.py` 的航点和避障几何定义；默认使用 `points_20260715_120154.json` 与 `1hao.xml`。

运行时生成的日志、临时 XML、`route_check_runs/` 和批量实验输出不纳入版本库。人工打点形成的路线快照保存在 `../tools/nav_tools/points/`，大量重复仿真轨迹仍不复制。
