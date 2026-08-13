# `sim2real`

当前版本只部署现在这套 `53D -> 16D` 模型，不再兼容旧版 `crawl`、多策略和历史观测。

## 当前部署模型

- 使用文件：`sim2real/policies/model_rough.pt`
- 来源文件：`model_2000.pt`

## 当前 actor 输入

- 单帧 `53D`
- 顺序：
  - `base_ang_vel * 0.25`
  - `projected_gravity`
  - `command`
  - `joint_pos_rel`（12）
  - `joint_vel_rel * 0.05`（12）
  - `wheel_vel * 0.05`（4）
  - `last_actions`（16）

不包含：

- `base_lin_vel`
- `height_scan`

## 当前控制参数

- 控制频率：`50Hz`
- 站立保持：`startup/stand_balance.py`
- `hip_abduction` 缩放：`0.125`
- 其他腿关节缩放：`0.25`
- 轮速缩放：`5.0`
- 腿 LPF：`5Hz`
- 轮 LPF：`15Hz`
- 零命令抑制：默认开启，可通过 `config.yaml > policy.enable_zero_cmd_suppression` 关闭
- 零命令保持：默认开启，可通过 `config.yaml > policy.hold_zero_command_pose` 控制
- 首次命令解锁：默认开启，可通过 `config.yaml > policy.require_active_command_to_release` 控制

## 当前站立逻辑

- `startup`：从实测姿态过渡到默认站姿
- `stand_balance`：根据 `IMU roll/pitch + gyro` 动态修正四条腿目标
- `runtime`：只有站立稳定后才进入策略控制

这次改动的重点是：策略不再承担“先把身体撑住”的职责。

另外当前部署逻辑改成：

- 零命令时默认不让策略直接接管腿和轮，保持站立目标
- 命令从零变为非零时，策略输出在 `command_release_s` 内平滑放开

## 启动命令

默认前提：当前目录就是 `sim2real/`

纯 `python`：

```bash
python -m pip install -r requirements-orin.txt
python tools/alignment_check.py --policy policies/model_rough.pt --manifest deployment_manifest.yaml
python tools/standalone_check.py
python main.py --dry-run
python main.py
python web/server.py --host 0.0.0.0 --port 8080
```

Windows 本机：

```bash
D:\Minicoda3\envs\py10\python.exe -m pip install -r requirements-orin.txt
D:\Minicoda3\envs\py10\python.exe tools\alignment_check.py --policy policies\model_rough.pt --manifest deployment_manifest.yaml
D:\Minicoda3\envs\py10\python.exe tools\standalone_check.py
D:\Minicoda3\envs\py10\python.exe main.py
D:\Minicoda3\envs\py10\python.exe web\server.py --host 0.0.0.0 --port 8080
```
#sim2real/policies/model_rough.pt