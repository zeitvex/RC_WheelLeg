# `sim2real` 部署说明

## 模型

当前只使用：

- `sim2real/policies/model_rough.pt`

## 模型契约

- `obs_dim = 53`
- `action_dim = 16`
- 单帧输入
- 无 `base_lin_vel`
- 无 `height_scan`

## 启动流程

1. 连接硬件
2. 使能电机
3. 从当前实测姿态起立
4. 进入 `stand_balance` 闭环站立
5. prime 当前观测
6. 进入 `50Hz` runtime

## 为什么这样改

- 之前版本在 `startup` 后只维持固定 `STAND_POSE`
- 实机上纯 PD 不足以持续抗姿态扰动
- 现在增加独立站立闭环，先保证身体支撑，再进入策略

## 运行开关

- `config.yaml > policy.enable_zero_cmd_suppression`
- `config.yaml > stand_balance.enabled`
- `config.yaml > policy.hold_zero_command_pose`
- `config.yaml > policy.command_release_s`

## 纯 Python 命令

默认前提：当前目录就是 `sim2real/`

```bash
python -m pip install -r requirements-orin.txt
python tools/alignment_check.py --policy policies/model_rough.pt --manifest deployment_manifest.yaml
python tools/standalone_check.py
python main.py --dry-run
python main.py
python web/server.py --host 0.0.0.0 --port 8080
```
