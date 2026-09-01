# `Orin Nano` 部署说明

## 是否必须转 ONNX

不必须。

当前优先级仍然是：

1. 先保证观测、动作、站立控制对齐
2. 再测 `50Hz` 实际环路稳定性
3. 最后才决定是否转 `ONNX/TensorRT`

## 当前代码重点

- `stand_balance` 已加入 `main.py` 和 `web/session.py`
- 启动后先站稳，再允许策略接管
- `PolicyRunner.step()` 仍保留零命令抑制开关，默认开启

## Orin 上先测什么

- 机器人能否在不启动策略时，仅靠 `startup + stand_balance` 稳定站住
- `loop_dt_ms`
- `imu_age_ms`
- 电机 stale
- policy forward 耗时

## 纯 Python 部署命令

默认前提：当前目录就是 `sim2real/`

```bash
python3 -m pip install -r requirements-orin.txt
python3 tools/alignment_check.py --policy policies/model_rough.pt --manifest deployment_manifest.yaml
python3 tools/standalone_check.py
python3 main.py --dry-run
python3 main.py
python3 web/server.py --host 0.0.0.0 --port 8080
```

## 首轮实机建议

1. 先不启动策略
2. 只验证 `startup -> stand_balance`
3. 站稳后再启动策略
4. 只给很小的 `vx / vy / yaw`
