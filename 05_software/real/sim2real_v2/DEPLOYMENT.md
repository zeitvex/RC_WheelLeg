# 真机部署与验证指南

本文档面向当前目录 `shiji/sim2real (10)/sim2real`，用于减少上机排错时间。默认不修改已验证的电机映射、方向、零位和策略观测。

## 1. 前期环境

推荐在 Orin / Linux 上运行：

```bash
cd sim2real
python -m pip install -r requirements-orin.txt
```

确认设备：

```bash
ip link show can0
ip link show can1
ls -l /dev/ttyACM0
```

确认 `config.yaml`：

- `can1_port` 和 `can2_port` 对应实际 SocketCAN 设备。
- `control_freq` 默认保持 `50`。
- `remote.port` 默认 `/dev/ttyACM0`。
- `policy.action_scale` 必须是 16 维。
- 默认策略文件优先使用 `policies/model_rough.onnx`；如果只有 `.pt`，先用 `tools/export_onnx.py` 导出。
- `controller.kp_leg/kd_leg/kd_wheel` 不要在未记录实验的情况下大改。

## 2. 无硬件/低风险检查

先做文件和模型契约检查：

```bash
python tools/export_onnx.py --pt policies/model_rough.pt --onnx policies/model_rough.onnx
python tools/alignment_check.py --policy policies/model_rough.onnx --manifest deployment_manifest.yaml
python tools/standalone_check.py
python -m py_compile tools/logger.py interface/motor_driver.py interface/imu_client.py interface/real_io.py web/session.py
```

`alignment_check.py` 会同时检查：

- 策略 obs/action 维度。
- action scale。
- default pose。
- joint order。
- wheel indices。
- `config.yaml` 与 `deployment_manifest.yaml` 的控制频率和 command filter 是否一致。
- ONNX 策略的 obs/action 维度是否仍为 `53D/16D`。

如果有 Node 环境，可检查前端语法：

```bash
node --check web/static/app.js
```

## 3. 上电前检查

上电前确认：

- 机器人架空或有可靠支撑。
- 16 个电机 CAN 线和电源线固定。
- Odin1 连接稳定，启动时机器人尽量静止，利于重力对齐。
- Web 急停可见，遥控软急停通道可用。
- CAN 设备名和遥控串口名与 `config.yaml` 一致。

## 4. Web 启动

```bash
python web/server.py --host 0.0.0.0 --port 8080
```

浏览器打开：

```text
http://<orin-ip>:8080
```

推荐先只看状态，不急着释放遥控。

## 5. 标准上机流程

1. 点击 `Connect`，确认 IMU/Odin 和 CAN 初始化正常。
2. 点击 `Enable Motors`，确认 16 个电机都有反馈。
3. 点击 `Startup`，从当前实测姿态过渡到站立。
4. 进入 `STAND_HOLD` 后观察 IMU age、Motor Fresh、Loop Profile。
5. 点击 `Start Runtime`，进入 50Hz 策略循环。
6. 策略 release 后再点击遥控接管。
7. 小幅给命令，先测试前后、转向，再测试组合动作。

## 6. 本轮新增诊断如何看

`Loop Profile`：

- `read_state_ms` 高：优先查电机接收、CAN 队列、Odin 获取是否阻塞。
- `policy_ms` 高：优先查策略推理和 CPU 负载。
- `send_actions_ms` 高：优先查 CAN 发送和 USB-CAN 适配器。
- `log_ms` 高：说明日志队列或磁盘仍可能有压力。

`Motor Fresh`：

- 理想状态是 `16/16 (cnt 16, val 0)` 或接近。
- `cnt` 高说明 `update_count` 正常增长，这是最可靠的电机反馈证据。
- 如果机器人静止时 `val` 为 0 是正常现象，不应据此判断丢电机。

`Odin Odom`：

- 显示 `STANDARD/HIGHFREQ/TF`、age、local x/y/yaw。
- 当前只用于诊断和全局坐标显示，不参与策略输入。
- odom 不可用时，当前 locomotion 仍应可以运行。
- 如果显示 `JUMP`，说明 odom 局部位置或 yaw 出现突变，先不要把它用于闭环导航。

`Latest Target`：

- 显示当前电机目标来源，例如 `runtime_policy`、`runtime_zero_hold`、`runtime_release_hold`。
- `age` 应随 runtime 正常刷新；如果明显超过控制周期很多，说明目标更新链路卡住。
- `d` 是相邻目标最大变化量，可用于观察停车/起步是否有目标突变。

`Obs / Action`：

- `obs` 接近 `100` 时，说明观测可能接近 clip 边界。
- `raw` 接近 `10` 时，说明策略输出可能接近 raw action 裁剪边界。
- `scaled` 长期很大时，检查 action scale、目标限幅和 safety clip。

`cmd/raw cmd`：

- `raw cmd` 是 Web/遥控原始输入。
- `cmd` 是经过 `command_filter` 限加速度后的策略命令。
- 如果机器人响应慢，先看两者差值是否由命令滤波造成。

## 7. 如果出现前后晃动

先不要直接改控制频率。按顺序排查：

1. 看 `Loop Overruns` 是否增长。
2. 看 `Loop Profile` 最慢阶段。
3. 看 `imu_age` 是否超过 30-60ms。
4. 看 `Motor Fresh` 是否掉到 16 以下。
5. 看停止时 `cmd` 是否真的回到 0。
6. 看 `runtime_released`、`release_alpha` 和 `track_err` 是否异常。

只有确认 50Hz 长期跑不稳时，才把 `control_freq: 40` 作为诊断实验，而不是默认方案。训练/部署频率不一致可能引入新的 sim2real gap。

## 8. 日志

每次运行会生成：

- `state.csv`：高频状态流，后台线程写入。
- `events.jsonl`：事件流，关键事件会即时 flush。

重点搜索：

```bash
grep LOOP_OVERRUN web/logs/*/events.jsonl
grep SAFETY web/logs/*/events.jsonl
grep GUARD web/logs/*/events.jsonl
```

## 9. 当前不建议改动的内容

- 16 个电机映射、方向、零位。
- 策略 53D 观测顺序和缩放。
- 16D 动作顺序和 action scale。
- 默认 `50Hz` 控制频率。
- 已经真机跑通过的遥控方向配置。

这些内容只有在有新日志和明确现象时再改，避免把已验证链路打散。
