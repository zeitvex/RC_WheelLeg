# Python Sim2Real v2

本目录归档 `real/sim2real_v2` 版本，对应 Python 真机部署从第一代闭环继续演进后的版本节点。该阶段保持 `53D -> 16D` 策略契约，重点完善电机反馈判断、IMU/Odin 可观测性、Web 调试和运行时诊断；后续 ROS 2/C++ 版本另行归档。

## 当前控制链路

- 策略模型：默认优先使用 `policies/model_rough.onnx`，`.pt` 仅作为导出源和 fallback
- 策略输入：单帧 `53D`
- 策略输出：`16D`
- 控制频率：默认 `50Hz`
- 电机数量：16 个 RS02/RobStride 电机
- 电机映射、方向、零位：以 `interface/motor_mapping.py` 为准，当前视为真机验证过，不在本轮改造中修改
- Web 入口：`python web/server.py --host 0.0.0.0 --port 8080`

策略观测顺序保持不变：

1. `base_ang_vel * 0.25`
2. `projected_gravity`
3. `command`
4. `joint_pos[:12] - default_dof_pos[:12]`
5. `joint_vel[:12] * 0.05`
6. `wheel_vel[12:] * 0.05`
7. `last_actions`

Odin odom 现在只作为诊断和全局状态显示接入，不进入策略观测，避免破坏已训练模型的输入契约。

## 本轮改造内容

- 日志写入改为后台线程，降低 `state.csv` 高频写入对 50Hz 控制循环的阻塞。
- 电机 fresh/stale 判断优先使用驱动层 `update_count`，避免静止电机反馈正常但位置速度不变时被误判为丢电机。
- `wait_feedback_ready()` 同步识别 `update_count`，降低使能后零位静止电机被误判 missing 的概率。
- `IMUClient` 增加 `get_latest_odom()`，读取 Odin odom 的位置、姿态、线速度、角速度、类型和 age。
- `RealIO.read_state()` 增加只读 `odom` 字段，但不改变策略观测。
- 增加 `OdomTracker`，把 Odin raw odom 转成启动点局部坐标 `local x/y/yaw`，并做跳变检测。
- 增加 `LatestTarget` 诊断，记录当前下发目标来源、age、目标变化量，为后续双 loop 解耦铺路。
- Web runtime 状态增加 `loop_profile`、odom、电机 fresh 计数、每电机 update_count。
- Web Diagnostics 面板增加 loop overrun、最慢阶段、电机 fresh 来源、Odin odom 状态。
- 增加 `command_filter`，对 Web/遥控命令统一做限加速度平滑，降低起停冲击；Web 同时显示 raw cmd 和 filtered cmd。
- 策略推理入口改为 ONNXRuntime 优先，默认查找 `policies/model_rough.onnx`；不存在时兼容回退到 `policies/model_rough.pt`。
- ONNXRuntime 使用 CPU 单线程顺序执行，减少推理线程池和 motor/status/logger 线程抢占。

## 重点诊断字段

Web 中重点看这些项：

- `loop_dt`：控制循环总耗时，50Hz 下目标约 `20ms`。
- `Loop Overruns`：runtime 超时累计次数和最大超时。
- `Loop Profile`：显示本轮最慢阶段，例如 `read_state_ms`、`policy_ms`、`send_actions_ms`。
- `Latest Target`：显示目标来源、age 和本次目标最大变化量；age 异常增大说明目标更新链路卡住。
- `Obs / Action`：显示观测绝对值最大值、raw action 最大值和 scaled action 最大值，用来发现输入爆炸或动作饱和。
- `Motor Fresh`：格式为 `fresh/16 (cnt x, val y)`；`cnt` 表示通过 `update_count` 确认的新反馈数量。
- `Odin Odom`：显示 odom 类型、age、local x/y/yaw；没有 odom 时不影响控制。
- `cmd/raw cmd`：`cmd` 是进入策略的滤波后命令，`raw cmd` 是 Web/遥控原始命令。
- 电机列表状态点：绿色代表 stale count 为 0，黄色代表短时未刷新，红色代表连续 stale。

## 当前仍需实机重点确认

- 如果继续出现 `LOOP_OVERRUN`，先看 Web 的 `Loop Profile`，不要直接调低控制频率。
- 如果 `read_state_ms` 慢，重点排查 SocketCAN/CAN 队列、Odin bridge 或电机反馈处理。
- 如果 `policy_ms` 慢，重点排查 ONNXRuntime 推理耗时、CPU 占用和是否有后台进程抢占。
- 如果 `send_actions_ms` 慢，重点排查 CAN 发送阻塞或 USB-CAN 适配器。
- 如果 `Obs / Action` 中 obs 接近 `100` 或 raw action 接近 `10`，说明策略输入/输出可能在饱和边界，需要优先检查观测缩放、IMU、关节速度和命令。
- 如果 `Motor Fresh` 不是 16/16，但 `update_counts` 在增长，需要检查 Web stale 阈值而不是电机丢失。
- 如果 odom age 长时间不更新，只影响全局坐标/诊断，不应影响当前策略控制。

## 启动命令

在 Orin / Linux 真机上：

```bash
python -m pip install -r requirements-orin.txt
python tools/export_onnx.py --pt policies/model_rough.pt --onnx policies/model_rough.onnx
python tools/alignment_check.py --policy policies/model_rough.onnx --manifest deployment_manifest.yaml
python tools/standalone_check.py
python web/server.py --host 0.0.0.0 --port 8080
```

Web 流程：

1. Connect
2. Enable Motors
3. Startup
4. Start Runtime
5. 策略 release 后，如需要，点击遥控接管
6. 实时观察 Diagnostics、Motors、Plots

## 安全边界

当前保留原有保护链路：

- Web 急停
- 遥控软急停
- runtime guard
- safety monitor
- NaN/Inf 检查
- 电机 stale holdover
- damping brake

这轮没有放宽安全边界，也没有调整电机限位、零位、方向、默认增益和策略动作缩放。
