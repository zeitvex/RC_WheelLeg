# sim2real_ros2 遥控器调用说明

本文档说明如何在 `sim2real_ros2` 中调用已接入的 SBUS UART 遥控器节点，以及执行后系统会产生什么效果。

## 1. 当前接入关系

遥控器节点位于：

```text
src/sim2real_runtime/src/remote_uart_node.py
```

该节点读取 SBUS 串口数据，并发布标准 ROS 2 控制话题：

| 输入 | 输出 | 作用 |
|---|---|---|
| SBUS UART 遥控器 | `/cmd_vel` | 给策略运行时发送速度命令 |
| SBUS CH7 高位 | `/safety/estop` | 触发软件急停 |

策略节点 `sim2real_runtime_node` 已经订阅 `/cmd_vel` 和 `/safety/estop`，所以遥控器不直接控制电机，而是通过 ROS 2 标准速度接口进入策略控制链路。

## 2. 通道映射

通道映射与前一阶段 Python Sim2Real 中的遥控器实现保持一致。

| 遥控器通道 | ROS 2 输出 | 含义 | 默认最大值 |
|---|---|---|---:|
| `CH2` | `cmd_vel.linear.x` | 前后速度 `vx` | `0.8 m/s` |
| `CH4` | `cmd_vel.linear.y` | 左右速度 `vy` | `0.3 m/s` |
| `CH1` | `cmd_vel.angular.z` | 转向角速度 `yaw` | `0.5 rad/s` |
| `CH7 HIGH` | `/safety/estop = true` | 软件急停 | - |

默认方向反转配置：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `remote_invert_vx` | `true` | 反转前后方向 |
| `remote_invert_vy` | `false` | 不反转横移方向 |
| `remote_invert_yaw` | `true` | 反转转向方向 |

## 3. 参数位置

遥控器参数在：

```text
src/sim2real_bringup/config/runtime.yaml
```

当前默认参数：

```yaml
remote_enabled: true
remote_port: "/dev/ttyACM0"
remote_baudrate: 100000
remote_timeout: 0.02
remote_axis_deadzone: 50
remote_active_threshold: 50
remote_axis_full_scale: 660.0
remote_max_vx: 0.8
remote_max_vy: 0.3
remote_max_yaw_rate: 0.5
remote_invert_vx: true
remote_invert_vy: false
remote_invert_yaw: true
remote_publish_inactive_zero: true
remote_estop_latch: true
remote_poll_hz: 50.0
```

如果遥控器串口不是 `/dev/ttyACM0`，需要修改：

```yaml
remote_port: "/dev/ttyUSB0"
```

或改成实际设备路径。

## 4. 启动前检查

### 4.1 确认串口存在

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

如果使用默认配置，应能看到：

```bash
/dev/ttyACM0
```

### 4.2 确认串口权限

如果节点提示串口权限不足，可以临时执行：

```bash
sudo chmod 666 /dev/ttyACM0
```

更推荐的长期方式是把当前用户加入 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
```

然后重新登录。

### 4.3 确认 Python serial 依赖

节点依赖 `pyserial`。如果系统没有安装：

```bash
sudo apt update
sudo apt install -y python3-serial
```

## 5. 构建

如果刚修改过代码或参数，建议重新构建相关包：

```bash
cd /path/to/sim2real_ros2_v2
source /opt/ros/humble/setup.bash
colcon build --packages-select sim2real_runtime sim2real_bringup --symlink-install --merge-install
```

构建完成后 source 环境：

```bash
source install/setup.bash
```

确认可执行节点存在：

```bash
ros2 pkg executables sim2real_runtime
```

应包含：

```text
sim2real_runtime remote_uart_node.py
```

## 6. 推荐启动方式

### 6.1 启动完整系统，不启动 Nav2

这是你当前常用方式：

```bash
cd /path/to/sim2real_ros2_v2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sim2real_bringup sim2real_system.launch.py launch_nav2:=false
```

默认情况下，`launch_remote:=true`，所以上面命令会同时启动遥控器节点。

等价完整写法：

```bash
ros2 launch sim2real_bringup sim2real_system.launch.py launch_nav2:=false launch_remote:=true
```

### 6.2 不启动遥控器

如果只想用手动 `ros2 topic pub` 或其他上位机发 `/cmd_vel`，可以关闭遥控器节点：

```bash
ros2 launch sim2real_bringup sim2real_system.launch.py launch_nav2:=false launch_remote:=false
```

## 7. 单独启动遥控器节点

如果系统已经在运行，只想单独测试遥控器节点：

```bash
cd /path/to/sim2real_ros2_v2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run sim2real_runtime remote_uart_node.py --ros-args --params-file src/sim2real_bringup/config/runtime.yaml
```

如果要临时指定串口：

```bash
ros2 run sim2real_runtime remote_uart_node.py --ros-args \
  --params-file src/sim2real_bringup/config/runtime.yaml \
  -p remote_port:=/dev/ttyUSB0
```

## 8. 执行后会产生什么效果

启动以下命令后：

```bash
ros2 launch sim2real_bringup sim2real_system.launch.py launch_nav2:=false
```

系统会产生以下效果。

### 8.1 启动硬件桥接节点

节点：

```text
/sim2real_hw_node
```

效果：

1. 打开 `can0` 和 `can1`。
2. 如果 `dry_run: false` 且 CAN 初始化成功，会使能 16 个 RobStride 电机。
3. 设置电机 MIT 模式。
4. 设置电机速度限制和力矩限制。
5. 以 `200Hz` 运行硬件读写循环。
6. 发布 `/runtime/state`。
7. 订阅 `/runtime/target` 执行策略目标。

### 8.2 启动策略运行节点

节点：

```text
/sim2real_runtime_node
```

效果：

1. 加载 ONNX 策略模型。
2. 订阅 `/runtime/state`。
3. 订阅 `/cmd_vel`。
4. 订阅 `/safety/estop`。
5. 执行启动站立流程：
   - `boot_hold`
   - `startup_soft_hold`
   - `startup_hold`
   - `runtime_zero_hold`
   - `runtime_policy`
6. 以 `50Hz` 发布 `/runtime/target`。

### 8.3 启动遥控器节点

节点：

```text
/sim2real_remote_uart_node
```

效果：

1. 打开默认串口 `/dev/ttyACM0`。
2. 以 `50Hz` 轮询 SBUS 数据。
3. 遥控器摇杆居中时持续发布零速度：

```text
/cmd_vel:
  linear.x = 0.0
  linear.y = 0.0
  angular.z = 0.0
```

4. 推动遥控器时发布非零速度，例如：

```text
/cmd_vel:
  linear.x = vx
  linear.y = vy
  angular.z = yaw
```

5. 当 CH7 打到高位时发布：

```text
/safety/estop: true
```

由于当前 `remote_estop_latch: true`，急停是锁存式行为：一旦 CH7 高位触发，节点会发布急停，并保持内部急停已触发状态。恢复运行通常需要重启系统或手动发布复位信号，并确认机器人安全。

### 8.4 机器人行为效果

正常启动后，机器人不会立即按策略行走，而是按阶段执行：

1. 电机使能。
2. 读取当前关节位置。
3. 软保持当前姿态。
4. 平滑过渡到默认站立姿态。
5. 稳定后进入 runtime。
6. 遥控器无输入时保持站立平衡，即 `runtime_zero_hold`。
7. 遥控器有输入时进入策略控制，即 `runtime_policy`。

也就是说：

| 遥控器状态 | 机器人效果 |
|---|---|
| 摇杆居中 | 站立保持，不主动行走 |
| CH2 前后推动 | 前进/后退 |
| CH4 左右推动 | 横向移动 |
| CH1 左右推动 | 原地转向 |
| CH7 高位 | 软件急停，进入安全刹车 |

## 9. 如何确认遥控器已经生效

### 9.1 查看节点是否存在

```bash
ros2 node list
```

应看到：

```text
/sim2real_remote_uart_node
/sim2real_runtime_node
/sim2real_hw_node
```

### 9.2 查看 `/cmd_vel`

```bash
ros2 topic echo /cmd_vel
```

摇动遥控器时应看到 `linear.x`、`linear.y` 或 `angular.z` 变化。

### 9.3 查看 `/safety/estop`

```bash
ros2 topic echo /safety/estop
```

CH7 高位时应看到：

```yaml
data: true
```

### 9.4 查看策略目标阶段

```bash
ros2 topic echo /runtime/target --field target_source
```

常见输出含义：

| `target_source` | 含义 |
|---|---|
| `boot_hold` | 刚启动，保持初始姿态 |
| `startup_soft_hold` | 启动软保持 |
| `startup_hold` | 正在站立或站立后保持 |
| `runtime_zero_hold` | 已进入 runtime，遥控器无有效输入 |
| `runtime_policy` | 遥控器有输入，策略已经介入 |
| `safety_brake` | 安全刹车 |
| `timeout_hold` | 目标超时，硬件保持默认姿态 |

### 9.5 查看完整目标状态

```bash
ros2 topic echo --once /runtime/target
```

重点关注字段：

```yaml
target_source:
zero_command:
runtime_released:
release_alpha:
command:
raw_command:
```

如果遥控器摇杆有输入，通常会看到：

```yaml
target_source: runtime_policy
zero_command: false
runtime_released: true
release_alpha: 1.0
```

## 10. 常见问题

### 10.1 启动后提示无法打开串口

可能原因：

1. 串口路径不对。
2. 权限不足。
3. 设备没有插好。
4. 设备被其他程序占用。

检查：

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

修改 `runtime.yaml`：

```yaml
remote_port: "/dev/ttyUSB0"
```

### 10.2 `/cmd_vel` 没有变化

检查：

```bash
ros2 node list
ros2 topic echo /cmd_vel
```

如果节点存在但无变化，可能是：

1. 遥控器没有输出 SBUS。
2. 串口波特率不对。
3. SBUS 接线错误。
4. 遥控器通道未校准。
5. 死区 `remote_axis_deadzone` 或 `remote_active_threshold` 太大。

### 10.3 摇杆方向反了

修改：

```yaml
remote_invert_vx: true
remote_invert_vy: false
remote_invert_yaw: true
```

例如前后方向反了，就切换：

```yaml
remote_invert_vx: false
```

### 10.4 急停后不恢复

当前配置：

```yaml
remote_estop_latch: true
```

这表示急停锁存。触发后建议：

1. 先确认机器人物理安全。
2. 停止 launch。
3. 将 CH7 打回安全位置。
4. 重新启动系统。

如果需要非锁存模式，可以改为：

```yaml
remote_estop_latch: false
```

但实机调试时更建议使用锁存模式。

## 11. 快速验证命令清单

```bash
cd /path/to/sim2real_ros2_v2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch sim2real_bringup sim2real_system.launch.py launch_nav2:=false
```

另开终端：

```bash
cd /path/to/sim2real_ros2_v2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list
ros2 topic echo /cmd_vel
ros2 topic echo /runtime/target --field target_source
```

如果只测遥控器，不启动电机系统：

```bash
ros2 run sim2real_runtime remote_uart_node.py --ros-args --params-file src/sim2real_bringup/config/runtime.yaml
```

另开终端：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /safety/estop
```
