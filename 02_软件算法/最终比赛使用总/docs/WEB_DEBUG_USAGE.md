# sim2real_ros2 Web UDP 调试说明

本文档说明本次新增的最小 Web 调试链路。

## 1. 架构

```text
Windows 本地浏览器/HTTP 服务
        |
        | UDP JSON
        v
Nano: sim2real_web_udp_bridge_node.py
        |
        | ROS 2 topics
        v
sim2real_cmd_mux_node.py -> /cmd_vel -> sim2real_runtime_node
```

Web 页面在 Windows 本地渲染，Nano 只运行轻量 UDP bridge 和 ROS2 节点。

## 2. 新增 ROS2 节点

### `remote_uart_node.py`

遥控器节点现在发布：

```text
/cmd_vel_remote
```

不再直接发布 `/cmd_vel`。

通道触发阈值改为：

```yaml
remote_axis_deadzone: 40
remote_active_threshold: 40
```

只有通道归一化值绝对值大于 `40` 才认为是有效输入。

### `cmd_mux_node.py`

输入：

```text
/cmd_vel_remote
/cmd_vel_web
/cmd_vel_nav
/control/mode
/remote/enabled
/web/enabled
/nav/enabled
/safety/estop
```

输出：

```text
/cmd_vel
/control/mode_state
/control/mux_status
```

控制模式：

```text
DISABLED
REMOTE
WEB
NAV
```

急停 `/safety/estop=true` 会强制进入 `DISABLED`，并输出零速度。

### `web_udp_bridge_node.py`

Nano 端 UDP 监听：

```text
0.0.0.0:15000
```

发布：

```text
/cmd_vel_web
/safety/estop
/control/mode
/web/enabled
/remote/enabled
/nav/enabled
```

订阅并回传状态：

```text
/runtime/state
/runtime/target
/cmd_vel
/safety/estop
/control/mode_state
/control/mux_status
```

## 3. Nano 启动

```bash
cd /path/to/sim2real_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sim2real_bringup sim2real_system.launch.py launch_nav2:=false
```

默认会启动：

```text
sim2real_remote_uart_node
sim2real_cmd_mux_node
sim2real_web_udp_bridge_node
```

如果不想启动 Web UDP bridge：

```bash
ros2 launch sim2real_bringup sim2real_system.launch.py launch_nav2:=false launch_web_bridge:=false
```

## 4. Windows 本地 Web 启动

把目录复制到 Windows 或通过共享目录访问：

```text
tools/win_web_debug
```

在 Windows 上安装 Python 3 后运行：

```bash
python server.py --nano-host <Nano_IP> --http-port 8088 --udp-port 15001
```

浏览器打开：

```text
http://127.0.0.1:8088
```

## 5. UDP 命令格式

### 切换模式

```json
{"type":"mode","mode":"REMOTE"}
```

```json
{"type":"mode","mode":"WEB"}
```

```json
{"type":"mode","mode":"DISABLED"}
```

### Web 速度控制

```json
{
  "type": "cmd_vel",
  "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
}
```

Nano 端会再次限幅：

```text
vx  <= ±0.8 m/s
vy  <= ±0.3 m/s
yaw <= ±0.5 rad/s
```

### 零速度

```json
{"type":"zero"}
```

### 软急停

```json
{"type":"estop","data":true}
```

## 6. 安全保护

当前最小版本已经包含：

1. 遥控器误触发阈值：`40`。
2. 遥控器/Web/Nav 互斥控制模式。
3. `cmd_mux` 二次限幅。
4. `cmd_mux` 加速度限制。
5. Web UDP 超时自动发布零速度。
6. 急停优先级最高。
7. Web 页面切换到 `WEB` 模式需要确认。
8. Web 松开虚拟摇杆会自动发送零速度。

建议实机调试流程：

1. 先点击 `DISABLED`。
2. 确认 `/cmd_vel` 为零。
3. 如果使用遥控器，点击 `REMOTE`。
4. 如果使用 Web，点击 `WEB` 并确认周围安全。
5. 一旦异常，立即点击 `软急停`。

## 7. 验证命令

查看最终输出速度：

```bash
ros2 topic echo /cmd_vel
```

查看遥控器输入：

```bash
ros2 topic echo /cmd_vel_remote
```

查看 Web 输入：

```bash
ros2 topic echo /cmd_vel_web
```

查看当前仲裁模式：

```bash
ros2 topic echo /control/mode_state
```

查看策略状态：

```bash
ros2 topic echo /runtime/target --field target_source
```
