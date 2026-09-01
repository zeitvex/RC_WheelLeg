# Odin1 的使用手册

## 1. 这份手册做什么

这份手册只讲 Odin1 在当前项目里的实际用法：它负责什么、需要哪些文件、怎么启动、怎么建图、怎么重定位。

相关资料当前放在：

- `D:\git\odin\odin_open\04_定位\doc`

## 2. Odin1 在这里负责什么

Odin1 驱动 `odin_ros_driver` 主要做这些事：

- 连接传感器并发布点云、IMU、RGB、里程计和 TF
- 支持里程计、SLAM 建图、已有地图重定位
- 提供 RViz 可视化
- 提供地图保存和在线参数调节

## 3. 本项目里最相关的文件

- `README.md`
- `RELOCALIZATION_GUIDE.md`
- `config/control_command.yaml`
- `launch_ROS2/odin1_ros2.launch.py`
- `set_param.sh`
- `D:\git\odin\odin_open\04_定位\地图与重定位\runros.sh`
- `D:\git\odin\odin_open\04_定位\地图与重定位\doc\DEPLOYMENT_GUIDE.md`

## 4. 最小启动流程

```bash
source /opt/ros/<distro>/setup.bash
source <workspace>/install/setup.bash
D:\git\odin\odin_open\04_定位\地图与重定位\runros.sh
```

如果只想进入一个干净 shell：

```bash
D:\git\odin\odin_open\04_定位\地图与重定位\runros.sh --shell
```

## 5. 三种模式

`custom_map_mode` 决定 Odin1 的工作方式：

- `0`：里程计模式
- `1`：SLAM 建图模式
- `2`：重定位模式

### 里程计模式

适合只看实时位姿和基础输出，不做地图保存。

### 建图模式

把 `custom_map_mode` 设为 `1`，启动后移动设备采图，最后用 `set_param.sh` 保存地图。

### 重定位模式

把 `custom_map_mode` 设为 `2`，再指定已有地图路径。

## 6. 建图和保存地图

配置示例：

```yaml
register_keys:
  custom_map_mode: 1
```

启动后，缓慢移动 Odin1，覆盖需要的区域。完成后进入驱动目录保存地图：

```bash
cd <workspace>/src/odin_ros_driver
./set_param.sh save_map 1
```

生成的地图通常是 `.bin` 文件。

## 7. 重定位

配置示例：

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: "<workspace>/src/odin_ros_driver/map/xxx.bin"
  custom_init_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
```

如果你知道大概起点，可以把 `custom_init_pos` 换成实际位姿。

## 8. 常用话题

- `/odin1/imu`
- `/odin1/image`
- `/odin1/cloud_raw`
- `/odin1/cloud_slam`
- `/odin1/odometry`
- `/tf`

重定位成功后，`/tf` 里会出现 `map -> odom`。

## 9. 一句总结

- 建图：`custom_map_mode = 1`
- 重定位：`custom_map_mode = 2`
- 启动：优先用 `D:\git\odin\odin_open\04_定位\地图与重定位\runros.sh`
