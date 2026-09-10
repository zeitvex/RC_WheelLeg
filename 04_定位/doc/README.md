# Odin1 定位工作区

本目录同时保存 Odin1 的说明文档和定位运行文件，可以作为独立的 ROS 工作区使用。这里的驱动、配置和启动脚本服务于 Odin1 的里程计、扫图、地图保存和重定位功能。

## 目录

```text
04_定位/doc/
├── runros.sh
├── build.sh
├── start_mapping.sh
├── start_relocalization.sh
├── save_map.sh
├── src/
│   └── odin_ros_driver/
│       ├── config/
│       ├── lib/
│       ├── launch_ROS1/
│       ├── launch_ROS2/
│       ├── script/
│       ├── set_param.sh
│       └── map/
└── *.md
```

`src/odin_ros_driver/` 是从比赛运行工程复制的完整 Odin1 驱动包；本目录的启动脚本不会调用其他分类目录中的运行代码。

## 快速开始

在 Linux/ROS 2 主机上进入本目录：

```bash
cd <repo>/odin_open/04_定位/doc
chmod +x *.sh src/odin_ros_driver/*.sh src/odin_ros_driver/script/*.sh
./build.sh
```

### 扫图与保存地图

```bash
./start_mapping.sh
```

设备完成扫图后，在另一个终端执行：

```bash
./save_map.sh
```

地图默认保存到：

```text
src/odin_ros_driver/map/
```

### 加载地图并重定位

将已保存的 `.bin` 地图路径作为参数传入：

```bash
./start_relocalization.sh /absolute/path/to/your_map.bin
```

脚本会生成一次性的运行配置，并以 `custom_map_mode: 2` 启动 Odin1。若需要指定初始位姿，请修改
`src/odin_ros_driver/config/control_command_relocal.yaml` 中的 `custom_init_pos`。

### 直接启动

`runros.sh` 是通用入口：

```bash
./runros.sh
./runros.sh --shell
```

默认入口使用 `src/odin_ros_driver/config/control_command.yaml`。建图和重定位时建议使用上面的专用脚本，避免误用模式配置。

## Odin1 的输出

- `/odin1/imu`：姿态、角速度和策略控制所需的 IMU 数据；
- `/odin1/cloud_raw`、`/odin1/cloud_slam`：原始点云与建图/定位点云；
- `/odin1/odometry`：里程计；
- `/tf`：重定位成功后发布 `map -> odom`。

完整流程见：

- `Odin1使用手册.md`
- `Odin1扫图与重定位总览.md`
- `Odin1重定位指南.md`
- `src/odin_ros_driver/RELOCALIZATION_GUIDE.md`

驱动构建依赖 Odin1 SDK、ROS 2、OpenCV、PCL、yaml-cpp、libusb 等环境；目标设备上的厂商 SDK 二进制库需按驱动包 README 中的要求提供。
