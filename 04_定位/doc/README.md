# Odin1 定位工作区

本目录是可独立运行的 Odin1 ROS 工作区，包含驱动源码、配置、启动脚本和运行说明，用于里程计、扫图、地图保存和重定位。

## 目录

```text
04_定位/doc/
├── runros.sh
├── build.sh
├── start_mapping.sh
├── start_relocalization.sh
├── save_map.sh
├── Odin1使用手册.md
└── src/odin_ros_driver/
```

其中 `src/odin_ros_driver/` 是 Odin1 驱动包，包含 ROS 1/ROS 2 启动文件、配置、源码和 x86/ARM 平台库。本目录的脚本均以当前目录为工作区，不依赖其他分类目录。

## 快速开始

在 Linux/ROS 2 主机上进入本目录：

```bash
cd <repo>/odin_open/04_定位/doc
chmod +x *.sh src/odin_ros_driver/*.sh src/odin_ros_driver/script/*.sh
./build.sh
```

启动 Odin1：

```bash
./runros.sh
```

扫图并保存地图：

```bash
./start_mapping.sh
./save_map.sh
```

加载地图并重定位：

```bash
./start_relocalization.sh /absolute/path/to/map.bin
```

完整的模式选择、参数配置、IMU 使用和故障排查见 [`Odin1使用手册.md`](Odin1使用手册.md)。

## 主要输出

- `/odin1/imu`：IMU 姿态、角速度和加速度数据；
- `/odin1/cloud_raw`、`/odin1/cloud_slam`：原始点云和建图/定位点云；
- `/odin1/odometry`：里程计；
- `/tf`：重定位成功后发布 `map -> odom`。

构建依赖 Odin1 SDK、ROS 2、OpenCV、PCL、yaml-cpp、Eigen3、OpenSSL 和 libusb。详细依赖及官方驱动说明见 `src/odin_ros_driver/README.md`。
