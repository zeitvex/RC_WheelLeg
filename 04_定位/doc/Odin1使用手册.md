# Odin1 使用手册

本手册说明 Odin1 在轮足机器人中的实际接入方式，以及如何在当前工作区完成启动、扫图、保存地图和重定位。

## 1. Odin1 的职责

Odin1 通过 `odin_ros_driver` 向 ROS 发布：

- IMU、RGB 图像、点云和里程计；
- 机器人当前姿态与坐标变换；
- SLAM 建图结果和已有地图重定位结果。

在本项目中，Odin1 的 IMU 为 Sim2Real 控制提供角速度和投影重力，地图重定位为导航提供全局 `map` 坐标系，点云地图则用于规划比赛路线。Odin1 提供感知与定位输入，不直接生成关节控制量。

## 2. 工作区结构

```text
<workspace>/
├── runros.sh
├── build.sh
├── start_mapping.sh
├── start_relocalization.sh
├── save_map.sh
└── src/
    └── odin_ros_driver/
        ├── config/
        ├── lib/
        ├── launch_ROS1/
        ├── launch_ROS2/
        ├── map/
        └── set_param.sh
```

当前仓库中的 `<workspace>` 为：

```text
<repo>/odin_open/04_定位/doc
```

## 3. 构建和启动

当前工作区面向 Linux/ROS 2 使用，推荐 ROS 2 Humble。先安装 ROS 2、Odin1 驱动依赖和 USB 权限规则，然后执行：

```bash
cd <workspace>
chmod +x *.sh src/odin_ros_driver/*.sh src/odin_ros_driver/script/*.sh
./build.sh
source install/setup.bash
```

直接启动默认配置：

```bash
./runros.sh
```

只加载环境并进入交互 Shell：

```bash
./runros.sh --shell
```

默认配置文件为 `src/odin_ros_driver/config/control_command.yaml`。扫图和重定位优先使用下面的专用脚本。

## 4. 工作模式

配置项 `custom_map_mode` 的含义：

```text
0 -> 里程计模式
1 -> SLAM 建图模式
2 -> 重定位模式
```

### 4.1 里程计模式

里程计模式只使用当前运动估计，不加载已有地图，适合检查设备连接、IMU 和基础里程计输出。

### 4.2 建图模式

使用建图配置启动：

```bash
./start_mapping.sh
```

该脚本使用 `src/odin_ros_driver/config/control_command_mapping.yaml`，其中 `custom_map_mode` 已设置为 `1`。启动后缓慢移动 Odin1，覆盖需要使用的区域，并在 RViz 中确认点云和轨迹正常。

完成扫图后，在另一个终端保存地图：

```bash
./save_map.sh
```

地图默认保存到：

```text
<workspace>/src/odin_ros_driver/map/
```

生成的 `.bin` 文件用于后续重定位；导出的 `.pcd` 点云可用于地图查看和路线打点。

### 4.3 重定位模式

直接指定地图启动：

```bash
./start_relocalization.sh /absolute/path/to/map.bin
```

脚本会生成一次性的运行配置，并将 `custom_map_mode` 设置为 `2`，不会修改模板配置。地图路径必须是存在的绝对路径。

如果需要手动设置初始位姿，可编辑：

```text
src/odin_ros_driver/config/control_command_relocal.yaml
```

配置示例：

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: "/absolute/path/to/map.bin"
  custom_init_pos: [x, y, z, qx, qy, qz, qw]
```

`custom_init_pos` 的前三项是位置，后四项是四元数姿态。若知道机器人在地图中的大致起点，填写接近实际位置和朝向的初值有助于提高启动速度。

重定位成功后，系统会发布：

```text
map -> odom
```

规划和导航模块据此获得统一的全局坐标。

## 5. 常用话题

```text
/odin1/imu
/odin1/image
/odin1/cloud_raw
/odin1/cloud_slam
/odin1/odometry
/tf
```

检查话题和频率：

```bash
ros2 topic list
ros2 topic hz /odin1/imu
ros2 topic echo /odin1/odometry --once
```

## 6. 使用注意

- 建图和重定位时，确认 Odin1 USB 连接、供电和设备固件正常；
- 地图路径使用绝对路径，地图文件应与建图场地保持一致；
- 重定位初始位置尽量靠近建图轨迹，初始朝向不要偏差过大；
- 启动前确认没有其他 `host_sdk_sample` 进程占用设备；
- ROS 2 工作区必须先完成构建，再执行 `source install/setup.bash`；
- 设备库已提供 x86 和 ARM 两个平台版本，编译时由 CMake 根据架构选择。

驱动原始说明和完整重定位参考位于：

- `src/odin_ros_driver/README.md`
- `src/odin_ros_driver/RELOCALIZATION_GUIDE.md`
