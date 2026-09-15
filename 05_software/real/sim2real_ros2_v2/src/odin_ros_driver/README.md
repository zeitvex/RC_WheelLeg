# Odin_ROS_Driver 说明文档

Odin 传感器模块 ROS 驱动套件（Manifold Tech Ltd.）

Odin1 Wiki：https://manifoldtechltd.github.io/wiki/Odin1/Cover.html

## Odin_ROS_Driver

兼容性：

● ROS 1（推荐 LTS 版本：Noetic）

● ROS 2（推荐 LTS 版本：Humble）

## 重要提示：

本驱动包提供点云 SLAM 应用的核心功能，面向特定使用场景。仅供专业技术人员进行二次开发使用。最终用户需根据实际部署环境进行场景优化和定制开发，以满足运行需求。

## 1. 版本

当前版本：v0.10.2

所需设备固件版本：v0.10.0

## 2. 准备工作

### 2.1 操作系统要求

● ROS Noetic 和 ROS2 Foxy 需 Ubuntu 20.04；

● ROS2 Humble 需 Ubuntu 22.04；

● 当前不支持 Ubuntu 18.04；

● Ubuntu 24.04 尚未官方支持，但可能经过一定修改后运行。

### 2.2 依赖项

● OpenCV >= 4.2.0（推荐 4.5.5/4.8.0，请确保仅安装一个 OpenCV 版本）

● yaml-cpp

● thread

● OpenSSL

● Eigen3

### 2.3 依赖安装

#### 2.3.1 系统基础
```shell
sudo apt update
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
```

#### 2.3.2 yaml-cpp
```shell
sudo apt update
sudo apt install -y libyaml-cpp-dev
```

#### 2.3.3 libusb
```shell
sudo apt update
sudo apt install -y libusb-1.0-0-dev
```

#### 2.3.4 OpenCV
```shell
sudo apt update
sudo apt-get install libopencv-dev
```

#### 2.3.5 ROS 安装

ROS Noetic 安装请参考：
[ROS Noetic 安装指南](https://wiki.ros.org/noetic/Installation)

ROS2 Foxy 安装请参考：
[ROS Foxy 安装指南](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

ROS2 Humble 安装请参考：
[ROS Humble 安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## 3. 准备

### 3.1 创建 Udev 规则
```shell
sudo vim /etc/udev/rules.d/99-odin-usb.rules
```
在 99-odin-usb.rules 文件中添加以下内容：
```shell
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
```
重新加载规则并重新插拔设备：
```shell
sudo udevadm control --reload
sudo udevadm trigger
```

### 3.2 克隆源码
```shell
git clone https://github.com/manifoldsdk/odin_ros_driver.git catkin_ws/src/odin_ros_driver
```
注意：
请将源码克隆到 "[ros_workspace]/src/" 目录下，否则会导致编译错误。

### 3.3 编译

#### 3.3.1 ROS1（以 Noetic 为例）：

```shell
source /opt/ros/noetic/setup.bash
./script/build_ros.sh
```

#### 3.3.2 ROS2（以 Foxy 为例）：

```shell
source /opt/ros/foxy/setup.bash
./script/build_ros2.sh
```

### 3.4 运行：

#### 3.4.1 ROS1（以 Noetic 为例）：

```shell
source [ros_workspace]/devel/setup.bash
roslaunch odin_ros_driver [launch file]
```
● odin_ros_driver：包名；

● launch file：启动文件名；

● ros_workspace：用户的 ROS 环境工作区；
```shell
roslaunch odin_ros_driver odin1_ros1.launch
```

#### 3.4.2 ROS2（以 Foxy 为例）：

```shell
source [ros2_workspace]/install/setup.bash
ros2 launch odin_ros_driver [launch file]
```
● odin_ros_driver：包名；

● launch file：启动文件名；

● ros2_workspace：用户的 ROS2 环境工作区；

ROS2 Demo 启动命令：
```shell
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

### 3.5 运行模式：

可通过 `config/control_command.yaml` 中的 `custom_map_mode` 参数配置运行模式。

#### 里程计模式

设置 `custom_map_mode = 0` 启用里程计模式。此模式下，map 坐标系与 odom 坐标系共享同一位姿。

若发现里程计数据漂移，可使用脚本命令 `./set_param.sh algo_reset 1` 动态复位算法。

#### SLAM 建图模式

设置 `custom_map_mode = 1` 启用 SLAM 模式。此模式在里程计模式基础上，提供**回环检测**和**地图保存**功能。

启动驱动后，odin1 将自动进行建图并缓存地图数据。场景采集完成后，需在驱动源码目录下执行 `./set_param.sh save_map 1` 以保存自程序启动以来采集的所有地图数据。地图将保存至 `config/control_command.yaml` 中 `mapping_result_dest_dir` 和 `mapping_result_file_name` 参数指定的路径。若未指定这些参数，将使用默认值。

首次保存后，可再次执行该命令保存新地图。每次保存操作都会生成一个新的地图文件。（连续保存操作之间请间隔至少 5 秒）

地图原点对应程序启动时 odom 坐标系的起点。

##### 重定位模式

要启用重定位，设置 `custom_map_mode = 2`，并通过 `config/control_command.yaml` 中的 `relocalization_map_abs_path` 参数指定预建地图的绝对路径。

启动后，odin1 将基于当前视点和指定地图启动重定位过程。为保障高成功率，建议在距 SLAM 轨迹原始位置 1 米、±10° 范围内启动。

注意，重定位性能高度依赖环境条件。在特征丰富的场景中，成功匹配可能发生在 1m/10° 范围之外，而其他环境可能需要更严格的条件。建议在实际部署环境中测试，以确定实际容忍范围。

若初始重定位失败，系统将临时以降级 SLAM 模式运行（此状态下地图保存功能禁用）。在此过程中可自由移动 odin1，它将在后台持续尝试重定位。一旦成功，将发布 map 与 odom 坐标系之间的 TF。（提示：初始化后轻轻晃动或移动设备有助于提高重定位准确率。）

以下话题在 odom 坐标系下发布：`/odin1/cloud_slam`、`/odin1/odom`、`/odin1/highodom` 和 `/odin1/path`。若需在 map 坐标系下获取这些数据，请应用从 odom 坐标系到 map 坐标系的 TF 变换。

## 4. 文件结构与数据格式
### 4.1 文件结构
```shell
Odin_ROS_Driver/                // ROS1/ROS2 驱动包
    3rdparty/                   // 第三方库
    src/
        host_sdk_sample.cpp     // 示例源码
        yaml_parser.cpp         // YAML 参数读取源码
        rawCloudRender.cpp      // RenderCloud 渲染源码
        depth_image_ros_node.cpp // depth_image_ros_node 节点
        depth_image_ros2_node.cpp // depth_image_ros2_node 节点
        pcd2depth_ros.cpp       // pcd2depth_ros 源码
        pcd2depth_ros2.cpp      // pcd2depth_ros2 源码
        pointcloud_depth_converter.cpp // pointcloud_depth_converter 源码
        cloud_reprojection_ros.cpp // 云重投影节点源码 (ROS1/ROS2)
        cloud_reprojector.cpp   // 云重投影核心逻辑
    lib/
        liblydHostApi_amd.a     // AMD 平台静态库
        liblydHostApi_arm.a     // ARM 平台静态库
    include/
        host_sdk_sample.h       // 示例头文件
        lidar_api_type.h        // API 数据结构头文件
        lidar_api.h             // API 函数声明
        yaml_parser.h           // 参数文件读取头文件
        rawCloudRender.h        // RenderCloud 相关 API
        data_logger.h           // 数据保存日志
        depth_image_ros_node.hpp // depth_image_ros_node 头文件
        depth_image_ros2_node.hpp // depth_image_ros2_node 头文件
        pointcloud_depth_converter.hpp // pointcloud_depth_convert 头文件
        cloud_reprojection_ros_node.hpp // cloud_reprojection_ros_node 头文件 (ROS1/ROS2)
        cloud_reprojector.hpp   // 云重投影核心类
    config/
        control_command.yaml    // 驱动控制参数文件
        calib.yaml              // 设备标定参数 yaml，每个设备独一无二。每次连接 ROS 驱动时从设备读取
    launch_ROS1/
        odin1_ros1.launch       // ROS1 启动文件
    launch_ROS2/
        odin1_ros2.launch.py    // ROS2 启动文件
    script/
        build_ros1.sh           // ROS1 安装脚本
        build_ros2.sh           // ROS2 安装脚本
    recorddata/                 // 存放可导入 MindCloud 的录制数据
    log/                        // 存放日志文件
        Driver_{timestamp}/     // 每次启动驱动时生成的日志文件夹
            Conn_{timestamp}/   // 每次 odin1 设备连接时生成的日志文件
                dev_status.csv  // 设备状态日志
    README.md                   // 使用说明
    CMakeLists.txt              // CMake 构建文件
    License                     // 许可证文件
```
### 4.2 启动文件
| 启动文件名               | 说明 |
|--------------------------|-------------|
| odin1_ros1.launch        | ROS1 启动文件 - Odin1 基础操作演示 |
| odin1_ros2.launch.py     | ROS2 启动文件 - Odin1 基础操作演示 |


### 4.3 ROS 话题
Odin ROS 驱动的内部参数定义在 config/control_command.yaml 中。以下是常用参数说明：

| 话题                      | control_command.yaml | 详细说明 |
|---------------------------|----------------------|----------------------|
| odin1/imu                     | sendimu           | IMU 话题 |
| odin1/image                   | sendrgb           | RGB 相机话题，由设备原始 JPEG 数据解码，bgr8 格式 |
| odin1/image_undistort         | sendrgbundistort  | 去畸变 RGB 相机话题，经设备 calib.yaml 标定参数处理 |
| odin1/image/compressed        | sendrgbcompressed | RGB 相机压缩话题，设备原始 JPEG 数据 |
| odin1/cloud_raw               | senddtof          | 原始点云话题 |
| odin1/cloud_render            | sendcloudrender   | 渲染点云话题，经原始点云、RGB 图像及设备 calib.yaml 处理 |
| odin1/cloud_slam              | sendcloudslam     | SLAM 点云话题 |
| odin1/odometry                | sendodom          | 里程计话题 |
| odin1/odometry_high           | sendodom          | 高频里程计话题 |
| odin1/path                    | showpath          | 里程计路径话题 |
| tf                            | sendodom          | TF 树话题 |
| odin1/depth_img_competetion   | senddepth         | 稠密深度图话题。需较高算力，仅作演示。与 odin1/image_undistort 一一对应。使用时请直接订阅本话题而非 echo。原始值即为深度数据，无需额外转换。 |
| odin1/depth_img_competetion_cloud  | senddepth         | 稠密深度点云话题。需较高算力，仅作演示 |
| odin1/reprojected_image       | sendreprojection  | 重投影像素话题。利用里程计将 cloud_slam 投影至相机图像。在主机端处理。 |

### 4.4 数据格式

1. 原始点云（cloud_raw）包含以下字段：
```
float32 x             // X 轴，单位：米
float32 y             // Y 轴，单位：米
float32 z             // Z 轴，单位：米
uint8  intensity      // 反射率，范围 0–255
uint16 confidence     // 点置信度，典型场景下取值范围约 0–1300，数值越高可靠性越强。推荐过滤阈值 30-35，应结合实际环境调整。
float32 offset_time   // 相对基准时间戳的时间偏移量，单位：秒
```

要在 PCL 中使用此自定义格式，首先定义点类型：
```cpp
/*** LS ***/
namespace ls_ros {
    struct EIGEN_ALIGN16 Point {
        float x;
        float y;
        float z;
        uint8_t intensity;
        uint16_t confidence;
        float offset_time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace ls_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(ls_ros::Point,
      (float, x, x)
      (float, y, y)
      (float, z, z)
      (uint8_t, intensity, intensity)
      (uint16_t, confidence, confidence)
      (float offset_time , offset_time)
)
```
然后即可轻松将 ROS sensor_msgs::PointCloud2 消息转换为 PCL 点云：
```
pcl::PointCloud<ls_ros::Point> ls_cloud;
pcl::fromROSMsg(*msg, ls_cloud);
```

2. SLAM 点云（cloud_slam）与直接渲染点云（cloud_render）包含以下字段：
```
float32 x             // X 轴，单位：米
float32 y             // Y 轴，单位：米
float32 z             // Z 轴，单位：米
float32 rgb           // RGB 颜色值
```

### 4.5 其他功能

| control_command.yaml 参数  | 详细说明 |
|----------------------------|----------------------|
| use_host_ros_time          | 时间同步模式：0 - 使用 odin 内部系统时间作为数据时间戳（典型用法，推荐）；1 - 接收时使用主机 ROS 时间（不推荐大多数用户使用）；2 - 通过类 NTP 同步将 odin1 时间对齐至主机时间，时间戳为传感器数据在主机时间轴上的接收时间。 |
| strict_usb3.0_check        | 严格 USB3.0 检查，关闭后即使 USB 连接低于 3.0 标准也允许连接 |
| recorddata                 | 以特定格式记录数据，可导入 MindCloud(TM) 进行后处理。请注意这将消耗大量存储空间，测试显示 10 分钟数据约占 9.5GB。 |
| devstatuslog               | 设备状态日志记录，当前将设备状态（SoC 温度、CPU 占用率、RAM 占用率、dToF 传感器温度等）及数据发送/接收速率保存至 log 目录下的 devstatus.csv。每次启动驱动时创建新文件。 |
| showcamerapose             | 显示相机位姿及视野范围。 |
| custom_map_mode             | 运行模式：模式 0 - 里程计模式：map 坐标系与 odom 坐标系共享同一位姿。模式 1 - 建图模式（带回环检测）：该模式支持地图保存。模式 2 - 重定位模式：需指定地图文件绝对路径，重定位成功后将输出 map 与 odom 坐标系之间的 TF 关系。|
| custom_init_pos             | 初始化位置（当前未启用）。 |
| relocalization_map_abs_path | 地图文件绝对路径：用于重定位模式。 |
| mapping_result_dest_dir 和 mapping_result_file_name | 建图模式下地图保存路径与文件名：若未指定，将使用默认值。 |

## 5. 常见问题
### 5.1 重新启动宿主 SDK 时出现段错误
**错误信息**
60 秒内未连接任何设备

**解决方案**
1. 请重新为 Odin 模块上电 # 断开并重新连接 odin 电源

2. 重新初始化 Odin SDK # 设备重启后执行 SDK


### 5.2 编译时出现库链接失败

**错误信息**
ld: cannot find -llydHostApi 或符号查找错误

**解决方案**

1. 清理之前的构建产物

ROS1
```shell
rm -rf devel/ build/
```
ROS2
```shell
rm -rf devel/ install/ log/
```
2. 重新运行脚本安装

### 5.3 Docker GUI 透传失败

**错误信息**
Unable to open X display 或 No protocol specified

**解决方案**
```shell
xhost + # 此命令启用 Docker 容器的图形透传
```

### 5.4 ROS 驱动以"获取版本失败"错误退出

**错误信息**
```shell
<ERROR><api.cpp:lidar_get_version:672>: get device version fail.
get version failed.
```

**解决方案**

设备固件版本过低，请升级至最新版本。


### 5.5 RVIZ 长时间无响应

**错误信息**
Rviz 无响应，稍后终端打印"Device disconnected, waiting for reconnection..."

**解决方案**

请重新为 Odin 模块上电

### 5.6 设备无响应

**错误信息**
Missed ok response from device, probably wrong interaction procedure.

**解决方案**

请采用 5.1 所述的解决方案

### 5.7 设备无外部标定文件

**错误信息**
ERROR：Missing camera node 'cam_0'

**解决方案**

请重新插拔 USB

### 5.8 ROS 驱动在数据流启动后立即报设备断开

**错误信息**

```shell
Device ready and streams activated
Device detaching...
Wating for device reconnection...
Device disconnected, waiting for reconnection...
```

**原因**

多见于 ROS2 环境且连接到复杂网络环境（如办公 WiFi 和以太网）的情况。ROS2 默认为广播模式，复杂网络环境可能导致 ROS2 发布阻塞，从而引发设备断开。

**解决方案**

若不需要跨设备通信，请将 ROS2 限制为仅本地通信：
```shell
export ROS_LOCALHOST_ONLY=1
```

若需要跨设备通信，请尽量简化网络环境。建议使用仅包含必要设备的小型局域网。

### 5.9 ROS 驱动在数据流启动后立即崩溃

**错误信息**

```shell
Device ready and streams activated
[host_sdk_sample-2] process has died ......
```

**测试**

在 control_command.yaml 中设置 sendrgb = 0 禁用 odin1/image，然后重试。若驱动此时正常工作，则问题很可能与系统安装了多个 OpenCV 版本有关。

**解决方案**

卸载多余的 OpenCV 版本，仅保留单一完整版本，然后重新编译驱动并重试。

### 5.10 ROS 驱动打印"TF_OLD_DATA ignoring data"警告

**错误信息**

```shell
[rviz2-3] Warning: TF_OLD_DATA ignoring data from the past for frame odin1_base_link at time 20.547632 according to authority Authority undetectable
[rviz2-3] Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained
[rviz2-3]          at line 294 in ./src/buffer_core.cpp
```

**原因**

这是 ROS 和 rviz 的一项功能，用于警告用户某些 TF 数据因时间戳冲突而被忽略。常见于用户保持 ROS 驱动运行的同时对 odin 设备断电重启，导致 odin 内部系统时间被重置，新数据时间戳与 rviz 上次运行期间接收的旧数据产生冲突。

**解决方案**

rviz GUI 底部有一个重置按钮。点击此按钮将重置 rviz 内部状态并停止警告。

### 5.11 ROS 驱动打印"unknown cmd code: xx"错误

**错误信息**

```shell
<ERROR><api.cpp:cmd_data_deal:418>: unknow command code 21.
```

**原因**

这是由于 ROS 驱动版本与设备固件版本不匹配，导致 ROS 驱动无法解码新版固件新增的数据。

**解决方案**

请确保使用最新版本的 ROS 驱动和设备固件。

### 5.12 USB 设备访问错误（LIBUSB_ERROR_BUSY 或 LIBUSB_ERROR_ACCESS）

**错误信息**

```shell
libusb: error [udev_hotplug_event] ignoring udev action bind
LIBUSB_ERROR_BUSY
```

或

```shell
libusb: error [_get_usbfs_fd] libusb couldn't open USB device /dev/bus/usb/xxx/xxx, errno=13
LIBUSB_ERROR_ACCESS
```

**原因**

- **LIBUSB_ERROR_BUSY**：另一个进程正在使用该 USB 设备。常见于多个 ROS 驱动实例正在运行，或其他应用程序（如之前崩溃的实例）仍持有设备句柄。

- **LIBUSB_ERROR_ACCESS**：当前用户无权访问 USB 设备。通常因缺少 udev 规则或用户权限不足导致。

**解决方案**

针对 **LIBUSB_ERROR_BUSY**：

1. 检查是否有其他驱动实例正在运行：
```shell
ps aux | grep host_sdk_sample
```

2. 终止所有现存实例：
```shell
killall host_sdk_sample
```

3. 若问题仍然存在，请拔插 USB 设备以重置设备状态。

针对 **LIBUSB_ERROR_ACCESS**：

1. 添加设备 udev 规则。创建文件 `/etc/udev/rules.d/99-odin.rules`，内容如下：
```shell
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
```

2. 重新加载 udev 规则：
```shell
sudo udevadm control --reload-rules
sudo udevadm trigger
```

3. 或者，使用 sudo 运行驱动（不推荐用于生产环境）：
```shell
sudo -E ros2 launch odin_ros_driver odin_ros_driver.launch.py
```

4. 确保当前用户属于 `plugdev` 用户组：
```shell
sudo usermod -aG plugdev $USER
```
然后注销并重新登录，使组变更生效。

## 6. 联系方式

您可通过 support@manifoldtech.cn 联系我们的技术支持。

为帮助诊断问题，请向我们的 FAE 工程师提供以下信息：

1. 当前固件版本
```shell
[device_version_capture]: ros_driver_version: [版本号]
```
2. 正在使用的电源适配器和转换线缆照片。

3. 问题是偶发性还是持续性的？

4. 提供问题场景的图像。

5. **第 V 节**中的故障排除方法是否解决了问题？

6. 问题解决的预期时间线。
