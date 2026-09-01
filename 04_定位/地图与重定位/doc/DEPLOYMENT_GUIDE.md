# ROS2 C++ Sim2Real 运动控制栈 - 部署指南

本工作区提供了一个自包含、独立的 C++ ROS2 Humble 实现，用于在 Jetson Orin 目标机上部署轮腿四足机器人控制策略。

---

## 1. 前提条件与环境

### 硬件
* **目标计算机**：运行 Ubuntu 22.04 LTS 的 Jetson Orin Nano / Orin NX / AGX Orin。
* **IMU 传感器**：Odin 集成 IMU，发布至 `/odin1/imu`。
* **CAN 总线适配器**：Peak CAN、USB-to-CAN 或板载 SocketCAN 接口，使用 CAN0 和 CAN1。

### 主机依赖
* **操作系统**：Ubuntu 22.04 LTS (Jammy Jellyfish)。
* **ROS 2 发行版**：ROS 2 Humble（Desktop-Base 或 ROS-Base）。
* **C++ 编译器**：支持 C++17 的 GCC/G++ 9.0+。
* **库与 ROS2 包**：
  * `libyaml-cpp-dev`
  * `libeigen3-dev`
  * `libusb-1.0-0-dev`（Odin USB 传感器通信）
  * `libpcl-dev` 和 `libopencv-dev`（3D 点云与相机处理）
  * `ros-humble-navigation2` 和 `ros-humble-nav2-bringup`（Nav2 规划器/控制器服务器）
  * `ros-humble-pointcloud-to-laserscan`（点云转激光扫描，供 AMCL 使用）
  * `ros-humble-cv-bridge` 和 `ros-humble-pcl-conversions`（Odin 传感器驱动图像与点云处理）
  * `can-utils`（SocketCAN 验证工具）

---

## 2. 本地编译与部署

按以下步骤在主机系统上编译运行整个栈：

### 步骤 1：安装系统依赖
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake can-utils libyaml-cpp-dev libeigen3-dev \
    libusb-1.0-0-dev libpcl-dev libopencv-dev ros-humble-navigation2 \
    ros-humble-nav2-bringup ros-humble-pointcloud-to-laserscan \
    ros-humble-cv-bridge ros-humble-pcl-conversions
```

### 步骤 2：下载 ONNXRuntime C++ SDK
策略需要 ONNXRuntime 库来运行推理。必须下载并解压到已知目录：

```bash
# 创建目录
sudo mkdir -p /opt/onnxruntime
cd /opt

# 针对 Jetson Orin (ARM64 / aarch64)：
sudo wget https://github.com/microsoft/onnxruntime/releases/download/v1.16.3/onnxruntime-linux-aarch64-1.16.3.tgz
sudo tar -zxvf onnxruntime-linux-aarch64-1.16.3.tgz --strip-components=1 -C /opt/onnxruntime

# 或标准桌面仿真 (x86_64 / amd64)：
# sudo wget https://github.com/microsoft/onnxruntime/releases/download/v1.16.3/onnxruntime-linux-x64-1.16.3.tgz
# sudo tar -zxvf onnxruntime-linux-x64-1.16.3.tgz --strip-components=1 -C /opt/onnxruntime
```

导出 CMake 辅助变量：
```bash
export ONNXRUNTIME_DIR=/opt/onnxruntime
```

### 步骤 3：构建工作区
进入包含 `src/` 的本包根目录，运行 `colcon`：
```bash
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 步骤 4：配置 SocketCAN 接口
启动前，以 1 Mbps 波特率激活 CAN 接口：
```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```
使用 `ifconfig` 或 `ip link` 验证接口已启动。

### 步骤 5：启动节点
使启动脚本可执行并运行：
```bash
chmod +x start_sim2real.sh
./start_sim2real.sh
```

---

## 3. Docker 部署（推荐）

强烈推荐使用 Docker 隔离依赖，避免 Jetson Orin 上的库版本冲突。

### 步骤 1：构建镜像
确保在 `sim2real_ros2` 目录中（包含 `Dockerfile`）：
```bash
# 使用标准 docker build：
docker build -t sim2real_ros2:latest .

# 或使用 Docker Compose：
docker compose build
```

### 步骤 2：运行容器
对于真实硬件部署，容器**必须**共享主机网络栈（用于 ROS2 DDS 和 SocketCAN）并具备线程优先级能力以实现实时调度：

```bash
# 选项 A：手动运行
docker run -it \
  --network host \
  --privileged \
  --cap-add=sys_nice \
  --volume=/dev:/dev \
  --shm-size=2g \
  --name sim2real_ros2_run \
  sim2real_ros2:latest

# 选项 B：通过 Docker Compose 运行（最简单）
docker compose up -d
```

---

## 4. 系统拓扑与话题

控制节点通过标准 ROS 2 DDS 消息与传感器驱动和导航栈交互：

* **IMU 输入**：订阅 `/odin1/imu`（`sensor_msgs/msg/Imu`）。硬件节点自动执行逆轴旋转（`x_raw = -y_ros`，`y_raw = x_ros`）以重建 RL 策略期望的原始坐标系。
* **控制命令**：订阅 `/cmd_vel` 和 `/cmd_vel_stamped`（`geometry_msgs/msg/Twist` / `TwistStamped`），由导航栈或手动键盘节点发布。
* **里程计输入**：订阅 `/odom`（`nav_msgs/msg/Odometry`），由 `odom_relay_node` 从 `/odin1/odometry` 中继并重映射帧名后提供。
* **急停**：订阅 `/safety/estop`（`std_msgs/msg/Bool`）。发布 `true` 触发软件急停，机器人进入低刚度阻尼刹车。
* **状态遥测**：发布 `runtime/state`（`sim2real_interfaces/msg/RuntimeState`），包含当前关节速度、温度、IMU 输出和诊断信息。
* **策略目标**：发布 `runtime/target`（`sim2real_interfaces/msg/RuntimeTarget`），包含策略推理输出的目标关节位置。

### TF 树
```
odom ──→ base_link    （由 odom_relay_node 广播）
map  ──→ odom         （由 AMCL / Odin SLAM 发布，取决于运行模式）
```

---

## 5. 集成 ROS 2 导航与传感器驱动

### USB 设备权限（Odin 传感器）
要运行物理 Odin 传感器驱动（`odin_ros_driver`），目标计算机必须具有传感器 USB 接口的读写权限。在主机系统上添加以下 udev 规则：

```bash
# 1. 添加 udev 规则
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-odin-usb.rules

# 2. 重新加载 udev 规则并重新插拔传感器
sudo udevadm control --reload
sudo udevadm trigger
```

### 集成启动参数
统一启动文件 `sim2real_system.launch.py` 支持模块化激活传感器驱动和 Nav2 导航栈：

* `launch_driver`（默认：`true`）：启动 `odin_ros_driver` 节点以获取 IMU 和点云遥测。
* `launch_nav2`（默认：`false`）：按需启动 ROS2 Navigation2；比赛默认使用 `simple_nav_node.py` 的路线跟踪。

#### 1. 完整真实硬件闭环（默认）
启动运动控制运行时、物理 CAN 桥接、Odin 传感器驱动和 Nav2 导航：
```bash
ros2 launch sim2real_bringup sim2real_system.launch.py dry_run:=false launch_driver:=true launch_nav2:=true
```

#### 2. Dry-Run / 仿真航点测试
在 dry-run 模式下运行策略运行时和 Nav2 导航（不访问 CAN 总线或物理 USB 传感器，适合测试导航话题路由）：
```bash
ros2 launch sim2real_bringup sim2real_system.launch.py dry_run:=true launch_driver:=false launch_nav2:=true
```

#### 3. 仅运动控制（无导航）
禁用传感器驱动和 Nav2，让运动策略等待 `/cmd_vel` 上的手动速度输入（如键盘遥操作）：
```bash
ros2 launch sim2real_bringup sim2real_system.launch.py launch_driver:=false launch_nav2:=false
```
