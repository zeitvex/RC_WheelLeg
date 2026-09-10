# 比赛任务集成

这是最终比赛使用的完整真机运行目录。Odin1 位于整套系统的上游，负责提供 IMU、里程计、点云和重定位结果；本目录负责把这些输入接入导航、策略、命令仲裁和电机执行链路。

## 运行链路

```text
Odin1
  -> src/odin_ros_driver
  -> map / odom / base_link
  -> 导航与比赛路线
  -> sim2real 策略
  -> sim2real_hw
  -> CAN 电机
```

## 目录

- `src/odin_ros_driver/`：Odin1 ROS 2 驱动；
- `src/sim2real_bringup/`：整机启动与运行参数；
- `src/sim2real_runtime/`：策略推理、命令仲裁、导航和安全逻辑；
- `src/sim2real_hw/`：状态读取、目标下发和 CAN 电机桥接；
- `map/`：比赛点云地图和路线文件；
- `../../../05_规划/打点工具/`：PCD 查看、打点、路线检查和路线数据；
- `policies/`：比赛策略模型；
- `sim2real_python/`：不依赖 ROS 2 的 Python 真机部署方案；
- `runros.sh`、`start_sim2real.sh`：启动脚本。

地图位于本目录的 `map/`，不是 `src/odin_ros_driver/map/`。规划工具位于 `05_规划/打点工具/`，这里的 `map/` 是比赛运行时使用的地图和路线资产。

## 启动

```bash
cd <workspace>
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
./runros.sh
```

使用 Odin1 建图或重定位前，先阅读 `04_定位/doc/` 中的设备说明，再根据本目录的部署、系统架构和路线说明配置运行参数。

## Odin1 的实际输入

- `/odin1/imu`：策略观测、站立平衡和安全保护；
- `/odin1/odometry`：短时运动状态；
- `/tf`：重定位后的全局坐标关系；
- `/odin1/cloud_slam`：点云地图与定位相关数据。

Odin1 的定位结果生成导航目标速度，IMU 数据进入策略观测和独立安全检查，最终由本目录的控制与硬件模块下发到电机。
