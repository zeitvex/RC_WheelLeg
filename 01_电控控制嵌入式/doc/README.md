# 电控、控制与嵌入式

这一部分负责把 Odin1 的感知与定位结果接入机器人真机，完成 `sim2real`、硬件桥接、嵌入式控制和比赛任务集成。

## Odin1 在控制链路中的位置

```text
Odin1
  -> odin_ros_driver
  -> IMU / odometry / point cloud / TF
  -> 比赛任务集成
  -> 导航与命令仲裁
  -> sim2real 策略
  -> hardware bridge
  -> CAN 电机
```

Odin1 是上游状态与环境输入，不直接负责关节控制。控制系统主要使用：

- `/odin1/imu`：机体角速度、加速度和姿态估计所需的惯性数据；
- `/odin1/odometry`：连续里程计状态；
- `/tf`：`map -> odom -> base_link` 坐标关系；
- `/odin1/cloud_slam`：建图、重定位和点云导航所需的环境数据。

## 目录

- `比赛任务集成/`：最终比赛使用的完整 ROS 2/C++ 运行栈，包含 `runros.sh`、Odin1 驱动、策略运行、导航、地图、路线、屏幕和部署文件；其中 `sim2real_python/` 是独立的 Python 真机部署方案。
- `真机控制/`：早期真机控制、逆运动学和轨迹插值代码。
- `固件/`：嵌入式固件资料。

需要运行比赛系统时，进入 `比赛任务集成/`，阅读其中的 `doc/README.md` 和部署说明。不要把 `src/`、地图、策略和启动脚本拆到其他分类后再拼接。

## 参考

电控、嵌入式、CAN 和真机控制的组织方式参考：

[Dichen33/RC_Legged_Control](https://github.com/Dichen33/RC_Legged_Control)

该仓库用于参考下游控制工程的组织方式。本项目的主线仍然是 Odin1 提供真实传感器、里程计和定位输入后的完整落地。
