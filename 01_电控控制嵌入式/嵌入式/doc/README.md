# 嵌入式固件参考

本目录保存前期 8DOF 阶段使用过的 `Dji_A` STM32/FreeRTOS 工程，主要用于参考 CAN/UART 通信、FreeRTOS 任务、PID、ramp 和电机反馈处理。

它不是当前 16DOF 轮足最终实机的下位机主线。最终 ROS 2 实机控制、Odin1 接入和策略部署统一位于：

```text
../ROS2控制/
```

## 工程位置

```text
Dji_A/
├── Project/RoboMentors_Board.uvprojx
├── Inc/                    # STM32F4、CMSIS、HAL、FreeRTOS 头文件
└── User/
    ├── app/                # FreeRTOS 应用任务
    ├── driver/             # CAN、UART、电机和腿部驱动
    └── algorithm/          # PID、ramp 等基础算法
```

## 重点文件

- `User/app/lingzu_task.c`：轮腿控制任务；
- `User/driver/can_device.c`：CAN 收发与反馈解析；
- `User/driver/lingzu_motor.c`：电机封装与控制输出；
- `User/driver/Movement.c`、`leg.c`：腿部运动基础逻辑；
- `User/algorithm/pid.c`、`ramp.c`：闭环控制与斜坡限幅。

使用 Keil MDK 打开工程前，需要根据实际开发板、芯片、电机和通信协议重新检查 CAN ID、方向、零位、限幅和急停链路。该参考工程不应直接与最终 ROS 2 主线混合编译。
