# 感知

这一类内容负责把 Odin 的传感器数据接入系统。

## Odin 在这里做什么

- 提供 IMU
- 提供相机、点云、里程计等传感器数据
- 作为感知链路的真实硬件入口

## 相关内容

- `Odin驱动/`
- `OdinIMU/`

## 主要调用链

```text
Odin sensor -> driver -> ROS topics -> upper modules
```

