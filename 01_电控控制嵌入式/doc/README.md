# 电控控制嵌入式

这一类内容负责把 Odin 的状态接进来，再把控制量送出去。

## Odin 在这里做什么

- 通过 IMU 和里程计给控制链路提供状态反馈
- 作为真机运行时的传感器来源
- 作为 CAN 电机控制前的上游输入

## 主要调用链

```text
Odin IMU / Odometry -> hardware bridge -> RuntimeState
RuntimeTarget -> hardware bridge -> CAN
```

## 相关内容

- `固件/`
- `电控与硬件桥接/`
- `真机控制/`

