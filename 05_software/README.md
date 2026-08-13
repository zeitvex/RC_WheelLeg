# 软件

本目录当前保存 16DOF 轮足机器人的第一代完整软件闭环。

```text
05_software/
├─ train/
│  └─ rc_mjlab/       # 训练、MJCF、MuJoCo、Sim2Sim 和本地 mjlab 依赖
└─ real/
   ├─ ik_real/        # IK 轨迹与早期真机控制
   └─ sim2real/       # 第一代 Python 策略真机部署
```

## 数据流

```text
MJCF + mjlab task
        |
        v
   PPO 训练策略
        |
        +----> MuJoCo 独立模型调试
        |
        +----> Sim2Sim 策略验证
        |
        +----> Python Sim2Real ----> 电机 / IMU

IK real --------------------------------> 电机
```

`rc_mjlab` 在早期版本中是自包含工程。训练、MJCF、独立 MuJoCo、Sim2Sim 和策略权重通过相对路径绑定，因此本次保留其原始内部布局，没有为了目录外观拆散。

详细说明见：

- [`train/README.md`](train/README.md)
- [`real/README.md`](real/README.md)
- [`../01_doc/architecture/early_software_stack.md`](../01_doc/architecture/early_software_stack.md)
