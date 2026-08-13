# RC_WheelLeg

RC_WheelLeg 是山东华宇工学院 16DOF 串联轮足机器人项目。

当前 `16dof` 分支用于整理 16DOF 机械、强化学习训练、Sim2Sim、Sim2Real、ROS 2 部署和比赛版本。机械资料和第一代软件闭环已经完成首轮整理。

## 平台概览

- 四条轮腿
- 每条腿 3 个腿部关节
- 4 个驱动轮
- 总计 16 个执行器
- 控制路线：强化学习训练、仿真验证和真机部署

## 目录结构

```text
RC_WheelLeg/
├─ 01_doc/          # 项目技术文档和使用说明
├─ 02_mechanical/   # CAD、STEP、URDF 和 MJCF
├─ 03_hardware/     # 自研电路、接线和硬件说明
├─ 04_firmware/     # MCU 或嵌入式控制器固件
├─ 05_software/     # 训练、仿真、部署和工具
├─ 06_assets/       # 图片、视频和测试结果
├─ .gitignore
└─ README.md
```

## 当前状态

- [x] 整理 SolidWorks 机械源文件
- [x] 整理整机与关节 STEP 文件
- [x] 整理第一代 mjlab 训练任务和本地依赖
- [x] 整理第一代 MuJoCo、Sim2Sim 和 MJCF
- [x] 整理 IK 真机控制与第一代 Python Sim2Real
- [ ] 核对比赛机械与仿真模型参数
- [ ] 整理 URDF/MJCF 机器人描述
- [ ] 整理后续统一训练、ROS 2 和比赛版本

机械资料入口见 [`02_mechanical/README.md`](02_mechanical/README.md)。

早期软件版本入口见 [`05_software/README.md`](05_software/README.md)。

## 早期真机验证

[![第一代 Sim2Real 真机验证](06_assets/images/early_sim2real_preview.jpg)](06_assets/videos/early_sim2real.mp4)

第一代 Sim2Real 真机测试记录，时长约 41 秒。点击预览图播放或下载原视频。

## 版本管理

- `8dof` 分支和 `v0.1.0` Tag 保存第一代 8DOF 实机版本。
- `16dof` 分支承载第二代轮足平台的完整演进。
- 线性演进阶段使用 Commit、Tag 和 Release 保存，不复制 `old`、`final` 或 `v2` 目录。
- 只有需要长期并行维护的不兼容路线才建立独立分支。
