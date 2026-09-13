# 早期真机控制验证

本目录保存 16DOF 轮足在进入最终 ROS 2 主线前的 IK、轨迹插值和接口验证代码。

- `sim2real_control_api.py`：早期真机控制接口；
- `trajectory_interpolator.py`：关节和姿态轨迹插值；
- `sim_to_real_deploy_beifen.py`：早期部署脚本备份。

这些文件用于理解控制接口和验证过程，不是当前推荐的比赛启动入口。最终 ROS 2 控制系统、Odin1 接入、策略运行和 CAN 硬件桥接统一使用：

```text
../../../ROS2控制/
```

其中部分脚本保留了历史部署路径，运行前需要按当前机器和策略文件修改配置。
