# `FACTS_AND_ASSUMPTIONS`

## 已确认

- 当前部署模型：`sim2real/policies/model_rough.pt`
- 源模型：`model_2000.pt`
- actor 输入：`53D`
- actor 输出：`16D`
- 当前 actor 不吃 `base_lin_vel`
- 当前 actor 不吃 `height_scan`

## 当前观测顺序

1. `base_ang_vel * 0.25`
2. `projected_gravity`
3. `command`
4. `joint_pos_rel`（12）
5. `joint_vel_rel * 0.05`（12）
6. `wheel_vel * 0.05`（4）
7. `last_actions`（16）

## 当前控制定义

- 控制频率：`50Hz`
- 腿缩放：`0.125 / 0.25`
- 轮缩放：`5.0`
- 腿 LPF：`5Hz`
- 轮 LPF：`15Hz`

## 当前仍依赖现场一致的部分

- IMU 安装方向与上一版校正一致
- 当前 MJCF / 电机参数对应这次重新训练后的模型
- 电机零位、方向、接线已按当前硬件修正

## 本次实现边界

不再支持：

- `crawl` 模型
- 多策略切换
- `318D` 历史输入
- 旧版 `startup.start_pose`

## 本次排查结论

代码应只围绕当前 rough 模型运行。  
如果后续模型结构再改，必须重新核对观测、动作缩放、控制频率和部署文档。
