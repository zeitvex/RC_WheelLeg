# Pure Pursuit Yaw检查优化 - 最终配置

## 🎯 核心问题

**现象**: 机器人没转到位就开始前进 → 斜着撞杆子

**根本原因**: Pure Pursuit的yaw检查不够严格

---

## ✅ 已修改的参数

### runtime.yaml

```yaml
# 关键修改1: Script模式yaw门限
nav_slalom_script_yaw_gate_deg: 8.0  # 从12度 → 8度

# 关键修改2: 全局yaw容差
nav_goal_yaw_tolerance_deg: 8.0      # 从12度 → 8度
```

**作用**:
- yaw偏差 > 8度时：只转向，不前进
- yaw偏差 ≤ 8度时：才允许前进

---

## 📊 完整配置总结

### 1. 禁用干扰的规划器
```yaml
nav_local_planner_enabled: false
nav_astar_enabled: false
```

### 2. 速度和精度
```yaml
nav_slalom_max_vx: 0.50
nav_slalom_lookahead: 0.15
nav_slalom_tolerance: 0.05
```

### 3. Yaw控制（新增）
```yaml
nav_goal_yaw_tolerance_deg: 8.0
nav_slalom_script_yaw_gate_deg: 8.0
```

---

## 🔍 工作原理

### Pure Pursuit算法流程
```
1. 看前方lookahead距离的目标点
2. 计算到目标的方向和距离
3. 检查yaw偏差
   - 如果 yaw偏差 > yaw_gate (8度):
       只发wz转向，vx=0
   - 如果 yaw偏差 ≤ yaw_gate (8度):
       发vx前进 + wz微调
4. 到达目标点，推进下一个
```

### 之前的问题
```
yaw_gate = 12度（太宽松）
↓
yaw偏差11度时就开始前进
↓
还没对准就冲出去
↓
斜着撞杆子
```

### 修改后
```
yaw_gate = 8度（更严格）
↓
yaw偏差必须≤8度才前进
↓
基本对准后才移动
↓
不会斜着撞
```

---

## 📁 使用的文件

**路线**: `points_nav1007_optimized.json`
- 总航点: 47
- 绕杆航点: 21
- 参数: speed=0.50, lookahead=0.15, tolerance=0.05

**地形**: `tools/nav_tools/xml/A.xml`

**配置**: `runtime.yaml` (已修改)

---

## 🚀 下一步

### 实机测试
```bash
# 1. 重启系统加载新配置
ros2 launch sim2real_bringup sim2real_system.launch.py

# 2. 验证参数
ros2 param get /sim2real_simple_nav_node nav_slalom_script_yaw_gate_deg
# 应该显示: 8.0

ros2 param get /sim2real_simple_nav_node nav_goal_yaw_tolerance_deg
# 应该显示: 8.0

# 3. 加载路线
# 使用: sim2real_ros2_v2_ooo/map/routes/points_nav1007_optimized.json

# 4. 监控
ros2 topic echo /cmd_vel_nav
# 观察: 转向时vx应该接近0，对准后才有vx速度
```

---

## 💡 预期效果

**修改前**:
- 机器人边转边进
- yaw偏差大时仍有前进速度
- 导致斜着撞杆子

**修改后**:
- 转向时几乎不前进（vx≈0）
- 对准后才快速前进（vx=0.5）
- 动作分离：先转向，后前进

---

## ⚠️ 如果还有问题

### 场景A: 还是斜着撞
可能需要进一步收紧:
```yaml
nav_slalom_script_yaw_gate_deg: 5.0  # 改为5度
```

### 场景B: 太慢，一直在转
说明yaw_gate太严格:
```yaml
nav_slalom_script_yaw_gate_deg: 10.0  # 放宽到10度
```

### 场景C: 卡顿
检查是否在等待yaw对准:
```bash
ros2 topic echo /simple_nav/status
# 看是否一直在"aligning"状态
```

---

**配置已优化完成，ready for testing!** 🎯
