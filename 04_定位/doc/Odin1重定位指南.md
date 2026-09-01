# Odin1 重定位指南

这份说明只讲 Odin1 的重定位怎么用，所有路径都按当前整理后的工作区来写。

## 1. 先准备什么

- 一张已经保存好的 `.bin` 地图
- Odin1 的配置文件 `config/control_command.yaml`
- 启动脚本 `D:\git\odin\odin_open\04_定位\地图与重定位\runros.sh`

建议把地图放到：

```text
<workspace>/src/odin_ros_driver/map/
```

## 2. 自动重定位

自动重定位就是只给地图，不额外给初始位姿。

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: "<workspace>/src/odin_ros_driver/map/xxx.bin"
```

建议条件：

- 起始位置尽量靠近建图轨迹
- 朝向不要差太多
- 场景里要有明显特征

启动后，Odin1 会持续尝试把当前观测和已有地图对上；成功后会发布 `map -> odom`。

## 3. 指定初始位姿重定位

如果你已经知道大概起点，就把初始位姿也一起写进去：

```yaml
register_keys:
  custom_map_mode: 2
  relocalization_map_abs_path: "<workspace>/src/odin_ros_driver/map/xxx.bin"
  custom_init_pos: [x, y, z, qx, qy, qz, qw]
```

`custom_init_pos` 一共 7 个值：

- `x, y, z`：位置
- `qx, qy, qz, qw`：四元数姿态

常见写法：

```yaml
custom_init_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
custom_init_pos: [5.2, -3.1, 0.0, 0.0, 0.0, 0.707, 0.707]
```

## 4. 常用流程

1. 先确认地图路径是绝对路径。
2. 再把 `custom_map_mode` 设为 `2`。
3. 需要初始位姿时再填 `custom_init_pos`。
4. 最后启动驱动。

```bash
source <workspace>/install/setup.bash
D:\git\odin\odin_open\04_定位\地图与重定位\runros.sh
```

## 5. 如果没成功

- 先确认地图文件真的存在
- 再确认 `custom_map_mode` 是否为 `2`
- 再确认 `custom_init_pos` 是否正好 7 个值
- 再确认四元数是否已归一化

## 6. 相关话题

- `/odin1/odometry`
- `/odin1/cloud_slam`
- `/tf`

重定位成功后，`/tf` 里应该能看到 `map -> odom`。
