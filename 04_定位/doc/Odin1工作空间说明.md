# Odin1 工作空间说明

本页只说明 Odin1 驱动相关工作空间怎么摆，不写赛事背景，也不写重复的长篇步骤。

## 目标结构

```text
<workspace>/
├── src/
│   └── odin_ros_driver/
├── build/
├── install/
└── log/
```

## 当前位置的整理结果

- Odin1 文档和运行工作区：`D:\git\odin\odin_open\04_定位\doc`
- 驱动包：`D:\git\odin\odin_open\04_定位\doc\src\odin_ros_driver`

## 关键位置

- 工作空间根目录：`<workspace>`
- 驱动包目录：`<workspace>/src/odin_ros_driver`
- 配置目录：`<workspace>/src/odin_ros_driver/config`
- 地图保存目录：`<workspace>/src/odin_ros_driver/map`
- 通用启动脚本：`<workspace>/runros.sh`
- 扫图脚本：`<workspace>/start_mapping.sh`
- 重定位脚本：`<workspace>/start_relocalization.sh`

## 基本流程

1. 进入工作空间根目录。
2. 安装 ROS 2 依赖。
3. 执行 `colcon build`。
4. 执行 `source <workspace>/install/setup.bash`。
5. 启动 `<workspace>/runros.sh`，或按模式使用 `start_mapping.sh`、`start_relocalization.sh`。

## 最常用命令

```bash
cd <workspace>
colcon build
source <workspace>/install/setup.bash
./runros.sh
```

## 说明

- `runros.sh`、建图和重定位脚本与驱动包都位于当前工作区。
- 如果还没有 `install/`，先 build 再运行。
- `doc/` 本身就是工作空间根目录，不需要再跳转到其他分类目录。
