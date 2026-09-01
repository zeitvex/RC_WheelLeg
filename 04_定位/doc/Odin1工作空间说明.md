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

- Odin1 相关文档：`D:\git\odin\odin_open\04_定位\doc`
- 部署脚本和运行说明：`D:\git\odin\odin_open\04_定位\地图与重定位`

## 关键位置

- 工作空间根目录：`<workspace>`
- 驱动包目录：`<workspace>/src/odin_ros_driver`
- 配置目录：`<workspace>/src/odin_ros_driver/config`
- 地图目录：`<workspace>/src/odin_ros_driver/map`
- 启动脚本：`D:\git\odin\odin_open\04_定位\地图与重定位\runros.sh`

## 基本流程

1. 进入工作空间根目录。
2. 安装 ROS 2 依赖。
3. 执行 `colcon build`。
4. 执行 `source <workspace>/install/setup.bash`。
5. 启动 `D:\git\odin\odin_open\04_定位\地图与重定位\runros.sh`。

## 最常用命令

```bash
cd <workspace>
colcon build
source <workspace>/install/setup.bash
D:\git\odin\odin_open\04_定位\地图与重定位\runros.sh
```

## 说明

- `runros.sh` 现在单独放在 `D:\git\odin\odin_open\04_定位\地图与重定位`，和部署说明放在一起。
- 如果还没有 `install/`，先 build 再运行。
- 当前目录里的 `doc/` 主要存放说明材料，真正运行时还是按工作空间结构来用。
