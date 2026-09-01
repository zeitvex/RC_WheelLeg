# Odin1 扫图与重定位总览

本页只整理 Odin1 在扫图、建图和重定位里的作用，不写活动规则和评分说明。

## 当前整理位置

- 文档目录：`D:\git\odin\odin_open\04_定位\doc`
- 配套脚本：`D:\git\odin\odin_open\04_定位\地图与重定位\runros.sh`
- 部署说明：`D:\git\odin\odin_open\04_定位\地图与重定位\doc\DEPLOYMENT_GUIDE.md`

## Odin1 在链路里的作用

Odin1 负责提供传感器数据、里程计和 TF 基础信息，支撑下面几类任务：

- 扫图 / 建图：采集点云和位姿，生成可保存地图
- 重定位：在已有地图上恢复当前位姿
- 上层调用：把结果提供给导航、控制或比赛任务系统

## 相关文件

- `README.md`
- `Odin1使用手册.md`
- `Odin1工作空间说明.md`
- `Odin1重定位指南.md`
- `RELOCALIZATION_GUIDE.md`（官方参考）

## 参考目录结构

```text
D:\git\odin\odin_open
├── 04_定位
│   ├── 地图与重定位
│   │   ├── runros.sh
│   │   └── doc
│   │       ├── README.md
│   │       └── DEPLOYMENT_GUIDE.md
│   └── doc
│       ├── README.md
│       ├── Odin1扫图与重定位总览.md
│       ├── Odin1工作空间说明.md
│       ├── Odin1使用手册.md
│       └── Odin1重定位指南.md
```

## 核心调用链

```text
Odin1传感器数据 -> odin_ros_driver -> 建图 / 重定位 -> map -> odom -> 上层系统
```

## 说明

- 只保留 Odin1 相关内容。
- 路径示例统一以当前仓库结构为准。
- 不写规则、奖项、报名等说明。
