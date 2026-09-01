# 导航地图与打点工具

本目录保存比赛后期使用的独立导航地图、PCD 查看、航点编辑、避障区域编辑和路线验证工具。它同时为 `sim2sim/nav_route_sim2sim_check.py` 提供航点与避障几何定义。

## 目录

```text
nav_tools/
├─ nav_map_viewer.py               # 地图、PCD、XML、航点和避障区综合编辑器
├─ avoid_region_tool.py            # 独立避障多边形编辑器
├─ route_safety_check.py           # 离线路线净空检查
├─ route_candidate_optimizer.py    # 航点候选优化
├─ run_route_experiments.py        # 安全检查与 Sim2Sim 批量实验
├─ mirror_nav_xml_points.py        # XML 与航点镜像
├─ pcd_transform_tool.py           # PCD 平移和旋转
├─ transform_pcd_xy.py             # PCD 坐标原点变换
├─ downsample_ascii_pcd.py         # ASCII PCD 确定性抽样
├─ annotate_odin1_relocalization_frame.py
├─ pcd/                            # 小于 10 MB 的打点预览点云
├─ points/                         # 比赛期间的路线迭代 JSON
├─ xml/                            # 1 号、2 号和 A/B 场地 XML
├─ regions/                        # 独立避障区域输出目录
└─ assets/                         # 坐标和机构示意图
```

## 运行

在 `rc_mjlab` 根目录执行：

```powershell
# 地图与航点综合编辑器
uv run --with-requirements .\tools\nav_tools\requirements.txt `
  python .\tools\nav_tools\nav_map_viewer.py

# 指定 PCD 的独立避障区域编辑器
uv run --with-requirements .\tools\nav_tools\requirements.txt `
  python .\tools\nav_tools\avoid_region_tool.py --pcd 1hao.pcd

# 默认检查 points_20260715_120154.json 与 1hao.xml
uv run python .\tools\nav_tools\route_safety_check.py

# 使用最终 Rough 策略进行默认路线 Sim2Sim 检查
uv run --with-requirements .\sim2sim\requirements.txt `
  python .\sim2sim\nav_route_sim2sim_check.py
```

候选路线和批量实验必须显式保存为新文件，不应覆盖历史路线：

```powershell
uv run python .\tools\nav_tools\route_candidate_optimizer.py

uv run --with-requirements .\sim2sim\requirements.txt `
  python .\tools\nav_tools\run_route_experiments.py `
  --points .\tools\nav_tools\points\points_20260715_120154.json
```

## PCD 抽样

仓库中的两份 PCD 从原始 ASCII 点云按固定步长均匀抽样，字段、坐标和 PCD 头结构保持不变。它们面向地图显示和人工打点，不替代原始高密度点云用于建图、定位精度评估或点云算法基准。

| 文件 | 原始点数 | 抽样步长 | 仓库点数 | 仓库大小 | SHA-256 |
| --- | ---: | ---: | ---: | ---: | --- |
| `pcd/1hao.pcd` | 9,163,893 | 46 | 199,215 | 9,876,010 B | `48B231C52BECA51316F352300C8B2046133E92359E0855227D93DEB0D927AD34` |
| `pcd/2hao.pcd` | 10,043,048 | 52 | 193,136 | 9,719,736 B | `714516A7A726D46311A58507149FBC93D622616BE79C59FFED622274D6526B1F` |

复现抽样：

```powershell
python .\tools\nav_tools\downsample_ascii_pcd.py `
  <原始PCD> <输出PCD> --max-bytes 9900000
```

## 航点数据

`points/` 保留原文件名和时间顺序，没有把多个路线重命名成 `old`、`new` 或 `final`。各文件的航点数、避障区和 XML 绑定见 [`points/README.md`](points/README.md)。

- 请求目录中的 `route_safety_check.py` 是较早版本，因此保留 `v0.8.0` 已归档的后期兼容版本。
- `1B_FF.json` 引用了请求目录中缺失的 `xml/B_C.xml`，本次从后期整合目录补齐该文件。
- `.uv-cache`、`__pycache__`、自动候选、实验日志和重复轨迹输出不归档。

整理时使用默认圆形包络和 `0.05 m` 额外净空检查 `points_20260715_120154.json`，报告了 3 组线段—避障区净空不足。该结果按原样保留，未自动移动航点；它表示保守几何检查仍有待复核，不等同于路线没有经过实机使用。

## 主要操作

- `Pan`：拖动地图；滚轮缩放；`F` 适应窗口。
- `Point`：添加、插入、选择和编辑航点；`Backspace` 删除。
- `Terrain`：选择并调整 XML 障碍组的位置和偏航。
- `Avoid`：绘制避障多边形；`Enter` 闭合；`Delete` 删除。
- `Load JSON`：载入 `waypoints` 或 `segments[].waypoints`。
- `Save JSON` / `Save All JSON`：同时保存航点和避障区域。
- `M`：切换 XML；`O`：切换坐标原点；`Esc`：退出。

任务字段约定：`none`、`slalom`、`gravel`、`wall`、`low_bar`、`stairs`、`ramp_bridge`、`spawn` 和 `return`。
