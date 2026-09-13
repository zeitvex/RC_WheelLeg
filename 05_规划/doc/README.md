# 规划与打点工具

本分类保存基于 Odin1 点云地图进行路线编辑、航点打点、避障区域标注和离线路线检查的工具。

## Odin1 在这里的作用

Odin1 输出的 `pcd` 点云用于建立比赛场地的平面参考；规划工具在点云坐标系中生成航点和路线，最终由 `01_电控控制嵌入式/ROS2控制/` 中的导航节点读取并执行。

```text
Odin1 点云 / 地图坐标
  -> PCD 查看与打点
  -> 航点、任务段和避障区域
  -> 路线安全检查
  -> ROS2控制/map/routes/
  -> 导航目标与策略切换
```

## 目录

- `打点工具/nav_map_viewer.py`：PCD、场地 XML、航点和避障区域综合编辑器；
- `打点工具/avoid_region_tool.py`：避障区域编辑器；
- `打点工具/route_safety_check.py`：路线净空检查；
- `打点工具/route_candidate_optimizer.py`：航点候选优化；
- `打点工具/run_route_experiments.py`：路线检查与 Sim2Sim 批量验证；
- `打点工具/pcd/`：点云预览文件；
- `打点工具/points/`：航点路线数据；
- `打点工具/xml/`：比赛场地 XML。

## 使用

```powershell
cd D:\git\odin\odin_open\05_规划\打点工具
python nav_map_viewer.py
python route_safety_check.py
python avoid_region_tool.py --pcd .\pcd\1hao.pcd
```

训练仿真中的批量路线验证会直接读取本目录，不再维护第二份 `nav_tools` 副本。

详细的工具输入、输出和 Odin1 地图使用流程见
[`打点工具说明.md`](打点工具说明.md)。
