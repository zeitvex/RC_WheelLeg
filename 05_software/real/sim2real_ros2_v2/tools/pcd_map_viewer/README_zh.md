# PCD 地图查看和打点工具

这是离线 Web 工具，用于查看 Odin PCD 地图和保存路线点。它不是 Nano 运行时 Web。

## 启动

在仓库根目录：

```powershell
python .\05_software\real\sim2real_ros2_v2\tools\pcd_map_viewer\server.py --http-port 8090
```

打开：

```text
http://127.0.0.1:8090
```

路线点保存：

- `x`
- `y`
- `yaw_deg`
- `speed`
- `policy`
- `tolerance`

不保存 `z` 和 `action`。

当前 `v0.12.0` 运行配置使用 `map/routes/A_min/A_min_route.json`。编辑器也能保存 YAML，但本快照没有旧文档曾描述的 `/route_runner/cmd` 控制接口。

地图抽样边界见 [`../../map/README.md`](../../map/README.md)，版本说明见 [`../../README.md`](../../README.md)。
