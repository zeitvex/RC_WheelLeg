# PCD 地图查看和打点工具

这是离线 Web 工具，用于查看 Odin PCD 地图和保存路线点。它不是 Nano 运行时 Web。

## 启动

在仓库根目录：

```powershell
python .\tools\pcd_map_viewer\server.py --http-port 8090
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
