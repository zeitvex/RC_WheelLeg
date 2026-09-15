# Windows Runtime Web 调试桥

大多数时候直接打开 Nano Web 即可：

```text
http://<nano-ip>:18080
```

这个工具只在你希望 HTTP 服务跑在 Windows、Nano 通过 UDP 传状态时使用。

## 启动

在仓库根目录：

```powershell
python .\tools\win_web_debug\server.py --nano-host <nano-ip> --http-port 8088
```

打开：

```text
http://127.0.0.1:8088
```
