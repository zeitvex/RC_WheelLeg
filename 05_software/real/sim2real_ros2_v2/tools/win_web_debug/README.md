# Windows Runtime Web Debug

This folder contains a small Windows-side HTTP server for debugging the Nano runtime Web bridge through UDP.

Most of the time you can open the Nano runtime Web directly:

```text
http://<nano-ip>:18080
```

Use this tool only when you want the browser and HTTP server to run on Windows while Nano communicates over UDP.

## Start

From the repository root:

```powershell
python .\05_software\real\sim2real_ros2_v2\tools\win_web_debug\server.py --nano-host <nano-ip> --http-port 8088
```

Or from this folder:

```powershell
python server.py --nano-host <nano-ip> --http-port 8088
```

Open:

```text
http://127.0.0.1:8088
```

## Related Runtime Node

Nano side:

```text
src/sim2real_runtime/src/web_udp_bridge_node.py
```

See:

- [Runtime Web and UDP bridge](../../docs/WEB_DEBUG_USAGE.md)
- [ROS 2 v2 snapshot](../../README.md)
