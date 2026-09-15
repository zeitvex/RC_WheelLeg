# PCD Map Viewer

Offline Web tool for Odin PCD inspection and route editing.

This is not the Nano runtime Web UI. Use it before a run to inspect the map and save route YAML files.

## Start

From the repository root:

```powershell
python .\tools\pcd_map_viewer\server.py --http-port 8090
```

Open:

```text
http://127.0.0.1:8090
```

You can also run directly from this directory:

```powershell
python server.py --http-port 8090
```

## What It Does

- Scans `map/*.pcd`.
- Shows raw Odin PCD as a rotatable 3D point cloud.
- Supports z filtering, default `z=-2.0..1.0m`.
- Supports voxel display for structure checks.
- Provides a simplified 2D layer view.
- Lets you click waypoints in 3D or 2D.
- Lets you draw obstacle terrain rectangles and edit their x/y/yaw/size.
- Tags new waypoints with the matching obstacle terrain, defaulting to `flat`.
- Saves route YAML and JSON under `map/routes/<pcd_name>/`.

## Route Fields

Saved waypoint fields:

- `x`
- `y`
- `yaw_deg`
- `speed`
- `policy`
- `tolerance`
- `obstacle`
- `obstacle_name`

Not saved:

- `z`
- `action`

The browser may keep local `_viewZ` only for drawing markers in 3D. The runtime route runner is planar.

## Saved Format

```yaml
name: test_route
map: map1
frame_id: map
obstacles:
  - id: 1
    name: wall_1
    obstacle: wall
    x: 1.5000
    y: 2.0000
    yaw_deg: 0.00
    length: 1.000
    width: 0.500
    policy: rough
waypoints:
  - id: 1
    x: 1.0000
    y: 2.0000
    yaw_deg: 0.00
    speed: 0.350
    policy: rough
    tolerance: 0.150
    obstacle: wall
    obstacle_name: wall_1
```

Default runtime route:

```text
map/routes/map1/test_route.yaml
```

## Runtime Test

```bash
ros2 topic pub --once /route_runner/cmd std_msgs/msg/String "{data: reload}"
ros2 topic pub --once /route_runner/cmd std_msgs/msg/String "{data: start}"
ros2 topic pub --once /route_runner/cmd std_msgs/msg/String "{data: stop}"
```

See also:

- [Maps And Routes](../../docs/ROUTES_AND_MAPS.md)
- [Common Commands](../../docs/COMMANDS.md)
