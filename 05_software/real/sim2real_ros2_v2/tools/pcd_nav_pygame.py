#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import pygame


COLOR_BG = (10, 15, 30)
COLOR_GRID = (22, 29, 48)
COLOR_AXIS = (45, 61, 94)
COLOR_PANEL = (18, 24, 42)
COLOR_PANEL_BORDER = (38, 52, 84)
COLOR_TEXT = (236, 241, 248)
COLOR_MUTED = (148, 163, 184)
COLOR_CYAN = (6, 182, 212)
COLOR_GREEN = (16, 185, 129)
COLOR_GOLD = (234, 179, 8)
COLOR_ROSE = (244, 63, 94)
COLOR_PURPLE = (139, 92, 246)
COLOR_PCD_LOW = (61, 112, 138)
COLOR_PCD_MID = (112, 165, 121)
COLOR_PCD_HIGH = (214, 166, 78)
COLOR_WALL = (244, 63, 94)
COLOR_LOW_BAR = (245, 158, 11)
COLOR_STAIRS = (234, 179, 8)
COLOR_RAMP = (16, 185, 129)
COLOR_ROUGH = (139, 92, 246)
COLOR_STRUCTURE = (148, 163, 184)


@dataclass
class PointXYZ:
    x: float
    y: float
    z: float


@dataclass
class Waypoint:
    x: float
    y: float
    id: int = 0
    yaw_deg: Optional[float] = None
    speed: float = 0.35
    policy: str = "rough"
    tolerance: float = 0.15
    yaw_tolerance_deg: Optional[float] = None
    require_yaw: Optional[bool] = None
    pre_dock_distance: Optional[float] = None
    pre_dock_tolerance: Optional[float] = None
    obstacle: str = "flat"
    obstacle_name: str = ""
    name: str = ""


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class SemanticElement:
    kind: str
    label: str
    x: float
    y: float
    yaw: float
    hx: float = 0.0
    hy: float = 0.0
    radius: float = 0.0
    color: tuple[int, int, int] = COLOR_STRUCTURE
    name: str = ""


@dataclass
class RouteMeta:
    name: str = "route"
    map_name: str = ""
    frame_id: str = "map"
    segment_name: str = "segment_1"
    segment_obstacle: str = "flat"
    yaw_tolerance_default: Optional[float] = 30.0
    require_yaw_default: Optional[bool] = None
    pre_dock_distance_default: Optional[float] = None
    pre_dock_tolerance_default: Optional[float] = None
    created_at: str = ""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Pygame top-down nav-style viewer for PCD maps."
    )
    parser.add_argument("--pcd", default="map/map_b.pcd", help="Input PCD file.")
    parser.add_argument("--route", default="", help="Optional route JSON/YAML-like JSON file.")
    parser.add_argument("--save-route", default="", help="Route JSON output path.")
    parser.add_argument("--route-name", default="", help="Exported route name.")
    parser.add_argument("--route-map", default="", help="Exported route map name.")
    parser.add_argument("--route-segment", default="segment_1", help="Exported segment name.")
    parser.add_argument("--route-obstacle", default="flat", help="Exported segment obstacle label.")
    parser.add_argument("--route-yaw-tol-default", type=float, default=30.0)
    parser.add_argument("--route-require-yaw-default", choices=["none", "true", "false"], default="none")
    parser.add_argument("--route-pre-dock-distance-default", type=float, default=-1.0)
    parser.add_argument("--route-pre-dock-tolerance-default", type=float, default=-1.0)
    parser.add_argument("--default-speed", type=float, default=0.35)
    parser.add_argument("--default-policy", default="rough", choices=["rough", "crawl", "stand", "none"])
    parser.add_argument("--default-tolerance", type=float, default=0.15)
    parser.add_argument("--floor-z-min", type=float, default=-1.6)
    parser.add_argument("--floor-z-max", type=float, default=0.4)
    parser.add_argument("--sample-step", type=int, default=10)
    parser.add_argument("--max-points", type=int, default=180000)
    parser.add_argument("--point-size", type=int, default=1)
    parser.add_argument("--window-width", type=int, default=1120)
    parser.add_argument("--window-height", type=int, default=760)
    parser.add_argument("--map-width", type=int, default=760)
    parser.add_argument("--zoom", type=float, default=35.0, help="Initial pixels per meter.")
    parser.add_argument("--initial-x", type=float, default=0.0)
    parser.add_argument("--initial-y", type=float, default=0.0)
    parser.add_argument("--initial-yaw-deg", type=float, default=0.0)
    parser.add_argument("--pose-file", default="", help="JSON pose file with x/y/yaw or yaw_deg.")
    parser.add_argument("--udp-port", type=int, default=0, help="Listen for pose JSON or 'x y yaw'.")
    parser.add_argument("--ros2-tf", action="store_true", help="Read robot pose from ROS2 TF.")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--mjcf", default="", help="Optional MuJoCo XML/MJCF semantic overlay.")
    parser.add_argument("--overlay-json", default="", help="Optional manual semantic overlay JSON.")
    parser.add_argument("--overlay-dx", type=float, default=0.0, help="Overlay x offset in map frame.")
    parser.add_argument("--overlay-dy", type=float, default=0.0, help="Overlay y offset in map frame.")
    parser.add_argument("--overlay-yaw-deg", type=float, default=0.0, help="Overlay rotation before drawing.")
    parser.add_argument("--overlay-scale", type=float, default=1.0, help="Overlay xy scale before drawing.")
    parser.add_argument("--hide-overlay", action="store_true", help="Do not draw semantic overlay.")
    parser.add_argument(
        "--ros2-goal-topic",
        default="",
        help="Optional std_msgs/String topic. Click publishes: go x y",
    )
    parser.add_argument("--title", default="PCD Nav Pygame")
    return parser.parse_args()


def color_lerp(a: tuple[int, int, int], b: tuple[int, int, int], t: float) -> tuple[int, int, int]:
    t = max(0.0, min(1.0, t))
    return (
        int(a[0] + (b[0] - a[0]) * t),
        int(a[1] + (b[1] - a[1]) * t),
        int(a[2] + (b[2] - a[2]) * t),
    )


def z_color(z: float, z_min: float, z_max: float) -> tuple[int, int, int]:
    t = (z - z_min) / max(1.0e-9, z_max - z_min)
    if t < 0.5:
        return color_lerp(COLOR_PCD_LOW, COLOR_PCD_MID, t * 2.0)
    return color_lerp(COLOR_PCD_MID, COLOR_PCD_HIGH, (t - 0.5) * 2.0)


def parse_float_list(value: str, default: list[float]) -> list[float]:
    if not value:
        return list(default)
    try:
        return [float(v) for v in value.split()]
    except ValueError:
        return list(default)


def quat_to_yaw(quat: list[float]) -> float:
    w, x, y, z = (quat + [0.0, 0.0, 0.0, 0.0])[:4]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rotate_quat_vec(quat: list[float], vec: tuple[float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = (quat + [0.0, 0.0, 0.0, 0.0])[:4]
    vx, vy, vz = vec
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def transform_overlay_xy(
    x: float,
    y: float,
    dx: float,
    dy: float,
    yaw: float,
    scale: float,
) -> tuple[float, float]:
    x *= scale
    y *= scale
    c = math.cos(yaw)
    s = math.sin(yaw)
    return x * c - y * s + dx, x * s + y * c + dy


def classify_semantic(kind: str, x: float, y: float, z: float, sx: float, sy: float, sz: float, horizontal: bool) -> tuple[str, tuple[int, int, int]]:
    if kind == "hfield":
        return "rough_hfield", COLOR_ROUGH
    if kind == "cylinder" and horizontal:
        return "low_bar", COLOR_LOW_BAR
    if kind == "cylinder" and y < -9.5 and sx <= 0.08:
        return "slalom_pole", COLOR_CYAN
    if y < -6.0 and min(sx, sy) <= 0.06 and sz >= 0.12:
        return "wall", COLOR_WALL
    if -5.1 <= y <= -2.0 and 1.2 <= x <= 3.4:
        return "stairs", COLOR_STAIRS
    if -3.8 <= y <= 0.6 and 1.2 <= x <= 6.3:
        return "bridge", COLOR_GREEN
    if -6.2 <= y <= -4.2 and 4.0 <= x <= 6.5:
        return "ramp", COLOR_RAMP
    if y < -11.0 and sx >= 0.35 and sy >= 0.35 and z < 0.2:
        return "rough_pit", COLOR_ROUGH
    return "structure", COLOR_STRUCTURE


def overlay_color(label: str) -> tuple[int, int, int]:
    table = {
        "wall": COLOR_WALL,
        "low_bar": COLOR_LOW_BAR,
        "stairs": COLOR_STAIRS,
        "stairs_test": COLOR_STAIRS,
        "ramp": COLOR_RAMP,
        "ramp_test": COLOR_RAMP,
        "bridge": COLOR_GREEN,
        "rough": COLOR_ROUGH,
        "rough_pit": COLOR_ROUGH,
        "rough_hfield": COLOR_ROUGH,
        "slalom_pole": COLOR_CYAN,
        "spawn": COLOR_ROSE,
    }
    return table.get(label, COLOR_STRUCTURE)


def load_mjcf_overlay(
    path: Path,
    dx: float,
    dy: float,
    yaw_offset: float,
    scale: float,
) -> list[SemanticElement]:
    if not path.exists():
        raise FileNotFoundError(path)
    root = ET.parse(path).getroot()
    hfields: dict[str, list[float]] = {}
    for hfield in root.iter("hfield"):
        name = hfield.attrib.get("name", "")
        size = parse_float_list(hfield.attrib.get("size", ""), [])
        if name and size:
            hfields[name] = size

    elements: list[SemanticElement] = []
    for index, geom in enumerate(root.iter("geom"), start=1):
        name = geom.attrib.get("name", f"geom_{index}")
        kind = geom.attrib.get("type", "sphere")
        if kind == "plane" or name == "floor":
            continue
        pos = parse_float_list(geom.attrib.get("pos", ""), [0.0, 0.0, 0.0])
        quat = parse_float_list(geom.attrib.get("quat", ""), [1.0, 0.0, 0.0, 0.0])
        if kind == "hfield":
            size = hfields.get(geom.attrib.get("hfield", ""), [1.0, 1.0, 0.0])
        else:
            size = parse_float_list(geom.attrib.get("size", ""), [0.0, 0.0, 0.0])

        x = pos[0] if len(pos) > 0 else 0.0
        y = pos[1] if len(pos) > 1 else 0.0
        z = pos[2] if len(pos) > 2 else 0.0
        sx = size[0] if len(size) > 0 else 0.0
        sy = size[1] if len(size) > 1 else sx
        sz = size[2] if len(size) > 2 else 0.0
        local_z = rotate_quat_vec(quat, (0.0, 0.0, 1.0))
        horizontal = math.hypot(local_z[0], local_z[1]) > 0.45
        label, color = classify_semantic(kind, x, y, z, sx, sy, sz, horizontal)
        tx, ty = transform_overlay_xy(x, y, dx, dy, yaw_offset, scale)

        if kind in {"box", "hfield"} and sx > 0.0 and sy > 0.0:
            elements.append(
                SemanticElement(
                    kind="rect",
                    label=label,
                    x=tx,
                    y=ty,
                    yaw=quat_to_yaw(quat) + yaw_offset,
                    hx=sx * scale,
                    hy=sy * scale,
                    color=color,
                    name=name,
                )
            )
        elif kind == "cylinder" and sx > 0.0:
            if horizontal and sy > 0.0:
                bar_yaw = math.atan2(local_z[1], local_z[0]) + yaw_offset
                elements.append(
                    SemanticElement(
                        kind="rect",
                        label=label,
                        x=tx,
                        y=ty,
                        yaw=bar_yaw,
                        hx=max(sy, sx) * scale,
                        hy=max(sx, 0.025) * scale,
                        color=color,
                        name=name,
                    )
                )
            else:
                elements.append(
                    SemanticElement(
                        kind="circle",
                        label=label,
                        x=tx,
                        y=ty,
                        yaw=0.0,
                        radius=sx * scale,
                        color=color,
                        name=name,
                    )
                )
    return elements


def load_json_overlay(
    path: Path,
    dx: float,
    dy: float,
    yaw_offset: float,
    scale: float,
) -> list[SemanticElement]:
    if not path.exists():
        raise FileNotFoundError(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    raw_elements = data.get("elements", data if isinstance(data, list) else [])
    elements: list[SemanticElement] = []
    for index, item in enumerate(raw_elements, start=1):
        if not isinstance(item, dict):
            continue
        kind = str(item.get("type", item.get("kind", "rect")))
        label = str(item.get("label", item.get("obstacle", kind)))
        x = float(item.get("x", 0.0))
        y = float(item.get("y", 0.0))
        tx, ty = transform_overlay_xy(x, y, dx, dy, yaw_offset, scale)
        yaw = math.radians(float(item.get("yawDeg", item.get("yaw_deg", 0.0)))) + yaw_offset
        size = item.get("size", [])
        hx = float(item.get("hx", item.get("length", 0.0))) * 0.5
        hy = float(item.get("hy", item.get("width", 0.0))) * 0.5
        if isinstance(size, list) and size:
            hx = float(size[0])
            hy = float(size[1] if len(size) > 1 else size[0])
        radius = float(item.get("radius", item.get("r", 0.0)))
        if kind in {"circle", "cylinder"}:
            elements.append(SemanticElement("circle", label, tx, ty, yaw, radius=radius * scale, color=overlay_color(label), name=str(item.get("name", f"obj_{index}"))))
        else:
            elements.append(SemanticElement("rect", label, tx, ty, yaw, hx=hx * scale, hy=hy * scale, color=overlay_color(label), name=str(item.get("name", f"obj_{index}"))))
    return elements


def rect_corners(cx: float, cy: float, hx: float, hy: float, yaw: float) -> list[tuple[float, float]]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    corners = []
    for lx, ly in [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]:
        corners.append((cx + lx * c - ly * s, cy + lx * s + ly * c))
    return corners


def parse_pcd_header(lines: list[str]) -> dict[str, Any]:
    header: dict[str, Any] = {}
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0].upper()
        values = parts[1:]
        if key in {"FIELDS", "TYPE"}:
            header[key] = values
        elif key in {"SIZE", "COUNT"}:
            header[key] = [int(v) for v in values]
        elif key in {"WIDTH", "HEIGHT", "POINTS"} and values:
            header[key] = int(values[0])
        elif key == "DATA" and values:
            header[key] = values[0].lower()
    return header


def pcd_struct_format(size: int, kind: str) -> str:
    kind = kind.upper()
    if kind == "F" and size == 4:
        return "f"
    if kind == "F" and size == 8:
        return "d"
    if kind == "I" and size == 1:
        return "b"
    if kind == "U" and size == 1:
        return "B"
    if kind == "I" and size == 2:
        return "h"
    if kind == "U" and size == 2:
        return "H"
    if kind == "I" and size == 4:
        return "i"
    if kind == "U" and size == 4:
        return "I"
    if kind == "I" and size == 8:
        return "q"
    if kind == "U" and size == 8:
        return "Q"
    raise RuntimeError(f"Unsupported PCD field type: TYPE={kind} SIZE={size}")


def load_pcd_points(
    path: Path,
    z_min: float,
    z_max: float,
    sample_step: int,
    max_points: int,
) -> list[PointXYZ]:
    if not path.exists():
        raise FileNotFoundError(path)
    sample_step = max(1, int(sample_step))
    max_points = max(1, int(max_points))

    with path.open("rb") as f:
        header_lines: list[str] = []
        data_offset = 0
        while True:
            raw = f.readline()
            if not raw:
                raise RuntimeError("PCD header has no DATA line")
            data_offset = f.tell()
            line = raw.decode("utf-8", errors="replace")
            header_lines.append(line)
            if line.strip().upper().startswith("DATA"):
                break
        payload = f.read()

    header = parse_pcd_header(header_lines)
    fields = header.get("FIELDS", ["x", "y", "z"])
    sizes = header.get("SIZE", [4] * len(fields))
    types = header.get("TYPE", ["F"] * len(fields))
    counts = header.get("COUNT", [1] * len(fields))
    data_kind = header.get("DATA", "ascii")
    point_count = int(header.get("POINTS", header.get("WIDTH", 0) * header.get("HEIGHT", 1)))

    if not {"x", "y", "z"}.issubset(set(fields)):
        raise RuntimeError(f"PCD needs x/y/z fields, got: {fields}")

    points: list[PointXYZ] = []
    accepted = 0

    if data_kind == "ascii":
        text = payload.decode("utf-8", errors="replace")
        x_index = fields.index("x")
        y_index = fields.index("y")
        z_index = fields.index("z")
        need = max(x_index, y_index, z_index)
        for line in text.splitlines():
            parts = line.split()
            if len(parts) <= need:
                continue
            try:
                x = float(parts[x_index])
                y = float(parts[y_index])
                z = float(parts[z_index])
            except ValueError:
                continue
            if z_min <= z <= z_max:
                accepted += 1
                if accepted % sample_step == 0:
                    points.append(PointXYZ(x, y, z))
                    if len(points) >= max_points:
                        break
        return points

    if data_kind != "binary":
        raise RuntimeError(f"Only ASCII and binary PCD are supported, got DATA {data_kind}")

    scalar_names: list[str] = []
    fmt_parts: list[str] = []
    for field, size, kind, count in zip(fields, sizes, types, counts):
        for count_index in range(count):
            scalar_names.append(field if count == 1 else f"{field}_{count_index}")
            fmt_parts.append(pcd_struct_format(size, kind))
    fmt = "<" + "".join(fmt_parts)
    step_bytes = struct.calcsize(fmt)
    x_index = scalar_names.index("x")
    y_index = scalar_names.index("y")
    z_index = scalar_names.index("z")
    total = point_count if point_count > 0 else len(payload) // step_bytes
    total = min(total, len(payload) // step_bytes)

    for index in range(total):
        row = struct.unpack_from(fmt, payload, index * step_bytes)
        x = float(row[x_index])
        y = float(row[y_index])
        z = float(row[z_index])
        if z_min <= z <= z_max:
            accepted += 1
            if accepted % sample_step == 0:
                points.append(PointXYZ(x, y, z))
                if len(points) >= max_points:
                    break
    return points


def bool_from_route_value(value: Any) -> Optional[bool]:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    return None


def optional_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def infer_map_name(pcd_path: Path) -> str:
    return pcd_path.stem or "map"


def route_meta_from_args(args: argparse.Namespace, pcd_path: Path) -> RouteMeta:
    require_default = None
    if args.route_require_yaw_default == "true":
        require_default = True
    elif args.route_require_yaw_default == "false":
        require_default = False
    pre_dock_distance = (
        float(args.route_pre_dock_distance_default)
        if float(args.route_pre_dock_distance_default) >= 0.0 else None
    )
    pre_dock_tolerance = (
        float(args.route_pre_dock_tolerance_default)
        if float(args.route_pre_dock_tolerance_default) >= 0.0 else None
    )
    name = str(args.route_name or f"{infer_map_name(pcd_path)}_route")
    return RouteMeta(
        name=name,
        map_name=str(args.route_map or infer_map_name(pcd_path)),
        frame_id=str(args.map_frame or "map"),
        segment_name=str(args.route_segment or "segment_1"),
        segment_obstacle=str(args.route_obstacle or "flat"),
        yaw_tolerance_default=float(args.route_yaw_tol_default),
        require_yaw_default=require_default,
        pre_dock_distance_default=pre_dock_distance,
        pre_dock_tolerance_default=pre_dock_tolerance,
    )


def load_route_file(path: Path, fallback_meta: RouteMeta) -> tuple[RouteMeta, list[Waypoint]]:
    if not path.exists():
        return fallback_meta, []
    data = json.loads(path.read_text(encoding="utf-8"))
    meta = RouteMeta(
        name=str(data.get("name", fallback_meta.name) or fallback_meta.name),
        map_name=str(data.get("map", fallback_meta.map_name) or fallback_meta.map_name),
        frame_id=str(data.get("frame_id", fallback_meta.frame_id) or fallback_meta.frame_id),
        segment_name=fallback_meta.segment_name,
        segment_obstacle=fallback_meta.segment_obstacle,
        yaw_tolerance_default=optional_float(
            data.get("yawToleranceDegDefault", data.get("yaw_tolerance_deg_default"))
        ),
        require_yaw_default=bool_from_route_value(
            data.get("requireYawDefault", data.get("require_yaw_default"))
        ),
        pre_dock_distance_default=optional_float(
            data.get("preDockDistanceDefault", data.get("pre_dock_distance_default"))
        ),
        pre_dock_tolerance_default=optional_float(
            data.get("preDockToleranceDefault", data.get("pre_dock_tolerance_default"))
        ),
        created_at=str(data.get("createdAt", data.get("created_at", "")) or ""),
    )
    raw_points: list[Any] = []
    if isinstance(data.get("segments"), list):
        for segment in data["segments"]:
            if isinstance(segment, dict) and isinstance(segment.get("waypoints"), list):
                meta.segment_name = str(segment.get("name", meta.segment_name) or meta.segment_name)
                meta.segment_obstacle = str(segment.get("obstacle", meta.segment_obstacle) or meta.segment_obstacle)
                raw_points.extend(segment["waypoints"])
    if not raw_points and isinstance(data.get("waypoints"), list):
        raw_points = data["waypoints"]

    waypoints: list[Waypoint] = []
    for index, item in enumerate(raw_points, start=1):
        if not isinstance(item, dict):
            continue
        try:
            x = float(item["x"])
            y = float(item["y"])
        except (KeyError, TypeError, ValueError):
            continue
        yaw_value = item.get("yawDeg", item.get("yaw_deg"))
        yaw_deg = None
        if yaw_value is not None:
            try:
                yaw_deg = float(yaw_value)
            except (TypeError, ValueError):
                yaw_deg = None
        point_id = item.get("id", index)
        try:
            point_id = int(point_id)
        except (TypeError, ValueError):
            point_id = index
        waypoints.append(
            Waypoint(
                x=x,
                y=y,
                id=point_id,
                yaw_deg=yaw_deg,
                speed=float(item.get("speed", 0.35)),
                policy=str(item.get("policy", "rough")),
                tolerance=float(item.get("tolerance", 0.15)),
                yaw_tolerance_deg=optional_float(item.get("yawToleranceDeg", item.get("yaw_tolerance_deg"))),
                require_yaw=bool_from_route_value(item.get("requireYaw", item.get("require_yaw"))),
                pre_dock_distance=optional_float(item.get("preDockDistance", item.get("pre_dock_distance"))),
                pre_dock_tolerance=optional_float(item.get("preDockTolerance", item.get("pre_dock_tolerance"))),
                obstacle=str(item.get("obstacle", "flat") or "flat"),
                obstacle_name=str(item.get("obstacleName", item.get("obstacle_name", "")) or ""),
                name=f"WP{point_id}",
            )
        )
    return meta, waypoints


def waypoint_to_json(wp: Waypoint, fallback_id: int) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "id": int(wp.id if wp.id > 0 else fallback_id),
        "x": round(float(wp.x), 4),
        "y": round(float(wp.y), 4),
        "yawDeg": round(float(wp.yaw_deg if wp.yaw_deg is not None else 0.0), 2),
        "speed": round(float(wp.speed), 3),
        "policy": str(wp.policy or "rough"),
        "tolerance": round(max(0.01, float(wp.tolerance)), 3),
    }
    if wp.yaw_tolerance_deg is not None:
        payload["yawToleranceDeg"] = round(float(wp.yaw_tolerance_deg), 2)
    if wp.require_yaw is not None:
        payload["requireYaw"] = bool(wp.require_yaw)
    if wp.pre_dock_distance is not None:
        payload["preDockDistance"] = round(float(wp.pre_dock_distance), 3)
    if wp.pre_dock_tolerance is not None:
        payload["preDockTolerance"] = round(float(wp.pre_dock_tolerance), 3)
    if wp.obstacle and wp.obstacle != "flat":
        payload["obstacle"] = str(wp.obstacle)
    if wp.obstacle_name:
        payload["obstacleName"] = str(wp.obstacle_name)
    return payload


def save_waypoints(path: Path, meta: RouteMeta, waypoints: list[Waypoint]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "name": meta.name or path.stem,
        "map": meta.map_name,
        "frame_id": meta.frame_id or "map",
        "createdAt": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "segments": [
            {
                "name": meta.segment_name or "segment_1",
                "obstacle": meta.segment_obstacle or "flat",
                "waypoints": [waypoint_to_json(wp, index) for index, wp in enumerate(waypoints, start=1)],
            }
        ],
    }
    if meta.yaw_tolerance_default is not None:
        payload["yawToleranceDegDefault"] = float(meta.yaw_tolerance_default)
    if meta.require_yaw_default is not None:
        payload["requireYawDefault"] = bool(meta.require_yaw_default)
    if meta.pre_dock_distance_default is not None:
        payload["preDockDistanceDefault"] = float(meta.pre_dock_distance_default)
    if meta.pre_dock_tolerance_default is not None:
        payload["preDockToleranceDefault"] = float(meta.pre_dock_tolerance_default)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


class PoseFileSource:
    def __init__(self, path: str) -> None:
        self.path = Path(path) if path else None
        self.last_mtime = 0.0

    def update(self, pose: Pose2D) -> Pose2D:
        if self.path is None or not self.path.exists():
            return pose
        mtime = self.path.stat().st_mtime
        if mtime == self.last_mtime:
            return pose
        self.last_mtime = mtime
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            x = float(data.get("x", pose.x))
            y = float(data.get("y", pose.y))
            if "yaw_deg" in data:
                yaw = math.radians(float(data["yaw_deg"]))
            else:
                yaw = float(data.get("yaw", pose.yaw))
            return Pose2D(x, y, yaw)
        except Exception:
            return pose


class UdpPoseSource:
    def __init__(self, port: int) -> None:
        self.sock: Optional[socket.socket] = None
        if port > 0:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)
            self.sock.bind(("0.0.0.0", port))

    def update(self, pose: Pose2D) -> Pose2D:
        if self.sock is None:
            return pose
        latest = pose
        while True:
            try:
                data, _addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            text = data.decode("utf-8", errors="replace").strip()
            try:
                if text.startswith("{"):
                    item = json.loads(text)
                    x = float(item.get("x", latest.x))
                    y = float(item.get("y", latest.y))
                    yaw = math.radians(float(item["yaw_deg"])) if "yaw_deg" in item else float(item.get("yaw", latest.yaw))
                    latest = Pose2D(x, y, yaw)
                else:
                    parts = text.replace(",", " ").split()
                    if len(parts) >= 3:
                        latest = Pose2D(float(parts[0]), float(parts[1]), float(parts[2]))
            except Exception:
                continue
        return latest


class Ros2PoseSource:
    def __init__(self, enabled: bool, map_frame: str, base_frame: str, goal_topic: str) -> None:
        self.enabled = False
        self.node = None
        self.rclpy = None
        self.buffer = None
        self.publisher = None
        self.String = None
        self.map_frame = map_frame
        self.base_frame = base_frame
        if not enabled and not goal_topic:
            return
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.time import Time
            from std_msgs.msg import String
            from tf2_ros import Buffer, TransformException, TransformListener
        except Exception as exc:
            print(f"[ros2] unavailable: {exc}")
            return

        if not rclpy.ok():
            rclpy.init(args=None)
        node = Node("pcd_nav_pygame")
        self.rclpy = rclpy
        self.node = node
        self.Time = Time
        self.String = String
        self.TransformException = TransformException
        if enabled:
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, node)
            self.enabled = True
        if goal_topic:
            self.publisher = node.create_publisher(String, goal_topic, 10)

    def update(self, pose: Pose2D) -> Pose2D:
        if self.node is None or self.rclpy is None:
            return pose
        self.rclpy.spin_once(self.node, timeout_sec=0.0)
        if not self.enabled or self.buffer is None:
            return pose
        try:
            transform = self.buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                self.Time(),
            )
        except Exception:
            return pose
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return Pose2D(float(t.x), float(t.y), yaw)

    def publish_goal(self, x: float, y: float) -> None:
        if self.publisher is None or self.String is None:
            return
        msg = self.String()
        msg.data = f"go {x:.3f} {y:.3f}"
        self.publisher.publish(msg)

    def close(self) -> None:
        if self.node is not None:
            self.node.destroy_node()


class Viewer:
    def __init__(
        self,
        args: argparse.Namespace,
        points: list[PointXYZ],
        waypoints: list[Waypoint],
        semantic_elements: list[SemanticElement],
        route_meta: RouteMeta,
        save_route_path: Path,
    ) -> None:
        self.args = args
        self.points = points
        self.waypoints = waypoints
        self.semantic_elements = semantic_elements
        self.route_meta = route_meta
        self.save_route_path = save_route_path
        self.show_overlay = not bool(args.hide_overlay)
        self.selected_wp_index: Optional[int] = 0 if waypoints else None
        self.command_mode = False
        self.command_text = ""
        self.message = "Shift+click add waypoint | Enter command | P save"
        self.target: Optional[tuple[float, float]] = None
        self.pose = Pose2D(
            float(args.initial_x),
            float(args.initial_y),
            math.radians(float(args.initial_yaw_deg)),
        )
        self.trail: list[tuple[float, float]] = []
        self.auto_center = False
        self.zoom = float(args.zoom)
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.dragging = False
        self.drag_start = (0, 0)
        self.drag_pan_start = (0.0, 0.0)
        self.pose_file = PoseFileSource(args.pose_file)
        self.udp_pose = UdpPoseSource(int(args.udp_port))
        self.ros2_pose = Ros2PoseSource(
            bool(args.ros2_tf),
            str(args.map_frame),
            str(args.base_frame),
            str(args.ros2_goal_topic),
        )

        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((args.window_width, args.window_height))
        pygame.display.set_caption(args.title)
        self.clock = pygame.time.Clock()
        self.font_large = pygame.font.SysFont("consolas", 20, bold=True) or pygame.font.Font(None, 22)
        self.font = pygame.font.SysFont("consolas", 15) or pygame.font.Font(None, 16)
        self.font_small = pygame.font.SysFont("consolas", 13) or pygame.font.Font(None, 14)

        self.z_min = min((p.z for p in points), default=float(args.floor_z_min))
        self.z_max = max((p.z for p in points), default=float(args.floor_z_max))
        if math.isclose(self.z_min, self.z_max):
            self.z_max = self.z_min + 1.0

        self.center_on_points()

    @property
    def map_width(self) -> int:
        return min(int(self.args.map_width), int(self.args.window_width) - 260)

    def center_on_points(self) -> None:
        if not self.points:
            return
        min_x = min(p.x for p in self.points)
        max_x = max(p.x for p in self.points)
        min_y = min(p.y for p in self.points)
        max_y = max(p.y for p in self.points)
        center_x = (min_x + max_x) * 0.5
        center_y = (min_y + max_y) * 0.5
        span_x = max(1.0, max_x - min_x)
        span_y = max(1.0, max_y - min_y)
        fit_zoom = min(
            (self.map_width - 80) / span_x,
            (self.args.window_height - 80) / span_y,
        )
        self.zoom = min(max(8.0, fit_zoom), max(self.zoom, 8.0))
        self.pan_x = -center_x * self.zoom
        self.pan_y = center_y * self.zoom

    def world_to_screen(self, x: float, y: float) -> tuple[int, int]:
        sx = int(self.map_width / 2.0 + x * self.zoom + self.pan_x)
        sy = int(self.args.window_height / 2.0 - y * self.zoom + self.pan_y)
        return sx, sy

    def screen_to_world(self, sx: int, sy: int) -> tuple[float, float]:
        x = (sx - self.map_width / 2.0 - self.pan_x) / self.zoom
        y = -(sy - self.args.window_height / 2.0 - self.pan_y) / self.zoom
        return x, y

    def selected_waypoint(self) -> Optional[Waypoint]:
        if self.selected_wp_index is None:
            return None
        if 0 <= self.selected_wp_index < len(self.waypoints):
            return self.waypoints[self.selected_wp_index]
        self.selected_wp_index = None
        return None

    def normalize_waypoint_ids(self) -> None:
        used: set[int] = set()
        for index, wp in enumerate(self.waypoints, start=1):
            if wp.id <= 0 or wp.id in used:
                wp.id = index
            used.add(wp.id)
            wp.name = f"WP{wp.id}"

    def add_waypoint(self, x: float, y: float, yaw_deg: Optional[float] = None) -> None:
        point_id = max([wp.id for wp in self.waypoints], default=0) + 1
        policy = str(self.args.default_policy)
        if policy == "none":
            policy = "rough"
        wp = Waypoint(
            x=x,
            y=y,
            id=point_id,
            yaw_deg=yaw_deg if yaw_deg is not None else math.degrees(self.pose.yaw),
            speed=max(0.0, float(self.args.default_speed)),
            policy=policy,
            tolerance=max(0.01, float(self.args.default_tolerance)),
            name=f"WP{point_id}",
        )
        self.waypoints.append(wp)
        self.selected_wp_index = len(self.waypoints) - 1
        self.message = f"Added WP{wp.id} x={wp.x:.3f} y={wp.y:.3f}"

    def select_nearest_waypoint(self, sx: int, sy: int, max_px: float = 14.0) -> bool:
        best_index: Optional[int] = None
        best_dist = max_px
        for index, wp in enumerate(self.waypoints):
            wx, wy = self.world_to_screen(wp.x, wp.y)
            dist = math.hypot(wx - sx, wy - sy)
            if dist <= best_dist:
                best_dist = dist
                best_index = index
        if best_index is None:
            return False
        self.selected_wp_index = best_index
        wp = self.waypoints[best_index]
        self.message = f"Selected WP{wp.id}"
        return True

    def handle_click_goal(self, sx: int, sy: int, add_waypoint: bool) -> None:
        x, y = self.screen_to_world(sx, sy)
        self.target = (x, y)
        if add_waypoint:
            self.add_waypoint(x, y)
        self.ros2_pose.publish_goal(x, y)

    def update_pose_sources(self) -> None:
        self.pose = self.pose_file.update(self.pose)
        self.pose = self.udp_pose.update(self.pose)
        self.pose = self.ros2_pose.update(self.pose)
        if not self.trail or math.hypot(self.pose.x - self.trail[-1][0], self.pose.y - self.trail[-1][1]) > 0.04:
            self.trail.append((self.pose.x, self.pose.y))
            if len(self.trail) > 1200:
                self.trail.pop(0)
        if self.auto_center:
            self.pan_x = -self.pose.x * self.zoom
            self.pan_y = self.pose.y * self.zoom

    def update_manual_pose(self, dt: float) -> None:
        if self.command_mode:
            return
        keys = pygame.key.get_pressed()
        move = 0.8 * dt
        turn = 1.7 * dt
        if keys[pygame.K_q]:
            self.pose.yaw += turn
        if keys[pygame.K_e]:
            self.pose.yaw -= turn
        forward = 0.0
        lateral = 0.0
        if keys[pygame.K_w]:
            forward += move
        if keys[pygame.K_s]:
            forward -= move
        if keys[pygame.K_a]:
            lateral += move
        if keys[pygame.K_d]:
            lateral -= move
        if forward or lateral:
            c = math.cos(self.pose.yaw)
            s = math.sin(self.pose.yaw)
            self.pose.x += forward * c - lateral * s
            self.pose.y += forward * s + lateral * c

    def save_route(self) -> None:
        self.normalize_waypoint_ids()
        save_waypoints(self.save_route_path, self.route_meta, self.waypoints)
        self.message = f"Saved {len(self.waypoints)} waypoints: {self.save_route_path}"
        print(self.message)

    def set_selected_field(self, key: str, value: str) -> None:
        wp = self.selected_waypoint()
        if wp is None:
            self.message = "No waypoint selected"
            return
        key = key.lower()
        if key in {"x", "y", "yaw", "yawdeg", "speed", "tol", "tolerance", "yawtol", "yawtolerance"}:
            number = float(value)
            if key == "x":
                wp.x = number
            elif key == "y":
                wp.y = number
            elif key in {"yaw", "yawdeg"}:
                wp.yaw_deg = number
            elif key == "speed":
                wp.speed = max(0.0, number)
            elif key in {"tol", "tolerance"}:
                wp.tolerance = max(0.01, number)
            else:
                wp.yaw_tolerance_deg = max(0.0, number)
        elif key == "id":
            wp.id = max(1, int(value))
            wp.name = f"WP{wp.id}"
        elif key == "policy":
            wp.policy = value
        elif key in {"requireyaw", "require_yaw"}:
            parsed = bool_from_route_value(value)
            wp.require_yaw = parsed
        elif key in {"predock", "pre_dock", "predockdistance"}:
            wp.pre_dock_distance = optional_float(value)
        elif key in {"predocktol", "pre_dock_tolerance"}:
            wp.pre_dock_tolerance = optional_float(value)
        elif key == "obstacle":
            wp.obstacle = value
        elif key in {"obstaclename", "obstacle_name"}:
            wp.obstacle_name = value
        else:
            raise ValueError(f"unknown field: {key}")
        self.message = f"Updated WP{wp.id}: {key}={value}"

    def run_command(self, text: str) -> None:
        parts = text.strip().split()
        if not parts:
            return
        cmd = parts[0].lower()
        try:
            if cmd == "set" and len(parts) >= 3:
                self.set_selected_field(parts[1], " ".join(parts[2:]))
            elif cmd == "move" and len(parts) >= 3:
                wp = self.selected_waypoint()
                if wp is None:
                    self.message = "No waypoint selected"
                else:
                    wp.x += float(parts[1])
                    wp.y += float(parts[2])
                    self.message = f"Moved WP{wp.id}"
            elif cmd == "route" and len(parts) >= 3:
                key = parts[1].lower()
                value = " ".join(parts[2:])
                if key == "name":
                    self.route_meta.name = value
                elif key == "map":
                    self.route_meta.map_name = value
                elif key == "segment":
                    self.route_meta.segment_name = value
                elif key == "obstacle":
                    self.route_meta.segment_obstacle = value
                elif key in {"yawtol", "yaw_tolerance"}:
                    self.route_meta.yaw_tolerance_default = float(value)
                else:
                    raise ValueError(f"unknown route field: {key}")
                self.message = f"Route {key}={value}"
            elif cmd == "select" and len(parts) >= 2:
                wanted = int(parts[1])
                for index, wp in enumerate(self.waypoints):
                    if wp.id == wanted:
                        self.selected_wp_index = index
                        self.message = f"Selected WP{wp.id}"
                        break
                else:
                    self.message = f"WP id not found: {wanted}"
            elif cmd in {"save", "export"}:
                self.save_route()
            elif cmd == "delete":
                self.delete_selected_waypoint()
            else:
                self.message = "Commands: set x/y/yaw/speed/tol/policy/id VALUE | route name/map VALUE | save"
        except Exception as exc:
            self.message = f"Command error: {exc}"

    def delete_selected_waypoint(self) -> None:
        if self.selected_wp_index is None or not (0 <= self.selected_wp_index < len(self.waypoints)):
            self.message = "No waypoint selected"
            return
        removed = self.waypoints.pop(self.selected_wp_index)
        if not self.waypoints:
            self.selected_wp_index = None
        else:
            self.selected_wp_index = min(self.selected_wp_index, len(self.waypoints) - 1)
        self.message = f"Deleted WP{removed.id}"

    def adjust_selected(self, dx: float = 0.0, dy: float = 0.0, dyaw: float = 0.0, dspeed: float = 0.0, dtol: float = 0.0) -> None:
        wp = self.selected_waypoint()
        if wp is None:
            return
        wp.x += dx
        wp.y += dy
        wp.yaw_deg = (wp.yaw_deg if wp.yaw_deg is not None else 0.0) + dyaw
        wp.speed = max(0.0, wp.speed + dspeed)
        wp.tolerance = max(0.01, wp.tolerance + dtol)
        self.message = f"WP{wp.id}: x={wp.x:.3f} y={wp.y:.3f} yaw={wp.yaw_deg:.1f} speed={wp.speed:.2f} tol={wp.tolerance:.2f}"

    def draw_grid(self) -> None:
        grid_min_x, grid_min_y = self.screen_to_world(0, self.args.window_height)
        grid_max_x, grid_max_y = self.screen_to_world(self.map_width, 0)
        start_x = math.floor(grid_min_x)
        end_x = math.ceil(grid_max_x)
        start_y = math.floor(grid_min_y)
        end_y = math.ceil(grid_max_y)
        for gx in range(start_x, end_x + 1):
            sx, _ = self.world_to_screen(gx, 0.0)
            color = COLOR_AXIS if gx == 0 else COLOR_GRID
            width = 2 if gx == 0 else 1
            pygame.draw.line(self.screen, color, (sx, 0), (sx, self.args.window_height), width)
            if gx % 2 == 0 and 0 < sx < self.map_width - 25:
                self.screen.blit(self.font_small.render(str(gx), True, COLOR_MUTED), (sx + 3, self.args.window_height - 42))
        for gy in range(start_y, end_y + 1):
            _, sy = self.world_to_screen(0.0, gy)
            color = COLOR_AXIS if gy == 0 else COLOR_GRID
            width = 2 if gy == 0 else 1
            pygame.draw.line(self.screen, color, (0, sy), (self.map_width, sy), width)
            if gy % 2 == 0 and 20 < sy < self.args.window_height - 28:
                self.screen.blit(self.font_small.render(str(gy), True, COLOR_MUTED), (8, sy + 2))

    def draw_points(self) -> None:
        point_size = max(1, int(self.args.point_size))
        surface = self.screen
        for p in self.points:
            sx, sy = self.world_to_screen(p.x, p.y)
            if sx < 0 or sy < 0 or sx >= self.map_width or sy >= self.args.window_height:
                continue
            color = z_color(p.z, self.z_min, self.z_max)
            if point_size == 1:
                surface.set_at((sx, sy), color)
            else:
                pygame.draw.circle(surface, color, (sx, sy), point_size)

    def draw_semantic_overlay(self) -> None:
        if not self.show_overlay or not self.semantic_elements:
            return
        layer = pygame.Surface((self.map_width, self.args.window_height), pygame.SRCALPHA)
        for element in self.semantic_elements:
            color = element.color
            fill = (color[0], color[1], color[2], 58)
            border = (color[0], color[1], color[2], 220)
            label_pos: tuple[int, int]
            if element.kind == "circle":
                sx, sy = self.world_to_screen(element.x, element.y)
                radius = max(3, int(abs(element.radius) * self.zoom))
                pygame.draw.circle(layer, fill, (sx, sy), radius)
                pygame.draw.circle(layer, border, (sx, sy), radius, width=2)
                label_pos = (sx + radius + 4, sy - 7)
            else:
                corners = rect_corners(element.x, element.y, element.hx, element.hy, element.yaw)
                screen_corners = [self.world_to_screen(x, y) for x, y in corners]
                if len(screen_corners) >= 3:
                    pygame.draw.polygon(layer, fill, screen_corners)
                    pygame.draw.polygon(layer, border, screen_corners, width=2)
                label_pos = self.world_to_screen(element.x, element.y)
                label_pos = (label_pos[0] + 6, label_pos[1] - 8)
            name = element.label if not element.name else f"{element.label}:{element.name}"
            label = self.font_small.render(name, True, color)
            layer.blit(label, label_pos)
        self.screen.blit(layer, (0, 0))

    def draw_waypoints(self) -> None:
        if len(self.waypoints) > 1:
            pts = [self.world_to_screen(wp.x, wp.y) for wp in self.waypoints]
            pygame.draw.lines(self.screen, COLOR_GREEN, False, pts, 2)
        for index, wp in enumerate(self.waypoints, start=1):
            sx, sy = self.world_to_screen(wp.x, wp.y)
            selected = self.selected_wp_index == index - 1
            color = COLOR_GOLD if selected else COLOR_PURPLE
            radius = 10 if selected else 7
            pygame.draw.circle(self.screen, color, (sx, sy), radius)
            pygame.draw.circle(self.screen, COLOR_TEXT, (sx, sy), radius, width=2 if selected else 1)
            label = wp.name or f"WP{wp.id if wp.id > 0 else index}"
            self.screen.blit(self.font_small.render(label, True, COLOR_TEXT), (sx + 9, sy - 16))
            if wp.yaw_deg is not None:
                yaw = math.radians(wp.yaw_deg)
                ex, ey = self.world_to_screen(wp.x + 0.35 * math.cos(yaw), wp.y + 0.35 * math.sin(yaw))
                pygame.draw.line(self.screen, color, (sx, sy), (ex, ey), 3 if selected else 2)

    def draw_robot(self) -> None:
        if len(self.trail) > 1:
            pts = [self.world_to_screen(x, y) for x, y in self.trail]
            pygame.draw.lines(self.screen, COLOR_CYAN, False, pts, 2)
        sx, sy = self.world_to_screen(self.pose.x, self.pose.y)
        radius = max(6, int(0.18 * self.zoom))
        pygame.draw.circle(self.screen, COLOR_CYAN, (sx, sy), radius)
        pygame.draw.circle(self.screen, COLOR_TEXT, (sx, sy), radius, width=2)
        nx, ny = self.world_to_screen(
            self.pose.x + 0.42 * math.cos(self.pose.yaw),
            self.pose.y + 0.42 * math.sin(self.pose.yaw),
        )
        pygame.draw.line(self.screen, COLOR_GREEN, (sx, sy), (nx, ny), 3)
        pygame.draw.circle(self.screen, COLOR_GREEN, (nx, ny), 4)

    def draw_target(self) -> None:
        if self.target is None:
            return
        x, y = self.target
        sx, sy = self.world_to_screen(x, y)
        pulse = int(9 + 4 * math.sin(time.time() * 7.0))
        pygame.draw.circle(self.screen, COLOR_GOLD, (sx, sy), pulse, width=2)
        pygame.draw.circle(self.screen, COLOR_GOLD, (sx, sy), 3)
        pygame.draw.line(self.screen, COLOR_GOLD, (sx - pulse - 5, sy), (sx + pulse + 5, sy), 1)
        pygame.draw.line(self.screen, COLOR_GOLD, (sx, sy - pulse - 5), (sx, sy + pulse + 5), 1)
        self.screen.blit(self.font.render(f"Target {x:.2f},{y:.2f}", True, COLOR_GOLD), (sx + 13, sy - 8))

    def draw_panel(self) -> None:
        x0 = self.map_width
        w = self.args.window_width - self.map_width
        pygame.draw.rect(self.screen, COLOR_PANEL, (x0, 0, w, self.args.window_height))
        pygame.draw.line(self.screen, COLOR_PANEL_BORDER, (x0, 0), (x0, self.args.window_height), 2)
        y = 18
        self.screen.blit(self.font_large.render("PCD NAV VIEW", True, COLOR_TEXT), (x0 + 18, y))
        y += 34
        lines = [
            f"route: {self.route_meta.name}",
            f"map: {self.route_meta.map_name}",
            f"points: {len(self.points)}",
            f"zoom: {self.zoom:.1f} px/m",
            f"pose: x={self.pose.x:.3f}",
            f"      y={self.pose.y:.3f}",
            f"yaw: {math.degrees(self.pose.yaw):.1f} deg",
            f"waypoints: {len(self.waypoints)}",
            f"overlay: {len(self.semantic_elements)} {'on' if self.show_overlay else 'off'}",
            f"auto-center: {'on' if self.auto_center else 'off'}",
        ]
        if self.target is not None:
            lines.append(f"goal: {self.target[0]:.3f}, {self.target[1]:.3f}")
        for line in lines:
            self.screen.blit(self.font.render(line, True, COLOR_TEXT), (x0 + 18, y))
            y += 22
        y += 12
        wp = self.selected_waypoint()
        self.screen.blit(self.font.render("Selected WP", True, COLOR_MUTED), (x0 + 18, y))
        y += 22
        if wp is None:
            selected_lines = ["none"]
        else:
            selected_lines = [
                f"id: {wp.id}",
                f"x: {wp.x:.4f}",
                f"y: {wp.y:.4f}",
                f"yaw: {(wp.yaw_deg if wp.yaw_deg is not None else 0.0):.2f}",
                f"speed: {wp.speed:.3f}",
                f"policy: {wp.policy}",
                f"tol: {wp.tolerance:.3f}",
                f"yawTol: {wp.yaw_tolerance_deg if wp.yaw_tolerance_deg is not None else '-'}",
                f"requireYaw: {wp.require_yaw if wp.require_yaw is not None else '-'}",
            ]
        for line in selected_lines:
            self.screen.blit(self.font_small.render(line, True, COLOR_TEXT), (x0 + 18, y))
            y += 17
        y += 8
        help_lines = [
            "Left click: select/goal",
            "Shift+click: add WP",
            "Enter: exact command",
            "Mouse wheel: zoom",
            "Right drag: pan",
            "F: fit map",
            "Space: center lock",
            "R: record robot WP",
            "O: toggle overlay",
            "P: save route",
            "Del: delete WP",
            "[]: select WP",
            "Y/U: yaw -/+5",
            "V/B: speed -/+0.05",
            "T/G: tol -/+0.01",
            "1/2: rough/crawl",
            "C: clear goal",
            "WASD/QE: manual pose",
            "Esc: quit",
        ]
        self.screen.blit(self.font.render("Controls", True, COLOR_MUTED), (x0 + 18, y))
        y += 24
        for line in help_lines:
            self.screen.blit(self.font_small.render(line, True, COLOR_MUTED), (x0 + 18, y))
            y += 19

    def draw_status_bar(self) -> None:
        pygame.draw.rect(self.screen, (7, 10, 19), (0, self.args.window_height - 25, self.map_width, 25))
        mx, my = pygame.mouse.get_pos()
        if self.command_mode:
            text = f": {self.command_text}"
        elif mx < self.map_width:
            wx, wy = self.screen_to_world(mx, my)
            text = f"cursor x={wx:.3f} y={wy:.3f} | {self.message}"
        else:
            text = self.message
        self.screen.blit(self.font_small.render(text, True, COLOR_MUTED), (10, self.args.window_height - 19))

    def handle_events(self) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if self.command_mode:
                    if event.key == pygame.K_ESCAPE:
                        self.command_mode = False
                        self.command_text = ""
                    elif event.key == pygame.K_RETURN:
                        command = self.command_text
                        self.command_mode = False
                        self.command_text = ""
                        self.run_command(command)
                    elif event.key == pygame.K_BACKSPACE:
                        self.command_text = self.command_text[:-1]
                    elif event.unicode:
                        self.command_text += event.unicode
                    continue
                if event.key == pygame.K_ESCAPE:
                    return False
                if event.key == pygame.K_RETURN:
                    self.command_mode = True
                    self.command_text = ""
                elif event.key == pygame.K_f:
                    self.center_on_points()
                elif event.key == pygame.K_SPACE:
                    self.auto_center = not self.auto_center
                elif event.key == pygame.K_c:
                    self.target = None
                elif event.key == pygame.K_o:
                    self.show_overlay = not self.show_overlay
                elif event.key == pygame.K_r:
                    self.add_waypoint(self.pose.x, self.pose.y, math.degrees(self.pose.yaw))
                elif event.key == pygame.K_p:
                    self.save_route()
                elif event.key == pygame.K_DELETE:
                    self.delete_selected_waypoint()
                elif event.key == pygame.K_LEFTBRACKET and self.waypoints:
                    self.selected_wp_index = (self.selected_wp_index or 0) - 1
                    self.selected_wp_index %= len(self.waypoints)
                elif event.key == pygame.K_RIGHTBRACKET and self.waypoints:
                    self.selected_wp_index = (self.selected_wp_index or 0) + 1
                    self.selected_wp_index %= len(self.waypoints)
                elif event.key == pygame.K_y:
                    self.adjust_selected(dyaw=-5.0)
                elif event.key == pygame.K_u:
                    self.adjust_selected(dyaw=5.0)
                elif event.key == pygame.K_v:
                    self.adjust_selected(dspeed=-0.05)
                elif event.key == pygame.K_b:
                    self.adjust_selected(dspeed=0.05)
                elif event.key == pygame.K_t:
                    self.adjust_selected(dtol=-0.01)
                elif event.key == pygame.K_g:
                    self.adjust_selected(dtol=0.01)
                elif event.key == pygame.K_1:
                    wp = self.selected_waypoint()
                    if wp is not None:
                        wp.policy = "rough"
                        self.message = f"WP{wp.id} policy=rough"
                elif event.key == pygame.K_2:
                    wp = self.selected_waypoint()
                    if wp is not None:
                        wp.policy = "crawl"
                        self.message = f"WP{wp.id} policy=crawl"
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1 and event.pos[0] < self.map_width:
                    mods = pygame.key.get_mods()
                    if bool(mods & pygame.KMOD_SHIFT):
                        self.handle_click_goal(event.pos[0], event.pos[1], True)
                    elif not self.select_nearest_waypoint(event.pos[0], event.pos[1]):
                        self.handle_click_goal(event.pos[0], event.pos[1], False)
                elif event.button in (2, 3):
                    self.dragging = True
                    self.drag_start = event.pos
                    self.drag_pan_start = (self.pan_x, self.pan_y)
                elif event.button in (4, 5) and event.pos[0] < self.map_width:
                    before = self.screen_to_world(event.pos[0], event.pos[1])
                    factor = 1.12 if event.button == 4 else 1.0 / 1.12
                    self.zoom = max(2.0, min(400.0, self.zoom * factor))
                    after = self.screen_to_world(event.pos[0], event.pos[1])
                    self.pan_x += (after[0] - before[0]) * self.zoom
                    self.pan_y -= (after[1] - before[1]) * self.zoom
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button in (2, 3):
                    self.dragging = False
            elif event.type == pygame.MOUSEMOTION and self.dragging:
                dx = event.pos[0] - self.drag_start[0]
                dy = event.pos[1] - self.drag_start[1]
                self.pan_x = self.drag_pan_start[0] + dx
                self.pan_y = self.drag_pan_start[1] + dy
        return True

    def run(self) -> None:
        running = True
        try:
            while running:
                dt = self.clock.tick(45) / 1000.0
                running = self.handle_events()
                self.update_manual_pose(dt)
                self.update_pose_sources()

                self.screen.fill(COLOR_BG)
                pygame.draw.rect(self.screen, COLOR_BG, (0, 0, self.map_width, self.args.window_height))
                self.draw_grid()
                self.draw_points()
                self.draw_semantic_overlay()
                self.draw_waypoints()
                self.draw_target()
                self.draw_robot()
                self.draw_panel()
                self.draw_status_bar()
                pygame.display.flip()
        finally:
            self.ros2_pose.close()
            pygame.quit()


def main() -> None:
    args = parse_args()
    pcd_path = Path(args.pcd)
    route_meta = route_meta_from_args(args, pcd_path)
    save_route_path = (
        Path(args.save_route)
        if args.save_route
        else Path("map") / "routes" / route_meta.map_name / f"{route_meta.name}.json"
    )
    print(f"Loading PCD: {pcd_path}")
    points = load_pcd_points(
        pcd_path,
        z_min=float(args.floor_z_min),
        z_max=float(args.floor_z_max),
        sample_step=int(args.sample_step),
        max_points=int(args.max_points),
    )
    if not points:
        raise RuntimeError("No points left after z filtering. Adjust --floor-z-min/--floor-z-max.")
    print(f"Loaded {len(points)} sampled PCD points.")
    if args.route:
        route_meta, waypoints = load_route_file(Path(args.route), route_meta)
    else:
        waypoints = []
    print(f"Route output: {save_route_path.resolve()}")
    overlay_yaw = math.radians(float(args.overlay_yaw_deg))
    semantic_elements: list[SemanticElement] = []
    if args.mjcf:
        mjcf_elements = load_mjcf_overlay(
            Path(args.mjcf),
            dx=float(args.overlay_dx),
            dy=float(args.overlay_dy),
            yaw_offset=overlay_yaw,
            scale=float(args.overlay_scale),
        )
        semantic_elements.extend(mjcf_elements)
        print(f"Loaded {len(mjcf_elements)} MJCF semantic elements.")
    if args.overlay_json:
        json_elements = load_json_overlay(
            Path(args.overlay_json),
            dx=float(args.overlay_dx),
            dy=float(args.overlay_dy),
            yaw_offset=overlay_yaw,
            scale=float(args.overlay_scale),
        )
        semantic_elements.extend(json_elements)
        print(f"Loaded {len(json_elements)} JSON semantic elements.")
    viewer = Viewer(args, points, waypoints, semantic_elements, route_meta, save_route_path)
    viewer.run()


if __name__ == "__main__":
    main()
