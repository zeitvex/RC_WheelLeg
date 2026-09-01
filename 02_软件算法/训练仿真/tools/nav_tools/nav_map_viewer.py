#!/usr/bin/env python3
"""
Standalone pygame navigation map tool.

Data owned by this tool lives beside this file:
  - xml/*.xml
  - pcd/*.pcd
  - points/*.json
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import pkgutil
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

if not hasattr(pkgutil, "ImpImporter"):
    pkgutil.ImpImporter = pkgutil.zipimporter  # type: ignore[attr-defined]

import pygame


TOOL_DIR = Path(__file__).resolve().parent
XML_DIR = TOOL_DIR / "xml"
PCD_DIR = TOOL_DIR / "pcd"
POINTS_DIR = TOOL_DIR / "points"
REGIONS_DIR = TOOL_DIR / "regions"
LOCAL_XML = XML_DIR / "scene_terrain_nav_tools.xml"
MIRROR_XML = XML_DIR / "scene_terrain_nav_tools_mirror.xml"
MIRROR_AXIS_X = 3.7

COLOR_BG = (10, 15, 30)
COLOR_GRID = (22, 29, 48)
COLOR_AXIS = (38, 50, 78)
COLOR_PANEL = (18, 24, 42)
COLOR_PANEL_DARK = (7, 10, 19)
COLOR_PANEL_BORDER = (38, 52, 84)
COLOR_TEXT = (248, 250, 252)
COLOR_MUTED = (148, 163, 184)
COLOR_CYAN = (6, 182, 212)
COLOR_GOLD = (234, 179, 8)
COLOR_GREEN = (16, 185, 129)
COLOR_PURPLE = (139, 92, 246)
COLOR_WARNING = (244, 63, 94)
COLOR_PCD = (92, 160, 255)

ROBOT_POSE_HIP = 0.550
ROBOT_POSE_KNEE = -1.125
ROBOT_BODY_LENGTH = 0.356
ROBOT_BODY_WIDTH = 0.235
ROBOT_BODY_CENTER_X = 0.1518
ROBOT_ORIGIN_FROM_FRONT = 0.105
ROBOT_BODY_CENTER_OFFSET_X = ROBOT_ORIGIN_FROM_FRONT - ROBOT_BODY_LENGTH * 0.5
ROBOT_WHEEL_VIS_LENGTH = 0.16
ROBOT_WHEEL_VIS_WIDTH = 0.055
ROBOT_WHEEL_RADIUS = 0.1
ROBOT_WHEEL_POSITIONS = {
    "fl": ((0.32826 + 0.06389) - ROBOT_BODY_CENTER_X, 0.066172 - 0.027344, 0.1035, 0.014699, 0.04074, 0.0),
    "fr": ((0.32826 + 0.06389) - ROBOT_BODY_CENTER_X, -0.065853 + 0.027311, -0.1035, -0.018447, -0.040735, -0.00075079),
    "rl": ((-0.024743 - 0.06389) - ROBOT_BODY_CENTER_X, 0.066141 - 0.027309, 0.099459, 0.012475, 0.040737, 0.0),
    "rr": ((-0.024743 - 0.06389) - ROBOT_BODY_CENTER_X, -0.065884 + 0.027341, -0.099408, -0.012435, -0.040737, -0.00075079),
}


@dataclass
class Geom:
    element: ET.Element
    xml_index: int
    group: str
    name: str
    kind: str
    pos: tuple[float, ...]
    size: tuple[float, ...]
    quat: tuple[float, float, float, float]
    rgba: tuple[float, float, float, float]
    collidable: bool


@dataclass
class PointCloud:
    path: Path | None = None
    points: list[tuple[float, float, float]] = field(default_factory=list)
    total_count: int = 0
    sampled_count: int = 0
    error: str = ""

    @property
    def label(self) -> str:
        return self.path.name if self.path else "None"


@dataclass
class Waypoint:
    x: float
    y: float
    world_x: float
    world_y: float
    origin_mode: str
    id: float | int = 0
    yawDeg: float = 0.0
    tolerance: float = 0.15
    speed: float = 0.35
    policy: str = "rough"
    task: str = "none"
    requireYaw: bool = False
    precisionFollow: bool = False
    stableCycles: int | None = None
    lookahead: float | None = None
    yawRateLimit: float | None = None
    slalomStraight: bool = False


@dataclass
class AvoidRegion:
    name: str
    points: list[tuple[float, float]]
    kind: str = "avoid"


@dataclass
class InputBox:
    rect: pygame.Rect
    key: str
    text: str


class CoordinateFrame:
    def __init__(self) -> None:
        self.mode = "world"
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0

    def toggle(self) -> None:
        self.mode = "custom" if self.mode == "world" else "world"

    def world_to_view(self, x: float, y: float) -> tuple[float, float]:
        if self.mode == "world":
            return x, y
        dx = x - self.origin_x
        dy = y - self.origin_y
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        return dx * c + dy * s, -dx * s + dy * c

    def view_to_world(self, x: float, y: float) -> tuple[float, float]:
        if self.mode == "world":
            return x, y
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        return self.origin_x + x * c - y * s, self.origin_y + x * s + y * c


class MapCamera:
    def __init__(self, width: int, height: int, panel_width: int = 460, zoom: float = 55.0) -> None:
        self.width = width
        self.height = height
        self.panel_width = panel_width
        self.zoom = zoom
        self.pan_x = 0.0
        self.pan_y = 0.0

    @property
    def map_width(self) -> int:
        return max(200, self.width - self.panel_width)

    def world_to_screen(self, x: float, y: float, frame: CoordinateFrame) -> tuple[int, int]:
        vx, vy = frame.world_to_view(x, y)
        return self.view_to_screen(vx, vy)

    def view_to_screen(self, x: float, y: float) -> tuple[int, int]:
        sx = int(self.map_width / 2.0 + x * self.zoom + self.pan_x)
        sy = int(self.height / 2.0 - y * self.zoom + self.pan_y)
        return sx, sy

    def screen_to_view(self, sx: float, sy: float) -> tuple[float, float]:
        x = (sx - self.map_width / 2.0 - self.pan_x) / self.zoom
        y = -(sy - self.height / 2.0 - self.pan_y) / self.zoom
        return x, y

    def screen_to_world(self, sx: float, sy: float, frame: CoordinateFrame) -> tuple[float, float]:
        vx, vy = self.screen_to_view(sx, sy)
        return frame.view_to_world(vx, vy)

    def zoom_at(self, factor: float, mouse_pos: tuple[int, int]) -> None:
        before = self.screen_to_view(*mouse_pos)
        self.zoom = max(8.0, min(220.0, self.zoom * factor))
        after = self.screen_to_view(*mouse_pos)
        self.pan_x += (after[0] - before[0]) * self.zoom
        self.pan_y -= (after[1] - before[1]) * self.zoom


class Button:
    def __init__(self, rect: pygame.Rect, text: str, action: str) -> None:
        self.rect = rect
        self.text = text
        self.action = action

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        color = (30, 41, 59)
        if self.rect.collidepoint(pygame.mouse.get_pos()):
            color = (51, 65, 85)
        pygame.draw.rect(surface, color, self.rect, border_radius=5)
        pygame.draw.rect(surface, COLOR_PANEL_BORDER, self.rect, width=1, border_radius=5)
        label = font.render(self.text, True, COLOR_TEXT)
        surface.blit(label, label.get_rect(center=self.rect.center))


def ensure_tool_dirs() -> None:
    XML_DIR.mkdir(parents=True, exist_ok=True)
    PCD_DIR.mkdir(parents=True, exist_ok=True)
    POINTS_DIR.mkdir(parents=True, exist_ok=True)
    REGIONS_DIR.mkdir(parents=True, exist_ok=True)


def parse_float_tuple(text: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if not text:
        return default
    return tuple(float(item) for item in text.split())


def is_gray_or_default_geom(geom: ET.Element) -> bool:
    rgba_text = geom.get("rgba")
    if not rgba_text:
        return True
    rgba = parse_float_tuple(rgba_text, ())
    if len(rgba) < 3:
        return True
    r, g, b = rgba[:3]
    neutral = max(r, g, b) - min(r, g, b) < 0.08
    return neutral and max(r, g, b) > 0.25


def classify_competition_geom(geom: ET.Element) -> str | None:
    if geom.get("name") == "floor":
        return "ground"
    pos = parse_float_tuple(geom.get("pos"), ())
    if len(pos) < 2:
        return None
    x, y = pos[0], pos[1]
    rgba = parse_float_tuple(geom.get("rgba"), ())
    red_visual = len(rgba) >= 4 and rgba[0] > 0.9 and rgba[1] < 0.05 and rgba[2] < 0.05 and rgba[3] < 0.5

    if geom.get("type") == "hfield" or is_gray_or_default_geom(geom):
        return None
    if abs(x - 1.8) < 0.15 and abs(y - (-7.0)) < 0.2:
        return "wall"
    if 4.2 <= x <= 6.4 and -12.8 <= y <= -11.7:
        return "gravel"
    if 5.0 <= x <= 6.4 and -9.3 <= y <= -8.7:
        return "low_bar"
    if red_visual and 3.2 <= x <= 4.2 and -9.5 <= y <= -8.5:
        return "spawn_ground"
    if 1.2 <= x <= 3.3 and -13.0 <= y <= -10.0:
        return "slalom"
    if 1.2 <= x <= 3.3 and -5.0 <= y <= -2.0:
        return "stairs"
    if 1.4 <= x <= 6.3 and -5.8 <= y <= 0.4:
        return "bridge_ramp"
    return None


COMPETITION_COMMENTS = {
    "ground": "比赛地图基准地面。",
    "wall": "高墙：机器人需从上方跃过或攀爬通过，自动/遥控均可计分。",
    "gravel": "砂砾碎木坑：L 形障碍，需从 1m 短边进入或离开，完整通过才计分。",
    "low_bar": "限高杆：机器人需从横杆下方通过，碰落横杆则越障失败。",
    "spawn_ground": "启动区：规则允许地面和 T 字形台阶上各一个启动区，机器人需完全纳入启动区后开始。",
    "slalom": "直角绕杆：需按 S 形绕过竖杆，并经过两端及拐角必达区。",
    "stairs": "T 字形台阶：可选启动区之一在最高平台；通过时每一级台阶顶面需至少接触一次。",
    "bridge_ramp": "大斜坡、木桥 A / 木桥 B：大斜坡需满足长边行走距离要求；木桥需从一侧平台经木桥到达另一侧平台。",
}


COMMENT_ORDER = ["ground", "wall", "gravel", "low_bar", "spawn_ground", "stairs", "bridge_ramp", "slalom"]
GROUP_LABELS = {
    "wall": "High Wall",
    "gravel": "Rough Pit",
    "low_bar": "Low Bar",
    "spawn_ground": "Ground Spawn",
    "stairs": "T Stairs",
    "bridge_ramp": "Bridge/Ramp",
    "slalom": "Slalom",
}


def rebuild_worldbody_with_comments(root: ET.Element, groups: dict[str, list[ET.Element]]) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        return
    for child in list(worldbody):
        worldbody.remove(child)
    for key in COMMENT_ORDER:
        items = groups.get(key, [])
        if not items:
            continue
        comment = ET.Comment(f" {COMPETITION_COMMENTS[key]} ")
        comment.tail = "\n    "
        worldbody.append(comment)
        for item in items:
            worldbody.append(item)


def filtered_xml_tree(source: Path) -> tuple[ET.ElementTree, int, int]:
    tree = ET.parse(source)
    root = tree.getroot()
    removed_hfield = 0
    removed_gray = 0
    groups: dict[str, list[ET.Element]] = {key: [] for key in COMMENT_ORDER}

    parents = {child: parent for parent in root.iter() for child in parent}
    for elem in list(root.iter()):
        if elem.tag == "hfield":
            parent = parents.get(elem)
            if parent is not None:
                parent.remove(elem)
                removed_hfield += 1

    for geom in list(root.iter("geom")):
        category = classify_competition_geom(geom)
        if geom.get("type") == "hfield":
            removed_hfield += 1
        elif category is None:
            removed_gray += 1
        else:
            groups[category].append(geom)

    rebuild_worldbody_with_comments(root, groups)
    ET.indent(tree, space="  ")
    return tree, removed_hfield, removed_gray


def format_float(value: float) -> str:
    if abs(value) < 0.0000005:
        value = 0.0
    return f"{value:.6f}".rstrip("0").rstrip(".")


def format_id(value: float | int) -> int | float:
    numeric = float(value)
    if abs(numeric - round(numeric)) < 1e-9:
        return int(round(numeric))
    return float(f"{numeric:.6f}".rstrip("0").rstrip("."))


def new_waypoint_id(waypoints: list[Waypoint], insert_index: int | None = None) -> float | int:
    if insert_index is None or insert_index >= len(waypoints):
        if not waypoints:
            return 1
        return format_id(max(float(point.id or idx + 1) for idx, point in enumerate(waypoints)) + 1)
    candidate = float(waypoints[insert_index - 1].id if insert_index > 0 else 0) + 1.0
    for point in waypoints[insert_index:]:
        if float(point.id or 0) >= candidate:
            point.id = format_id(float(point.id) + 1.0)
    return format_id(candidate)


def normalize_waypoint_ids(waypoints: list[Waypoint]) -> None:
    for index, point in enumerate(waypoints, start=1):
        point.id = index


def sort_and_normalize_waypoint_ids(waypoints: list[Waypoint]) -> None:
    def sort_key(item: tuple[int, Waypoint]) -> tuple[float, int]:
        index, point = item
        try:
            return float(point.id or index + 1), index
        except (TypeError, ValueError):
            return float(index + 1), index

    waypoints[:] = [point for _, point in sorted(enumerate(waypoints), key=sort_key)]
    normalize_waypoint_ids(waypoints)


def move_list_item(items: list, selected: int, target_one_based: float | int) -> int:
    if not items or not (0 <= selected < len(items)):
        return selected
    target = max(0, min(len(items) - 1, int(float(target_one_based)) - 1))
    item = items.pop(selected)
    items.insert(target, item)
    return target


def nav_tools_relative_path(path: Path | None) -> str | None:
    if path is None:
        return None
    try:
        return str(path.resolve().relative_to(TOOL_DIR)).replace("\\", "/")
    except ValueError:
        return path.name


def set_geom_pos(geom: Geom, x: float | None = None, y: float | None = None, z: float | None = None) -> None:
    pos = list(geom.pos)
    while len(pos) < 3:
        pos.append(0.0)
    if x is not None:
        pos[0] = x
    if y is not None:
        pos[1] = y
    if z is not None:
        pos[2] = z
    geom.pos = tuple(pos)
    geom.element.set("pos", " ".join(format_float(v) for v in pos))


def yaw_to_quat_z(yaw: float) -> tuple[float, float, float, float]:
    return math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)


def set_geom_yaw(geom: Geom, yaw: float) -> None:
    quat = yaw_to_quat_z(yaw)
    geom.quat = quat
    geom.element.set("quat", " ".join(format_float(v) for v in quat))


def save_xml_as_new(tree: ET.ElementTree, source_path: Path) -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    out = XML_DIR / f"{source_path.stem}_edited_{timestamp}.xml"
    ET.indent(tree, space="  ")
    tree.write(out, encoding="utf-8", xml_declaration=False)
    return out


def parse_xml_tree(path: Path) -> ET.ElementTree:
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(path, parser=parser)


def group_from_comment(text: str | None) -> str | None:
    clean = (text or "").strip()
    for key, comment in COMPETITION_COMMENTS.items():
        if clean == comment or clean in comment or comment in clean:
            return key
    return None


def mirror_xml_tree(tree: ET.ElementTree) -> ET.ElementTree:
    mirrored = copy.deepcopy(tree)
    root = mirrored.getroot()
    model_name = root.get("model", "")
    root.set("model", f"{model_name} mirror".strip())
    for geom in root.iter("geom"):
        pos = parse_float_tuple(geom.get("pos"), ())
        if len(pos) < 2:
            continue
        shifted = (2.0 * MIRROR_AXIS_X - pos[0], *pos[1:])
        geom.set("pos", " ".join(format_float(value) for value in shifted))
        quat = parse_float_tuple(geom.get("quat"), ())
        if len(quat) == 4:
            mirrored_quat = (quat[0], quat[1], -quat[2], quat[3])
            geom.set("quat", " ".join(format_float(value) for value in mirrored_quat))
    ET.indent(mirrored, space="  ")
    return mirrored


def write_filtered_xmls(source: Path) -> None:
    tree, _, _ = filtered_xml_tree(source)
    XML_DIR.mkdir(parents=True, exist_ok=True)
    tree.write(LOCAL_XML, encoding="utf-8", xml_declaration=False)
    mirror_xml_tree(tree).write(MIRROR_XML, encoding="utf-8", xml_declaration=False)


def ensure_local_xml(force: bool = False) -> Path:
    ensure_tool_dirs()
    xml_paths = scan_xml_files()
    if not xml_paths:
        raise FileNotFoundError(f"No XML files found in {XML_DIR}")
    return xml_paths[0]


def parse_xml_geoms(xml_path: Path) -> list[Geom]:
    if not xml_path.exists():
        raise FileNotFoundError(f"XML map not found: {xml_path}")

    tree = parse_xml_tree(xml_path)
    return parse_xml_geoms_from_root(tree.getroot())


def parse_xml_geoms_from_root(root: ET.Element) -> list[Geom]:
    geoms: list[Geom] = []
    worldbody = root.find("worldbody")
    if worldbody is None:
        return geoms
    current_group: str | None = None
    geom_index = 0
    for geom in list(worldbody):
        if geom.tag is ET.Comment:
            current_group = group_from_comment(geom.text)
            continue
        if geom.tag != "geom":
            continue
        name = geom.get("name", f"geom_{geom_index}")
        if name == "floor":
            geom_index += 1
            continue

        kind = geom.get("type")
        pos = parse_float_tuple(geom.get("pos"), ())
        if kind == "hfield" or not kind or not pos:
            geom_index += 1
            continue

        quat = parse_float_tuple(geom.get("quat"), (1.0, 0.0, 0.0, 0.0))
        rgba = parse_float_tuple(geom.get("rgba"), (0.7, 0.7, 0.7, 1.0))
        size = parse_float_tuple(geom.get("size"), ())
        contype = geom.get("contype", "1")
        conaffinity = geom.get("conaffinity", "1")

        if len(quat) != 4 or len(rgba) < 3:
            continue
        if len(rgba) == 3:
            rgba = (*rgba, 1.0)

        geoms.append(
            Geom(
                element=geom,
                xml_index=geom_index,
                group=current_group or classify_competition_geom(geom) or "ungrouped",
                name=name,
                kind=kind,
                pos=pos,
                size=size,
                quat=(quat[0], quat[1], quat[2], quat[3]),
                rgba=(rgba[0], rgba[1], rgba[2], rgba[3]),
                collidable=(contype != "0" and conaffinity != "0"),
            )
        )
        geom_index += 1

    return geoms


def load_xml_scene(xml_path: Path) -> tuple[ET.ElementTree, list[Geom]]:
    tree = parse_xml_tree(xml_path)
    return tree, parse_xml_geoms_from_root(tree.getroot())


def quat_to_yaw(quat_wxyz: tuple[float, float, float, float]) -> float:
    qw, qx, qy, qz = quat_wxyz
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def rgba_to_color(rgba: tuple[float, float, float, float]) -> tuple[int, int, int, int]:
    return tuple(max(0, min(255, int(value * 255))) for value in rgba)


def brighten(color: tuple[int, int, int, int], amount: int = 45) -> tuple[int, int, int]:
    return min(color[0] + amount, 255), min(color[1] + amount, 255), min(color[2] + amount, 255)


def iter_ascii_pcd_points(source: Path):
    with source.open("r", encoding="utf-8", errors="ignore") as handle:
        data_started = False
        fields: list[str] = []
        x_idx, y_idx, z_idx = 0, 1, 2
        for raw in handle:
            line = raw.strip()
            if not line:
                continue
            if data_started:
                parts = line.split()
                if len(parts) <= max(x_idx, y_idx, z_idx):
                    continue
                try:
                    yield float(parts[x_idx]), float(parts[y_idx]), float(parts[z_idx])
                except ValueError:
                    continue
            elif line.upper().startswith("FIELDS "):
                fields = line.split()[1:]
                lower_fields = [field.lower() for field in fields]
                if all(name in lower_fields for name in ("x", "y", "z")):
                    x_idx = lower_fields.index("x")
                    y_idx = lower_fields.index("y")
                    z_idx = lower_fields.index("z")
            elif line.upper().startswith("DATA"):
                if "ascii" not in line.lower():
                    raise RuntimeError(f"Only ASCII PCD is supported: {source}")
                if fields:
                    lower_fields = [field.lower() for field in fields]
                    missing = [name for name in ("x", "y", "z") if name not in lower_fields]
                    if missing:
                        raise RuntimeError(f"PCD missing fields {missing}: {source}")
                data_started = True


def scan_pcd_files() -> list[Path]:
    ensure_tool_dirs()
    return sorted(PCD_DIR.glob("*.pcd"), key=lambda item: item.name.lower())


def scan_xml_files() -> list[Path]:
    ensure_tool_dirs()
    paths = sorted(XML_DIR.glob("*.xml"), key=lambda item: item.name.lower())
    ordered: list[Path] = []
    for path in (LOCAL_XML, MIRROR_XML):
        if path.exists() and path not in ordered:
            ordered.append(path)
    for path in paths:
        if path not in ordered:
            ordered.append(path)
    return ordered


def scan_point_files() -> list[Path]:
    ensure_tool_dirs()
    return sorted(POINTS_DIR.glob("*.json"), key=lambda item: item.stat().st_mtime, reverse=True)


def load_pcd(path: Path, max_points: int = 80000, z_min: float = -5.0, z_max: float = 0.5) -> PointCloud:
    cloud = PointCloud(path=path)
    try:
        filtered: list[tuple[float, float, float]] = []
        for x, y, z in iter_ascii_pcd_points(path):
            cloud.total_count += 1
            if z_min <= z <= z_max:
                filtered.append((x, y, z))

        stride = max(1, math.ceil(len(filtered) / max_points)) if filtered else 1
        cloud.points = filtered[::stride]
        cloud.sampled_count = len(cloud.points)
    except Exception as exc:
        cloud.error = str(exc)
    return cloud


def optional_float(row: dict, *keys: str) -> float | None:
    for key in keys:
        if key in row and row[key] is not None:
            try:
                return float(row[key])
            except (TypeError, ValueError):
                return None
    return None


def optional_int(row: dict, *keys: str) -> int | None:
    for key in keys:
        if key in row and row[key] is not None:
            try:
                return int(row[key])
            except (TypeError, ValueError):
                return None
    return None


def optional_bool(row: dict, default: bool, *keys: str) -> bool:
    for key in keys:
        if key not in row or row[key] is None:
            continue
        value = row[key]
        if isinstance(value, bool):
            return value
        text = str(value).strip().lower()
        if text in {"1", "true", "yes", "on"}:
            return True
        if text in {"0", "false", "no", "off"}:
            return False
    return default


def normalize_task_name(value: object) -> str:
    text = str(value or "none").strip().lower()
    if text == "ramp_bridge":
        return "ramp"
    if text == "stairs":
        return "stairs_up"
    if text in {"", "none", "null", "custom", "flat"}:
        return "none"
    return text or "none"


def load_waypoints_json(path: Path, frame: CoordinateFrame) -> list[Waypoint]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    origin = payload.get("origin", {})
    if isinstance(origin, dict):
        frame.mode = str(origin.get("mode", frame.mode))
        frame.origin_x = float(origin.get("x", frame.origin_x))
        frame.origin_y = float(origin.get("y", frame.origin_y))
        if "yaw_rad" in origin:
            frame.origin_yaw = float(origin["yaw_rad"])
        elif "yaw_deg" in origin:
            frame.origin_yaw = math.radians(float(origin["yaw_deg"]))

    rows = payload.get("waypoints")
    if rows is None and isinstance(payload.get("segments"), list):
        rows = []
        for segment in payload["segments"]:
            if isinstance(segment, dict) and isinstance(segment.get("waypoints"), list):
                rows.extend(segment["waypoints"])
    if not isinstance(rows, list):
        raise ValueError(f"JSON has no waypoints array or segments[].waypoints: {path}")

    loaded: list[Waypoint] = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        x = float(row.get("x", 0.0))
        y = float(row.get("y", 0.0))
        fallback_world_x, fallback_world_y = frame.view_to_world(x, y)
        world_x = float(row.get("world_x", fallback_world_x))
        world_y = float(row.get("world_y", fallback_world_y))
        loaded.append(
            Waypoint(
                x=x,
                y=y,
                world_x=world_x,
                world_y=world_y,
                origin_mode=str(row.get("origin_mode", frame.mode)),
                id=float(row.get("id", len(loaded) + 1)),
                yawDeg=float(row.get("yawDeg", row.get("yaw", 0.0))),
                tolerance=float(row.get("tolerance", 0.15)),
                speed=float(row.get("speed", 0.35)),
                policy=str(row.get("policy", "rough")),
                task=normalize_task_name(row.get("task", row.get("task_id", "none"))),
                requireYaw=optional_bool(row, False, "requireYaw", "require_yaw"),
                precisionFollow=optional_bool(row, False, "precisionFollow", "precision_follow"),
                stableCycles=optional_int(row, "stableCycles", "stable_cycles"),
                lookahead=optional_float(row, "lookahead", "lookAhead"),
                yawRateLimit=optional_float(row, "yawRateLimit", "yaw_rate_limit"),
                slalomStraight=optional_bool(row, False, "slalomStraight", "slalom_straight"),
            )
        )
    sort_and_normalize_waypoint_ids(loaded)
    return loaded


def load_avoid_regions_json(path: Path) -> list[AvoidRegion]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    rows = payload.get("regions", [])
    if not isinstance(rows, list):
        return []

    regions: list[AvoidRegion] = []
    for index, row in enumerate(rows, start=1):
        if not isinstance(row, dict):
            continue
        polygon = row.get("polygon", row.get("points", []))
        if not isinstance(polygon, list):
            continue
        points: list[tuple[float, float]] = []
        for point in polygon:
            if isinstance(point, dict):
                points.append((float(point.get("x", 0.0)), float(point.get("y", 0.0))))
            elif isinstance(point, (list, tuple)) and len(point) >= 2:
                points.append((float(point[0]), float(point[1])))
        if len(points) >= 3:
            regions.append(
                AvoidRegion(
                    name=str(row.get("name", f"avoid_{index}")),
                    points=points,
                    kind=str(row.get("kind", "avoid")),
                )
            )
    return regions


def geom_polygon_world(geom: Geom) -> list[tuple[float, float]]:
    cx, cy = geom.pos[0], geom.pos[1]
    sx, sy = geom.size[0], geom.size[1]
    yaw = quat_to_yaw(geom.quat)
    points = []
    for lx, ly in [(-sx, -sy), (sx, -sy), (sx, sy), (-sx, sy)]:
        wx = cx + lx * math.cos(yaw) - ly * math.sin(yaw)
        wy = cy + lx * math.sin(yaw) + ly * math.cos(yaw)
        points.append((wx, wy))
    return points


def group_items(geoms: list[Geom], group: str | None) -> list[Geom]:
    if not group:
        return []
    return [geom for geom in geoms if geom.group == group]


def geom_xy_bounds(geom: Geom) -> tuple[float, float, float, float] | None:
    if len(geom.pos) < 2:
        return None
    if geom.kind == "box" and len(geom.size) >= 2:
        points = geom_polygon_world(geom)
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return min(xs), max(xs), min(ys), max(ys)
    if geom.kind == "cylinder" and geom.size:
        r = geom.size[0]
        return geom.pos[0] - r, geom.pos[0] + r, geom.pos[1] - r, geom.pos[1] + r
    return geom.pos[0], geom.pos[0], geom.pos[1], geom.pos[1]


def group_center(geoms: list[Geom], group: str | None) -> tuple[float, float, float] | None:
    items = group_items(geoms, group)
    bounds = [b for b in (geom_xy_bounds(geom) for geom in items) if b]
    if not bounds:
        return None
    min_x = min(b[0] for b in bounds)
    max_x = max(b[1] for b in bounds)
    min_y = min(b[2] for b in bounds)
    max_y = max(b[3] for b in bounds)
    z_values = [geom.pos[2] for geom in items if len(geom.pos) >= 3]
    z = sum(z_values) / len(z_values) if z_values else 0.0
    return (min_x + max_x) / 2.0, (min_y + max_y) / 2.0, z


def group_yaw_deg(geoms: list[Geom], group: str | None) -> float | None:
    items = group_items(geoms, group)
    if not items:
        return None
    return math.degrees(quat_to_yaw(items[0].quat))


def move_group_center(geoms: list[Geom], group: str | None, x: float | None = None, y: float | None = None, z: float | None = None) -> None:
    center = group_center(geoms, group)
    if center is None:
        return
    dx = 0.0 if x is None else x - center[0]
    dy = 0.0 if y is None else y - center[1]
    dz = 0.0 if z is None else z - center[2]
    for geom in group_items(geoms, group):
        pos = list(geom.pos)
        while len(pos) < 3:
            pos.append(0.0)
        set_geom_pos(geom, pos[0] + dx, pos[1] + dy, pos[2] + dz)


def rotate_group_yaw(geoms: list[Geom], group: str | None, yaw_deg: float) -> None:
    center = group_center(geoms, group)
    current = group_yaw_deg(geoms, group)
    if center is None or current is None:
        return
    delta = math.radians(yaw_deg - current)
    c = math.cos(delta)
    s = math.sin(delta)
    cx, cy, _ = center
    for geom in group_items(geoms, group):
        pos = list(geom.pos)
        while len(pos) < 3:
            pos.append(0.0)
        dx = pos[0] - cx
        dy = pos[1] - cy
        nx = cx + dx * c - dy * s
        ny = cy + dx * s + dy * c
        set_geom_pos(geom, nx, ny, pos[2])
        set_geom_yaw(geom, quat_to_yaw(geom.quat) + delta)


def bounds_from_data(
    geoms: list[Geom],
    cloud: PointCloud,
    frame: CoordinateFrame,
) -> tuple[float, float, float, float] | None:
    xs: list[float] = []
    ys: list[float] = []

    for geom in geoms:
        if len(geom.pos) < 2:
            continue
        if geom.kind == "box" and len(geom.size) >= 2:
            points = geom_polygon_world(geom)
        elif geom.kind == "cylinder" and geom.size:
            r = geom.size[0]
            points = [(geom.pos[0] - r, geom.pos[1] - r), (geom.pos[0] + r, geom.pos[1] + r)]
        else:
            points = [(geom.pos[0], geom.pos[1])]
        for wx, wy in points:
            xs.append(wx)
            ys.append(wy)

    for wx, wy, _ in cloud.points[:: max(1, len(cloud.points) // 20000 or 1)]:
        vx, vy = frame.world_to_view(wx, wy)
        xs.append(vx)
        ys.append(vy)

    if not xs or not ys:
        return None
    return min(xs), max(xs), min(ys), max(ys)


def fit_camera(camera: MapCamera, geoms: list[Geom], cloud: PointCloud, frame: CoordinateFrame) -> None:
    bounds = bounds_from_data(geoms, cloud, frame)
    if not bounds:
        return
    min_x, max_x, min_y, max_y = bounds
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    margin = 70
    camera.zoom = min((camera.map_width - margin * 2) / span_x, (camera.height - margin * 2) / span_y)
    camera.zoom = max(8.0, min(220.0, camera.zoom))
    camera.pan_x = -((min_x + max_x) / 2.0) * camera.zoom
    camera.pan_y = ((min_y + max_y) / 2.0) * camera.zoom


def draw_grid(surface: pygame.Surface, camera: MapCamera, frame: CoordinateFrame, font: pygame.font.Font) -> None:
    min_vx, min_vy = camera.screen_to_view(0, camera.height)
    max_vx, max_vy = camera.screen_to_view(camera.map_width, 0)
    for gx in range(math.floor(min_vx), math.ceil(max_vx) + 1):
        sx, _ = camera.view_to_screen(gx, 0)
        color = COLOR_AXIS if gx == 0 else COLOR_GRID
        pygame.draw.line(surface, color, (sx, 0), (sx, camera.height), 2 if gx == 0 else 1)
        if gx % 2 == 0 and 0 < sx < camera.map_width - 30:
            surface.blit(font.render(f"{gx}m", True, COLOR_MUTED), (sx + 4, camera.height - 22))
    for gy in range(math.floor(min_vy), math.ceil(max_vy) + 1):
        _, sy = camera.view_to_screen(0, gy)
        color = COLOR_AXIS if gy == 0 else COLOR_GRID
        pygame.draw.line(surface, color, (0, sy), (camera.map_width, sy), 2 if gy == 0 else 1)
        if gy % 2 == 0 and 0 < sy < camera.height - 24:
            surface.blit(font.render(f"{gy}m", True, COLOR_MUTED), (6, sy + 4))

    mode = "custom" if frame.mode == "custom" else "world"
    surface.blit(font.render(f"pcd origin: {mode}", True, COLOR_CYAN), (10, 10))


def draw_transparent_polygon(surface: pygame.Surface, points: list[tuple[int, int]], fill, outline) -> None:
    if fill[3] >= 250:
        pygame.draw.polygon(surface, fill[:3], points)
    else:
        layer = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
        pygame.draw.polygon(layer, fill, points)
        surface.blit(layer, (0, 0))
    pygame.draw.polygon(surface, outline, points, width=1)


def draw_geoms(surface: pygame.Surface, camera: MapCamera, geoms: list[Geom], selected_group: str | None = None) -> None:
    for geom in geoms:
        if len(geom.pos) < 2:
            continue
        color = rgba_to_color(geom.rgba)
        outline = brighten(color)
        if geom.kind == "box" and len(geom.size) >= 2:
            points = [camera.view_to_screen(x, y) for x, y in geom_polygon_world(geom)]
            draw_transparent_polygon(surface, points, color, outline)
            if geom.group == selected_group:
                pygame.draw.polygon(surface, COLOR_GREEN, points, width=3)
        elif geom.kind == "cylinder" and geom.size:
            center = camera.view_to_screen(geom.pos[0], geom.pos[1])
            radius = max(int(geom.size[0] * camera.zoom), 3)
            if color[3] >= 250:
                pygame.draw.circle(surface, color[:3], center, radius)
            else:
                layer = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
                pygame.draw.circle(layer, color, center, radius)
                surface.blit(layer, (0, 0))
            pygame.draw.circle(surface, outline, center, radius, width=1)
            if geom.group == selected_group:
                pygame.draw.circle(surface, COLOR_GREEN, center, radius + 4, width=3)


def draw_pcd(surface: pygame.Surface, camera: MapCamera, frame: CoordinateFrame, cloud: PointCloud) -> None:
    if not cloud.points:
        return
    width, height = camera.map_width, camera.height
    for x, y, z in cloud.points:
        px, py = camera.world_to_screen(x, y, frame)
        if 0 <= px < width and 0 <= py < height:
            if z > 0.3:
                color = (148, 210, 255)
            elif z < -0.2:
                color = (55, 92, 150)
            else:
                color = COLOR_PCD
            surface.set_at((px, py), color)


def draw_avoid_regions(
    surface: pygame.Surface,
    camera: MapCamera,
    regions: list[AvoidRegion],
    current_region: list[tuple[float, float]],
    selected_region: int = -1,
) -> None:
    layer = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    for index, region in enumerate(regions):
        points = [camera.view_to_screen(x, y) for x, y in region.points]
        if len(points) >= 3:
            pygame.draw.polygon(layer, (*COLOR_WARNING, 70), points)
            outline = COLOR_GOLD if index == selected_region else COLOR_WARNING
            pygame.draw.polygon(surface, outline, points, width=3 if index == selected_region else 2)
    if current_region:
        points = [camera.view_to_screen(x, y) for x, y in current_region]
        for point in points:
            pygame.draw.circle(surface, COLOR_GOLD, point, 5)
        if len(points) >= 2:
            pygame.draw.lines(surface, COLOR_GOLD, False, points, width=2)
    surface.blit(layer, (0, 0))


def robot_wheel_local_points() -> list[tuple[str, float, float]]:
    thigh_dx = -0.25 * math.sin(ROBOT_POSE_HIP)
    shank_dx = -0.2 * math.sin(ROBOT_POSE_HIP + ROBOT_POSE_KNEE)
    wheels = []
    for name, (pitch_x, pitch_y, knee_y, wheel_y, wheel_geom_y, knee_x) in ROBOT_WHEEL_POSITIONS.items():
        x = ROBOT_BODY_CENTER_OFFSET_X + pitch_x + knee_x + thigh_dx + shank_dx
        y = pitch_y + knee_y + wheel_y + wheel_geom_y
        wheels.append((name, x, y))
    return wheels


def transform_local_point(cx: float, cy: float, yaw: float, lx: float, ly: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return cx + lx * c - ly * s, cy + lx * s + ly * c


def local_rect_polygon(
    cx: float,
    cy: float,
    yaw: float,
    lx: float,
    ly: float,
    length: float,
    width: float,
) -> list[tuple[float, float]]:
    hx = length * 0.5
    hy = width * 0.5
    return [
        transform_local_point(cx, cy, yaw, lx + hx, ly + hy),
        transform_local_point(cx, cy, yaw, lx + hx, ly - hy),
        transform_local_point(cx, cy, yaw, lx - hx, ly - hy),
        transform_local_point(cx, cy, yaw, lx - hx, ly + hy),
    ]


def draw_robot_waypoint(
    surface: pygame.Surface,
    camera: MapCamera,
    point: Waypoint,
    font: pygame.font.Font,
    selected: bool,
) -> None:
    yaw = math.radians(point.yawDeg)
    body_cx = ROBOT_BODY_CENTER_OFFSET_X
    body = local_rect_polygon(point.x, point.y, yaw, body_cx, 0.0, ROBOT_BODY_LENGTH, ROBOT_BODY_WIDTH)
    body_screen = [camera.view_to_screen(x, y) for x, y in body]
    fill = (31, 41, 55, 220) if not selected else (49, 63, 90, 235)
    outline = COLOR_GOLD if selected else COLOR_TEXT
    draw_transparent_polygon(surface, body_screen, fill, outline)

    front_left = transform_local_point(point.x, point.y, yaw, body_cx + ROBOT_BODY_LENGTH * 0.5, ROBOT_BODY_WIDTH * 0.5)
    front_right = transform_local_point(point.x, point.y, yaw, body_cx + ROBOT_BODY_LENGTH * 0.5, -ROBOT_BODY_WIDTH * 0.5)
    nose = transform_local_point(point.x, point.y, yaw, body_cx + ROBOT_BODY_LENGTH * 0.72, 0.0)
    nose_screen = [camera.view_to_screen(x, y) for x, y in (front_left, nose, front_right)]
    pygame.draw.polygon(surface, COLOR_GREEN, nose_screen, width=2)

    wheel_fill = (8, 13, 25, 245)
    wheel_outline = COLOR_CYAN if not selected else COLOR_GOLD
    for _, lx, ly in robot_wheel_local_points():
        wheel = local_rect_polygon(
            point.x,
            point.y,
            yaw,
            lx,
            ly,
            ROBOT_WHEEL_VIS_LENGTH,
            ROBOT_WHEEL_VIS_WIDTH,
        )
        wheel_screen = [camera.view_to_screen(x, y) for x, y in wheel]
        pygame.draw.polygon(surface, wheel_fill, wheel_screen)
        pygame.draw.polygon(surface, wheel_outline, wheel_screen, width=1)
        wx, wy = transform_local_point(point.x, point.y, yaw, lx, ly)
        pygame.draw.circle(surface, wheel_outline, camera.view_to_screen(wx, wy), max(2, int(camera.zoom * 0.018)), width=1)

    center = camera.view_to_screen(point.x, point.y)
    origin_radius = max(2, int(camera.zoom * 0.025))
    pygame.draw.circle(surface, COLOR_GOLD if selected else COLOR_GREEN, center, origin_radius)
    pygame.draw.line(surface, COLOR_TEXT, (center[0] - origin_radius - 2, center[1]), (center[0] + origin_radius + 2, center[1]), 1)
    pygame.draw.line(surface, COLOR_TEXT, (center[0], center[1] - origin_radius - 2), (center[0], center[1] + origin_radius + 2), 1)
    label = font.render(str(format_id(point.id)), True, COLOR_TEXT)
    surface.blit(label, (center[0] + 10, center[1] - 13))


def draw_waypoints(
    surface: pygame.Surface,
    camera: MapCamera,
    frame: CoordinateFrame,
    waypoints: list[Waypoint],
    font: pygame.font.Font,
    selected_waypoint: int,
) -> None:
    for index, point in enumerate(waypoints, start=1):
        draw_robot_waypoint(surface, camera, point, font, index - 1 == selected_waypoint)


def save_waypoints(
    waypoints: list[Waypoint],
    frame: CoordinateFrame,
    xml_path: Path,
    cloud: PointCloud,
    regions: list[AvoidRegion] | None = None,
) -> Path:
    normalize_waypoint_ids(waypoints)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    created_at = time.strftime("%Y-%m-%dT%H:%M:%S")
    path = POINTS_DIR / f"points_{timestamp}.json"
    route_waypoints = []
    for index, point in enumerate(waypoints, start=1):
        task = normalize_task_name(point.task)
        row = {
            "id": format_id(point.id or index),
            "x": point.x,
            "y": point.y,
            "yawDeg": point.yawDeg,
            "speed": point.speed,
            "policy": point.policy,
            "task": task,
            "tolerance": point.tolerance,
        }
        is_slalom = task == "slalom"
        if point.requireYaw:
            row["requireYaw"] = True
        if point.precisionFollow or is_slalom:
            row["requireYaw"] = False
            row["precisionFollow"] = True
            row["stableCycles"] = 0 if point.stableCycles is None else point.stableCycles
            row["lookahead"] = 0.35 if point.lookahead is None else point.lookahead
            row["yawRateLimit"] = 0.45 if point.yawRateLimit is None else point.yawRateLimit
        elif point.stableCycles is not None:
            row["stableCycles"] = point.stableCycles
        if point.lookahead is not None and not (point.precisionFollow or is_slalom):
            row["lookahead"] = point.lookahead
        if point.yawRateLimit is not None and not (point.precisionFollow or is_slalom):
            row["yawRateLimit"] = point.yawRateLimit
        if point.slalomStraight:
            row["slalomStraight"] = True
        route_waypoints.append(row)
    route_regions = [
        {
            "id": index,
            "name": region.name,
            "kind": region.kind,
            "polygon": [{"x": x, "y": y} for x, y in region.points],
        }
        for index, region in enumerate(regions or [], start=1)
    ]
    payload = {
        "name": f"nav_tools_{timestamp}",
        "map": cloud.path.stem if cloud.path else "",
        "frame_id": "map",
        "createdAt": created_at,
        "segments": [
            {
                "name": "segment_1",
                "obstacle": "custom",
                "waypoints": route_waypoints,
            }
        ],
        "regions": route_regions,
        "avoid_regions": route_regions,
        "yawToleranceDegDefault": 15.0,
        "xml": nav_tools_relative_path(xml_path),
        "pcd": nav_tools_relative_path(cloud.path),
        "origin": {
            "mode": frame.mode,
            "x": frame.origin_x,
            "y": frame.origin_y,
            "yaw_rad": frame.origin_yaw,
            "yaw_deg": math.degrees(frame.origin_yaw),
        },
        "waypoints": [
            {
                **route_waypoints[index - 1],
                "world_x": point.world_x,
                "world_y": point.world_y,
                "origin_mode": point.origin_mode,
            }
            for index, point in enumerate(waypoints, start=1)
        ],
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def save_avoid_regions(regions: list[AvoidRegion], cloud: PointCloud) -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    path = REGIONS_DIR / f"avoid_regions_{timestamp}.json"
    payload = {
        "name": f"avoid_regions_{timestamp}",
        "map": cloud.path.stem if cloud.path else "",
        "frame_id": "map",
        "regions": [
            {
                "id": index,
                "name": region.name,
                "kind": region.kind,
                "polygon": [{"x": x, "y": y} for x, y in region.points],
            }
            for index, region in enumerate(regions, start=1)
        ],
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def get_font(size: int, bold: bool = False) -> pygame.font.Font:
    for name in ("microsoftyahei", "segoeui", "arial", "sans"):
        try:
            font = pygame.font.SysFont(name, size, bold=bold)
            if font:
                return font
        except Exception:
            continue
    return pygame.font.Font(None, max(size + 2, size))


def draw_text(surface: pygame.Surface, font: pygame.font.Font, text: str, x: int, y: int, color=COLOR_TEXT) -> None:
    surface.blit(font.render(text, True, color), (x, y))


def draw_input(
    surface: pygame.Surface,
    font: pygame.font.Font,
    rect: pygame.Rect,
    key: str,
    text: str,
    active_input: str | None,
    input_text: str,
    boxes: list[InputBox],
) -> None:
    shown = input_text if active_input == key else text
    if active_input == key and int(time.time() * 2) % 2 == 0:
        shown += "_"
    border = COLOR_GREEN if active_input == key else COLOR_PANEL_BORDER
    pygame.draw.rect(surface, COLOR_PANEL_DARK, rect, border_radius=4)
    pygame.draw.rect(surface, border, rect, width=1, border_radius=4)
    label = font.render(shown, True, COLOR_TEXT)
    surface.blit(label, (rect.x + 6, rect.y + 5))
    boxes.append(InputBox(rect, key, text))


def input_current_value(
    key: str,
    frame: CoordinateFrame,
    waypoints: list[Waypoint],
    selected: int,
    geoms: list[Geom] | None = None,
    selected_group: str | None = None,
    regions: list[AvoidRegion] | None = None,
    selected_region: int = -1,
) -> str:
    if key == "origin_x":
        return f"{frame.origin_x:.4f}"
    if key == "origin_y":
        return f"{frame.origin_y:.4f}"
    if key == "origin_yaw":
        return f"{math.degrees(frame.origin_yaw):.2f}"
    if 0 <= selected < len(waypoints):
        wp = waypoints[selected]
        if key == "wp_id":
            return str(format_id(wp.id or selected + 1))
        if key == "wp_x":
            return f"{wp.x:.4f}"
        if key == "wp_y":
            return f"{wp.y:.4f}"
        if key == "wp_yaw":
            return f"{wp.yawDeg:.2f}"
        if key == "wp_tolerance":
            return f"{wp.tolerance:.3f}"
        if key == "wp_speed":
            return f"{wp.speed:.3f}"
        if key == "wp_policy":
            return wp.policy
        if key == "wp_task":
            return wp.task
    if geoms is not None and selected_group:
        center = group_center(geoms, selected_group)
        if center:
            if key == "xml_group_x":
                return f"{center[0]:.4f}"
            if key == "xml_group_y":
                return f"{center[1]:.4f}"
            if key == "xml_group_yaw":
                yaw = group_yaw_deg(geoms, selected_group)
                return f"{yaw:.2f}" if yaw is not None else "0"
    if regions is not None and 0 <= selected_region < len(regions):
        region = regions[selected_region]
        if key == "region_id":
            return str(selected_region + 1)
        if key == "region_name":
            return region.name
        if key == "region_kind":
            return region.kind
    return ""


def commit_input_value(
    key: str,
    value: str,
    frame: CoordinateFrame,
    waypoints: list[Waypoint],
    selected: int,
    geoms: list[Geom] | None = None,
    selected_group: str | None = None,
    regions: list[AvoidRegion] | None = None,
    selected_region: int = -1,
) -> tuple[int, int]:
    value = value.strip()
    if not value:
        return selected, selected_region
    next_selected = selected
    next_selected_region = selected_region
    try:
        if key == "origin_x":
            frame.origin_x = float(value)
        elif key == "origin_y":
            frame.origin_y = float(value)
        elif key == "origin_yaw":
            frame.origin_yaw = math.radians(float(value))
        elif 0 <= selected < len(waypoints) and key.startswith("wp_"):
            wp = waypoints[selected]
            if key == "wp_id":
                next_selected = move_list_item(waypoints, selected, float(value))
                normalize_waypoint_ids(waypoints)
            elif key == "wp_x":
                wp.x = float(value)
                wp.world_x, wp.world_y = frame.view_to_world(wp.x, wp.y)
            elif key == "wp_y":
                wp.y = float(value)
                wp.world_x, wp.world_y = frame.view_to_world(wp.x, wp.y)
            elif key == "wp_yaw":
                wp.yawDeg = float(value)
            elif key == "wp_tolerance":
                wp.tolerance = max(0.0, float(value))
            elif key == "wp_speed":
                wp.speed = max(0.0, float(value))
            elif key == "wp_policy":
                wp.policy = value or "rough"
            elif key == "wp_task":
                wp.task = normalize_task_name(value)
        elif geoms is not None and selected_group and key.startswith("xml_group_"):
            if key == "xml_group_x":
                move_group_center(geoms, selected_group, x=float(value))
            elif key == "xml_group_y":
                move_group_center(geoms, selected_group, y=float(value))
            elif key == "xml_group_yaw":
                rotate_group_yaw(geoms, selected_group, float(value))
        elif regions is not None and 0 <= selected_region < len(regions):
            region = regions[selected_region]
            if key == "region_id":
                next_selected_region = move_list_item(regions, selected_region, float(value))
            elif key == "region_name":
                region.name = value
            elif key == "region_kind":
                region.kind = value or "avoid"
    except ValueError:
        return selected, selected_region
    return next_selected, next_selected_region


def next_input_key(current: str | None, boxes: list[InputBox]) -> str | None:
    if not boxes:
        return None
    keys = [box.key for box in boxes]
    if current not in keys:
        return keys[0]
    return keys[(keys.index(current) + 1) % len(keys)]


def key_to_input_char(event: pygame.event.Event) -> str:
    if event.unicode and event.unicode.isprintable():
        return event.unicode
    digit_keys = {
        pygame.K_0: "0",
        pygame.K_1: "1",
        pygame.K_2: "2",
        pygame.K_3: "3",
        pygame.K_4: "4",
        pygame.K_5: "5",
        pygame.K_6: "6",
        pygame.K_7: "7",
        pygame.K_8: "8",
        pygame.K_9: "9",
        pygame.K_KP0: "0",
        pygame.K_KP1: "1",
        pygame.K_KP2: "2",
        pygame.K_KP3: "3",
        pygame.K_KP4: "4",
        pygame.K_KP5: "5",
        pygame.K_KP6: "6",
        pygame.K_KP7: "7",
        pygame.K_KP8: "8",
        pygame.K_KP9: "9",
        pygame.K_PERIOD: ".",
        pygame.K_KP_PERIOD: ".",
        pygame.K_MINUS: "-",
        pygame.K_KP_MINUS: "-",
        pygame.K_PLUS: "+",
        pygame.K_KP_PLUS: "+",
    }
    return digit_keys.get(event.key, "")


def activate_input(
    key: str,
    frame: CoordinateFrame,
    waypoints: list[Waypoint],
    selected: int,
    geoms: list[Geom] | None = None,
    selected_group: str | None = None,
    regions: list[AvoidRegion] | None = None,
    selected_region: int = -1,
) -> tuple[str, str]:
    pygame.key.start_text_input()
    return key, input_current_value(key, frame, waypoints, selected, geoms, selected_group, regions, selected_region)


def clear_input() -> tuple[None, str]:
    return None, ""


def select_waypoint_at(
    mouse_pos: tuple[int, int],
    camera: MapCamera,
    waypoints: list[Waypoint],
) -> int | None:
    for idx in range(len(waypoints) - 1, -1, -1):
        sx, sy = camera.view_to_screen(waypoints[idx].x, waypoints[idx].y)
        if math.hypot(mouse_pos[0] - sx, mouse_pos[1] - sy) <= 12:
            return idx
    return None


def point_in_polygon(point: tuple[int, int], polygon: list[tuple[int, int]]) -> bool:
    px, py = point
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if (yi > py) != (yj > py):
            x_at_y = (xj - xi) * (py - yi) / ((yj - yi) or 1e-9) + xi
            if px < x_at_y:
                inside = not inside
        j = i
    return inside


def select_group_at(mouse_pos: tuple[int, int], camera: MapCamera, geoms: list[Geom]) -> str | None:
    for idx in range(len(geoms) - 1, -1, -1):
        geom = geoms[idx]
        if geom.kind == "box" and len(geom.size) >= 2:
            points = [camera.view_to_screen(x, y) for x, y in geom_polygon_world(geom)]
            if point_in_polygon(mouse_pos, points):
                return geom.group
        elif geom.kind == "cylinder" and geom.size:
            sx, sy = camera.view_to_screen(geom.pos[0], geom.pos[1])
            radius = max(int(geom.size[0] * camera.zoom), 4)
            if math.hypot(mouse_pos[0] - sx, mouse_pos[1] - sy) <= radius + 5:
                return geom.group
    return None


def select_avoid_region_at(mouse_pos: tuple[int, int], camera: MapCamera, regions: list[AvoidRegion]) -> int | None:
    for idx in range(len(regions) - 1, -1, -1):
        points = [camera.view_to_screen(x, y) for x, y in regions[idx].points]
        if len(points) >= 3 and point_in_polygon(mouse_pos, points):
            return idx
    return None


def default_yaw_for_new_waypoint(x: float, y: float, waypoints: list[Waypoint]) -> float:
    if not waypoints:
        return 0.0
    prev = waypoints[-1]
    return math.degrees(math.atan2(y - prev.y, x - prev.x))


def draw_panel(
    surface: pygame.Surface,
    camera: MapCamera,
    font: pygame.font.Font,
    small_font: pygame.font.Font,
    panel_scroll: int,
    geoms: list[Geom],
    xml_files: list[Path],
    selected_xml: int,
    cloud: PointCloud,
    pcd_files: list[Path],
    selected_pcd: int,
    point_files: list[Path],
    selected_point_file: int,
    dropdown_open: str,
    tool_mode: str,
    avoid_regions: list[AvoidRegion],
    current_region: list[tuple[float, float]],
    selected_region: int,
    frame: CoordinateFrame,
    waypoints: list[Waypoint],
    last_saved: Path | None,
    xml_path: Path,
    selected_waypoint: int,
    active_input: str | None,
    input_text: str,
    selected_group: str | None,
    last_xml_saved: Path | None,
    last_region_saved: Path | None,
) -> tuple[list[Button], list[InputBox]]:
    x0 = camera.map_width
    pygame.draw.rect(surface, COLOR_PANEL, (x0, 0, camera.panel_width, camera.height))
    pygame.draw.line(surface, COLOR_PANEL_BORDER, (x0, 0), (x0, camera.height), 2)
    previous_clip = surface.get_clip()
    surface.set_clip(pygame.Rect(x0, 0, camera.panel_width, camera.height))
    x = x0 + 18
    y = 16 - panel_scroll
    buttons: list[Button] = []
    inputs: list[InputBox] = []

    draw_text(surface, font, "nav_tools", x, y)
    y += 31
    draw_text(surface, small_font, f"XML: {xml_path.name}", x, y, COLOR_CYAN)
    y += 20
    draw_text(surface, small_font, f"geoms: {len(geoms)}  hfield/gray removed", x, y, COLOR_MUTED)
    y += 24

    draw_text(surface, small_font, "Mode", x, y, COLOR_TEXT)
    y += 22
    mode_specs = [("Pan", "mode_pan"), ("Point", "mode_point"), ("Terrain", "mode_terrain"), ("Avoid", "mode_avoid")]
    for idx, (label_text, action) in enumerate(mode_specs):
        rect = pygame.Rect(x + (idx % 2) * 94, y + (idx // 2) * 30, 86, 25)
        color = (51, 65, 85) if action == f"mode_{tool_mode}" else (30, 41, 59)
        pygame.draw.rect(surface, color, rect, border_radius=5)
        pygame.draw.rect(surface, COLOR_GOLD if action == f"mode_{tool_mode}" else COLOR_PANEL_BORDER, rect, width=1, border_radius=5)
        surface.blit(small_font.render(label_text, True, COLOR_TEXT), (rect.x + 10, rect.y + 6))
        buttons.append(Button(rect, "", action))
    y += 66

    xml_btn = Button(pygame.Rect(x, y, 116, 26), "Switch XML (M)", "xml_toggle")
    xml_load_btn = Button(pygame.Rect(x + 126, y, 132, 26), "XML List", "toggle_xml")
    for btn in (xml_btn, xml_load_btn):
        btn.draw(surface, small_font)
        buttons.append(btn)
    y += 32
    save_xml_btn = Button(pygame.Rect(x, y, 126, 25), "Save New XML", "xml_save")
    save_xml_btn.draw(surface, small_font)
    buttons.append(save_xml_btn)
    if last_xml_saved:
        draw_text(surface, small_font, f"xml: {last_xml_saved.name[:28]}", x + 136, y + 5, COLOR_GREEN)
    y += 31
    if dropdown_open == "xml":
        for idx, path in enumerate(xml_files[:8]):
            item = pygame.Rect(x, y + idx * 25, camera.panel_width - 36, 24)
            color = (45, 55, 80) if idx == selected_xml else (30, 41, 59)
            pygame.draw.rect(surface, color, item)
            pygame.draw.rect(surface, COLOR_PANEL_BORDER, item, width=1)
            draw_text(surface, small_font, path.name[:38], item.x + 7, item.y + 5, COLOR_TEXT)
            buttons.append(Button(item, "", f"xml:{idx}"))
        y += max(1, min(len(xml_files), 8)) * 25 + 8

    draw_text(surface, small_font, "XML Terrain", x, y, COLOR_TEXT)
    y += 22
    selected_items = group_items(geoms, selected_group)
    if selected_group and selected_items:
        label = GROUP_LABELS.get(selected_group, selected_group)
        draw_text(surface, small_font, f"{label}  ({len(selected_items)} geoms)", x, y, COLOR_GOLD)
        y += 20
        for label_text, key, unit in (("X", "xml_group_x", "m"), ("Y", "xml_group_y", "m"), ("Yaw", "xml_group_yaw", "deg")):
            draw_text(surface, small_font, label_text, x, y + 5, COLOR_MUTED)
            draw_input(
                surface,
                small_font,
                pygame.Rect(x + 36, y, 120, 25),
                key,
                input_current_value(key, frame, waypoints, selected_waypoint, geoms, selected_group),
                active_input,
                input_text,
                inputs,
            )
            draw_text(surface, small_font, unit, x + 164, y + 5, COLOR_MUTED)
            y += 30
        clear_xml_btn = Button(pygame.Rect(x, y, 82, 25), "Clear", "xml_clear")
        for btn in (clear_xml_btn,):
            btn.draw(surface, small_font)
            buttons.append(btn)
        y += 30
    else:
        draw_text(surface, small_font, "click terrain to edit as a group", x, y, COLOR_MUTED)
        y += 22

    y += 8
    draw_text(surface, small_font, "PCD", x, y, COLOR_TEXT)
    dropdown = pygame.Rect(x, y + 20, camera.panel_width - 36, 28)
    pygame.draw.rect(surface, COLOR_PANEL_DARK, dropdown, border_radius=5)
    pygame.draw.rect(surface, COLOR_PANEL_BORDER, dropdown, width=1, border_radius=5)
    label = pcd_files[selected_pcd].name if pcd_files and selected_pcd >= 0 else "put .pcd files in pcd/"
    draw_text(surface, small_font, label[:38], dropdown.x + 8, dropdown.y + 7, COLOR_TEXT)
    draw_text(surface, small_font, "v", dropdown.right - 18, dropdown.y + 7, COLOR_MUTED)
    buttons.append(Button(dropdown, "", "toggle_pcd"))
    y += 58

    if dropdown_open == "pcd":
        for idx, path in enumerate(pcd_files[:8]):
            item = pygame.Rect(x, y + idx * 26, camera.panel_width - 36, 25)
            pygame.draw.rect(surface, (30, 41, 59), item)
            pygame.draw.rect(surface, COLOR_PANEL_BORDER, item, width=1)
            draw_text(surface, small_font, path.name[:38], item.x + 7, item.y + 6, COLOR_TEXT)
            buttons.append(Button(item, "", f"pcd:{idx}"))
        y += max(1, min(len(pcd_files), 8)) * 26 + 8

    pcd_status = "none"
    if cloud.path:
        pcd_status = f"{cloud.sampled_count}/{cloud.total_count} pts"
    if cloud.error:
        pcd_status = cloud.error[:36]
    draw_text(surface, small_font, pcd_status, x, y, COLOR_MUTED if not cloud.error else COLOR_WARNING)
    y += 34

    draw_text(surface, small_font, "Origin", x, y, COLOR_TEXT)
    y += 22
    mode = Button(pygame.Rect(x, y, 86, 26), f"mode: {frame.mode}", "origin_mode")
    mode.draw(surface, small_font)
    buttons.append(mode)
    fit = Button(pygame.Rect(x + 96, y, 68, 26), "Fit", "fit")
    fit.draw(surface, small_font)
    buttons.append(fit)
    save = Button(pygame.Rect(x + 174, y, 88, 26), "Save JSON", "save")
    save.draw(surface, small_font)
    buttons.append(save)
    y += 38

    rows = [
        ("X", "origin_x", "origin_x_minus", "origin_x_plus", "m"),
        ("Y", "origin_y", "origin_y_minus", "origin_y_plus", "m"),
        ("Yaw", "origin_yaw", "origin_yaw_minus", "origin_yaw_plus", "deg"),
    ]
    for label_text, key, minus_action, plus_action, unit in rows:
        draw_text(surface, small_font, f"{label_text}", x, y + 5, COLOR_MUTED)
        draw_input(
            surface,
            small_font,
            pygame.Rect(x + 44, y, 108, 25),
            key,
            input_current_value(key, frame, waypoints, selected_waypoint),
            active_input,
            input_text,
            inputs,
        )
        draw_text(surface, small_font, unit, x + 158, y + 5, COLOR_MUTED)
        minus = Button(pygame.Rect(x + 190, y, 34, 25), "-", minus_action)
        plus = Button(pygame.Rect(x + 230, y, 34, 25), "+", plus_action)
        minus.draw(surface, small_font)
        plus.draw(surface, small_font)
        buttons.extend([minus, plus])
        y += 32

    y += 8
    draw_text(surface, small_font, "Points", x, y, COLOR_TEXT)
    y += 22
    load_points = Button(pygame.Rect(x, y, 104, 25), "Load JSON", "toggle_points")
    load_points.draw(surface, small_font)
    buttons.append(load_points)
    point_label = point_files[selected_point_file].name if point_files and selected_point_file >= 0 else "no json"
    draw_text(surface, small_font, point_label[:28], x + 114, y + 5, COLOR_MUTED)
    y += 30
    if dropdown_open == "points":
        for idx, path in enumerate(point_files[:8]):
            item = pygame.Rect(x, y + idx * 25, camera.panel_width - 36, 24)
            color = (45, 55, 80) if idx == selected_point_file else (30, 41, 59)
            pygame.draw.rect(surface, color, item)
            pygame.draw.rect(surface, COLOR_PANEL_BORDER, item, width=1)
            draw_text(surface, small_font, path.name[:38], item.x + 7, item.y + 5, COLOR_TEXT)
            buttons.append(Button(item, "", f"points:{idx}"))
        y += max(1, min(len(point_files), 8)) * 25 + 8
    selected_label = "-"
    if 0 <= selected_waypoint < len(waypoints):
        selected_label = str(format_id(waypoints[selected_waypoint].id or selected_waypoint + 1))
    draw_text(surface, small_font, f"count: {len(waypoints)}  selected id: {selected_label}", x, y, COLOR_GOLD)
    y += 20
    if 0 <= selected_waypoint < len(waypoints):
        wp_rows = [
            ("ID", "wp_id", ""),
            ("X", "wp_x", "m"),
            ("Y", "wp_y", "m"),
            ("Yaw", "wp_yaw", "deg"),
            ("Tol", "wp_tolerance", "m"),
            ("Speed", "wp_speed", "m/s"),
            ("Policy", "wp_policy", ""),
            ("Task", "wp_task", ""),
        ]
        for label_text, key, unit in wp_rows:
            draw_text(surface, small_font, label_text, x, y + 5, COLOR_MUTED)
            draw_input(
                surface,
                small_font,
                pygame.Rect(x + 64, y, 128, 25),
                key,
                input_current_value(key, frame, waypoints, selected_waypoint),
                active_input,
                input_text,
                inputs,
            )
            if unit:
                draw_text(surface, small_font, unit, x + 200, y + 5, COLOR_MUTED)
            y += 30
        del_btn = Button(pygame.Rect(x, y, 82, 25), "Delete", "wp_delete")
        rough_btn = Button(pygame.Rect(x + 92, y, 76, 25), "rough", "wp_policy_rough")
        crawl_btn = Button(pygame.Rect(x + 178, y, 76, 25), "crawl", "wp_policy_crawl")
        wall_btn = Button(pygame.Rect(x + 264, y, 76, 25), "wall", "wp_policy_wall")
        for btn in (del_btn, rough_btn, crawl_btn, wall_btn):
            btn.draw(surface, small_font)
            buttons.append(btn)
        y += 31
        task_buttons = [
            ("none", "wp_task_none"),
            ("slalom", "wp_task_slalom"),
            ("gravel", "wp_task_gravel"),
            ("wall", "wp_task_wall"),
            ("low_bar", "wp_task_low_bar"),
            ("stairs_up", "wp_task_stairs_up"),
            ("stairs_down", "wp_task_stairs_down"),
            ("bridge_a", "wp_task_bridge_a"),
            ("bridge_b", "wp_task_bridge_b"),
            ("ramp", "wp_task_ramp"),
        ]
        for idx, (label_text, action) in enumerate(task_buttons):
            rect = pygame.Rect(x + (idx % 3) * 112, y + (idx // 3) * 28, 104, 24)
            btn = Button(rect, label_text, action)
            btn.draw(surface, small_font)
            buttons.append(btn)
        y += ((len(task_buttons) + 2) // 3) * 28 + 5
    else:
        draw_text(surface, small_font, "Ctrl+Left: add point", x, y, COLOR_MUTED)
        y += 18
        draw_text(surface, small_font, "Click point: select/edit", x, y, COLOR_MUTED)
        y += 18
        draw_text(surface, small_font, "S: save JSON", x, y, COLOR_MUTED)
        y += 26
    if last_saved:
        draw_text(surface, small_font, f"saved: {last_saved.name}", x, y, COLOR_GREEN)
        y += 20

    y += 6
    draw_text(surface, small_font, "Avoid Regions", x, y, COLOR_TEXT)
    y += 22
    region_label = selected_region + 1 if 0 <= selected_region < len(avoid_regions) else "-"
    draw_text(surface, small_font, f"regions: {len(avoid_regions)}  selected: {region_label}  current: {len(current_region)}", x, y, COLOR_GOLD)
    y += 24
    if 0 <= selected_region < len(avoid_regions):
        for label_text, key in (("ID", "region_id"), ("Name", "region_name"), ("Kind", "region_kind")):
            draw_text(surface, small_font, label_text, x, y + 5, COLOR_MUTED)
            draw_input(
                surface,
                small_font,
                pygame.Rect(x + 64, y, 150, 25),
                key,
                input_current_value(key, frame, waypoints, selected_waypoint, geoms, selected_group, avoid_regions, selected_region),
                active_input,
                input_text,
                inputs,
            )
            y += 30
    close_region = Button(pygame.Rect(x, y, 98, 25), "Close", "region_close")
    undo_region = Button(pygame.Rect(x + 108, y, 76, 25), "Undo", "region_undo")
    delete_region = Button(pygame.Rect(x + 194, y, 78, 25), "Delete", "region_delete")
    for btn in (close_region, undo_region, delete_region):
        btn.draw(surface, small_font)
        buttons.append(btn)
    y += 31
    save_region = Button(pygame.Rect(x, y, 126, 25), "Save All JSON", "region_save")
    save_region.draw(surface, small_font)
    buttons.append(save_region)
    if last_region_saved:
        draw_text(surface, small_font, f"json: {last_region_saved.name[:26]}", x + 136, y + 5, COLOR_GREEN)
    y += 32

    if panel_scroll > 0:
        pygame.draw.rect(surface, COLOR_PANEL, (x0 + camera.panel_width - 12, 0, 12, camera.height))
        thumb_h = max(40, int(camera.height * 0.25))
        thumb_y = min(camera.height - thumb_h - 8, 8 + int(panel_scroll * 0.16))
        pygame.draw.rect(surface, COLOR_PANEL_BORDER, (x0 + camera.panel_width - 8, thumb_y, 4, thumb_h), border_radius=2)
    surface.set_clip(previous_clip)
    return buttons, inputs


def apply_action(
    action: str,
    frame: CoordinateFrame,
    camera: MapCamera,
    geoms: list[Geom],
    cloud: PointCloud,
    pcd_files: list[Path],
    selected_pcd: int,
    waypoints: list[Waypoint],
    regions: list[AvoidRegion],
    xml_path: Path,
    selected_waypoint: int,
) -> tuple[str, int, PointCloud, Path | None]:
    dropdown_open = ""
    last_saved = None
    if action == "toggle_pcd":
        dropdown_open = "pcd"
    elif action.startswith("pcd:"):
        selected_pcd = int(action.split(":", 1)[1])
        cloud = load_pcd(pcd_files[selected_pcd])
        fit_camera(camera, geoms, cloud, frame)
    elif action == "origin_mode":
        frame.toggle()
    elif action == "fit":
        fit_camera(camera, geoms, cloud, frame)
    elif action == "save":
        last_saved = save_waypoints(waypoints, frame, xml_path, cloud, regions)
    elif action == "origin_x_minus":
        frame.origin_x -= 0.05
    elif action == "origin_x_plus":
        frame.origin_x += 0.05
    elif action == "origin_y_minus":
        frame.origin_y -= 0.05
    elif action == "origin_y_plus":
        frame.origin_y += 0.05
    elif action == "origin_yaw_minus":
        frame.origin_yaw -= math.radians(1.0)
    elif action == "origin_yaw_plus":
        frame.origin_yaw += math.radians(1.0)
    elif action == "wp_delete" and 0 <= selected_waypoint < len(waypoints):
        waypoints.pop(selected_waypoint)
    elif action == "wp_policy_rough" and 0 <= selected_waypoint < len(waypoints):
        waypoints[selected_waypoint].policy = "rough"
    elif action == "wp_policy_crawl" and 0 <= selected_waypoint < len(waypoints):
        waypoints[selected_waypoint].policy = "crawl"
    elif action == "wp_policy_wall" and 0 <= selected_waypoint < len(waypoints):
        waypoints[selected_waypoint].policy = "wall"
    elif action.startswith("wp_task_") and 0 <= selected_waypoint < len(waypoints):
        waypoints[selected_waypoint].task = normalize_task_name(action.removeprefix("wp_task_"))
    return dropdown_open, selected_pcd, cloud, last_saved


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pygame nav map, PCD, origin, and point JSON tool.")
    parser.add_argument("--width", type=int, default=1280, help="Window width.")
    parser.add_argument("--height", type=int, default=820, help="Window height.")
    parser.add_argument("--refresh-xml", action="store_true", help="Kept for compatibility; local XML is used as-is.")
    parser.add_argument("--copy-pcd", type=Path, help="Copy a PCD into pcd/ and exit.")
    return parser.parse_args(argv)


def copy_pcd_into_tool(path: Path) -> Path:
    ensure_tool_dirs()
    target = PCD_DIR / path.name
    shutil.copy2(path, target)
    return target


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    ensure_tool_dirs()
    if args.copy_pcd:
        target = copy_pcd_into_tool(args.copy_pcd)
        print(f"Copied PCD to {target}")
        return 0

    ensure_local_xml(force=args.refresh_xml)
    xml_paths = scan_xml_files()
    selected_xml = 0
    xml_path = xml_paths[selected_xml]
    xml_tree, geoms = load_xml_scene(xml_path)
    pcd_files = scan_pcd_files()
    selected_pcd = 0 if pcd_files else -1
    point_files = scan_point_files()
    selected_point_file = 0 if point_files else -1
    cloud = load_pcd(pcd_files[selected_pcd]) if selected_pcd >= 0 else PointCloud()
    frame = CoordinateFrame()
    waypoints: list[Waypoint] = []
    selected_waypoint = -1
    selected_group: str | None = None
    last_saved: Path | None = None
    last_xml_saved: Path | None = None
    last_region_saved: Path | None = None
    tool_mode = "pan"
    avoid_regions: list[AvoidRegion] = []
    current_region: list[tuple[float, float]] = []
    selected_region = -1
    dropdown_open = ""
    active_input: str | None = None
    input_text = ""
    panel_scroll = 0

    pygame.init()
    pygame.font.init()
    pygame.key.start_text_input()
    screen = pygame.display.set_mode((args.width, args.height), pygame.RESIZABLE)
    pygame.display.set_caption("nav_tools - map / pcd / points")
    title_font = get_font(18, bold=True)
    small_font = get_font(13)
    map_font = get_font(12)
    camera = MapCamera(args.width, args.height)
    fit_camera(camera, geoms, cloud, frame)

    dragging = False
    terrain_drag_group: str | None = None
    terrain_drag_last_view: tuple[float, float] | None = None
    last_mouse = (0, 0)
    buttons: list[Button] = []
    input_boxes: list[InputBox] = []
    clock = pygame.time.Clock()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                camera.width, camera.height = event.w, event.h
                screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
                panel_scroll = min(panel_scroll, 900)
            elif event.type == pygame.TEXTINPUT and active_input:
                input_text += event.text
            elif event.type == pygame.MOUSEWHEEL:
                mouse_pos = pygame.mouse.get_pos()
                if mouse_pos[0] >= camera.map_width:
                    panel_scroll = max(0, min(1100, panel_scroll - event.y * 70))
                elif event.y > 0:
                    camera.zoom_at(1.12, mouse_pos)
                elif event.y < 0:
                    camera.zoom_at(1.0 / 1.12, mouse_pos)
            elif event.type == pygame.KEYDOWN:
                if active_input:
                    if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                        selected_waypoint, selected_region = commit_input_value(active_input, input_text, frame, waypoints, selected_waypoint, geoms, selected_group, avoid_regions, selected_region)
                        active_input, input_text = clear_input()
                    elif event.key == pygame.K_TAB:
                        selected_waypoint, selected_region = commit_input_value(active_input, input_text, frame, waypoints, selected_waypoint, geoms, selected_group, avoid_regions, selected_region)
                        active_input = next_input_key(active_input, input_boxes)
                        input_text = input_current_value(active_input, frame, waypoints, selected_waypoint, geoms, selected_group, avoid_regions, selected_region) if active_input else ""
                    elif event.key == pygame.K_ESCAPE:
                        active_input, input_text = clear_input()
                    elif event.key == pygame.K_BACKSPACE:
                        input_text = input_text[:-1]
                    else:
                        char = key_to_input_char(event)
                        if char and not event.unicode:
                            input_text += char
                elif event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_m:
                    xml_paths = scan_xml_files()
                    selected_xml = (selected_xml + 1) % len(xml_paths)
                    xml_path = xml_paths[selected_xml]
                    xml_tree, geoms = load_xml_scene(xml_path)
                    selected_group = None
                    selected_region = -1
                    dropdown_open = ""
                    fit_camera(camera, geoms, cloud, frame)
                elif event.key == pygame.K_f:
                    fit_camera(camera, geoms, cloud, frame)
                elif event.key == pygame.K_o:
                    frame.toggle()
                elif event.key == pygame.K_s:
                    last_saved = save_waypoints(waypoints, frame, xml_path, cloud, avoid_regions)
                    last_region_saved = last_saved
                elif event.key == pygame.K_BACKSPACE and tool_mode == "avoid" and current_region:
                    current_region.pop()
                elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER) and tool_mode == "avoid":
                    if len(current_region) >= 3:
                        avoid_regions.append(AvoidRegion(f"avoid_{len(avoid_regions) + 1}", current_region[:]))
                        selected_region = len(avoid_regions) - 1
                        current_region.clear()
                elif event.key == pygame.K_DELETE and tool_mode == "avoid" and avoid_regions:
                    delete_index = selected_region if 0 <= selected_region < len(avoid_regions) else len(avoid_regions) - 1
                    avoid_regions.pop(delete_index)
                    selected_region = min(delete_index, len(avoid_regions) - 1)
                elif event.key == pygame.K_BACKSPACE and waypoints:
                    if 0 <= selected_waypoint < len(waypoints):
                        waypoints.pop(selected_waypoint)
                        normalize_waypoint_ids(waypoints)
                        selected_waypoint = min(selected_waypoint, len(waypoints) - 1)
                    else:
                        waypoints.pop()
                        normalize_waypoint_ids(waypoints)
                    selected_region = -1
                elif event.key == pygame.K_z:
                    frame.origin_x -= 0.05
                elif event.key == pygame.K_x:
                    frame.origin_x += 0.05
                elif event.key == pygame.K_t:
                    frame.origin_y -= 0.05
                elif event.key == pygame.K_y:
                    frame.origin_y += 0.05
                elif event.key == pygame.K_a:
                    frame.origin_yaw -= math.radians(1.0)
                elif event.key == pygame.K_d:
                    frame.origin_yaw += math.radians(1.0)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                clicked_button = None
                for button in buttons:
                    if button.rect.collidepoint(event.pos):
                        clicked_button = button.action
                        break
                clicked_input = None
                for box in input_boxes:
                    if box.rect.collidepoint(event.pos):
                        clicked_input = box.key
                        break

                if event.button == 1 and event.pos[0] < camera.map_width and tool_mode == "terrain":
                    group_hit = select_group_at(event.pos, camera, geoms)
                    if group_hit is not None:
                        terrain_drag_group = group_hit
                        terrain_drag_last_view = camera.screen_to_view(event.pos[0], event.pos[1])
                        selected_group = group_hit
                        selected_waypoint = -1
                        selected_region = -1
                        active_input, input_text = clear_input()
                        dropdown_open = ""
                    else:
                        selected_group = None
                elif event.button == 1 and clicked_input:
                    if active_input:
                        selected_waypoint, selected_region = commit_input_value(active_input, input_text, frame, waypoints, selected_waypoint, geoms, selected_group, avoid_regions, selected_region)
                    active_input, input_text = activate_input(clicked_input, frame, waypoints, selected_waypoint, geoms, selected_group, avoid_regions, selected_region)
                elif event.button == 1 and clicked_button == "xml_toggle":
                    xml_paths = scan_xml_files()
                    selected_xml = (selected_xml + 1) % len(xml_paths)
                    xml_path = xml_paths[selected_xml]
                    xml_tree, geoms = load_xml_scene(xml_path)
                    selected_group = None
                    selected_region = -1
                    dropdown_open = ""
                    fit_camera(camera, geoms, cloud, frame)
                elif event.button == 1 and clicked_button == "toggle_xml":
                    dropdown_open = "" if dropdown_open == "xml" else "xml"
                elif event.button == 1 and clicked_button and clicked_button.startswith("xml:"):
                    if active_input:
                        selected_waypoint, selected_region = commit_input_value(active_input, input_text, frame, waypoints, selected_waypoint, geoms, selected_group, avoid_regions, selected_region)
                        active_input, input_text = clear_input()
                    selected_xml = int(clicked_button.split(":", 1)[1])
                    xml_path = xml_paths[selected_xml]
                    xml_tree, geoms = load_xml_scene(xml_path)
                    selected_group = None
                    selected_region = -1
                    dropdown_open = ""
                    fit_camera(camera, geoms, cloud, frame)
                elif event.button == 1 and clicked_button == "xml_save":
                    if active_input:
                        selected_waypoint, selected_region = commit_input_value(active_input, input_text, frame, waypoints, selected_waypoint, geoms, selected_group, avoid_regions, selected_region)
                        active_input, input_text = clear_input()
                    last_xml_saved = save_xml_as_new(xml_tree, xml_path)
                    xml_paths = scan_xml_files()
                    if last_xml_saved in xml_paths:
                        selected_xml = xml_paths.index(last_xml_saved)
                        xml_path = last_xml_saved
                elif event.button == 1 and clicked_button == "xml_clear":
                    selected_group = None
                elif event.button == 1 and clicked_button and clicked_button.startswith("mode_"):
                    tool_mode = clicked_button.removeprefix("mode_")
                    dragging = False
                    terrain_drag_group = None
                    terrain_drag_last_view = None
                    if tool_mode != "avoid":
                        selected_region = -1
                    dropdown_open = ""
                elif event.button == 1 and clicked_button == "region_close":
                    if len(current_region) >= 3:
                        avoid_regions.append(AvoidRegion(f"avoid_{len(avoid_regions) + 1}", current_region[:]))
                        selected_region = len(avoid_regions) - 1
                        current_region.clear()
                elif event.button == 1 and clicked_button == "region_undo":
                    if current_region:
                        current_region.pop()
                elif event.button == 1 and clicked_button == "region_delete":
                    if avoid_regions:
                        delete_index = selected_region if 0 <= selected_region < len(avoid_regions) else len(avoid_regions) - 1
                        avoid_regions.pop(delete_index)
                        selected_region = min(delete_index, len(avoid_regions) - 1)
                    current_region.clear()
                elif event.button == 1 and clicked_button == "region_save":
                    last_saved = save_waypoints(waypoints, frame, xml_path, cloud, avoid_regions)
                    last_region_saved = last_saved
                elif event.button == 1 and clicked_button == "toggle_points":
                    point_files = scan_point_files()
                    if selected_point_file >= len(point_files):
                        selected_point_file = 0 if point_files else -1
                    dropdown_open = "" if dropdown_open == "points" else "points"
                elif event.button == 1 and clicked_button and clicked_button.startswith("points:"):
                    if active_input:
                        selected_waypoint, selected_region = commit_input_value(active_input, input_text, frame, waypoints, selected_waypoint, geoms, selected_group, avoid_regions, selected_region)
                        active_input, input_text = clear_input()
                    selected_point_file = int(clicked_button.split(":", 1)[1])
                    try:
                        waypoints[:] = load_waypoints_json(point_files[selected_point_file], frame)
                        avoid_regions[:] = load_avoid_regions_json(point_files[selected_point_file])
                        current_region.clear()
                        selected_waypoint = 0 if waypoints else -1
                        selected_region = 0 if avoid_regions else -1
                        last_saved = point_files[selected_point_file]
                        last_region_saved = point_files[selected_point_file] if avoid_regions else last_region_saved
                    except Exception:
                        selected_waypoint = -1
                    selected_group = None
                    dropdown_open = ""
                elif event.button == 1 and clicked_button:
                    if active_input:
                        selected_waypoint, selected_region = commit_input_value(active_input, input_text, frame, waypoints, selected_waypoint, geoms, selected_group, avoid_regions, selected_region)
                        active_input, input_text = clear_input()
                    dropdown_open, selected_pcd, cloud, saved = apply_action(
                        clicked_button,
                        frame,
                        camera,
                        geoms,
                        cloud,
                        pcd_files,
                        selected_pcd,
                        waypoints,
                        avoid_regions,
                        xml_path,
                        selected_waypoint,
                    )
                    normalize_waypoint_ids(waypoints)
                    selected_waypoint = min(selected_waypoint, len(waypoints) - 1)
                    if saved:
                        last_saved = saved
                elif event.button == 1 and event.pos[0] < camera.map_width:
                    if tool_mode == "point":
                        wp_hit = select_waypoint_at(event.pos, camera, waypoints)
                        if wp_hit is not None:
                            selected_waypoint = wp_hit
                            selected_group = None
                            selected_region = -1
                            active_input, input_text = clear_input()
                            dropdown_open = ""
                        else:
                            vx, vy = camera.screen_to_view(event.pos[0], event.pos[1])
                            wx, wy = frame.view_to_world(vx, vy)
                            yaw = default_yaw_for_new_waypoint(vx, vy, waypoints)
                            insert_at = selected_waypoint + 1 if 0 <= selected_waypoint < len(waypoints) else len(waypoints)
                            waypoints.insert(
                                insert_at,
                                Waypoint(
                                    x=vx,
                                    y=vy,
                                    world_x=wx,
                                    world_y=wy,
                                    origin_mode=frame.mode,
                                    id=new_waypoint_id(waypoints, insert_at),
                                    yawDeg=yaw,
                                ),
                            )
                            normalize_waypoint_ids(waypoints)
                            selected_waypoint = insert_at
                            selected_group = None
                            selected_region = -1
                            dropdown_open = ""
                    elif tool_mode == "avoid":
                        hit_region = select_avoid_region_at(event.pos, camera, avoid_regions)
                        selected_waypoint = -1
                        selected_group = None
                        dropdown_open = ""
                        if hit_region is not None:
                            selected_region = hit_region
                            current_region.clear()
                            active_input, input_text = clear_input()
                        else:
                            vx, vy = camera.screen_to_view(event.pos[0], event.pos[1])
                            current_region.append((vx, vy))
                            selected_region = -1
                    else:
                        dragging = True
                        last_mouse = event.pos
                        dropdown_open = ""
                elif event.button in (4, 5) and event.pos[0] >= camera.map_width:
                    delta = -70 if event.button == 4 else 70
                    panel_scroll = max(0, min(1100, panel_scroll + delta))
                elif event.button == 4 and event.pos[0] < camera.map_width:
                    camera.zoom_at(1.12, event.pos)
                elif event.button == 5 and event.pos[0] < camera.map_width:
                    camera.zoom_at(1.0 / 1.12, event.pos)
                else:
                    if active_input:
                        selected_waypoint, selected_region = commit_input_value(active_input, input_text, frame, waypoints, selected_waypoint, geoms, selected_group, avoid_regions, selected_region)
                        active_input, input_text = clear_input()
                    dropdown_open = ""
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    dragging = False
                    terrain_drag_group = None
                    terrain_drag_last_view = None
                elif event.button == 3:
                    terrain_drag_group = None
                    terrain_drag_last_view = None
            elif event.type == pygame.MOUSEMOTION and terrain_drag_group and terrain_drag_last_view:
                current_view = camera.screen_to_view(event.pos[0], event.pos[1])
                dx = current_view[0] - terrain_drag_last_view[0]
                dy = current_view[1] - terrain_drag_last_view[1]
                center = group_center(geoms, terrain_drag_group)
                if center is not None:
                    move_group_center(geoms, terrain_drag_group, x=center[0] + dx, y=center[1] + dy)
                terrain_drag_last_view = current_view
            elif event.type == pygame.MOUSEMOTION and dragging:
                dx = event.pos[0] - last_mouse[0]
                dy = event.pos[1] - last_mouse[1]
                camera.pan_x += dx
                camera.pan_y += dy
                last_mouse = event.pos

        screen.fill(COLOR_BG)
        pygame.draw.rect(screen, COLOR_BG, (0, 0, camera.map_width, camera.height))
        draw_grid(screen, camera, frame, map_font)
        draw_pcd(screen, camera, frame, cloud)
        draw_geoms(screen, camera, geoms, selected_group)
        draw_avoid_regions(screen, camera, avoid_regions, current_region, selected_region)
        draw_waypoints(screen, camera, frame, waypoints, map_font, selected_waypoint)
        buttons, input_boxes = draw_panel(
            screen,
            camera,
            title_font,
            small_font,
            panel_scroll,
            geoms,
            xml_paths,
            selected_xml,
            cloud,
            pcd_files,
            selected_pcd,
            point_files,
            selected_point_file,
            dropdown_open,
            tool_mode,
            avoid_regions,
            current_region,
            selected_region,
            frame,
            waypoints,
            last_saved,
            xml_path,
            selected_waypoint,
            active_input,
            input_text,
            selected_group,
            last_xml_saved,
            last_region_saved,
        )

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
