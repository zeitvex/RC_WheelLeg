#!/usr/bin/env python3
"""Mirror a nav_tools terrain XML and route JSON together."""

from __future__ import annotations

import argparse
import copy
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


TOOL_DIR = Path(__file__).resolve().parent
XML_DIR = TOOL_DIR / "xml"
POINTS_DIR = TOOL_DIR / "points"
DEFAULT_XML = XML_DIR / "1hao.xml"
DEFAULT_POINTS = POINTS_DIR / "points_20260715_120154.json"


def format_float(value: float) -> str:
    if abs(value) < 0.0000005:
        value = 0.0
    return f"{value:.6f}".rstrip("0").rstrip(".")


def parse_float_tuple(text: str | None) -> tuple[float, ...]:
    if not text:
        return ()
    return tuple(float(item) for item in text.split())


def mirror_coord(value: float, axis_value: float) -> float:
    return 2.0 * axis_value - value


def normalize_yaw_deg(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value <= -180.0:
        value += 360.0
    return value


def mirror_yaw_deg(value: float, axis: str) -> float:
    if axis == "x":
        return normalize_yaw_deg(180.0 - value)
    return normalize_yaw_deg(-value)


def mirror_quat(quat: tuple[float, ...], axis: str) -> tuple[float, float, float, float]:
    if axis == "x":
        return quat[0], quat[1], -quat[2], quat[3]
    return quat[0], -quat[1], quat[2], quat[3]


def mirror_xml(source: Path, target: Path, axis: str, axis_value: float) -> None:
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(source, parser=parser)
    mirrored = copy.deepcopy(tree)
    root = mirrored.getroot()
    model_name = root.get("model", source.stem)
    if "mirror" not in model_name.lower():
        root.set("model", f"{model_name} mirror")

    for element in root.iter():
        pos = parse_float_tuple(element.get("pos"))
        if len(pos) >= 1:
            mirrored_pos = list(pos)
            coord_index = 0 if axis == "x" else 1
            if len(mirrored_pos) > coord_index:
                mirrored_pos[coord_index] = mirror_coord(mirrored_pos[coord_index], axis_value)
            element.set("pos", " ".join(format_float(value) for value in mirrored_pos))

        center = parse_float_tuple(element.get("center"))
        if len(center) >= 1:
            mirrored_center = list(center)
            coord_index = 0 if axis == "x" else 1
            if len(mirrored_center) > coord_index:
                mirrored_center[coord_index] = mirror_coord(mirrored_center[coord_index], axis_value)
            element.set("center", " ".join(format_float(value) for value in mirrored_center))

        quat = parse_float_tuple(element.get("quat"))
        if len(quat) == 4:
            mirrored_quat = mirror_quat(quat, axis)
            element.set("quat", " ".join(format_float(value) for value in mirrored_quat))

    ET.indent(mirrored, space="  ")
    target.parent.mkdir(parents=True, exist_ok=True)
    mirrored.write(target, encoding="utf-8", xml_declaration=False)


def mirror_point_fields(row: dict[str, Any], axis: str, axis_value: float) -> None:
    coord_keys = ("x", "world_x") if axis == "x" else ("y", "world_y")
    for key in coord_keys:
        if key in row and isinstance(row[key], (int, float)):
            row[key] = mirror_coord(float(row[key]), axis_value)
    if "yawDeg" in row and isinstance(row["yawDeg"], (int, float)):
        row["yawDeg"] = mirror_yaw_deg(float(row["yawDeg"]), axis)
    if "yaw" in row and isinstance(row["yaw"], (int, float)):
        row["yaw"] = mirror_yaw_deg(float(row["yaw"]), axis)


def mirror_polygon(points: Any, axis: str, axis_value: float) -> None:
    if not isinstance(points, list):
        return
    for point in points:
        if isinstance(point, dict):
            key = "x" if axis == "x" else "y"
            if isinstance(point.get(key), (int, float)):
                point[key] = mirror_coord(float(point[key]), axis_value)
        elif isinstance(point, list):
            coord_index = 0 if axis == "x" else 1
            if len(point) > coord_index and isinstance(point[coord_index], (int, float)):
                point[coord_index] = mirror_coord(float(point[coord_index]), axis_value)


def mirror_regions(payload: dict[str, Any], axis: str, axis_value: float) -> None:
    for key in ("regions", "avoid_regions"):
        rows = payload.get(key)
        if not isinstance(rows, list):
            continue
        for region in rows:
            if isinstance(region, dict):
                mirror_polygon(region.get("polygon", region.get("points")), axis, axis_value)


def mirror_waypoint_rows(rows: Any, axis: str, axis_value: float) -> None:
    if not isinstance(rows, list):
        return
    for row in rows:
        if isinstance(row, dict):
            mirror_point_fields(row, axis, axis_value)


def mirror_points_json(source: Path, target: Path, axis: str, axis_value: float, mirrored_xml: Path | None) -> None:
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"Expected top-level JSON object: {source}")

    payload["name"] = f"{payload.get('name', source.stem)}_mirror"
    payload["mirroredFrom"] = source.name
    payload["mirror"] = {
        "axis": axis,
        f"axis_{axis}": axis_value,
        "createdAt": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    if mirrored_xml is not None:
        payload["xml"] = f"xml/{mirrored_xml.name}"

    origin = payload.get("origin")
    if isinstance(origin, dict):
        coord_key = "x" if axis == "x" else "y"
        if isinstance(origin.get(coord_key), (int, float)):
            origin[coord_key] = mirror_coord(float(origin[coord_key]), axis_value)
        if isinstance(origin.get("yaw_deg"), (int, float)):
            origin["yaw_deg"] = mirror_yaw_deg(float(origin["yaw_deg"]), axis)
            origin["yaw_rad"] = math.radians(float(origin["yaw_deg"]))
        elif isinstance(origin.get("yaw_rad"), (int, float)):
            origin["yaw_rad"] = math.radians(mirror_yaw_deg(math.degrees(float(origin["yaw_rad"])), axis))
            origin["yaw_deg"] = math.degrees(float(origin["yaw_rad"]))

    mirror_waypoint_rows(payload.get("waypoints"), axis, axis_value)
    segments = payload.get("segments")
    if isinstance(segments, list):
        for segment in segments:
            if isinstance(segment, dict):
                mirror_waypoint_rows(segment.get("waypoints"), axis, axis_value)

    mirror_regions(payload, axis, axis_value)

    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def default_output_path(source: Path, suffix: str) -> Path:
    return source.with_name(f"{source.stem}{suffix}{source.suffix}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mirror a nav_tools XML and matching points JSON.")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML, help="Input XML path.")
    parser.add_argument("--points", type=Path, default=DEFAULT_POINTS, help="Input points JSON path.")
    parser.add_argument("--mirror-axis", choices=("x", "y"), default="x", help="Coordinate axis to mirror.")
    parser.add_argument("--axis-x", type=float, default=0.0, help="Mirror axis x value. Default: 0.")
    parser.add_argument("--axis-y", type=float, default=0.0, help="Mirror axis y value.")
    parser.add_argument("--out-xml", type=Path, help="Output XML path. Default: <input>_mirror.xml.")
    parser.add_argument("--out-points", type=Path, help="Output JSON path. Default: <input>_mirror.json.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    xml_path = args.xml if args.xml.is_absolute() else TOOL_DIR / args.xml
    points_path = args.points if args.points.is_absolute() else TOOL_DIR / args.points
    out_xml = args.out_xml if args.out_xml else default_output_path(xml_path, "_mirror")
    out_points = args.out_points if args.out_points else default_output_path(points_path, "_mirror")
    if not out_xml.is_absolute():
        out_xml = TOOL_DIR / out_xml
    if not out_points.is_absolute():
        out_points = TOOL_DIR / out_points

    axis_value = args.axis_x if args.mirror_axis == "x" else args.axis_y
    mirror_xml(xml_path, out_xml, args.mirror_axis, axis_value)
    mirror_points_json(points_path, out_points, args.mirror_axis, axis_value, out_xml)
    print(f"XML: {out_xml}")
    print(f"JSON: {out_points}")
    print(f"axis_{args.mirror_axis}: {axis_value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
