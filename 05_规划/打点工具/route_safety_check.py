#!/usr/bin/env python3
"""Offline route safety checker for nav_tools waypoint JSON files.

The checker treats avoid regions as hard no-go polygons and validates the
route centerline with a circular robot footprint. It is intentionally light on
dependencies so it can run on the robot laptop without ROS, pygame, or shapely.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROBOT_BODY_LENGTH = 0.356
ROBOT_BODY_WIDTH = 0.235
ROBOT_BODY_CENTER_X = 0.1518
ROBOT_ORIGIN_FROM_FRONT = 0.105
ROBOT_WHEEL_VIS_LENGTH = 0.16
ROBOT_WHEEL_VIS_WIDTH = 0.055
ROBOT_POSE_HIP = 0.550
ROBOT_POSE_KNEE = -1.125
PCD_ROBOT_RADIUS = 0.18
ROBOT_FOOTPRINT_PADDING = 0.03


@dataclass(frozen=True)
class Waypoint:
    index: int
    id: str
    x: float
    y: float
    yaw_deg: float
    speed: float | None
    policy: str
    tolerance: float | None
    slalom_straight: bool = False
    slalom_script_break: bool = False
    slalom_script_pos_tolerance: float | None = None
    exact_reach: bool = False
    precision_follow: bool = False
    require_yaw: bool = False
    yaw_tolerance_deg: float | None = None
    stable_cycles: int | None = None
    mandatory_cross: bool = False
    mandatory_radius: float | None = None
    mandatory_center_x: float | None = None
    mandatory_center_y: float | None = None


@dataclass(frozen=True)
class AvoidRegion:
    name: str
    kind: str
    polygon: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class SegmentRisk:
    start_id: str
    end_id: str
    region: str
    clearance_m: float
    required_m: float
    margin_m: float
    length_m: float
    centerline_intersects: bool

    @property
    def status(self) -> str:
        if self.centerline_intersects:
            return "INTERSECT"
        if self.margin_m < 0.0:
            return "VIOLATION"
        if self.margin_m < 0.05:
            return "TIGHT"
        return "OK"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check nav_tools waypoint routes against avoid/no-go polygons."
    )
    parser.add_argument(
        "--points",
        type=Path,
        default=Path("tools/nav_tools/points/points_20260715_120154.json"),
        help="Route JSON exported by nav_map_viewer.",
    )
    parser.add_argument(
        "--xml",
        type=Path,
        default=Path("tools/nav_tools/xml/1hao.xml"),
        help="Optional MuJoCo terrain XML used for metadata checks.",
    )
    parser.add_argument(
        "--onnx",
        type=Path,
        default=Path("model_6800.onnx"),
        help="Optional ONNX policy path used for input/output shape reporting.",
    )
    parser.add_argument(
        "--footprint-radius",
        type=float,
        default=None,
        help="Robot circular footprint radius in meters. Defaults to sim2real lateral footprint.",
    )
    parser.add_argument(
        "--avoid-margin",
        type=float,
        default=0.05,
        help="Extra clearance added outside the robot footprint.",
    )
    parser.add_argument(
        "--warn-margin",
        type=float,
        default=0.05,
        help="Report a TIGHT warning when spare margin is below this value.",
    )
    parser.add_argument(
        "--top",
        type=int,
        default=20,
        help="Number of closest segment-region pairs to print.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional machine-readable report path.",
    )
    parser.add_argument(
        "--allow-violations",
        action="store_true",
        help="Exit with code 0 even when violations are detected.",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return data


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_waypoints(payload: dict[str, Any]) -> list[Waypoint]:
    rows = None
    if isinstance(payload.get("segments"), list) and payload["segments"]:
        rows = []
        for segment in payload["segments"]:
            if isinstance(segment, dict) and isinstance(segment.get("waypoints"), list):
                rows.extend(segment["waypoints"])
    if rows is None:
        rows = payload.get("waypoints")
    if not isinstance(rows, list):
        raise ValueError("Route JSON has no top-level waypoints or segments[].waypoints")

    waypoints: list[Waypoint] = []
    for index, row in enumerate(rows, start=1):
        if not isinstance(row, dict):
            continue
        x = float(row.get("world_x", row.get("x", 0.0)))
        y = float(row.get("world_y", row.get("y", 0.0)))
        yaw = float(row.get("yawDeg", row.get("yaw_deg", row.get("yaw", 0.0))))
        speed = row.get("speed")
        tolerance = row.get("tolerance")
        waypoints.append(
            Waypoint(
                index=index,
                id=str(row.get("id", index)),
                x=x,
                y=y,
                yaw_deg=yaw,
                speed=float(speed) if speed is not None else None,
                policy=str(row.get("policy", "")),
                tolerance=float(tolerance) if tolerance is not None else None,
                slalom_straight=bool(row.get("slalom_straight", row.get("slalomStraight", False))),
                slalom_script_break=bool(
                    row.get("slalom_script_break", row.get("slalomScriptBreak", False))
                ),
                slalom_script_pos_tolerance=_optional_float(
                    row.get(
                        "slalom_script_pos_tolerance",
                        row.get("slalomScriptPosTolerance", row.get("scriptTolerance")),
                    )
                ),
                exact_reach=bool(row.get("exact_reach", row.get("exactReach", False))),
                precision_follow=bool(row.get("precision_follow", row.get("precisionFollow", False))),
                require_yaw=bool(row.get("require_yaw", row.get("requireYaw", False))),
                yaw_tolerance_deg=_optional_float(
                    row.get("yaw_tolerance_deg", row.get("yawToleranceDeg"))
                ),
                stable_cycles=(
                    int(row.get("stable_cycles", row.get("stableCycles")))
                    if row.get("stable_cycles", row.get("stableCycles")) is not None
                    else None
                ),
                mandatory_cross=bool(row.get("mandatory_cross", row.get("mandatoryCross", False))),
                mandatory_radius=_optional_float(
                    row.get("mandatory_radius", row.get("mandatoryRadius"))
                ),
                mandatory_center_x=_optional_float(
                    row.get("mandatory_center_x", row.get("mandatoryCenterX"))
                ),
                mandatory_center_y=_optional_float(
                    row.get("mandatory_center_y", row.get("mandatoryCenterY"))
                ),
            )
        )
    if len(waypoints) < 2:
        raise ValueError("Route must contain at least two waypoints")
    return waypoints


def load_regions(payload: dict[str, Any]) -> list[AvoidRegion]:
    rows = payload.get("regions", payload.get("avoid_regions", []))
    if not isinstance(rows, list):
        return []

    regions: list[AvoidRegion] = []
    for index, row in enumerate(rows, start=1):
        if not isinstance(row, dict):
            continue
        polygon_rows = row.get("polygon", row.get("points", []))
        if not isinstance(polygon_rows, list):
            continue
        polygon: list[tuple[float, float]] = []
        for point in polygon_rows:
            if isinstance(point, dict):
                polygon.append((float(point.get("x", 0.0)), float(point.get("y", 0.0))))
            elif isinstance(point, (list, tuple)) and len(point) >= 2:
                polygon.append((float(point[0]), float(point[1])))
        if len(polygon) >= 3:
            regions.append(
                AvoidRegion(
                    name=str(row.get("name", f"avoid_{index}")),
                    kind=str(row.get("kind", "avoid")),
                    polygon=tuple(polygon),
                )
            )
    return regions


def robot_wheel_local_points(body_center_offset_x: float) -> list[tuple[float, float]]:
    thigh_dx = -0.25 * math.sin(ROBOT_POSE_HIP)
    shank_dx = -0.2 * math.sin(ROBOT_POSE_HIP + ROBOT_POSE_KNEE)
    wheel_positions = (
        ((0.32826 + 0.06389) - ROBOT_BODY_CENTER_X, 0.066172 - 0.027344, 0.1035, 0.014699, 0.04074, 0.0),
        ((0.32826 + 0.06389) - ROBOT_BODY_CENTER_X, -0.065853 + 0.027311, -0.1035, -0.018447, -0.040735, -0.00075079),
        ((-0.024743 - 0.06389) - ROBOT_BODY_CENTER_X, 0.066141 - 0.027309, 0.099459, 0.012475, 0.040737, 0.0),
        ((-0.024743 - 0.06389) - ROBOT_BODY_CENTER_X, -0.065884 + 0.027341, -0.099408, -0.012435, -0.040737, -0.00075079),
    )
    return [
        (
            body_center_offset_x + pitch_x + knee_x + thigh_dx + shank_dx,
            pitch_y + knee_y + wheel_y + wheel_geom_y,
        )
        for pitch_x, pitch_y, knee_y, wheel_y, wheel_geom_y, knee_x in wheel_positions
    ]


def default_lateral_footprint_radius() -> float:
    half_width = ROBOT_BODY_WIDTH * 0.5
    radius = max(PCD_ROBOT_RADIUS, half_width)
    body_center_offset_x = ROBOT_ORIGIN_FROM_FRONT - ROBOT_BODY_LENGTH * 0.5
    for _, wheel_y in robot_wheel_local_points(body_center_offset_x):
        radius = max(radius, abs(wheel_y) + ROBOT_WHEEL_VIS_WIDTH * 0.5)
    return radius + ROBOT_FOOTPRINT_PADDING


def point_segment_distance(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    qx = ax + t * dx
    qy = ay + t * dy
    return math.hypot(px - qx, py - qy)


def orientation(
    ax: float,
    ay: float,
    bx: float,
    by: float,
    cx: float,
    cy: float,
) -> float:
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def on_segment(
    ax: float,
    ay: float,
    bx: float,
    by: float,
    cx: float,
    cy: float,
) -> bool:
    return (
        min(ax, bx) - 1.0e-9 <= cx <= max(ax, bx) + 1.0e-9
        and min(ay, by) - 1.0e-9 <= cy <= max(ay, by) + 1.0e-9
        and abs(orientation(ax, ay, bx, by, cx, cy)) <= 1.0e-9
    )


def segments_intersect(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> bool:
    ax, ay = a
    bx, by = b
    cx, cy = c
    dx, dy = d
    o1 = orientation(ax, ay, bx, by, cx, cy)
    o2 = orientation(ax, ay, bx, by, dx, dy)
    o3 = orientation(cx, cy, dx, dy, ax, ay)
    o4 = orientation(cx, cy, dx, dy, bx, by)
    if o1 * o2 < 0.0 and o3 * o4 < 0.0:
        return True
    return (
        on_segment(ax, ay, bx, by, cx, cy)
        or on_segment(ax, ay, bx, by, dx, dy)
        or on_segment(cx, cy, dx, dy, ax, ay)
        or on_segment(cx, cy, dx, dy, bx, by)
    )


def point_in_polygon(x: float, y: float, polygon: tuple[tuple[float, float], ...]) -> bool:
    inside = False
    for index, (ax, ay) in enumerate(polygon):
        bx, by = polygon[(index + 1) % len(polygon)]
        if point_segment_distance(x, y, ax, ay, bx, by) <= 1.0e-9:
            return True
        if (ay > y) != (by > y):
            x_cross = (bx - ax) * (y - ay) / (by - ay) + ax
            if x < x_cross:
                inside = not inside
    return inside


def segment_polygon_intersects(
    a: tuple[float, float],
    b: tuple[float, float],
    polygon: tuple[tuple[float, float], ...],
) -> bool:
    if point_in_polygon(a[0], a[1], polygon) or point_in_polygon(b[0], b[1], polygon):
        return True
    return any(
        segments_intersect(a, b, polygon[index], polygon[(index + 1) % len(polygon)])
        for index in range(len(polygon))
    )


def segment_polygon_distance(
    a: tuple[float, float],
    b: tuple[float, float],
    polygon: tuple[tuple[float, float], ...],
) -> float:
    if segment_polygon_intersects(a, b, polygon):
        return 0.0
    distances = [point_segment_distance(px, py, a[0], a[1], b[0], b[1]) for px, py in polygon]
    for index, (ax, ay) in enumerate(polygon):
        bx, by = polygon[(index + 1) % len(polygon)]
        distances.append(point_segment_distance(a[0], a[1], ax, ay, bx, by))
        distances.append(point_segment_distance(b[0], b[1], ax, ay, bx, by))
    return min(distances)


def analyze_route(
    waypoints: list[Waypoint],
    regions: list[AvoidRegion],
    required_clearance: float,
) -> list[SegmentRisk]:
    risks: list[SegmentRisk] = []
    for start, end in zip(waypoints, waypoints[1:]):
        a = (start.x, start.y)
        b = (end.x, end.y)
        length = math.hypot(end.x - start.x, end.y - start.y)
        for region in regions:
            intersects = segment_polygon_intersects(a, b, region.polygon)
            clearance = 0.0 if intersects else segment_polygon_distance(a, b, region.polygon)
            risks.append(
                SegmentRisk(
                    start_id=start.id,
                    end_id=end.id,
                    region=region.name,
                    clearance_m=clearance,
                    required_m=required_clearance,
                    margin_m=clearance - required_clearance,
                    length_m=length,
                    centerline_intersects=intersects,
                )
            )
    risks.sort(key=lambda item: (item.margin_m, item.clearance_m))
    return risks


def parse_xml_summary(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"path": str(path), "exists": False}
    root = ET.parse(path).getroot()
    geoms = [geom for geom in root.iter("geom")]
    collidable = [
        geom for geom in geoms
        if geom.get("name") != "floor"
        and geom.get("contype", "1") != "0"
        and geom.get("conaffinity", "1") != "0"
    ]
    return {
        "path": str(path),
        "exists": True,
        "model": root.get("model", ""),
        "geom_count": len(geoms),
        "collidable_geom_count": len(collidable),
    }


def parse_onnx_summary(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"path": str(path), "exists": False}
    try:
        import onnx  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on local env
        return {"path": str(path), "exists": True, "error": f"onnx import failed: {exc}"}

    model = onnx.load(str(path))
    inputs = [
        {
            "name": item.name,
            "shape": [
                dim.dim_value if dim.dim_value else dim.dim_param
                for dim in item.type.tensor_type.shape.dim
            ],
        }
        for item in model.graph.input
    ]
    outputs = [
        {
            "name": item.name,
            "shape": [
                dim.dim_value if dim.dim_value else dim.dim_param
                for dim in item.type.tensor_type.shape.dim
            ],
        }
        for item in model.graph.output
    ]
    return {
        "path": str(path),
        "exists": True,
        "inputs": inputs,
        "outputs": outputs,
        "metadata_keys": [prop.key for prop in model.metadata_props],
    }


def risk_to_dict(risk: SegmentRisk) -> dict[str, Any]:
    return {
        "start_id": risk.start_id,
        "end_id": risk.end_id,
        "region": risk.region,
        "clearance_m": round(risk.clearance_m, 6),
        "required_m": round(risk.required_m, 6),
        "margin_m": round(risk.margin_m, 6),
        "length_m": round(risk.length_m, 6),
        "centerline_intersects": risk.centerline_intersects,
        "status": risk.status,
    }


def print_report(
    points_path: Path,
    xml_summary: dict[str, Any],
    onnx_summary: dict[str, Any],
    waypoints: list[Waypoint],
    regions: list[AvoidRegion],
    footprint_radius: float,
    avoid_margin: float,
    warn_margin: float,
    risks: list[SegmentRisk],
    top: int,
) -> None:
    required_clearance = footprint_radius + avoid_margin
    violations = [risk for risk in risks if risk.margin_m < 0.0 or risk.centerline_intersects]
    tight = [
        risk for risk in risks
        if risk.margin_m >= 0.0 and risk.margin_m < warn_margin
    ]
    route_len = sum(
        math.hypot(b.x - a.x, b.y - a.y)
        for a, b in zip(waypoints, waypoints[1:])
    )

    print("Route safety check")
    print(f"  points: {points_path}")
    print(f"  waypoints: {len(waypoints)}, regions: {len(regions)}, path_length: {route_len:.3f} m")
    print(
        "  clearance: "
        f"footprint={footprint_radius:.3f} m + avoid_margin={avoid_margin:.3f} m "
        f"=> required={required_clearance:.3f} m"
    )
    if xml_summary.get("exists"):
        print(
            "  xml: "
            f"{xml_summary.get('path')} "
            f"(model={xml_summary.get('model')}, geoms={xml_summary.get('geom_count')}, "
            f"collidable={xml_summary.get('collidable_geom_count')})"
        )
    else:
        print(f"  xml: missing ({xml_summary.get('path')})")
    if onnx_summary.get("exists") and not onnx_summary.get("error"):
        print(f"  onnx: {onnx_summary.get('path')}")
        print(f"    inputs: {onnx_summary.get('inputs')}")
        print(f"    outputs: {onnx_summary.get('outputs')}")
    elif onnx_summary.get("exists"):
        print(f"  onnx: {onnx_summary.get('error')}")
    else:
        print(f"  onnx: missing ({onnx_summary.get('path')})")

    print("")
    if violations:
        print(f"FAIL: {len(violations)} segment-region pairs are inside required clearance.")
    elif tight:
        print(f"WARN: no violations, but {len(tight)} segment-region pairs are tight.")
    else:
        print("PASS: all segment-region pairs satisfy the requested clearance.")

    print("")
    print(f"Closest {min(top, len(risks))} segment-region pairs:")
    print("  status      wp_start->wp_end  region       clear   req     spare")
    for risk in risks[:top]:
        print(
            f"  {risk.status:<10}  "
            f"{risk.start_id:>4}->{risk.end_id:<4}      "
            f"{risk.region:<10}  "
            f"{risk.clearance_m:>6.3f}  "
            f"{risk.required_m:>6.3f}  "
            f"{risk.margin_m:>7.3f}"
        )


def main() -> int:
    args = parse_args()
    points_path = args.points.resolve()
    xml_path = args.xml.resolve()
    onnx_path = args.onnx.resolve()

    payload = load_json(points_path)
    waypoints = load_waypoints(payload)
    regions = load_regions(payload)
    if not regions:
        raise ValueError(f"No avoid regions found in {points_path}")

    footprint_radius = (
        float(args.footprint_radius)
        if args.footprint_radius is not None
        else default_lateral_footprint_radius()
    )
    required_clearance = footprint_radius + float(args.avoid_margin)
    risks = analyze_route(waypoints, regions, required_clearance)
    xml_summary = parse_xml_summary(xml_path)
    onnx_summary = parse_onnx_summary(onnx_path)

    print_report(
        points_path,
        xml_summary,
        onnx_summary,
        waypoints,
        regions,
        footprint_radius,
        float(args.avoid_margin),
        float(args.warn_margin),
        risks,
        max(0, int(args.top)),
    )

    violations = [risk for risk in risks if risk.margin_m < 0.0 or risk.centerline_intersects]
    tight = [
        risk for risk in risks
        if risk.margin_m >= 0.0 and risk.margin_m < float(args.warn_margin)
    ]
    report = {
        "points": str(points_path),
        "waypoint_count": len(waypoints),
        "region_count": len(regions),
        "footprint_radius_m": round(footprint_radius, 6),
        "avoid_margin_m": round(float(args.avoid_margin), 6),
        "required_clearance_m": round(required_clearance, 6),
        "violations": [risk_to_dict(risk) for risk in violations],
        "tight": [risk_to_dict(risk) for risk in tight],
        "closest": [risk_to_dict(risk) for risk in risks[: max(0, int(args.top))]],
        "xml": xml_summary,
        "onnx": onnx_summary,
    }
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    if violations and not args.allow_violations:
        return 2
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
