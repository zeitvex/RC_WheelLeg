#!/usr/bin/env python3
"""Generate safer waypoint candidates for nav_tools route JSON files.

The optimizer keeps the original JSON intact, adjusts copied waypoints away
from avoid polygons, recomputes yaw, and writes candidate JSON files for
offline/sim2sim validation.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from route_safety_check import (
    AvoidRegion,
    Waypoint,
    analyze_route,
    default_lateral_footprint_radius,
    load_regions,
    load_waypoints,
    point_in_polygon,
    point_segment_distance,
)


@dataclass
class MutablePoint:
    id: str
    x: float
    y: float
    yaw_deg: float
    speed: float | None
    policy: str
    tolerance: float | None
    row_refs: list[dict[str, Any]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate safer route candidates.")
    parser.add_argument("--points", type=Path, default=Path("tools/nav_tools/points/points_20260715_120154.json"))
    parser.add_argument("--out-dir", type=Path, default=Path("tools/nav_tools/points/auto_candidates"))
    parser.add_argument("--avoid-margin", type=float, default=0.05)
    parser.add_argument("--extra-margin", type=float, default=0.08)
    parser.add_argument("--footprint-radius", type=float, default=None)
    parser.add_argument("--iterations", type=int, default=240)
    parser.add_argument("--step-scale", type=float, default=0.45)
    parser.add_argument("--max-move-per-iter", type=float, default=0.08)
    parser.add_argument("--max-total-move", type=float, default=0.55)
    parser.add_argument("--smooth-weight", type=float, default=0.12)
    parser.add_argument("--lock-ends", action="store_true", default=True)
    parser.add_argument("--no-lock-ends", dest="lock_ends", action="store_false")
    parser.add_argument("--slow-near-risk", action="store_true", default=True)
    parser.add_argument("--no-slow-near-risk", dest="slow_near_risk", action="store_false")
    parser.add_argument("--min-speed", type=float, default=0.22)
    parser.add_argument("--max-segment-length", type=float, default=0.65)
    parser.add_argument(
        "--focus-start-index",
        type=int,
        default=None,
        help="1-based first waypoint index allowed to move; points outside the focus range are locked.",
    )
    parser.add_argument(
        "--focus-end-index",
        type=int,
        default=None,
        help="1-based last waypoint index allowed to move; points outside the focus range are locked.",
    )
    parser.add_argument(
        "--policy-range",
        action="append",
        default=[],
        metavar="START:END:POLICY",
        help="Override waypoint policy by 1-based inclusive index range, e.g. 21:26:crawl. Can be repeated.",
    )
    parser.add_argument(
        "--speed-range",
        action="append",
        default=[],
        metavar="START:END:SPEED",
        help="Override waypoint speed by 1-based inclusive index range, e.g. 21:26:0.22. Can be repeated.",
    )
    parser.add_argument(
        "--shift-range",
        action="append",
        default=[],
        metavar="START:END:DX:DY",
        help="Shift waypoint positions by 1-based inclusive index range. Applied after optimization.",
    )
    parser.add_argument("--top", type=int, default=12)
    return parser.parse_args()


def load_payload(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be object: {path}")
    return payload


def iter_waypoint_rows(payload: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if isinstance(payload.get("waypoints"), list):
        rows.extend(row for row in payload["waypoints"] if isinstance(row, dict))
    if isinstance(payload.get("segments"), list):
        for segment in payload["segments"]:
            if isinstance(segment, dict) and isinstance(segment.get("waypoints"), list):
                rows.extend(row for row in segment["waypoints"] if isinstance(row, dict))
    return rows


def load_mutable_points(payload: dict[str, Any]) -> list[MutablePoint]:
    unique: list[MutablePoint] = []
    refs_by_key: dict[tuple[str, float, float], MutablePoint] = {}
    for index, row in enumerate(iter_waypoint_rows(payload), start=1):
        x = float(row.get("world_x", row.get("x", 0.0)))
        y = float(row.get("world_y", row.get("y", 0.0)))
        key = (str(row.get("id", index)), round(x, 9), round(y, 9))
        point = refs_by_key.get(key)
        if point is None:
            point = MutablePoint(
                id=str(row.get("id", index)),
                x=x,
                y=y,
                yaw_deg=float(row.get("yawDeg", row.get("yaw_deg", row.get("yaw", 0.0)))),
                speed=float(row["speed"]) if row.get("speed") is not None else None,
                policy=str(row.get("policy", "rough")),
                tolerance=float(row["tolerance"]) if row.get("tolerance") is not None else None,
                row_refs=[],
            )
            refs_by_key[key] = point
            unique.append(point)
        point.row_refs.append(row)
    if len(unique) < 2:
        raise ValueError("Route must contain at least two unique waypoints")
    return unique


def mutable_to_waypoints(points: list[MutablePoint]) -> list[Waypoint]:
    return [
        Waypoint(
            index=i,
            id=p.id,
            x=p.x,
            y=p.y,
            yaw_deg=p.yaw_deg,
            speed=p.speed,
            policy=p.policy,
            tolerance=p.tolerance,
        )
        for i, p in enumerate(points, start=1)
    ]


def closest_point_on_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> tuple[float, float]:
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-12:
        return ax, ay
    t = ((px - ax) * dx + (py - ay) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    return ax + t * dx, ay + t * dy


def closest_segment_polygon_pair(
    ax: float,
    ay: float,
    bx: float,
    by: float,
    polygon: tuple[tuple[float, float], ...],
) -> tuple[float, float, float, float, float]:
    best = (float("inf"), ax, ay, polygon[0][0], polygon[0][1])
    samples = 17
    for i in range(samples):
        t = i / (samples - 1)
        sx = ax + (bx - ax) * t
        sy = ay + (by - ay) * t
        for px, py in polygon:
            dist = math.hypot(sx - px, sy - py)
            if dist < best[0]:
                best = (dist, sx, sy, px, py)
        for (ex0, ey0), (ex1, ey1) in zip(polygon, polygon[1:] + polygon[:1]):
            qx, qy = closest_point_on_segment(sx, sy, ex0, ey0, ex1, ey1)
            dist = math.hypot(sx - qx, sy - qy)
            if dist < best[0]:
                best = (dist, sx, sy, qx, qy)
    for sx, sy in ((ax, ay), (bx, by)):
        for (ex0, ey0), (ex1, ey1) in zip(polygon, polygon[1:] + polygon[:1]):
            qx, qy = closest_point_on_segment(sx, sy, ex0, ey0, ex1, ey1)
            dist = math.hypot(sx - qx, sy - qy)
            if dist < best[0]:
                best = (dist, sx, sy, qx, qy)
    return best


def polygon_centroid(polygon: tuple[tuple[float, float], ...]) -> tuple[float, float]:
    return (
        sum(x for x, _ in polygon) / len(polygon),
        sum(y for _, y in polygon) / len(polygon),
    )


def point_inside_any_region(x: float, y: float, regions: list[AvoidRegion]) -> bool:
    return any(point_in_polygon(x, y, region.polygon) for region in regions)


def nearest_region_clearance(x: float, y: float, regions: list[AvoidRegion]) -> float:
    best = float("inf")
    for region in regions:
        if point_in_polygon(x, y, region.polygon):
            return 0.0
        polygon = region.polygon
        for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1]):
            best = min(best, point_segment_distance(x, y, ax, ay, bx, by))
    return best


def optimize_points(
    points: list[MutablePoint],
    regions: list[AvoidRegion],
    required_clearance: float,
    args: argparse.Namespace,
) -> tuple[list[MutablePoint], dict[str, Any]]:
    original = [(p.x, p.y) for p in points]
    target_clearance = required_clearance + float(args.extra_margin)
    locked = {0, len(points) - 1} if args.lock_ends else set()
    if args.focus_start_index is not None or args.focus_end_index is not None:
        focus_start = max(1, int(args.focus_start_index or 1)) - 1
        focus_end = min(len(points), int(args.focus_end_index or len(points))) - 1
        if focus_start > focus_end:
            raise ValueError(
                f"Invalid focus range: start={args.focus_start_index} end={args.focus_end_index}"
            )
        locked.update(i for i in range(len(points)) if i < focus_start or i > focus_end)
    risk_hit_counts = [0 for _ in points]

    for _ in range(int(args.iterations)):
        deltas = [[0.0, 0.0] for _ in points]
        weights = [0.0 for _ in points]
        waypoints = mutable_to_waypoints(points)
        risks = analyze_route(waypoints, regions, target_clearance)
        active_risks = [risk for risk in risks if risk.margin_m < 0.0]
        if not active_risks:
            break

        id_to_index = {p.id: i for i, p in enumerate(points)}
        for risk in active_risks:
            ia = id_to_index.get(risk.start_id)
            ib = id_to_index.get(risk.end_id)
            region = next((item for item in regions if item.name == risk.region), None)
            if ia is None or ib is None or region is None:
                continue
            ax, ay = points[ia].x, points[ia].y
            bx, by = points[ib].x, points[ib].y
            dist, sx, sy, qx, qy = closest_segment_polygon_pair(ax, ay, bx, by, region.polygon)
            vx = sx - qx
            vy = sy - qy
            norm = math.hypot(vx, vy)
            if norm < 1.0e-6 or risk.centerline_intersects:
                cx, cy = polygon_centroid(region.polygon)
                mx = 0.5 * (ax + bx)
                my = 0.5 * (ay + by)
                vx = mx - cx
                vy = my - cy
                norm = math.hypot(vx, vy)
            if norm < 1.0e-6:
                vx, vy, norm = 1.0, 0.0, 1.0
            ux = vx / norm
            uy = vy / norm
            push = min(float(args.max_move_per_iter), max(0.0, target_clearance - dist) * float(args.step_scale))
            for idx in (ia, ib):
                if idx in locked:
                    continue
                deltas[idx][0] += ux * push
                deltas[idx][1] += uy * push
                weights[idx] += 1.0
                risk_hit_counts[idx] += 1

        if args.smooth_weight > 0.0 and len(points) > 2:
            for i in range(1, len(points) - 1):
                if i in locked:
                    continue
                avg_x = 0.5 * (points[i - 1].x + points[i + 1].x)
                avg_y = 0.5 * (points[i - 1].y + points[i + 1].y)
                deltas[i][0] += (avg_x - points[i].x) * float(args.smooth_weight)
                deltas[i][1] += (avg_y - points[i].y) * float(args.smooth_weight)
                weights[i] += 1.0

        for i, point in enumerate(points):
            if i in locked or weights[i] <= 0.0:
                continue
            dx = deltas[i][0] / weights[i]
            dy = deltas[i][1] / weights[i]
            ox, oy = original[i]
            next_x = point.x + dx
            next_y = point.y + dy
            total_dx = next_x - ox
            total_dy = next_y - oy
            total = math.hypot(total_dx, total_dy)
            if total > float(args.max_total_move):
                scale = float(args.max_total_move) / total
                next_x = ox + total_dx * scale
                next_y = oy + total_dy * scale
            point.x = next_x
            point.y = next_y

    for i, point in enumerate(points[:-1]):
        nxt = points[i + 1]
        point.yaw_deg = math.degrees(math.atan2(nxt.y - point.y, nxt.x - point.x))
    points[-1].yaw_deg = points[-2].yaw_deg

    if args.slow_near_risk:
        for i, point in enumerate(points):
            clearance = nearest_region_clearance(point.x, point.y, regions)
            if clearance < target_clearance + 0.12:
                current = point.speed if point.speed is not None else 0.35
                point.speed = max(float(args.min_speed), min(current, 0.28))
            if risk_hit_counts[i] > 0:
                current = point.speed if point.speed is not None else 0.35
                point.speed = max(float(args.min_speed), min(current, 0.25))

    final_risks = analyze_route(mutable_to_waypoints(points), regions, required_clearance)
    violations = [risk for risk in final_risks if risk.margin_m < 0.0 or risk.centerline_intersects]
    moves = [
        {
            "id": point.id,
            "dx": round(point.x - ox, 4),
            "dy": round(point.y - oy, 4),
            "dist": round(math.hypot(point.x - ox, point.y - oy), 4),
            "risk_hits": risk_hit_counts[i],
        }
        for i, (point, (ox, oy)) in enumerate(zip(points, original))
        if math.hypot(point.x - ox, point.y - oy) > 1.0e-4 or risk_hit_counts[i] > 0
    ]
    return points, {
        "violations": len(violations),
        "min_margin": round(final_risks[0].margin_m, 6) if final_risks else None,
        "focus_range": [
            args.focus_start_index,
            args.focus_end_index,
        ],
        "moves": moves,
        "top_risks": [
            {
                "start_id": risk.start_id,
                "end_id": risk.end_id,
                "region": risk.region,
                "margin_m": round(risk.margin_m, 6),
                "clearance_m": round(risk.clearance_m, 6),
                "centerline_intersects": risk.centerline_intersects,
            }
            for risk in final_risks[: int(args.top)]
        ],
    }


def apply_policy_ranges(points: list[MutablePoint], ranges: list[str]) -> list[dict[str, Any]]:
    applied: list[dict[str, Any]] = []
    for item in ranges:
        parts = [part.strip() for part in str(item).split(":")]
        if len(parts) != 3 or not parts[2]:
            raise ValueError(f"Invalid --policy-range {item!r}; expected START:END:POLICY")
        start = max(1, int(parts[0]))
        end = min(len(points), int(parts[1]))
        if start > end:
            raise ValueError(f"Invalid --policy-range {item!r}; start is after end")
        policy = parts[2]
        for index in range(start - 1, end):
            points[index].policy = policy
        applied.append({"start": start, "end": end, "policy": policy})
    return applied


def apply_speed_ranges(points: list[MutablePoint], ranges: list[str]) -> list[dict[str, Any]]:
    applied: list[dict[str, Any]] = []
    for item in ranges:
        parts = [part.strip() for part in str(item).split(":")]
        if len(parts) != 3:
            raise ValueError(f"Invalid --speed-range {item!r}; expected START:END:SPEED")
        start = max(1, int(parts[0]))
        end = min(len(points), int(parts[1]))
        if start > end:
            raise ValueError(f"Invalid --speed-range {item!r}; start is after end")
        speed = float(parts[2])
        for index in range(start - 1, end):
            points[index].speed = speed
        applied.append({"start": start, "end": end, "speed": speed})
    return applied


def apply_shift_ranges(points: list[MutablePoint], ranges: list[str]) -> list[dict[str, Any]]:
    applied: list[dict[str, Any]] = []
    for item in ranges:
        parts = [part.strip() for part in str(item).split(":")]
        if len(parts) != 4:
            raise ValueError(f"Invalid --shift-range {item!r}; expected START:END:DX:DY")
        start = max(1, int(parts[0]))
        end = min(len(points), int(parts[1]))
        if start > end:
            raise ValueError(f"Invalid --shift-range {item!r}; start is after end")
        dx = float(parts[2])
        dy = float(parts[3])
        for index in range(start - 1, end):
            points[index].x += dx
            points[index].y += dy
        applied.append({"start": start, "end": end, "dx": dx, "dy": dy})
    if applied:
        for i, point in enumerate(points[:-1]):
            nxt = points[i + 1]
            point.yaw_deg = math.degrees(math.atan2(nxt.y - point.y, nxt.x - point.x))
        points[-1].yaw_deg = points[-2].yaw_deg
    return applied


def refresh_safety_summary(
    summary: dict[str, Any],
    points: list[MutablePoint],
    regions: list[AvoidRegion],
    required_clearance: float,
    args: argparse.Namespace,
) -> None:
    final_risks = analyze_route(mutable_to_waypoints(points), regions, required_clearance)
    violations = [risk for risk in final_risks if risk.margin_m < 0.0 or risk.centerline_intersects]
    summary["violations"] = len(violations)
    summary["min_margin"] = round(final_risks[0].margin_m, 6) if final_risks else None
    summary["top_risks"] = [
        {
            "start_id": risk.start_id,
            "end_id": risk.end_id,
            "region": risk.region,
            "margin_m": round(risk.margin_m, 6),
            "clearance_m": round(risk.clearance_m, 6),
            "centerline_intersects": risk.centerline_intersects,
        }
        for risk in final_risks[: int(args.top)]
    ]


def apply_points_to_payload(payload: dict[str, Any], points: list[MutablePoint]) -> None:
    for point in points:
        for row in point.row_refs:
            if "world_x" in row:
                row["world_x"] = point.x
            row["x"] = point.x
            if "world_y" in row:
                row["world_y"] = point.y
            row["y"] = point.y
            row["yawDeg"] = point.yaw_deg
            if point.speed is not None:
                row["speed"] = point.speed
            if point.tolerance is not None:
                row["tolerance"] = point.tolerance
            if point.policy:
                row["policy"] = point.policy


def densify_segments(payload: dict[str, Any], max_len: float) -> None:
    if max_len <= 0.0:
        return
    if not isinstance(payload.get("segments"), list):
        return
    for segment in payload["segments"]:
        rows = segment.get("waypoints") if isinstance(segment, dict) else None
        if not isinstance(rows, list) or len(rows) < 2:
            continue
        new_rows: list[dict[str, Any]] = []
        for a, b in zip(rows, rows[1:]):
            new_rows.append(a)
            ax, ay = float(a.get("x", 0.0)), float(a.get("y", 0.0))
            bx, by = float(b.get("x", 0.0)), float(b.get("y", 0.0))
            dist = math.hypot(bx - ax, by - ay)
            inserts = max(0, int(math.ceil(dist / max_len)) - 1)
            for j in range(inserts):
                t = (j + 1) / (inserts + 1)
                row = copy.deepcopy(a)
                row["id"] = f"{a.get('id')}_{j + 1}"
                row["x"] = ax + (bx - ax) * t
                row["y"] = ay + (by - ay) * t
                row["yawDeg"] = math.degrees(math.atan2(by - ay, bx - ax))
                if a.get("speed") is not None and b.get("speed") is not None:
                    row["speed"] = min(float(a["speed"]), float(b["speed"]))
                new_rows.append(row)
        new_rows.append(rows[-1])
        segment["waypoints"] = new_rows
    if isinstance(payload.get("waypoints"), list):
        flat: list[dict[str, Any]] = []
        for segment in payload["segments"]:
            rows = segment.get("waypoints") if isinstance(segment, dict) else None
            if isinstance(rows, list):
                flat.extend(copy.deepcopy(row) for row in rows if isinstance(row, dict))
        if flat:
            payload["waypoints"] = flat


def main() -> int:
    args = parse_args()
    payload = load_payload(args.points)
    regions = load_regions(payload)
    points = load_mutable_points(payload)
    footprint = float(args.footprint_radius) if args.footprint_radius is not None else default_lateral_footprint_radius()
    required = footprint + float(args.avoid_margin)

    candidate_payload = copy.deepcopy(payload)
    candidate_points = load_mutable_points(candidate_payload)
    optimized, summary = optimize_points(candidate_points, regions, required, args)
    shift_overrides = apply_shift_ranges(optimized, list(args.shift_range))
    policy_overrides = apply_policy_ranges(optimized, list(args.policy_range))
    speed_overrides = apply_speed_ranges(optimized, list(args.speed_range))
    if shift_overrides:
        summary["shift_overrides"] = shift_overrides
    if policy_overrides:
        summary["policy_overrides"] = policy_overrides
    if speed_overrides:
        summary["speed_overrides"] = speed_overrides
    if shift_overrides or policy_overrides or speed_overrides:
        refresh_safety_summary(summary, optimized, regions, required, args)
    apply_points_to_payload(candidate_payload, optimized)
    densify_segments(candidate_payload, float(args.max_segment_length))

    stamp = time.strftime("%Y%m%d_%H%M%S")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_path = args.out_dir / f"{args.points.stem}_auto_{stamp}.json"
    candidate_payload["name"] = f"{payload.get('name', args.points.stem)}_auto_{stamp}"
    candidate_payload["autoOptimize"] = {
        "source": str(args.points),
        "createdAt": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "footprintRadius": footprint,
        "avoidMargin": args.avoid_margin,
        "extraMargin": args.extra_margin,
        "summary": summary,
    }
    out_path.write_text(json.dumps(candidate_payload, ensure_ascii=False, indent=2), encoding="utf-8")

    print("Route candidate optimizer")
    print(f"  source: {args.points}")
    print(f"  output: {out_path}")
    print(f"  required_clearance: {required:.3f} m, target: {required + args.extra_margin:.3f} m")
    print(f"  final_violations: {summary['violations']}")
    print(f"  min_margin: {summary['min_margin']} m")
    print(f"  moved_points: {len(summary['moves'])}")
    for item in summary["top_risks"][: int(args.top)]:
        print(
            "  risk "
            f"{item['start_id']}->{item['end_id']} {item['region']} "
            f"margin={item['margin_m']:.3f} clearance={item['clearance_m']:.3f}"
        )
    return 0 if summary["violations"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
