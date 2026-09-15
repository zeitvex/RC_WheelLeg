#!/usr/bin/env python3
from __future__ import annotations

import argparse
import html
import http.server
import json
import math
import socket
import socketserver
from dataclasses import dataclass
from pathlib import Path


@dataclass
class RoutePoint:
    id: int
    x: float
    y: float
    yaw_deg: float | None
    segment: str


SVG_SEGMENT_COLORS = [
    "#d81b60",
    "#1e88e5",
    "#43a047",
    "#fb8c00",
    "#8e24aa",
    "#00897b",
    "#6d4c41",
    "#546e7a",
    "#e53935",
    "#3949ab",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preview an ASCII PCD map with route points and yaw arrows."
    )
    parser.add_argument(
        "--pcd",
        default="map/map_b.pcd",
        help="Path to the ASCII PCD file. Default: map/map_b.pcd",
    )
    parser.add_argument(
        "--route",
        default="tools/test_route.json",
        help="Path to the route JSON file. Default: tools/test_route.json",
    )
    parser.add_argument(
        "--floor-z-min",
        type=float,
        default=-1.6,
        help="Minimum z value kept from the PCD floor points.",
    )
    parser.add_argument(
        "--floor-z-max",
        type=float,
        default=0.4,
        help="Maximum z value kept from the PCD floor points.",
    )
    parser.add_argument(
        "--sample-step",
        type=int,
        default=25,
        help="Keep one point every N points after filtering.",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=1.0,
        help="Scatter point size for the PCD map.",
    )
    parser.add_argument(
        "--arrow-len",
        type=float,
        default=0.35,
        help="Arrow length used to visualize waypoint yaw.",
    )
    parser.add_argument(
        "--show-id",
        action="store_true",
        help="Draw waypoint id labels.",
    )
    parser.add_argument(
        "--show-yaw-text",
        action="store_true",
        help="Draw yawDeg text next to each waypoint.",
    )
    parser.add_argument(
        "--connect",
        action="store_true",
        help="Connect route points in order.",
    )
    parser.add_argument(
        "--renderer",
        choices=["auto", "mpl", "html"],
        default="html",
        help="Rendering backend. Default: html.",
    )
    parser.add_argument(
        "--output",
        default="tools/pcd_route_preview.html",
        help="Output html file used by the html renderer.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1400,
        help="Canvas width for the html/svg renderer.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=980,
        help="Canvas height for the html/svg renderer.",
    )
    parser.add_argument(
        "--serve",
        action="store_true",
        help="Serve the generated html over HTTP after rendering.",
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host used by the built-in HTTP server. Default: 0.0.0.0",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Port used by the built-in HTTP server. Default: 8000",
    )
    return parser.parse_args()


def load_ascii_pcd_xy(
    path: Path,
    floor_z_min: float,
    floor_z_max: float,
    sample_step: int,
) -> list[tuple[float, float]]:
    if not path.exists():
        raise FileNotFoundError(f"PCD file not found: {path}")

    points: list[tuple[float, float]] = []
    data_started = False
    fields: list[str] = []
    x_index = 0
    y_index = 1
    z_index = 2

    with path.open("r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if not stripped:
                continue

            if not data_started:
                upper = stripped.upper()
                if upper.startswith("FIELDS "):
                    fields = stripped.split()[1:]
                    if {"x", "y", "z"}.issubset(set(fields)):
                        x_index = fields.index("x")
                        y_index = fields.index("y")
                        z_index = fields.index("z")
                elif upper.startswith("DATA"):
                    if "ascii" not in stripped.lower():
                        raise RuntimeError("Only ASCII PCD is supported.")
                    data_started = True
                continue

            parts = stripped.split()
            needed_index = max(x_index, y_index, z_index)
            if len(parts) <= needed_index:
                continue

            try:
                x = float(parts[x_index])
                y = float(parts[y_index])
                z = float(parts[z_index])
            except ValueError:
                continue

            if floor_z_min <= z <= floor_z_max:
                points.append((x, y))

    if not points:
        raise RuntimeError("No usable floor points found after z filtering.")

    return points[:: max(1, sample_step)]


def load_route_points(path: Path) -> list[RoutePoint]:
    if not path.exists():
        raise FileNotFoundError(f"Route file not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    raw_segments = data.get("segments", [])
    if not isinstance(raw_segments, list) or not raw_segments:
        top_level_waypoints = data.get("waypoints", [])
        if isinstance(top_level_waypoints, list) and top_level_waypoints:
            raw_segments = [
                {
                    "name": "segment_1",
                    "waypoints": top_level_waypoints,
                }
            ]
        else:
            raise RuntimeError("Route JSON has no usable segments or waypoints.")

    points: list[RoutePoint] = []
    fallback_id = 1
    for segment_index, segment in enumerate(raw_segments, start=1):
        if not isinstance(segment, dict):
            continue
        segment_name = str(segment.get("name", f"segment_{segment_index}")).strip() or f"segment_{segment_index}"
        raw_waypoints = segment.get("waypoints", [])
        if not isinstance(raw_waypoints, list):
            continue

        for waypoint in raw_waypoints:
            if not isinstance(waypoint, dict):
                continue
            try:
                x = float(waypoint["x"])
                y = float(waypoint["y"])
            except (KeyError, TypeError, ValueError):
                continue

            yaw_deg = waypoint.get("yawDeg", waypoint.get("yaw_deg"))
            if yaw_deg is not None:
                try:
                    yaw_deg = float(yaw_deg)
                except (TypeError, ValueError):
                    yaw_deg = None

            point_id = waypoint.get("id", fallback_id)
            try:
                point_id = int(point_id)
            except (TypeError, ValueError):
                point_id = fallback_id

            points.append(
                RoutePoint(
                    id=point_id,
                    x=x,
                    y=y,
                    yaw_deg=yaw_deg,
                    segment=segment_name,
                )
            )
            fallback_id += 1

    if not points:
        raise RuntimeError("Route JSON contains no valid waypoint coordinates.")

    return points


def plot_preview(
    map_points: list[tuple[float, float]],
    route_points: list[RoutePoint],
    point_size: float,
    arrow_len: float,
    show_id: bool,
    show_yaw_text: bool,
    connect: bool,
    title: str,
) -> None:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 9))

    map_x = [p[0] for p in map_points]
    map_y = [p[1] for p in map_points]
    ax.scatter(map_x, map_y, s=point_size, c="black", alpha=0.35, label="PCD floor")

    segment_names: list[str] = []
    for point in route_points:
        if point.segment not in segment_names:
            segment_names.append(point.segment)

    segment_colors = {
        name: plt.cm.tab10(index % 10) for index, name in enumerate(segment_names)
    }

    for index, point in enumerate(route_points):
        color = segment_colors[point.segment]
        ax.scatter([point.x], [point.y], s=55, c=[color], edgecolors="white", linewidths=0.8)

        if connect and index > 0:
            prev = route_points[index - 1]
            ax.plot([prev.x, point.x], [prev.y, point.y], color=color, linewidth=1.4, alpha=0.9)

        if point.yaw_deg is not None:
            yaw_rad = math.radians(point.yaw_deg)
            dx = arrow_len * math.cos(yaw_rad)
            dy = arrow_len * math.sin(yaw_rad)
            ax.arrow(
                point.x,
                point.y,
                dx,
                dy,
                width=0.018,
                head_width=0.12,
                head_length=0.12,
                length_includes_head=True,
                color=color,
                alpha=0.95,
            )

        label_parts: list[str] = []
        if show_id:
            label_parts.append(str(point.id))
        if show_yaw_text and point.yaw_deg is not None:
            label_parts.append(f"{point.yaw_deg:.1f}deg")
        if label_parts:
            ax.text(
                point.x + 0.05,
                point.y + 0.05,
                " | ".join(label_parts),
                color=color,
                fontsize=9,
                weight="bold",
            )

    ax.set_title(title)
    ax.set_xlabel("map x")
    ax.set_ylabel("map y")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)
    plt.tight_layout()
    plt.show()


def compute_bounds(
    map_points: list[tuple[float, float]],
    route_points: list[RoutePoint],
) -> tuple[float, float, float, float]:
    xs = [p[0] for p in map_points] + [p.x for p in route_points]
    ys = [p[1] for p in map_points] + [p.y for p in route_points]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)

    if math.isclose(min_x, max_x):
        max_x = min_x + 1.0
    if math.isclose(min_y, max_y):
        max_y = min_y + 1.0
    return min_x, max_x, min_y, max_y


def build_segment_color_map(route_points: list[RoutePoint]) -> dict[str, str]:
    segment_names: list[str] = []
    for point in route_points:
        if point.segment not in segment_names:
            segment_names.append(point.segment)
    return {
        name: SVG_SEGMENT_COLORS[index % len(SVG_SEGMENT_COLORS)]
        for index, name in enumerate(segment_names)
    }


def compute_canvas_transform(
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    width: int,
    height: int,
    padding: int,
) -> tuple[float, float, float, float, float]:
    usable_width = max(1.0, float(width - padding * 2))
    usable_height = max(1.0, float(height - padding * 2))
    scale_x = usable_width / max(1e-9, max_x - min_x)
    scale_y = usable_height / max(1e-9, max_y - min_y)
    scale = min(scale_x, scale_y)

    draw_width = (max_x - min_x) * scale
    draw_height = (max_y - min_y) * scale
    offset_x = padding + (usable_width - draw_width) * 0.5
    offset_y = padding + (usable_height - draw_height) * 0.5
    return scale, draw_width, draw_height, offset_x, offset_y


def map_to_canvas(
    x: float,
    y: float,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    width: int,
    height: int,
    padding: int,
) -> tuple[float, float]:
    scale, _draw_width, _draw_height, offset_x, offset_y = compute_canvas_transform(
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        width=width,
        height=height,
        padding=padding,
    )

    canvas_x = offset_x + (x - min_x) * scale
    canvas_y = height - (offset_y + (y - min_y) * scale)
    return canvas_x, canvas_y


def svg_arrow_polygon(
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    color: str,
) -> str:
    dx = end_x - start_x
    dy = end_y - start_y
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return ""

    ux = dx / length
    uy = dy / length
    px = -uy
    py = ux

    head_len = min(16.0, max(8.0, length * 0.35))
    shaft_half = 2.5
    head_half = 7.0

    base_x = end_x - ux * head_len
    base_y = end_y - uy * head_len

    p1 = (start_x + px * shaft_half, start_y + py * shaft_half)
    p2 = (base_x + px * shaft_half, base_y + py * shaft_half)
    p3 = (base_x + px * head_half, base_y + py * head_half)
    p4 = (end_x, end_y)
    p5 = (base_x - px * head_half, base_y - py * head_half)
    p6 = (base_x - px * shaft_half, base_y - py * shaft_half)
    p7 = (start_x - px * shaft_half, start_y - py * shaft_half)

    points_text = " ".join(f"{x:.2f},{y:.2f}" for x, y in [p1, p2, p3, p4, p5, p6, p7])
    return f'<polygon points="{points_text}" fill="{color}" fill-opacity="0.95" />'


def write_html_preview(
    output_path: Path,
    map_points: list[tuple[float, float]],
    route_points: list[RoutePoint],
    point_size: float,
    arrow_len: float,
    show_id: bool,
    show_yaw_text: bool,
    connect: bool,
    title: str,
    width: int,
    height: int,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    min_x, max_x, min_y, max_y = compute_bounds(map_points, route_points)
    padding = 48
    segment_colors = build_segment_color_map(route_points)
    scale, draw_width, draw_height, offset_x, offset_y = compute_canvas_transform(
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        width=width,
        height=height,
        padding=padding,
    )

    svg_parts: list[str] = []
    svg_parts.append(
        f'<svg id="pcd-map-svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}" '
        f'data-min-x="{min_x:.10f}" data-min-y="{min_y:.10f}" data-scale="{scale:.10f}" '
        f'data-offset-x="{offset_x:.10f}" data-offset-y="{offset_y:.10f}" '
        f'data-canvas-height="{float(height):.10f}" data-draw-width="{draw_width:.10f}" '
        f'data-draw-height="{draw_height:.10f}" xmlns="http://www.w3.org/2000/svg">'
    )
    svg_parts.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="#f7f7f5" />')

    for x, y in map_points:
        cx, cy = map_to_canvas(x, y, min_x, max_x, min_y, max_y, width, height, padding)
        radius = max(0.35, point_size * 0.7)
        svg_parts.append(
            f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{radius:.2f}" fill="#1f1f1f" fill-opacity="0.35" />'
        )

    if connect:
        for prev, curr in zip(route_points, route_points[1:]):
            color = segment_colors[curr.segment]
            x1, y1 = map_to_canvas(prev.x, prev.y, min_x, max_x, min_y, max_y, width, height, padding)
            x2, y2 = map_to_canvas(curr.x, curr.y, min_x, max_x, min_y, max_y, width, height, padding)
            svg_parts.append(
                f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                f'stroke="{color}" stroke-width="2" stroke-opacity="0.85" />'
            )

    for point in route_points:
        color = segment_colors[point.segment]
        cx, cy = map_to_canvas(point.x, point.y, min_x, max_x, min_y, max_y, width, height, padding)
        svg_parts.append(
            f'<circle class="route-point" data-id="{point.id}" data-segment="{html.escape(point.segment)}" '
            f'data-map-x="{point.x:.6f}" data-map-y="{point.y:.6f}" '
            + (
                f'data-yaw-deg="{point.yaw_deg:.3f}" '
                if point.yaw_deg is not None else
                ""
            )
            + f'cx="{cx:.2f}" cy="{cy:.2f}" r="6.5" fill="{color}" stroke="#ffffff" stroke-width="1.5">'
            f"<title>ID {point.id} | x={point.x:.3f} y={point.y:.3f}"
            + (f" | yaw={point.yaw_deg:.1f}deg" if point.yaw_deg is not None else "")
            + "</title></circle>"
        )

        if point.yaw_deg is not None:
            yaw_rad = math.radians(point.yaw_deg)
            end_x, end_y = map_to_canvas(
                point.x + arrow_len * math.cos(yaw_rad),
                point.y + arrow_len * math.sin(yaw_rad),
                min_x,
                max_x,
                min_y,
                max_y,
                width,
                height,
                padding,
            )
            svg_parts.append(svg_arrow_polygon(cx, cy, end_x, end_y, color))

        label_parts: list[str] = []
        if show_id:
            label_parts.append(str(point.id))
        if show_yaw_text and point.yaw_deg is not None:
            label_parts.append(f"{point.yaw_deg:.1f}deg")
        if label_parts:
            svg_parts.append(
                f'<text x="{cx + 8:.2f}" y="{cy - 8:.2f}" font-size="12" font-weight="700" '
                f'fill="{color}">{html.escape(" | ".join(label_parts))}</text>'
            )

    svg_parts.append(
        f'<text x="24" y="30" font-size="20" font-weight="700" fill="#222">{html.escape(title)}</text>'
    )
    svg_parts.append(
        f'<text x="24" y="{height - 24}" font-size="13" fill="#444">'
        f'{html.escape(f"route points: {len(route_points)} | pcd samples: {len(map_points)} | scale: {scale:.2f}px/m")}'
        "</text>"
    )
    svg_parts.append(
        '<g id="click-marker" visibility="hidden">'
        '<circle cx="0" cy="0" r="8" fill="none" stroke="#ff1744" stroke-width="2" />'
        '<line x1="-12" y1="0" x2="12" y2="0" stroke="#ff1744" stroke-width="2" />'
        '<line x1="0" y1="-12" x2="0" y2="12" stroke="#ff1744" stroke-width="2" />'
        "</g>"
    )
    svg_parts.append("</svg>")

    html_text = "\n".join(
        [
            "<!DOCTYPE html>",
            '<html lang="zh-CN">',
            "<head>",
            '<meta charset="utf-8" />',
            f"<title>{html.escape(title)}</title>",
            "<style>",
            "body { margin: 0; background: #ece9e1; font-family: 'Segoe UI', sans-serif; color: #222; }",
            ".wrap { padding: 20px; }",
            ".panel { background: #ffffff; border-radius: 12px; box-shadow: 0 10px 30px rgba(0,0,0,0.10); overflow: auto; }",
            ".toolbar { display: flex; flex-wrap: wrap; gap: 12px 18px; align-items: center; padding: 14px 20px 0 20px; font-size: 14px; }",
            ".toolbar strong { color: #111; }",
            ".meta { padding: 10px 20px 6px 20px; color: #444; font-size: 14px; }",
            ".chip { display: inline-flex; align-items: center; gap: 8px; padding: 6px 10px; border-radius: 999px; background: #f3f1eb; }",
            ".swatch { width: 12px; height: 12px; border-radius: 50%; display: inline-block; }",
            "#click-coords { font-family: Consolas, 'Courier New', monospace; }",
            "#point-detail { font-family: Consolas, 'Courier New', monospace; }",
            "svg { display: block; margin: 0 auto; cursor: crosshair; user-select: none; }",
            ".hint { padding: 0 20px 12px 20px; color: #666; font-size: 13px; }",
            "</style>",
            "</head>",
            "<body>",
            '<div class="wrap">',
            '<div class="panel">',
            '<div class="toolbar">',
            '<div class="chip"><strong>点击坐标</strong><span id="click-coords">尚未点击</span></div>',
            '<div class="chip"><strong>点位信息</strong><span id="point-detail">点击路线点可查看 id / yaw</span></div>',
            "</div>",
            '<div class="meta">这是纯浏览器预览页。点击 PCD 或空白位置会显示当前地图坐标，点击路线点还会显示该点的 id、segment 和 yaw。</div>',
            '<div class="hint">浏览器可直接缩放页面查看细节，图上的红色十字为你最近一次点击的位置。</div>',
            "\n".join(svg_parts),
            "<script>",
            "(() => {",
            "  const svg = document.getElementById('pcd-map-svg');",
            "  const marker = document.getElementById('click-marker');",
            "  const clickCoords = document.getElementById('click-coords');",
            "  const pointDetail = document.getElementById('point-detail');",
            "  const minX = parseFloat(svg.dataset.minX);",
            "  const minY = parseFloat(svg.dataset.minY);",
            "  const scale = parseFloat(svg.dataset.scale);",
            "  const offsetX = parseFloat(svg.dataset.offsetX);",
            "  const offsetY = parseFloat(svg.dataset.offsetY);",
            "  const canvasHeight = parseFloat(svg.dataset.canvasHeight);",
            "  function svgPointFromEvent(evt) {",
            "    const pt = svg.createSVGPoint();",
            "    pt.x = evt.clientX;",
            "    pt.y = evt.clientY;",
            "    return pt.matrixTransform(svg.getScreenCTM().inverse());",
            "  }",
            "  function svgToMap(px, py) {",
            "    const mapX = minX + (px - offsetX) / scale;",
            "    const mapY = minY + ((canvasHeight - py) - offsetY) / scale;",
            "    return { x: mapX, y: mapY };",
            "  }",
            "  function formatNum(v) {",
            "    return Number.isFinite(v) ? v.toFixed(3) : 'NaN';",
            "  }",
            "  function updateFromEvent(evt) {",
            "    const svgPoint = svgPointFromEvent(evt);",
            "    const mapPoint = svgToMap(svgPoint.x, svgPoint.y);",
            "    marker.setAttribute('transform', `translate(${svgPoint.x} ${svgPoint.y})`);",
            "    marker.setAttribute('visibility', 'visible');",
            "    clickCoords.textContent = `x=${formatNum(mapPoint.x)}, y=${formatNum(mapPoint.y)}`;",
            "    const routePoint = evt.target.closest('.route-point');",
            "    if (routePoint) {",
            "      const yaw = routePoint.dataset.yawDeg;",
            "      pointDetail.textContent = `id=${routePoint.dataset.id}, segment=${routePoint.dataset.segment}, x=${routePoint.dataset.mapX}, y=${routePoint.dataset.mapY}` + (yaw ? `, yaw=${Number(yaw).toFixed(1)}deg` : '');",
            "    } else {",
            "      pointDetail.textContent = '未点击路线点';",
            "    }",
            "  }",
            "  svg.addEventListener('click', updateFromEvent);",
            "})();",
            "</script>",
            "</div>",
            "</div>",
            "</body>",
            "</html>",
        ]
    )

    output_path.write_text(html_text, encoding="utf-8")


def guess_local_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return str(sock.getsockname()[0])
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def serve_output_file(output_path: Path, host: str, port: int) -> None:
    output_path = output_path.resolve()
    web_root = output_path.parent.resolve()
    relative_url = output_path.relative_to(web_root).as_posix()

    class PreviewHttpHandler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(web_root), **kwargs)

    server_address = (host, int(port))
    with socketserver.TCPServer(server_address, PreviewHttpHandler) as httpd:
        local_ip = guess_local_ip()
        bound_host = host if host not in {"0.0.0.0", "::"} else local_ip
        print("Preview server is running.")
        print(f"Local URL:   http://127.0.0.1:{port}/{relative_url}")
        print(f"LAN URL:     http://{bound_host}:{port}/{relative_url}")
        print("Open the LAN URL from your Windows browser.")
        print("Press Ctrl+C to stop the server.")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped.")


def render_preview(
    renderer: str,
    output_path: Path,
    map_points: list[tuple[float, float]],
    route_points: list[RoutePoint],
    point_size: float,
    arrow_len: float,
    show_id: bool,
    show_yaw_text: bool,
    connect: bool,
    title: str,
    width: int,
    height: int,
) -> str:
    if renderer in {"auto", "mpl"}:
        try:
            plot_preview(
                map_points=map_points,
                route_points=route_points,
                point_size=point_size,
                arrow_len=arrow_len,
                show_id=show_id,
                show_yaw_text=show_yaw_text,
                connect=connect,
                title=title,
            )
            return "mpl"
        except Exception as exc:
            if renderer == "mpl":
                raise RuntimeError(
                    "matplotlib 渲染失败。当前环境很可能存在 numpy / matplotlib 二进制不兼容问题。"
                ) from exc
            print(f"[info] matplotlib 不可用，自动切换到 html 渲染: {exc}")

    write_html_preview(
        output_path=output_path,
        map_points=map_points,
        route_points=route_points,
        point_size=point_size,
        arrow_len=arrow_len,
        show_id=show_id,
        show_yaw_text=show_yaw_text,
        connect=connect,
        title=title,
        width=width,
        height=height,
    )
    return "html"


def main() -> None:
    args = parse_args()
    pcd_path = Path(args.pcd)
    route_path = Path(args.route)
    output_path = Path(args.output)

    map_points = load_ascii_pcd_xy(
        path=pcd_path,
        floor_z_min=float(args.floor_z_min),
        floor_z_max=float(args.floor_z_max),
        sample_step=max(1, int(args.sample_step)),
    )
    route_points = load_route_points(route_path)

    title = f"PCD Route Preview: {pcd_path.name} + {route_path.name}"
    used_renderer = render_preview(
        renderer=str(args.renderer),
        output_path=output_path,
        map_points=map_points,
        route_points=route_points,
        point_size=float(args.point_size),
        arrow_len=float(args.arrow_len),
        show_id=bool(args.show_id),
        show_yaw_text=bool(args.show_yaw_text),
        connect=bool(args.connect),
        title=title,
        width=max(600, int(args.width)),
        height=max(400, int(args.height)),
    )
    if used_renderer == "html":
        print(f"HTML preview written to: {output_path.resolve()}")
    if bool(args.serve):
        if used_renderer != "html":
            raise RuntimeError("Built-in server currently supports html output only.")
        serve_output_file(
            output_path=output_path,
            host=str(args.host),
            port=int(args.port),
        )


if __name__ == "__main__":
    main()
