#!/usr/bin/env python3
"""Pygame tool for marking no-go / avoid regions on a PCD map."""

from __future__ import annotations

import argparse
import json
import math
import os
import pkgutil
import time
from dataclasses import dataclass, field
from pathlib import Path

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

if not hasattr(pkgutil, "ImpImporter"):
    pkgutil.ImpImporter = pkgutil.zipimporter  # type: ignore[attr-defined]

import pygame


TOOL_DIR = Path(__file__).resolve().parent
PCD_DIR = TOOL_DIR / "pcd"
REGION_DIR = TOOL_DIR / "regions"

COLOR_BG = (10, 15, 30)
COLOR_GRID = (22, 29, 48)
COLOR_PANEL = (18, 24, 42)
COLOR_BORDER = (50, 65, 95)
COLOR_TEXT = (248, 250, 252)
COLOR_MUTED = (148, 163, 184)
COLOR_PCD = (92, 160, 255)
COLOR_REGION = (244, 63, 94)
COLOR_CURRENT = (234, 179, 8)


@dataclass
class PointCloud:
    path: Path | None = None
    points: list[tuple[float, float, float]] = field(default_factory=list)
    total_count: int = 0
    sampled_count: int = 0
    error: str = ""


@dataclass
class Region:
    name: str
    points: list[tuple[float, float]]
    kind: str = "avoid"


class Camera:
    def __init__(self, width: int, height: int, panel_width: int = 330) -> None:
        self.width = width
        self.height = height
        self.panel_width = panel_width
        self.zoom = 55.0
        self.pan_x = 0.0
        self.pan_y = 0.0

    @property
    def map_width(self) -> int:
        return max(200, self.width - self.panel_width)

    def world_to_screen(self, x: float, y: float) -> tuple[int, int]:
        return int(self.map_width / 2 + x * self.zoom + self.pan_x), int(self.height / 2 - y * self.zoom + self.pan_y)

    def screen_to_world(self, sx: float, sy: float) -> tuple[float, float]:
        return (sx - self.map_width / 2 - self.pan_x) / self.zoom, -(sy - self.height / 2 - self.pan_y) / self.zoom

    def zoom_at(self, factor: float, pos: tuple[int, int]) -> None:
        before = self.screen_to_world(*pos)
        self.zoom = max(8.0, min(260.0, self.zoom * factor))
        after = self.screen_to_world(*pos)
        self.pan_x += (after[0] - before[0]) * self.zoom
        self.pan_y -= (after[1] - before[1]) * self.zoom


def get_font(size: int, bold: bool = False) -> pygame.font.Font:
    return pygame.font.Font(None, size)


def scan_pcd_files() -> list[Path]:
    PCD_DIR.mkdir(parents=True, exist_ok=True)
    return sorted(PCD_DIR.glob("*.pcd"), key=lambda item: item.name.lower())


def iter_ascii_pcd_points(path: Path):
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        data = False
        for raw in handle:
            line = raw.strip()
            if data and line:
                parts = line.split()
                if len(parts) >= 3:
                    yield float(parts[0]), float(parts[1]), float(parts[2])
            elif line.upper().startswith("DATA"):
                if "ascii" not in line.lower():
                    raise RuntimeError(f"Only ASCII PCD is supported: {path}")
                data = True


def load_pcd(path: Path, max_points: int = 90000, z_min: float = -5.0, z_max: float = 0.5) -> PointCloud:
    cloud = PointCloud(path=path)
    try:
        filtered = []
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


def fit_camera(camera: Camera, cloud: PointCloud, regions: list[Region]) -> None:
    xs = [p[0] for p in cloud.points]
    ys = [p[1] for p in cloud.points]
    for region in regions:
        xs.extend(p[0] for p in region.points)
        ys.extend(p[1] for p in region.points)
    if not xs or not ys:
        return
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    margin = 70
    camera.zoom = max(8.0, min(260.0, min((camera.map_width - margin * 2) / span_x, (camera.height - margin * 2) / span_y)))
    camera.pan_x = -((min_x + max_x) / 2) * camera.zoom
    camera.pan_y = ((min_y + max_y) / 2) * camera.zoom


def draw_grid(surface: pygame.Surface, camera: Camera, font: pygame.font.Font) -> None:
    min_x, min_y = camera.screen_to_world(0, camera.height)
    max_x, max_y = camera.screen_to_world(camera.map_width, 0)
    for gx in range(math.floor(min_x), math.ceil(max_x) + 1):
        sx, _ = camera.world_to_screen(gx, 0)
        color = (38, 50, 78) if gx == 0 else COLOR_GRID
        pygame.draw.line(surface, color, (sx, 0), (sx, camera.height), 2 if gx == 0 else 1)
        if gx % 2 == 0 and 0 < sx < camera.map_width - 30:
            surface.blit(font.render(f"{gx}m", True, COLOR_MUTED), (sx + 4, camera.height - 22))
    for gy in range(math.floor(min_y), math.ceil(max_y) + 1):
        _, sy = camera.world_to_screen(0, gy)
        color = (38, 50, 78) if gy == 0 else COLOR_GRID
        pygame.draw.line(surface, color, (0, sy), (camera.map_width, sy), 2 if gy == 0 else 1)


def draw_pcd(surface: pygame.Surface, camera: Camera, cloud: PointCloud) -> None:
    for x, y, z in cloud.points:
        sx, sy = camera.world_to_screen(x, y)
        if 0 <= sx < camera.map_width and 0 <= sy < camera.height:
            color = (148, 210, 255) if z > 0.3 else COLOR_PCD
            surface.set_at((sx, sy), color)


def draw_regions(surface: pygame.Surface, camera: Camera, regions: list[Region], current: list[tuple[float, float]]) -> None:
    overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    for region in regions:
        points = [camera.world_to_screen(x, y) for x, y in region.points]
        if len(points) >= 3:
            pygame.draw.polygon(overlay, (*COLOR_REGION, 75), points)
            pygame.draw.polygon(surface, COLOR_REGION, points, width=2)
    if current:
        points = [camera.world_to_screen(x, y) for x, y in current]
        for point in points:
            pygame.draw.circle(surface, COLOR_CURRENT, point, 5)
        if len(points) >= 2:
            pygame.draw.lines(surface, COLOR_CURRENT, False, points, width=2)
    surface.blit(overlay, (0, 0))


def save_regions(regions: list[Region], cloud: PointCloud) -> Path:
    REGION_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = REGION_DIR / f"avoid_regions_{stamp}.json"
    payload = {
        "name": f"avoid_regions_{stamp}",
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


def draw_panel(
    surface: pygame.Surface,
    camera: Camera,
    title_font: pygame.font.Font,
    small_font: pygame.font.Font,
    pcd_files: list[Path],
    selected_pcd: int,
    cloud: PointCloud,
    regions: list[Region],
    current: list[tuple[float, float]],
    saved: Path | None,
) -> list[tuple[pygame.Rect, str]]:
    x0 = camera.map_width
    pygame.draw.rect(surface, COLOR_PANEL, (x0, 0, camera.panel_width, camera.height))
    pygame.draw.line(surface, COLOR_BORDER, (x0, 0), (x0, camera.height), 2)
    x = x0 + 18
    y = 16
    buttons: list[tuple[pygame.Rect, str]] = []

    surface.blit(title_font.render("avoid regions", True, COLOR_TEXT), (x, y))
    y += 34
    label = pcd_files[selected_pcd].name if pcd_files and selected_pcd >= 0 else "put .pcd in nav_tools/pcd"
    surface.blit(small_font.render(f"PCD: {label[:32]}", True, COLOR_MUTED), (x, y))
    y += 24
    if cloud.error:
        surface.blit(small_font.render(cloud.error[:36], True, COLOR_REGION), (x, y))
    else:
        surface.blit(small_font.render(f"points: {cloud.sampled_count}/{cloud.total_count}", True, COLOR_MUTED), (x, y))
    y += 34

    for text, action in (("Prev PCD", "prev_pcd"), ("Next PCD", "next_pcd"), ("Fit", "fit"), ("Save JSON", "save")):
        rect = pygame.Rect(x, y, 128, 26)
        pygame.draw.rect(surface, (30, 41, 59), rect, border_radius=5)
        pygame.draw.rect(surface, COLOR_BORDER, rect, width=1, border_radius=5)
        surface.blit(small_font.render(text, True, COLOR_TEXT), (rect.x + 10, rect.y + 6))
        buttons.append((rect, action))
        y += 32

    y += 12
    lines = [
        f"regions: {len(regions)}",
        f"current vertices: {len(current)}",
        "",
        "Left click: add vertex",
        "Enter: close polygon",
        "Backspace: undo vertex",
        "Delete: remove last region",
        "Right/Middle drag: pan",
        "Wheel: zoom",
        "S: save, F: fit",
    ]
    for line in lines:
        color = COLOR_TEXT if line and ":" not in line else COLOR_MUTED
        surface.blit(small_font.render(line, True, color), (x, y))
        y += 20
    if saved:
        y += 8
        surface.blit(small_font.render(f"saved: {saved.name[:28]}", True, (16, 185, 129)), (x, y))
    return buttons


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mark avoid/no-go regions on a PCD map.")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=820)
    parser.add_argument("--pcd", type=Path, help="PCD file name/path. Relative paths are resolved from tools/nav_tools/pcd.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    pcd_files = scan_pcd_files()
    selected_pcd = 0
    if args.pcd:
        if args.pcd.is_absolute():
            raise SystemExit("Use a PCD file name under tools/nav_tools/pcd, not an absolute path.")
        requested = PCD_DIR / args.pcd
        if requested.exists() and requested not in pcd_files:
            pcd_files.append(requested)
        if requested in pcd_files:
            selected_pcd = pcd_files.index(requested)
    cloud = load_pcd(pcd_files[selected_pcd]) if pcd_files else PointCloud()
    regions: list[Region] = []
    current: list[tuple[float, float]] = []
    saved: Path | None = None

    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((args.width, args.height), pygame.RESIZABLE)
    pygame.display.set_caption("nav_tools - avoid region editor")
    title_font = get_font(18, bold=True)
    small_font = get_font(13)
    map_font = get_font(12)
    camera = Camera(args.width, args.height)
    fit_camera(camera, cloud, regions)
    clock = pygame.time.Clock()
    dragging = False
    last_mouse = (0, 0)
    buttons: list[tuple[pygame.Rect, str]] = []
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                camera.width, camera.height = event.w, event.h
                screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    if len(current) >= 3:
                        regions.append(Region(f"avoid_{len(regions) + 1}", current[:]))
                        current.clear()
                elif event.key == pygame.K_BACKSPACE and current:
                    current.pop()
                elif event.key == pygame.K_DELETE and regions:
                    regions.pop()
                elif event.key == pygame.K_f:
                    fit_camera(camera, cloud, regions)
                elif event.key == pygame.K_s:
                    saved = save_regions(regions, cloud)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                clicked = None
                for rect, action in buttons:
                    if rect.collidepoint(event.pos):
                        clicked = action
                        break
                if event.button == 1 and clicked:
                    if clicked == "prev_pcd" and pcd_files:
                        selected_pcd = (selected_pcd - 1) % len(pcd_files)
                        cloud = load_pcd(pcd_files[selected_pcd])
                        fit_camera(camera, cloud, regions)
                    elif clicked == "next_pcd" and pcd_files:
                        selected_pcd = (selected_pcd + 1) % len(pcd_files)
                        cloud = load_pcd(pcd_files[selected_pcd])
                        fit_camera(camera, cloud, regions)
                    elif clicked == "fit":
                        fit_camera(camera, cloud, regions)
                    elif clicked == "save":
                        saved = save_regions(regions, cloud)
                elif event.button == 1 and event.pos[0] < camera.map_width:
                    current.append(camera.screen_to_world(*event.pos))
                elif event.button in (2, 3) or (event.button == 1 and event.pos[0] >= camera.map_width):
                    dragging = True
                    last_mouse = event.pos
                elif event.button == 4 and event.pos[0] < camera.map_width:
                    camera.zoom_at(1.12, event.pos)
                elif event.button == 5 and event.pos[0] < camera.map_width:
                    camera.zoom_at(1 / 1.12, event.pos)
            elif event.type == pygame.MOUSEBUTTONUP:
                dragging = False
            elif event.type == pygame.MOUSEMOTION and dragging:
                dx = event.pos[0] - last_mouse[0]
                dy = event.pos[1] - last_mouse[1]
                camera.pan_x += dx
                camera.pan_y += dy
                last_mouse = event.pos

        screen.fill(COLOR_BG)
        draw_grid(screen, camera, map_font)
        draw_pcd(screen, camera, cloud)
        draw_regions(screen, camera, regions, current)
        buttons = draw_panel(screen, camera, title_font, small_font, pcd_files, selected_pcd, cloud, regions, current, saved)
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
