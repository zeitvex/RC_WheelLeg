#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ast
import json
import math
import re
import struct
import time
import xml.etree.ElementTree as ET
import zlib
from collections import deque
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from posixpath import normpath
from typing import Any, Optional
from urllib.parse import parse_qs, unquote, urlparse


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAP_ROOT = ROOT / "map" / "processed"
RAW_MAP_ROOT = ROOT / "map"
DEFAULT_MJCF = ROOT.parent / "mjcf" / "sim2sim_temp.xml"
STATIC_DIR = Path(__file__).resolve().parent / "static"
REFERENCE_VENDOR_DIR = ROOT.parent / "00_ reference" / "jie_3d_nav" / "jie_octomap" / "web" / "vendor"
PGM_CACHE: dict[Path, tuple[float, int, int, list[int]]] = {}
PNG_CACHE: dict[Path, tuple[float, bytes]] = {}
POINT_CLOUD_CACHE: dict[tuple[Path, int, float, float], tuple[float, dict[str, Any]]] = {}
VOXEL_CACHE: dict[tuple[Path, float, float, float, int, int, int], tuple[float, dict[str, Any]]] = {}
SEMANTIC_CACHE: dict[Path, tuple[float, dict[str, Any]]] = {}
TOPDOWN_CACHE: dict[tuple[Path, float, float, float, float, int, int], tuple[float, bytes]] = {}
RAW_META_CACHE: dict[Path, tuple[float, dict[str, Any]]] = {}
TOPDOWN_LAYER = "pcd_z1_topdown.png"


def safe_name(value: str) -> str:
    value = unquote(value or "").strip()
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", value):
        raise ValueError(f"unsafe name: {value!r}")
    return value


def safe_file_stem(value: str, fallback: str = "route") -> str:
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip()).strip("._-")
    return stem or fallback


def parse_meta(path: Path) -> dict[str, Any]:
    meta: dict[str, Any] = {}
    if not path.exists():
        return meta
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        value = value.strip()
        if value.startswith("[") and value.endswith("]"):
            try:
                meta[key.strip()] = ast.literal_eval(value)
            except Exception:
                meta[key.strip()] = value
        else:
            try:
                if "." in value:
                    meta[key.strip()] = float(value)
                else:
                    meta[key.strip()] = int(value)
            except ValueError:
                meta[key.strip()] = value
    return meta


def read_token(data: bytes, pos: int) -> tuple[str, int]:
    n = len(data)
    while pos < n:
        b = data[pos]
        if b == 35:
            while pos < n and data[pos] not in (10, 13):
                pos += 1
        elif chr(b).isspace():
            pos += 1
        else:
            break
    start = pos
    while pos < n and not chr(data[pos]).isspace():
        pos += 1
    return data[start:pos].decode("ascii"), pos


def load_pgm(path: Path) -> tuple[int, int, list[int]]:
    mtime = path.stat().st_mtime
    cached = PGM_CACHE.get(path)
    if cached and cached[0] == mtime:
        return cached[1], cached[2], cached[3]

    data = path.read_bytes()
    magic, pos = read_token(data, 0)
    if magic not in ("P5", "P2"):
        raise RuntimeError(f"unsupported PGM magic {magic}: {path}")
    width_s, pos = read_token(data, pos)
    height_s, pos = read_token(data, pos)
    max_s, pos = read_token(data, pos)
    width = int(width_s)
    height = int(height_s)
    max_value = int(max_s)
    if max_value <= 0 or max_value > 255:
        raise RuntimeError(f"unsupported PGM max value {max_value}: {path}")

    if magic == "P5":
        while pos < len(data) and chr(data[pos]).isspace():
            pos += 1
        values = list(data[pos:pos + width * height])
    else:
        values = []
        for _ in range(width * height):
            token, pos = read_token(data, pos)
            values.append(int(token))
    if len(values) != width * height:
        raise RuntimeError(f"PGM size mismatch: {path}")
    PGM_CACHE[path] = (mtime, width, height, values)
    return width, height, values


def png_chunk(chunk_type: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + chunk_type
        + payload
        + struct.pack(">I", zlib.crc32(chunk_type + payload) & 0xFFFFFFFF)
    )


def gray_pgm_to_png(path: Path) -> bytes:
    mtime = path.stat().st_mtime
    cached = PNG_CACHE.get(path)
    if cached and cached[0] == mtime:
        return cached[1]
    width, height, values = load_pgm(path)
    rows = bytearray()
    for row in range(height):
        rows.append(0)
        start = row * width
        rows.extend(values[start:start + width])
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    out = b"\x89PNG\r\n\x1a\n"
    out += png_chunk(b"IHDR", ihdr)
    out += png_chunk(b"IDAT", zlib.compress(bytes(rows), level=6))
    out += png_chunk(b"IEND", b"")
    PNG_CACHE[path] = (mtime, out)
    return out


def map_dir(map_root: Path, name: str) -> Path:
    target = (map_root / safe_name(name)).resolve()
    root = map_root.resolve()
    if root != target and root not in target.parents:
        raise ValueError("map path escapes root")
    if not target.is_dir():
        raise FileNotFoundError(target)
    return target


def raw_pcd_path(name: str) -> Path:
    target = (RAW_MAP_ROOT / f"{safe_name(name)}.pcd").resolve()
    root = RAW_MAP_ROOT.resolve()
    if root != target and root not in target.parents:
        raise ValueError("raw map path escapes root")
    if not target.is_file():
        raise FileNotFoundError(target)
    return target


def scan_raw_pcd_meta(path: Path, resolution: float = 0.05, padding: float = 0.25) -> dict[str, Any]:
    path = path.resolve()
    mtime = path.stat().st_mtime
    cached = RAW_META_CACHE.get(path)
    if cached and cached[0] == mtime:
        return cached[1]

    count = 0
    min_x = min_y = min_z = math.inf
    max_x = max_y = max_z = -math.inf
    for x, y, z in iter_ascii_pcd_points(path):
        count += 1
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        min_z = min(min_z, z)
        max_x = max(max_x, x)
        max_y = max(max_y, y)
        max_z = max(max_z, z)
    if count <= 0:
        raise RuntimeError(f"No valid xyz points found in {path}")

    origin_x = math.floor((min_x - padding) / resolution) * resolution
    origin_y = math.floor((min_y - padding) / resolution) * resolution
    top_x = math.ceil((max_x + padding) / resolution) * resolution
    top_y = math.ceil((max_y + padding) / resolution) * resolution
    meta = {
        "source_pcd": str(path),
        "point_count": count,
        "bounds_min": [min_x, min_y, min_z],
        "bounds_max": [max_x, max_y, max_z],
        "resolution": resolution,
        "origin": [origin_x, origin_y, 0.0],
        "width": int(math.ceil((top_x - origin_x) / resolution)),
        "height": int(math.ceil((top_y - origin_y) / resolution)),
        "raw_pcd_map": True,
    }
    RAW_META_CACHE[path] = (mtime, meta)
    return meta


def map_context(map_root: Path, name: str) -> dict[str, Any]:
    try:
        directory = map_dir(map_root, name)
        meta = parse_meta(directory / "meta.yaml")
        pcd_name = Path(str(meta.get("source_pcd", name))).stem
        return {
            "name": name,
            "directory": directory,
            "meta": meta,
            "routes_dir": RAW_MAP_ROOT / "routes" / pcd_name,
            "raw": False,
        }
    except FileNotFoundError:
        pcd = raw_pcd_path(name)
        return {
            "name": name,
            "directory": None,
            "meta": scan_raw_pcd_meta(pcd),
            "routes_dir": RAW_MAP_ROOT / "routes" / pcd.stem,
            "raw": True,
        }


def list_maps(map_root: Path) -> list[dict[str, Any]]:
    result = []
    seen_sources: set[str] = set()
    if not map_root.exists():
        pass
    else:
        for item in sorted(map_root.iterdir()):
            if not item.is_dir():
                continue
            meta = parse_meta(item / "meta.yaml")
            layers = sorted(
                p.name for p in item.iterdir()
                if p.suffix.lower() in (".pgm", ".png")
            )
            if meta.get("source_pcd"):
                layers.insert(0, TOPDOWN_LAYER)
                seen_sources.add(str(Path(str(meta.get("source_pcd"))).resolve()).lower())
            if not layers:
                continue
            debug_variants = ("floorlow", "obstacles_high", "zwide", "combined")
            result.append({
                "name": item.name,
                "is_debug": any(token in item.name for token in debug_variants),
                "raw": False,
                "meta": meta,
                "layers": layers,
                "routes": list_routes(RAW_MAP_ROOT / "routes" / Path(str(meta.get("source_pcd", item.name))).stem),
            })

    if RAW_MAP_ROOT.exists():
        for pcd in sorted(RAW_MAP_ROOT.glob("*.pcd")):
            resolved = str(pcd.resolve()).lower()
            if resolved in seen_sources:
                continue
            try:
                meta = scan_raw_pcd_meta(pcd)
            except Exception:
                continue
            result.append({
                "name": pcd.stem,
                "is_debug": False,
                "raw": True,
                "meta": meta,
                "layers": [TOPDOWN_LAYER],
                "routes": list_routes(RAW_MAP_ROOT / "routes" / pcd.stem),
            })
    return result


def list_routes(directory: Path) -> list[str]:
    if not directory.is_dir():
        return []
    return sorted(p.name for p in directory.glob("*.json"))


def image_info(path: Path, meta: dict[str, Any]) -> dict[str, Any]:
    if path.name == TOPDOWN_LAYER:
        width = int(meta.get("width", 0))
        height = int(meta.get("height", 0))
    elif path.suffix.lower() == ".pgm":
        width, height, _ = load_pgm(path)
    elif path.suffix.lower() == ".png":
        width = int(meta.get("width", 0))
        height = int(meta.get("height", 0))
    else:
        width = height = 0
    return {
        "width": width,
        "height": height,
        "resolution": float(meta.get("resolution", 1.0)),
        "origin": meta.get("origin", [0.0, 0.0, 0.0]),
    }


def resolve_source_pcd(meta: dict[str, Any]) -> Path:
    raw = str(meta.get("source_pcd", "")).strip()
    if not raw:
        raise RuntimeError("meta.yaml does not contain source_pcd")
    path = Path(raw).expanduser()
    if path.exists():
        return path.resolve()
    candidate = (ROOT / raw).resolve()
    if candidate.exists():
        return candidate
    raise FileNotFoundError(raw)


def iter_ascii_pcd_points(source: Path):
    with source.open("r", encoding="utf-8") as f:
        data_started = False
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if data_started:
                parts = line.split()
                if len(parts) < 3:
                    continue
                try:
                    yield float(parts[0]), float(parts[1]), float(parts[2])
                except ValueError:
                    continue
            elif line.upper().startswith("DATA"):
                if "ascii" not in line.lower():
                    raise RuntimeError(f"Only ASCII PCD is supported: {source}")
                data_started = True


def rgb_png(width: int, height: int, pixels: bytearray) -> bytes:
    rows = bytearray()
    for row in range(height):
        rows.append(0)
        start = row * width * 3
        rows.extend(pixels[start:start + width * 3])
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    out = b"\x89PNG\r\n\x1a\n"
    out += png_chunk(b"IHDR", ihdr)
    out += png_chunk(b"IDAT", zlib.compress(bytes(rows), level=6))
    out += png_chunk(b"IEND", b"")
    return out


def load_z_filtered_topdown(meta: dict[str, Any], z_min: float = -2.0, z_max: float = 1.0) -> bytes:
    source = resolve_source_pcd(meta)
    resolution = float(meta.get("resolution", 0.05))
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    width = int(meta.get("width", 0))
    height = int(meta.get("height", 0))
    if width <= 0 or height <= 0:
        raise RuntimeError("meta.yaml missing width/height")

    key = (source, z_min, z_max, resolution, float(origin[0]), width, height)
    mtime = source.stat().st_mtime
    cached = TOPDOWN_CACHE.get(key)
    if cached and cached[0] == mtime:
        return cached[1]

    count = [0] * (width * height)
    max_z = [-math.inf] * (width * height)
    min_z = [math.inf] * (width * height)
    for x, y, z in iter_ascii_pcd_points(source):
        if z < z_min or z > z_max:
            continue
        ix = int(math.floor((x - float(origin[0])) / resolution))
        iy = int(math.floor((y - float(origin[1])) / resolution))
        if ix < 0 or iy < 0 or ix >= width or iy >= height:
            continue
        idx = iy * width + ix
        count[idx] += 1
        max_z[idx] = max(max_z[idx], z)
        min_z[idx] = min(min_z[idx], z)

    max_count = max(count) if count else 1
    max_count = max(max_count, 1)
    pixels = bytearray(width * height * 3)
    for row in range(height):
        src_y = height - 1 - row
        for col in range(width):
            idx = src_y * width + col
            out = (row * width + col) * 3
            if count[idx] <= 0:
                pixels[out:out + 3] = bytes((13, 17, 23))
                continue
            density = min(1.0, math.log1p(count[idx]) / math.log1p(max_count))
            z_norm = (max_z[idx] - z_min) / max(1.0e-6, z_max - z_min)
            z_norm = max(0.0, min(1.0, z_norm))
            span = max(0.0, max_z[idx] - min_z[idx])
            edge = min(1.0, span / 0.35)
            r = int(42 + 155 * z_norm + 55 * edge)
            g = int(105 + 95 * (1.0 - abs(z_norm - 0.45)) + 25 * density)
            b = int(120 + 85 * (1.0 - z_norm))
            boost = 0.45 + 0.55 * density
            pixels[out:out + 3] = bytes((
                max(0, min(255, int(r * boost))),
                max(0, min(255, int(g * boost))),
                max(0, min(255, int(b * boost))),
            ))

    payload = rgb_png(width, height, pixels)
    TOPDOWN_CACHE[key] = (mtime, payload)
    return payload


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
    # q * v * q^-1, expanded to avoid a dependency just for template parsing.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def oriented_rect(cx: float, cy: float, hx: float, hy: float, yaw: float) -> list[list[float]]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    points = []
    for lx, ly in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
        points.append([cx + lx * c - ly * s, cy + lx * s + ly * c])
    return points


def classify_geom(kind: str, pos: list[float], size: list[float], quat: list[float], hfield: str) -> dict[str, str]:
    x = pos[0] if len(pos) > 0 else 0.0
    y = pos[1] if len(pos) > 1 else 0.0
    z = pos[2] if len(pos) > 2 else 0.0
    sx = size[0] if len(size) > 0 else 0.0
    sy = size[1] if len(size) > 1 else sx
    sz = size[2] if len(size) > 2 else 0.0
    local_z = rotate_quat_vec(quat, (0.0, 0.0, 1.0))
    horizontal_axis = math.hypot(local_z[0], local_z[1]) > 0.45

    if kind == "hfield":
        return {"label": "rough_hfield", "policy": "rough", "action": "drive"}
    if kind == "cylinder" and horizontal_axis:
        return {"label": "low_bar", "policy": "crawl", "action": "pass_low_bar"}
    if kind == "cylinder" and y < -9.5 and sx <= 0.08:
        return {"label": "slalom_pole", "policy": "rough", "action": "drive"}
    if y < -11.0 and sx >= 0.35 and sy >= 0.35 and z < 0.2:
        return {"label": "rough_pit", "policy": "rough", "action": "drive"}
    if y < -6.0 and min(sx, sy) <= 0.06 and sz >= 0.12:
        return {"label": "wall", "policy": "rough", "action": "charge_wall"}
    if -5.1 <= y <= -2.0 and 1.2 <= x <= 3.4:
        return {"label": "stairs", "policy": "rough", "action": "climb"}
    if -3.8 <= y <= 0.6 and 1.2 <= x <= 6.3:
        return {"label": "bridge", "policy": "rough", "action": "drive"}
    if -6.2 <= y <= -4.2 and 4.0 <= x <= 6.5:
        return {"label": "ramp", "policy": "rough", "action": "drive"}
    if 1.5 <= y <= 2.5 and 0.6 <= x <= 6.3:
        return {"label": "stairs_test", "policy": "rough", "action": "climb"}
    if 3.0 <= y <= 4.8 and 0.2 <= x <= 3.8:
        return {"label": "ramp_test", "policy": "rough", "action": "drive"}
    if 5.0 <= y <= 7.0 and -2.8 <= x <= 3.4:
        return {"label": "rough_test", "policy": "rough", "action": "drive"}
    return {"label": "structure", "policy": "rough", "action": "drive"}


def semantic_region_from_geom(index: int, geom: ET.Element) -> Optional[dict[str, Any]]:
    kind = geom.attrib.get("type", "sphere")
    name = geom.attrib.get("name", f"geom_{index}")
    if kind == "plane" or name == "floor":
        return None

    pos = parse_float_list(geom.attrib.get("pos", ""), [0.0, 0.0, 0.0])
    size = parse_float_list(geom.attrib.get("size", ""), [0.0, 0.0, 0.0])
    quat = parse_float_list(geom.attrib.get("quat", ""), [1.0, 0.0, 0.0, 0.0])
    yaw = quat_to_yaw(quat)
    hfield = geom.attrib.get("hfield", "")
    meta = classify_geom(kind, pos, size, quat, hfield)

    if kind in ("box", "hfield"):
        hx = size[0] if len(size) > 0 else 0.0
        hy = size[1] if len(size) > 1 else hx
        if hx <= 0.0 or hy <= 0.0:
            return None
        polygon = oriented_rect(pos[0], pos[1], hx, hy, yaw)
        shape = "polygon"
    elif kind == "cylinder":
        radius = size[0] if len(size) > 0 else 0.0
        half_len = size[1] if len(size) > 1 else radius
        local_z = rotate_quat_vec(quat, (0.0, 0.0, 1.0))
        if math.hypot(local_z[0], local_z[1]) > 0.45:
            yaw = math.atan2(local_z[1], local_z[0])
            polygon = oriented_rect(pos[0], pos[1], max(half_len, radius), max(radius, 0.02), yaw)
            shape = "capsule_rect"
        else:
            steps = 18
            polygon = [
                [
                    pos[0] + radius * math.cos(2.0 * math.pi * i / steps),
                    pos[1] + radius * math.sin(2.0 * math.pi * i / steps),
                ]
                for i in range(steps)
            ]
            shape = "circle"
    else:
        return None

    return {
        "id": f"{index:03d}_{name}",
        "source_name": name,
        "type": kind,
        "shape": shape,
        "is_test": meta["label"].endswith("_test"),
        "label": meta["label"],
        "policy": meta["policy"],
        "action": meta["action"],
        "pos": pos,
        "size": size,
        "yaw": yaw,
        "polygon": polygon,
    }


def bounds_polygon(regions: list[dict[str, Any]], margin: float = 0.0) -> list[list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    for region in regions:
        for x, y in region.get("polygon", []):
            xs.append(float(x))
            ys.append(float(y))
    if not xs or not ys:
        return []
    return [
        [min(xs) - margin, min(ys) - margin],
        [max(xs) + margin, min(ys) - margin],
        [max(xs) + margin, max(ys) + margin],
        [min(xs) - margin, max(ys) + margin],
    ]


def make_competition_regions(regions: list[dict[str, Any]]) -> list[dict[str, Any]]:
    specs = [
        ("wall", "wall", "rough", "charge_wall", 0.25),
        ("rough_pit", "rough_pit", "rough", "drive", 0.20),
        ("low_bar", "low_bar", "crawl", "pass_low_bar", 0.25),
        ("slalom_pole", "slalom", "rough", "drive", 0.35),
        ("stairs", "stairs", "rough", "climb", 0.20),
        ("bridge", "bridge", "rough", "drive", 0.20),
        ("ramp", "ramp", "rough", "drive", 0.20),
    ]
    result: list[dict[str, Any]] = []
    for label, out_label, policy, action, margin in specs:
        group = [r for r in regions if r.get("label") == label and not r.get("is_test")]
        polygon = bounds_polygon(group, margin)
        if not polygon:
            continue
        result.append({
            "id": out_label,
            "source_name": out_label,
            "type": "semantic_region",
            "shape": "polygon",
            "is_test": False,
            "label": out_label,
            "policy": policy,
            "action": action,
            "polygon": polygon,
            "members": [r.get("id") for r in group],
        })
    return result


def load_semantic_template(path: Path) -> dict[str, Any]:
    path = path.expanduser().resolve()
    mtime = path.stat().st_mtime
    cached = SEMANTIC_CACHE.get(path)
    if cached and cached[0] == mtime:
        return cached[1]

    tree = ET.parse(path)
    root = tree.getroot()
    regions: list[dict[str, Any]] = []
    for index, geom in enumerate(root.findall(".//worldbody/geom")):
        region = semantic_region_from_geom(index, geom)
        if region is not None:
            regions.append(region)

    labels: dict[str, int] = {}
    for region in regions:
        label = str(region["label"])
        labels[label] = labels.get(label, 0) + 1
    competition_regions = make_competition_regions(regions)

    payload = {
        "source": str(path),
        "frame_id": "sim_mjcf_xy",
        "regions": competition_regions,
        "raw_regions": regions,
        "label_counts": labels,
        "default_alignment": {
            "dx": 0.0,
            "dy": 0.0,
            "yaw_deg": 0.0,
            "scale": 1.0,
        },
    }
    SEMANTIC_CACHE[path] = (mtime, payload)
    return payload


def load_pointcloud(meta: dict[str, Any], max_points: int, z_min: float, z_max: float) -> dict[str, Any]:
    source = resolve_source_pcd(meta)
    max_points = max(1000, min(max_points, 300000))
    z_min = float(z_min)
    z_max = float(z_max)
    mtime = source.stat().st_mtime
    key = (source, max_points, z_min, z_max)
    cached = POINT_CLOUD_CACHE.get(key)
    if cached and cached[0] == mtime:
        return cached[1]

    meta_point_count = int(meta.get("point_count", 0) or 0)
    total_point_count = meta_point_count if meta_point_count > 0 else 0
    filtered_point_count = 0
    if meta_point_count > 0:
        for _, _, z in iter_ascii_pcd_points(source):
            if z_min <= z <= z_max:
                filtered_point_count += 1
    else:
        for _, _, z in iter_ascii_pcd_points(source):
            total_point_count += 1
            if z_min <= z <= z_max:
                filtered_point_count += 1

    stride = max(1, math.ceil(filtered_point_count / max_points)) if filtered_point_count > 0 else 1
    positions: list[float] = []
    sampled = 0
    kept_seen = 0
    min_x = min_y = min_z = math.inf
    max_x = max_y = max_z = -math.inf
    for x, y, z in iter_ascii_pcd_points(source):
        if z < z_min or z > z_max:
            continue
        if kept_seen % stride == 0:
            positions.extend((x, y, z))
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            min_z = min(min_z, z)
            max_x = max(max_x, x)
            max_y = max(max_y, y)
            max_z = max(max_z, z)
            sampled += 1
        kept_seen += 1

    if sampled == 0:
        min_x = min_y = min_z = 0.0
        max_x = max_y = max_z = 0.0

    payload = {
        "source": str(source),
        "point_count": total_point_count,
        "filtered_point_count": filtered_point_count,
        "sampled_count": sampled,
        "stride": stride,
        "z_min": z_min,
        "z_max": z_max,
        "bounds": {
            "min": [min_x, min_y, min_z],
            "max": [max_x, max_y, max_z],
        },
        "positions": positions,
    }
    POINT_CLOUD_CACHE[key] = (mtime, payload)
    return payload


def filter_voxel_clusters(
    occupied: list[tuple[int, int, int]],
    min_cluster_voxels: int,
) -> tuple[list[tuple[int, int, int]], int]:
    if min_cluster_voxels <= 1 or not occupied:
        return occupied, 0

    occupied_set = set(occupied)
    visited: set[tuple[int, int, int]] = set()
    kept: list[tuple[int, int, int]] = []
    removed = 0

    for seed in occupied:
        if seed in visited:
            continue

        queue: deque[tuple[int, int, int]] = deque([seed])
        visited.add(seed)
        cluster: list[tuple[int, int, int]] = []

        while queue:
            current = queue.popleft()
            cluster.append(current)
            cx, cy, cz = current
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        neighbor = (cx + dx, cy + dy, cz + dz)
                        if neighbor in occupied_set and neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)

        if len(cluster) >= min_cluster_voxels:
            kept.extend(cluster)
        else:
            removed += len(cluster)

    return kept, removed


def load_voxels(
    meta: dict[str, Any],
    voxel_size: float,
    z_min: float,
    z_max: float,
    min_points_per_voxel: int,
    min_cluster_voxels: int,
    max_voxels: int,
) -> dict[str, Any]:
    source = resolve_source_pcd(meta)
    voxel_size = max(0.01, min(float(voxel_size), 2.0))
    z_min = float(z_min)
    z_max = float(z_max)
    min_points_per_voxel = max(1, min(int(min_points_per_voxel), 100))
    min_cluster_voxels = max(1, min(int(min_cluster_voxels), 10000))
    max_voxels = max(1000, min(int(max_voxels), 300000))
    mtime = source.stat().st_mtime
    key = (source, voxel_size, z_min, z_max, min_points_per_voxel, min_cluster_voxels, max_voxels)
    cached = VOXEL_CACHE.get(key)
    if cached and cached[0] == mtime:
        return cached[1]

    voxels: dict[tuple[int, int, int], int] = {}
    filtered_point_count = 0
    for x, y, z in iter_ascii_pcd_points(source):
        if z < z_min or z > z_max:
            continue
        filtered_point_count += 1
        key_xyz = (
            int(math.floor(x / voxel_size)),
            int(math.floor(y / voxel_size)),
            int(math.floor(z / voxel_size)),
        )
        voxels[key_xyz] = voxels.get(key_xyz, 0) + 1

    occupied = [k for k, count in voxels.items() if count >= min_points_per_voxel]
    threshold_voxel_count = len(occupied)
    occupied, removed_cluster_voxels = filter_voxel_clusters(occupied, min_cluster_voxels)
    occupied_count = len(occupied)
    stride = max(1, math.ceil(occupied_count / max_voxels)) if occupied_count > 0 else 1

    positions: list[float] = []
    sampled_count = 0
    min_x = min_y = min_z = math.inf
    max_x = max_y = max_z = -math.inf
    half = voxel_size * 0.5
    for index, key_xyz in enumerate(occupied):
        if index % stride != 0:
            continue
        x = (key_xyz[0] + 0.5) * voxel_size
        y = (key_xyz[1] + 0.5) * voxel_size
        z = (key_xyz[2] + 0.5) * voxel_size
        positions.extend((x, y, z))
        min_x = min(min_x, x - half)
        min_y = min(min_y, y - half)
        min_z = min(min_z, z - half)
        max_x = max(max_x, x + half)
        max_y = max(max_y, y + half)
        max_z = max(max_z, z + half)
        sampled_count += 1

    if sampled_count == 0:
        min_x = min_y = min_z = 0.0
        max_x = max_y = max_z = 0.0

    payload = {
        "source": str(source),
        "voxel_size": voxel_size,
        "z_min": z_min,
        "z_max": z_max,
        "min_points_per_voxel": min_points_per_voxel,
        "min_cluster_voxels": min_cluster_voxels,
        "filtered_point_count": filtered_point_count,
        "counted_voxel_count": len(voxels),
        "threshold_voxel_count": threshold_voxel_count,
        "removed_cluster_voxels": removed_cluster_voxels,
        "occupied_voxel_count": occupied_count,
        "sampled_voxel_count": sampled_count,
        "stride": stride,
        "bounds": {
            "min": [min_x, min_y, min_z],
            "max": [max_x, max_y, max_z],
        },
        "positions": positions,
    }
    VOXEL_CACHE[key] = (mtime, payload)
    return payload


def clean_waypoint(wp: dict[str, Any]) -> dict[str, Any]:
    yaw_tolerance_deg = wp.get("yawToleranceDeg", wp.get("yaw_tolerance_deg"))
    require_yaw = wp.get("requireYaw", wp.get("require_yaw"))
    pre_dock_distance = wp.get("preDockDistance", wp.get("pre_dock_distance"))
    pre_dock_tolerance = wp.get("preDockTolerance", wp.get("pre_dock_tolerance"))
    return {
        "x": float(wp.get("x", 0.0)),
        "y": float(wp.get("y", 0.0)),
        "yawDeg": float(wp.get("yawDeg", wp.get("yaw_deg", 0.0))),
        "yawToleranceDeg": (
            float(yaw_tolerance_deg)
            if yaw_tolerance_deg is not None else None
        ),
        "speed": float(wp.get("speed", 0.35)),
        "policy": str(wp.get("policy", "rough")),
        "tolerance": float(wp.get("tolerance", 0.15)),
        "requireYaw": bool(require_yaw) if require_yaw is not None else False,
        "preDockDistance": (
            float(pre_dock_distance)
            if pre_dock_distance is not None else None
        ),
        "preDockTolerance": (
            float(pre_dock_tolerance)
            if pre_dock_tolerance is not None else None
        ),
        "obstacle": str(wp.get("obstacle", "flat") or "flat"),
        "obstacleName": str(wp.get("obstacleName", wp.get("obstacle_name", "")) or ""),
    }


def clean_segment(segment: dict[str, Any], index: int) -> dict[str, Any]:
    raw_waypoints = segment.get("waypoints", [])
    return {
        "name": str(segment.get("name", f"segment_{index + 1}") or f"segment_{index + 1}"),
        "obstacle": str(segment.get("obstacle", "flat") or "flat"),
        "waypoints": [
            clean_waypoint(wp)
            for wp in raw_waypoints
            if isinstance(wp, dict)
        ],
    }


def clean_obstacle(item: dict[str, Any], index: int) -> dict[str, Any]:
    obstacle = str(item.get("obstacle", "rough_pit") or "rough_pit")
    return {
        "name": str(item.get("name", f"obstacle_{index + 1}") or f"obstacle_{index + 1}"),
        "obstacle": obstacle,
        "x": float(item.get("x", 0.0)),
        "y": float(item.get("y", 0.0)),
        "yawDeg": float(item.get("yawDeg", item.get("yaw_deg", 0.0))),
        "length": max(0.05, float(item.get("length", 1.0))),
        "width": max(0.05, float(item.get("width", 1.0))),
        "policy": str(item.get("policy", "crawl" if obstacle == "low_bar" else "rough")),
    }


def route_waypoints(route: dict[str, Any]) -> list[dict[str, Any]]:
    raw_segments = route.get("segments", [])
    if isinstance(raw_segments, list) and raw_segments:
        points: list[dict[str, Any]] = []
        for index, segment in enumerate(raw_segments):
            if not isinstance(segment, dict):
                continue
            cleaned_segment = clean_segment(segment, index)
            points.extend(cleaned_segment["waypoints"])
        if points:
            return points
    return [
        clean_waypoint(wp)
        for wp in route.get("waypoints", [])
        if isinstance(wp, dict)
    ]


def route_segments(route: dict[str, Any]) -> list[dict[str, Any]]:
    raw_segments = route.get("segments", [])
    if isinstance(raw_segments, list) and raw_segments:
        segments = [
            clean_segment(segment, index)
            for index, segment in enumerate(raw_segments)
            if isinstance(segment, dict)
        ]
        segments = [segment for segment in segments if segment["waypoints"]]
        if segments:
            return segments
    return [{
        "name": "segment_1",
        "obstacle": "flat",
        "waypoints": route_waypoints(route),
    }] if route_waypoints(route) else []


def route_obstacles(route: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        clean_obstacle(item, index)
        for index, item in enumerate(route.get("obstacles", []))
        if isinstance(item, dict)
    ]


def route_yaml(route: dict[str, Any]) -> str:
    obstacles = route_obstacles(route)
    segments = route_segments(route)
    lines = [
        f"name: {route.get('name', 'route')}",
        f"map: {route.get('map', '')}",
        f"frame_id: {route.get('frame_id', 'map')}",
    ]
    route_default_yaw_tol = route.get("yawToleranceDegDefault", route.get("yaw_tolerance_deg_default"))
    if route_default_yaw_tol is not None:
        lines.append(f"yaw_tolerance_deg_default: {float(route_default_yaw_tol):.2f}")
    route_default_require_yaw = route.get("requireYawDefault", route.get("require_yaw_default"))
    if route_default_require_yaw is not None:
        lines.append(f"require_yaw_default: {str(bool(route_default_require_yaw)).lower()}")
    route_default_pre_dock_distance = route.get("preDockDistanceDefault", route.get("pre_dock_distance_default"))
    if route_default_pre_dock_distance is not None:
        lines.append(f"pre_dock_distance_default: {float(route_default_pre_dock_distance):.3f}")
    route_default_pre_dock_tolerance = route.get("preDockToleranceDefault", route.get("pre_dock_tolerance_default"))
    if route_default_pre_dock_tolerance is not None:
        lines.append(f"pre_dock_tolerance_default: {float(route_default_pre_dock_tolerance):.3f}")
    if obstacles:
        lines.append("obstacles:")
    else:
        lines.append("obstacles: []")
    for index, obstacle in enumerate(obstacles, start=1):
        lines.extend([
            f"  - id: {index}",
            f"    name: {obstacle.get('name', 'obstacle')}",
            f"    obstacle: {obstacle.get('obstacle', 'rough_pit')}",
            f"    x: {float(obstacle.get('x', 0.0)):.4f}",
            f"    y: {float(obstacle.get('y', 0.0)):.4f}",
            f"    yaw_deg: {float(obstacle.get('yawDeg', 0.0)):.2f}",
            f"    length: {float(obstacle.get('length', 1.0)):.3f}",
            f"    width: {float(obstacle.get('width', 1.0)):.3f}",
            f"    policy: {obstacle.get('policy', 'rough')}",
        ])
    if segments:
        lines.append("segments:")
    else:
        lines.append("segments: []")
    for segment_index, segment in enumerate(segments, start=1):
        lines.extend([
            f"  - name: {segment.get('name', f'segment_{segment_index}')}",
            f"    obstacle: {segment.get('obstacle', 'flat')}",
            "    waypoints:",
        ])
        for waypoint_index, wp in enumerate(segment.get("waypoints", []), start=1):
            lines.extend([
                f"      - id: {waypoint_index}",
                f"        x: {float(wp.get('x', 0.0)):.4f}",
                f"        y: {float(wp.get('y', 0.0)):.4f}",
                f"        yaw_deg: {float(wp.get('yawDeg', 0.0)):.2f}",
                f"        speed: {float(wp.get('speed', 0.35)):.3f}",
                f"        policy: {wp.get('policy', 'rough')}",
                f"        tolerance: {float(wp.get('tolerance', 0.15)):.3f}",
                (
                    f"        yaw_tolerance_deg: {float(wp.get('yawToleranceDeg')):.2f}"
                    if wp.get("yawToleranceDeg") is not None else None
                ),
                (
                    f"        require_yaw: {str(bool(wp.get('requireYaw'))).lower()}"
                    if wp.get("requireYaw") is not None else None
                ),
                (
                    f"        pre_dock_distance: {float(wp.get('preDockDistance')):.3f}"
                    if wp.get("preDockDistance") is not None else None
                ),
                (
                    f"        pre_dock_tolerance: {float(wp.get('preDockTolerance')):.3f}"
                    if wp.get("preDockTolerance") is not None else None
                ),
                f"        obstacle: {wp.get('obstacle', 'flat')}",
                f"        obstacle_name: {wp.get('obstacleName', '')}",
            ])
    lines = [line for line in lines if line is not None]
    return "\n".join(lines) + "\n"


def clean_route(route: dict[str, Any]) -> dict[str, Any]:
    cleaned = {
        "name": str(route.get("name", "route")),
        "map": str(route.get("map", "")),
        "frame_id": str(route.get("frame_id", "map")),
        "obstacles": route_obstacles(route),
        "segments": route_segments(route),
        "waypoints": route_waypoints(route),
    }
    route_default_yaw_tol = route.get("yawToleranceDegDefault", route.get("yaw_tolerance_deg_default"))
    if route_default_yaw_tol is not None:
        cleaned["yawToleranceDegDefault"] = float(route_default_yaw_tol)
    route_default_require_yaw = route.get("requireYawDefault", route.get("require_yaw_default"))
    if route_default_require_yaw is not None:
        cleaned["requireYawDefault"] = bool(route_default_require_yaw)
    route_default_pre_dock_distance = route.get("preDockDistanceDefault", route.get("pre_dock_distance_default"))
    if route_default_pre_dock_distance is not None:
        cleaned["preDockDistanceDefault"] = float(route_default_pre_dock_distance)
    route_default_pre_dock_tolerance = route.get("preDockToleranceDefault", route.get("pre_dock_tolerance_default"))
    if route_default_pre_dock_tolerance is not None:
        cleaned["preDockToleranceDefault"] = float(route_default_pre_dock_tolerance)
    if route.get("createdAt"):
        cleaned["createdAt"] = str(route.get("createdAt"))
    return cleaned


class Handler(SimpleHTTPRequestHandler):
    map_root: Path = DEFAULT_MAP_ROOT

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        try:
            if parsed.path == "/api/maps":
                self.send_json({"maps": list_maps(self.map_root)})
                return
            if parsed.path.startswith("/vendor/"):
                self.handle_vendor(parsed.path)
                return
            if parsed.path == "/api/layer":
                qs = parse_qs(parsed.query)
                map_name = qs.get("map", [""])[0]
                layer_name = safe_name(qs.get("layer", [""])[0])
                ctx = map_context(self.map_root, map_name)
                directory = ctx["directory"]
                meta = ctx["meta"]
                path = directory / layer_name if directory is not None else Path(layer_name)
                if layer_name == TOPDOWN_LAYER:
                    self.send_bytes(200, "image/png", load_z_filtered_topdown(meta))
                elif path.suffix.lower() == ".pgm":
                    self.send_bytes(200, "image/png", gray_pgm_to_png(path))
                elif path.suffix.lower() == ".png":
                    self.send_bytes(200, "image/png", path.read_bytes())
                else:
                    self.send_error(404)
                return
            if parsed.path == "/api/layer_info":
                qs = parse_qs(parsed.query)
                ctx = map_context(self.map_root, qs.get("map", [""])[0])
                layer_name = safe_name(qs.get("layer", [""])[0])
                directory = ctx["directory"]
                path = directory / layer_name if directory is not None else Path(layer_name)
                self.send_json(image_info(path, ctx["meta"]))
                return
            if parsed.path == "/api/cell":
                self.handle_cell(parsed.query)
                return
            if parsed.path == "/api/pointcloud":
                qs = parse_qs(parsed.query)
                ctx = map_context(self.map_root, qs.get("map", [""])[0])
                max_points = int(qs.get("max_points", ["80000"])[0])
                z_min = float(qs.get("z_min", ["-1000000000.0"])[0])
                z_max = float(qs.get("z_max", ["1.0"])[0])
                self.send_json(load_pointcloud(ctx["meta"], max_points, z_min, z_max))
                return
            if parsed.path == "/api/voxels":
                qs = parse_qs(parsed.query)
                ctx = map_context(self.map_root, qs.get("map", [""])[0])
                meta = ctx["meta"]
                voxel_size = float(qs.get("voxel_size", ["0.20"])[0])
                z_min = float(qs.get("z_min", ["-2.0"])[0])
                z_max = float(qs.get("z_max", ["1.0"])[0])
                min_points_per_voxel = int(qs.get("min_points_per_voxel", ["1"])[0])
                min_cluster_voxels = int(qs.get("min_cluster_voxels", ["1"])[0])
                max_voxels = int(qs.get("max_voxels", ["120000"])[0])
                self.send_json(
                    load_voxels(
                        meta,
                        voxel_size,
                        z_min,
                        z_max,
                        min_points_per_voxel,
                        min_cluster_voxels,
                        max_voxels,
                    )
                )
                return
            if parsed.path == "/api/semantics":
                qs = parse_qs(parsed.query)
                raw_path = qs.get("mjcf", [""])[0].strip()
                path = Path(raw_path) if raw_path else DEFAULT_MJCF
                self.send_json(load_semantic_template(path))
                return
            if parsed.path == "/api/route":
                self.handle_get_route(parsed.query)
                return
        except Exception as exc:
            self.send_json({"ok": False, "error": str(exc)}, code=400)
            return
        super().do_GET()

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        try:
            if parsed.path == "/api/route":
                length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(length).decode("utf-8") if length else "{}")
                map_name = str(payload.get("map", ""))
                route = payload.get("route", {})
                name = safe_file_stem(str(route.get("name", "route")))
                ctx = map_context(self.map_root, map_name)
                routes_dir = ctx["routes_dir"]
                routes_dir.mkdir(parents=True, exist_ok=True)
                route["name"] = name
                route = clean_route(route)
                route["name"] = name
                json_path = routes_dir / f"{name}.json"
                yaml_path = routes_dir / f"{name}.yaml"
                json_path.write_text(json.dumps(route, ensure_ascii=False, indent=2), encoding="utf-8")
                yaml_path.write_text(route_yaml(route), encoding="utf-8")
                self.send_json({"ok": True, "json": str(json_path), "yaml": str(yaml_path)})
                return
        except Exception as exc:
            self.send_json({"ok": False, "error": str(exc)}, code=400)
            return
        self.send_error(404)

    def handle_cell(self, query: str) -> None:
        qs = parse_qs(query)
        ctx = map_context(self.map_root, qs.get("map", [""])[0])
        directory = ctx["directory"]
        meta = ctx["meta"]
        x = float(qs.get("x", ["0"])[0])
        y = float(qs.get("y", ["0"])[0])
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        resolution = float(meta.get("resolution", 1.0))
        width = int(meta.get("width", 0))
        height = int(meta.get("height", 0))
        ix = int(math.floor((x - float(origin[0])) / resolution))
        iy = int(math.floor((y - float(origin[1])) / resolution))
        if ix < 0 or iy < 0 or ix >= width or iy >= height:
            self.send_json({"inside": False, "x": x, "y": y})
            return
        image_row = height - 1 - iy
        index = image_row * width + ix
        values = {}
        if directory is not None:
            for layer in sorted(directory.glob("*.pgm")):
                lw, lh, pixels = load_pgm(layer)
                if lw == width and lh == height and 0 <= index < len(pixels):
                    values[layer.name] = pixels[index]
        self.send_json({
            "inside": True,
            "x": x,
            "y": y,
            "ix": ix,
            "iy": iy,
            "image_row": image_row,
            "values": values,
        })

    def handle_get_route(self, query: str) -> None:
        qs = parse_qs(query)
        ctx = map_context(self.map_root, qs.get("map", [""])[0])
        route_name = safe_name(qs.get("name", [""])[0])
        path = ctx["routes_dir"] / route_name
        if not path.exists():
            raise FileNotFoundError(path)
        self.send_json(clean_route(json.loads(path.read_text(encoding="utf-8"))))

    def handle_vendor(self, request_path: str) -> None:
        relative = normpath(request_path[len("/vendor/"):]).replace("\\", "/")
        if relative.startswith("../") or relative == "..":
            raise ValueError("vendor path escapes root")
        target = (REFERENCE_VENDOR_DIR / relative).resolve()
        vendor_root = REFERENCE_VENDOR_DIR.resolve()
        if vendor_root != target and vendor_root not in target.parents:
            raise ValueError("vendor path escapes root")
        if not target.is_file():
            raise FileNotFoundError(target)
        content_type = "application/javascript"
        if target.suffix.lower() == ".css":
            content_type = "text/css"
        self.send_bytes(200, content_type, target.read_bytes())

    def send_json(self, payload: dict[str, Any], code: int = 200) -> None:
        self.send_bytes(code, "application/json", json.dumps(payload, ensure_ascii=False).encode("utf-8"))

    def send_bytes(self, code: int, content_type: str, payload: bytes) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        super().end_headers()

    def log_message(self, format: str, *args: object) -> None:
        return


def main() -> None:
    parser = argparse.ArgumentParser(description="Windows viewer for processed Odin PCD map layers")
    parser.add_argument("--map-root", type=Path, default=DEFAULT_MAP_ROOT)
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8090)
    args = parser.parse_args()

    Handler.map_root = args.map_root.expanduser().resolve()
    handler = lambda *a, **kw: Handler(*a, directory=str(STATIC_DIR), **kw)
    server = ThreadingHTTPServer((args.listen_host, args.http_port), handler)
    print(f"PCD map viewer: http://{args.listen_host}:{args.http_port}")
    print(f"Map root: {Handler.map_root}")
    print(f"Started at {time.strftime('%Y-%m-%d %H:%M:%S')}")
    server.serve_forever()


if __name__ == "__main__":
    main()
