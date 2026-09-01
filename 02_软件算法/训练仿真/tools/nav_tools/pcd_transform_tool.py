#!/usr/bin/env python3
"""Transform ASCII PCD maps by XY translation and yaw rotation.

Examples:
  python pcd_transform_tool.py pcd/1hao.pcd --dx -9.34 --dy -0.88 --yaw-deg -90 --output pcd/1hao_tf.pcd
  python pcd_transform_tool.py pcd/1hao.pcd --yaw-deg 90 --in-place
"""

from __future__ import annotations

import argparse
import math
import shutil
import time
from pathlib import Path


TOOL_DIR = Path(__file__).resolve().parent


def tool_relative(path: Path) -> Path:
    if path.is_absolute():
        raise ValueError("Use a path relative to this nav_tools folder.")
    return TOOL_DIR / path


def format_float(value: float) -> str:
    if abs(value) < 5e-10:
        value = 0.0
    return f"{value:.8f}".rstrip("0").rstrip(".")


def transform_xy(x: float, y: float, dx: float, dy: float, yaw_rad: float) -> tuple[float, float]:
    tx = x + dx
    ty = y + dy
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return tx * c - ty * s, tx * s + ty * c


def transform_ascii_pcd(source: Path, target: Path, dx: float, dy: float, yaw_deg: float) -> int:
    yaw_rad = math.radians(yaw_deg)
    data_started = False
    transformed = 0
    with source.open("r", encoding="utf-8", errors="ignore") as fin, target.open("w", encoding="utf-8", newline="\n") as fout:
        for raw in fin:
            line = raw.strip()
            if data_started and line:
                parts = line.split()
                if len(parts) >= 2:
                    x, y = transform_xy(float(parts[0]), float(parts[1]), dx, dy, yaw_rad)
                    parts[0] = format_float(x)
                    parts[1] = format_float(y)
                    fout.write(" ".join(parts) + "\n")
                    transformed += 1
                else:
                    fout.write(raw)
            else:
                fout.write(raw)
                if line.upper().startswith("DATA"):
                    if "ascii" not in line.lower():
                        raise RuntimeError(f"Only ASCII PCD is supported: {source}")
                    data_started = True
    if transformed == 0:
        raise RuntimeError(f"No points transformed: {source}")
    return transformed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Translate and yaw-rotate an ASCII PCD file.")
    parser.add_argument("pcd", type=Path, help="Input PCD path relative to this nav_tools folder.")
    parser.add_argument("--dx", type=float, default=0.0, help="X translation before rotation, meters.")
    parser.add_argument("--dy", type=float, default=0.0, help="Y translation before rotation, meters.")
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="Yaw rotation after translation, degrees.")
    parser.add_argument("--output", type=Path, help="Output PCD path relative to this nav_tools folder.")
    parser.add_argument("--in-place", action="store_true", help="Overwrite input PCD after creating a .bak copy.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = tool_relative(args.pcd)
    if not source.exists():
        raise FileNotFoundError(source)

    if args.in_place:
        target = source.with_suffix(source.suffix + ".tmp")
    elif args.output:
        target = tool_relative(args.output)
    else:
        target = source.with_name(f"{source.stem}_tf{source.suffix}")

    target.parent.mkdir(parents=True, exist_ok=True)
    count = transform_ascii_pcd(source, target, args.dx, args.dy, args.yaw_deg)

    if args.in_place:
        backup = source.with_suffix(source.suffix + f".bak_{time.strftime('%Y%m%d_%H%M%S')}")
        shutil.copy2(source, backup)
        target.replace(source)
        print(f"Transformed {count} points in-place: {source}")
        print(f"Backup: {backup}")
    else:
        print(f"Transformed {count} points: {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
