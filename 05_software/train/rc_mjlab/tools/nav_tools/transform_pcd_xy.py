#!/usr/bin/env python3
"""Translate and yaw-rotate an ASCII PCD in XY.

Example:
  python transform_pcd_xy.py pcd/1hao.pcd --in-place --origin-x 9.34 --origin-y 0.88 --yaw-deg -90

The transform is:
  1. subtract origin from x/y
  2. rotate around (0, 0) by yaw-deg

Only ASCII PCD files are supported. Non-x/y fields are preserved.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path


TOOL_DIR = Path(__file__).resolve().parent


def tool_relative(path: Path) -> Path:
    if path.is_absolute():
        raise ValueError("Use a path relative to this nav_tools folder.")
    return TOOL_DIR / path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Translate and yaw-rotate an ASCII PCD in XY.")
    parser.add_argument("input", type=Path, help="Input .pcd file.")
    parser.add_argument("--output", type=Path, help="Output .pcd file. Required unless --in-place is used.")
    parser.add_argument("--in-place", action="store_true", help="Replace the input file after a successful transform.")
    parser.add_argument("--origin-x", type=float, default=0.0, help="X value to subtract before rotation.")
    parser.add_argument("--origin-y", type=float, default=0.0, help="Y value to subtract before rotation.")
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="Yaw rotation in degrees after origin subtraction.")
    parser.add_argument("--precision", type=int, default=8, help="Decimal precision for transformed x/y.")
    return parser.parse_args()


def fmt(value: float, precision: int) -> str:
    if abs(value) < 0.5 * 10 ** (-precision):
        value = 0.0
    return f"{value:.{precision}f}".rstrip("0").rstrip(".")


def transform_file(
    source: Path,
    target: Path,
    origin_x: float,
    origin_y: float,
    yaw_deg: float,
    precision: int,
) -> int:
    yaw = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    data_started = False
    fields: list[str] = []
    x_idx = 0
    y_idx = 1
    count = 0

    with source.open("r", encoding="utf-8", errors="ignore") as fin, target.open("w", encoding="utf-8", newline="\n") as fout:
        for line in fin:
            stripped = line.strip()
            upper = stripped.upper()
            if data_started and stripped:
                parts = stripped.split()
                if len(parts) > max(x_idx, y_idx):
                    x0 = float(parts[x_idx]) - origin_x
                    y0 = float(parts[y_idx]) - origin_y
                    x1 = x0 * cos_yaw - y0 * sin_yaw
                    y1 = x0 * sin_yaw + y0 * cos_yaw
                    parts[x_idx] = fmt(x1, precision)
                    parts[y_idx] = fmt(y1, precision)
                    fout.write(" ".join(parts) + "\n")
                    count += 1
                else:
                    fout.write(line)
            else:
                fout.write(line)
                if upper.startswith("FIELDS "):
                    fields = stripped.split()[1:]
                    if "x" in fields and "y" in fields:
                        x_idx = fields.index("x")
                        y_idx = fields.index("y")
                elif upper.startswith("DATA "):
                    if "ASCII" not in upper:
                        raise RuntimeError(f"Only ASCII PCD is supported: {source}")
                    data_started = True
    return count


def main() -> int:
    args = parse_args()
    source = tool_relative(args.input)
    if args.in_place:
        target = source.with_suffix(source.suffix + ".tmp")
    elif args.output:
        target = tool_relative(args.output)
    else:
        raise SystemExit("--output is required unless --in-place is used")

    count = transform_file(source, target, args.origin_x, args.origin_y, args.yaw_deg, args.precision)
    if count <= 0:
        target.unlink(missing_ok=True)
        raise SystemExit("No PCD points transformed")
    if args.in_place:
        target.replace(source)
    print(f"transformed {count} points: origin=({args.origin_x}, {args.origin_y}), yaw={args.yaw_deg} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
