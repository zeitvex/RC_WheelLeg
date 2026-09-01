#!/usr/bin/env python3
"""Deterministically downsample an ASCII PCD file to a byte-size limit."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path


DEFAULT_MAX_BYTES = 9_900_000


def read_header(path: Path) -> tuple[list[bytes], int, int]:
    header: list[bytes] = []
    declared_points: int | None = None

    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"PCD header has no DATA line: {path}")
            header.append(line)
            fields = line.strip().split(maxsplit=1)
            if fields and fields[0].upper() == b"POINTS" and len(fields) == 2:
                declared_points = int(fields[1])
            if fields and fields[0].upper() == b"DATA":
                if len(fields) != 2 or fields[1].lower() != b"ascii":
                    raise ValueError("Only DATA ascii PCD files are supported")
                data_offset = stream.tell()
                break

    if declared_points is None:
        raise ValueError(f"PCD header has no POINTS field: {path}")
    return header, data_offset, declared_points


def render_header(header: list[bytes], point_count: int) -> bytes:
    rendered: list[bytes] = []
    replaced_width = False
    replaced_points = False

    for line in header:
        newline = b"\r\n" if line.endswith(b"\r\n") else b"\n"
        fields = line.strip().split(maxsplit=1)
        key = fields[0].upper() if fields else b""
        if key == b"WIDTH":
            rendered.append(f"WIDTH {point_count}".encode("ascii") + newline)
            replaced_width = True
        elif key == b"POINTS":
            rendered.append(f"POINTS {point_count}".encode("ascii") + newline)
            replaced_points = True
        elif key == b"HEIGHT":
            rendered.append(b"HEIGHT 1" + newline)
        else:
            rendered.append(line)

    if not replaced_width or not replaced_points:
        raise ValueError("PCD header must contain WIDTH and POINTS fields")
    return b"".join(rendered)


def measure_sample(
    path: Path,
    data_offset: int,
    stride: int,
) -> tuple[int, int, int]:
    offset = stride // 2
    source_count = 0
    selected_count = 0
    selected_bytes = 0

    with path.open("rb") as stream:
        stream.seek(data_offset)
        for line in stream:
            if not line.strip():
                continue
            if source_count % stride == offset:
                selected_count += 1
                selected_bytes += len(line)
            source_count += 1

    return source_count, selected_count, selected_bytes


def write_sample(
    source: Path,
    output: Path,
    header: bytes,
    data_offset: int,
    stride: int,
) -> None:
    offset = stride // 2
    point_index = 0
    temporary = output.with_suffix(output.suffix + ".tmp")
    output.parent.mkdir(parents=True, exist_ok=True)

    try:
        with source.open("rb") as src, temporary.open("wb") as dst:
            dst.write(header)
            src.seek(data_offset)
            for line in src:
                if not line.strip():
                    continue
                if point_index % stride == offset:
                    dst.write(line)
                point_index += 1
        os.replace(temporary, output)
    finally:
        if temporary.exists():
            temporary.unlink()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="Source DATA ascii PCD file")
    parser.add_argument("output", type=Path, help="Downsampled output PCD file")
    parser.add_argument(
        "--max-bytes",
        type=int,
        default=DEFAULT_MAX_BYTES,
        help=f"Maximum output size in bytes (default: {DEFAULT_MAX_BYTES})",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    max_bytes = int(args.max_bytes)

    if source == output:
        raise ValueError("Source and output paths must be different")
    if max_bytes <= 8192:
        raise ValueError("--max-bytes must be greater than 8192")

    header, data_offset, declared_points = read_header(source)
    data_bytes = source.stat().st_size - data_offset
    payload_budget = max_bytes - 8192
    stride = max(1, math.ceil(data_bytes / payload_budget))

    while True:
        source_count, selected_count, selected_bytes = measure_sample(
            source,
            data_offset,
            stride,
        )
        if source_count != declared_points:
            raise ValueError(
                f"POINTS declares {declared_points}, but {source_count} data rows were read"
            )
        output_header = render_header(header, selected_count)
        if len(output_header) + selected_bytes <= max_bytes:
            break
        stride += 1

    write_sample(source, output, output_header, data_offset, stride)
    output_bytes = output.stat().st_size
    if output_bytes > max_bytes:
        raise RuntimeError(f"Generated file exceeds limit: {output_bytes} > {max_bytes}")

    print(f"source={source}")
    print(f"output={output}")
    print(f"source_points={source_count}")
    print(f"output_points={selected_count}")
    print(f"stride={stride}")
    print(f"output_bytes={output_bytes}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
