#!/usr/bin/env python3
"""Generate an annotated Odin1 relocalization-frame PNG.

The wiki defines Odin1 frames as:
  I: IMU frame
  L: LiDAR / point-cloud frame
  C = camera frame

Relocalization poses are tied to the SLAM point-cloud map, so the device pose
should be treated as T_map_L unless a driver-specific TF remaps it.
"""

from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


TOOL_DIR = Path(__file__).resolve().parent
IMG_DIR = TOOL_DIR / "assets"
OUT = TOOL_DIR / "odin1_relocalization_frame.png"


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    return ImageFont.load_default()


def arrow(draw: ImageDraw.ImageDraw, start: tuple[int, int], end: tuple[int, int], color: tuple[int, int, int], width: int = 5) -> None:
    draw.line([start, end], fill=color, width=width)
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    length = max((dx * dx + dy * dy) ** 0.5, 1.0)
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    head = 18
    wing = 9
    points = [
        (ex, ey),
        (int(ex - ux * head + px * wing), int(ey - uy * head + py * wing)),
        (int(ex - ux * head - px * wing), int(ey - uy * head - py * wing)),
    ]
    draw.polygon(points, fill=color)


def label_box(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    fill: tuple[int, int, int],
    text_color: tuple[int, int, int] = (255, 255, 255),
    size: int = 24,
) -> None:
    x, y = xy
    fnt = font(size, bold=True)
    lines = text.splitlines()
    widths = [draw.textbbox((0, 0), line, font=fnt)[2] for line in lines]
    heights = [draw.textbbox((0, 0), line, font=fnt)[3] - draw.textbbox((0, 0), line, font=fnt)[1] for line in lines]
    box_w = max(widths) + 28
    box_h = sum(heights) + 14 * (len(lines) - 1) + 24
    draw.rounded_rectangle((x, y, x + box_w, y + box_h), radius=10, fill=fill, outline=(255, 255, 255), width=2)
    cy = y + 12
    for line, h in zip(lines, heights):
        draw.text((x + 14, cy), line, fill=text_color, font=fnt)
        cy += h + 14


def main() -> int:
    coordinate = Image.open(IMG_DIR / "coordinate.png").convert("RGB")
    structure = Image.open(IMG_DIR / "structure1.png").convert("RGB")

    target_w = 1180
    coordinate = coordinate.resize((target_w, int(coordinate.height * target_w / coordinate.width)))
    structure = structure.resize((target_w, int(structure.height * target_w / structure.width)))

    gap = 24
    margin = 30
    title_h = 118
    canvas = Image.new("RGB", (target_w + margin * 2, title_h + coordinate.height + gap + structure.height + margin), (245, 247, 250))
    draw = ImageDraw.Draw(canvas)

    title_font = font(34, bold=True)
    body_font = font(22)
    draw.text((margin, 22), "Odin1 relocalization frame: LiDAR / point-cloud frame L", fill=(15, 23, 42), font=title_font)
    draw.text(
        (margin, 68),
        "Use the relocalization pose as T_map_L. Convert to robot base with your measured T_L_base.",
        fill=(51, 65, 85),
        font=body_font,
    )

    top_y = title_h
    bot_y = title_h + coordinate.height + gap
    canvas.paste(coordinate, (margin, top_y))
    canvas.paste(structure, (margin, bot_y))

    red = (220, 38, 38)
    blue = (37, 99, 235)
    green = (22, 163, 74)
    amber = (217, 119, 6)

    # coordinate.png positions after scaling to 1180 px.
    lidar_origin = (margin + 690, top_y + 255)
    imu_origin = (margin + 286, top_y + 655)
    camera_origin = (margin + 885, top_y + 500)

    draw.ellipse((lidar_origin[0] - 12, lidar_origin[1] - 12, lidar_origin[0] + 12, lidar_origin[1] + 12), fill=red, outline=(255, 255, 255), width=3)
    arrow(draw, (margin + 900, top_y + 150), lidar_origin, red, width=6)
    label_box(draw, (margin + 725, top_y + 70), "Relocalization pose\nis here: frame L", red)

    draw.ellipse((imu_origin[0] - 9, imu_origin[1] - 9, imu_origin[0] + 9, imu_origin[1] + 9), fill=blue)
    arrow(draw, (margin + 160, top_y + 555), imu_origin, blue, width=4)
    label_box(draw, (margin + 40, top_y + 460), "IMU frame I\nnot the relocalization origin", blue, size=20)

    draw.ellipse((camera_origin[0] - 9, camera_origin[1] - 9, camera_origin[0] + 9, camera_origin[1] + 9), fill=green)
    arrow(draw, (margin + 1015, top_y + 565), camera_origin, green, width=4)
    label_box(draw, (margin + 880, top_y + 585), "Camera frame C\nseparate optical frame", green, size=20)

    # structure1.png positions after scaling to 1180 px.
    struct_imu = (margin + 398, bot_y + 357)
    struct_lidar_hint = (margin + 677, bot_y + 350)
    draw.ellipse((struct_imu[0] - 9, struct_imu[1] - 9, struct_imu[0] + 9, struct_imu[1] + 9), fill=blue)
    arrow(draw, (margin + 245, bot_y + 250), struct_imu, blue, width=4)
    label_box(draw, (margin + 45, bot_y + 165), "Wiki marks IMU separately", blue, size=20)

    arrow(draw, (margin + 820, bot_y + 230), struct_lidar_hint, red, width=5)
    label_box(draw, (margin + 830, bot_y + 145), "L is the point-cloud/LiDAR frame\nshown in the coordinate diagram", red, size=20)

    note = (
        "Fixed wiki extrinsic: T^imu_lidar translation = [-0.02663, 0.03447, 0.02174] m, rotation = identity.\n"
        "So L and I axes are parallel, but their origins are offset. Do not use shell center as pose origin."
    )
    note_box = (margin, canvas.height - margin - 78, canvas.width - margin, canvas.height - margin)
    draw.rounded_rectangle(note_box, radius=10, fill=(255, 251, 235), outline=amber, width=2)
    draw.text((margin + 16, canvas.height - margin - 64), note, fill=(120, 53, 15), font=font(19))

    canvas.save(OUT)
    print(OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
