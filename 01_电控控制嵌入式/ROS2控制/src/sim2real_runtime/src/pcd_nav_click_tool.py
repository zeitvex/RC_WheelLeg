#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrow
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class PcdNavClickTool(Node):
    def __init__(self) -> None:
        super().__init__("pcd_nav_click_tool")

        self.map_frame = str(self.declare_parameter("nav_map_frame", "map").value)
        self.base_frame = str(self.declare_parameter("nav_base_frame", "base_link").value)
        self.pcd_file = str(self.declare_parameter("pcd_nav_file", "").value)
        self.floor_z_min = float(self.declare_parameter("pcd_floor_z_min", -1.6).value)
        self.floor_z_max = float(self.declare_parameter("pcd_floor_z_max", 0.4).value)
        self.sample_step = max(1, int(self.declare_parameter("pcd_sample_step", 25).value))
        self.robot_radius = float(self.declare_parameter("pcd_robot_radius", 0.18).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(String, "/simple_nav/cmd", 10)

        self.xy_points = self.load_filtered_pcd(Path(self.pcd_file))
        self.current_target: Optional[tuple[float, float]] = None

    def load_filtered_pcd(self, path: Path) -> list[tuple[float, float]]:
        if not path.exists():
            raise FileNotFoundError(f"PCD file not found: {path}")

        points: list[tuple[float, float]] = []
        data_started = False
        with path.open("r", encoding="utf-8") as f:
            for line in f:
                stripped = line.strip()
                if not stripped:
                    continue
                if data_started:
                    parts = stripped.split()
                    if len(parts) < 3:
                        continue
                    try:
                        x = float(parts[0])
                        y = float(parts[1])
                        z = float(parts[2])
                    except ValueError:
                        continue
                    if self.floor_z_min <= z <= self.floor_z_max:
                        points.append((x, y))
                elif stripped.upper().startswith("DATA"):
                    if "ascii" not in stripped.lower():
                        raise RuntimeError("Only ASCII PCD is supported by this simple tool")
                    data_started = True

        if not points:
            raise RuntimeError("No usable floor points after z filtering")
        return points[:: self.sample_step]

    def lookup_pose(self) -> Optional[tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        return float(t.x), float(t.y), float(yaw)

    def publish_command(self, text: str) -> None:
        self.cmd_pub.publish(String(data=text))
        self.get_logger().info(text)

    def on_click(self, event) -> None:
        if event.inaxes is None or event.xdata is None or event.ydata is None:
            return
        x = float(event.xdata)
        y = float(event.ydata)
        self.current_target = (x, y)
        self.publish_command(f"go {x:.3f} {y:.3f}")
        self.refresh_plot()

    def on_key(self, event) -> None:
        if event.key == "r":
            pose = self.lookup_pose()
            if pose is not None:
                x, y, _ = pose
                self.publish_command("record p_click")
                self.current_target = (x, y)
                self.refresh_plot()
        elif event.key == "s":
            self.publish_command("stop")
        elif event.key == "escape":
            plt.close("all")

    def refresh_plot(self) -> None:
        self.ax.clear()
        xs = [p[0] for p in self.xy_points]
        ys = [p[1] for p in self.xy_points]
        self.ax.scatter(xs, ys, s=1, c="black", alpha=0.35)

        pose = self.lookup_pose()
        if pose is not None:
            rx, ry, ryaw = pose
            self.ax.add_patch(Circle((rx, ry), self.robot_radius, color="tab:blue", alpha=0.8))
            self.ax.add_patch(
                FancyArrow(
                    rx,
                    ry,
                    0.35 * math.cos(ryaw),
                    0.35 * math.sin(ryaw),
                    width=0.03,
                    color="tab:blue",
                )
            )
            self.ax.text(rx, ry, f" robot\n({rx:.2f},{ry:.2f})", color="tab:blue")

        if self.current_target is not None:
            gx, gy = self.current_target
            self.ax.scatter([gx], [gy], s=80, c="tab:red", marker="x")
            self.ax.text(gx, gy, f" goal\n({gx:.2f},{gy:.2f})", color="tab:red")

        self.ax.set_title(
            "PCD 2D Click Nav\nLeft click: go absolute | r: record | s: stop | Esc: quit"
        )
        self.ax.set_xlabel("map x")
        self.ax.set_ylabel("map y")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, alpha=0.2)
        self.fig.canvas.draw_idle()

    def run(self) -> None:
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.refresh_plot()

        timer = self.create_timer(0.5, self.refresh_plot)
        try:
            plt.show()
        finally:
            timer.cancel()

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PcdNavClickTool()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
