#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class SimpleNavNode(Node):
    def __init__(self) -> None:
        super().__init__("sim2real_simple_nav_node")

        self.map_frame = str(self.declare_parameter("nav_map_frame", "map").value)
        self.base_frame = str(self.declare_parameter("nav_base_frame", "base_link").value)
        self.control_hz = float(self.declare_parameter("nav_control_hz", 20.0).value)
        self.goal_tolerance = float(self.declare_parameter("nav_goal_tolerance", 0.20).value)
        self.yaw_stop_threshold = float(self.declare_parameter("nav_yaw_stop_threshold", 0.80).value)
        self.max_vx = float(self.declare_parameter("nav_max_vx", 0.45).value)
        self.max_wz = float(self.declare_parameter("nav_max_wz", 0.8).value)
        self.kp_dist = float(self.declare_parameter("nav_kp_dist", 0.8).value)
        self.kp_yaw = float(self.declare_parameter("nav_kp_yaw", 1.8).value)
        self.goals_file = str(self.declare_parameter("nav_goals_file", "").value)
        self.missions_file = str(self.declare_parameter("nav_missions_file", "").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_nav", 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "simple_nav/goal_pose", 10)
        self.status_pub = self.create_publisher(String, "simple_nav/status", 10)
        self.record_pub = self.create_publisher(String, "simple_nav/recorded_pose", 10)
        self.create_subscription(String, "simple_nav/cmd", self.on_command, 10)

        self.goals = self._load_goals(self.goals_file)
        self.missions = self._load_missions(self.missions_file)
        self.active_goal_name: Optional[str] = None
        self.active_goal_xy: Optional[tuple[float, float]] = None
        self.active_mission_name: Optional[str] = None
        self.active_mission_goals: list[str] = []
        self.active_mission_index = 0
        self.pose_waiting_reported = False

        period = 1.0 / self.control_hz if self.control_hz > 0.0 else 0.05
        self.timer = self.create_timer(period, self.on_timer)
        self.get_logger().info(
            f"Simple nav started (map_frame={self.map_frame}, base_frame={self.base_frame}, goals={len(self.goals)}, missions={len(self.missions)})"
        )

    def _load_yaml(self, path_value: str) -> dict[str, Any]:
        if not path_value:
            return {}
        path = Path(path_value).expanduser()
        if not path.exists():
            self.get_logger().warn(f"YAML file not found: {path}")
            return {}
        try:
            with path.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            return data if isinstance(data, dict) else {}
        except Exception as exc:
            self.get_logger().error(f"Failed to load YAML {path}: {exc}")
            return {}

    def _load_goals(self, path_value: str) -> dict[str, dict[str, float]]:
        data = self._load_yaml(path_value)
        raw_goals = data.get("goals", {})
        parsed: dict[str, dict[str, float]] = {}
        if not isinstance(raw_goals, dict):
            return parsed
        for name, spec in raw_goals.items():
            if not isinstance(spec, dict):
                continue
            pos = spec.get("position", [0.0, 0.0, 0.0])
            if not isinstance(pos, list) or len(pos) < 2:
                continue
            try:
                parsed[name] = {"x": float(pos[0]), "y": float(pos[1])}
            except (TypeError, ValueError):
                continue
        return parsed

    def _load_missions(self, path_value: str) -> dict[str, list[str]]:
        data = self._load_yaml(path_value)
        raw_missions = data.get("missions", {})
        parsed: dict[str, list[str]] = {}
        if not isinstance(raw_missions, dict):
            return parsed
        for name, spec in raw_missions.items():
            if not isinstance(spec, dict):
                continue
            goals = spec.get("goals", [])
            if isinstance(goals, list) and all(isinstance(item, str) for item in goals) and goals:
                parsed[name] = goals
        return parsed

    def on_command(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            return
        parts = command.split()
        op = parts[0].lower()

        if op == "stop":
            self.cancel_navigation("manual stop")
        elif op == "record" and len(parts) >= 2:
            self.record_current_pose(parts[1])
        elif op == "goto" and len(parts) >= 2:
            self.start_goal(parts[1])
        elif op == "go" and len(parts) >= 3:
            try:
                self.start_direct_goal(float(parts[1]), float(parts[2]), f"direct({parts[1]},{parts[2]})")
            except ValueError:
                self.publish_status("go expects numeric x y")
        elif op == "go_rel" and len(parts) >= 3:
            try:
                self.start_relative_goal(float(parts[1]), float(parts[2]))
            except ValueError:
                self.publish_status("go_rel expects numeric dx dy")
        elif op == "run" and len(parts) >= 2:
            self.start_mission(parts[1])
        elif op == "reload":
            self.goals = self._load_goals(self.goals_file)
            self.missions = self._load_missions(self.missions_file)
            self.publish_status(f"reloaded goals={len(self.goals)} missions={len(self.missions)}")
        else:
            self.publish_status(f"unknown or incomplete command: {command}")

    def start_goal(self, goal_name: str) -> None:
        goal = self.goals.get(goal_name)
        if goal is None:
            self.publish_status(f"goal not found: {goal_name}")
            return
        self.active_mission_name = None
        self.active_mission_goals = []
        self.active_mission_index = 0
        self.start_direct_goal(goal["x"], goal["y"], goal_name)

    def start_direct_goal(self, x: float, y: float, goal_name: str) -> None:
        self.active_goal_name = goal_name
        self.active_goal_xy = (x, y)
        self.pose_waiting_reported = False
        self.publish_goal_pose(x, y)
        self.publish_status(f"nav target set in {self.map_frame}: {goal_name} -> ({x:.2f}, {y:.2f})")

    def start_relative_goal(self, dx: float, dy: float) -> None:
        pose = self.lookup_pose()
        if pose is None:
            self.publish_status("go_rel failed: pose unavailable")
            return
        rx, ry, ryaw = pose
        gx = rx + math.cos(ryaw) * dx - math.sin(ryaw) * dy
        gy = ry + math.sin(ryaw) * dx + math.cos(ryaw) * dy
        self.active_mission_name = None
        self.active_mission_goals = []
        self.active_mission_index = 0
        self.start_direct_goal(gx, gy, f"relative(dx={dx:.2f},dy={dy:.2f})")

    def start_mission(self, mission_name: str) -> None:
        goals = self.missions.get(mission_name)
        if goals is None:
            self.publish_status(f"mission not found: {mission_name}")
            return
        self.active_mission_name = mission_name
        self.active_mission_goals = list(goals)
        self.active_mission_index = 0
        self._activate_mission_goal()

    def _activate_mission_goal(self) -> None:
        if self.active_mission_index >= len(self.active_mission_goals):
            self.cancel_navigation("mission complete")
            return
        goal_name = self.active_mission_goals[self.active_mission_index]
        goal = self.goals.get(goal_name)
        if goal is None:
            self.cancel_navigation(f"mission goal missing: {goal_name}")
            return
        self.active_goal_name = goal_name
        self.active_goal_xy = (goal["x"], goal["y"])
        self.publish_goal_pose(goal["x"], goal["y"])
        self.publish_status(
            f"mission {self.active_mission_name}: waypoint {self.active_mission_index + 1}/{len(self.active_mission_goals)} -> {goal_name}"
        )

    def record_current_pose(self, name: str) -> None:
        pose = self.lookup_pose()
        if pose is None:
            self.publish_status("record failed: pose unavailable")
            return
        x, y, yaw = pose
        text = f"{name}: frame={self.map_frame}, x={x:.3f}, y={y:.3f}, yaw_deg={math.degrees(yaw):.1f}"
        self.record_pub.publish(String(data=text))
        self.publish_status(f"recorded {text}")

    def lookup_pose(self) -> Optional[tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        return float(t.x), float(t.y), float(yaw)

    def on_timer(self) -> None:
        if self.active_goal_xy is None:
            return
        pose = self.lookup_pose()
        if pose is None:
            if not self.pose_waiting_reported:
                self.publish_status(f"pose unavailable: waiting for TF {self.map_frame} -> {self.base_frame}")
                self.pose_waiting_reported = True
            self.cmd_pub.publish(Twist())
            return
        if self.pose_waiting_reported:
            self.publish_status(f"pose available: TF {self.map_frame} -> {self.base_frame} restored")
            self.pose_waiting_reported = False

        rx, ry, ryaw = pose
        gx, gy = self.active_goal_xy
        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)

        if dist < self.goal_tolerance:
            reached_name = self.active_goal_name or "goal"
            self.cmd_pub.publish(Twist())
            if self.active_mission_name is not None:
                self.publish_status(f"reached {reached_name}")
                self.active_mission_index += 1
                self._activate_mission_goal()
            else:
                self.cancel_navigation(f"reached {reached_name}")
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = self.normalize_angle(target_yaw - ryaw)
        cmd = Twist()
        cmd.angular.z = self.clamp(self.kp_yaw * yaw_err, -self.max_wz, self.max_wz)
        if abs(yaw_err) <= self.yaw_stop_threshold:
            cmd.linear.x = self.clamp(self.kp_dist * dist * math.cos(yaw_err), 0.0, self.max_vx)
        self.cmd_pub.publish(cmd)

    def cancel_navigation(self, reason: str) -> None:
        self.active_goal_name = None
        self.active_goal_xy = None
        self.active_mission_name = None
        self.active_mission_goals = []
        self.active_mission_index = 0
        self.cmd_pub.publish(Twist())
        self.publish_status(f"navigation stopped: {reason}")

    def publish_goal_pose(self, x: float, y: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self.goal_pose_pub.publish(msg)

    def publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, float(value)))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimpleNavNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
