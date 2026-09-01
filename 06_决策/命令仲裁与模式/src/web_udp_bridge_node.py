#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import socket
import threading
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional
from urllib.parse import urlparse

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sim2real_interfaces.msg import RuntimeState, RuntimeTarget
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class WebUdpBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("sim2real_web_udp_bridge_node", allow_undeclared_parameters=True)

        self.enabled = bool(self.declare_parameter("web_bridge_enabled", True).value)
        self.listen_host = str(self.declare_parameter("web_udp_listen_host", "0.0.0.0").value)
        self.listen_port = int(self.declare_parameter("web_udp_listen_port", 15000).value)
        self.remote_host = str(self.declare_parameter("web_udp_remote_host", "").value)
        self.remote_port = int(self.declare_parameter("web_udp_remote_port", 15001).value)
        self.state_hz = float(self.declare_parameter("web_udp_state_hz", 20.0).value)
        self.cmd_timeout_ms = float(self.declare_parameter("web_udp_cmd_timeout_ms", 300.0).value)
        self.max_packet_bytes = int(self.declare_parameter("web_udp_max_packet_bytes", 8192).value)
        self.max_vx = float(self.declare_parameter("web_udp_max_vx", 0.8).value)
        self.max_vy = float(self.declare_parameter("web_udp_max_vy", 0.3).value)
        self.max_yaw = float(self.declare_parameter("web_udp_max_yaw_rate", 0.5).value)
        self.estop_on_timeout = bool(self.declare_parameter("web_udp_estop_on_timeout", False).value)

        self.http_host = str(self.declare_parameter("web_http_host", "0.0.0.0").value)
        self.http_port = int(self.declare_parameter("web_http_port", 18080).value)
        self.web_static_dir = str(self.declare_parameter("web_static_dir", "").value)
        raw_localization_mode = str(self.declare_parameter("localization_mode", "odom").value)
        self.localization_mode = self.normalize_localization_mode(raw_localization_mode)
        self.odom_fallback_allowed = self.localization_mode == "odom"
        self.nav_map_frame = str(self.declare_parameter("nav_map_frame", "map").value)
        self.nav_odom_frame = str(self.declare_parameter("nav_odom_frame", "odom").value)
        self.nav_base_frame = str(self.declare_parameter("nav_base_frame", "base_link").value)
        self.nav_goals_file = str(self.declare_parameter("nav_goals_file", "").value)
        self.nav_missions_file = str(self.declare_parameter("nav_missions_file", "").value)
        self.nav_route_file = str(self.declare_parameter("nav_route_file", "").value)
        self.nav_route_task_file = str(self.declare_parameter("nav_route_task_file", "").value)
        self.pcd_nav_file = str(self.declare_parameter("pcd_nav_file", "").value)
        self.pcd_floor_z_min = float(self.declare_parameter("pcd_floor_z_min", -1.6).value)
        self.pcd_floor_z_max = float(self.declare_parameter("pcd_floor_z_max", 0.4).value)
        self.pcd_sample_step = max(1, int(self.declare_parameter("pcd_sample_step", 25).value))
        self.route_align_enabled = bool(
            self.declare_parameter("nav_route_auto_align_enabled", True).value
        )
        self.route_rotation_offset_deg = float(
            self.declare_parameter("nav_route_rotation_offset_deg", 0.0).value
        )
        self.route_align_max_angle_deg = abs(
            float(self.declare_parameter("nav_route_align_max_angle_deg", 6.0).value)
        )
        self.route_align_angle_step_deg = max(
            0.1,
            float(self.declare_parameter("nav_route_align_angle_step_deg", 0.5).value),
        )
        self.route_align_search_radius = max(
            0.05,
            float(self.declare_parameter("nav_route_align_search_radius", 0.35).value),
        )
        self.odom_fallback_require_odom_fresh = bool(
            self.declare_parameter("odom_fallback_require_odom_fresh", True).value
        )
        self.odom_fallback_max_odom_age_ms = max(
            0.0,
            float(self.declare_parameter("odom_fallback_max_odom_age_ms", 500.0).value),
        )
        self.odom_fallback_block_existing_map_odom_tf = bool(
            self.declare_parameter("odom_fallback_block_existing_map_odom_tf", True).value
        )
        self.odom_fallback_tf_conflict_window_s = max(
            0.0,
            float(self.declare_parameter("odom_fallback_tf_conflict_window_s", 1.0).value),
        )
        self.odom_fallback_tf_conflict_xy_tolerance = max(
            0.0,
            float(self.declare_parameter("odom_fallback_tf_conflict_xy_tolerance", 0.05).value),
        )
        self.odom_fallback_tf_conflict_yaw_tolerance = math.radians(
            max(0.0, float(self.declare_parameter("odom_fallback_tf_conflict_yaw_tolerance_deg", 2.0).value))
        )
        self.odom_fallback_stop_on_external_tf = bool(
            self.declare_parameter("odom_fallback_stop_on_external_tf", False).value
        )
        default_odom_trace_dir = Path(__file__).resolve().parents[3] / "map" / "load"
        self.odom_trace_export_dir = str(
            self.declare_parameter("odom_trace_export_dir", str(default_odom_trace_dir)).value
        )
        self.odom_trace_sample_hz = max(
            0.2,
            float(self.declare_parameter("odom_trace_sample_hz", 5.0).value),
        )
        self.odom_trace_min_distance = max(
            0.0,
            float(self.declare_parameter("odom_trace_min_distance", 0.03).value),
        )
        self.odom_trace_max_points = max(
            2,
            int(self.declare_parameter("odom_trace_max_points", 20000).value),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_fallback_active = False
        self.odom_fallback_transform: Optional[TransformStamped] = None
        self.odom_fallback_anchor: dict[str, Any] = {}
        self.odom_fallback_handoff_pending = False
        self.odom_fallback_handoff_info: dict[str, Any] = {}
        self.odom_trace_active = False
        self.odom_trace_points: list[dict[str, Any]] = []
        self.odom_trace_started_at = 0.0
        self.odom_trace_mission_name = ""
        self.odom_trace_anchor: dict[str, Any] = {}
        self.odom_trace_last_pose: Optional[dict[str, Any]] = None
        self.odom_trace_export_path = ""
        self.last_external_map_odom_tf: Optional[dict[str, Any]] = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((self.listen_host, self.listen_port))

        self.client_addr: Optional[tuple[str, int]] = None
        if self.remote_host:
            self.client_addr = (self.remote_host, self.remote_port)

        self.latest_target: Optional[RuntimeTarget] = None
        self.latest_state: Optional[RuntimeState] = None
        self.latest_model_status: dict[str, Any] = {
            "current_model": "rough",
            "requested_model": "rough",
            "switch_state": "idle",
            "backend": "unknown",
            "switching": False,
        }
        self.latest_cmd = Twist()
        self.latest_mode = "UNKNOWN"
        self.latest_mux_status = ""
        self.latest_nav_status = ""
        self.latest_nav_path: dict[str, Any] = {
            "goal_name": "",
            "stage": "idle",
            "path_index": 0,
            "points": [],
        }
        self.estop = False
        self.web_enabled = False
        self.last_cmd_time = self.get_clock().now()
        self.timeout_estop_sent = False

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_web", 10)
        self.nav_cmd_pub = self.create_publisher(String, "/simple_nav/cmd", 10)
        self.estop_pub = self.create_publisher(Bool, "/safety/estop", 10)
        self.web_enabled_pub = self.create_publisher(Bool, "web/enabled", 10)
        self.remote_enabled_pub = self.create_publisher(Bool, "remote/enabled", 10)
        self.nav_enabled_pub = self.create_publisher(Bool, "nav/enabled", 10)
        self.mode_pub = self.create_publisher(String, "control/mode", 10)
        self.model_cmd_pub = self.create_publisher(String, "runtime/model_cmd", 10)
        self.posture_cmd_pub = self.create_publisher(String, "runtime/posture_cmd", 10)

        self.create_subscription(RuntimeTarget, "runtime/target", self.on_target, 10)
        self.create_subscription(RuntimeState, "runtime/state", self.on_state, 10)
        self.create_subscription(String, "runtime/model_status", self.on_model_status, 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Bool, "/safety/estop", self.on_estop, 10)
        self.create_subscription(String, "control/mode_state", self.on_mode_state, 10)
        self.create_subscription(String, "control/mux_status", self.on_mux_status, 10)
        self.create_subscription(String, "simple_nav/status", self.on_nav_status, 10)
        self.create_subscription(String, "simple_nav/path", self.on_nav_path, 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 50)

        self.map_points = self.load_filtered_pcd(Path(self.pcd_nav_file)) if self.pcd_nav_file else []
        self.goal_specs: list[dict[str, Any]] = []
        self.mission_specs: list[dict[str, Any]] = []
        self.default_mission_name: Optional[str] = None
        self.task_specs: list[dict[str, Any]] = []
        self.route_alignment_info: dict[str, Any] = {}
        self.route_avoid_regions: list[dict[str, Any]] = []
        self.reload_nav_task_config()

        self.http_server: Optional[ThreadingHTTPServer] = None
        self.http_thread: Optional[threading.Thread] = None
        self.static_dir = (
            Path(self.web_static_dir)
            if self.web_static_dir
            else Path(__file__).resolve().parents[3] / "tools" / "win_web_debug" / "static"
        )
        self.start_http_server()

        self.rx_timer = self.create_timer(0.01, self.on_rx_timer)
        self.state_timer = self.create_timer(
            1.0 / self.state_hz if self.state_hz > 0.0 else 0.05,
            self.on_state_timer,
        )
        self.guard_timer = self.create_timer(0.05, self.on_guard_timer)
        self.odom_fallback_timer = self.create_timer(0.05, self.on_odom_fallback_timer)
        self.odom_trace_timer = self.create_timer(
            1.0 / self.odom_trace_sample_hz,
            self.on_odom_trace_timer,
        )

        self.get_logger().info(f"Web UDP bridge listening on {self.listen_host}:{self.listen_port}")
        self.get_logger().info(f"Web HTTP UI serving on http://{self.http_host}:{self.http_port}")
        self.get_logger().info(
            f"Localization mode: {self.localization_mode} "
            f"(odom_fallback_allowed={self.odom_fallback_allowed})"
        )

    def load_filtered_pcd(self, path: Path) -> list[list[float]]:
        if not path.exists():
            self.get_logger().warn(f"PCD file not found: {path}")
            return []

        points: list[list[float]] = []
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
                    if self.pcd_floor_z_min <= z <= self.pcd_floor_z_max:
                        points.append([round(x, 3), round(y, 3)])
                elif stripped.upper().startswith("DATA"):
                    data_started = True
        return points[:: self.pcd_sample_step]

    def _load_yaml(self, path_value: str) -> dict[str, Any]:
        if not path_value:
            return {}
        path = Path(path_value).expanduser()
        if not path.exists():
            self.get_logger().warn(f"Navigation file not found: {path}")
            return {}
        try:
            with path.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            return data if isinstance(data, dict) else {}
        except Exception as exc:
            self.get_logger().warn(f"Failed to load navigation file {path}: {exc}")
            return {}

    @staticmethod
    def _normalize_policy(value: Any) -> Optional[str]:
        text = str(value).strip().lower()
        if not text:
            return None
        if text == "ik":
            return "crawl"
        if text in {"rough", "crawl", "wall"}:
            return text
        return None

    @staticmethod
    def _normalize_task(value: Any) -> Optional[str]:
        text = str(value).strip().lower()
        return text or None

    @staticmethod
    def normalize_localization_mode(value: Any) -> str:
        text = str(value or "").strip().lower()
        if text in {"relocal", "reloc", "relocalization", "localization"}:
            return "relocal"
        return "odom"

    @staticmethod
    def _get_value(data: dict[str, Any], *keys: str) -> Any:
        for key in keys:
            if key in data and data.get(key) is not None:
                return data.get(key)
        return None

    @classmethod
    def _get_float(cls, data: dict[str, Any], *keys: str) -> Optional[float]:
        value = cls._get_value(data, *keys)
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @classmethod
    def _get_bool(cls, data: dict[str, Any], *keys: str) -> Optional[bool]:
        value = cls._get_value(data, *keys)
        if value is None:
            return None
        if isinstance(value, bool):
            return value
        text = str(value).strip().lower()
        if text in {"1", "true", "yes", "on"}:
            return True
        if text in {"0", "false", "no", "off"}:
            return False
        return None

    @classmethod
    def _get_int(cls, data: dict[str, Any], *keys: str) -> Optional[int]:
        value = cls._get_value(data, *keys)
        if value is None:
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def load_nav_task_config(
        self,
    ) -> tuple[
        list[dict[str, Any]],
        list[dict[str, Any]],
        Optional[str],
        dict[str, Any],
        list[dict[str, Any]],
        list[dict[str, Any]],
    ]:
        goals_by_name = self._load_goals(self.nav_goals_file)
        missions_by_name = self._load_missions(self.nav_missions_file)
        route_goals, route_missions, route_default, route_alignment, route_regions = self._load_route(
            self.get_route_source_file()
        )
        goals_by_name.update(route_goals)
        missions_by_name.update(route_missions)
        goal_specs = list(goals_by_name.values())
        mission_specs = [
            {"name": mission_name, "goals": list(goal_names)}
            for mission_name, goal_names in missions_by_name.items()
        ]
        default_mission_name = route_default or (mission_specs[0]["name"] if mission_specs else None)
        task_specs = self.build_task_specs(goal_specs, mission_specs, default_mission_name)
        return goal_specs, mission_specs, default_mission_name, route_alignment, route_regions, task_specs

    def reload_nav_task_config(self) -> None:
        (
            self.goal_specs,
            self.mission_specs,
            self.default_mission_name,
            self.route_alignment_info,
            self.route_avoid_regions,
            self.task_specs,
        ) = self.load_nav_task_config()

    def build_task_specs(
        self,
        goal_specs: list[dict[str, Any]],
        mission_specs: list[dict[str, Any]],
        default_mission_name: Optional[str],
    ) -> list[dict[str, Any]]:
        if not default_mission_name:
            return []
        goal_lookup = {str(goal.get("name", "")): goal for goal in goal_specs if goal.get("name")}
        default_mission = next(
            (mission for mission in mission_specs if mission.get("name") == default_mission_name),
            None,
        )
        if not default_mission:
            return []

        tasks: list[dict[str, Any]] = []
        task_by_name: dict[str, dict[str, Any]] = {}
        for index, goal_name in enumerate(default_mission.get("goals", []), start=1):
            goal = goal_lookup.get(str(goal_name))
            if not goal:
                continue
            task_name = self._normalize_task(goal.get("task"))
            if not task_name:
                continue
            spec = task_by_name.get(task_name)
            if spec is None:
                spec = {
                    "name": task_name,
                    "start_index": index,
                    "end_index": index,
                    "count": 0,
                    "first_goal": str(goal_name),
                    "last_goal": str(goal_name),
                    "first_id": goal.get("id"),
                    "last_id": goal.get("id"),
                    "goals": [],
                    "policies": [],
                }
                task_by_name[task_name] = spec
                tasks.append(spec)
            spec["end_index"] = index
            spec["count"] = int(spec["count"]) + 1
            spec["last_goal"] = str(goal_name)
            spec["last_id"] = goal.get("id")
            spec["goals"].append(str(goal_name))
            policy = goal.get("policy")
            if policy and policy not in spec["policies"]:
                spec["policies"].append(policy)
        return tasks

    def get_route_source_file(self) -> str:
        candidate = self.nav_route_task_file.strip() if self.nav_route_task_file else ""
        if candidate:
            return candidate
        return self.nav_route_file

    def _load_goals(self, path_value: str) -> dict[str, dict[str, Any]]:
        data = self._load_yaml(path_value)
        raw_goals = data.get("goals", {})
        parsed: dict[str, dict[str, Any]] = {}
        if not isinstance(raw_goals, dict):
            return parsed
        for name, spec in raw_goals.items():
            if not isinstance(spec, dict):
                continue
            pos = spec.get("position", [0.0, 0.0, 0.0])
            if not isinstance(pos, list) or len(pos) < 2:
                continue
            try:
                yaw_tolerance_deg = self._get_float(spec, "yaw_tolerance_deg", "yawToleranceDeg")
                yaw_deg = self._get_float(spec, "yaw_deg", "yawDeg")
                require_yaw = self._get_bool(spec, "require_yaw", "requireYaw")
                parsed[str(name)] = {
                    "name": str(name),
                    "id": self._get_int(spec, "id", "waypoint_id", "waypointId"),
                    "task": self._normalize_task(spec.get("task")),
                    "segment": self._normalize_task(spec.get("segment")),
                    "x": float(pos[0]),
                    "y": float(pos[1]),
                    "yaw_deg": yaw_deg,
                    "yaw_tolerance_deg": yaw_tolerance_deg,
                    "tolerance": float(spec.get("tolerance", 0.20)),
                    "policy": self._normalize_policy(spec.get("policy")),
                    "speed": float(spec["speed"]) if spec.get("speed") is not None else None,
                    "require_yaw": bool(yaw_deg is not None if require_yaw is None else require_yaw) and yaw_deg is not None,
                    "pre_dock_distance": self._get_float(spec, "pre_dock_distance", "preDockDistance"),
                    "pre_dock_tolerance": self._get_float(spec, "pre_dock_tolerance", "preDockTolerance"),
                }
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
            if isinstance(goals, list) and goals and all(isinstance(item, str) for item in goals):
                parsed[str(name)] = [str(item) for item in goals]
        return parsed

    def _load_route(
        self, path_value: str
    ) -> tuple[dict[str, dict[str, Any]], dict[str, list[str]], Optional[str], dict[str, Any], list[dict[str, Any]]]:
        data = self._load_yaml(path_value)
        if not data:
            return {}, {}, None, {}, []

        route_name = str(data.get("name", "")).strip() or Path(path_value).stem
        route_default_yaw_tolerance_deg = self._get_float(
            data,
            "yaw_tolerance_deg_default",
            "yawToleranceDegDefault",
        )
        route_default_require_yaw = self._get_bool(
            data,
            "require_yaw_default",
            "requireYawDefault",
        )
        route_default_pre_dock_distance = self._get_float(
            data,
            "pre_dock_distance_default",
            "preDockDistanceDefault",
        )
        route_default_pre_dock_tolerance = self._get_float(
            data,
            "pre_dock_tolerance_default",
            "preDockToleranceDefault",
        )
        raw_segments = data.get("segments", [])
        if not isinstance(raw_segments, list) or not raw_segments:
            top_level_waypoints = data.get("waypoints", [])
            if isinstance(top_level_waypoints, list) and top_level_waypoints:
                raw_segments = [{
                    "name": "segment_1",
                    "obstacle": str(data.get("obstacle", "flat") or "flat"),
                    "waypoints": top_level_waypoints,
                }]
            else:
                return {}, {}, None, {}, []

        raw_waypoints: list[dict[str, Any]] = []
        for segment_index, segment in enumerate(raw_segments, start=1):
            if not isinstance(segment, dict):
                continue
            segment_name = str(segment.get("name", f"segment_{segment_index}")).strip() or f"segment_{segment_index}"
            waypoints = segment.get("waypoints", [])
            if not isinstance(waypoints, list):
                continue
            for waypoint_index, waypoint in enumerate(waypoints, start=1):
                if not isinstance(waypoint, dict):
                    continue
                try:
                    waypoint_yaw_deg = self._get_float(waypoint, "yaw_deg", "yawDeg")
                    waypoint_yaw_tolerance_deg = self._get_float(
                        waypoint,
                        "yaw_tolerance_deg",
                        "yawToleranceDeg",
                    )
                    waypoint_require_yaw = self._get_bool(
                        waypoint,
                        "require_yaw",
                        "requireYaw",
                    )
                    if waypoint_require_yaw is None:
                        waypoint_require_yaw = bool(route_default_require_yaw) if route_default_require_yaw is not None else False
                    if waypoint_yaw_tolerance_deg is None:
                        waypoint_yaw_tolerance_deg = route_default_yaw_tolerance_deg
                    waypoint_pre_dock_distance = self._get_float(
                        waypoint,
                        "pre_dock_distance",
                        "preDockDistance",
                    )
                    if waypoint_pre_dock_distance is None:
                        waypoint_pre_dock_distance = route_default_pre_dock_distance
                    waypoint_pre_dock_tolerance = self._get_float(
                        waypoint,
                        "pre_dock_tolerance",
                        "preDockTolerance",
                    )
                    if waypoint_pre_dock_tolerance is None:
                        waypoint_pre_dock_tolerance = route_default_pre_dock_tolerance
                    raw_waypoints.append(
                        {
                            "segment": segment_name,
                            "id": int(waypoint.get("id", waypoint_index)),
                            "task": self._normalize_task(waypoint.get("task", segment.get("obstacle", ""))),
                            "x": float(waypoint["x"]),
                            "y": float(waypoint["y"]),
                            "yaw_deg": waypoint_yaw_deg,
                            "yaw_tolerance_deg": waypoint_yaw_tolerance_deg,
                            "tolerance": float(waypoint.get("tolerance", 0.20)),
                            "policy": self._normalize_policy(waypoint.get("policy")),
                            "speed": float(waypoint["speed"]) if waypoint.get("speed") is not None else None,
                            "require_yaw": bool(waypoint_require_yaw) and waypoint_yaw_deg is not None,
                            "pre_dock_distance": waypoint_pre_dock_distance,
                            "pre_dock_tolerance": waypoint_pre_dock_tolerance,
                        }
                    )
                except (KeyError, TypeError, ValueError):
                    continue

        if not raw_waypoints:
            return {}, {}, None, {}, []

        aligned_waypoints, alignment_info = self._align_route_waypoints(raw_waypoints)
        applied_angle_rad = math.radians(float(alignment_info.get("applied_deg", 0.0))) if alignment_info else 0.0
        aligned_regions = self._rotate_avoid_regions(
            self._load_avoid_regions(data),
            float(raw_waypoints[0]["x"]),
            float(raw_waypoints[0]["y"]),
            applied_angle_rad,
        )
        goals: dict[str, dict[str, Any]] = {}
        mission_goal_names: list[str] = []
        for index, waypoint in enumerate(aligned_waypoints, start=1):
            goal_name = f"{route_name}_p{index:02d}"
            goals[goal_name] = {
                "name": goal_name,
                "id": int(waypoint["id"]) if waypoint.get("id") is not None else None,
                "task": self._normalize_task(waypoint.get("task")),
                "segment": str(waypoint.get("segment", "") or "") or None,
                "x": round(float(waypoint["x"]), 3),
                "y": round(float(waypoint["y"]), 3),
                "yaw_deg": round(float(waypoint["yaw_deg"]), 3) if waypoint.get("yaw_deg") is not None else None,
                "yaw_tolerance_deg": round(float(waypoint["yaw_tolerance_deg"]), 3)
                if waypoint.get("yaw_tolerance_deg") is not None else None,
                "tolerance": round(float(waypoint.get("tolerance", 0.20)), 3),
                "policy": self._normalize_policy(waypoint.get("policy")),
                "speed": float(waypoint["speed"]) if waypoint.get("speed") is not None else None,
                "require_yaw": bool(waypoint.get("require_yaw", False)),
                "pre_dock_distance": round(float(waypoint["pre_dock_distance"]), 3)
                if waypoint.get("pre_dock_distance") is not None else None,
                "pre_dock_tolerance": round(float(waypoint["pre_dock_tolerance"]), 3)
                if waypoint.get("pre_dock_tolerance") is not None else None,
            }
            mission_goal_names.append(goal_name)

        return goals, {route_name: mission_goal_names}, route_name, alignment_info, aligned_regions

    def _load_avoid_regions(self, data: dict[str, Any]) -> list[dict[str, Any]]:
        rows = data.get("regions")
        if not isinstance(rows, list) or not rows:
            rows = data.get("avoid_regions", [])
        if not isinstance(rows, list):
            return []

        regions: list[dict[str, Any]] = []
        for index, row in enumerate(rows, start=1):
            if not isinstance(row, dict):
                continue
            kind = str(row.get("kind", "avoid")).strip().lower()
            if kind not in {"avoid", "no_go", "no-go", "nogo", "forbidden", "blocked"}:
                continue
            polygon_rows = row.get("polygon", [])
            if not isinstance(polygon_rows, list):
                continue
            polygon: list[dict[str, float]] = []
            for point in polygon_rows:
                try:
                    if isinstance(point, dict):
                        polygon.append({"x": float(point["x"]), "y": float(point["y"])})
                    elif isinstance(point, (list, tuple)) and len(point) >= 2:
                        polygon.append({"x": float(point[0]), "y": float(point[1])})
                except (KeyError, TypeError, ValueError):
                    continue
            if len(polygon) >= 3:
                regions.append(
                    {
                        "name": str(row.get("name", f"avoid_{index}")),
                        "kind": kind,
                        "polygon": polygon,
                    }
                )
        return regions

    def _rotate_avoid_regions(
        self,
        regions: list[dict[str, Any]],
        anchor_x: float,
        anchor_y: float,
        angle_rad: float,
    ) -> list[dict[str, Any]]:
        if not regions or abs(angle_rad) <= 1.0e-12:
            return regions
        rotated: list[dict[str, Any]] = []
        for region in regions:
            polygon = []
            for point in region.get("polygon", []):
                x, y = self._rotate_xy(float(point["x"]), float(point["y"]), anchor_x, anchor_y, angle_rad)
                polygon.append({"x": x, "y": y})
            rotated.append({**region, "polygon": polygon})
        return rotated

    def _align_route_waypoints(
        self, waypoints: list[dict[str, Any]]
    ) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        if not waypoints:
            return [], {}

        if not self.map_points:
            info = {
                "enabled": False,
                "reason": "pcd unavailable",
                "manual_offset_deg": round(self.route_rotation_offset_deg, 3),
                "auto_offset_deg": 0.0,
                "applied_deg": round(self.route_rotation_offset_deg, 3),
                "hits": 0,
                "total": len(waypoints),
            }
            return self._rotate_waypoints(waypoints, math.radians(self.route_rotation_offset_deg)), info

        manual_offset_deg = self.route_rotation_offset_deg
        if not self.route_align_enabled or len(waypoints) < 2:
            info = {
                "enabled": False,
                "reason": "auto align disabled",
                "manual_offset_deg": round(manual_offset_deg, 3),
                "auto_offset_deg": 0.0,
                "applied_deg": round(manual_offset_deg, 3),
                "hits": 0,
                "total": len(waypoints),
            }
            return self._rotate_waypoints(waypoints, math.radians(manual_offset_deg)), info

        anchor_x = float(waypoints[0]["x"])
        anchor_y = float(waypoints[0]["y"])
        search_radius_sq = self.route_align_search_radius * self.route_align_search_radius
        best_hits = -1
        best_score = float("inf")
        best_angle_deg = manual_offset_deg
        manual_hits = -1
        manual_score = float("inf")
        steps = max(
            1,
            int(round((self.route_align_max_angle_deg * 2.0) / self.route_align_angle_step_deg)),
        )

        for step_index in range(steps + 1):
            auto_delta_deg = -self.route_align_max_angle_deg + step_index * self.route_align_angle_step_deg
            angle_deg = manual_offset_deg + auto_delta_deg
            angle_rad = math.radians(angle_deg)
            hits = 0
            score = 0.0
            for waypoint in waypoints:
                tx, ty = self._rotate_xy(
                    float(waypoint["x"]),
                    float(waypoint["y"]),
                    anchor_x,
                    anchor_y,
                    angle_rad,
                )
                nearest_sq = search_radius_sq
                for px, py in self.map_points:
                    dx = px - tx
                    dy = py - ty
                    dist_sq = dx * dx + dy * dy
                    if dist_sq < nearest_sq:
                        nearest_sq = dist_sq
                if nearest_sq < search_radius_sq:
                    hits += 1
                score += nearest_sq
            if abs(angle_deg - manual_offset_deg) <= 1.0e-9:
                manual_hits = hits
                manual_score = score
            better_hits = hits > best_hits
            better_score = hits == best_hits and (
                score < best_score - 1.0e-9
                or (
                    abs(score - best_score) <= 1.0e-9
                    and abs(auto_delta_deg) < abs(best_angle_deg - manual_offset_deg)
                )
            )
            if better_hits or better_score:
                best_hits = hits
                best_score = score
                best_angle_deg = angle_deg

        hits_improved = best_hits > manual_hits
        score_improvement = (
            (manual_score - best_score) / max(manual_score, 1.0e-9)
            if manual_score < float("inf")
            else 0.0
        )
        if not hits_improved and score_improvement < 0.05:
            applied_angle_deg = manual_offset_deg
            rotated = self._rotate_waypoints(waypoints, math.radians(applied_angle_deg))
            info = {
                "enabled": True,
                "reason": "ambiguous-auto-align",
                "manual_offset_deg": round(manual_offset_deg, 3),
                "auto_offset_deg": 0.0,
                "applied_deg": round(applied_angle_deg, 3),
                "score": round(manual_score, 6) if manual_score < float("inf") else 0.0,
                "hits": int(manual_hits if manual_hits >= 0 else 0),
                "total": len(waypoints),
            }
            return rotated, info

        rotated = self._rotate_waypoints(waypoints, math.radians(best_angle_deg))
        info = {
            "enabled": True,
            "manual_offset_deg": round(manual_offset_deg, 3),
            "auto_offset_deg": round(best_angle_deg - manual_offset_deg, 3),
            "applied_deg": round(best_angle_deg, 3),
            "score": round(best_score, 6),
            "hits": int(best_hits),
            "total": len(waypoints),
        }
        return rotated, info

    def _rotate_waypoints(self, waypoints: list[dict[str, Any]], angle_rad: float) -> list[dict[str, Any]]:
        if not waypoints:
            return []
        anchor_x = float(waypoints[0]["x"])
        anchor_y = float(waypoints[0]["y"])
        rotated: list[dict[str, Any]] = []
        for waypoint in waypoints:
            x, y = self._rotate_xy(
                float(waypoint["x"]),
                float(waypoint["y"]),
                anchor_x,
                anchor_y,
                angle_rad,
            )
            updated = dict(waypoint)
            updated["x"] = x
            updated["y"] = y
            if updated.get("yaw_deg") is not None:
                updated["yaw_deg"] = round(
                    math.degrees(
                        self.normalize_angle(math.radians(float(updated["yaw_deg"])) + angle_rad)
                    ),
                    3,
                )
            rotated.append(updated)
        return rotated

    @staticmethod
    def _rotate_xy(x: float, y: float, anchor_x: float, anchor_y: float, angle_rad: float) -> tuple[float, float]:
        dx = x - anchor_x
        dy = y - anchor_y
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        return (
            anchor_x + dx * cos_a - dy * sin_a,
            anchor_y + dx * sin_a + dy * cos_a,
        )

    def lookup_pose(self) -> Optional[dict[str, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.nav_map_frame,
                self.nav_base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        return {"x": round(float(t.x), 3), "y": round(float(t.y), 3), "yaw": round(float(yaw), 6)}

    def start_http_server(self) -> None:
        node = self
        static_dir = self.static_dir

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                request_path = urlparse(self.path).path
                if request_path in ["/", "/index.html"]:
                    self.serve_file(static_dir / "index.html", "text/html; charset=utf-8")
                elif request_path == "/app.js":
                    self.serve_file(static_dir / "app.js", "application/javascript; charset=utf-8")
                elif request_path == "/style.css":
                    self.serve_file(static_dir / "style.css", "text/css; charset=utf-8")
                elif request_path == "/api/state":
                    self.send_json(node.build_state_packet())
                elif request_path == "/api/map":
                    self.send_json(node.build_map_packet())
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self):
                if self.path != "/api/control":
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    raw = self.rfile.read(length)
                    payload = json.loads(raw.decode("utf-8"))
                    node.handle_http_control(payload)
                    self.send_json({"ok": True})
                except Exception as exc:
                    self.send_json(
                        {"ok": False, "error": str(exc)},
                        status=HTTPStatus.BAD_REQUEST,
                    )

            def serve_file(self, path: Path, content_type: str):
                if not path.exists():
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                data = path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK):
                data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, format: str, *args):
                return

        self.http_server = ThreadingHTTPServer((self.http_host, self.http_port), Handler)
        self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()

    def build_odom_trace_packet(self) -> dict[str, Any]:
        points = [[self._f(point["x"]), self._f(point["y"])] for point in self.odom_trace_points]
        max_display_points = 240
        if len(points) > max_display_points:
            stride = max(1, math.ceil(len(points) / max_display_points))
            points = points[::stride]
        return {
            "active": self.odom_trace_active,
            "count": len(self.odom_trace_points),
            "display_count": len(points),
            "points": points,
            "mission": self.odom_trace_mission_name,
            "export_path": self.odom_trace_export_path,
            "sample_hz": self.odom_trace_sample_hz,
            "min_distance": self.odom_trace_min_distance,
        }
    def build_map_packet(self) -> dict[str, Any]:
        default_goals = self.get_default_mission_goals()
        return {
            "points": self.map_points,
            "map_frame": self.nav_map_frame,
            "base_frame": self.nav_base_frame,
            "odom_frame": self.nav_odom_frame,
            "pose": self.lookup_pose(),
            "route_source_file": self.get_route_source_file(),
            "goal_specs": self.goal_specs,
            "mission_specs": self.mission_specs,
            "default_mission_name": self.default_mission_name,
            "default_mission_goals": default_goals,
            "task_specs": self.task_specs,
            "route_alignment": self.route_alignment_info,
            "avoid_regions": self.route_avoid_regions,
            "localization_mode": self.localization_mode,
            "odom_fallback_allowed": self.odom_fallback_allowed,
            "odom_fallback": {
                "active": self.odom_fallback_active,
                "handoff_pending": self.odom_fallback_handoff_pending,
                "handoff": dict(self.odom_fallback_handoff_info),
                **self.odom_fallback_anchor,
            },
            "nav_path": dict(self.latest_nav_path),
            "odom_trace": self.build_odom_trace_packet(),
        }

    def handle_http_control(self, payload: dict[str, Any]) -> None:
        msg_type = str(payload.get("type", "")).lower()
        if msg_type in {
            "cmd_vel",
            "zero",
            "estop",
            "mode",
            "web_enable",
            "remote_enable",
            "nav_enable",
            "ping",
            "nav_cmd",
            "nav_stop_keep",
            "task_resume",
            "task_only",
            "task_skip",
            "go_to",
            "go_rel",
            "odom_task",
            "odom_stop",
            "estop_reset",
            "model_toggle",
            "model_cmd",
        }:
            self.handle_packet(payload)
            return
        raise ValueError(f"unknown control type: {msg_type}")

    def on_target(self, msg: RuntimeTarget) -> None:
        self.latest_target = msg

    def on_state(self, msg: RuntimeState) -> None:
        self.latest_state = msg

    def on_model_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data) if msg.data else {}
            if isinstance(data, dict):
                self.latest_model_status = {
                    "current_model": str(data.get("current_model", "rough")),
                    "requested_model": str(
                        data.get("requested_model", data.get("current_model", "rough"))
                    ),
                    "switch_state": str(data.get("switch_state", "idle")),
                    "backend": str(data.get("backend", "unknown")),
                    "switching": bool(data.get("switching", False)),
                }
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse model status: {exc}")

    def on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = msg

    def on_estop(self, msg: Bool) -> None:
        was_estop = self.estop
        self.estop = bool(msg.data)
        if self.estop and not was_estop:
            self.stop_navigation_for_keep("estop requested", enter_keep=False)

    def on_mode_state(self, msg: String) -> None:
        self.latest_mode = msg.data

    def on_mux_status(self, msg: String) -> None:
        self.latest_mux_status = msg.data

    def on_nav_status(self, msg: String) -> None:
        self.latest_nav_status = msg.data
        if msg.data.startswith("reloaded goals="):
            self.reload_nav_task_config()

    def on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            parent = transform.header.frame_id.strip("/")
            child = transform.child_frame_id.strip("/")
            map_frame = self.nav_map_frame.strip("/")
            odom_frame = self.nav_odom_frame.strip("/")
            if parent == map_frame and child == odom_frame:
                self.handle_map_odom_tf(transform, "map_to_odom")
            elif parent == odom_frame and child == map_frame:
                self.handle_map_odom_tf(transform, "odom_to_map")

    def handle_map_odom_tf(self, transform: TransformStamped, direction: str) -> None:
        if not self.odom_fallback_active:
            self.last_external_map_odom_tf = {
                "direction": direction,
                "stamp": self.stamp_to_sec(transform.header.stamp),
                "received_at": round(self.now_sec(), 3),
            }
            return

        if direction == "odom_to_map":
            self.on_external_tf_during_odom_fallback(
                direction,
                "external odom->map TF detected, likely Odin relocalization recovered",
            )
            return

        if not self.is_own_odom_fallback_tf(transform):
            self.on_external_tf_during_odom_fallback(
                direction,
                "external map->odom TF conflict detected",
            )

    def on_external_tf_during_odom_fallback(self, direction: str, reason: str) -> None:
        if self.odom_fallback_stop_on_external_tf:
            self.stop_odom_fallback(reason, stop_nav=True)
            return
        self.mark_odom_fallback_handoff_pending(direction, reason)

    def mark_odom_fallback_handoff_pending(self, direction: str, reason: str) -> None:
        first_notice = not self.odom_fallback_handoff_pending
        self.odom_fallback_handoff_pending = True
        self.odom_fallback_handoff_info = {
            "pending": True,
            "direction": direction,
            "reason": reason,
            "stamp": round(self.now_sec(), 3),
        }
        if first_notice:
            self.latest_nav_status = f"odom fallback handoff pending: {reason}"
            self.get_logger().warn(
                f"{self.latest_nav_status}; keeping pure odom task active until mission ends or Exit odom"
            )

    def on_nav_path(self, msg: String) -> None:
        try:
            data = json.loads(msg.data) if msg.data else {}
        except Exception:
            return
        if not isinstance(data, dict):
            return
        raw_points = data.get("points", [])
        points: list[list[float]] = []
        if isinstance(raw_points, list):
            for item in raw_points:
                if (
                    isinstance(item, (list, tuple))
                    and len(item) >= 2
                ):
                    try:
                        points.append([round(float(item[0]), 3), round(float(item[1]), 3)])
                    except (TypeError, ValueError):
                        continue
        try:
            path_index = max(0, int(data.get("path_index", 0) or 0))
        except (TypeError, ValueError):
            path_index = 0
        self.latest_nav_path = {
            "goal_name": str(data.get("goal_name", "")),
            "stage": str(data.get("stage", "idle")),
            "path_index": path_index,
            "points": points,
        }

    def on_rx_timer(self) -> None:
        if not self.enabled:
            return
        while True:
            try:
                data, addr = self.sock.recvfrom(self.max_packet_bytes)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().warn(f"UDP receive failed: {exc}")
                break
            self.client_addr = addr
            try:
                payload = json.loads(data.decode("utf-8"))
                self.handle_packet(payload)
            except Exception as exc:
                self.send_packet({"type": "error", "message": str(exc)})

    def handle_packet(self, payload: dict[str, Any]) -> None:
        msg_type = str(payload.get("type", "")).lower()
        if msg_type == "cmd_vel":
            cmd = self.parse_twist(payload)
            self.set_control_mode("WEB")
            self.cmd_pub.publish(cmd)
            self.last_cmd_time = self.get_clock().now()
            self.timeout_estop_sent = False
        elif msg_type == "zero":
            self.cmd_pub.publish(Twist())
            self.last_cmd_time = self.get_clock().now()
        elif msg_type == "estop":
            self.handle_estop_request(bool(payload.get("data", True)))
        elif msg_type == "mode":
            mode = str(payload.get("mode", "DISABLED")).upper()
            self.set_control_mode(mode)
        elif msg_type == "web_enable":
            self.web_enabled = bool(payload.get("data", False))
            self.web_enabled_pub.publish(Bool(data=self.web_enabled))
            if self.web_enabled:
                self.set_control_mode("WEB")
        elif msg_type == "remote_enable":
            enabled = bool(payload.get("data", False))
            self.remote_enabled_pub.publish(Bool(data=enabled))
            if enabled:
                self.set_control_mode("REMOTE")
        elif msg_type == "nav_enable":
            enabled = bool(payload.get("data", False))
            self.nav_enabled_pub.publish(Bool(data=enabled))
            if enabled:
                self.set_control_mode("NAV")
        elif msg_type == "nav_cmd":
            command = str(payload.get("command", "")).strip()
            if command:
                self.set_control_mode("NAV")
                self.nav_cmd_pub.publish(String(data=command))
        elif msg_type == "nav_stop_keep":
            self.stop_navigation_for_keep("manual stop")
        elif msg_type == "task_resume":
            task_name = str(payload.get("task", "")).strip()
            if task_name:
                self.ensure_navigation_pose(f"task resume {task_name}")
                self.set_control_mode("NAV")
                self.nav_cmd_pub.publish(String(data="reload"))
                self.nav_cmd_pub.publish(String(data=f"run_from_task {task_name}"))
        elif msg_type == "task_only":
            task_name = str(payload.get("task", "")).strip()
            if task_name:
                self.ensure_navigation_pose(f"task only {task_name}")
                self.set_control_mode("NAV")
                self.nav_cmd_pub.publish(String(data="reload"))
                self.nav_cmd_pub.publish(String(data=f"run_only_task {task_name}"))
        elif msg_type == "task_skip":
            task_name = str(payload.get("task", "")).strip()
            self.ensure_navigation_pose(f"task skip {task_name or 'current'}")
            self.set_control_mode("NAV")
            command = f"skip_task {task_name}" if task_name else "skip_task"
            self.nav_cmd_pub.publish(String(data="reload"))
            self.nav_cmd_pub.publish(String(data=command))
        elif msg_type == "go_to":
            x = float(payload.get("x", 0.0))
            y = float(payload.get("y", 0.0))
            self.set_control_mode("NAV")
            self.nav_cmd_pub.publish(String(data=f"go {x:.3f} {y:.3f}"))
        elif msg_type == "go_rel":
            dx = float(payload.get("dx", 0.0))
            dy = float(payload.get("dy", 0.0))
            self.set_control_mode("NAV")
            self.nav_cmd_pub.publish(String(data=f"go_rel {dx:.3f} {dy:.3f}"))
        elif msg_type == "odom_task":
            self.start_odom_fallback_task()
        elif msg_type == "odom_stop":
            self.stop_odom_fallback("manual stop", stop_nav=True)
        elif msg_type == "estop_reset":
            self.handle_estop_request(False)
        elif msg_type == "model_toggle":
            self.model_cmd_pub.publish(String(data="toggle"))
        elif msg_type == "model_cmd":
            command = str(payload.get("command", "")).strip()
            if command:
                self.model_cmd_pub.publish(String(data=command))
        elif msg_type == "ping":
            self.send_packet({"type": "pong", "stamp": self.now_sec()})
        elif msg_type == "map_request":
            packet = self.build_map_packet()
            packet["type"] = "map"
            self.send_packet(packet)
        else:
            self.send_packet({"type": "error", "message": f"unknown packet type: {msg_type}"})

    def set_control_mode(self, mode: str) -> None:
        mode = str(mode).upper()
        self.mode_pub.publish(String(data=mode))
        self.web_enabled = mode == "WEB"
        self.web_enabled_pub.publish(Bool(data=mode == "WEB"))
        self.remote_enabled_pub.publish(Bool(data=mode == "REMOTE"))
        self.nav_enabled_pub.publish(Bool(data=mode == "NAV"))
        if mode == "KEEP":
            self.posture_cmd_pub.publish(String(data="keep"))
        elif mode in {"REMOTE", "NAV", "WEB"}:
            self.posture_cmd_pub.publish(String(data="default"))

    def stop_navigation_for_keep(self, reason: str, enter_keep: bool = True) -> None:
        self.nav_cmd_pub.publish(String(data="stop"))
        if enter_keep:
            self.set_control_mode("KEEP")

    def handle_estop_request(self, active: bool) -> None:
        if active:
            self.stop_navigation_for_keep("estop requested", enter_keep=False)
            self.estop_pub.publish(Bool(data=True))
        else:
            self.estop_pub.publish(Bool(data=False))
            self.stop_navigation_for_keep("estop reset")

    def get_default_mission_goals(self) -> list[dict[str, Any]]:
        goal_lookup = {goal["name"]: goal for goal in self.goal_specs if goal.get("name")}
        if not self.default_mission_name:
            return []
        for mission in self.mission_specs:
            if mission["name"] != self.default_mission_name:
                continue
            return [
                goal_lookup[goal_name]
                for goal_name in mission.get("goals", [])
                if goal_name in goal_lookup
            ]
        return []

    def get_current_task_name(self) -> str:
        status = self.latest_nav_status or ""
        marker = "task="
        marker_index = status.find(marker)
        if marker_index >= 0:
            tail = status[marker_index + len(marker):].strip()
            task = tail.split()[0].strip() if tail else ""
            if task:
                return task
        active_goal_name = str(self.latest_nav_path.get("goal_name", "") or "")
        if active_goal_name:
            for goal in self.goal_specs:
                if goal.get("name") == active_goal_name:
                    return str(goal.get("task") or "")
        return ""

    def ensure_navigation_pose(self, reason: str) -> None:
        if self.lookup_pose() is not None:
            return
        if self.odom_fallback_active:
            return
        if not self.odom_fallback_allowed:
            raise ValueError(
                f"{reason} waiting for relocalization TF {self.nav_map_frame}->{self.nav_base_frame}; "
                "move the robot until reloc shows on/tf ok"
            )

        self.reload_nav_task_config()
        default_goals = self.get_default_mission_goals()
        if not self.default_mission_name or not default_goals:
            raise ValueError(f"{reason} failed: no default mission loaded")

        anchor_goal = default_goals[0]
        yaw_deg = anchor_goal.get("yaw_deg")
        if yaw_deg is None:
            yaw_deg = 0.0
        self.activate_odom_fallback(
            float(anchor_goal["x"]),
            float(anchor_goal["y"]),
            math.radians(float(yaw_deg)),
            str(anchor_goal.get("name", "route_p01")),
        )
        if not self.odom_trace_active:
            self.start_odom_trace(self.default_mission_name or "odom_task", anchor_goal)
        self.get_logger().info(f"{reason}: started odom fallback anchor before navigation command")

    def start_odom_fallback_task(self) -> None:
        if not self.odom_fallback_allowed:
            raise ValueError("odom task unavailable in relocalization mode; wait for reloc on/tf ok then use resume")
        self.reload_nav_task_config()
        self.ensure_navigation_pose("odom task")
        if not self.default_mission_name:
            raise ValueError("odom task unavailable: no default mission loaded")
        self.set_control_mode("NAV")
        self.nav_cmd_pub.publish(String(data="reload"))
        self.nav_cmd_pub.publish(String(data=f"run {self.default_mission_name}"))

    def activate_odom_fallback(self, map_x: float, map_y: float, map_yaw: float, anchor_name: str) -> None:
        self.validate_odom_fallback_preconditions()
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.nav_odom_frame,
                self.nav_base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            raise ValueError(
                f"odom fallback failed: missing TF {self.nav_odom_frame}->{self.nav_base_frame}: {exc}"
            ) from exc

        t = odom_to_base.transform.translation
        q = odom_to_base.transform.rotation
        odom_x = float(t.x)
        odom_y = float(t.y)
        odom_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        map_to_odom_yaw = self.normalize_angle(map_yaw - odom_yaw)
        cos_yaw = math.cos(map_to_odom_yaw)
        sin_yaw = math.sin(map_to_odom_yaw)
        map_to_odom_x = map_x - (cos_yaw * odom_x - sin_yaw * odom_y)
        map_to_odom_y = map_y - (sin_yaw * odom_x + cos_yaw * odom_y)

        transform = TransformStamped()
        transform.header.frame_id = self.nav_map_frame
        transform.child_frame_id = self.nav_odom_frame
        transform.transform.translation.x = map_to_odom_x
        transform.transform.translation.y = map_to_odom_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(map_to_odom_yaw * 0.5)
        transform.transform.rotation.w = math.cos(map_to_odom_yaw * 0.5)

        self.odom_fallback_transform = transform
        self.odom_fallback_active = True
        self.odom_fallback_handoff_pending = False
        self.odom_fallback_handoff_info = {}
        self.odom_fallback_anchor = {
            "anchor": anchor_name,
            "map_pose": {
                "x": round(map_x, 3),
                "y": round(map_y, 3),
                "yaw_deg": round(math.degrees(map_yaw), 3),
            },
            "odom_pose_at_init": {
                "x": round(odom_x, 3),
                "y": round(odom_y, 3),
                "yaw_deg": round(math.degrees(odom_yaw), 3),
            },
            "map_to_odom": {
                "x": round(map_to_odom_x, 3),
                "y": round(map_to_odom_y, 3),
                "yaw_deg": round(math.degrees(map_to_odom_yaw), 3),
            },
        }
        self.broadcast_odom_fallback()
        self.latest_nav_status = (
            f"odom fallback active: {anchor_name} -> "
            f"map({map_x:.2f},{map_y:.2f},{math.degrees(map_yaw):.1f}deg)"
        )
        self.get_logger().info(self.latest_nav_status)

    def start_odom_trace(self, mission_name: str, anchor_goal: dict[str, Any]) -> None:
        self.odom_trace_active = True
        self.odom_trace_points = []
        self.odom_trace_started_at = self.now_sec()
        self.odom_trace_mission_name = mission_name
        self.odom_trace_last_pose = None
        self.odom_trace_export_path = ""
        self.odom_trace_anchor = {
            "name": str(anchor_goal.get("name", "route_p01")),
            "x": self._f(anchor_goal.get("x", 0.0)),
            "y": self._f(anchor_goal.get("y", 0.0)),
            "yaw_deg": self._f(anchor_goal.get("yaw_deg", anchor_goal.get("yawDeg", 0.0))),
        }
        self.record_odom_trace_sample(force=True)
        self.get_logger().info(
            f"Odom trace recording started: mission={self.odom_trace_mission_name}, "
            f"export_dir={self.odom_trace_export_dir}"
        )

    def on_odom_trace_timer(self) -> None:
        if self.odom_trace_active:
            self.record_odom_trace_sample(force=False)

    def record_odom_trace_sample(self, force: bool = False) -> None:
        if not self.odom_trace_active and not force:
            return
        if len(self.odom_trace_points) >= self.odom_trace_max_points:
            return
        pose = self.lookup_pose()
        if pose is None:
            return
        now_sec = self.now_sec()
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        yaw = float(pose.get("yaw", 0.0))
        if self.odom_trace_last_pose is not None and not force:
            dx = x - float(self.odom_trace_last_pose.get("x", 0.0))
            dy = y - float(self.odom_trace_last_pose.get("y", 0.0))
            if math.hypot(dx, dy) < self.odom_trace_min_distance:
                return
        sample = {
            "t": round(now_sec - self.odom_trace_started_at, 3),
            "x": round(x, 4),
            "y": round(y, 4),
            "yaw": round(yaw, 6),
            "yawDeg": round(math.degrees(yaw), 3),
        }
        self.odom_trace_points.append(sample)
        self.odom_trace_last_pose = sample

    def finish_odom_trace(self, reason: str) -> Optional[Path]:
        if not self.odom_trace_active and not self.odom_trace_points:
            return None
        if self.odom_trace_active:
            self.record_odom_trace_sample(force=True)
        self.odom_trace_active = False
        if not self.odom_trace_points:
            return None

        try:
            export_dir = Path(self.odom_trace_export_dir).expanduser()
            export_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            safe_mission = "".join(
                ch if ch.isalnum() or ch in {"_", "-"} else "_"
                for ch in (self.odom_trace_mission_name or "odom")
            ).strip("_") or "odom"
            export_path = export_dir / f"odom_trace_{safe_mission}_{stamp}.json"
            waypoints = []
            for index, point in enumerate(self.odom_trace_points, start=1):
                waypoints.append({
                    "id": index,
                    "x": point["x"],
                    "y": point["y"],
                    "world_x": point["x"],
                    "world_y": point["y"],
                    "yawDeg": point["yawDeg"],
                    "speed": 0.0,
                    "policy": "odom_trace",
                    "tolerance": 0.0,
                })
            payload = {
                "name": f"odom_trace_{safe_mission}_{stamp}",
                "type": "odom_trace",
                "created_at": datetime.now().isoformat(timespec="seconds"),
                "frame": self.nav_map_frame,
                "base_frame": self.nav_base_frame,
                "odom_frame": self.nav_odom_frame,
                "mission": self.odom_trace_mission_name,
                "reason": reason,
                "source_route_file": self.get_route_source_file(),
                "anchor": dict(self.odom_trace_anchor),
                "origin": {"mode": "world", "x": 0.0, "y": 0.0, "yaw_deg": 0.0},
                "sample_hz": self.odom_trace_sample_hz,
                "min_distance": self.odom_trace_min_distance,
                "samples": self.odom_trace_points,
                "waypoints": waypoints,
                "segments": [{"name": "odom_trace", "obstacle": "odom_trace", "waypoints": waypoints}],
            }
            export_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception as exc:
            self.get_logger().error(f"Odom trace export failed: {exc}")
            self.odom_trace_export_path = ""
            return None
        self.odom_trace_export_path = str(export_path)
        self.get_logger().info(
            f"Odom trace exported: {export_path} points={len(self.odom_trace_points)} reason={reason}"
        )
        return export_path

    def validate_odom_fallback_preconditions(self) -> None:
        if not self.odom_fallback_allowed:
            raise ValueError("odom fallback disabled in relocalization mode")
        if self.odom_fallback_require_odom_fresh:
            state = self.latest_state
            if state is None:
                raise ValueError("odom fallback failed: runtime/state unavailable, cannot verify odom freshness")
            odom_age_ms = float(state.odom_age_ms)
            if not bool(state.odom_fresh):
                raise ValueError(f"odom fallback failed: odom is not fresh (age={odom_age_ms:.1f}ms)")
            if self.odom_fallback_max_odom_age_ms > 0.0 and odom_age_ms > self.odom_fallback_max_odom_age_ms:
                raise ValueError(
                    "odom fallback failed: "
                    f"odom age {odom_age_ms:.1f}ms > {self.odom_fallback_max_odom_age_ms:.1f}ms"
                )

        if self.odom_fallback_block_existing_map_odom_tf and not self.odom_fallback_active:
            if self.recent_external_map_odom_tf_exists():
                direction = str(self.last_external_map_odom_tf.get("direction", "map<->odom"))
                raise ValueError(
                    f"odom fallback failed: recent external {direction} TF detected. "
                    "Disable Odin relocalization/map TF before starting pure odom fallback."
                )
            try:
                existing = self.tf_buffer.lookup_transform(
                    self.nav_map_frame,
                    self.nav_odom_frame,
                    rclpy.time.Time(),
                )
            except TransformException:
                return
            age_s = self.transform_age_s(existing)
            if age_s is not None and age_s > self.odom_fallback_tf_conflict_window_s:
                return
            raise ValueError(
                "odom fallback failed: existing map<->odom TF detected. "
                "Disable Odin relocalization/map TF before starting pure odom fallback."
            )

    def recent_external_map_odom_tf_exists(self) -> bool:
        if not self.last_external_map_odom_tf:
            return False
        stamp = self.last_external_map_odom_tf.get("stamp")
        if not isinstance(stamp, (int, float)):
            return False
        return self.now_sec() - float(stamp) <= self.odom_fallback_tf_conflict_window_s

    def transform_age_s(self, transform: TransformStamped) -> Optional[float]:
        stamp = self.stamp_to_sec(transform.header.stamp)
        if stamp <= 0.0:
            return None
        return max(0.0, self.now_sec() - stamp)

    def is_own_odom_fallback_tf(self, transform: TransformStamped) -> bool:
        expected = self.odom_fallback_transform
        if expected is None:
            return False
        t = transform.transform.translation
        e = expected.transform.translation
        xy_error = math.hypot(float(t.x) - float(e.x), float(t.y) - float(e.y))
        q = transform.transform.rotation
        eq = expected.transform.rotation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        expected_yaw = self.quaternion_to_yaw(eq.x, eq.y, eq.z, eq.w)
        yaw_error = abs(self.normalize_angle(yaw - expected_yaw))
        return (
            xy_error <= self.odom_fallback_tf_conflict_xy_tolerance
            and yaw_error <= self.odom_fallback_tf_conflict_yaw_tolerance
        )

    def stop_odom_fallback(self, reason: str, stop_nav: bool = True) -> None:
        was_active = self.odom_fallback_active
        if was_active:
            self.finish_odom_trace(reason)
        self.odom_fallback_active = False
        self.odom_fallback_transform = None
        self.odom_fallback_anchor = {}
        self.odom_fallback_handoff_pending = False
        self.odom_fallback_handoff_info = {}
        if stop_nav:
            self.nav_cmd_pub.publish(String(data="stop"))
        status = f"odom fallback stopped: {reason}" if was_active else f"odom fallback already inactive: {reason}"
        self.latest_nav_status = status
        self.get_logger().info(status)

    def on_odom_fallback_timer(self) -> None:
        if self.odom_fallback_active:
            self.broadcast_odom_fallback()

    def broadcast_odom_fallback(self) -> None:
        if self.odom_fallback_transform is None:
            return
        transform = self.odom_fallback_transform
        transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(transform)

    def parse_twist(self, payload: dict[str, Any]) -> Twist:
        cmd = Twist()
        linear = payload.get("linear", {}) or {}
        angular = payload.get("angular", {}) or {}
        cmd.linear.x = self.clamp(float(linear.get("x", 0.0)), -self.max_vx, self.max_vx)
        cmd.linear.y = self.clamp(float(linear.get("y", 0.0)), -self.max_vy, self.max_vy)
        cmd.angular.z = self.clamp(float(angular.get("z", 0.0)), -self.max_yaw, self.max_yaw)
        return cmd

    def on_guard_timer(self) -> None:
        age_ms = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1.0e6
        if age_ms > self.cmd_timeout_ms:
            self.cmd_pub.publish(Twist())
            if self.estop_on_timeout and not self.timeout_estop_sent:
                self.estop_pub.publish(Bool(data=True))
                self.timeout_estop_sent = True

    def on_state_timer(self) -> None:
        if not self.enabled:
            return
        self.send_packet(self.build_state_packet())

    def build_state_packet(self) -> dict[str, Any]:
        target = self.latest_target
        state = self.latest_state
        packet: dict[str, Any] = {
            "type": "state",
            "stamp": self.now_sec(),
            "mode": self.latest_mode,
            "mux_status": self.latest_mux_status,
            "nav_status": self.latest_nav_status,
            "estop": self.estop,
            "web_enabled": self.web_enabled,
            "connected": True,
            "local_receive_time": self.now_sec(),
            "cmd_vel": self.twist_to_dict(self.latest_cmd),
            "runtime": {},
            "robot": {},
            "nav": {
                "pose": self.lookup_pose(),
                "map_frame": self.nav_map_frame,
                "base_frame": self.nav_base_frame,
                "odom_frame": self.nav_odom_frame,
                "path": dict(self.latest_nav_path),
                "current_task": self.get_current_task_name(),
                "task_specs": self.task_specs,
                "localization_mode": self.localization_mode,
                "odom_fallback_allowed": self.odom_fallback_allowed,
                "odom_trace": self.build_odom_trace_packet(),
                "odom_fallback": {
                    "active": self.odom_fallback_active,
                    "handoff_pending": self.odom_fallback_handoff_pending,
                    "handoff": dict(self.odom_fallback_handoff_info),
                    **self.odom_fallback_anchor,
                },
                "relocalization": self.build_relocalization_packet(),
            },
            "model": dict(self.latest_model_status),
        }
        if target is not None:
            packet["runtime"] = {
                "target_source": str(target.target_source),
                "zero_command": bool(target.zero_command),
                "runtime_released": bool(target.runtime_released),
                "release_alpha": self._f(target.release_alpha),
                "command": [self._f(v) for v in target.command],
                "raw_command": [self._f(v) for v in target.raw_command],
            }
        if state is not None:
            packet["robot"] = {
                "joint_pos": [self._f(v) for v in state.joint_pos],
                "joint_vel": [self._f(v) for v in state.joint_vel],
                "joint_torque": [self._f(v) for v in state.joint_torque],
                "imu_gyro": [self._f(v) for v in state.imu_gyro],
                "imu_accel": [self._f(v) for v in state.imu_accel],
                "projected_gravity": [self._f(v) for v in state.projected_gravity],
                "quat_wxyz": [self._f(v) for v in state.quat_wxyz],
                "imu_age_ms": self._f(state.imu_age_ms),
                "imu_fresh": bool(state.imu_fresh),
                "odom_age_ms": self._f(state.odom_age_ms),
                "odom_fresh": bool(state.odom_fresh),
                "odom_local_pos": [self._f(v) for v in state.odom_local_pos],
                "odom_local_yaw": self._f(state.odom_local_yaw),
                "fresh_count": int(state.fresh_count),
                "holdover_count": int(state.holdover_count),
                "stale_max": int(state.stale_max),
                "update_counts": [int(v) for v in state.update_counts],
            }
        return packet

    def build_relocalization_packet(self) -> dict[str, Any]:
        external_tf = dict(self.last_external_map_odom_tf or {})
        received_at = self._f(external_tf.get("received_at")) if external_tf else 0.0
        age_s = self.now_sec() - received_at if received_at > 0.0 else 0.0
        return {
            "localization_mode": self.localization_mode,
            "odom_fallback_allowed": self.odom_fallback_allowed,
            "external_map_odom_tf": external_tf,
            "external_tf_seen": bool(external_tf),
            "external_tf_age_s": self._f(age_s),
            "handoff_pending": self.odom_fallback_handoff_pending,
            "handoff": dict(self.odom_fallback_handoff_info),
        }

    def send_packet(self, payload: dict[str, Any]) -> None:
        if self.client_addr is None:
            return
        try:
            data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.sock.sendto(data, self.client_addr)
        except OSError as exc:
            self.get_logger().warn(f"UDP send failed: {exc}")

    @staticmethod
    def _f(v: Any) -> float:
        try:
            f = float(v)
            if f != f:
                return 0.0
            return round(f, 6)
        except (TypeError, ValueError):
            return 0.0

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def stamp_to_sec(stamp: Any) -> float:
        try:
            return float(stamp.sec) + float(stamp.nanosec) / 1.0e9
        except Exception:
            return 0.0

    @staticmethod
    def twist_to_dict(msg: Twist) -> dict[str, Any]:
        return {
            "linear": {"x": msg.linear.x, "y": msg.linear.y, "z": msg.linear.z},
            "angular": {"x": msg.angular.x, "y": msg.angular.y, "z": msg.angular.z},
        }

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
        return max(low, min(high, value))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WebUdpBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.http_server is not None:
            node.http_server.shutdown()
            node.http_server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
