#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
import heapq
import json
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


@dataclass(frozen=True)
class GoalSpec:
    name: str
    x: float
    y: float
    waypoint_id: Optional[int] = None
    task: Optional[str] = None
    segment: Optional[str] = None
    yaw: Optional[float] = None
    yaw_tolerance: Optional[float] = None
    tolerance: float = 0.20
    speed: Optional[float] = None
    policy: Optional[str] = None
    require_yaw: bool = False
    pre_dock_distance: Optional[float] = None
    pre_dock_tolerance: Optional[float] = None
    precision_follow: bool = False
    stable_cycles: Optional[int] = None
    lookahead: Optional[float] = None
    yaw_rate_limit: Optional[float] = None
    slalom_straight: bool = False
    slalom_script_pos_tolerance: Optional[float] = None


@dataclass(frozen=True)
class SlalomScriptStep:
    kind: str
    start_index: int
    end_index: int
    target_x: float
    target_y: float
    target_yaw: Optional[float]
    pos_tolerance: Optional[float] = None
    forward: float = 0.0
    left: float = 0.0


@dataclass(frozen=True)
class AvoidRegion:
    name: str
    kind: str
    polygon: tuple[tuple[float, float], ...]


class SimpleNavNode(Node):
    def __init__(self) -> None:
        super().__init__("sim2real_simple_nav_node")

        self.map_frame = str(self.declare_parameter("nav_map_frame", "map").value)
        self.base_frame = str(self.declare_parameter("nav_base_frame", "base_link").value)
        self.control_hz = float(self.declare_parameter("nav_control_hz", 20.0).value)
        self.goal_tolerance = float(self.declare_parameter("nav_goal_tolerance", 0.20).value)
        self.yaw_stop_threshold = float(self.declare_parameter("nav_yaw_stop_threshold", 0.80).value)
        self.max_vx = float(self.declare_parameter("nav_max_vx", 0.45).value)
        self.max_vy = max(0.0, float(self.declare_parameter("nav_max_vy", 0.0).value))
        self.max_wz = float(self.declare_parameter("nav_max_wz", 0.8).value)
        self.kp_dist = float(self.declare_parameter("nav_kp_dist", 0.8).value)
        self.kp_yaw = float(self.declare_parameter("nav_kp_yaw", 1.8).value)
        self.goal_exit_tolerance_margin = max(
            0.0,
            float(self.declare_parameter("nav_goal_exit_tolerance_margin", 0.08).value),
        )
        self.goal_complete_stable_cycles = max(
            1,
            int(self.declare_parameter("nav_goal_complete_stable_cycles", 4).value),
        )
        self.final_align_kp_yaw_scale = max(
            0.1,
            float(self.declare_parameter("nav_final_align_kp_yaw_scale", 0.6).value),
        )
        self.final_align_max_wz = max(
            0.05,
            float(self.declare_parameter("nav_final_align_max_wz", 0.45).value),
        )
        self.final_align_creep_speed = max(
            0.0,
            float(self.declare_parameter("nav_final_align_creep_speed", 0.05).value),
        )
        self.goals_file = str(self.declare_parameter("nav_goals_file", "").value)
        self.missions_file = str(self.declare_parameter("nav_missions_file", "").value)
        self.route_file = str(self.declare_parameter("nav_route_file", "").value)
        self.route_task_file = str(self.declare_parameter("nav_route_task_file", "").value)
        self.goal_yaw_tolerance = math.radians(
            float(self.declare_parameter("nav_goal_yaw_tolerance_deg", 12.0).value)
        )
        self.turn_in_place_enabled = bool(
            self.declare_parameter("nav_turn_in_place_enabled", True).value
        )
        self.turn_in_place_enter_yaw = math.radians(
            max(0.0, float(self.declare_parameter("nav_turn_in_place_enter_yaw_deg", 70.0).value))
        )
        self.turn_in_place_exit_yaw = math.radians(
            max(0.0, float(self.declare_parameter("nav_turn_in_place_exit_yaw_deg", 18.0).value))
        )
        self.turn_in_place_max_wz = max(
            0.05,
            float(self.declare_parameter("nav_turn_in_place_max_wz", 0.8).value),
        )
        self.astar_enabled = bool(self.declare_parameter("nav_astar_enabled", True).value)
        self.astar_resolution = max(
            0.05,
            float(self.declare_parameter("nav_astar_resolution", 0.10).value),
        )
        self.astar_pcd_sample_step = max(
            1,
            int(self.declare_parameter("nav_astar_pcd_sample_step", 5).value),
        )
        self.astar_allow_diagonal = bool(
            self.declare_parameter("nav_astar_allow_diagonal", True).value
        )
        self.astar_smooth_enabled = bool(
            self.declare_parameter("nav_astar_smooth_enabled", True).value
        )
        self.astar_corner_blend_dist = max(
            0.0,
            float(self.declare_parameter("nav_astar_corner_blend_dist", 0.20).value),
        )
        self.astar_waypoint_reach_dist = max(
            0.05,
            float(self.declare_parameter("nav_astar_waypoint_reach_dist", 0.18).value),
        )
        self.astar_lookahead_dist = max(
            self.astar_waypoint_reach_dist,
            float(self.declare_parameter("nav_astar_lookahead_dist", 0.35).value),
        )
        self.astar_snap_radius = max(
            self.astar_resolution,
            float(self.declare_parameter("nav_astar_snap_radius", 0.60).value),
        )
        self.astar_max_expansions = max(
            1000,
            int(self.declare_parameter("nav_astar_max_expansions", 120000).value),
        )
        self.slalom_auto_precision_enabled = bool(
            self.declare_parameter("nav_slalom_auto_precision_enabled", True).value
        )
        self.slalom_auto_precision_force = bool(
            self.declare_parameter("nav_slalom_auto_precision_force", True).value
        )
        self.slalom_task_names = self.parse_csv_set(
            self.declare_parameter("nav_slalom_task_names", "slalom").value,
            {"slalom"},
        )
        self.slalom_stable_cycles = max(
            0,
            int(self.declare_parameter("nav_slalom_stable_cycles", 0).value),
        )
        self.slalom_lookahead = max(
            self.astar_waypoint_reach_dist,
            float(self.declare_parameter("nav_slalom_lookahead", 0.35).value),
        )
        self.slalom_yaw_rate_limit = max(
            0.01,
            float(self.declare_parameter("nav_slalom_yaw_rate_limit", 0.45).value),
        )
        self.slalom_tolerance = max(
            0.01,
            float(self.declare_parameter("nav_slalom_tolerance", 0.15).value),
        )
        self.slalom_max_vx = max(
            0.0,
            float(self.declare_parameter("nav_slalom_max_vx", 0.55).value),
        )
        self.slalom_min_vx = max(
            0.0,
            float(self.declare_parameter("nav_slalom_min_vx", 0.08).value),
        )
        self.slalom_curvature_slowdown_enabled = bool(
            self.declare_parameter("nav_slalom_curvature_slowdown_enabled", True).value
        )
        self.slalom_min_turn_speed_scale = self.clamp(
            float(self.declare_parameter("nav_slalom_min_turn_speed_scale", 0.45).value),
            0.05,
            1.0,
        )
        self.slalom_script_enabled = bool(
            self.declare_parameter("nav_slalom_script_enabled", False).value
        )
        self.slalom_script_start_tolerance = max(
            0.01,
            float(self.declare_parameter("nav_slalom_script_start_tolerance", 0.10).value),
        )
        self.slalom_script_pos_tolerance = max(
            0.01,
            float(self.declare_parameter("nav_slalom_script_pos_tolerance", 0.08).value),
        )
        self.slalom_script_yaw_tolerance = math.radians(
            max(0.0, float(self.declare_parameter("nav_slalom_script_yaw_tolerance_deg", 5.0).value))
        )
        self.slalom_script_drive_yaw_deadband = math.radians(
            max(0.0, float(self.declare_parameter("nav_slalom_script_drive_yaw_deadband_deg", 5.0).value))
        )
        self.slalom_script_segment_yaw_min_dist = max(
            0.0,
            float(self.declare_parameter("nav_slalom_script_segment_yaw_min_dist", 0.0).value),
        )
        self.slalom_script_stable_cycles = max(
            1,
            int(self.declare_parameter("nav_slalom_script_stable_cycles", 1).value),
        )
        self.slalom_script_rotate_steps_enabled = bool(
            self.declare_parameter("nav_slalom_script_rotate_steps_enabled", False).value
        )
        self.slalom_script_final_rotate_enabled = bool(
            self.declare_parameter("nav_slalom_script_final_rotate_enabled", False).value
        )
        self.slalom_script_require_yaw_at_step = bool(
            self.declare_parameter("nav_slalom_script_require_yaw_at_step", False).value
        )
        self.slalom_script_kp_dist = max(
            0.0,
            float(self.declare_parameter("nav_slalom_script_kp_dist", 1.0).value),
        )
        self.slalom_script_kp_yaw = max(
            0.0,
            float(self.declare_parameter("nav_slalom_script_kp_yaw", 1.6).value),
        )
        self.slalom_script_max_vx = max(
            0.0,
            float(self.declare_parameter("nav_slalom_script_max_vx", 0.60).value),
        )
        self.slalom_script_max_vy = max(
            0.0,
            float(self.declare_parameter("nav_slalom_script_max_vy", 0.50).value),
        )
        self.slalom_script_max_wz = max(
            0.01,
            float(self.declare_parameter("nav_slalom_script_max_wz", 0.50).value),
        )
        self.slalom_script_min_cmd_linear = max(
            0.0,
            float(self.declare_parameter("nav_slalom_script_min_cmd_linear", 0.20).value),
        )
        self.slalom_script_min_cmd_angular = max(
            0.0,
            float(self.declare_parameter("nav_slalom_script_min_cmd_angular", 0.0).value),
        )
        self.slalom_script_min_cmd_epsilon = max(
            0.0,
            float(self.declare_parameter("nav_slalom_script_min_cmd_epsilon", 0.05).value),
        )
        self.slalom_script_min_step_distance = max(
            0.0,
            float(self.declare_parameter("nav_slalom_script_min_step_distance", 0.02).value),
        )
        self.slalom_script_yaw_gate = math.radians(
            max(0.0, float(self.declare_parameter("nav_slalom_script_yaw_gate_deg", 25.0).value))
        )
        self.slalom_script_lateral_gate = max(
            0.0,
            float(self.declare_parameter("nav_slalom_script_lateral_gate", 0.08).value),
        )
        self.slalom_script_lateral_slow_gate = max(
            self.slalom_script_lateral_gate,
            float(self.declare_parameter("nav_slalom_script_lateral_slow_gate", 0.18).value),
        )
        self.slalom_script_lateral_creep_vx = max(
            0.0,
            float(self.declare_parameter("nav_slalom_script_lateral_creep_vx", 0.22).value),
        )
        self.slalom_script_drive_yaw_source = str(
            self.declare_parameter("nav_slalom_script_drive_yaw_source", "segment").value
        ).strip().lower()
        if self.slalom_script_drive_yaw_source not in {"current", "next", "segment"}:
            self.slalom_script_drive_yaw_source = "current"
        self.precision_lateral_control_enabled = bool(
            self.declare_parameter("nav_precision_lateral_control_enabled", False).value
        )
        self.precision_lateral_kp = max(
            0.0,
            float(self.declare_parameter("nav_precision_lateral_kp", 0.8).value),
        )
        self.precision_lateral_max_vy = max(
            0.0,
            float(self.declare_parameter("nav_precision_lateral_max_vy", 0.15).value),
        )
        self.local_planner_enabled = bool(
            self.declare_parameter("nav_local_planner_enabled", True).value
        )
        self.local_planner_task_names = self.parse_csv_set(
            self.declare_parameter("nav_local_planner_tasks", "slalom").value,
            {"slalom"},
        )
        self.local_planner_precision_enabled = bool(
            self.declare_parameter("nav_local_planner_precision_enabled", True).value
        )
        self.local_planner_sim_time = max(
            0.05,
            float(self.declare_parameter("nav_local_planner_sim_time", 0.9).value),
        )
        self.local_planner_sim_dt = max(
            0.02,
            float(self.declare_parameter("nav_local_planner_sim_dt", 0.1).value),
        )
        self.local_planner_v_samples = max(
            1,
            int(self.declare_parameter("nav_local_planner_v_samples", 5).value),
        )
        self.local_planner_w_samples = max(
            1,
            int(self.declare_parameter("nav_local_planner_w_samples", 7).value),
        )
        self.local_planner_vy_samples = max(
            1,
            int(self.declare_parameter("nav_local_planner_vy_samples", 3).value),
        )
        self.local_planner_obstacle_margin = max(
            0.0,
            float(self.declare_parameter("nav_local_planner_obstacle_margin", 0.08).value),
        )
        self.local_planner_recovery_clearance_epsilon = max(
            0.0,
            float(
                self.declare_parameter(
                    "nav_local_planner_recovery_clearance_epsilon",
                    0.005,
                ).value
            ),
        )
        self.configured_local_planner_robot_radius = float(
            self.declare_parameter("nav_local_planner_robot_radius", 0.0).value
        )
        self.local_planner_clearance_weight = float(
            self.declare_parameter("nav_local_planner_clearance_weight", 2.0).value
        )
        self.local_planner_path_weight = float(
            self.declare_parameter("nav_local_planner_path_weight", 2.0).value
        )
        self.local_planner_heading_weight = float(
            self.declare_parameter("nav_local_planner_heading_weight", 0.7).value
        )
        self.local_planner_speed_weight = float(
            self.declare_parameter("nav_local_planner_speed_weight", 0.3).value
        )
        self.local_planner_nominal_weight = float(
            self.declare_parameter("nav_local_planner_nominal_weight", 1.0).value
        )
        self.local_planner_min_vx = max(
            0.0,
            float(self.declare_parameter("nav_local_planner_min_vx", 0.08).value),
        )
        self.slalom_script_safety_filter_enabled = bool(
            self.declare_parameter("nav_slalom_script_safety_filter_enabled", False).value
        )
        self.local_planner_use_astar_grid = bool(
            self.declare_parameter("nav_local_planner_use_astar_grid", False).value
        )
        self.pre_dock_enabled = bool(self.declare_parameter("nav_pre_dock_enabled", True).value)
        self.pre_dock_distance = max(
            0.0,
            float(self.declare_parameter("nav_pre_dock_distance", 0.35).value),
        )
        self.pre_dock_tolerance = max(
            0.01,
            float(self.declare_parameter("nav_pre_dock_tolerance", 0.18).value),
        )
        self.pre_dock_skip_within_goal_dist = max(
            0.0,
            float(self.declare_parameter("nav_pre_dock_skip_within_goal_dist", 0.45).value),
        )
        self.pcd_nav_file = str(self.declare_parameter("pcd_nav_file", "").value)
        self.pcd_floor_z_min = float(self.declare_parameter("pcd_floor_z_min", -1.6).value)
        self.pcd_floor_z_max = float(self.declare_parameter("pcd_floor_z_max", 0.4).value)
        self.pcd_sample_step = max(1, int(self.declare_parameter("pcd_sample_step", 25).value))
        self.pcd_robot_radius = max(
            0.0,
            float(self.declare_parameter("pcd_robot_radius", 0.18).value),
        )
        self.avoid_regions_enabled = bool(
            self.declare_parameter("nav_avoid_regions_enabled", True).value
        )
        self.avoid_region_margin = max(
            0.0,
            float(self.declare_parameter("nav_avoid_region_margin", 0.05).value),
        )
        self.configured_avoid_footprint_radius = float(
            self.declare_parameter("nav_avoid_footprint_radius", 0.0).value
        )
        self.robot_body_length = max(
            0.0,
            float(self.declare_parameter("nav_robot_body_length", 0.356).value),
        )
        self.robot_body_width = max(
            0.0,
            float(self.declare_parameter("nav_robot_body_width", 0.235).value),
        )
        self.robot_origin_from_front = max(
            0.0,
            float(self.declare_parameter("nav_robot_origin_from_front", 0.105).value),
        )
        self.robot_body_center_x = float(
            self.declare_parameter("nav_robot_body_center_x", 0.1518).value
        )
        self.robot_pose_hip = float(
            self.declare_parameter("nav_robot_pose_hip", 0.550).value
        )
        self.robot_pose_knee = float(
            self.declare_parameter("nav_robot_pose_knee", -1.125).value
        )
        self.robot_wheel_vis_length = max(
            0.0,
            float(self.declare_parameter("nav_robot_wheel_vis_length", 0.16).value),
        )
        self.robot_wheel_vis_width = max(
            0.0,
            float(self.declare_parameter("nav_robot_wheel_vis_width", 0.055).value),
        )
        self.robot_footprint_padding = max(
            0.0,
            float(self.declare_parameter("nav_robot_footprint_padding", 0.03).value),
        )
        self.robot_footprint_radius = self.compute_robot_footprint_radius()
        self.robot_lateral_footprint_radius = self.compute_robot_lateral_footprint_radius()
        self.avoid_footprint_radius = (
            max(0.0, self.configured_avoid_footprint_radius)
            if self.configured_avoid_footprint_radius > 0.0
            else self.robot_lateral_footprint_radius
        )
        self.local_planner_robot_radius = (
            max(0.0, self.configured_local_planner_robot_radius)
            if self.configured_local_planner_robot_radius > 0.0
            else self.robot_lateral_footprint_radius
        )
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
        self.policy_retry_period_s = max(
            0.1,
            float(self.declare_parameter("nav_policy_retry_period_s", 0.5).value),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_nav", 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "simple_nav/goal_pose", 10)
        self.status_pub = self.create_publisher(String, "simple_nav/status", 10)
        self.record_pub = self.create_publisher(String, "simple_nav/recorded_pose", 10)
        self.path_pub = self.create_publisher(String, "simple_nav/path", 10)
        self.model_cmd_pub = self.create_publisher(String, "runtime/model_cmd", 10)
        self.create_subscription(String, "simple_nav/cmd", self.on_command, 10)
        self.create_subscription(String, "runtime/model_status", self.on_model_status, 10)

        self.route_alignment_info: dict[str, Any] = {}
        self.pcd_points = self.load_filtered_pcd(Path(self.pcd_nav_file)) if self.pcd_nav_file else []
        self.astar_source_points = (
            self.load_filtered_pcd(Path(self.pcd_nav_file), sample_step=self.astar_pcd_sample_step)
            if self.pcd_nav_file and self.astar_enabled
            else []
        )
        self.route_avoid_regions: list[AvoidRegion] = []
        self.astar_walkable_cells: set[tuple[int, int]] = set()
        self.astar_grid_info: dict[str, Any] = {}
        self.goals: dict[str, GoalSpec] = {}
        self.missions: dict[str, list[str]] = {}
        self.default_mission_name: Optional[str] = None
        self.task_goal_names: dict[str, list[str]] = {}
        self.task_order: list[str] = []
        self.waypoint_id_goal_names: dict[int, str] = {}
        self.reload_navigation_data()
        self.active_goal: Optional[GoalSpec] = None
        self.active_mission_name: Optional[str] = None
        self.active_mission_goals: list[str] = []
        self.active_mission_index = 0
        self.active_goal_stage = "idle"
        self.pending_final_goal: Optional[GoalSpec] = None
        self.active_path_world: list[tuple[float, float]] = []
        self.active_path_index = 0
        self.active_path_goal_key: Optional[tuple[Any, ...]] = None
        self.active_precision_segment_start: Optional[int] = None
        self.active_precision_segment_end: Optional[int] = None
        self.last_published_path_signature: Optional[tuple[str, str, int, int]] = None
        self.slalom_script_active = False
        self.slalom_script_key: Optional[tuple[str, int, int]] = None
        self.slalom_script_steps: list[SlalomScriptStep] = []
        self.slalom_script_step_index = 0
        self.slalom_script_step_stable_count = 0
        self.turn_in_place_mode = False
        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0
        self.pose_waiting_reported = False
        self.current_model_policy: Optional[str] = None
        self.pending_policy: Optional[str] = None
        self.model_switching = False
        self.last_policy_request_time = self.get_clock().now()

        period = 1.0 / self.control_hz if self.control_hz > 0.0 else 0.05
        self.timer = self.create_timer(period, self.on_timer)
        self.get_logger().info(
            "Simple nav started "
            f"(map_frame={self.map_frame}, base_frame={self.base_frame}, "
            f"goals={len(self.goals)}, missions={len(self.missions)}, "
            f"default_mission={self.default_mission_name or 'none'}, "
            f"route_source={self.get_route_source_file() or 'none'}, "
            f"avoid_regions={len(self.route_avoid_regions)}, "
            f"astar_cells={len(self.astar_walkable_cells)})"
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

    def _normalize_policy(self, value: Any) -> Optional[str]:
        text = str(value).strip().lower()
        if not text:
            return None
        if text == "ik":
            return "crawl"
        if text in {"rough", "crawl", "wall"}:
            return text
        return None

    def _normalize_task(self, value: Any) -> Optional[str]:
        if value is None:
            return None
        text = str(value).strip().lower()
        if text == "ramp_bridge":
            text = "ramp"
        elif text == "stairs":
            text = "stairs_up"
        if text in {"", "none", "null", "custom", "flat"}:
            return None
        return text or None

    def parse_csv_set(self, value: Any, default: set[str]) -> set[str]:
        if isinstance(value, (list, tuple)):
            items = value
        else:
            items = str(value or "").replace(";", ",").split(",")
        parsed = {
            normalized
            for item in items
            if (normalized := self._normalize_task(item)) is not None
        }
        return parsed or set(default)

    def is_slalom_task(self, task: Any) -> bool:
        normalized = self._normalize_task(task)
        return bool(normalized is not None and normalized in self.slalom_task_names)

    def should_auto_precision_for_task(self, task: Any) -> bool:
        return bool(self.slalom_auto_precision_enabled and self.is_slalom_task(task))

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

    def compute_robot_footprint_radius(self) -> float:
        half_width = self.robot_body_width * 0.5
        body_center_offset_x = self.robot_origin_from_front - self.robot_body_length * 0.5
        radius = max(
            self.pcd_robot_radius,
            math.hypot(body_center_offset_x + self.robot_body_length * 0.5, half_width),
            math.hypot(body_center_offset_x + self.robot_body_length * -0.5, half_width),
        )

        for wheel_x, wheel_y in self.robot_wheel_local_points(body_center_offset_x):
            for sx in (-0.5, 0.5):
                for sy in (-0.5, 0.5):
                    radius = max(
                        radius,
                        math.hypot(
                            wheel_x + sx * self.robot_wheel_vis_length,
                            wheel_y + sy * self.robot_wheel_vis_width,
                        ),
                    )
        return radius + self.robot_footprint_padding

    def compute_robot_lateral_footprint_radius(self) -> float:
        half_width = self.robot_body_width * 0.5
        radius = max(self.pcd_robot_radius, half_width)
        body_center_offset_x = self.robot_origin_from_front - self.robot_body_length * 0.5
        for _, wheel_y in self.robot_wheel_local_points(body_center_offset_x):
            radius = max(radius, abs(wheel_y) + self.robot_wheel_vis_width * 0.5)
        return radius + self.robot_footprint_padding

    def robot_wheel_local_points(self, body_center_offset_x: float) -> list[tuple[float, float]]:
        thigh_dx = -0.25 * math.sin(self.robot_pose_hip)
        shank_dx = -0.2 * math.sin(self.robot_pose_hip + self.robot_pose_knee)
        wheel_positions = (
            ((0.32826 + 0.06389) - self.robot_body_center_x, 0.066172 - 0.027344, 0.1035, 0.014699, 0.04074, 0.0),
            ((0.32826 + 0.06389) - self.robot_body_center_x, -0.065853 + 0.027311, -0.1035, -0.018447, -0.040735, -0.00075079),
            ((-0.024743 - 0.06389) - self.robot_body_center_x, 0.066141 - 0.027309, 0.099459, 0.012475, 0.040737, 0.0),
            ((-0.024743 - 0.06389) - self.robot_body_center_x, -0.065884 + 0.027341, -0.099408, -0.012435, -0.040737, -0.00075079),
        )
        return [
            (
                body_center_offset_x + pitch_x + knee_x + thigh_dx + shank_dx,
                pitch_y + knee_y + wheel_y + wheel_geom_y,
            )
            for pitch_x, pitch_y, knee_y, wheel_y, wheel_geom_y, knee_x in wheel_positions
        ]

    def _load_goals(self, path_value: str) -> dict[str, GoalSpec]:
        data = self._load_yaml(path_value)
        raw_goals = data.get("goals", {})
        parsed: dict[str, GoalSpec] = {}
        if not isinstance(raw_goals, dict):
            return parsed
        for name, spec in raw_goals.items():
            if not isinstance(spec, dict):
                continue
            pos = spec.get("position", [0.0, 0.0, 0.0])
            if not isinstance(pos, list) or len(pos) < 2:
                continue
            try:
                x = float(pos[0])
                y = float(pos[1])
                yaw_value = self._get_float(spec, "yaw_deg", "yawDeg")
                yaw = math.radians(float(yaw_value)) if yaw_value is not None else None
                tolerance = float(spec.get("tolerance", self.goal_tolerance))
                speed = float(spec["speed"]) if spec.get("speed") is not None else None
                yaw_tolerance_deg = self._get_float(spec, "yaw_tolerance_deg", "yawToleranceDeg")
                require_yaw = self._get_bool(spec, "require_yaw", "requireYaw")
                parsed[name] = GoalSpec(
                    name=str(name),
                    x=x,
                    y=y,
                    yaw=yaw,
                    yaw_tolerance=(
                        math.radians(max(0.0, yaw_tolerance_deg))
                        if yaw_tolerance_deg is not None else None
                    ),
                    tolerance=max(0.01, tolerance),
                    speed=speed,
                    policy=self._normalize_policy(spec.get("policy")),
                    require_yaw=bool(yaw is not None if require_yaw is None else require_yaw) and yaw is not None,
                    pre_dock_distance=self._get_float(spec, "pre_dock_distance", "preDockDistance"),
                    pre_dock_tolerance=self._get_float(spec, "pre_dock_tolerance", "preDockTolerance"),
                    precision_follow=bool(self._get_bool(spec, "precision_follow", "precisionFollow")),
                    stable_cycles=self._get_int(spec, "stable_cycles", "stableCycles"),
                    lookahead=self._get_float(spec, "lookahead", "lookAhead"),
                    yaw_rate_limit=self._get_float(spec, "yaw_rate_limit", "yawRateLimit"),
                    slalom_straight=bool(self._get_bool(spec, "slalom_straight", "slalomStraight")),
                )
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

    def load_filtered_pcd(
        self,
        path: Path,
        sample_step: Optional[int] = None,
    ) -> list[tuple[float, float]]:
        if not path.exists():
            self.get_logger().warn(f"PCD file not found for route alignment: {path}")
            return []

        points: list[tuple[float, float]] = []
        data_started = False
        try:
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
                            points.append((x, y))
                    elif stripped.upper().startswith("DATA"):
                        if "ascii" not in stripped.lower():
                            self.get_logger().warn("Only ASCII PCD is supported for route alignment")
                            return []
                        data_started = True
        except Exception as exc:
            self.get_logger().warn(f"Failed to load PCD {path}: {exc}")
            return []
        step = self.pcd_sample_step if sample_step is None else max(1, int(sample_step))
        return points[::step]

    def build_astar_grid(
        self,
        points: list[tuple[float, float]],
    ) -> tuple[set[tuple[int, int]], dict[str, Any]]:
        if not self.astar_enabled or not points:
            return set(), {}

        resolution = self.astar_resolution
        raw_cells = {
            self.world_to_cell(x, y)
            for x, y in points
        }
        if not raw_cells:
            return set(), {}

        clearance_cells = max(0, int(math.ceil(self.pcd_robot_radius / resolution)) - 1)
        walkable_cells = raw_cells
        if clearance_cells > 0:
            offsets = [
                (dx, dy)
                for dx in range(-clearance_cells, clearance_cells + 1)
                for dy in range(-clearance_cells, clearance_cells + 1)
                if math.hypot(dx, dy) * resolution <= self.pcd_robot_radius + 1.0e-9
            ]
            filtered = {
                cell for cell in raw_cells
                if all((cell[0] + dx, cell[1] + dy) in raw_cells for dx, dy in offsets)
            }
            if filtered:
                walkable_cells = filtered

        before_avoid_count = len(walkable_cells)
        if self.avoid_regions_enabled and self.route_avoid_regions:
            walkable_cells = {
                cell for cell in walkable_cells
                if not self.cell_blocked_by_avoid_regions(cell)
            }
            if not walkable_cells:
                return set(), {
                    "resolution": resolution,
                    "clearance_cells": clearance_cells,
                    "cell_count": 0,
                    "avoid_regions": len(self.route_avoid_regions),
                    "avoid_blocked_cells": before_avoid_count,
                    "bounds": [],
                }

        xs = [cell[0] for cell in walkable_cells]
        ys = [cell[1] for cell in walkable_cells]
        return walkable_cells, {
            "resolution": resolution,
            "clearance_cells": clearance_cells,
            "cell_count": len(walkable_cells),
            "avoid_regions": len(self.route_avoid_regions),
            "avoid_clearance": self.avoid_clearance(),
            "avoid_blocked_cells": before_avoid_count - len(walkable_cells),
            "bounds": [min(xs), min(ys), max(xs), max(ys)],
        }

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        resolution = self.astar_resolution
        return int(round(float(x) / resolution)), int(round(float(y) / resolution))

    def cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        resolution = self.astar_resolution
        return float(cell[0]) * resolution, float(cell[1]) * resolution

    def avoid_clearance(self) -> float:
        if not self.avoid_regions_enabled or not self.route_avoid_regions:
            return 0.0
        return self.avoid_footprint_radius + self.avoid_region_margin

    def cell_blocked_by_avoid_regions(self, cell: tuple[int, int]) -> bool:
        x, y = self.cell_to_world(cell)
        return self.point_blocked_by_avoid_regions(x, y)

    def point_blocked_by_avoid_regions(self, x: float, y: float) -> bool:
        if not self.avoid_regions_enabled or not self.route_avoid_regions:
            return False
        clearance = self.avoid_clearance()
        for region in self.route_avoid_regions:
            if self.point_in_region_clearance(x, y, region.polygon, clearance):
                return True
        return False

    @classmethod
    def point_in_region_clearance(
        cls,
        x: float,
        y: float,
        polygon: tuple[tuple[float, float], ...],
        clearance: float,
    ) -> bool:
        if len(polygon) < 3:
            return False
        min_x = min(point[0] for point in polygon) - clearance
        max_x = max(point[0] for point in polygon) + clearance
        min_y = min(point[1] for point in polygon) - clearance
        max_y = max(point[1] for point in polygon) + clearance
        if x < min_x or x > max_x or y < min_y or y > max_y:
            return False
        if cls.point_in_polygon(x, y, polygon):
            return True
        if clearance <= 0.0:
            return False
        return any(
            cls.point_segment_distance(x, y, ax, ay, bx, by) <= clearance
            for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1])
        )

    @staticmethod
    def point_in_polygon(
        x: float,
        y: float,
        polygon: tuple[tuple[float, float], ...],
    ) -> bool:
        inside = False
        count = len(polygon)
        for index in range(count):
            ax, ay = polygon[index]
            bx, by = polygon[(index + 1) % count]
            if SimpleNavNode.point_segment_distance(x, y, ax, ay, bx, by) <= 1.0e-9:
                return True
            intersects = (ay > y) != (by > y)
            if intersects:
                cross_x = (bx - ax) * (y - ay) / ((by - ay) or 1.0e-12) + ax
                if x < cross_x:
                    inside = not inside
        return inside

    @staticmethod
    def point_segment_distance(
        px: float,
        py: float,
        ax: float,
        ay: float,
        bx: float,
        by: float,
    ) -> float:
        dx = bx - ax
        dy = by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 1.0e-12:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
        cx = ax + t * dx
        cy = ay + t * dy
        return math.hypot(px - cx, py - cy)

    def nearest_walkable_cell(
        self,
        x: float,
        y: float,
    ) -> Optional[tuple[int, int]]:
        if not self.astar_walkable_cells:
            return None

        base = self.world_to_cell(x, y)
        if base in self.astar_walkable_cells:
            return base

        max_radius_cells = max(1, int(math.ceil(self.astar_snap_radius / self.astar_resolution)))
        best_cell: Optional[tuple[int, int]] = None
        best_dist_sq = float("inf")
        for radius in range(1, max_radius_cells + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    candidate = (base[0] + dx, base[1] + dy)
                    if candidate not in self.astar_walkable_cells:
                        continue
                    wx, wy = self.cell_to_world(candidate)
                    dist_sq = (wx - x) * (wx - x) + (wy - y) * (wy - y)
                    if dist_sq < best_dist_sq:
                        best_dist_sq = dist_sq
                        best_cell = candidate
            if best_cell is not None:
                return best_cell
        return None

    def astar_neighbors(self, cell: tuple[int, int]) -> list[tuple[tuple[int, int], float]]:
        steps = [
            ((1, 0), 1.0),
            ((-1, 0), 1.0),
            ((0, 1), 1.0),
            ((0, -1), 1.0),
        ]
        if self.astar_allow_diagonal:
            diag_cost = math.sqrt(2.0)
            steps.extend([
                ((1, 1), diag_cost),
                ((1, -1), diag_cost),
                ((-1, 1), diag_cost),
                ((-1, -1), diag_cost),
            ])

        neighbors: list[tuple[tuple[int, int], float]] = []
        for (dx, dy), cost in steps:
            candidate = (cell[0] + dx, cell[1] + dy)
            if candidate not in self.astar_walkable_cells:
                continue
            if dx != 0 and dy != 0:
                if (
                    (cell[0] + dx, cell[1]) not in self.astar_walkable_cells
                    or (cell[0], cell[1] + dy) not in self.astar_walkable_cells
                ):
                    continue
            neighbors.append((candidate, cost))
        return neighbors

    @staticmethod
    def reconstruct_cell_path(
        parents: dict[tuple[int, int], tuple[int, int]],
        goal_cell: tuple[int, int],
    ) -> list[tuple[int, int]]:
        path = [goal_cell]
        current = goal_cell
        while current in parents:
            current = parents[current]
            path.append(current)
        path.reverse()
        return path

    def simplify_cell_path(self, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
        if len(cells) <= 2:
            return cells
        simplified = [cells[0]]
        prev_dir: Optional[tuple[int, int]] = None
        for index in range(1, len(cells)):
            dx = cells[index][0] - cells[index - 1][0]
            dy = cells[index][1] - cells[index - 1][1]
            step = (
                0 if dx == 0 else int(dx / abs(dx)),
                0 if dy == 0 else int(dy / abs(dy)),
            )
            if prev_dir is None:
                prev_dir = step
                continue
            if step != prev_dir:
                simplified.append(cells[index - 1])
                prev_dir = step
        simplified.append(cells[-1])
        return simplified

    def cells_line_of_sight(self, start: tuple[int, int], end: tuple[int, int]) -> bool:
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 >= x0 else -1
        sy = 1 if y1 >= y0 else -1
        err = dx - dy
        x = x0
        y = y0
        while True:
            if (x, y) not in self.astar_walkable_cells:
                return False
            if x == x1 and y == y1:
                return True
            e2 = 2 * err
            next_x = x
            next_y = y
            if e2 > -dy:
                err -= dy
                next_x += sx
            if e2 < dx:
                err += dx
                next_y += sy
            if next_x != x and next_y != y:
                if (
                    (next_x, y) not in self.astar_walkable_cells
                    or (x, next_y) not in self.astar_walkable_cells
                ):
                    return False
            x = next_x
            y = next_y

    def shortcut_cell_path(self, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
        if len(cells) <= 2:
            return cells
        shortened = [cells[0]]
        anchor_index = 0
        while anchor_index < len(cells) - 1:
            best_index = anchor_index + 1
            for candidate_index in range(len(cells) - 1, anchor_index, -1):
                if self.cells_line_of_sight(cells[anchor_index], cells[candidate_index]):
                    best_index = candidate_index
                    break
            shortened.append(cells[best_index])
            anchor_index = best_index
        return shortened

    def blend_world_path(self, points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        if not self.astar_smooth_enabled or len(points) <= 2 or self.astar_corner_blend_dist <= 1.0e-6:
            return points

        smoothed: list[tuple[float, float]] = [points[0]]
        for index in range(1, len(points) - 1):
            ax, ay = points[index - 1]
            bx, by = points[index]
            cx, cy = points[index + 1]
            in_dx = bx - ax
            in_dy = by - ay
            out_dx = cx - bx
            out_dy = cy - by
            in_len = math.hypot(in_dx, in_dy)
            out_len = math.hypot(out_dx, out_dy)
            if in_len <= 1.0e-6 or out_len <= 1.0e-6:
                smoothed.append((bx, by))
                continue
            in_ux = in_dx / in_len
            in_uy = in_dy / in_len
            out_ux = out_dx / out_len
            out_uy = out_dy / out_len
            turn_measure = in_ux * out_ux + in_uy * out_uy
            if turn_measure > 0.98:
                smoothed.append((bx, by))
                continue
            blend = min(self.astar_corner_blend_dist, in_len * 0.35, out_len * 0.35)
            if blend <= 1.0e-6:
                smoothed.append((bx, by))
                continue
            pre = (bx - in_ux * blend, by - in_uy * blend)
            post = (bx + out_ux * blend, by + out_uy * blend)
            if math.hypot(pre[0] - smoothed[-1][0], pre[1] - smoothed[-1][1]) > 1.0e-6:
                smoothed.append(pre)
            smoothed.append((bx, by))
            if math.hypot(post[0] - bx, post[1] - by) > 1.0e-6:
                smoothed.append(post)
        smoothed.append(points[-1])
        return smoothed

    def compute_astar_path(
        self,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
    ) -> list[tuple[float, float]]:
        if not self.astar_enabled or not self.astar_walkable_cells:
            return []
        if self.point_blocked_by_avoid_regions(start_xy[0], start_xy[1]):
            return []
        if self.point_blocked_by_avoid_regions(goal_xy[0], goal_xy[1]):
            return []

        start_cell = self.nearest_walkable_cell(start_xy[0], start_xy[1])
        goal_cell = self.nearest_walkable_cell(goal_xy[0], goal_xy[1])
        if start_cell is None or goal_cell is None:
            return []
        if start_cell == goal_cell:
            return [start_xy, goal_xy]

        open_heap: list[tuple[float, float, tuple[int, int]]] = []
        parents: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], float] = {start_cell: 0.0}
        goal_x, goal_y = goal_cell
        start_h = math.hypot(goal_x - start_cell[0], goal_y - start_cell[1])
        heapq.heappush(open_heap, (start_h, 0.0, start_cell))
        expansions = 0

        while open_heap and expansions < self.astar_max_expansions:
            _, current_cost, current = heapq.heappop(open_heap)
            best_cost = g_score.get(current)
            if best_cost is None or current_cost > best_cost + 1.0e-9:
                continue
            if current == goal_cell:
                cell_path = self.reconstruct_cell_path(parents, goal_cell)
                cell_path = self.simplify_cell_path(cell_path)
                cell_path = self.shortcut_cell_path(cell_path)
                world_path = [start_xy]
                for cell in cell_path[1:-1]:
                    world_path.append(self.cell_to_world(cell))
                world_path.append(goal_xy)
                return self.blend_world_path(world_path)

            expansions += 1
            for neighbor, step_cost in self.astar_neighbors(current):
                tentative_cost = current_cost + step_cost
                if tentative_cost >= g_score.get(neighbor, float("inf")) - 1.0e-9:
                    continue
                parents[neighbor] = current
                g_score[neighbor] = tentative_cost
                heuristic = math.hypot(goal_x - neighbor[0], goal_y - neighbor[1])
                heapq.heappush(open_heap, (tentative_cost + heuristic, tentative_cost, neighbor))

        return []

    def reload_navigation_data(self) -> None:
        goals = self._load_goals(self.goals_file)
        missions = self._load_missions(self.missions_file)
        route_source_file = self.get_route_source_file()
        route_goals, route_missions, route_default, route_alignment, route_regions = self._load_route(route_source_file)
        goals.update(route_goals)
        missions.update(route_missions)
        self.goals = goals
        self.missions = missions
        self.default_mission_name = route_default or (next(iter(missions)) if missions else None)
        self.route_alignment_info = route_alignment
        self.route_avoid_regions = route_regions
        self.astar_walkable_cells, self.astar_grid_info = self.build_astar_grid(self.astar_source_points)
        self.rebuild_task_indexes()

    def rebuild_task_indexes(self) -> None:
        self.task_goal_names = {}
        self.task_order = []
        self.waypoint_id_goal_names = {}

        mission_name = self.default_mission_name
        mission_goals = self.missions.get(mission_name, []) if mission_name else []
        if not mission_goals:
            return

        for goal_name in mission_goals:
            goal = self.goals.get(goal_name)
            if goal is None:
                continue
            if goal.waypoint_id is not None and goal.waypoint_id not in self.waypoint_id_goal_names:
                self.waypoint_id_goal_names[goal.waypoint_id] = goal_name
            if not goal.task:
                continue
            if goal.task not in self.task_goal_names:
                self.task_goal_names[goal.task] = []
                self.task_order.append(goal.task)
            self.task_goal_names[goal.task].append(goal_name)

    def get_route_source_file(self) -> str:
        candidate = self.route_task_file.strip() if self.route_task_file else ""
        if candidate:
            return candidate
        return self.route_file

    def _load_route(
        self, path_value: str
    ) -> tuple[dict[str, GoalSpec], dict[str, list[str]], Optional[str], dict[str, Any], list[AvoidRegion]]:
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
                    waypoint_task = self._normalize_task(waypoint.get("task", waypoint.get("task_id")))
                    segment_task = self._normalize_task(segment.get("task", segment.get("obstacle", "")))
                    if waypoint_task is None and self.is_slalom_task(segment_task):
                        waypoint_task = segment_task
                    waypoint_tolerance = max(0.01, float(waypoint.get("tolerance", self.goal_tolerance)))
                    waypoint_speed = float(waypoint["speed"]) if waypoint.get("speed") is not None else None
                    waypoint_policy = self._normalize_policy(waypoint.get("policy"))
                    waypoint_precision_value = self._get_bool(waypoint, "precision_follow", "precisionFollow")
                    waypoint_precision_follow = bool(waypoint_precision_value)
                    waypoint_stable_cycles = self._get_int(waypoint, "stable_cycles", "stableCycles")
                    waypoint_lookahead = self._get_float(waypoint, "lookahead", "lookAhead")
                    waypoint_yaw_rate_limit = self._get_float(waypoint, "yaw_rate_limit", "yawRateLimit")
                    waypoint_slalom_script_pos_tolerance = self._get_float(
                        waypoint,
                        "slalom_script_pos_tolerance",
                        "slalomScriptPosTolerance",
                        "scriptTolerance",
                    )
                    waypoint_slalom_straight = bool(
                        self._get_bool(waypoint, "slalom_straight", "slalomStraight")
                    )
                    script_controls_waypoint = bool(
                        self.slalom_script_enabled and waypoint_slalom_straight
                    )
                    if self.should_auto_precision_for_task(waypoint_task) and not script_controls_waypoint:
                        if self.slalom_auto_precision_force or waypoint_precision_value is None:
                            waypoint_precision_follow = True
                        waypoint_require_yaw = False
                        waypoint_tolerance = max(waypoint_tolerance, self.slalom_tolerance)
                        if waypoint_speed is None:
                            waypoint_speed = self.slalom_max_vx if self.slalom_max_vx > 0.0 else self.max_vx
                        elif self.slalom_max_vx > 0.0:
                            waypoint_speed = min(waypoint_speed, self.slalom_max_vx)
                        if self.slalom_auto_precision_force or waypoint_stable_cycles is None:
                            waypoint_stable_cycles = self.slalom_stable_cycles
                        if self.slalom_auto_precision_force or waypoint_lookahead is None:
                            waypoint_lookahead = self.slalom_lookahead
                        if self.slalom_auto_precision_force or waypoint_yaw_rate_limit is None:
                            waypoint_yaw_rate_limit = self.slalom_yaw_rate_limit
                    raw_waypoints.append(
                        {
                            "segment": segment_name,
                            "id": int(waypoint.get("id", waypoint_index)),
                            "task": waypoint_task,
                            "x": float(waypoint["x"]),
                            "y": float(waypoint["y"]),
                            "yaw": math.radians(float(waypoint_yaw_deg)) if waypoint_yaw_deg is not None else None,
                            "yaw_tolerance": (
                                math.radians(max(0.0, waypoint_yaw_tolerance_deg))
                                if waypoint_yaw_tolerance_deg is not None else None
                            ),
                            "tolerance": waypoint_tolerance,
                            "speed": waypoint_speed,
                            "policy": waypoint_policy,
                            "require_yaw": bool(waypoint_require_yaw) and waypoint_yaw_deg is not None,
                            "pre_dock_distance": waypoint_pre_dock_distance,
                            "pre_dock_tolerance": waypoint_pre_dock_tolerance,
                            "precision_follow": waypoint_precision_follow,
                            "stable_cycles": waypoint_stable_cycles,
                            "lookahead": waypoint_lookahead,
                            "yaw_rate_limit": waypoint_yaw_rate_limit,
                            "slalom_straight": waypoint_slalom_straight,
                            "slalom_script_pos_tolerance": waypoint_slalom_script_pos_tolerance,
                        }
                    )
                except (KeyError, TypeError, ValueError):
                    continue

        if not raw_waypoints:
            self.get_logger().warn(f"Route file has no usable waypoints: {path_value}")
            return {}, {}, None, {}, []

        aligned_waypoints, alignment_info = self._align_route_waypoints(raw_waypoints)
        applied_angle_rad = math.radians(float(alignment_info.get("applied_deg", 0.0))) if alignment_info else 0.0
        aligned_regions = self._rotate_avoid_regions(
            self._load_avoid_regions(data),
            float(raw_waypoints[0]["x"]),
            float(raw_waypoints[0]["y"]),
            applied_angle_rad,
        )
        goals: dict[str, GoalSpec] = {}
        mission_goals: list[str] = []
        for index, waypoint in enumerate(aligned_waypoints, start=1):
            goal_name = f"{route_name}_p{index:02d}"
            goals[goal_name] = GoalSpec(
                name=goal_name,
                x=float(waypoint["x"]),
                y=float(waypoint["y"]),
                waypoint_id=int(waypoint["id"]) if waypoint.get("id") is not None else None,
                task=self._normalize_task(waypoint.get("task")),
                segment=str(waypoint.get("segment", "") or "") or None,
                yaw=waypoint.get("yaw"),
                yaw_tolerance=waypoint.get("yaw_tolerance"),
                tolerance=float(waypoint.get("tolerance", self.goal_tolerance)),
                speed=waypoint.get("speed"),
                policy=self._normalize_policy(waypoint.get("policy")),
                require_yaw=bool(waypoint.get("require_yaw", False)),
                pre_dock_distance=waypoint.get("pre_dock_distance"),
                pre_dock_tolerance=waypoint.get("pre_dock_tolerance"),
                precision_follow=bool(waypoint.get("precision_follow", False)),
                stable_cycles=waypoint.get("stable_cycles"),
                lookahead=waypoint.get("lookahead"),
                yaw_rate_limit=waypoint.get("yaw_rate_limit"),
                slalom_straight=bool(waypoint.get("slalom_straight", False)),
                slalom_script_pos_tolerance=waypoint.get("slalom_script_pos_tolerance"),
            )
            mission_goals.append(goal_name)

        if alignment_info:
            applied = float(alignment_info.get("applied_deg", 0.0))
            hits = int(alignment_info.get("hits", 0))
            total = int(alignment_info.get("total", 0))
            reason = str(alignment_info.get("reason", "")).strip()
            reason_text = f" reason={reason}" if reason else ""
            self.get_logger().info(
                f"Loaded route {route_name}: {len(mission_goals)} waypoints, "
                f"avoid_regions={len(aligned_regions)}, alignment={applied:.2f}deg hits={hits}/{total}{reason_text}"
            )
        else:
            self.get_logger().info(
                f"Loaded route {route_name}: {len(mission_goals)} waypoints, avoid_regions={len(aligned_regions)}"
            )

        return goals, {route_name: mission_goals}, route_name, alignment_info, aligned_regions

    def _load_avoid_regions(self, data: dict[str, Any]) -> list[AvoidRegion]:
        rows = data.get("regions")
        if not isinstance(rows, list) or not rows:
            rows = data.get("avoid_regions", [])
        if not isinstance(rows, list):
            return []

        regions: list[AvoidRegion] = []
        for index, row in enumerate(rows, start=1):
            if not isinstance(row, dict):
                continue
            kind = str(row.get("kind", "avoid")).strip().lower()
            if kind not in {"avoid", "no_go", "no-go", "nogo", "forbidden", "blocked"}:
                continue
            polygon_rows = row.get("polygon", [])
            if not isinstance(polygon_rows, list):
                continue
            points: list[tuple[float, float]] = []
            for point in polygon_rows:
                try:
                    if isinstance(point, dict):
                        points.append((float(point["x"]), float(point["y"])))
                    elif isinstance(point, (list, tuple)) and len(point) >= 2:
                        points.append((float(point[0]), float(point[1])))
                except (KeyError, TypeError, ValueError):
                    continue
            if len(points) >= 3:
                regions.append(
                    AvoidRegion(
                        name=str(row.get("name", f"avoid_{index}")),
                        kind=kind,
                        polygon=tuple(points),
                    )
                )
        return regions

    def _rotate_avoid_regions(
        self,
        regions: list[AvoidRegion],
        anchor_x: float,
        anchor_y: float,
        angle_rad: float,
    ) -> list[AvoidRegion]:
        if not regions or abs(angle_rad) <= 1.0e-12:
            return regions
        rotated: list[AvoidRegion] = []
        for region in regions:
            rotated.append(
                AvoidRegion(
                    name=region.name,
                    kind=region.kind,
                    polygon=tuple(
                        self._rotate_xy(x, y, anchor_x, anchor_y, angle_rad)
                        for x, y in region.polygon
                    ),
                )
            )
        return rotated

    def _align_route_waypoints(
        self, waypoints: list[dict[str, Any]]
    ) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        if not waypoints:
            return [], {}

        if not self.pcd_points:
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

        steps = max(1, int(round((self.route_align_max_angle_deg * 2.0) / self.route_align_angle_step_deg)))
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
                for px, py in self.pcd_points:
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
            if updated.get("yaw") is not None:
                updated["yaw"] = self.normalize_angle(float(updated["yaw"]) + angle_rad)
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
                yaw = math.radians(float(parts[3])) if len(parts) >= 4 else None
                self.start_direct_goal(float(parts[1]), float(parts[2]), f"direct({parts[1]},{parts[2]})", yaw=yaw)
            except ValueError:
                self.publish_status("go expects numeric x y [yaw_deg]")
        elif op == "go_rel" and len(parts) >= 3:
            try:
                self.start_relative_goal(float(parts[1]), float(parts[2]))
            except ValueError:
                self.publish_status("go_rel expects numeric dx dy")
        elif op == "run":
            mission_name = parts[1] if len(parts) >= 2 else (self.default_mission_name or "")
            if mission_name:
                self.start_mission(mission_name)
            else:
                self.publish_status("run expects a mission name and no default mission is configured")
        elif op in {"run_from_task", "resume_task", "restart_task"} and len(parts) >= 2:
            self.start_mission_from_task(parts[1])
        elif op == "run_only_task" and len(parts) >= 2:
            self.start_task_only(parts[1])
        elif op in {"run_from_id", "resume_id"} and len(parts) >= 2:
            try:
                self.start_mission_from_waypoint_id(int(parts[1]))
            except ValueError:
                self.publish_status("run_from_id expects a numeric waypoint id")
        elif op == "skip_task":
            task_name = parts[1] if len(parts) >= 2 else None
            self.skip_task(task_name)
        elif op == "reload":
            self.reload_navigation_data()
            self.publish_status(
                f"reloaded goals={len(self.goals)} missions={len(self.missions)} "
                f"default={self.default_mission_name or 'none'} "
                f"tasks={len(self.task_order)} "
                f"avoid_regions={len(self.route_avoid_regions)} "
                f"astar_cells={len(self.astar_walkable_cells)}"
            )
        else:
            self.publish_status(f"unknown or incomplete command: {command}")

    def get_default_mission_goals(self) -> list[str]:
        if not self.default_mission_name:
            return []
        return list(self.missions.get(self.default_mission_name, []))

    def mission_index_for_goal(self, goal_name: str) -> Optional[int]:
        mission_goals = self.get_default_mission_goals()
        try:
            return mission_goals.index(goal_name)
        except ValueError:
            return None

    def task_start_index(self, task_name: str) -> Optional[int]:
        task_key = self._normalize_task(task_name)
        if not task_key:
            return None
        task_goals = self.task_goal_names.get(task_key)
        if not task_goals:
            return None
        return self.mission_index_for_goal(task_goals[0])

    def task_end_index(self, task_name: str) -> Optional[int]:
        task_key = self._normalize_task(task_name)
        if not task_key:
            return None
        task_goals = self.task_goal_names.get(task_key)
        if not task_goals:
            return None
        return self.mission_index_for_goal(task_goals[-1])

    def current_task_name(self) -> Optional[str]:
        if self.active_goal is not None and self.active_goal.task:
            return self.active_goal.task
        if self.active_mission_name and 0 <= self.active_mission_index < len(self.active_mission_goals):
            goal = self.goals.get(self.active_mission_goals[self.active_mission_index])
            if goal is not None:
                return goal.task
        return None

    def start_default_mission_from_index(self, start_index: int, reason: str) -> None:
        mission_name = self.default_mission_name
        mission_goals = self.get_default_mission_goals()
        if not mission_name or not mission_goals:
            self.publish_status("resume failed: no default mission loaded")
            return
        if start_index < 0 or start_index >= len(mission_goals):
            self.publish_status(f"resume failed: waypoint index {start_index + 1} out of range")
            return
        self.active_mission_name = mission_name
        self.active_mission_goals = mission_goals
        self.active_mission_index = start_index
        goal_name = mission_goals[start_index]
        goal = self.goals.get(goal_name)
        detail = ""
        if goal is not None:
            id_text = f"id={goal.waypoint_id}" if goal.waypoint_id is not None else goal_name
            task_text = f" task={goal.task}" if goal.task else ""
            detail = f" at {id_text}{task_text}"
        self.publish_status(
            f"{reason}: resume default mission from waypoint {start_index + 1}/{len(mission_goals)}{detail}"
        )
        self._activate_mission_goal()

    def start_mission_from_task(self, task_name: str) -> None:
        task_key = self._normalize_task(task_name)
        if not task_key:
            self.publish_status("run_from_task expects a task name")
            return
        start_index = self.task_start_index(task_key)
        if start_index is None:
            known = ",".join(self.task_order) if self.task_order else "none"
            self.publish_status(f"task not found: {task_key} known={known}")
            return
        self.start_default_mission_from_index(start_index, f"task resume {task_key}")

    def start_task_only(self, task_name: str) -> None:
        task_key = self._normalize_task(task_name)
        if not task_key:
            self.publish_status("run_only_task expects a task name")
            return
        task_goals = self.task_goal_names.get(task_key)
        if not task_goals:
            known = ",".join(self.task_order) if self.task_order else "none"
            self.publish_status(f"task not found: {task_key} known={known}")
            return
        self.active_mission_name = f"{self.default_mission_name or 'route'}:{task_key}"
        self.active_mission_goals = list(task_goals)
        self.active_mission_index = 0
        self.publish_status(f"task-only run {task_key}: {len(task_goals)} waypoints")
        self._activate_mission_goal()

    def start_mission_from_waypoint_id(self, waypoint_id: int) -> None:
        goal_name = self.waypoint_id_goal_names.get(waypoint_id)
        if goal_name is None:
            known = ",".join(str(item) for item in sorted(self.waypoint_id_goal_names.keys()))
            self.publish_status(f"waypoint id not found: {waypoint_id} known={known or 'none'}")
            return
        start_index = self.mission_index_for_goal(goal_name)
        if start_index is None:
            self.publish_status(f"waypoint id {waypoint_id} is not in default mission")
            return
        self.start_default_mission_from_index(start_index, f"id resume {waypoint_id}")

    def skip_task(self, task_name: Optional[str]) -> None:
        task_key = self._normalize_task(task_name) if task_name else self.current_task_name()
        if not task_key:
            self.publish_status("skip_task failed: no task specified and no active task")
            return
        if task_key not in self.task_goal_names:
            known = ",".join(self.task_order) if self.task_order else "none"
            self.publish_status(f"skip_task failed: task not found {task_key} known={known}")
            return
        try:
            task_order_index = self.task_order.index(task_key)
        except ValueError:
            self.publish_status(f"skip_task failed: task not indexed {task_key}")
            return
        if task_order_index + 1 >= len(self.task_order):
            self.cancel_navigation(f"skipped final task {task_key}; no later task")
            return
        next_task = self.task_order[task_order_index + 1]
        next_index = self.task_start_index(next_task)
        if next_index is None:
            self.publish_status(f"skip_task failed: next task has no start index {next_task}")
            return
        self.start_default_mission_from_index(next_index, f"skip task {task_key} -> {next_task}")

    def start_goal(self, goal_name: str) -> None:
        goal = self.goals.get(goal_name)
        if goal is None:
            self.publish_status(f"goal not found: {goal_name}")
            return
        self.active_mission_name = None
        self.active_mission_goals = []
        self.active_mission_index = 0
        self.start_direct_goal(
            goal.x,
            goal.y,
            goal_name,
            yaw=goal.yaw,
            yaw_tolerance=goal.yaw_tolerance,
            tolerance=goal.tolerance,
            speed=goal.speed,
            policy=goal.policy,
            require_yaw=goal.require_yaw,
            pre_dock_distance=goal.pre_dock_distance,
            pre_dock_tolerance=goal.pre_dock_tolerance,
            precision_follow=goal.precision_follow,
            stable_cycles=goal.stable_cycles,
            lookahead=goal.lookahead,
            yaw_rate_limit=goal.yaw_rate_limit,
            slalom_straight=goal.slalom_straight,
        )

    def start_direct_goal(
        self,
        x: float,
        y: float,
        goal_name: str,
        yaw: Optional[float] = None,
        yaw_tolerance: Optional[float] = None,
        tolerance: Optional[float] = None,
        speed: Optional[float] = None,
        policy: Optional[str] = None,
        require_yaw: Optional[bool] = None,
        pre_dock_distance: Optional[float] = None,
        pre_dock_tolerance: Optional[float] = None,
        precision_follow: bool = False,
        stable_cycles: Optional[int] = None,
        lookahead: Optional[float] = None,
        yaw_rate_limit: Optional[float] = None,
        slalom_straight: bool = False,
    ) -> None:
        goal = GoalSpec(
            name=goal_name,
            x=x,
            y=y,
            yaw=yaw,
            yaw_tolerance=max(0.0, yaw_tolerance) if yaw_tolerance is not None else None,
            tolerance=max(0.01, tolerance if tolerance is not None else self.goal_tolerance),
            speed=speed,
            policy=self._normalize_policy(policy),
            require_yaw=bool(yaw is not None if require_yaw is None else require_yaw) and yaw is not None,
            pre_dock_distance=pre_dock_distance,
            pre_dock_tolerance=pre_dock_tolerance,
            precision_follow=bool(precision_follow),
            stable_cycles=stable_cycles,
            lookahead=max(0.01, lookahead) if lookahead is not None else None,
            yaw_rate_limit=max(0.01, yaw_rate_limit) if yaw_rate_limit is not None else None,
            slalom_straight=bool(slalom_straight),
        )
        stage_note = self.start_goal_execution(goal)
        suffix = self.describe_goal(goal)
        self.publish_status(
            f"nav target set in {self.map_frame}: {goal_name} -> ({x:.2f}, {y:.2f}) {' '.join(suffix)}"
        )
        if stage_note:
            self.publish_status(stage_note)

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
        stage_note = self.start_goal_execution(goal)
        suffix = self.describe_goal(goal)
        self.publish_status(
            f"mission {self.active_mission_name}: waypoint {self.active_mission_index + 1}/{len(self.active_mission_goals)} -> {goal_name} {' '.join(suffix)}"
        )
        if stage_note:
            self.publish_status(stage_note)

    def record_current_pose(self, name: str) -> None:
        pose = self.lookup_pose()
        if pose is None:
            self.publish_status("record failed: pose unavailable")
            return
        x, y, yaw = pose
        text = f"{name}: frame={self.map_frame}, x={x:.3f}, y={y:.3f}, yaw_deg={math.degrees(yaw):.1f}"
        self.record_pub.publish(String(data=text))
        self.publish_status(f"recorded {text}")

    def activate_goal(self, goal: GoalSpec, stage: str = "final") -> None:
        self.active_goal = goal
        self.active_goal_stage = stage
        self.active_path_world = []
        self.active_path_index = 0
        self.active_path_goal_key = None
        self.active_precision_segment_start = None
        self.active_precision_segment_end = None
        self.last_published_path_signature = None
        self.turn_in_place_mode = False
        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0
        self.pose_waiting_reported = False
        self.reset_slalom_script()
        self.request_policy(goal.policy)
        self.publish_goal_pose(goal)

    def reset_slalom_script(self) -> None:
        self.slalom_script_active = False
        self.slalom_script_key = None
        self.slalom_script_steps = []
        self.slalom_script_step_index = 0
        self.slalom_script_step_stable_count = 0

    def publish_active_path(self, force: bool = False) -> None:
        if self.active_goal is None:
            payload = {
                "goal_name": "",
                "stage": self.active_goal_stage,
                "path_index": 0,
                "points": [],
            }
            self.path_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
            self.last_published_path_signature = None
            return

        signature = (
            self.active_goal.name,
            self.active_goal_stage,
            len(self.active_path_world),
            self.active_path_index,
        )
        if not force and signature == self.last_published_path_signature:
            return

        payload = {
            "goal_name": self.active_goal.name,
            "stage": self.active_goal_stage,
            "path_index": self.active_path_index,
            "points": [
                [round(float(x), 3), round(float(y), 3)]
                for x, y in self.active_path_world
            ],
        }
        self.path_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        self.last_published_path_signature = signature

    def ensure_active_path(
        self,
        pose_xy: tuple[float, float],
        goal: GoalSpec,
    ) -> bool:
        if (
            goal.precision_follow
            and self.active_mission_name is not None
            and self.active_goal_stage == "final"
        ):
            start_index = self.active_precision_segment_start
            end_index = self.active_precision_segment_end
            if (
                start_index is None
                or end_index is None
                or self.active_mission_index < start_index
                or self.active_mission_index > end_index
            ):
                start_index, end_index = self.get_precision_segment_bounds(
                    self.active_mission_index,
                    goal,
                )
                self.active_precision_segment_start = start_index
                self.active_precision_segment_end = end_index

            if end_index > start_index:
                goal_key = ("precision_segment", self.active_mission_name, start_index, end_index)
                if self.active_path_goal_key == goal_key and self.active_path_world:
                    offset = max(0, min(self.active_mission_index - start_index, len(self.active_path_world) - 1))
                    self.active_path_index = max(self.active_path_index, offset)
                    return True

                path_world = self.build_precision_segment_path(start_index, end_index)
                if len(path_world) >= 2:
                    self.active_path_world = path_world
                    self.active_path_index = max(0, min(self.active_mission_index - start_index, len(path_world) - 1))
                    self.active_path_goal_key = goal_key
                    self.publish_active_path(force=True)
                    self.publish_status(
                        f"precision segment planned: waypoints {start_index + 1}-{end_index + 1} "
                        f"({len(path_world)} points)"
                    )
                    return True

        if not self.astar_enabled or not self.astar_walkable_cells:
            return False

        goal_key = (goal.name, self.active_goal_stage)
        if self.active_path_goal_key == goal_key and self.active_path_world:
            return True

        path_world = self.compute_astar_path(pose_xy, (goal.x, goal.y))
        if len(path_world) < 2:
            self.active_path_world = []
            self.active_path_index = 0
            self.active_path_goal_key = None
            return False

        self.active_path_world = path_world
        self.active_path_index = 1
        self.active_path_goal_key = goal_key
        self.publish_active_path(force=True)
        self.publish_status(
            f"path planned for {goal.name}: {len(path_world)} points via A*"
        )
        return True

    def get_path_follow_target(
        self,
        pose_xy: tuple[float, float],
        goal: GoalSpec,
    ) -> Optional[tuple[float, float]]:
        if goal.slalom_straight:
            self.active_path_world = []
            self.active_path_index = 0
            self.active_path_goal_key = None
            return goal.x, goal.y

        if not self.ensure_active_path(pose_xy, goal):
            if self.astar_enabled and self.astar_walkable_cells:
                return None
            return goal.x, goal.y

        rx, ry = pose_xy
        lookahead_dist = self.get_goal_lookahead(goal)
        advance_dist = max(self.astar_waypoint_reach_dist, min(goal.tolerance, lookahead_dist))
        while self.active_path_index < len(self.active_path_world) - 1:
            tx, ty = self.active_path_world[self.active_path_index]
            if math.hypot(tx - rx, ty - ry) <= advance_dist:
                self.active_path_index += 1
                continue
            break

        target_index = self.active_path_index
        while target_index < len(self.active_path_world) - 1:
            next_index = target_index + 1
            tx, ty = self.active_path_world[next_index]
            if math.hypot(tx - rx, ty - ry) > lookahead_dist:
                break
            target_index = next_index
        previous_index = self.active_path_index
        self.active_path_index = max(self.active_path_index, target_index)
        if self.active_path_index != previous_index:
            self.publish_active_path(force=False)
        return self.active_path_world[self.active_path_index]

    def goal_requires_yaw(self, goal: GoalSpec) -> bool:
        return bool(goal.require_yaw and goal.yaw is not None)

    def get_goal_stable_cycles(self, goal: GoalSpec) -> int:
        if goal.stable_cycles is None:
            return self.goal_complete_stable_cycles
        return max(0, int(goal.stable_cycles))

    def get_goal_lookahead(self, goal: GoalSpec) -> float:
        if goal.lookahead is None:
            return self.astar_lookahead_dist
        return max(self.astar_waypoint_reach_dist, float(goal.lookahead))

    def get_goal_yaw_rate_limit(self, goal: GoalSpec) -> float:
        if goal.yaw_rate_limit is None:
            limit = self.max_wz
        else:
            limit = max(0.01, min(self.max_wz, float(goal.yaw_rate_limit)))
        if goal.slalom_straight:
            limit = min(limit, self.slalom_script_max_wz)
        return limit

    def precision_segment_goal_compatible(self, goal: GoalSpec, reference: GoalSpec) -> bool:
        return bool(
            goal.precision_follow
            and not self.goal_requires_yaw(goal)
            and goal.policy == reference.policy
        )

    def get_precision_segment_bounds(self, start_index: int, reference: GoalSpec) -> tuple[int, int]:
        if self.active_mission_name is None:
            return start_index, start_index
        if start_index < 0 or start_index >= len(self.active_mission_goals):
            return start_index, start_index
        if not self.precision_segment_goal_compatible(reference, reference):
            return start_index, start_index

        end_index = start_index
        while end_index + 1 < len(self.active_mission_goals):
            next_name = self.active_mission_goals[end_index + 1]
            next_goal = self.goals.get(next_name)
            if next_goal is None or not self.precision_segment_goal_compatible(next_goal, reference):
                break
            end_index += 1
        return start_index, end_index

    def build_precision_segment_path(self, start_index: int, end_index: int) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        for index in range(start_index, end_index + 1):
            goal_name = self.active_mission_goals[index]
            goal = self.goals.get(goal_name)
            if goal is None:
                continue
            points.append((goal.x, goal.y))
        return points

    def should_continue_precision_goal(self, goal: GoalSpec) -> bool:
        if not (
            goal.precision_follow
            and not self.goal_requires_yaw(goal)
            and self.active_mission_name is not None
            and self.active_mission_index + 1 < len(self.active_mission_goals)
        ):
            return False
        next_name = self.active_mission_goals[self.active_mission_index + 1]
        next_goal = self.goals.get(next_name)
        return bool(next_goal is not None and self.precision_segment_goal_compatible(next_goal, goal))

    def advance_precision_goal_without_stop(self) -> bool:
        goal = self.active_goal
        if goal is None or not self.should_continue_precision_goal(goal):
            return False
        reached_name = goal.name
        self.active_mission_index += 1
        next_name = self.active_mission_goals[self.active_mission_index]
        next_goal = self.goals.get(next_name)
        if next_goal is None:
            self.cancel_navigation(f"mission goal missing: {next_name}")
            return True

        self.active_goal = next_goal
        self.active_goal_stage = "final"
        self.pending_final_goal = None
        self.turn_in_place_mode = False
        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0
        self.pose_waiting_reported = False
        if self.active_precision_segment_start is not None and self.active_path_world:
            offset = max(
                0,
                min(
                    self.active_mission_index - self.active_precision_segment_start,
                    len(self.active_path_world) - 1,
                ),
            )
            self.active_path_index = max(self.active_path_index, offset)
        self.request_policy(next_goal.policy)
        self.publish_goal_pose(next_goal)
        self.publish_status(f"precision reached {reached_name}, continuing to {next_name}")
        return True

    def sync_precision_goal_to_path_progress(self) -> None:
        goal = self.active_goal
        if goal is None or not goal.precision_follow or self.active_mission_name is None:
            return
        start_index = self.active_precision_segment_start
        end_index = self.active_precision_segment_end
        if start_index is None or end_index is None or end_index <= start_index:
            return
        if not self.active_path_world:
            return
        if (
            not isinstance(self.active_path_goal_key, tuple)
            or len(self.active_path_goal_key) < 4
            or self.active_path_goal_key[0] != "precision_segment"
        ):
            return

        path_offset = max(0, min(self.active_path_index, end_index - start_index))
        target_mission_index = start_index + path_offset
        if target_mission_index <= self.active_mission_index:
            return
        if target_mission_index >= len(self.active_mission_goals):
            return

        next_name = self.active_mission_goals[target_mission_index]
        next_goal = self.goals.get(next_name)
        if next_goal is None or not self.precision_segment_goal_compatible(next_goal, goal):
            return

        previous_name = goal.name
        self.active_mission_index = target_mission_index
        self.active_goal = next_goal
        self.active_goal_stage = "final"
        self.pending_final_goal = None
        self.turn_in_place_mode = False
        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0
        self.pose_waiting_reported = False
        self.publish_goal_pose(next_goal)
        self.publish_status(f"precision path progress: {previous_name} -> {next_name}")

    def get_pre_dock_distance(self, goal: GoalSpec) -> float:
        if goal.pre_dock_distance is not None:
            return max(0.0, float(goal.pre_dock_distance))
        return self.pre_dock_distance

    def get_pre_dock_tolerance(self, goal: GoalSpec) -> float:
        base = goal.tolerance
        if goal.pre_dock_tolerance is not None:
            base = max(base, float(goal.pre_dock_tolerance))
        else:
            base = max(base, self.pre_dock_tolerance)
        return max(0.01, base)

    def describe_goal(self, goal: GoalSpec) -> list[str]:
        suffix: list[str] = []
        if goal.waypoint_id is not None:
            suffix.append(f"id={goal.waypoint_id}")
        if goal.task:
            suffix.append(f"task={goal.task}")
        if goal.policy:
            suffix.append(f"policy={goal.policy}")
        if goal.yaw is not None:
            suffix.append(f"yaw={math.degrees(goal.yaw):.1f}deg")
        if goal.yaw_tolerance is not None:
            suffix.append(f"yaw_tol={math.degrees(goal.yaw_tolerance):.1f}deg")
        suffix.append(f"tol={goal.tolerance:.2f}")
        if self.goal_requires_yaw(goal):
            suffix.append("require_yaw")
            pre_dock_distance = self.get_pre_dock_distance(goal)
            if self.pre_dock_enabled and pre_dock_distance > 0.0:
                suffix.append(f"pre_dock={pre_dock_distance:.2f}")
        if goal.precision_follow:
            suffix.append("precision_follow")
            if goal.lookahead is not None:
                suffix.append(f"lookahead={goal.lookahead:.2f}")
            if goal.yaw_rate_limit is not None:
                suffix.append(f"yaw_limit={goal.yaw_rate_limit:.2f}")
        if goal.slalom_straight:
            suffix.append("slalom_script")
        return suffix

    def plan_goal_execution(self, goal: GoalSpec) -> tuple[GoalSpec, Optional[GoalSpec], Optional[str]]:
        if goal.slalom_straight:
            return goal, None, None
        if not self.pre_dock_enabled or not self.goal_requires_yaw(goal) or goal.yaw is None:
            return goal, None, None

        pre_dock_distance = self.get_pre_dock_distance(goal)
        if pre_dock_distance <= 0.0:
            return goal, None, None

        pre_x = goal.x - math.cos(goal.yaw) * pre_dock_distance
        pre_y = goal.y - math.sin(goal.yaw) * pre_dock_distance
        pre_tol = self.get_pre_dock_tolerance(goal)
        pose = self.lookup_pose()
        if pose is not None:
            rx, ry, _ = pose
            dist_to_goal = math.hypot(goal.x - rx, goal.y - ry)
            dist_to_pre_dock = math.hypot(pre_x - rx, pre_y - ry)
            if (
                dist_to_goal <= max(goal.tolerance + 0.05, self.pre_dock_skip_within_goal_dist)
                or dist_to_pre_dock <= pre_tol
            ):
                return goal, None, None

        pre_dock_goal = GoalSpec(
            name=f"{goal.name}__predock",
            x=pre_x,
            y=pre_y,
            yaw=None,
            yaw_tolerance=None,
            tolerance=pre_tol,
            speed=goal.speed,
            policy=goal.policy,
            require_yaw=False,
            precision_follow=False,
        )
        stage_note = (
            f"pre-dock enabled for {goal.name}: stage1 -> ({pre_x:.2f}, {pre_y:.2f}) "
            f"tol={pre_tol:.2f}, stage2 keeps yaw={math.degrees(goal.yaw):.1f}deg"
        )
        return pre_dock_goal, goal, stage_note

    def start_goal_execution(self, goal: GoalSpec) -> Optional[str]:
        self.pending_final_goal = None
        next_goal, pending_final_goal, stage_note = self.plan_goal_execution(goal)
        self.pending_final_goal = pending_final_goal
        stage = "pre_dock" if pending_final_goal is not None else "final"
        self.activate_goal(next_goal, stage=stage)
        return stage_note

    def request_policy(self, policy: Optional[str]) -> None:
        normalized = self._normalize_policy(policy)
        if normalized is None:
            self.pending_policy = None
            return
        self.pending_policy = normalized
        self.maybe_publish_pending_policy(force=True)

    def on_model_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data) if msg.data else {}
        except Exception:
            return
        if not isinstance(data, dict):
            return
        current_model = self._normalize_policy(data.get("current_model"))
        if current_model is not None:
            self.current_model_policy = current_model
        self.model_switching = bool(data.get("switching", False))
        if self.pending_policy is not None and self.current_model_policy == self.pending_policy and not self.model_switching:
            self.pending_policy = None
            return
        self.maybe_publish_pending_policy(force=False)

    def maybe_publish_pending_policy(self, force: bool) -> None:
        if self.pending_policy is None:
            return
        if self.current_model_policy == self.pending_policy and not self.model_switching:
            self.pending_policy = None
            return
        if self.model_switching:
            return
        now = self.get_clock().now()
        age_s = (now - self.last_policy_request_time).nanoseconds / 1.0e9
        if not force and age_s < self.policy_retry_period_s:
            return
        self.model_cmd_pub.publish(String(data=self.pending_policy))
        self.last_policy_request_time = now

    def lookup_pose(self) -> Optional[tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        return float(t.x), float(t.y), float(yaw)

    def get_goal_max_vx(self, goal: GoalSpec) -> float:
        max_vx = self.max_vx if goal.speed is None else min(self.max_vx, max(0.0, goal.speed))
        if self.is_slalom_task(goal.task) and self.slalom_max_vx > 0.0:
            max_vx = min(max_vx, self.slalom_max_vx)
        return max(0.0, max_vx)

    def get_goal_max_vy(self, goal: GoalSpec) -> float:
        if not goal.precision_follow or not self.precision_lateral_control_enabled:
            return 0.0
        return min(self.max_vy, self.precision_lateral_max_vy)

    def slalom_script_goal_compatible(self, goal: GoalSpec) -> bool:
        return bool(goal.slalom_straight and self.active_goal_stage == "final")

    def get_slalom_script_bounds(self, start_index: int) -> tuple[int, int]:
        if self.active_mission_name is None:
            return start_index, start_index
        if start_index < 0 or start_index >= len(self.active_mission_goals):
            return start_index, start_index

        start_goal = self.goals.get(self.active_mission_goals[start_index])
        if start_goal is None or not self.slalom_script_goal_compatible(start_goal):
            return start_index, start_index

        end_index = start_index
        while end_index + 1 < len(self.active_mission_goals):
            next_goal = self.goals.get(self.active_mission_goals[end_index + 1])
            if next_goal is None or not self.slalom_script_goal_compatible(next_goal):
                break
            end_index += 1
        return start_index, end_index

    def build_slalom_script_steps(self, start_index: int, end_index: int) -> list[SlalomScriptStep]:
        goals: list[GoalSpec] = []
        for index in range(start_index, end_index + 1):
            goal = self.goals.get(self.active_mission_goals[index])
            if goal is None:
                return []
            goals.append(goal)
        if len(goals) < 2:
            return []

        steps: list[SlalomScriptStep] = []
        for offset, goal in enumerate(goals[:-1]):
            next_goal = goals[offset + 1]
            route_dx = next_goal.x - goal.x
            route_dy = next_goal.y - goal.y
            route_dist = math.hypot(route_dx, route_dy)
            route_yaw = math.atan2(route_dy, route_dx) if route_dist > 1.0e-6 else None
            target_yaw = goal.yaw
            use_segment_yaw = route_yaw is not None and route_dist >= self.slalom_script_segment_yaw_min_dist
            if self.slalom_script_drive_yaw_source == "segment" and use_segment_yaw:
                target_yaw = route_yaw
            elif next_goal.yaw is not None and (
                self.slalom_script_drive_yaw_source == "next" or not use_segment_yaw
            ):
                target_yaw = next_goal.yaw
            elif target_yaw is None:
                target_yaw = route_yaw
            if target_yaw is not None and self.slalom_script_rotate_steps_enabled:
                steps.append(
                    SlalomScriptStep(
                        kind="rotate",
                        start_index=start_index + offset,
                        end_index=start_index + offset,
                        target_x=goal.x,
                        target_y=goal.y,
                        target_yaw=target_yaw,
                        pos_tolerance=goal.slalom_script_pos_tolerance,
                    )
                )
            if route_dist >= self.slalom_script_min_step_distance:
                yaw_for_body = target_yaw if target_yaw is not None else math.atan2(route_dy, route_dx)
                forward = math.cos(yaw_for_body) * route_dx + math.sin(yaw_for_body) * route_dy
                left = -math.sin(yaw_for_body) * route_dx + math.cos(yaw_for_body) * route_dy
                steps.append(
                    SlalomScriptStep(
                        kind="drive",
                        start_index=start_index + offset,
                        end_index=start_index + offset + 1,
                        target_x=next_goal.x,
                        target_y=next_goal.y,
                        target_yaw=target_yaw,
                        pos_tolerance=next_goal.slalom_script_pos_tolerance,
                        forward=forward,
                        left=left,
                    )
                )

        final_goal = goals[-1]
        if final_goal.yaw is not None and self.slalom_script_final_rotate_enabled:
            steps.append(
                SlalomScriptStep(
                    kind="rotate",
                    start_index=end_index,
                    end_index=end_index,
                    target_x=final_goal.x,
                    target_y=final_goal.y,
                    target_yaw=final_goal.yaw,
                    pos_tolerance=final_goal.slalom_script_pos_tolerance,
                )
            )
        return steps

    def maybe_start_slalom_script(self, pose: tuple[float, float, float]) -> bool:
        goal = self.active_goal
        if (
            not self.slalom_script_enabled
            or goal is None
            or self.active_mission_name is None
            or not self.slalom_script_goal_compatible(goal)
        ):
            return False

        rx, ry, _ = pose
        start_dist = math.hypot(goal.x - rx, goal.y - ry)
        start_tolerance = self.slalom_script_start_tolerance
        if start_dist > start_tolerance:
            return False

        start_index, end_index = self.get_slalom_script_bounds(self.active_mission_index)
        if end_index <= start_index:
            return False
        steps = self.build_slalom_script_steps(start_index, end_index)
        if not steps:
            return False

        self.slalom_script_active = True
        self.slalom_script_key = (self.active_mission_name, start_index, end_index)
        self.slalom_script_steps = steps
        self.slalom_script_step_index = 0
        self.slalom_script_step_stable_count = 0
        self.turn_in_place_mode = False
        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0
        self.active_path_world = self.build_precision_segment_path(start_index, end_index)
        self.active_path_index = 0
        self.active_path_goal_key = ("slalom_script", self.active_mission_name, start_index, end_index)
        self.publish_active_path(force=True)
        self.publish_status(
            f"slalom script started: waypoints {start_index + 1}-{end_index + 1}, "
            f"steps={len(steps)}, start_dist={start_dist:.2f}"
        )
        return True

    def handle_slalom_script(self, pose: tuple[float, float, float]) -> bool:
        if not self.slalom_script_active and not self.maybe_start_slalom_script(pose):
            return False
        if not self.slalom_script_active:
            return False

        while self.slalom_script_step_index < len(self.slalom_script_steps):
            step = self.slalom_script_steps[self.slalom_script_step_index]
            cmd, complete = self.compute_slalom_script_command(step, pose)
            if complete:
                self.slalom_script_step_stable_count += 1
                if self.slalom_script_step_stable_count >= self.slalom_script_stable_cycles:
                    self.slalom_script_step_index += 1
                    self.slalom_script_step_stable_count = 0
                    self.active_path_index = min(
                        max(0, step.end_index - (self.slalom_script_key[1] if self.slalom_script_key else step.end_index)),
                        max(0, len(self.active_path_world) - 1),
                    )
                    self.publish_active_path(force=False)
                    continue
            else:
                self.slalom_script_step_stable_count = 0
            if self.slalom_script_safety_filter_enabled and step.kind == "drive":
                cmd = self.filter_slalom_script_command(
                    pose,
                    (step.target_x, step.target_y),
                    cmd,
                )
            self.cmd_pub.publish(cmd)
            return True

        self.finish_slalom_script()
        return True

    def compute_slalom_script_command(
        self,
        step: SlalomScriptStep,
        pose: tuple[float, float, float],
    ) -> tuple[Twist, bool]:
        rx, ry, ryaw = pose
        cmd = Twist()
        target_yaw = step.target_yaw

        if step.kind == "rotate":
            if target_yaw is None:
                return cmd, True
            yaw_err = self.normalize_angle(target_yaw - ryaw)
            if abs(yaw_err) <= self.slalom_script_yaw_tolerance:
                return cmd, True
            cmd.angular.z = self.clamp(
                self.slalom_script_kp_yaw * yaw_err,
                -self.slalom_script_max_wz,
                self.slalom_script_max_wz,
            )
            cmd.angular.z = self.apply_slalom_script_min_command(
                cmd.angular.z,
                self.slalom_script_min_cmd_angular,
                self.slalom_script_max_wz,
            )
            return cmd, False

        dx = step.target_x - rx
        dy = step.target_y - ry
        if target_yaw is None:
            target_yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1.0e-6 else ryaw
        yaw_err = self.normalize_angle(target_yaw - ryaw)
        pos_err = math.hypot(dx, dy)
        pos_tolerance = (
            self.slalom_script_pos_tolerance
            if step.pos_tolerance is None
            else max(0.01, float(step.pos_tolerance))
        )
        complete = (
            pos_err <= pos_tolerance
            and (
                not self.slalom_script_require_yaw_at_step
                or abs(yaw_err) <= self.slalom_script_yaw_tolerance
            )
        )
        if complete:
            return cmd, True

        if abs(yaw_err) > self.slalom_script_drive_yaw_deadband:
            cmd.angular.z = self.clamp(
                self.slalom_script_kp_yaw * yaw_err,
                -self.slalom_script_max_wz,
                self.slalom_script_max_wz,
            )
            cmd.angular.z = self.apply_slalom_script_min_command(
                cmd.angular.z,
                self.slalom_script_min_cmd_angular,
                self.slalom_script_max_wz,
            )
        if abs(yaw_err) <= min(self.slalom_script_yaw_gate, self.slalom_script_drive_yaw_deadband):
            err_forward = math.cos(ryaw) * dx + math.sin(ryaw) * dy
            err_left = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
            lateral_abs = abs(err_left)

            if lateral_abs > self.slalom_script_lateral_gate:
                cmd.linear.y = self.clamp(
                    self.slalom_script_kp_dist * err_left,
                    -self.slalom_script_max_vy,
                    self.slalom_script_max_vy,
                )
                cmd.linear.y = self.apply_slalom_script_min_command(
                    cmd.linear.y,
                    self.slalom_script_min_cmd_linear,
                    self.slalom_script_max_vy,
                )

            if lateral_abs <= self.slalom_script_lateral_gate:
                vx_limit = self.slalom_script_max_vx
            elif lateral_abs <= self.slalom_script_lateral_slow_gate:
                vx_limit = self.slalom_script_lateral_creep_vx
            else:
                vx_limit = 0.0

            if vx_limit > 0.0 and abs(err_forward) > pos_tolerance:
                raw_vx = self.clamp(
                    self.slalom_script_kp_dist * err_forward,
                    -vx_limit,
                    vx_limit,
                )
                cmd.linear.x = self.apply_slalom_script_min_command(
                    raw_vx,
                    self.slalom_script_min_cmd_linear,
                    vx_limit,
                )
        return cmd, False

    def apply_slalom_script_min_command(
        self,
        value: float,
        min_abs: float,
        max_abs: float,
    ) -> float:
        if max_abs <= 0.0 or min_abs <= 0.0:
            return value
        abs_value = abs(value)
        if abs_value < self.slalom_script_min_cmd_epsilon or abs_value >= min_abs:
            return value
        return self.clamp(math.copysign(min_abs, value), -max_abs, max_abs)

    def finish_slalom_script(self) -> None:
        key = self.slalom_script_key
        self.cmd_pub.publish(Twist())
        if key is None:
            self.reset_slalom_script()
            return

        mission_name, start_index, end_index = key
        self.publish_status(
            f"slalom script complete: waypoints {start_index + 1}-{end_index + 1}"
        )
        self.reset_slalom_script()
        if self.active_mission_name != mission_name:
            return
        self.active_mission_index = end_index + 1
        if self.active_mission_index >= len(self.active_mission_goals):
            self.cancel_navigation("mission complete")
            return
        self._activate_mission_goal()

    def get_curvature_limited_max_vx(
        self,
        goal: GoalSpec,
        max_vx: float,
        yaw_err: float,
    ) -> float:
        if (
            max_vx <= 0.0
            or not goal.precision_follow
            or not self.slalom_curvature_slowdown_enabled
        ):
            return max_vx

        turn_fraction = min(1.0, abs(yaw_err) / (math.pi * 0.5))
        if 0 < self.active_path_index < len(self.active_path_world) - 1:
            ax, ay = self.active_path_world[self.active_path_index - 1]
            bx, by = self.active_path_world[self.active_path_index]
            cx, cy = self.active_path_world[self.active_path_index + 1]
            in_yaw = math.atan2(by - ay, bx - ax)
            out_yaw = math.atan2(cy - by, cx - bx)
            turn_fraction = max(
                turn_fraction,
                min(1.0, abs(self.normalize_angle(out_yaw - in_yaw)) / (math.pi * 0.5)),
            )

        scale = 1.0 - turn_fraction * (1.0 - self.slalom_min_turn_speed_scale)
        capped = max_vx * scale
        if max_vx > self.slalom_min_vx:
            capped = max(capped, self.slalom_min_vx)
        return min(max_vx, max(0.0, capped))

    def local_planner_enabled_for_goal(self, goal: GoalSpec) -> bool:
        if goal.slalom_straight:
            return False
        if not self.local_planner_enabled:
            return False
        if not self.route_avoid_regions and not (
            self.local_planner_use_astar_grid and self.astar_walkable_cells
        ):
            return False
        task = self._normalize_task(goal.task)
        if task is not None and task in self.local_planner_task_names:
            return True
        return bool(self.local_planner_precision_enabled and goal.precision_follow)

    def point_region_distance(
        self,
        x: float,
        y: float,
        polygon: tuple[tuple[float, float], ...],
    ) -> float:
        if len(polygon) < 3:
            return float("inf")
        if self.point_in_polygon(x, y, polygon):
            return 0.0
        return min(
            self.point_segment_distance(x, y, ax, ay, bx, by)
            for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1])
        )

    def avoid_region_clearance_at(self, x: float, y: float) -> float:
        if not self.avoid_regions_enabled or not self.route_avoid_regions:
            return float("inf")
        raw_distance = min(
            self.point_region_distance(x, y, region.polygon)
            for region in self.route_avoid_regions
        )
        return raw_distance - (self.local_planner_robot_radius + self.local_planner_obstacle_margin)

    def sample_range(self, low: float, high: float, count: int) -> list[float]:
        if count <= 1 or abs(high - low) <= 1.0e-9:
            return [(low + high) * 0.5]
        return [low + (high - low) * index / (count - 1) for index in range(count)]

    def unique_values(self, values: list[float], low: float, high: float) -> list[float]:
        rounded = {
            round(self.clamp(value, low, high), 4)
            for value in values
            if math.isfinite(value)
        }
        return sorted(rounded)

    def effective_velocity_values(
        self,
        values: list[float],
        low: float,
        high: float,
        min_effective: float,
    ) -> list[float]:
        unique = self.unique_values(values, low, high)
        if min_effective <= 0.0:
            return unique
        filtered = [value for value in unique if abs(value) < 1.0e-6 or abs(value) >= min_effective]
        return filtered if filtered else [0.0]

    def make_twist(self, vx: float, vy: float, wz: float) -> Twist:
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        return cmd

    def local_candidate_score(
        self,
        pose: tuple[float, float, float],
        target_xy: tuple[float, float],
        candidate: Twist,
        nominal: Twist,
        precision_follow: bool,
    ) -> Optional[float]:
        rx, ry, ryaw = pose
        target_x, target_y = target_xy
        start_dist = math.hypot(target_x - rx, target_y - ry)
        start_clearance = self.avoid_region_clearance_at(rx, ry)
        min_clearance = float("inf")
        steps = max(1, int(math.ceil(self.local_planner_sim_time / self.local_planner_sim_dt)))

        for _ in range(steps):
            cos_yaw = math.cos(ryaw)
            sin_yaw = math.sin(ryaw)
            rx += (candidate.linear.x * cos_yaw - candidate.linear.y * sin_yaw) * self.local_planner_sim_dt
            ry += (candidate.linear.x * sin_yaw + candidate.linear.y * cos_yaw) * self.local_planner_sim_dt
            ryaw = self.normalize_angle(ryaw + candidate.angular.z * self.local_planner_sim_dt)

            if self.local_planner_use_astar_grid and self.astar_walkable_cells:
                if self.world_to_cell(rx, ry) not in self.astar_walkable_cells:
                    return None

            clearance = self.avoid_region_clearance_at(rx, ry)
            min_clearance = min(min_clearance, clearance)
            if clearance < 0.0:
                if start_clearance >= 0.0:
                    return None
                if clearance < start_clearance - self.local_planner_recovery_clearance_epsilon:
                    return None

        end_dist = math.hypot(target_x - rx, target_y - ry)
        target_yaw = math.atan2(target_y - ry, target_x - rx)
        heading_err = abs(self.normalize_angle(target_yaw - ryaw))
        progress = start_dist - end_dist
        speed = math.hypot(candidate.linear.x, candidate.linear.y)
        nominal_delta = (
            abs(candidate.linear.x - nominal.linear.x)
            + abs(candidate.linear.y - nominal.linear.y)
            + 0.4 * abs(candidate.angular.z - nominal.angular.z)
        )
        clearance_score = 0.0 if math.isinf(min_clearance) else min(min_clearance, 1.0)
        score = (
            self.local_planner_path_weight * progress
            + self.local_planner_clearance_weight * clearance_score
            + self.local_planner_speed_weight * speed
            - self.local_planner_heading_weight * heading_err
            - self.local_planner_nominal_weight * nominal_delta
        )
        if (
            precision_follow
            and nominal.linear.x >= self.local_planner_min_vx
            and candidate.linear.x < self.local_planner_min_vx
        ):
            score -= 2.0
        if speed < 0.02 and abs(candidate.angular.z) < 0.02 and math.hypot(nominal.linear.x, nominal.linear.y) > 0.02:
            score -= 1.0
        return score

    def filter_slalom_script_command(
        self,
        pose: tuple[float, float, float],
        target_xy: tuple[float, float],
        nominal: Twist,
    ) -> Twist:
        if not self.route_avoid_regions and not (
            self.local_planner_use_astar_grid and self.astar_walkable_cells
        ):
            return nominal

        if self.local_candidate_score(
            pose,
            target_xy,
            nominal,
            nominal,
            precision_follow=True,
        ) is not None:
            return nominal

        def scaled_linear(value: float, scale: float) -> float:
            scaled = value * scale
            if abs(scaled) < self.slalom_script_min_cmd_linear:
                return 0.0
            return scaled

        candidates: list[Twist] = []
        for linear_scale in (0.75, 0.5, 0.0):
            for angular_scale in (1.0, 0.5, 0.0):
                candidates.append(
                    self.make_twist(
                        scaled_linear(nominal.linear.x, linear_scale),
                        scaled_linear(nominal.linear.y, linear_scale),
                        nominal.angular.z * angular_scale,
                    )
                )

        best_cmd: Optional[Twist] = None
        best_score: Optional[float] = None
        for candidate in candidates:
            score = self.local_candidate_score(
                pose,
                target_xy,
                candidate,
                nominal,
                precision_follow=True,
            )
            if score is None:
                continue
            if best_score is None or score > best_score:
                best_score = score
                best_cmd = candidate
        return best_cmd if best_cmd is not None else Twist()

    def apply_local_planner(
        self,
        pose: tuple[float, float, float],
        target_xy: tuple[float, float],
        goal: GoalSpec,
        nominal: Twist,
        max_vx: float,
        max_vy: float,
        yaw_rate_limit: float,
    ) -> Twist:
        if not self.local_planner_enabled_for_goal(goal):
            return nominal

        return self.select_local_command(
            pose,
            target_xy,
            nominal,
            max_vx,
            max_vy,
            yaw_rate_limit,
            goal.precision_follow,
        )

    def select_local_command(
        self,
        pose: tuple[float, float, float],
        target_xy: tuple[float, float],
        nominal: Twist,
        max_vx: float,
        max_vy: float,
        yaw_rate_limit: float,
        precision_follow: bool,
    ) -> Twist:
        if not self.route_avoid_regions and not (
            self.local_planner_use_astar_grid and self.astar_walkable_cells
        ):
            return nominal

        vx_values = [0.0]
        if max_vx >= self.local_planner_min_vx:
            vx_values.extend(
                self.sample_range(
                    self.local_planner_min_vx,
                    max_vx,
                    self.local_planner_v_samples,
                )
            )
        else:
            vx_values.append(max_vx)
        vx_values.extend(
            [
                nominal.linear.x,
                max_vx,
                self.local_planner_min_vx,
            ]
        )
        vx_values = self.effective_velocity_values(
            vx_values,
            0.0,
            max_vx,
            self.local_planner_min_vx,
        )

        wz_values = self.sample_range(-yaw_rate_limit, yaw_rate_limit, self.local_planner_w_samples)
        wz_values.extend([nominal.angular.z, 0.0])
        wz_values = self.unique_values(wz_values, -yaw_rate_limit, yaw_rate_limit)

        if max_vy > 1.0e-6:
            vy_values = self.sample_range(-max_vy, max_vy, self.local_planner_vy_samples)
            vy_values.extend(
                [
                    nominal.linear.y,
                    0.0,
                    -min(self.slalom_script_min_cmd_linear, max_vy),
                    min(self.slalom_script_min_cmd_linear, max_vy),
                ]
            )
            vy_values = self.effective_velocity_values(
                vy_values,
                -max_vy,
                max_vy,
                min(self.slalom_script_min_cmd_linear, max_vy),
            )
        else:
            vy_values = [0.0]

        best_cmd: Optional[Twist] = None
        best_score: Optional[float] = None
        for vx in vx_values:
            for vy in vy_values:
                for wz in wz_values:
                    candidate = self.make_twist(vx, vy, wz)
                    score = self.local_candidate_score(
                        pose,
                        target_xy,
                        candidate,
                        nominal,
                        precision_follow,
                    )
                    if score is None:
                        continue
                    if best_score is None or score > best_score:
                        best_score = score
                        best_cmd = candidate

        if best_cmd is None:
            return Twist()
        return best_cmd

    def on_timer(self) -> None:
        if self.active_goal is None:
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

        self.maybe_publish_pending_policy(force=False)

        rx, ry, ryaw = pose
        if self.handle_slalom_script(pose):
            return

        gx = self.active_goal.x
        gy = self.active_goal.y
        goal_dx = gx - rx
        goal_dy = gy - ry
        dist = math.hypot(goal_dx, goal_dy)
        tol_enter = self.active_goal.tolerance
        if (
            self.slalom_script_enabled
            and self.active_goal.slalom_straight
            and not self.slalom_script_active
        ):
            tol_enter = min(tol_enter, self.slalom_script_start_tolerance)
        tol_exit = self.active_goal.tolerance + self.goal_exit_tolerance_margin
        position_ready = dist < (tol_exit if self.goal_entered_tolerance else tol_enter)
        max_vx = self.get_goal_max_vx(self.active_goal)

        if position_ready:
            self.goal_entered_tolerance = True
            stable_cycles = self.get_goal_stable_cycles(self.active_goal)
            if stable_cycles <= 0 and self.advance_precision_goal_without_stop():
                return
            if self.goal_requires_yaw(self.active_goal):
                yaw_tolerance = (
                    self.active_goal.yaw_tolerance
                    if self.active_goal.yaw_tolerance is not None else self.goal_yaw_tolerance
                )
                yaw_err = self.normalize_angle(self.active_goal.yaw - ryaw)
                if abs(yaw_err) > yaw_tolerance:
                    self.goal_complete_stable_count = 0
                    cmd = Twist()
                    cmd.angular.z = self.clamp(
                        self.kp_yaw * self.final_align_kp_yaw_scale * yaw_err,
                        -min(self.final_align_max_wz, self.get_goal_yaw_rate_limit(self.active_goal)),
                        min(self.final_align_max_wz, self.get_goal_yaw_rate_limit(self.active_goal)),
                    )
                    # Keep a very small forward creep near the goal to reduce left-right rocking
                    if self.final_align_creep_speed > 0.0 and dist > max(0.02, tol_enter * 0.35):
                        align_target_yaw = math.atan2(goal_dy, goal_dx)
                        align_path_err = self.normalize_angle(align_target_yaw - ryaw)
                        if abs(align_path_err) <= self.yaw_stop_threshold:
                            cmd.linear.x = self.clamp(
                                self.kp_dist * dist * math.cos(align_path_err),
                                0.0,
                                min(max_vx, self.final_align_creep_speed),
                            )
                    self.cmd_pub.publish(cmd)
                    return
            self.goal_complete_stable_count += 1
            if self.goal_complete_stable_count < stable_cycles:
                self.cmd_pub.publish(Twist())
                return
            self.cmd_pub.publish(Twist())
            if self.active_goal_stage == "pre_dock" and self.pending_final_goal is not None:
                next_goal = self.pending_final_goal
                self.pending_final_goal = None
                self.activate_goal(next_goal, stage="final")
                self.publish_status(f"pre-dock reached for {next_goal.name}, entering final approach")
                return
            reached_name = self.active_goal.name
            if self.active_mission_name is not None:
                self.publish_status(f"reached {reached_name}")
                self.active_mission_index += 1
                self._activate_mission_goal()
            else:
                self.cancel_navigation(f"reached {reached_name}")
            return
        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0

        target = self.get_path_follow_target((rx, ry), self.active_goal)
        if target is None:
            self.cmd_pub.publish(Twist())
            return
        if self.active_goal is not None and self.active_goal.precision_follow:
            self.sync_precision_goal_to_path_progress()
            max_vx = self.get_goal_max_vx(self.active_goal)
        target_x, target_y = target
        dx = target_x - rx
        dy = target_y - ry
        target_yaw = math.atan2(dy, dx)
        yaw_err = self.normalize_angle(target_yaw - ryaw)
        if self.turn_in_place_enabled and not self.active_goal.precision_follow:
            if self.turn_in_place_mode:
                if abs(yaw_err) <= self.turn_in_place_exit_yaw:
                    self.turn_in_place_mode = False
            elif abs(yaw_err) >= self.turn_in_place_enter_yaw:
                self.turn_in_place_mode = True

        if self.turn_in_place_mode:
            cmd = Twist()
            cmd.angular.z = self.clamp(
                self.kp_yaw * yaw_err,
                -min(self.turn_in_place_max_wz, self.get_goal_yaw_rate_limit(self.active_goal)),
                min(self.turn_in_place_max_wz, self.get_goal_yaw_rate_limit(self.active_goal)),
            )
            self.cmd_pub.publish(cmd)
            return

        cmd = Twist()
        yaw_rate_limit = self.get_goal_yaw_rate_limit(self.active_goal)
        max_vx = self.get_curvature_limited_max_vx(self.active_goal, max_vx, yaw_err)
        max_vy = self.get_goal_max_vy(self.active_goal)
        cmd.angular.z = self.clamp(self.kp_yaw * yaw_err, -yaw_rate_limit, yaw_rate_limit)
        forward_yaw_threshold = math.pi if self.active_goal.precision_follow else self.yaw_stop_threshold
        if abs(yaw_err) <= forward_yaw_threshold:
            path_dist = math.hypot(dx, dy)
            cmd.linear.x = self.clamp(self.kp_dist * path_dist * math.cos(yaw_err), 0.0, max_vx)
            if max_vy > 0.0:
                lateral_error = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
                cmd.linear.y = self.clamp(
                    self.precision_lateral_kp * lateral_error,
                    -max_vy,
                    max_vy,
                )
        cmd = self.apply_local_planner(
            pose,
            (target_x, target_y),
            self.active_goal,
            cmd,
            max_vx,
            max_vy,
            yaw_rate_limit,
        )
        self.cmd_pub.publish(cmd)

    def cancel_navigation(self, reason: str) -> None:
        self.active_goal = None
        self.active_mission_name = None
        self.active_mission_goals = []
        self.active_mission_index = 0
        self.active_goal_stage = "idle"
        self.pending_final_goal = None
        self.active_path_world = []
        self.active_path_index = 0
        self.active_path_goal_key = None
        self.active_precision_segment_start = None
        self.active_precision_segment_end = None
        self.reset_slalom_script()
        self.turn_in_place_mode = False
        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0
        self.cmd_pub.publish(Twist())
        self.publish_active_path(force=True)
        self.publish_status(f"navigation stopped: {reason}")

    def publish_goal_pose(self, goal: GoalSpec) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = goal.x
        msg.pose.position.y = goal.y
        if goal.yaw is None:
            msg.pose.orientation.w = 1.0
        else:
            half_yaw = goal.yaw * 0.5
            msg.pose.orientation.z = math.sin(half_yaw)
            msg.pose.orientation.w = math.cos(half_yaw)
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
