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
    yaw: Optional[float] = None
    yaw_tolerance: Optional[float] = None
    tolerance: float = 0.20
    speed: Optional[float] = None
    policy: Optional[str] = None
    require_yaw: bool = False
    pre_dock_distance: Optional[float] = None
    pre_dock_tolerance: Optional[float] = None


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
        self.astar_walkable_cells, self.astar_grid_info = self.build_astar_grid(self.astar_source_points)
        self.goals: dict[str, GoalSpec] = {}
        self.missions: dict[str, list[str]] = {}
        self.default_mission_name: Optional[str] = None
        self.reload_navigation_data()
        self.active_goal: Optional[GoalSpec] = None
        self.active_mission_name: Optional[str] = None
        self.active_mission_goals: list[str] = []
        self.active_mission_index = 0
        self.active_goal_stage = "idle"
        self.pending_final_goal: Optional[GoalSpec] = None
        self.active_path_world: list[tuple[float, float]] = []
        self.active_path_index = 0
        self.active_path_goal_key: Optional[tuple[str, str]] = None
        self.last_published_path_signature: Optional[tuple[str, str, int, int]] = None
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
            f"route_source={self.get_route_source_file() or 'none'})"
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
        if text in {"rough", "crawl"}:
            return text
        return None

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

        xs = [cell[0] for cell in walkable_cells]
        ys = [cell[1] for cell in walkable_cells]
        return walkable_cells, {
            "resolution": resolution,
            "clearance_cells": clearance_cells,
            "cell_count": len(walkable_cells),
            "bounds": [min(xs), min(ys), max(xs), max(ys)],
        }

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        resolution = self.astar_resolution
        return int(round(float(x) / resolution)), int(round(float(y) / resolution))

    def cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        resolution = self.astar_resolution
        return float(cell[0]) * resolution, float(cell[1]) * resolution

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
        route_goals, route_missions, route_default, route_alignment = self._load_route(route_source_file)
        goals.update(route_goals)
        missions.update(route_missions)
        self.goals = goals
        self.missions = missions
        self.default_mission_name = route_default or (next(iter(missions)) if missions else None)
        self.route_alignment_info = route_alignment

    def get_route_source_file(self) -> str:
        candidate = self.route_task_file.strip() if self.route_task_file else ""
        if candidate:
            return candidate
        return self.route_file

    def _load_route(
        self, path_value: str
    ) -> tuple[dict[str, GoalSpec], dict[str, list[str]], Optional[str], dict[str, Any]]:
        data = self._load_yaml(path_value)
        if not data:
            return {}, {}, None, {}

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
                return {}, {}, None, {}

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
                            "x": float(waypoint["x"]),
                            "y": float(waypoint["y"]),
                            "yaw": math.radians(float(waypoint_yaw_deg)) if waypoint_yaw_deg is not None else None,
                            "yaw_tolerance": (
                                math.radians(max(0.0, waypoint_yaw_tolerance_deg))
                                if waypoint_yaw_tolerance_deg is not None else None
                            ),
                            "tolerance": max(0.01, float(waypoint.get("tolerance", self.goal_tolerance))),
                            "speed": float(waypoint["speed"]) if waypoint.get("speed") is not None else None,
                            "policy": self._normalize_policy(waypoint.get("policy")),
                            "require_yaw": bool(waypoint_require_yaw) and waypoint_yaw_deg is not None,
                            "pre_dock_distance": waypoint_pre_dock_distance,
                            "pre_dock_tolerance": waypoint_pre_dock_tolerance,
                        }
                    )
                except (KeyError, TypeError, ValueError):
                    continue

        if not raw_waypoints:
            self.get_logger().warn(f"Route file has no usable waypoints: {path_value}")
            return {}, {}, None, {}

        aligned_waypoints, alignment_info = self._align_route_waypoints(raw_waypoints)
        goals: dict[str, GoalSpec] = {}
        mission_goals: list[str] = []
        for index, waypoint in enumerate(aligned_waypoints, start=1):
            goal_name = f"{route_name}_p{index:02d}"
            goals[goal_name] = GoalSpec(
                name=goal_name,
                x=float(waypoint["x"]),
                y=float(waypoint["y"]),
                yaw=waypoint.get("yaw"),
                yaw_tolerance=waypoint.get("yaw_tolerance"),
                tolerance=float(waypoint.get("tolerance", self.goal_tolerance)),
                speed=waypoint.get("speed"),
                policy=self._normalize_policy(waypoint.get("policy")),
                require_yaw=bool(waypoint.get("require_yaw", False)),
                pre_dock_distance=waypoint.get("pre_dock_distance"),
                pre_dock_tolerance=waypoint.get("pre_dock_tolerance"),
            )
            mission_goals.append(goal_name)

        if alignment_info:
            applied = float(alignment_info.get("applied_deg", 0.0))
            hits = int(alignment_info.get("hits", 0))
            total = int(alignment_info.get("total", 0))
            reason = str(alignment_info.get("reason", "")).strip()
            reason_text = f" reason={reason}" if reason else ""
            self.get_logger().info(
                f"Loaded route {route_name}: {len(mission_goals)} waypoints, alignment={applied:.2f}deg hits={hits}/{total}{reason_text}"
            )
        else:
            self.get_logger().info(f"Loaded route {route_name}: {len(mission_goals)} waypoints")

        return goals, {route_name: mission_goals}, route_name, alignment_info

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
        elif op == "reload":
            self.reload_navigation_data()
            self.publish_status(
                f"reloaded goals={len(self.goals)} missions={len(self.missions)} default={self.default_mission_name or 'none'}"
            )
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
        self.last_published_path_signature = None
        self.turn_in_place_mode = False
        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0
        self.pose_waiting_reported = False
        self.request_policy(goal.policy)
        self.publish_goal_pose(goal)

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
    ) -> tuple[float, float]:
        if not self.ensure_active_path(pose_xy, goal):
            return goal.x, goal.y

        rx, ry = pose_xy
        advance_dist = max(self.astar_waypoint_reach_dist, min(goal.tolerance, self.astar_lookahead_dist))
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
            if math.hypot(tx - rx, ty - ry) > self.astar_lookahead_dist:
                break
            target_index = next_index
        previous_index = self.active_path_index
        self.active_path_index = max(self.active_path_index, target_index)
        if self.active_path_index != previous_index:
            self.publish_active_path(force=False)
        return self.active_path_world[self.active_path_index]

    def goal_requires_yaw(self, goal: GoalSpec) -> bool:
        return bool(goal.require_yaw and goal.yaw is not None)

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
        return suffix

    def plan_goal_execution(self, goal: GoalSpec) -> tuple[GoalSpec, Optional[GoalSpec], Optional[str]]:
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
        gx = self.active_goal.x
        gy = self.active_goal.y
        goal_dx = gx - rx
        goal_dy = gy - ry
        dist = math.hypot(goal_dx, goal_dy)
        tol_enter = self.active_goal.tolerance
        tol_exit = self.active_goal.tolerance + self.goal_exit_tolerance_margin
        position_ready = dist < (tol_exit if self.goal_entered_tolerance else tol_enter)
        max_vx = self.max_vx if self.active_goal.speed is None else min(self.max_vx, max(0.0, self.active_goal.speed))

        if position_ready:
            self.goal_entered_tolerance = True
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
                        -self.final_align_max_wz,
                        self.final_align_max_wz,
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
            if self.goal_complete_stable_count < self.goal_complete_stable_cycles:
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

        target_x, target_y = self.get_path_follow_target((rx, ry), self.active_goal)
        dx = target_x - rx
        dy = target_y - ry
        target_yaw = math.atan2(dy, dx)
        yaw_err = self.normalize_angle(target_yaw - ryaw)
        if self.turn_in_place_enabled:
            if self.turn_in_place_mode:
                if abs(yaw_err) <= self.turn_in_place_exit_yaw:
                    self.turn_in_place_mode = False
            elif abs(yaw_err) >= self.turn_in_place_enter_yaw:
                self.turn_in_place_mode = True

        if self.turn_in_place_mode:
            cmd = Twist()
            cmd.angular.z = self.clamp(
                self.kp_yaw * yaw_err,
                -self.turn_in_place_max_wz,
                self.turn_in_place_max_wz,
            )
            self.cmd_pub.publish(cmd)
            return

        cmd = Twist()
        cmd.angular.z = self.clamp(self.kp_yaw * yaw_err, -self.max_wz, self.max_wz)
        if abs(yaw_err) <= self.yaw_stop_threshold:
            path_dist = math.hypot(dx, dy)
            cmd.linear.x = self.clamp(self.kp_dist * path_dist * math.cos(yaw_err), 0.0, max_vx)
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
