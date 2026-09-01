#!/usr/bin/env python3
"""Batch sim2sim route validation for nav_tools waypoints.

This script runs the MuJoCo robot with an ONNX locomotion policy and a simple
waypoint follower, then reports whether the route can be completed without
falling or entering avoid-region clearance.

Example:
    uv run python sim2sim/nav_route_sim2sim_check.py \
        --terrain-xml tools/nav_tools/xml/A.xml \
        --points tools/nav_tools/points/points_20260705_174627.json \
        --onnx model_6800.onnx \
        --start-yaw-offset-deg -180 \
        --heading-offset-deg 180
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import pkgutil
import random
import sys
import time
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import mujoco.viewer
import numpy as np
import onnxruntime as ort
import torch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SIM2SIM_DIR = Path(__file__).resolve().parent
if str(SIM2SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM2SIM_DIR))
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from interface.mujoco_io import MuJoCoIO  # noqa: E402
from policy.policy_runner import PolicyRunner  # noqa: E402
from tools.nav_tools.route_safety_check import (  # noqa: E402
    AvoidRegion,
    Waypoint,
    default_lateral_footprint_radius,
    load_regions,
    load_waypoints,
    point_in_polygon,
    point_segment_distance,
)


ACTION_SCALE = np.array(
    [0.125, 0.25, 0.25] * 4 + [5.0, 5.0, 5.0, 5.0],
    dtype=np.float32,
)
DEPLOY_DEFAULT_DOF_POS = np.array([0.0, 0.550, -1.125] * 4 + [0.0] * 4, dtype=np.float32)
ROBOT_BODY_LENGTH = 0.356
ROBOT_BODY_WIDTH = 0.235
ROBOT_BODY_CENTER_X = 0.1518
ROBOT_ORIGIN_FROM_FRONT = 0.105
ROBOT_BODY_CENTER_OFFSET_X = ROBOT_ORIGIN_FROM_FRONT - ROBOT_BODY_LENGTH * 0.5
ROBOT_WHEEL_VIS_LENGTH = 0.16
ROBOT_WHEEL_VIS_WIDTH = 0.055
ROBOT_WHEEL_POSITIONS = {
    "fl": ((0.32826 + 0.06389) - ROBOT_BODY_CENTER_X, 0.066172 - 0.027344, 0.1035, 0.014699, 0.04074, 0.0),
    "fr": ((0.32826 + 0.06389) - ROBOT_BODY_CENTER_X, -0.065853 + 0.027311, -0.1035, -0.018447, -0.040735, -0.00075079),
    "rl": ((-0.024743 - 0.06389) - ROBOT_BODY_CENTER_X, 0.066141 - 0.027309, 0.099459, 0.012475, 0.040737, 0.0),
    "rr": ((-0.024743 - 0.06389) - ROBOT_BODY_CENTER_X, -0.065884 + 0.027341, -0.099408, -0.012435, -0.040737, -0.00075079),
}


@dataclass
class SimConfig:
    control_hz: float = 50.0
    max_vx: float = 1.2
    max_vy: float = 0.0
    max_wz: float = 0.8
    kp_dist: float = 0.8
    kp_yaw: float = 1.8
    yaw_stop_threshold_deg: float = 45.0
    turn_in_place_enter_deg: float = 70.0
    turn_in_place_exit_deg: float = 18.0
    turn_in_place_max_wz: float = 0.8
    final_align_max_wz: float = 0.45
    final_align_kp_scale: float = 0.6
    waypoint_timeout_s: float = 30.0
    max_total_time_s: float = 420.0
    stable_cycles: int = 2
    lookahead_m: float = 0.45
    min_cmd_vx: float = 0.08
    creep_cmd_vx: float = 0.04
    cmd_vx_scale: float = 1.0
    stuck_timeout_s: float = 4.0
    stuck_progress_m: float = 0.08
    recovery_duration_s: float = 2.0
    max_recoveries: int = 4
    slalom_script_enabled: bool = True
    slalom_script_start_tolerance: float = 0.10
    slalom_script_pos_tolerance: float = 0.08
    slalom_script_yaw_tolerance_deg: float = 5.0
    slalom_script_stable_cycles: int = 1
    slalom_script_rotate_steps_enabled: bool = False
    slalom_script_final_rotate_enabled: bool = False
    slalom_script_require_yaw_at_step: bool = False
    slalom_script_kp_dist: float = 1.0
    slalom_script_kp_yaw: float = 1.6
    slalom_script_max_vx: float = 0.60
    slalom_script_max_vy: float = 0.50
    slalom_script_max_wz: float = 0.50
    slalom_script_min_cmd_linear: float = 0.2
    slalom_script_min_cmd_angular: float = 0.0
    slalom_script_min_cmd_epsilon: float = 0.05
    slalom_script_min_step_distance: float = 0.02
    slalom_script_yaw_gate_deg: float = 25.0
    slalom_script_drive_yaw_source: str = "segment"
    slalom_script_curvature_speed_enabled: bool = True
    slalom_script_curvature_min_scale: float = 0.55


@dataclass(frozen=True)
class SlalomScriptStep:
    kind: str
    start_index: int
    end_index: int
    target_x: float
    target_y: float
    target_yaw: float | None
    pos_tolerance: float | None = None
    forward: float = 0.0
    left: float = 0.0


@dataclass
class SimResult:
    success: bool
    reason: str
    sim_time: float
    reached_count: int
    waypoint_count: int
    min_clearance: float
    min_margin: float
    min_clearance_region: str
    min_clearance_wp: str
    max_roll_deg: float
    max_pitch_deg: float
    max_tilt_deg: float
    samples: int
    wall_time: float
    real_time_factor: float


@dataclass
class RouteSnapshot:
    time: float
    qpos: np.ndarray
    qvel: np.ndarray
    ctrl: np.ndarray
    follower_index: int
    follower_turn_in_place: bool
    follower_stable_count: int
    follower_wp_start_time: float
    follower_best_dist: float | None
    follower_last_progress_time: float | None
    follower_recovery_until: float | None
    follower_recovery_count: int | None
    follower_recovery_turn_sign: float | None


class OnnxPolicy:
    def __init__(self, path: Path) -> None:
        providers = ["CPUExecutionProvider"]
        self.session = ort.InferenceSession(str(path), providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        shape = self.session.get_inputs()[0].shape
        dim = shape[1] if len(shape) >= 2 else 53
        self.obs_dim = 53 if isinstance(dim, str) else int(dim)
        self.last_actions = np.zeros(16, dtype=np.float32)

    def reset(self) -> None:
        self.last_actions[:] = 0.0

    def step(self, obs_53d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if self.obs_dim == obs_53d.shape[0]:
            model_input = obs_53d
        else:
            raise RuntimeError(
                f"ONNX expects obs dim {self.obs_dim}, but this checker provides {obs_53d.shape[0]}"
            )
        raw = self.session.run(
            [self.output_name],
            {self.input_name: model_input.astype(np.float32)[None, :]},
        )[0][0].astype(np.float32)
        raw = np.clip(raw, -100.0, 100.0)
        self.last_actions = raw.copy()
        return raw * ACTION_SCALE, raw


class ExistingPolicyBackend:
    """Adapter around sim2sim/policy/PolicyRunner to match nav_sim2sim.py."""

    def __init__(self, path: Path, crawl_path: Path | None = None) -> None:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        policy_path: Path | dict[str, Path]
        if crawl_path is not None:
            policy_path = {"rough": path, "crawl": crawl_path}
        else:
            policy_path = path
        self.runner = PolicyRunner(policy_path, device)
        self.runner.reset()

    @property
    def default_dof_pos(self) -> np.ndarray:
        return self.runner.default_dof_pos

    @property
    def last_actions(self) -> np.ndarray:
        return self.runner.last_actions

    def reset(self) -> None:
        self.runner.reset()

    def maybe_switch_policy(self, policy_name: str | None) -> None:
        if not policy_name:
            return
        if policy_name == self.runner.current_policy_name:
            return
        if self.runner.transition_in_progress and policy_name == self.runner.current_policy_name:
            return
        if policy_name in self.runner.policies:
            self.runner.trigger_transition(policy_name)

    def step(self, obs_53d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return self.runner.step(obs_53d)

    def close(self) -> None:
        listener = getattr(self.runner, "listener", None)
        if listener is not None:
            listener.stop()


class RoughPolicyIkCrawlBackend(ExistingPolicyBackend):
    def __init__(self, path: Path) -> None:
        super().__init__(path, None)
        self.mode = "rough"
        self.requested_mode = "rough"
        self.crawl_default_dof_pos = np.array(
            [
                0.2, 1.697, -2.650,
                -0.2, 1.697, -2.650,
                0.2, 1.697, -2.650,
                -0.2, 1.697, -2.650,
                0.0, 0.0, 0.0, 0.0,
            ],
            dtype=np.float32,
        )
        self.slalom_ik_default_dof_pos = np.array(
            [
                0.0, 1.697, -2.650,
                0.0, 1.697, -2.650,
                0.0, 1.697, -2.650,
                0.0, 1.697, -2.650,
                0.0, 0.0, 0.0, 0.0,
            ],
            dtype=np.float32,
        )
        self._last_actions = np.zeros(16, dtype=np.float32)
        self.crawl_ik_wheel_linear_gain = 6.25
        self.crawl_ik_wheel_yaw_gain = 4.0
        self.crawl_ik_max_wheel_speed = 6.0
        self.transition_steps = 60
        self.transition_step = self.transition_steps
        self.transition_start_pose = self.runner.default_dof_pos.copy()
        self.transition_target_pose = self.runner.default_dof_pos.copy()

    def reset(self) -> None:
        self.runner.reset()
        self.mode = "rough"
        self.requested_mode = "rough"
        self.transition_step = self.transition_steps
        self.transition_start_pose = self.runner.default_dof_pos.copy()
        self.transition_target_pose = self.runner.default_dof_pos.copy()
        self._last_actions[:] = 0.0

    @property
    def default_dof_pos(self) -> np.ndarray:
        if self.transition_step < self.transition_steps:
            alpha = self.transition_step / max(1, self.transition_steps)
            return (1.0 - alpha) * self.transition_start_pose + alpha * self.transition_target_pose
        if self.requested_mode == "crawl":
            return self.crawl_default_dof_pos
        if self.requested_mode == "slalom_ik":
            return self.slalom_ik_default_dof_pos
        return self.runner.default_dof_pos

    @property
    def last_actions(self) -> np.ndarray:
        if self.requested_mode in {"crawl", "slalom_ik"}:
            return self._last_actions
        return self.runner.last_actions

    def maybe_switch_policy(self, policy_name: str | None) -> None:
        normalized = (policy_name or "").lower()
        if normalized in {"slalom", "slalom_ik", "slalomik"}:
            next_mode = "slalom_ik"
        elif normalized in {"crawl", "ik"}:
            next_mode = "crawl"
        else:
            next_mode = "rough"
        if next_mode == self.requested_mode:
            return
        self.transition_start_pose = self.default_dof_pos.copy()
        if next_mode == "crawl":
            self.transition_target_pose = self.crawl_default_dof_pos.copy()
        elif next_mode == "slalom_ik":
            self.transition_target_pose = self.slalom_ik_default_dof_pos.copy()
        else:
            self.transition_target_pose = self.runner.default_dof_pos.copy()
        self.transition_step = 0
        self.requested_mode = next_mode

    def step(self, obs_53d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        in_transition = self.transition_step < self.transition_steps
        if in_transition:
            self.transition_step += 1
        if self.requested_mode not in {"crawl", "slalom_ik"}:
            return self.runner.step(obs_53d)
        cmd = obs_53d[6:9].astype(np.float32)
        target = (
            self.slalom_ik_default_dof_pos.copy()
            if self.requested_mode == "slalom_ik"
            else self.crawl_default_dof_pos.copy()
        )
        left_wheel = float(np.clip(
            cmd[0] * self.crawl_ik_wheel_linear_gain - cmd[2] * self.crawl_ik_wheel_yaw_gain,
            -self.crawl_ik_max_wheel_speed,
            self.crawl_ik_max_wheel_speed,
        ))
        right_wheel = float(np.clip(
            cmd[0] * self.crawl_ik_wheel_linear_gain + cmd[2] * self.crawl_ik_wheel_yaw_gain,
            -self.crawl_ik_max_wheel_speed,
            self.crawl_ik_max_wheel_speed,
        ))
        alpha = 1.0
        if in_transition:
            alpha = min(1.0, self.transition_step / max(1, self.transition_steps))
        scaled = target.copy()
        scaled[12:] = [left_wheel, right_wheel, left_wheel, right_wheel]
        scaled[12:] *= alpha
        raw = np.zeros(16, dtype=np.float32)
        raw[12:] = (np.array([left_wheel, right_wheel, left_wheel, right_wheel], dtype=np.float32) * alpha) / 5.0
        self._last_actions = raw
        # send_actions adds default_dof_pos to scaled actions, so return leg offsets.
        scaled[:12] = 0.0
        return scaled, raw


class DirectOnnxBackend:
    """Lightweight ONNX adapter kept for isolating PolicyRunner effects."""

    def __init__(self, path: Path, default_dof_pos: np.ndarray) -> None:
        self.policy = OnnxPolicy(path)
        self._default_dof_pos = default_dof_pos.astype(np.float32)

    @property
    def default_dof_pos(self) -> np.ndarray:
        return self._default_dof_pos

    @property
    def last_actions(self) -> np.ndarray:
        return self.policy.last_actions

    def reset(self) -> None:
        self.policy.reset()

    def maybe_switch_policy(self, policy_name: str | None) -> None:
        _ = policy_name

    def step(self, obs_53d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return self.policy.step(obs_53d)

    def close(self) -> None:
        pass


class WaypointFollower:
    def __init__(self, waypoints: list[Waypoint], cfg: SimConfig) -> None:
        self.waypoints = waypoints
        self.cfg = cfg
        self.index = 0
        self.turn_in_place = False
        self.stable_count = 0
        self.wp_start_time = 0.0

    @property
    def active(self) -> Waypoint | None:
        if self.index >= len(self.waypoints):
            return None
        return self.waypoints[self.index]

    def reset_timing(self, sim_time: float) -> None:
        self.wp_start_time = sim_time

    @staticmethod
    def normalize_angle(value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    @staticmethod
    def clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def update(self, x: float, y: float, yaw: float, sim_time: float) -> tuple[np.ndarray, str | None]:
        wp = self.active
        if wp is None:
            return np.zeros(3, dtype=np.float32), "complete"

        if sim_time - self.wp_start_time > self.cfg.waypoint_timeout_s:
            return np.zeros(3, dtype=np.float32), f"timeout at waypoint {wp.id}"

        dx = wp.x - x
        dy = wp.y - y
        dist = math.hypot(dx, dy)
        tolerance = wp.tolerance if wp.tolerance is not None else 0.15
        if dist <= tolerance:
            self.stable_count += 1
            if self.stable_count >= self.cfg.stable_cycles:
                self.index += 1
                self.turn_in_place = False
                self.stable_count = 0
                self.wp_start_time = sim_time
                if self.index >= len(self.waypoints):
                    return np.zeros(3, dtype=np.float32), "complete"
                return np.zeros(3, dtype=np.float32), None
            return np.zeros(3, dtype=np.float32), None
        self.stable_count = 0

        target_yaw = math.atan2(dy, dx)
        yaw_err = self.normalize_angle(target_yaw - yaw)
        yaw_stop_threshold = math.radians(self.cfg.yaw_stop_threshold_deg)

        if self.turn_in_place:
            if abs(yaw_err) <= math.radians(self.cfg.turn_in_place_exit_deg):
                self.turn_in_place = False
        elif abs(yaw_err) >= math.radians(self.cfg.turn_in_place_enter_deg):
            self.turn_in_place = True

        if self.turn_in_place:
            return np.array(
                [
                    0.0,
                    0.0,
                    self.clamp(
                        self.cfg.kp_yaw * yaw_err,
                        -self.cfg.turn_in_place_max_wz,
                        self.cfg.turn_in_place_max_wz,
                    ),
                ],
                dtype=np.float32,
            ), None

        speed_limit = self.cfg.max_vx if wp.speed is None else min(self.cfg.max_vx, max(0.0, wp.speed))
        cmd = np.zeros(3, dtype=np.float32)
        cmd[2] = self.clamp(self.cfg.kp_yaw * yaw_err, -self.cfg.max_wz, self.cfg.max_wz)
        if abs(yaw_err) <= yaw_stop_threshold:
            cmd[0] = self.clamp(self.cfg.kp_dist * dist * math.cos(yaw_err), 0.0, speed_limit)
        cmd[0] *= self.cfg.cmd_vx_scale
        return cmd, None


class PurePursuitFollower(WaypointFollower):
    def __init__(self, waypoints: list[Waypoint], cfg: SimConfig) -> None:
        super().__init__(waypoints, cfg)
        self.best_dist = float("inf")
        self.last_progress_time = 0.0
        self.recovery_until = -1.0
        self.recovery_count = 0
        self.recovery_turn_sign = 1.0

    def reset_timing(self, sim_time: float) -> None:
        super().reset_timing(sim_time)
        self.best_dist = float("inf")
        self.last_progress_time = sim_time
        self.recovery_until = -1.0
        self.recovery_count = 0

    def _advance_reached(self, x: float, y: float, sim_time: float) -> str | None:
        while self.active is not None:
            wp = self.active
            if wp.require_yaw:
                break
            tol = wp.tolerance if wp.tolerance is not None else 0.15
            if wp.mandatory_cross:
                tol = min(tol, 0.15 if wp.mandatory_radius is None else max(0.01, wp.mandatory_radius))
            completion_x = wp.x if wp.mandatory_center_x is None else wp.mandatory_center_x
            completion_y = wp.y if wp.mandatory_center_y is None else wp.mandatory_center_y
            if math.hypot(completion_x - x, completion_y - y) > tol:
                break
            self.index += 1
            self.turn_in_place = False
            self.stable_count = 0
            self.wp_start_time = sim_time
            self.best_dist = float("inf")
            self.last_progress_time = sim_time
        if self.index >= len(self.waypoints):
            return "complete"
        return None

    def _lookahead_target(self, x: float, y: float) -> Waypoint:
        active = self.active
        if active is None:
            return self.waypoints[-1]
        if active.exact_reach:
            return active
        target = active
        for wp in self.waypoints[self.index :]:
            target = wp
            if wp.mandatory_cross:
                mandatory_radius = 0.15 if wp.mandatory_radius is None else max(0.01, wp.mandatory_radius)
                center_x = wp.x if wp.mandatory_center_x is None else wp.mandatory_center_x
                center_y = wp.y if wp.mandatory_center_y is None else wp.mandatory_center_y
                if math.hypot(center_x - x, center_y - y) > mandatory_radius:
                    break
            if math.hypot(wp.x - x, wp.y - y) >= self.cfg.lookahead_m:
                break
        return target

    def _current_speed_limit(self) -> float:
        speed = self.cfg.max_vx
        end = min(len(self.waypoints), self.index + 4)
        for wp in self.waypoints[self.index : end]:
            if wp.speed is not None:
                speed = min(speed, max(0.0, wp.speed))
        return speed

    def update(self, x: float, y: float, yaw: float, sim_time: float) -> tuple[np.ndarray, str | None]:
        nav_state = self._advance_reached(x, y, sim_time)
        if nav_state is not None:
            return np.zeros(3, dtype=np.float32), nav_state

        active = self.active
        if active is None:
            return np.zeros(3, dtype=np.float32), "complete"
        active_dist = math.hypot(active.x - x, active.y - y)
        if active.require_yaw and active_dist <= (active.tolerance if active.tolerance is not None else 0.15):
            yaw_tolerance_deg = 5.0 if active.yaw_tolerance_deg is None else max(0.0, active.yaw_tolerance_deg)
            yaw_err = self.normalize_angle(math.radians(active.yaw_deg) - yaw)
            if abs(yaw_err) > math.radians(yaw_tolerance_deg):
                self.stable_count = 0
                return np.array(
                    [
                        0.0,
                        0.0,
                        self.clamp(
                            self.cfg.kp_yaw * self.cfg.final_align_kp_scale * yaw_err,
                            -self.cfg.final_align_max_wz,
                            self.cfg.final_align_max_wz,
                        ),
                    ],
                    dtype=np.float32,
                ), None
            required_stable_cycles = (
                self.cfg.stable_cycles if active.stable_cycles is None else max(1, active.stable_cycles)
            )
            self.stable_count += 1
            if self.stable_count < required_stable_cycles:
                return np.zeros(3, dtype=np.float32), None
            self.index += 1
            self.turn_in_place = False
            self.stable_count = 0
            self.wp_start_time = sim_time
            self.best_dist = float("inf")
            self.last_progress_time = sim_time
            if self.index >= len(self.waypoints):
                return np.zeros(3, dtype=np.float32), "complete"
            return np.zeros(3, dtype=np.float32), None
        if active_dist + self.cfg.stuck_progress_m < self.best_dist:
            self.best_dist = active_dist
            self.last_progress_time = sim_time

        if self.recovery_until > sim_time:
            return np.array(
                [
                    -0.10 * abs(self.cfg.cmd_vx_scale),
                    0.0,
                    0.45 * self.recovery_turn_sign,
                ],
                dtype=np.float32,
            ), None

        if sim_time - self.wp_start_time > self.cfg.waypoint_timeout_s:
            return np.zeros(3, dtype=np.float32), f"timeout at waypoint {active.id}"

        if sim_time - self.last_progress_time > self.cfg.stuck_timeout_s:
            if active.require_yaw:
                self.last_progress_time = sim_time
            elif self.recovery_count >= self.cfg.max_recoveries:
                return np.zeros(3, dtype=np.float32), f"stuck near waypoint {active.id}"
            else:
                self.recovery_count += 1
                self.recovery_turn_sign *= -1.0
                self.recovery_until = sim_time + self.cfg.recovery_duration_s
                self.last_progress_time = sim_time
                return np.array([-0.10 * abs(self.cfg.cmd_vx_scale), 0.0, 0.45 * self.recovery_turn_sign], dtype=np.float32), None

        target = self._lookahead_target(x, y)
        dx = target.x - x
        dy = target.y - y
        target_dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_err = self.normalize_angle(target_yaw - yaw)

        cmd = np.zeros(3, dtype=np.float32)
        cmd[2] = self.clamp(self.cfg.kp_yaw * yaw_err, -self.cfg.max_wz, self.cfg.max_wz)
        if abs(yaw_err) > math.radians(self.cfg.turn_in_place_enter_deg):
            cmd[0] = 0.0
            return cmd, None

        speed_limit = self._current_speed_limit()
        curvature_slow = max(0.35, 1.0 - abs(yaw_err) / math.radians(80.0))
        cmd_vx = self.cfg.kp_dist * target_dist * math.cos(yaw_err)
        cmd_vx = self.clamp(cmd_vx, self.cfg.min_cmd_vx, speed_limit * curvature_slow)
        if abs(yaw_err) > math.radians(self.cfg.yaw_stop_threshold_deg):
            if abs(yaw_err) < math.radians(self.cfg.turn_in_place_enter_deg):
                cmd_vx = min(self.cfg.creep_cmd_vx, speed_limit)
            else:
                cmd_vx = 0.0
        cmd[0] = cmd_vx * self.cfg.cmd_vx_scale
        return cmd, None


class NavGoodFollower(WaypointFollower):
    """Follower shaped after sim2real nav_good/simple_nav_node.py.

    This keeps the route as the active path, advances through near path points,
    selects a lookahead target, then applies the same yaw gating and
    turn-in-place structure as the ROS2 simple_nav node.
    """

    def __init__(self, waypoints: list[Waypoint], cfg: SimConfig) -> None:
        super().__init__(waypoints, cfg)
        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0
        self.goal_exit_tolerance_margin = 0.05
        self.final_align_kp_yaw_scale = 0.6
        self.final_align_creep_speed = 0.03
        self.path_reach_dist = 0.18
        self.best_dist = float("inf")
        self.last_progress_time = 0.0

    def reset_timing(self, sim_time: float) -> None:
        super().reset_timing(sim_time)
        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0
        self.turn_in_place = False
        self.best_dist = float("inf")
        self.last_progress_time = sim_time

    def _current_goal(self) -> Waypoint | None:
        return self.active

    def _is_charge_segment(self, x: float, y: float, goal: Waypoint) -> bool:
        policy = (goal.policy or "").lower()
        if policy in {"charge", "wall", "obstacle", "climb"}:
            return True
        return -0.45 <= x <= 0.35 and -4.35 <= y <= -2.85

    def _point_is_behind_segment(
        self,
        x: float,
        y: float,
        point: Waypoint,
        next_point: Waypoint,
    ) -> bool:
        seg_x = next_point.x - point.x
        seg_y = next_point.y - point.y
        seg_len_sq = seg_x * seg_x + seg_y * seg_y
        if seg_len_sq <= 1.0e-9:
            return False
        proj = ((x - point.x) * seg_x + (y - point.y) * seg_y) / seg_len_sq
        return proj > 0.65 and math.hypot(next_point.x - x, next_point.y - y) < math.hypot(point.x - x, point.y - y)

    def _advance_path_nodes(self, x: float, y: float, sim_time: float) -> str | None:
        while self.index < len(self.waypoints) - 1:
            goal = self.waypoints[self.index]
            if goal.require_yaw:
                break
            next_goal = self.waypoints[self.index + 1]
            tol = max(self.path_reach_dist, goal.tolerance if goal.tolerance is not None else 0.15)
            dist = math.hypot(goal.x - x, goal.y - y)
            if dist > tol and not self._point_is_behind_segment(x, y, goal, next_goal):
                break
            self.index += 1
            self.wp_start_time = sim_time
            self.turn_in_place = False
            self.goal_entered_tolerance = False
            self.goal_complete_stable_count = 0
            self.best_dist = float("inf")
            self.last_progress_time = sim_time
        if self.index >= len(self.waypoints):
            return "complete"
        return None

    def _path_follow_target(self, x: float, y: float, goal: Waypoint) -> Waypoint:
        target_index = self.index
        while target_index < len(self.waypoints) - 1:
            candidate = self.waypoints[target_index + 1]
            if math.hypot(candidate.x - x, candidate.y - y) > self.cfg.lookahead_m:
                break
            target_index += 1
        return self.waypoints[target_index]

    def update(self, x: float, y: float, yaw: float, sim_time: float) -> tuple[np.ndarray, str | None]:
        nav_state = self._advance_path_nodes(x, y, sim_time)
        if nav_state is not None:
            return np.zeros(3, dtype=np.float32), nav_state

        goal = self._current_goal()
        if goal is None:
            return np.zeros(3, dtype=np.float32), "complete"
        if sim_time - self.wp_start_time > self.cfg.waypoint_timeout_s:
            return np.zeros(3, dtype=np.float32), f"timeout at waypoint {goal.id}"

        goal_dx = goal.x - x
        goal_dy = goal.y - y
        dist = math.hypot(goal_dx, goal_dy)
        tol_enter = goal.tolerance if goal.tolerance is not None else 0.15
        tol_exit = tol_enter + self.goal_exit_tolerance_margin
        position_ready = dist < (tol_exit if self.goal_entered_tolerance else tol_enter)
        max_vx = self.cfg.max_vx if goal.speed is None else min(self.cfg.max_vx, max(0.0, goal.speed))

        if goal.require_yaw and position_ready:
            self.goal_entered_tolerance = True
            yaw_tolerance_deg = 5.0 if goal.yaw_tolerance_deg is None else max(0.0, goal.yaw_tolerance_deg)
            yaw_err = self.normalize_angle(math.radians(goal.yaw_deg) - yaw)
            if abs(yaw_err) > math.radians(yaw_tolerance_deg):
                self.goal_complete_stable_count = 0
                return np.array(
                    [
                        0.0,
                        0.0,
                        self.clamp(
                            self.cfg.kp_yaw * self.final_align_kp_yaw_scale * yaw_err,
                            -self.cfg.final_align_max_wz,
                            self.cfg.final_align_max_wz,
                        ),
                    ],
                    dtype=np.float32,
                ), None
            required_stable_cycles = (
                self.cfg.stable_cycles if goal.stable_cycles is None else max(1, goal.stable_cycles)
            )
            self.goal_complete_stable_count += 1
            if self.goal_complete_stable_count < required_stable_cycles:
                return np.zeros(3, dtype=np.float32), None
            self.index += 1
            self.wp_start_time = sim_time
            self.turn_in_place = False
            self.goal_entered_tolerance = False
            self.goal_complete_stable_count = 0
            self.best_dist = float("inf")
            self.last_progress_time = sim_time
            if self.index >= len(self.waypoints):
                return np.zeros(3, dtype=np.float32), "complete"
            return np.zeros(3, dtype=np.float32), None

        is_final_goal = self.index >= len(self.waypoints) - 1
        if is_final_goal and position_ready:
            self.goal_entered_tolerance = True
            self.goal_complete_stable_count += 1
            if self.goal_complete_stable_count < self.cfg.stable_cycles:
                return np.zeros(3, dtype=np.float32), None
            self.index += 1
            self.wp_start_time = sim_time
            self.turn_in_place = False
            self.goal_entered_tolerance = False
            self.goal_complete_stable_count = 0
            if self.index >= len(self.waypoints):
                return np.zeros(3, dtype=np.float32), "complete"
            return np.zeros(3, dtype=np.float32), None

        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0
        if dist + self.cfg.stuck_progress_m < self.best_dist:
            self.best_dist = dist
            self.last_progress_time = sim_time
        if sim_time - self.last_progress_time > self.cfg.waypoint_timeout_s:
            return np.zeros(3, dtype=np.float32), f"no progress near waypoint {goal.id}"

        target = goal if goal.require_yaw else self._path_follow_target(x, y, goal)
        dx = target.x - x
        dy = target.y - y
        target_yaw = math.atan2(dy, dx)
        yaw_err = self.normalize_angle(target_yaw - yaw)

        if self.turn_in_place:
            if abs(yaw_err) <= math.radians(self.cfg.turn_in_place_exit_deg):
                self.turn_in_place = False
        elif abs(yaw_err) >= math.radians(self.cfg.turn_in_place_enter_deg):
            self.turn_in_place = True

        if self.turn_in_place:
            return np.array(
                [
                    0.0,
                    0.0,
                    self.clamp(self.cfg.kp_yaw * yaw_err, -self.cfg.turn_in_place_max_wz, self.cfg.turn_in_place_max_wz),
                ],
                dtype=np.float32,
            ), None

        cmd = np.zeros(3, dtype=np.float32)
        cmd[2] = self.clamp(self.cfg.kp_yaw * yaw_err, -self.cfg.max_wz, self.cfg.max_wz)
        if abs(yaw_err) <= math.radians(self.cfg.yaw_stop_threshold_deg):
            path_dist = math.hypot(dx, dy)
            if self._is_charge_segment(x, y, goal):
                cmd[0] = max_vx
            else:
                cmd[0] = self.clamp(self.cfg.kp_dist * path_dist * math.cos(yaw_err), 0.0, max_vx)
        cmd[0] *= self.cfg.cmd_vx_scale
        return cmd, None


class NavSim2SimFollower(WaypointFollower):
    """Follower matched to sim2sim/nav_sim2sim.py NavController.

    It tracks only the current waypoint, switches at 0.15 m, stops forward
    motion above ~45 deg yaw error, and can keep full waypoint speed through
    obstacle-charge segments instead of scaling speed down by distance.
    """

    def __init__(self, waypoints: list[Waypoint], cfg: SimConfig) -> None:
        super().__init__(waypoints, cfg)
        self.reach_dist = 0.15

    def _advance_reached(self, x: float, y: float, sim_time: float) -> str | None:
        while self.active is not None:
            wp = self.active
            tol = wp.tolerance if wp.tolerance is not None else self.reach_dist
            tol = max(self.reach_dist, tol)
            if math.hypot(wp.x - x, wp.y - y) >= tol:
                break
            if self.index < len(self.waypoints) - 1:
                self.index += 1
                self.turn_in_place = False
                self.stable_count = 0
                self.wp_start_time = sim_time
                return None
            self.index += 1
            return "complete"
        if self.index >= len(self.waypoints):
            return "complete"
        return None

    def _is_charge_segment(self, x: float, y: float, wp: Waypoint) -> bool:
        policy = (wp.policy or "").lower()
        if policy in {"charge", "wall", "obstacle", "climb"}:
            return True
        # A.xml has a low wall/step around x=0, y=-3.5. nav_sim2sim keeps
        # speed through comparable obstacle sections instead of distance taper.
        return -0.45 <= x <= 0.35 and -4.35 <= y <= -2.85

    def update(self, x: float, y: float, yaw: float, sim_time: float) -> tuple[np.ndarray, str | None]:
        nav_state = self._advance_reached(x, y, sim_time)
        if nav_state is not None:
            return np.zeros(3, dtype=np.float32), nav_state
        wp = self.active
        if wp is None:
            return np.zeros(3, dtype=np.float32), "complete"
        if sim_time - self.wp_start_time > self.cfg.waypoint_timeout_s:
            return np.zeros(3, dtype=np.float32), f"timeout at waypoint {wp.id}"

        dx = wp.x - x
        dy = wp.y - y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_err = self.normalize_angle(target_yaw - yaw)
        speed_limit = self.cfg.max_vx if wp.speed is None else min(self.cfg.max_vx, max(0.0, wp.speed))

        cmd = np.zeros(3, dtype=np.float32)
        cmd[2] = self.clamp(self.cfg.kp_yaw * yaw_err, -self.cfg.max_wz, self.cfg.max_wz)
        if abs(yaw_err) > 0.8:
            cmd[0] = 0.0
        elif self._is_charge_segment(x, y, wp):
            cmd[0] = speed_limit
        else:
            cmd[0] = self.clamp(self.cfg.kp_dist * dist * math.cos(yaw_err), -0.2, speed_limit)
        cmd[0] *= self.cfg.cmd_vx_scale
        return cmd, None


class SlalomScriptFollower(NavGoodFollower):
    """NavGood follower plus the sim2real slalomStraight odometry script.

    The script starts only after the robot reaches the first slalomStraight
    point, then drives consecutive slalomStraight points with body-frame
    vx/vy/wz feedback. This mirrors simple_nav_node.py instead of the older
    pure waypoint followers.
    """

    def __init__(self, waypoints: list[Waypoint], cfg: SimConfig) -> None:
        super().__init__(waypoints, cfg)
        self.slalom_script_active = False
        self.slalom_script_start_index = 0
        self.slalom_script_end_index = 0
        self.slalom_script_steps: list[SlalomScriptStep] = []
        self.slalom_script_step_index = 0
        self.slalom_script_step_stable_count = 0

    def reset_timing(self, sim_time: float) -> None:
        super().reset_timing(sim_time)
        self.reset_slalom_script()

    def reset_slalom_script(self) -> None:
        self.slalom_script_active = False
        self.slalom_script_start_index = 0
        self.slalom_script_end_index = 0
        self.slalom_script_steps = []
        self.slalom_script_step_index = 0
        self.slalom_script_step_stable_count = 0

    @staticmethod
    def _is_slalom_script_waypoint(wp: Waypoint | None) -> bool:
        return bool(wp is not None and getattr(wp, "slalom_straight", False))

    def _advance_path_nodes(self, x: float, y: float, sim_time: float) -> str | None:
        active = self.active
        if (
            self.cfg.slalom_script_enabled
            and self._is_slalom_script_waypoint(active)
            and not self.slalom_script_active
            and self.index + 1 < len(self.waypoints)
            and self._is_slalom_script_waypoint(self.waypoints[self.index + 1])
        ):
            # Do not let the normal path follower skip the script start point at
            # the wider path tolerance; the real node gates this with
            # nav_slalom_script_start_tolerance.
            return None
        return super()._advance_path_nodes(x, y, sim_time)

    def get_slalom_script_bounds(self, start_index: int) -> tuple[int, int]:
        if start_index < 0 or start_index >= len(self.waypoints):
            return start_index, start_index
        if not self._is_slalom_script_waypoint(self.waypoints[start_index]):
            return start_index, start_index
        end_index = start_index
        while end_index + 1 < len(self.waypoints):
            if getattr(self.waypoints[end_index], "slalom_script_break", False):
                break
            if not self._is_slalom_script_waypoint(self.waypoints[end_index + 1]):
                break
            end_index += 1
        return start_index, end_index

    def build_slalom_script_steps(self, start_index: int, end_index: int) -> list[SlalomScriptStep]:
        if end_index <= start_index:
            return []

        steps: list[SlalomScriptStep] = []
        for index in range(start_index, end_index):
            goal = self.waypoints[index]
            next_goal = self.waypoints[index + 1]
            route_dx = next_goal.x - goal.x
            route_dy = next_goal.y - goal.y
            route_dist = math.hypot(route_dx, route_dy)
            route_yaw = math.atan2(route_dy, route_dx) if route_dist > 1.0e-6 else math.radians(goal.yaw_deg)
            yaw_source = self.cfg.slalom_script_drive_yaw_source
            if yaw_source == "segment":
                target_yaw = route_yaw
            elif yaw_source == "blend":
                target_yaw = route_yaw
                if index + 2 <= end_index:
                    following_goal = self.waypoints[index + 2]
                    next_dx = following_goal.x - next_goal.x
                    next_dy = following_goal.y - next_goal.y
                    if math.hypot(next_dx, next_dy) > 1.0e-6:
                        next_yaw = math.atan2(next_dy, next_dx)
                        delta = self.normalize_angle(next_yaw - route_yaw)
                        target_yaw = route_yaw + 0.15 * delta
            elif yaw_source == "next":
                target_yaw = math.radians(next_goal.yaw_deg)
            else:
                target_yaw = math.radians(goal.yaw_deg)
            target_yaw = self.normalize_angle(target_yaw)
            if self.cfg.slalom_script_rotate_steps_enabled:
                steps.append(
                    SlalomScriptStep(
                        kind="rotate",
                        start_index=index,
                        end_index=index,
                        target_x=goal.x,
                        target_y=goal.y,
                        target_yaw=target_yaw,
                        pos_tolerance=getattr(goal, "slalom_script_pos_tolerance", None),
                    )
                )
            if route_dist >= self.cfg.slalom_script_min_step_distance:
                forward = math.cos(target_yaw) * route_dx + math.sin(target_yaw) * route_dy
                left = -math.sin(target_yaw) * route_dx + math.cos(target_yaw) * route_dy
                steps.append(
                    SlalomScriptStep(
                        kind="drive",
                        start_index=index,
                        end_index=index + 1,
                        target_x=next_goal.x,
                        target_y=next_goal.y,
                        target_yaw=target_yaw,
                        pos_tolerance=getattr(next_goal, "slalom_script_pos_tolerance", None),
                        forward=forward,
                        left=left,
                    )
                )

        final_goal = self.waypoints[end_index]
        if self.cfg.slalom_script_final_rotate_enabled:
            steps.append(
                SlalomScriptStep(
                    kind="rotate",
                    start_index=end_index,
                    end_index=end_index,
                    target_x=final_goal.x,
                    target_y=final_goal.y,
                    target_yaw=math.radians(final_goal.yaw_deg),
                    pos_tolerance=getattr(final_goal, "slalom_script_pos_tolerance", None),
                )
            )
        return steps

    def maybe_start_slalom_script(self, x: float, y: float, sim_time: float) -> bool:
        if not self.cfg.slalom_script_enabled or self.slalom_script_active:
            return self.slalom_script_active
        active = self.active
        if not self._is_slalom_script_waypoint(active):
            return False
        if math.hypot(active.x - x, active.y - y) > self.cfg.slalom_script_start_tolerance:
            return False

        start_index, end_index = self.get_slalom_script_bounds(self.index)
        if end_index <= start_index:
            return False
        steps = self.build_slalom_script_steps(start_index, end_index)
        if not steps:
            return False

        self.slalom_script_active = True
        self.slalom_script_start_index = start_index
        self.slalom_script_end_index = end_index
        self.slalom_script_steps = steps
        self.slalom_script_step_index = 0
        self.slalom_script_step_stable_count = 0
        self.turn_in_place = False
        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0
        self.wp_start_time = sim_time
        return True

    def compute_slalom_script_command(
        self,
        step: SlalomScriptStep,
        x: float,
        y: float,
        yaw: float,
    ) -> tuple[np.ndarray, bool]:
        cmd = np.zeros(3, dtype=np.float32)
        target_yaw = step.target_yaw
        yaw_tolerance = math.radians(self.cfg.slalom_script_yaw_tolerance_deg)

        if step.kind == "rotate":
            if target_yaw is None:
                return cmd, True
            yaw_err = self.normalize_angle(target_yaw - yaw)
            if abs(yaw_err) <= yaw_tolerance:
                return cmd, True
            cmd[2] = self.clamp(
                self.cfg.slalom_script_kp_yaw * yaw_err,
                -self.cfg.slalom_script_max_wz,
                self.cfg.slalom_script_max_wz,
            )
            cmd[2] = self.apply_min_command(
                float(cmd[2]),
                self.cfg.slalom_script_min_cmd_angular,
                self.cfg.slalom_script_max_wz,
                self.cfg.slalom_script_min_cmd_epsilon,
            )
            return cmd, False

        dx = step.target_x - x
        dy = step.target_y - y
        if target_yaw is None:
            target_yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1.0e-6 else yaw
        yaw_err = self.normalize_angle(target_yaw - yaw)
        pos_err = math.hypot(dx, dy)
        pos_tolerance = (
            self.cfg.slalom_script_pos_tolerance
            if step.pos_tolerance is None
            else max(0.01, float(step.pos_tolerance))
        )
        complete = (
            pos_err <= pos_tolerance
            and (
                not self.cfg.slalom_script_require_yaw_at_step
                or abs(yaw_err) <= yaw_tolerance
            )
        )
        if complete:
            return cmd, True

        cmd[2] = self.clamp(
            self.cfg.slalom_script_kp_yaw * yaw_err,
            -self.cfg.slalom_script_max_wz,
            self.cfg.slalom_script_max_wz,
        )
        cmd[2] = self.apply_min_command(
            float(cmd[2]),
            self.cfg.slalom_script_min_cmd_angular,
            self.cfg.slalom_script_max_wz,
            self.cfg.slalom_script_min_cmd_epsilon,
        )
        if abs(yaw_err) <= math.radians(self.cfg.slalom_script_yaw_gate_deg):
            err_forward = math.cos(yaw) * dx + math.sin(yaw) * dy
            err_left = -math.sin(yaw) * dx + math.cos(yaw) * dy
            yaw_scale = max(0.35, math.cos(yaw_err) ** 2)
            curvature_scale = 1.0
            if self.cfg.slalom_script_curvature_speed_enabled:
                gate = max(1.0e-6, math.radians(self.cfg.slalom_script_yaw_gate_deg))
                curvature_scale = max(
                    self.cfg.slalom_script_curvature_min_scale,
                    1.0 - abs(yaw_err) / gate,
                )
            cmd[0] = self.clamp(
                self.cfg.slalom_script_kp_dist * err_forward,
                -self.cfg.slalom_script_max_vx * yaw_scale * curvature_scale,
                self.cfg.slalom_script_max_vx * yaw_scale * curvature_scale,
            )
            cmd[1] = self.clamp(
                self.cfg.slalom_script_kp_dist * err_left,
                -self.cfg.slalom_script_max_vy * curvature_scale,
                self.cfg.slalom_script_max_vy * curvature_scale,
            )
            cmd[0] = self.apply_min_command(
                float(cmd[0]),
                self.cfg.slalom_script_min_cmd_linear,
                self.cfg.slalom_script_max_vx,
                self.cfg.slalom_script_min_cmd_epsilon,
            )
            cmd[1] = self.apply_min_command(
                float(cmd[1]),
                self.cfg.slalom_script_min_cmd_linear,
                self.cfg.slalom_script_max_vy,
                self.cfg.slalom_script_min_cmd_epsilon,
            )
        return cmd, False

    @classmethod
    def apply_min_command(
        cls,
        value: float,
        min_abs: float,
        max_abs: float,
        epsilon: float,
    ) -> float:
        if max_abs <= 0.0 or min_abs <= 0.0:
            return value
        abs_value = abs(value)
        if abs_value < max(0.0, epsilon) or abs_value >= min_abs:
            return value
        return cls.clamp(math.copysign(min_abs, value), -max_abs, max_abs)

    def finish_slalom_script(self, sim_time: float) -> tuple[np.ndarray, str | None]:
        end_index = self.slalom_script_end_index
        self.reset_slalom_script()
        self.index = end_index + 1
        self.wp_start_time = sim_time
        self.turn_in_place = False
        self.goal_entered_tolerance = False
        self.goal_complete_stable_count = 0
        self.best_dist = float("inf")
        self.last_progress_time = sim_time
        if self.index >= len(self.waypoints):
            return np.zeros(3, dtype=np.float32), "complete"
        return np.zeros(3, dtype=np.float32), None

    def handle_slalom_script(
        self,
        x: float,
        y: float,
        yaw: float,
        sim_time: float,
    ) -> tuple[np.ndarray, str | None] | None:
        if not self.slalom_script_active and not self.maybe_start_slalom_script(x, y, sim_time):
            return None
        if sim_time - self.wp_start_time > self.cfg.waypoint_timeout_s:
            active = self.active
            active_id = active.id if active is not None else "unknown"
            return np.zeros(3, dtype=np.float32), (
                f"timeout at slalom step {self.slalom_script_step_index + 1}/"
                f"{len(self.slalom_script_steps)} near waypoint {active_id}"
            )

        while self.slalom_script_step_index < len(self.slalom_script_steps):
            step = self.slalom_script_steps[self.slalom_script_step_index]
            cmd, complete = self.compute_slalom_script_command(step, x, y, yaw)
            if complete:
                self.slalom_script_step_stable_count += 1
                if self.slalom_script_step_stable_count >= self.cfg.slalom_script_stable_cycles:
                    self.slalom_script_step_index += 1
                    self.index = max(self.index, min(step.end_index, self.slalom_script_end_index))
                    self.wp_start_time = sim_time
                    self.slalom_script_step_stable_count = 0
                    continue
            else:
                self.slalom_script_step_stable_count = 0
            return cmd, None

        return self.finish_slalom_script(sim_time)

    def update(self, x: float, y: float, yaw: float, sim_time: float) -> tuple[np.ndarray, str | None]:
        script_result = self.handle_slalom_script(x, y, yaw, sim_time)
        if script_result is not None:
            return script_result
        return super().update(x, y, yaw, sim_time)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run MuJoCo+ONNX route validation.")
    parser.add_argument("--terrain-xml", type=Path, default=PROJECT_ROOT / "tools/nav_tools/xml/1hao.xml")
    parser.add_argument("--points", type=Path, default=PROJECT_ROOT / "tools/nav_tools/points/points_20260715_120154.json")
    parser.add_argument("--onnx", type=Path, default=PROJECT_ROOT / "model_6800.onnx")
    parser.add_argument("--crawl-onnx", type=Path, default=PROJECT_ROOT / "model_crawl.onnx")
    parser.add_argument(
        "--policy-backend",
        choices=("policy-runner", "onnx", "rough-ik-crawl"),
        default="policy-runner",
        help="policy-runner matches nav_sim2sim.py; rough-ik-crawl matches sim2real nav_good crawl_backend=ik.",
    )
    parser.add_argument("--enable-policy-switch", action="store_true")
    parser.add_argument(
        "--mission",
        choices=("json", "slalom", "grand"),
        default="json",
        help="json uses --points; slalom/grand use the hardcoded nav_sim2sim coordinate frame.",
    )
    parser.add_argument("--robot-xml", type=Path, default=PROJECT_ROOT / "mjcf/wheelleg.xml")
    parser.add_argument("--hfield-dir", type=Path, default=PROJECT_ROOT / "sim2sim/terrain")
    parser.add_argument("--out-dir", type=Path, default=PROJECT_ROOT / "sim2sim/route_check_runs")
    parser.add_argument("--avoid-margin", type=float, default=0.05)
    parser.add_argument("--footprint-radius", type=float, default=None)
    parser.add_argument("--max-time", type=float, default=420.0)
    parser.add_argument("--waypoint-timeout", type=float, default=30.0)
    parser.add_argument("--max-vx", type=float, default=1.2)
    parser.add_argument("--max-vy", type=float, default=0.0)
    parser.add_argument("--max-wz", type=float, default=0.8)
    parser.add_argument("--kp-dist", type=float, default=0.8)
    parser.add_argument("--kp-yaw", type=float, default=1.8)
    parser.add_argument("--yaw-stop-threshold-deg", type=float, default=45.0)
    parser.add_argument("--turn-in-place-enter-deg", type=float, default=70.0)
    parser.add_argument("--turn-in-place-exit-deg", type=float, default=18.0)
    parser.add_argument("--follower", choices=("waypoint", "pure-pursuit", "nav-good", "nav-sim2sim", "slalom-script", "nav-script"), default="pure-pursuit")
    parser.add_argument("--lookahead", type=float, default=0.45)
    parser.add_argument("--min-cmd-vx", type=float, default=0.08)
    parser.add_argument("--creep-cmd-vx", type=float, default=0.04)
    parser.add_argument("--cmd-vx-scale", type=float, default=1.0)
    parser.add_argument("--stuck-timeout", type=float, default=4.0)
    parser.add_argument("--recovery-duration", type=float, default=2.0)
    parser.add_argument("--max-recoveries", type=int, default=4)
    parser.add_argument("--slalom-script", action="store_true", default=True)
    parser.add_argument("--no-slalom-script", dest="slalom_script", action="store_false")
    parser.add_argument("--slalom-script-start-tolerance", type=float, default=0.10)
    parser.add_argument("--slalom-script-pos-tolerance", type=float, default=0.08)
    parser.add_argument("--slalom-script-yaw-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--slalom-script-stable-cycles", type=int, default=1)
    parser.add_argument("--slalom-script-rotate-steps", action="store_true", default=False)
    parser.add_argument("--slalom-script-final-rotate", action="store_true", default=False)
    parser.add_argument("--slalom-script-require-yaw-at-step", action="store_true", default=False)
    parser.add_argument("--slalom-script-kp-dist", type=float, default=1.0)
    parser.add_argument("--slalom-script-kp-yaw", type=float, default=1.6)
    parser.add_argument("--slalom-script-max-vx", type=float, default=0.60)
    parser.add_argument("--slalom-script-max-vy", type=float, default=0.50)
    parser.add_argument("--slalom-script-max-wz", type=float, default=0.50)
    parser.add_argument("--slalom-script-min-cmd-linear", type=float, default=0.2)
    parser.add_argument("--slalom-script-min-cmd-angular", type=float, default=0.0)
    parser.add_argument("--slalom-script-min-cmd-epsilon", type=float, default=0.05)
    parser.add_argument("--slalom-script-min-step-distance", type=float, default=0.02)
    parser.add_argument("--slalom-script-yaw-gate-deg", type=float, default=25.0)
    parser.add_argument("--slalom-script-curvature-speed", action="store_true", default=True)
    parser.add_argument(
        "--no-slalom-script-curvature-speed",
        dest="slalom_script_curvature_speed",
        action="store_false",
    )
    parser.add_argument("--slalom-script-curvature-min-scale", type=float, default=0.55)
    parser.add_argument(
        "--slalom-script-drive-yaw-source",
        choices=("current", "next", "segment", "blend"),
        default="segment",
        help="Yaw used while driving each slalom segment.",
    )
    parser.add_argument("--local-safety", action="store_true", default=False)
    parser.add_argument("--no-local-safety", dest="local_safety", action="store_false")
    parser.add_argument("--safety-horizon", type=float, default=1.0)
    parser.add_argument("--safety-dt", type=float, default=0.1)
    parser.add_argument(
        "--start-yaw-offset-deg",
        type=float,
        default=0.0,
        help="Added to route yawDeg when placing the robot.",
    )
    parser.add_argument(
        "--heading-offset-deg",
        type=float,
        default=0.0,
        help="Added to MuJoCo base yaw before waypoint heading control.",
    )
    parser.add_argument("--sample-every", type=int, default=5)
    parser.add_argument("--sim-speed", type=float, default=1.0, help="Simulation wall-clock speed factor, matching nav_sim2sim.py timing semantics.")
    parser.add_argument("--viewer", action="store_true", help="Open MuJoCo passive viewer while the check runs.")
    parser.add_argument("--viewer-sync-every", type=int, default=2)
    parser.add_argument("--dashboard", action="store_true", help="Open a pygame telemetry dashboard with route, avoid regions, and trail.")
    parser.add_argument("--dashboard-fps", type=float, default=30.0)
    parser.add_argument("--dashboard-sim-speed", type=float, default=1.0, help="Initial dashboard simulation speed factor; can be changed in the UI up to 10x.")
    parser.add_argument("--dashboard-width", type=int, default=1180)
    parser.add_argument("--dashboard-height", type=int, default=760)
    parser.add_argument("--dashboard-trail", type=int, default=6000)
    parser.add_argument("--start-index", type=int, default=1, help="1-based waypoint index to start testing from.")
    parser.add_argument("--end-index", type=int, default=None, help="1-based waypoint index to stop testing at.")
    parser.add_argument("--start-id", type=str, default=None, help="Waypoint id to start testing from; overrides --start-index.")
    parser.add_argument("--end-id", type=str, default=None, help="Waypoint id to stop testing at; overrides --end-index.")
    parser.add_argument(
        "--skip-initial-waypoint",
        action="store_true",
        help="Use the first sliced waypoint as spawn pose, then navigate to the next waypoint.",
    )
    parser.add_argument(
        "--auto-skip-slice-start",
        action="store_true",
        default=True,
        help="When --start-index > 1, skip the sliced start waypoint by default.",
    )
    parser.add_argument("--no-auto-skip-slice-start", dest="auto_skip_slice_start", action="store_false")
    parser.add_argument("--start-z", type=float, default=0.75, help="Initial base z before settling.")
    parser.add_argument("--start-x-offset", type=float, default=0.0, help="World x offset added to the initial spawn pose.")
    parser.add_argument("--start-y-offset", type=float, default=0.0, help="World y offset added to the initial spawn pose.")
    parser.add_argument("--settle-steps", type=int, default=500)
    parser.add_argument("--min-base-z", type=float, default=0.12)
    parser.add_argument("--stop-on-clearance-violation", action="store_true")
    parser.add_argument("--deployment-randomization", action="store_true")
    parser.add_argument("--random-seed", type=int, default=1)
    parser.add_argument("--pose-noise-xy", type=float, default=0.015)
    parser.add_argument("--pose-noise-yaw-deg", type=float, default=1.0)
    parser.add_argument("--command-delay-ms", type=float, default=40.0)
    parser.add_argument("--action-delay-ms", type=float, default=20.0)
    parser.add_argument("--control-jitter-ms", type=float, default=4.0)
    parser.add_argument("--command-dropout-prob", type=float, default=0.01)
    parser.add_argument("--push-force", type=float, default=25.0)
    parser.add_argument("--push-interval", type=float, default=8.0)
    parser.add_argument("--push-duration", type=float, default=0.08)
    parser.add_argument("--no-csv", action="store_true")
    return parser.parse_args()


def make_combined_terrain_xml(terrain_xml: Path, work_dir: Path) -> Path:
    text = terrain_xml.read_text(encoding="utf-8")
    if '<include file="go2w.xml"/>' not in text and '<include file="go2w.xml" />' not in text:
        end = text.find(">")
        if end < 0:
            raise ValueError(f"Invalid terrain XML: {terrain_xml}")
        text = text[: end + 1] + '\n<include file="go2w.xml"/>' + text[end + 1 :]
    out_path = work_dir / f"{terrain_xml.stem}_with_robot_include.xml"
    out_path.write_text(text, encoding="utf-8")
    return out_path


def builtin_mission_waypoints(name: str) -> list[Waypoint]:
    slalom = [
        (2.9, -9.5, 0.4, "rough"),
        (1.8, -9.95, 0.4, "rough"),
        (0.9, -10.0, 0.4, "rough"),
        (0.8, -10.8, 0.4, "rough"),
        (0.9, -11.0, 0.4, "rough"),
        (2.7, -11.0, 0.4, "rough"),
        (2.8, -11.8, 0.4, "rough"),
        (2.7, -12.0, 0.4, "rough"),
        (0.9, -12.0, 0.4, "rough"),
        (0.9, -12.75, 0.4, "rough"),
        (1.42, -12.82, 0.4, "rough"),
        (2.3, -13.3, 0.4, "rough"),
        (2.3, -11.5, 0.4, "rough"),
        (3.32, -12.44, 0.4, "rough"),
        (3.7, -9.0, 0.6, "rough"),
    ]
    grand_tail = [
        (5.7, -10.2, 0.5, "rough"),
        (5.7, -9.0, 0.3, "crawl"),
        (5.7, -7.8, 0.3, "rough"),
        (5.7, -7.0, 0.5, "rough"),
        (3.7, -9.0, 0.5, "rough"),
        (3.7, -12.5, 0.5, "rough"),
        (4.84, -12.5, 0.4, "rough"),
        (5.84, -12.0, 0.4, "rough"),
        (5.84, -10.2, 0.4, "rough"),
        (3.3, -10.2, 0.5, "rough"),
        (3.3, -9.0, 0.5, "rough"),
        (3.7, -9.0, 0.5, "rough"),
        (3.3, -9.0, 0.5, "rough"),
        (1.8, -9.0, 0.5, "rough"),
        (1.8, -8.0, 0.4, "rough"),
        (1.8, -7.5, 0.4, "rough"),
        (1.8, -5.8, 1.2, "rough"),
        (1.8, -3.5, 0.4, "rough"),
        (1.8, -1.6, 0.4, "rough"),
        (1.8, -1.38, 0.4, "rough"),
        (1.8, 0.0, 0.4, "rough"),
        (3.2, 0.0, 0.4, "rough"),
        (4.85, 0.0, 0.4, "rough"),
        (5.7, -0.5, 0.4, "rough"),
        (5.7, -2.25, 0.4, "rough"),
        (5.7, -3.5, 0.4, "rough"),
        (5.7, -5.0, 0.4, "rough"),
        (5.7, -7.0, 0.4, "rough"),
        (3.3, -7.0, 0.5, "rough"),
        (3.3, -9.0, 0.5, "rough"),
        (3.7, -9.0, 0.6, "rough"),
    ]
    rows = slalom if name == "slalom" else slalom + grand_tail
    return [
        Waypoint(
            index=i,
            id=f"{name}_{i}",
            x=x,
            y=y,
            yaw_deg=0.0,
            speed=speed,
            policy=policy,
            tolerance=0.15,
        )
        for i, (x, y, speed, policy) in enumerate(rows, start=1)
    ]


def load_route(path: Path) -> tuple[list[Waypoint], list[AvoidRegion]]:
    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    if not isinstance(payload, dict):
        raise ValueError(f"Route JSON root must be object: {path}")
    return load_waypoints(payload), load_regions(payload)


def load_requested_route(args: argparse.Namespace) -> tuple[list[Waypoint], list[AvoidRegion]]:
    if args.mission == "json":
        waypoints, regions = load_route(args.points)
    else:
        waypoints, regions = builtin_mission_waypoints(args.mission), []

    id_to_pos = {wp.id: i + 1 for i, wp in enumerate(waypoints)}
    start = id_to_pos.get(str(args.start_id), max(1, int(args.start_index))) if args.start_id is not None else max(1, int(args.start_index))
    end_default = int(args.end_index) if args.end_index is not None else len(waypoints)
    end = id_to_pos.get(str(args.end_id), end_default) if args.end_id is not None else end_default
    if start > end or start > len(waypoints):
        raise ValueError(f"Invalid waypoint slice: start={start} end={end} total={len(waypoints)}")
    sliced = waypoints[start - 1 : min(end, len(waypoints))]
    if len(sliced) < 1:
        raise ValueError("Waypoint slice is empty")
    return sliced, regions


def initial_pose_for_route(args: argparse.Namespace, waypoints: list[Waypoint]) -> tuple[float, float, float]:
    if args.mission == "json":
        start = waypoints[0]
        return start.x, start.y, start.yaw_deg + float(args.start_yaw_offset_deg)
    return 3.7, -9.0, 0.0


def set_robot_pose(
    io: MuJoCoIO,
    x: float,
    y: float,
    yaw_deg: float,
    z: float,
    settle_steps: int,
    default_dof_pos: np.ndarray,
) -> None:
    yaw = math.radians(yaw_deg)
    io.d.qpos[0] = x
    io.d.qpos[1] = y
    io.d.qpos[2] = z
    io.d.qpos[3:7] = [math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)]
    io.d.qpos[io.qpos_ids] = default_dof_pos
    io.d.qvel[:] = 0.0
    io.d.ctrl[io.ctrl_ids[:12]] = default_dof_pos[:12]
    io.d.ctrl[io.ctrl_ids[12:]] = 0.0
    mujoco.mj_forward(io.m, io.d)
    for _ in range(max(0, int(settle_steps))):
        mujoco.mj_step(io.m, io.d)


def clearance_to_regions(
    x: float,
    y: float,
    regions: list[AvoidRegion],
) -> tuple[float, str]:
    best = float("inf")
    best_name = ""
    for region in regions:
        polygon = region.polygon
        if len(polygon) < 3:
            continue
        if point_in_polygon(x, y, polygon):
            return 0.0, region.name
        dist = min(
            point_segment_distance(x, y, ax, ay, bx, by)
            for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1])
        )
        if dist < best:
            best = dist
            best_name = region.name
    return best, best_name


def safety_filter_command(
    desired: np.ndarray,
    x: float,
    y: float,
    yaw: float,
    target: Waypoint | None,
    regions: list[AvoidRegion],
    required_clearance: float,
    cfg: SimConfig,
    horizon_s: float,
    dt_s: float,
) -> np.ndarray:
    if not regions or target is None:
        return desired

    base_vx = float(desired[0])
    base_vy = float(desired[1])
    base_wz = float(desired[2])
    vx_abs = abs(base_vx)
    vy_abs = abs(base_vy)
    vx_sign = -1.0 if base_vx < 0.0 else 1.0
    vy_sign = -1.0 if base_vy < 0.0 else 1.0
    vx_samples = [0.0, 0.35 * vx_abs, 0.65 * vx_abs, vx_abs]
    vy_samples = [0.0, 0.5 * vy_abs, vy_abs]
    wz_span = max(0.35, min(cfg.max_wz, abs(base_wz) + 0.35))
    wz_samples = [
        base_wz - wz_span,
        base_wz - 0.5 * wz_span,
        base_wz,
        base_wz + 0.5 * wz_span,
        base_wz + wz_span,
    ]
    best_cmd = desired.copy()
    best_score = -float("inf")
    initial_dist = math.hypot(target.x - x, target.y - y)
    steps = max(1, int(round(horizon_s / dt_s)))

    for vx_mag in vx_samples:
        for vy_mag in vy_samples:
            for wz in wz_samples:
                px, py, pyaw = x, y, yaw
                min_margin = float("inf")
                for _ in range(steps):
                    vx = vx_sign * vx_mag
                    vy = vy_sign * vy_mag
                    px += (vx * math.cos(pyaw) - vy * math.sin(pyaw)) * dt_s
                    py += (vx * math.sin(pyaw) + vy * math.cos(pyaw)) * dt_s
                    pyaw = WaypointFollower.normalize_angle(pyaw + wz * dt_s)
                    clearance, _ = clearance_to_regions(px, py, regions)
                    min_margin = min(min_margin, clearance - required_clearance)
                final_dist = math.hypot(target.x - px, target.y - py)
                progress = initial_dist - final_dist
                yaw_cost = abs(wz - base_wz)
                speed_cost = abs(vx_mag - vx_abs) + abs(vy_mag - vy_abs)
                risk_penalty = 20.0 * max(0.0, -min_margin)
                margin_reward = min(0.4, min_margin)
                score = 3.0 * progress + margin_reward - risk_penalty - 0.12 * yaw_cost - 0.08 * speed_cost
                if score > best_score:
                    best_score = score
                    best_cmd = np.array(
                        [
                            vx_sign * vx_mag,
                            vy_sign * vy_mag,
                            WaypointFollower.clamp(wz, -cfg.max_wz, cfg.max_wz),
                        ],
                        dtype=np.float32,
                    )
    return best_cmd


def robot_yaw_from_qpos(qpos: np.ndarray) -> float:
    qw, qx, qy, qz = qpos[3:7]
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_to_euler_wxyz(quat: np.ndarray) -> tuple[float, float, float]:
    qw, qx, qy, qz = [float(v) for v in quat]
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    yaw = robot_yaw_from_qpos(np.array([0.0, 0.0, 0.0, qw, qx, qy, qz], dtype=np.float64))
    return roll, pitch, yaw


@dataclass(frozen=True)
class DashboardGeom:
    name: str
    kind: str
    pos: tuple[float, ...]
    size: tuple[float, ...]
    quat: tuple[float, float, float, float]
    rgba: tuple[float, float, float, float]
    collidable: bool


def parse_dashboard_geoms(xml_path: Path) -> list[DashboardGeom]:
    geoms: list[DashboardGeom] = []
    if not xml_path.exists():
        return geoms
    try:
        root = ET.parse(xml_path).getroot()
    except ET.ParseError:
        return geoms
    for index, geom in enumerate(root.iter("geom"), start=1):
        name = geom.get("name", f"geom_{index}")
        if name == "floor":
            continue
        kind = geom.get("type", "box")
        pos_text = geom.get("pos")
        size_text = geom.get("size")
        if not pos_text or not size_text:
            continue
        try:
            pos = tuple(float(v) for v in pos_text.split())
            size = tuple(float(v) for v in size_text.split())
            quat_values = tuple(float(v) for v in geom.get("quat", "1 0 0 0").split())
            rgba_values = tuple(float(v) for v in geom.get("rgba", "0.65 0.65 0.65 1").split())
        except ValueError:
            continue
        if len(pos) < 2 or len(size) < 1 or len(quat_values) != 4:
            continue
        rgba = rgba_values if len(rgba_values) == 4 else (0.65, 0.65, 0.65, 1.0)
        collidable = geom.get("contype", "1") != "0" and geom.get("conaffinity", "1") != "0"
        geoms.append(
            DashboardGeom(
                name=name,
                kind=kind,
                pos=pos,
                size=size,
                quat=quat_values,  # type: ignore[arg-type]
                rgba=rgba,  # type: ignore[arg-type]
                collidable=collidable,
            )
        )
    return geoms


def quat_yaw_wxyz(quat: tuple[float, float, float, float]) -> float:
    qw, qx, qy, qz = quat
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def local_rect_polygon(
    origin_x: float,
    origin_y: float,
    yaw: float,
    center_x: float,
    center_y: float,
    length: float,
    width: float,
) -> list[tuple[float, float]]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    hx = length * 0.5
    hy = width * 0.5
    points = []
    for lx, ly in [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]:
        px = center_x + lx
        py = center_y + ly
        points.append((origin_x + px * c - py * s, origin_y + px * s + py * c))
    return points


def robot_wheel_local_points() -> list[tuple[str, float, float]]:
    wheels = []
    for name, (pitch_x, pitch_y, knee_y, wheel_y, wheel_geom_y, knee_x) in ROBOT_WHEEL_POSITIONS.items():
        x = ROBOT_BODY_CENTER_OFFSET_X + pitch_x + knee_x
        y = pitch_y + knee_y + wheel_y + wheel_geom_y
        wheels.append((name, x, y))
    return wheels


class DashboardButton:
    def __init__(self, rect: tuple[int, int, int, int], label: str, action: str) -> None:
        self.rect_tuple = rect
        self.label = label
        self.action = action

    def rect(self, pygame: Any) -> Any:
        return pygame.Rect(*self.rect_tuple)


class PygameRouteDashboard:
    def __init__(
        self,
        waypoints: list[Waypoint],
        regions: list[AvoidRegion],
        terrain_xml: Path,
        width: int,
        height: int,
        fps: float,
        max_trail: int,
        control_hz: float,
        initial_sim_speed: float,
    ) -> None:
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        # Older pkg_resources versions used by some pygame installs still
        # reference pkgutil.ImpImporter, which was removed in Python 3.12.
        if not hasattr(pkgutil, "ImpImporter"):
            import zipimport
            pkgutil.ImpImporter = zipimport.zipimporter  # type: ignore[attr-defined]
        import pygame

        self.pygame = pygame
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
        pygame.display.set_caption("nav_route_sim2sim_check dashboard")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 19)
        self.title_font = pygame.font.Font(None, 28)
        self.waypoints = waypoints
        self.regions = regions
        self.geoms = parse_dashboard_geoms(terrain_xml)
        self.width = width
        self.height = height
        self.panel_width = 400
        self.fps = max(1.0, float(fps))
        self.control_hz = max(1.0, float(control_hz))
        self.max_trail = max(50, int(max_trail))
        self.trail: list[tuple[float, float]] = []
        self.zoom = 60.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.speed_scale = 1.0
        self.sim_speed = max(0.1, min(10.0, float(initial_sim_speed)))
        self.rewind_seconds = 0.0
        self.paused = False
        self.buttons: list[DashboardButton] = []
        self._dragging = False
        self._drag_start: tuple[int, int] | None = None
        self._last_draw_time = 0.0
        self._fit_view()

    @property
    def map_width(self) -> int:
        return max(240, self.width - self.panel_width)

    def _fit_view(self) -> None:
        points: list[tuple[float, float]] = [(wp.x, wp.y) for wp in self.waypoints]
        for region in self.regions:
            points.extend(region.polygon)
        for geom in self.geoms:
            if len(geom.pos) >= 2:
                points.append((geom.pos[0], geom.pos[1]))
        if not points:
            return
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        span_x = max(1.0, max(xs) - min(xs))
        span_y = max(1.0, max(ys) - min(ys))
        margin = 70
        self.zoom = max(
            12.0,
            min(
                120.0,
                min((self.map_width - margin * 2) / span_x, (self.height - margin * 2) / span_y),
            ),
        )
        center_x = (min(xs) + max(xs)) * 0.5
        center_y = (min(ys) + max(ys)) * 0.5
        self.pan_x = -center_x * self.zoom
        self.pan_y = center_y * self.zoom

    def _screen(self, x: float, y: float) -> tuple[int, int]:
        sx = int(self.map_width / 2.0 + x * self.zoom + self.pan_x)
        sy = int(self.height / 2.0 - y * self.zoom + self.pan_y)
        return sx, sy

    def _draw_text(self, text: str, x: int, y: int, color: tuple[int, int, int] = (226, 232, 240), small: bool = False) -> None:
        font = self.small_font if small else self.font
        self.screen.blit(font.render(text, True, color), (x, y))

    def _draw_card(self, rect: tuple[int, int, int, int], title: str | None = None) -> None:
        pygame = self.pygame
        pygame.draw.rect(self.screen, (20, 28, 48), rect, border_radius=7)
        pygame.draw.rect(self.screen, (47, 61, 90), rect, width=1, border_radius=7)
        if title:
            self._draw_text(title, rect[0] + 12, rect[1] + 9, (203, 213, 225), small=True)

    def _draw_bar(self, x: int, y: int, w: int, value: float, lo: float, hi: float, color: tuple[int, int, int]) -> None:
        pygame = self.pygame
        frac = 0.0 if hi <= lo else max(0.0, min(1.0, (value - lo) / (hi - lo)))
        pygame.draw.rect(self.screen, (8, 13, 25), (x, y, w, 8), border_radius=4)
        pygame.draw.rect(self.screen, color, (x, y, int(w * frac), 8), border_radius=4)

    def _rgba_to_color(self, rgba: tuple[float, float, float, float], alpha: int | None = None) -> tuple[int, int, int] | tuple[int, int, int, int]:
        rgb = tuple(max(0, min(255, int(v * 255))) for v in rgba[:3])
        if alpha is None:
            return rgb
        return (*rgb, alpha)

    def _draw_grid(self) -> None:
        pygame = self.pygame
        bg = (9, 14, 26)
        grid = (28, 36, 58)
        axis = (58, 72, 106)
        self.screen.fill(bg)
        step = 1.0
        left_world = -(self.map_width / 2.0 + self.pan_x) / self.zoom
        right_world = (self.map_width / 2.0 - self.pan_x) / self.zoom
        bottom_world = -(self.height / 2.0 - self.pan_y) / self.zoom
        top_world = (self.height / 2.0 + self.pan_y) / self.zoom
        gx = math.floor(left_world / step) * step
        while gx <= right_world:
            sx, _ = self._screen(gx, 0.0)
            pygame.draw.line(self.screen, axis if abs(gx) < 1e-6 else grid, (sx, 0), (sx, self.height), 2 if abs(gx) < 1e-6 else 1)
            gx += step
        gy = math.floor(bottom_world / step) * step
        while gy <= top_world:
            _, sy = self._screen(0.0, gy)
            pygame.draw.line(self.screen, axis if abs(gy) < 1e-6 else grid, (0, sy), (self.map_width, sy), 2 if abs(gy) < 1e-6 else 1)
            gy += step

    def _draw_map(self, telemetry: dict[str, Any]) -> None:
        pygame = self.pygame
        self._draw_grid()
        self._draw_xml_geoms()
        overlay = pygame.Surface((self.map_width, self.height), pygame.SRCALPHA)
        for region in self.regions:
            pts = [self._screen(px, py) for px, py in region.polygon]
            if len(pts) >= 3:
                pygame.draw.polygon(overlay, (244, 63, 94, 72), pts)
                pygame.draw.polygon(self.screen, (251, 113, 133), pts, width=2)
        self.screen.blit(overlay, (0, 0))

        route = [self._screen(wp.x, wp.y) for wp in self.waypoints]
        if len(route) >= 2:
            pygame.draw.lines(self.screen, (14, 165, 233), False, route, width=2)
        active_index = int(telemetry.get("wp_index", 0))
        for index, wp in enumerate(self.waypoints):
            color = (148, 163, 184)
            radius = 4
            if index < active_index:
                color = (34, 197, 94)
            elif index == active_index:
                color = (250, 204, 21)
                radius = 7
            pygame.draw.circle(self.screen, color, self._screen(wp.x, wp.y), radius)

        if len(self.trail) >= 2:
            pygame.draw.lines(self.screen, (34, 197, 94), False, [self._screen(x, y) for x, y in self.trail], width=3)

        x = float(telemetry.get("x", 0.0))
        y = float(telemetry.get("y", 0.0))
        yaw = float(telemetry.get("yaw", 0.0))
        self._draw_robot_pose(x, y, yaw)

    def _draw_robot_pose(self, x: float, y: float, yaw: float) -> None:
        pygame = self.pygame
        body = local_rect_polygon(
            x,
            y,
            yaw,
            ROBOT_BODY_CENTER_OFFSET_X,
            0.0,
            ROBOT_BODY_LENGTH,
            ROBOT_BODY_WIDTH,
        )
        body_screen = [self._screen(px, py) for px, py in body]
        pygame.draw.polygon(self.screen, (30, 41, 59), body_screen)
        pygame.draw.polygon(self.screen, (226, 232, 240), body_screen, width=2)

        c = math.cos(yaw)
        s = math.sin(yaw)
        for _, lx, ly in robot_wheel_local_points():
            wx = x + lx * c - ly * s
            wy = y + lx * s + ly * c
            wheel = local_rect_polygon(
                wx,
                wy,
                yaw,
                0.0,
                0.0,
                ROBOT_WHEEL_VIS_LENGTH,
                ROBOT_WHEEL_VIS_WIDTH,
            )
            pygame.draw.polygon(self.screen, (2, 6, 23), [self._screen(px, py) for px, py in wheel])

        origin = self._screen(x, y)
        nose = self._screen(x + 0.32 * math.cos(yaw), y + 0.32 * math.sin(yaw))
        radius = max(3, int(self.zoom * 0.025))
        pygame.draw.circle(self.screen, (45, 212, 191), origin, radius)
        pygame.draw.line(self.screen, (45, 212, 191), (origin[0] - radius - 2, origin[1]), (origin[0] + radius + 2, origin[1]), 1)
        pygame.draw.line(self.screen, (45, 212, 191), (origin[0], origin[1] - radius - 2), (origin[0], origin[1] + radius + 2), 1)
        pygame.draw.line(self.screen, (240, 253, 250), origin, nose, 2)

    def _draw_xml_geoms(self) -> None:
        pygame = self.pygame
        geom_layer = pygame.Surface((self.map_width, self.height), pygame.SRCALPHA)
        for geom in self.geoms:
            if len(geom.pos) < 2 or not geom.size:
                continue
            color = self._rgba_to_color(geom.rgba, 130 if geom.collidable else 55)
            outline = self._rgba_to_color(geom.rgba)
            if geom.kind == "box" and len(geom.size) >= 2:
                cx, cy = geom.pos[0], geom.pos[1]
                sx, sy = geom.size[0], geom.size[1]
                yaw = quat_yaw_wxyz(geom.quat)
                points = []
                for lx, ly in [(-sx, -sy), (sx, -sy), (sx, sy), (-sx, sy)]:
                    wx = cx + lx * math.cos(yaw) - ly * math.sin(yaw)
                    wy = cy + lx * math.sin(yaw) + ly * math.cos(yaw)
                    points.append(self._screen(wx, wy))
                pygame.draw.polygon(geom_layer, color, points)
                pygame.draw.polygon(self.screen, outline, points, width=1)
            elif geom.kind == "cylinder":
                cx, cy = self._screen(geom.pos[0], geom.pos[1])
                radius = max(2, int(abs(geom.size[0]) * self.zoom))
                pygame.draw.circle(geom_layer, color, (cx, cy), radius)
                pygame.draw.circle(self.screen, outline, (cx, cy), radius, width=1)
            elif geom.kind == "plane":
                continue
        self.screen.blit(geom_layer, (0, 0))

    def _draw_panel(self, telemetry: dict[str, Any]) -> None:
        pygame = self.pygame
        x0 = self.map_width
        pygame.draw.rect(self.screen, (12, 18, 32), (x0, 0, self.panel_width, self.height))
        pygame.draw.line(self.screen, (55, 65, 88), (x0, 0), (x0, self.height), 2)
        x = x0 + 16
        y = 14
        self.screen.blit(self.title_font.render("Route Monitor", True, (248, 250, 252)), (x, y))
        self._draw_text("MuJoCo sim2sim telemetry", x, y + 25, (148, 163, 184), small=True)

        y += 58
        card_w = self.panel_width - 32
        self._draw_card((x, y, card_w, 112), "state")
        mode_color = (250, 204, 21) if str(telemetry.get("mode", "")).lower().startswith("crawl") else (45, 212, 191)
        self._draw_text(str(telemetry.get("mode", "rough")), x + 12, y + 31, mode_color)
        self._draw_text(f"t {float(telemetry.get('time', 0.0)):.2f}s", x + 130, y + 31, (226, 232, 240))
        self._draw_text(
            f"wp {telemetry.get('wp_id', '')}  {int(telemetry.get('wp_index', 0))}/{int(telemetry.get('wp_count', 0))}",
            x + 12,
            y + 60,
            (226, 232, 240),
        )
        self._draw_text(
            f"x {float(telemetry.get('x', 0.0)):.2f}  y {float(telemetry.get('y', 0.0)):.2f}  yaw {math.degrees(float(telemetry.get('yaw', 0.0))):.1f}",
            x + 12,
            y + 86,
            (148, 163, 184),
            small=True,
        )

        y += 124
        self._draw_card((x, y, card_w, 128), "command limits")
        cmd_vx = float(telemetry.get("cmd_vx", 0.0))
        cmd_vy = float(telemetry.get("cmd_vy", 0.0))
        cmd_wz = float(telemetry.get("cmd_wz", 0.0))
        for row, (name, value, color) in enumerate(
            [
                ("x", cmd_vx, (34, 197, 94)),
                ("y", cmd_vy, (14, 165, 233)),
                ("yaw", cmd_wz, (250, 204, 21)),
            ]
        ):
            yy = y + 32 + row * 30
            self._draw_text(f"{name} {value:+.3f}", x + 12, yy - 7, (226, 232, 240), small=True)
            self._draw_bar(x + 90, yy, 245, value, -1.0, 1.0, color)
            cx = x + 90 + 245 // 2
            pygame.draw.line(self.screen, (71, 85, 105), (cx, yy - 3), (cx, yy + 11), 1)

        y += 140
        self._draw_card((x, y, card_w, 112), "safety")
        margin = float(telemetry.get("margin", 0.0))
        margin_color = (251, 113, 133) if margin < 0.0 else (34, 197, 94)
        self._draw_text(f"margin {margin:+.3f} m", x + 12, y + 32, margin_color)
        self._draw_text(f"clearance {float(telemetry.get('clearance', 0.0)):.3f} m", x + 190, y + 32, (226, 232, 240))
        self._draw_text(f"region {telemetry.get('region', '')}", x + 12, y + 62, (148, 163, 184), small=True)
        self._draw_text(
            f"roll {float(telemetry.get('roll_deg', 0.0)):.1f}  pitch {float(telemetry.get('pitch_deg', 0.0)):.1f}  tilt {float(telemetry.get('max_tilt_deg', 0.0)):.1f}",
            x + 12,
            y + 84,
            (148, 163, 184),
            small=True,
        )

        y += 124
        self._draw_controls(x, y, card_w)

    def _draw_controls(self, x: int, y: int, card_w: int) -> None:
        pygame = self.pygame
        self._draw_card((x, y, card_w, min(245, self.height - y - 12)), "controls")
        x += 12
        y += 32
        self._draw_text(f"nav {self.speed_scale:.2f}x", x, y, (226, 232, 240))
        self._draw_text(f"sim {self.sim_speed:.1f}x", x + 132, y, (226, 232, 240))
        self._draw_text(f"rtf {float(self._last_rtf):.1f}x" if hasattr(self, "_last_rtf") else "rtf --", x + 250, y, (148, 163, 184), small=True)
        y += 30
        labels = [
            ("nav -", "speed_down"),
            ("nav +", "speed_up"),
            ("nav 1x", "speed_normal"),
            ("sim -", "sim_down"),
            ("sim +", "sim_up"),
            ("sim 1x", "sim_normal"),
            ("sim 5x", "sim_5"),
            ("sim 10x", "sim_10"),
            ("rew -1s", "rewind_1"),
            ("rew -5s", "rewind_5"),
            ("pause", "pause"),
            ("fit", "fit"),
        ]
        self.buttons = []
        bx = x
        by = y
        for i, (label, action) in enumerate(labels):
            if i and i % 3 == 0:
                bx = x
                by += 36
            button = DashboardButton((bx, by, 104, 28), label, action)
            rect = button.rect(pygame)
            fill = (31, 41, 55)
            if action == "pause" and self.paused:
                fill = (245, 158, 11)
            elif action == "speed_normal" and abs(self.speed_scale - 1.0) < 1e-6:
                fill = (16, 185, 129)
            elif action == "sim_normal" and abs(self.sim_speed - 1.0) < 1e-6:
                fill = (14, 165, 233)
            pygame.draw.rect(self.screen, fill, rect, border_radius=5)
            pygame.draw.rect(self.screen, (71, 85, 105), rect, width=1, border_radius=5)
            text = self.small_font.render(label, True, (248, 250, 252))
            self.screen.blit(text, (rect.centerx - text.get_width() // 2, rect.centery - text.get_height() // 2))
            self.buttons.append(button)
            bx += 112
        by += 48
        self._draw_text("keys: +/- nav, [/ ] sim, backspace rewind, space pause", x, by, (148, 163, 184), small=True)
        self._draw_text("wheel zoom, right drag pan, f fit", x, by + 20, (148, 163, 184), small=True)

    def pump_events(self) -> bool:
        pygame = self.pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
                    self.speed_scale = min(10.0, self.speed_scale + 0.10)
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    self.speed_scale = max(0.05, self.speed_scale - 0.10)
                elif event.key == pygame.K_0:
                    self.speed_scale = 1.0
                elif event.key == pygame.K_RIGHTBRACKET:
                    self.sim_speed = self._next_sim_speed(1)
                elif event.key == pygame.K_LEFTBRACKET:
                    self.sim_speed = self._next_sim_speed(-1)
                elif event.key == pygame.K_BACKSPACE:
                    self.rewind_seconds = max(self.rewind_seconds, 1.0)
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_f:
                    self._fit_view()
            if event.type == pygame.VIDEORESIZE:
                self.width = max(700, int(event.w))
                self.height = max(480, int(event.h))
                self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
                self._fit_view()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    for button in self.buttons:
                        if button.rect(pygame).collidepoint(event.pos):
                            self._handle_button(button.action)
                            break
                elif event.button == 3:
                    self._dragging = True
                    self._drag_start = event.pos
                elif event.button == 4 and event.pos[0] < self.map_width:
                    self.zoom = min(240.0, self.zoom * 1.12)
                elif event.button == 5 and event.pos[0] < self.map_width:
                    self.zoom = max(8.0, self.zoom / 1.12)
            if event.type == pygame.MOUSEBUTTONUP and event.button == 3:
                self._dragging = False
                self._drag_start = None
            if event.type == pygame.MOUSEMOTION and self._dragging and self._drag_start is not None:
                dx = event.pos[0] - self._drag_start[0]
                dy = event.pos[1] - self._drag_start[1]
                self.pan_x += dx
                self.pan_y += dy
                self._drag_start = event.pos
        return True

    def draw_if_due(self, telemetry: dict[str, Any], force: bool = False) -> None:
        now = time.perf_counter()
        draw_interval = 1.0 / max(1.0, self.fps)
        if force or now - self._last_draw_time >= draw_interval:
            self._last_rtf = float(telemetry.get("rtf", 0.0))
            self.trail.append((float(telemetry.get("x", 0.0)), float(telemetry.get("y", 0.0))))
            if len(self.trail) > self.max_trail:
                self.trail = self.trail[-self.max_trail :]
            self._draw_map(telemetry)
            self._draw_panel(telemetry)
            self.pygame.display.flip()
            self._last_draw_time = now

    def _handle_button(self, action: str) -> None:
        if action == "speed_down":
            self.speed_scale = max(0.05, self.speed_scale - 0.10)
        elif action == "speed_up":
            self.speed_scale = min(10.0, self.speed_scale + 0.10)
        elif action == "speed_normal":
            self.speed_scale = 1.0
        elif action == "sim_down":
            self.sim_speed = self._next_sim_speed(-1)
        elif action == "sim_up":
            self.sim_speed = self._next_sim_speed(1)
        elif action == "sim_normal":
            self.sim_speed = 1.0
        elif action == "sim_5":
            self.sim_speed = 5.0
        elif action == "sim_10":
            self.sim_speed = 10.0
        elif action == "rewind_1":
            self.rewind_seconds = max(self.rewind_seconds, 1.0)
        elif action == "rewind_5":
            self.rewind_seconds = max(self.rewind_seconds, 5.0)
        elif action == "pause":
            self.paused = not self.paused
        elif action == "fit":
            self._fit_view()

    def _next_sim_speed(self, direction: int) -> float:
        steps = [0.2, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0]
        current = float(self.sim_speed)
        if direction > 0:
            for value in steps:
                if value > current + 1e-6:
                    return value
            return steps[-1]
        for value in reversed(steps):
            if value < current - 1e-6:
                return value
        return steps[0]

    def consume_rewind_seconds(self) -> float:
        value = self.rewind_seconds
        self.rewind_seconds = 0.0
        return value

    def close(self) -> None:
        self.pygame.quit()


def capture_route_snapshot(io: MuJoCoIO, follower: WaypointFollower) -> RouteSnapshot:
    return RouteSnapshot(
        time=float(io.d.time),
        qpos=io.d.qpos.copy(),
        qvel=io.d.qvel.copy(),
        ctrl=io.d.ctrl.copy(),
        follower_index=int(follower.index),
        follower_turn_in_place=bool(follower.turn_in_place),
        follower_stable_count=int(follower.stable_count),
        follower_wp_start_time=float(follower.wp_start_time),
        follower_best_dist=float(getattr(follower, "best_dist", 0.0)) if hasattr(follower, "best_dist") else None,
        follower_last_progress_time=float(getattr(follower, "last_progress_time", 0.0)) if hasattr(follower, "last_progress_time") else None,
        follower_recovery_until=float(getattr(follower, "recovery_until", 0.0)) if hasattr(follower, "recovery_until") else None,
        follower_recovery_count=int(getattr(follower, "recovery_count", 0)) if hasattr(follower, "recovery_count") else None,
        follower_recovery_turn_sign=float(getattr(follower, "recovery_turn_sign", 1.0)) if hasattr(follower, "recovery_turn_sign") else None,
    )


def restore_route_snapshot(io: MuJoCoIO, follower: WaypointFollower, snapshot: RouteSnapshot) -> None:
    io.d.time = snapshot.time
    io.d.qpos[:] = snapshot.qpos
    io.d.qvel[:] = snapshot.qvel
    io.d.ctrl[:] = snapshot.ctrl
    follower.index = snapshot.follower_index
    follower.turn_in_place = snapshot.follower_turn_in_place
    follower.stable_count = snapshot.follower_stable_count
    follower.wp_start_time = snapshot.follower_wp_start_time
    if snapshot.follower_best_dist is not None and hasattr(follower, "best_dist"):
        setattr(follower, "best_dist", snapshot.follower_best_dist)
    if snapshot.follower_last_progress_time is not None and hasattr(follower, "last_progress_time"):
        setattr(follower, "last_progress_time", snapshot.follower_last_progress_time)
    if snapshot.follower_recovery_until is not None and hasattr(follower, "recovery_until"):
        setattr(follower, "recovery_until", snapshot.follower_recovery_until)
    if snapshot.follower_recovery_count is not None and hasattr(follower, "recovery_count"):
        setattr(follower, "recovery_count", snapshot.follower_recovery_count)
    if snapshot.follower_recovery_turn_sign is not None and hasattr(follower, "recovery_turn_sign"):
        setattr(follower, "recovery_turn_sign", snapshot.follower_recovery_turn_sign)
    mujoco.mj_forward(io.m, io.d)


def run_sim(args: argparse.Namespace) -> tuple[SimResult, Path | None, Path]:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1_000_000_000:09d}"
    run_dir = args.out_dir / f"route_check_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    waypoints, regions = load_requested_route(args)
    footprint_radius = (
        float(args.footprint_radius)
        if args.footprint_radius is not None
        else default_lateral_footprint_radius()
    )
    required_clearance = footprint_radius + float(args.avoid_margin)
    cfg = SimConfig(
        max_vx=float(args.max_vx),
        max_vy=float(args.max_vy),
        max_wz=float(args.max_wz),
        kp_dist=float(args.kp_dist),
        kp_yaw=float(args.kp_yaw),
        yaw_stop_threshold_deg=float(args.yaw_stop_threshold_deg),
        turn_in_place_enter_deg=float(args.turn_in_place_enter_deg),
        turn_in_place_exit_deg=float(args.turn_in_place_exit_deg),
        max_total_time_s=float(args.max_time),
        waypoint_timeout_s=float(args.waypoint_timeout),
        lookahead_m=float(args.lookahead),
        min_cmd_vx=float(args.min_cmd_vx),
        creep_cmd_vx=float(args.creep_cmd_vx),
        cmd_vx_scale=float(args.cmd_vx_scale),
        stuck_timeout_s=float(args.stuck_timeout),
        recovery_duration_s=float(args.recovery_duration),
        max_recoveries=int(args.max_recoveries),
        slalom_script_enabled=bool(args.slalom_script),
        slalom_script_start_tolerance=float(args.slalom_script_start_tolerance),
        slalom_script_pos_tolerance=float(args.slalom_script_pos_tolerance),
        slalom_script_yaw_tolerance_deg=float(args.slalom_script_yaw_tolerance_deg),
        slalom_script_stable_cycles=int(args.slalom_script_stable_cycles),
        slalom_script_rotate_steps_enabled=bool(args.slalom_script_rotate_steps),
        slalom_script_final_rotate_enabled=bool(args.slalom_script_final_rotate),
        slalom_script_require_yaw_at_step=bool(args.slalom_script_require_yaw_at_step),
        slalom_script_kp_dist=float(args.slalom_script_kp_dist),
        slalom_script_kp_yaw=float(args.slalom_script_kp_yaw),
        slalom_script_max_vx=float(args.slalom_script_max_vx),
        slalom_script_max_vy=float(args.slalom_script_max_vy),
        slalom_script_max_wz=float(args.slalom_script_max_wz),
        slalom_script_min_cmd_linear=float(args.slalom_script_min_cmd_linear),
        slalom_script_min_cmd_angular=float(args.slalom_script_min_cmd_angular),
        slalom_script_min_cmd_epsilon=float(args.slalom_script_min_cmd_epsilon),
        slalom_script_min_step_distance=float(args.slalom_script_min_step_distance),
        slalom_script_yaw_gate_deg=float(args.slalom_script_yaw_gate_deg),
        slalom_script_drive_yaw_source=str(args.slalom_script_drive_yaw_source),
        slalom_script_curvature_speed_enabled=bool(args.slalom_script_curvature_speed),
        slalom_script_curvature_min_scale=max(
            0.1, min(1.0, float(args.slalom_script_curvature_min_scale))
        ),
    )

    combined_xml = make_combined_terrain_xml(args.terrain_xml.resolve(), run_dir)
    old_cwd = Path.cwd()
    try:
        io = MuJoCoIO(combined_xml, args.robot_xml.resolve(), args.hfield_dir.resolve())
    finally:
        os.chdir(old_cwd)
    if args.policy_backend == "rough-ik-crawl":
        policy = RoughPolicyIkCrawlBackend(args.onnx.resolve())
    elif args.policy_backend == "policy-runner":
        crawl_path = args.crawl_onnx.resolve() if args.crawl_onnx and args.crawl_onnx.exists() else None
        policy = ExistingPolicyBackend(args.onnx.resolve(), crawl_path)
    else:
        policy = DirectOnnxBackend(args.onnx.resolve(), DEPLOY_DEFAULT_DOF_POS)
    follower: WaypointFollower
    if args.follower in {"slalom-script", "nav-script"}:
        follower = SlalomScriptFollower(waypoints, cfg)
    elif args.follower == "nav-good":
        follower = NavGoodFollower(waypoints, cfg)
    elif args.follower == "nav-sim2sim":
        follower = NavSim2SimFollower(waypoints, cfg)
    elif args.follower == "pure-pursuit":
        follower = PurePursuitFollower(waypoints, cfg)
    else:
        follower = WaypointFollower(waypoints, cfg)
    if (
        args.skip_initial_waypoint
        or (args.auto_skip_slice_start and int(args.start_index) > 1)
    ) and len(waypoints) > 1:
        follower.index = 1

    start_x, start_y, start_yaw_deg = initial_pose_for_route(args, waypoints)
    start_x += float(args.start_x_offset)
    start_y += float(args.start_y_offset)
    set_robot_pose(
        io,
        start_x,
        start_y,
        start_yaw_deg,
        float(args.start_z),
        int(args.settle_steps),
        policy.default_dof_pos,
    )
    policy.reset()
    follower.reset_timing(io.d.time)

    sim_steps_per_control = max(1, int(round((1.0 / cfg.control_hz) / io.m.opt.timestep)))
    csv_path = None if args.no_csv else run_dir / "trajectory.csv"
    csv_file = None
    writer = None
    if csv_path is not None:
        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "time",
                "wp_index",
                "wp_id",
                "script_active",
                "script_step",
                "script_step_count",
                "x",
                "y",
                "z",
                "yaw_deg",
                "roll_deg",
                "pitch_deg",
                "cmd_vx",
                "cmd_vy",
                "cmd_wz",
                "clearance",
                "margin",
                "region",
                "raw_wheel_fl",
                "raw_wheel_fr",
                "raw_wheel_rl",
                "raw_wheel_rr",
                "qvel_wheel_fl",
                "qvel_wheel_fr",
                "qvel_wheel_rl",
                "qvel_wheel_rr",
            ]
        )

    min_clearance = float("inf")
    min_clearance_region = ""
    min_clearance_wp = ""
    max_roll = 0.0
    max_pitch = 0.0
    max_tilt = 0.0
    samples = 0
    reason = "max time reached"
    success = False

    viewer = None
    if args.viewer:
        viewer = mujoco.viewer.launch_passive(io.m, io.d)
        viewer.cam.distance = 5.0
        viewer.cam.elevation = -35.0
        viewer.cam.azimuth = 135.0
    sim_speed_factor = max(0.1, min(10.0, float(args.sim_speed)))
    if args.dashboard:
        sim_speed_factor = max(0.1, min(10.0, float(args.dashboard_sim_speed)))
    dashboard = None
    if args.dashboard:
        dashboard = PygameRouteDashboard(
            waypoints=waypoints,
            regions=regions,
            terrain_xml=args.terrain_xml.resolve(),
            width=int(args.dashboard_width),
            height=int(args.dashboard_height),
            fps=float(args.dashboard_fps),
            max_trail=int(args.dashboard_trail),
            control_hz=cfg.control_hz,
            initial_sim_speed=sim_speed_factor,
        )
    snapshots: list[RouteSnapshot] = []
    snapshot_keep_s = 45.0
    control_dt = 1.0 / cfg.control_hz
    next_exec_time = time.perf_counter()
    wall_start_time = next_exec_time
    sim_start_time = float(io.d.time)
    randomizer = random.Random(int(args.random_seed))
    command_delay_steps = max(0, int(round(float(args.command_delay_ms) * 0.001 * cfg.control_hz)))
    action_delay_steps = max(0, int(round(float(args.action_delay_ms) * 0.001 * cfg.control_hz)))
    command_queue: deque[np.ndarray] = deque()
    action_queue: deque[np.ndarray] = deque()
    last_deployed_command = np.zeros(3, dtype=np.float32)
    base_body_id = mujoco.mj_name2id(io.m, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    try:
        while float(io.d.time) - sim_start_time < cfg.max_total_time_s:
            if viewer is not None and not viewer.is_running():
                reason = "viewer closed"
                break
            if dashboard is not None and not dashboard.pump_events():
                reason = "dashboard closed"
                break
            if dashboard is not None:
                sim_speed_factor = max(0.1, min(10.0, float(dashboard.sim_speed)))
            x = float(io.d.qpos[0])
            y = float(io.d.qpos[1])
            z = float(io.d.qpos[2])
            yaw = robot_yaw_from_qpos(io.d.qpos)
            sensed_x = x
            sensed_y = y
            sensed_yaw = yaw
            if args.deployment_randomization:
                sensed_x += randomizer.gauss(0.0, max(0.0, float(args.pose_noise_xy)))
                sensed_y += randomizer.gauss(0.0, max(0.0, float(args.pose_noise_xy)))
                sensed_yaw += math.radians(
                    randomizer.gauss(0.0, max(0.0, float(args.pose_noise_yaw_deg)))
                )
            heading_yaw = follower.normalize_angle(sensed_yaw + math.radians(float(args.heading_offset_deg)))
            roll, pitch, _ = quat_to_euler_wxyz(io.d.qpos[3:7].copy())
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            max_roll = max(max_roll, abs(roll_deg))
            max_pitch = max(max_pitch, abs(pitch_deg))
            max_tilt = max(max_tilt, math.hypot(roll_deg, pitch_deg))

            active = follower.active
            active_id = active.id if active is not None else "done"
            clearance, region_name = clearance_to_regions(x, y, regions)
            margin = clearance - required_clearance
            if clearance < min_clearance:
                min_clearance = clearance
                min_clearance_region = region_name
                min_clearance_wp = active_id

            if z < float(args.min_base_z) or abs(roll_deg) > 70.0 or abs(pitch_deg) > 70.0:
                reason = f"robot fell or tipped at t={io.d.time:.2f}s"
                break
            if args.stop_on_clearance_violation and margin < 0.0:
                reason = (
                    f"clearance violation at t={io.d.time:.2f}s: "
                    f"clearance={clearance:.3f} required={required_clearance:.3f} region={region_name}"
                )
                break

            command, nav_state = follower.update(sensed_x, sensed_y, heading_yaw, io.d.time)
            if nav_state == "complete":
                success = True
                reason = "route complete"
                break
            if nav_state and nav_state.startswith("timeout"):
                reason = nav_state
                break
            if nav_state:
                reason = nav_state
                break
            skip_local_safety = False
            if active is not None and (active.policy or "").lower() == "crawl":
                skip_local_safety = True
            if hasattr(follower, "_is_charge_segment") and active is not None:
                skip_local_safety = bool(getattr(follower, "_is_charge_segment")(x, y, active)) or skip_local_safety
            if args.local_safety and not skip_local_safety:
                command = safety_filter_command(
                    command,
                    x,
                    y,
                    heading_yaw,
                    follower.active,
                    regions,
                    required_clearance,
                    cfg,
                    float(args.safety_horizon),
                    float(args.safety_dt),
                )
            if dashboard is not None:
                command[0] = float(command[0]) * dashboard.speed_scale
                if dashboard.paused:
                    command[:] = 0.0
            command = np.clip(command, -1.0, 1.0).astype(np.float32)
            if args.deployment_randomization:
                if randomizer.random() < max(0.0, min(1.0, float(args.command_dropout_prob))):
                    command = last_deployed_command.copy()
                command_queue.append(command.copy())
                command = command_queue.popleft() if len(command_queue) > command_delay_steps else np.zeros(3, dtype=np.float32)
                last_deployed_command = command.copy()

            use_route_policy = args.enable_policy_switch or args.policy_backend == "rough-ik-crawl"
            active_policy = active.policy if active is not None and use_route_policy else "rough"
            policy.maybe_switch_policy(active_policy)
            policy_mode = str(getattr(policy, "requested_mode", getattr(getattr(policy, "runner", None), "current_policy_name", active_policy)))
            default_dof_pos = policy.default_dof_pos
            obs = io.get_obs_53d(command, default_dof_pos, policy.last_actions)
            scaled_actions, raw_actions = policy.step(obs)
            if args.deployment_randomization:
                action_queue.append(scaled_actions.copy())
                scaled_actions = action_queue.popleft() if len(action_queue) > action_delay_steps else np.zeros_like(scaled_actions)
            io.send_actions(scaled_actions, default_dof_pos)
            for _ in range(sim_steps_per_control):
                if args.deployment_randomization and base_body_id != -1:
                    io.d.xfrc_applied[base_body_id] = 0.0
                    push_phase = (float(io.d.time) - sim_start_time) % max(0.1, float(args.push_interval))
                    if push_phase < max(0.0, float(args.push_duration)):
                        angle = randomizer.uniform(-math.pi, math.pi)
                        force = max(0.0, float(args.push_force))
                        io.d.xfrc_applied[base_body_id, 0] = force * math.cos(angle)
                        io.d.xfrc_applied[base_body_id, 1] = force * math.sin(angle)
                mujoco.mj_step(io.m, io.d)
                pole_contact = None
                pole_contact_robot_geom = None
                pole_contact_robot_body = None
                pole_contact_position = None
                border_contact = None
                for contact_index in range(io.d.ncon):
                    contact = io.d.contact[contact_index]
                    geom1 = mujoco.mj_id2name(io.m, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1) or ""
                    geom2 = mujoco.mj_id2name(io.m, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2) or ""
                    if geom1.startswith("slalom_pole_") or geom2.startswith("slalom_pole_"):
                        pole_contact = geom1 if geom1.startswith("slalom_pole_") else geom2
                        robot_geom_id = contact.geom2 if geom1.startswith("slalom_pole_") else contact.geom1
                        pole_contact_robot_geom = geom2 if geom1.startswith("slalom_pole_") else geom1
                        robot_body_id = int(io.m.geom_bodyid[robot_geom_id])
                        pole_contact_robot_body = (
                            mujoco.mj_id2name(io.m, mujoco.mjtObj.mjOBJ_BODY, robot_body_id) or "unknown"
                        )
                        pole_contact_position = tuple(float(value) for value in contact.pos)
                        break
                    if geom1.startswith("border_") or geom2.startswith("border_"):
                        border_contact = geom1 if geom1.startswith("border_") else geom2
                        break
                if pole_contact is not None:
                    contact_xyz = ""
                    if pole_contact_position is not None:
                        contact_xyz = (
                            f" at xyz=({pole_contact_position[0]:.3f},"
                            f"{pole_contact_position[1]:.3f},{pole_contact_position[2]:.3f})"
                        )
                    reason = (
                        f"physical contact: robot body {pole_contact_robot_body or 'unknown'} "
                        f"geom {pole_contact_robot_geom or 'unnamed'} with "
                        f"{pole_contact}{contact_xyz} at t={io.d.time:.2f}s near waypoint {active_id}"
                    )
                    break
                if border_contact is not None:
                    reason = f"physical contact with field boundary {border_contact} at t={io.d.time:.2f}s near waypoint {active_id}"
                    break
            if pole_contact is not None or border_contact is not None:
                break
            if dashboard is not None:
                snapshots.append(capture_route_snapshot(io, follower))
                cutoff_time = float(io.d.time) - snapshot_keep_s
                while len(snapshots) > 2 and snapshots[0].time < cutoff_time:
                    snapshots.pop(0)
            viewer_sync_stride = max(1, int(args.viewer_sync_every), int(math.ceil(max(1.0, sim_speed_factor) * 2.0)))
            if viewer is not None and samples % viewer_sync_stride == 0:
                base_id = mujoco.mj_name2id(io.m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
                if base_id != -1:
                    viewer.cam.lookat[:] = io.d.xpos[base_id]
                viewer.sync()

            if dashboard is not None:
                wall_elapsed = max(1e-6, time.perf_counter() - wall_start_time)
                rtf = max(0.0, (float(io.d.time) - sim_start_time) / wall_elapsed)
                telemetry = {
                    "time": float(io.d.time),
                    "rtf": rtf,
                    "status": "running",
                    "mode": policy_mode,
                    "wp_index": follower.index,
                    "wp_count": len(waypoints),
                    "wp_id": active_id,
                    "x": x,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                    "roll_deg": roll_deg,
                    "pitch_deg": pitch_deg,
                    "cmd_vx": float(command[0]),
                    "cmd_vy": float(command[1]),
                    "cmd_wz": float(command[2]),
                    "clearance": clearance,
                    "margin": margin,
                    "region": region_name,
                    "min_margin": min_clearance - required_clearance,
                    "max_tilt_deg": max_tilt,
                }
                dashboard.draw_if_due(telemetry)
                rewind_seconds = dashboard.consume_rewind_seconds()
                if rewind_seconds > 0.0 and snapshots:
                    target_time = max(0.0, float(io.d.time) - rewind_seconds)
                    rewind_snapshot = snapshots[0]
                    for snapshot in snapshots:
                        if snapshot.time <= target_time:
                            rewind_snapshot = snapshot
                        else:
                            break
                    restore_route_snapshot(io, follower, rewind_snapshot)
                    snapshots = [snapshot for snapshot in snapshots if snapshot.time <= rewind_snapshot.time]
                    if hasattr(policy, "reset"):
                        policy.reset()
                    reason = "max time reached"
                    next_exec_time = time.perf_counter()

            if writer is not None and samples % max(1, int(args.sample_every)) == 0:
                writer.writerow(
                    [
                        f"{io.d.time:.4f}",
                        follower.index,
                        active_id,
                        int(bool(getattr(follower, "slalom_script_active", False))),
                        int(getattr(follower, "slalom_script_step_index", -1)),
                        len(getattr(follower, "slalom_script_steps", [])),
                        f"{x:.5f}",
                        f"{y:.5f}",
                        f"{z:.5f}",
                        f"{math.degrees(yaw):.3f}",
                        f"{roll_deg:.3f}",
                        f"{pitch_deg:.3f}",
                        f"{float(command[0]):.4f}",
                        f"{float(command[1]):.4f}",
                        f"{float(command[2]):.4f}",
                        f"{clearance:.5f}",
                        f"{margin:.5f}",
                        region_name,
                        f"{raw_actions[12]:.5f}",
                        f"{raw_actions[13]:.5f}",
                        f"{raw_actions[14]:.5f}",
                        f"{raw_actions[15]:.5f}",
                        f"{io.d.qvel[io.qvel_ids[12]]:.5f}",
                        f"{io.d.qvel[io.qvel_ids[13]]:.5f}",
                        f"{io.d.qvel[io.qvel_ids[14]]:.5f}",
                        f"{io.d.qvel[io.qvel_ids[15]]:.5f}",
                    ]
                )
            samples += 1
            jitter_s = 0.0
            if args.deployment_randomization:
                jitter_s = randomizer.uniform(
                    -max(0.0, float(args.control_jitter_ms)) * 0.001,
                    max(0.0, float(args.control_jitter_ms)) * 0.001,
                )
            dt_real = max(0.001, control_dt + jitter_s) / sim_speed_factor
            next_exec_time += dt_real
            now = time.perf_counter()
            sleep_time = next_exec_time - now
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            elif sleep_time < -dt_real:
                next_exec_time = now
    finally:
        wall_time = max(1e-6, time.perf_counter() - wall_start_time)
        elapsed_sim_time = max(0.0, float(io.d.time) - sim_start_time)
        real_time_factor = max(0.0, elapsed_sim_time / wall_time)
        if csv_file is not None:
            csv_file.close()
        if viewer is not None:
            viewer.close()
        if dashboard is not None:
            dashboard.close()
        policy.close()

    result = SimResult(
        success=success,
        reason=reason,
        sim_time=elapsed_sim_time,
        reached_count=follower.index,
        waypoint_count=len(waypoints),
        min_clearance=float(min_clearance),
        min_margin=float(min_clearance - required_clearance),
        min_clearance_region=min_clearance_region,
        min_clearance_wp=min_clearance_wp,
        max_roll_deg=max_roll,
        max_pitch_deg=max_pitch,
        max_tilt_deg=max_tilt,
        samples=samples,
        wall_time=wall_time,
        real_time_factor=real_time_factor,
    )
    report = {
        "success": result.success,
        "reason": result.reason,
        "sim_time": round(result.sim_time, 4),
        "reached_count": result.reached_count,
        "waypoint_count": result.waypoint_count,
        "min_clearance": round(result.min_clearance, 6),
        "required_clearance": round(required_clearance, 6),
        "min_margin": round(result.min_margin, 6),
        "min_clearance_region": result.min_clearance_region,
        "min_clearance_wp": result.min_clearance_wp,
        "max_roll_deg": round(result.max_roll_deg, 3),
        "max_pitch_deg": round(result.max_pitch_deg, 3),
        "max_tilt_deg": round(result.max_tilt_deg, 3),
        "samples": result.samples,
        "wall_time": round(result.wall_time, 4),
        "real_time_factor": round(result.real_time_factor, 4),
        "sim_speed": round(sim_speed_factor, 4),
        "points": str(args.points.resolve()),
        "mission": args.mission,
        "terrain_xml": str(args.terrain_xml.resolve()),
        "onnx": str(args.onnx.resolve()),
        "crawl_onnx": str(args.crawl_onnx.resolve()) if args.crawl_onnx else None,
        "policy_backend": args.policy_backend,
        "enable_policy_switch": args.enable_policy_switch,
        "follower": args.follower,
        "max_vx": args.max_vx,
        "max_vy": args.max_vy,
        "max_wz": args.max_wz,
        "lookahead": args.lookahead,
        "cmd_vx_scale": args.cmd_vx_scale,
        "slalom_script": args.slalom_script,
        "slalom_script_start_tolerance": args.slalom_script_start_tolerance,
        "slalom_script_pos_tolerance": args.slalom_script_pos_tolerance,
        "slalom_script_kp_dist": args.slalom_script_kp_dist,
        "slalom_script_kp_yaw": args.slalom_script_kp_yaw,
        "slalom_script_max_vx": args.slalom_script_max_vx,
        "slalom_script_max_vy": args.slalom_script_max_vy,
        "slalom_script_max_wz": args.slalom_script_max_wz,
        "slalom_script_min_cmd_linear": args.slalom_script_min_cmd_linear,
        "slalom_script_min_cmd_angular": args.slalom_script_min_cmd_angular,
        "slalom_script_min_cmd_epsilon": args.slalom_script_min_cmd_epsilon,
        "slalom_script_yaw_gate_deg": args.slalom_script_yaw_gate_deg,
        "slalom_script_drive_yaw_source": args.slalom_script_drive_yaw_source,
        "local_safety": args.local_safety,
        "safety_horizon": args.safety_horizon,
        "start_yaw_offset_deg": args.start_yaw_offset_deg,
        "start_x_offset": args.start_x_offset,
        "start_y_offset": args.start_y_offset,
        "heading_offset_deg": args.heading_offset_deg,
        "start_z": args.start_z,
        "settle_steps": args.settle_steps,
        "viewer": args.viewer,
        "start_index": args.start_index,
        "end_index": args.end_index,
        "skip_initial_waypoint": args.skip_initial_waypoint,
        "auto_skip_slice_start": args.auto_skip_slice_start,
        "trajectory_csv": str(csv_path) if csv_path else None,
    }
    (run_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return result, csv_path, run_dir


def main() -> int:
    args = parse_args()
    result, csv_path, run_dir = run_sim(args)
    print("Route sim2sim check")
    print(f"  success: {result.success}")
    print(f"  reason: {result.reason}")
    print(f"  reached: {result.reached_count}/{result.waypoint_count}")
    print(f"  sim_time: {result.sim_time:.2f}s")
    print(f"  wall_time: {result.wall_time:.2f}s rtf={result.real_time_factor:.2f}x")
    print(
        "  clearance: "
        f"min={result.min_clearance:.3f}m margin={result.min_margin:.3f}m "
        f"region={result.min_clearance_region} wp={result.min_clearance_wp}"
    )
    print(
        "  attitude: "
        f"max_roll={result.max_roll_deg:.1f}deg "
        f"max_pitch={result.max_pitch_deg:.1f}deg max_tilt={result.max_tilt_deg:.1f}deg"
    )
    print(f"  report_dir: {run_dir}")
    if csv_path:
        print(f"  trajectory: {csv_path}")
    return 0 if result.success and result.min_margin >= 0.0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
