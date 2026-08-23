#!/usr/bin/env python3
"""
Standalone sim2sim check for wheel-leg IK navigation.

This file is intentionally isolated from nav_sim2sim.py and the ROS2 runtime.
It keeps the 12 leg joints at the RL rough-terrain default pose and drives the
four wheel velocity actuators with a differential-drive inverse kinematics law.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import mujoco
import mujoco.viewer
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SIM2SIM_DIR = PROJECT_ROOT / "sim2sim"
if str(SIM2SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM2SIM_DIR))

from tools.math_utils import get_gravity_orientation  # noqa: E402


LEG_JOINT_NAMES = [
    "fl_hip_abduction_joint",
    "fl_hip_pitch_joint",
    "fl_knee_joint",
    "fr_hip_abduction_joint",
    "fr_hip_pitch_joint",
    "fr_knee_joint",
    "rl_hip_abduction_joint",
    "rl_hip_pitch_joint",
    "rl_knee_joint",
    "rr_hip_abduction_joint",
    "rr_hip_pitch_joint",
    "rr_knee_joint",
]
WHEEL_JOINT_NAMES = [
    "fl_wheel_joint",
    "fr_wheel_joint",
    "rl_wheel_joint",
    "rr_wheel_joint",
]
ALL_JOINT_NAMES = LEG_JOINT_NAMES + WHEEL_JOINT_NAMES
WHEEL_BODY_NAMES = [
    "fl_wheel_Link",
    "fr_wheel_Link",
    "rl_wheel_Link",
    "rr_wheel_Link",
]

# Must match src/robot/robot_cfg.py rough policy initial state.
ROUGH_DEFAULT_DOF_POS = np.array(
    [0.0, 0.550, -1.125] * 4 + [0.0] * 4,
    dtype=np.float64,
)

# From mujoco_sim/config.py. These poses keep the wheel contact close to the
# hip in sagittal X while changing body height.
HEIGHT_TABLE = np.array(
    [
        (0.17, 0.914, -2.628),
        (0.19, 0.926, -2.528),
        (0.21, 0.924, -2.428),
        (0.23, 0.912, -2.328),
        (0.25, 0.892, -2.226),
        (0.27, 0.864, -2.122),
        (0.29, 0.834, -2.014),
        (0.31, 0.798, -1.906),
        (0.33, 0.758, -1.792),
        (0.35, 0.714, -1.672),
        (0.37, 0.666, -1.546),
        (0.39, 0.612, -1.412),
        (0.41, 0.552, -1.266),
        (0.43, 0.484, -1.104),
        (0.45, 0.404, -0.918),
    ],
    dtype=np.float64,
)


def wrap_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_euler_wxyz(q: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(float(np.clip(sinp, -1.0, 1.0)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def yaw_to_quat_wxyz(yaw: float) -> np.ndarray:
    half = 0.5 * yaw
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


def quat_to_mat_wxyz(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def table_pose_for_height(height: float) -> np.ndarray:
    h = HEIGHT_TABLE[:, 0]
    hip = float(np.interp(np.clip(height, h[0], h[-1]), h, HEIGHT_TABLE[:, 1]))
    knee = float(np.interp(np.clip(height, h[0], h[-1]), h, HEIGHT_TABLE[:, 2]))
    return np.array([0.0, hip, knee] * 4 + [0.0] * 4, dtype=np.float64)


def nominal_dof_pos(args: argparse.Namespace) -> np.ndarray:
    if args.posture == "custom":
        ab = args.custom_abduction
        return np.array(
            [
                ab,
                args.custom_hip,
                args.custom_knee,
                -ab,
                args.custom_hip,
                args.custom_knee,
                ab,
                args.custom_hip,
                args.custom_knee,
                -ab,
                args.custom_hip,
                args.custom_knee,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            dtype=np.float64,
        )
    if args.posture == "table":
        return table_pose_for_height(args.body_height)
    return ROUGH_DEFAULT_DOF_POS.copy()


def make_flat_scene_xml(robot_abs: str, tmp_dir: Path) -> Path:
    out = tmp_dir / "ik_flat_scene.xml"
    out.write_text(
        f"""<mujoco model="wheelleg_flat_scene">
  <include file="{robot_abs}"/>

  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>
  <statistic center="0 0 0.35" extent="3.0"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
      rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane" friction="0.8 0.05 0.01"/>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )
    return out


def make_sim_xml(terrain_xml: Path | None, robot_xml: Path, tmp_dir: Path) -> Path:
    robot_content = robot_xml.read_text(encoding="utf-8")
    mesh_dir = str((robot_xml.parent / "meshes").resolve()).replace("\\", "/")
    robot_content = robot_content.replace(
        '<compiler angle="radian" meshdir="meshes/"/>',
        f'<compiler angle="radian" meshdir="{mesh_dir}"/>',
    )
    tmp_dir.mkdir(parents=True, exist_ok=True)
    robot_out = tmp_dir / "wheelleg_abs_mesh.xml"
    robot_out.write_text(robot_content, encoding="utf-8")

    robot_abs = str(robot_out.resolve()).replace("\\", "/")
    if terrain_xml is None:
        return make_flat_scene_xml(robot_abs, tmp_dir)

    content = terrain_xml.read_text(encoding="utf-8")
    content = content.replace('<include file="go2w.xml"/>', f'<include file="{robot_abs}"/>')
    content = content.replace('<include file="wheelleg.xml"/>', f'<include file="{robot_abs}"/>')

    terrain_dir = terrain_xml.parent
    replacements = {
        "../height_field.png": str((terrain_dir / "height_field.png").resolve()).replace("\\", "/"),
        "../unitree_hfield.png": str((terrain_dir / "unitree_hfield.png").resolve()).replace("\\", "/"),
    }
    for old, new in replacements.items():
        content = content.replace(old, new)

    out = tmp_dir / "ik_slalom_scene.xml"
    out.write_text(content, encoding="utf-8")
    return out


def rebuild_actuators(spec: mujoco.MjSpec) -> None:
    for actuator in list(spec.actuators):
        spec.delete(actuator)

    kp_leg = 50.0
    kd_leg = 1.5
    kd_wheel = 1.0
    effort_limit = 17.0

    for jname in LEG_JOINT_NAMES:
        act = spec.add_actuator(name=jname, target=jname)
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.dyntype = mujoco.mjtDyn.mjDYN_NONE
        act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        act.gainprm[0] = kp_leg
        act.biasprm[1] = -kp_leg
        act.biasprm[2] = -kd_leg
        act.forcelimited = True
        act.forcerange[:] = [-effort_limit, effort_limit]
        act.ctrllimited = False

    for jname in WHEEL_JOINT_NAMES:
        act = spec.add_actuator(name=jname, target=jname)
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.dyntype = mujoco.mjtDyn.mjDYN_NONE
        act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        act.gainprm[0] = kd_wheel
        act.biasprm[2] = -kd_wheel
        act.forcelimited = True
        act.forcerange[:] = [-effort_limit, effort_limit]
        act.ctrllimited = False


@dataclass(frozen=True)
class ModelIds:
    qpos: np.ndarray
    qvel: np.ndarray
    ctrl: np.ndarray
    wheel_body: np.ndarray
    wheel_geom: np.ndarray


@dataclass
class SensorFrame:
    time: float
    base_pos: np.ndarray
    quat_wxyz: np.ndarray
    rpy: np.ndarray
    gyro: np.ndarray
    projected_gravity: np.ndarray
    joint_pos: np.ndarray
    joint_vel: np.ndarray
    wheel_vel: np.ndarray


def name_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> int:
    idx = mujoco.mj_name2id(model, obj_type, name)
    if idx < 0:
        raise RuntimeError(f"MuJoCo object not found: {name}")
    return idx


def build_model(args: argparse.Namespace, tmp_dir: Path) -> tuple[mujoco.MjModel, mujoco.MjData, ModelIds]:
    scene_xml = make_sim_xml(args.terrain_xml, args.robot_xml, tmp_dir)

    old_cwd = Path.cwd()
    os.chdir(str(args.robot_xml.parent))
    try:
        spec = mujoco.MjSpec.from_file(str(scene_xml))
        rebuild_actuators(spec)
        model = spec.compile()
    finally:
        os.chdir(str(old_cwd))

    data = mujoco.MjData(model)
    qpos_ids = np.array(
        [model.jnt_qposadr[name_id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in ALL_JOINT_NAMES],
        dtype=np.int32,
    )
    qvel_ids = np.array(
        [model.jnt_dofadr[name_id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in ALL_JOINT_NAMES],
        dtype=np.int32,
    )
    ctrl_ids = np.array(
        [name_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ALL_JOINT_NAMES],
        dtype=np.int32,
    )
    wheel_body_ids = np.array(
        [name_id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in WHEEL_BODY_NAMES],
        dtype=np.int32,
    )

    wheel_geom_ids: list[int] = []
    for body_id in wheel_body_ids:
        for geom_id in range(model.ngeom):
            if model.geom_bodyid[geom_id] == body_id and model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                wheel_geom_ids.append(geom_id)
                break
        else:
            raise RuntimeError(f"No cylinder collision geom found for body id {body_id}")

    return model, data, ModelIds(qpos_ids, qvel_ids, ctrl_ids, wheel_body_ids, np.array(wheel_geom_ids))


def reset_robot(model: mujoco.MjModel, data: mujoco.MjData, ids: ModelIds, args: argparse.Namespace) -> np.ndarray:
    default_dof_pos = nominal_dof_pos(args)
    data.qpos[:3] = [args.start_x, args.start_y, args.start_z]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qpos[ids.qpos] = default_dof_pos
    data.qvel[:] = 0.0
    data.ctrl[ids.ctrl[:12]] = default_dof_pos[:12]
    data.ctrl[ids.ctrl[12:]] = 0.0
    mujoco.mj_forward(model, data)

    settle_steps = int(round(args.settle / model.opt.timestep))
    for _ in range(settle_steps):
        data.ctrl[ids.ctrl[:12]] = default_dof_pos[:12]
        data.ctrl[ids.ctrl[12:]] = 0.0
        mujoco.mj_step(model, data)
    return default_dof_pos


def read_sensors(model: mujoco.MjModel, data: mujoco.MjData, ids: ModelIds) -> SensorFrame:
    del model
    quat = data.qpos[3:7].copy()
    rpy = np.array(quat_to_euler_wxyz(quat), dtype=np.float64)
    joint_pos = data.qpos[ids.qpos].copy()
    joint_vel = data.qvel[ids.qvel].copy()
    return SensorFrame(
        time=float(data.time),
        base_pos=data.qpos[:3].copy(),
        quat_wxyz=quat,
        rpy=rpy,
        gyro=data.qvel[3:6].copy(),
        projected_gravity=get_gravity_orientation(quat),
        joint_pos=joint_pos,
        joint_vel=joint_vel,
        wheel_vel=joint_vel[12:].copy(),
    )


@dataclass(frozen=True)
class WheelGeometry:
    radius: float
    track_width: float
    wheel_y: np.ndarray


def infer_wheel_geometry(model: mujoco.MjModel, data: mujoco.MjData, ids: ModelIds) -> WheelGeometry:
    radius_values = model.geom_size[ids.wheel_geom, 0]
    radius = float(np.mean(radius_values))
    wheel_y = data.xipos[ids.wheel_body, 1].copy()
    left_y = wheel_y[[0, 2]]
    right_y = wheel_y[[1, 3]]
    track = float(np.mean(left_y) - np.mean(right_y))
    if radius <= 0.0 or track <= 0.0:
        raise RuntimeError(f"Invalid wheel geometry: radius={radius}, track={track}")
    return WheelGeometry(radius=radius, track_width=track, wheel_y=wheel_y)


@dataclass
class IkCommand:
    linear_x: float
    yaw_rate: float
    target_index: int = -1
    target_distance: float = 0.0


class DifferentialWheelIk:
    def __init__(
        self,
        geometry: WheelGeometry,
        max_wheel_speed: float,
        wheel_signs: np.ndarray,
        yaw_wheel_gain: float,
        args: argparse.Namespace,
    ) -> None:
        self.geometry = geometry
        self.max_wheel_speed = max_wheel_speed
        self.wheel_signs = wheel_signs.astype(np.float64)
        self.yaw_wheel_gain = yaw_wheel_gain
        self.args = args

    def solve(self, cmd: IkCommand, sensor: SensorFrame | None = None) -> np.ndarray:
        half_track = 0.5 * self.geometry.track_width
        yaw_rate = cmd.yaw_rate
        if sensor is not None:
            yaw_rate += self.args.yaw_rate_kp * (cmd.yaw_rate - float(sensor.gyro[2]))

        if self.args.wheel_model == "direct":
            linear_wheel = self.args.linear_wheel_gain * cmd.linear_x
            yaw_wheel = self.args.direct_yaw_wheel_gain * yaw_rate
        else:
            linear_wheel = cmd.linear_x / self.geometry.radius
            yaw_wheel = self.yaw_wheel_gain * yaw_rate * half_track / self.geometry.radius
            yaw_wheel = float(np.clip(yaw_wheel, -self.args.max_yaw_wheel_speed, self.args.max_yaw_wheel_speed))
        left = linear_wheel - yaw_wheel
        right = linear_wheel + yaw_wheel
        wheel = np.array([left, right, left, right], dtype=np.float64)
        wheel = np.clip(wheel, -self.max_wheel_speed, self.max_wheel_speed)
        return wheel * self.wheel_signs


class LegStanceIk:
    """Numerical wheel-center IK for keeping the rough RL stance level."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, ids: ModelIds, args: argparse.Namespace) -> None:
        self.model = model
        self.ids = ids
        self.args = args
        self.shadow = mujoco.MjData(model)
        self.jacp = np.zeros((3, model.nv), dtype=np.float64)
        self.jacr = np.zeros((3, model.nv), dtype=np.float64)
        self.last_target = ROUGH_DEFAULT_DOF_POS[:12].copy()

        self.leg_qpos = [ids.qpos[i * 3 : i * 3 + 3] for i in range(4)]
        self.leg_qvel = [ids.qvel[i * 3 : i * 3 + 3] for i in range(4)]
        self.leg_joint_ids = [
            [name_id(model, mujoco.mjtObj.mjOBJ_JOINT, LEG_JOINT_NAMES[i * 3 + j]) for j in range(3)]
            for i in range(4)
        ]
        self.joint_ranges = [
            np.array([model.jnt_range[jid].copy() for jid in leg], dtype=np.float64)
            for leg in self.leg_joint_ids
        ]

        mujoco.mj_forward(model, data)
        base_pos = data.qpos[:3].copy()
        base_rot = quat_to_mat_wxyz(data.qpos[3:7])
        self.nominal_body_wheel = (base_rot.T @ (data.xipos[ids.wheel_body] - base_pos).T).T

    def compute(self, data: mujoco.MjData, sensor: SensorFrame) -> np.ndarray:
        current_rot = quat_to_mat_wxyz(sensor.quat_wxyz)
        level_rot = quat_to_mat_wxyz(yaw_to_quat_wxyz(float(sensor.rpy[2])))
        base_targets = (current_rot @ self.nominal_body_wheel.T).T
        level_targets = (level_rot @ self.nominal_body_wheel.T).T
        target_world = sensor.base_pos + base_targets + self.args.posture_gain * (level_targets - base_targets)

        self.shadow.qpos[:] = data.qpos
        self.shadow.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.shadow)

        damping = self.args.leg_ik_damping
        for _ in range(self.args.leg_ik_iterations):
            for leg_idx, body_id in enumerate(self.ids.wheel_body):
                err = target_world[leg_idx] - self.shadow.xipos[body_id]
                if float(np.linalg.norm(err)) < 1e-4:
                    continue

                self.jacp.fill(0.0)
                self.jacr.fill(0.0)
                mujoco.mj_jacBody(self.model, self.shadow, self.jacp, self.jacr, int(body_id))
                cols = self.leg_qvel[leg_idx]
                j_leg = self.jacp[:, cols]
                lhs = j_leg @ j_leg.T + (damping * damping) * np.eye(3)
                dq = j_leg.T @ np.linalg.solve(lhs, err)
                dq = np.clip(dq, -self.args.leg_ik_step, self.args.leg_ik_step)

                qpos_ids = self.leg_qpos[leg_idx]
                self.shadow.qpos[qpos_ids] += dq
                ranges = self.joint_ranges[leg_idx]
                self.shadow.qpos[qpos_ids] = np.clip(self.shadow.qpos[qpos_ids], ranges[:, 0], ranges[:, 1])
            mujoco.mj_forward(self.model, self.shadow)

        raw_target = self.shadow.qpos[self.ids.qpos[:12]].copy()
        offset = np.clip(
            raw_target - ROUGH_DEFAULT_DOF_POS[:12],
            -self.args.max_leg_offset,
            self.args.max_leg_offset,
        )
        raw_target = ROUGH_DEFAULT_DOF_POS[:12] + offset

        max_delta = self.args.leg_target_rate
        filtered = self.last_target + np.clip(raw_target - self.last_target, -max_delta, max_delta)
        alpha = self.args.leg_target_filter
        filtered = (1.0 - alpha) * self.last_target + alpha * filtered
        self.last_target = filtered.copy()
        return filtered


class TablePostureController:
    def __init__(self, args: argparse.Namespace, initial_target: np.ndarray) -> None:
        self.args = args
        self.target = initial_target[:12].copy()

    def compute(self, sensor: SensorFrame) -> np.ndarray:
        if self.args.posture == "rough":
            raw = ROUGH_DEFAULT_DOF_POS[:12].copy()
        elif self.args.posture == "custom":
            raw = nominal_dof_pos(self.args)[:12].copy()
        else:
            pose = table_pose_for_height(self.args.body_height)
            raw = pose[:12].copy()

        if self.args.imu_posture:
            roll_corr = -self.args.roll_comp_gain * float(sensor.rpy[0])
            pitch_corr = -self.args.pitch_comp_gain * float(sensor.rpy[1])
            for i, side in enumerate((1.0, -1.0, 1.0, -1.0)):
                raw[i * 3] = np.clip(raw[i * 3] + side * roll_corr, -0.5, 0.5)
                raw[i * 3 + 1] = np.clip(raw[i * 3 + 1] + pitch_corr, -1.0, 2.5)
                raw[i * 3 + 2] = np.clip(raw[i * 3 + 2], -2.65, -0.3)

        if self.args.encoder_posture_kp > 0.0:
            encoder_err = self.target - sensor.joint_pos[:12]
            raw += np.clip(
                self.args.encoder_posture_kp * encoder_err,
                -self.args.encoder_posture_max,
                self.args.encoder_posture_max,
            )

        max_delta = self.args.leg_target_rate
        raw = self.target + np.clip(raw - self.target, -max_delta, max_delta)
        alpha = self.args.leg_target_filter
        self.target = (1.0 - alpha) * self.target + alpha * raw
        return self.target.copy()


def sensor_command_scale(sensor: SensorFrame, leg_target: np.ndarray, args: argparse.Namespace) -> float:
    scale = 1.0
    if args.encoder_guard:
        leg_error = float(np.max(np.abs(sensor.joint_pos[:12] - leg_target)))
        if leg_error >= args.encoder_guard_stop:
            scale = 0.0
        elif leg_error > args.encoder_guard_start:
            span = max(1e-6, args.encoder_guard_stop - args.encoder_guard_start)
            scale *= 1.0 - (leg_error - args.encoder_guard_start) / span

    if args.imu_guard:
        tilt = math.hypot(float(sensor.rpy[0]), float(sensor.rpy[1]))
        start = math.radians(args.imu_guard_start_deg)
        stop = math.radians(args.imu_guard_stop_deg)
        if tilt >= stop:
            scale = 0.0
        elif tilt > start:
            scale *= 1.0 - (tilt - start) / max(1e-6, stop - start)

    return float(np.clip(scale, 0.0, 1.0))


class PathFollower:
    def __init__(
        self,
        waypoints: Iterable[tuple[float, float]],
        max_speed: float,
        max_yaw_rate: float,
        arrive_radius: float,
        allow_reverse: bool,
    ) -> None:
        self.waypoints = list(waypoints)
        self.idx = 0
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.arrive_radius = arrive_radius
        self.allow_reverse = allow_reverse

    @property
    def done(self) -> bool:
        return self.idx >= len(self.waypoints)

    def update(self, sensor: SensorFrame) -> IkCommand:
        if self.done:
            return IkCommand(0.0, 0.0, self.idx, 0.0)

        x, y = sensor.base_pos[:2]
        target = self.waypoints[self.idx]
        dx = target[0] - float(x)
        dy = target[1] - float(y)
        dist = math.hypot(dx, dy)
        while dist < self.arrive_radius and self.idx < len(self.waypoints) - 1:
            self.idx += 1
            target = self.waypoints[self.idx]
            dx = target[0] - float(x)
            dy = target[1] - float(y)
            dist = math.hypot(dx, dy)
        if dist < self.arrive_radius and self.idx == len(self.waypoints) - 1:
            self.idx = len(self.waypoints)
            return IkCommand(0.0, 0.0, self.idx, dist)

        desired_yaw = math.atan2(dy, dx)
        yaw_error = wrap_pi(desired_yaw - float(sensor.rpy[2]))
        direction = 1.0
        if self.allow_reverse and abs(yaw_error) > math.pi * 0.5:
            desired_yaw = wrap_pi(desired_yaw + math.pi)
            yaw_error = wrap_pi(desired_yaw - float(sensor.rpy[2]))
            direction = -1.0
        yaw_rate = float(np.clip(2.8 * yaw_error, -self.max_yaw_rate, self.max_yaw_rate))

        speed = min(self.max_speed, 1.2 * dist)
        speed *= max(0.15, math.cos(min(abs(yaw_error), math.pi * 0.5)))
        speed *= direction
        return IkCommand(speed, yaw_rate, self.idx, dist)


class PurePursuitFollower:
    def __init__(self, waypoints: Iterable[tuple[float, float]], args: argparse.Namespace) -> None:
        self.points = np.array(list(waypoints), dtype=np.float64)
        self.args = args
        self.progress = 0
        self.closest_distance = float("inf")
        self.finished = False
        if len(self.points) < 2:
            raise ValueError("PurePursuitFollower needs at least two points")
        seg = self.points[1:] - self.points[:-1]
        self.seg_len = np.linalg.norm(seg, axis=1)
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.total_len = float(self.cum[-1])

    @property
    def done(self) -> bool:
        return self.finished

    @property
    def idx(self) -> int:
        return int(self.progress)

    def _project_s(self, p: np.ndarray) -> tuple[float, float]:
        best_s = 0.0
        best_d = float("inf")
        start_i = max(0, self.progress - 1)
        end_i = min(len(self.points) - 1, self.progress + self.args.pure_search_segments)
        for i in range(start_i, end_i):
            a = self.points[i]
            b = self.points[i + 1]
            ab = b - a
            denom = float(np.dot(ab, ab))
            if denom < 1e-9:
                continue
            t = float(np.clip(np.dot(p - a, ab) / denom, 0.0, 1.0))
            proj = a + t * ab
            d = float(np.linalg.norm(p - proj))
            if d < best_d:
                best_d = d
                best_s = float(self.cum[i] + t * self.seg_len[i])
                self.progress = max(self.progress, i)
        return best_s, best_d

    def _point_at_s(self, s: float) -> np.ndarray:
        s = float(np.clip(s, 0.0, self.total_len))
        i = int(np.searchsorted(self.cum, s, side="right") - 1)
        i = int(np.clip(i, 0, len(self.seg_len) - 1))
        local = 0.0 if self.seg_len[i] < 1e-9 else (s - self.cum[i]) / self.seg_len[i]
        return self.points[i] + local * (self.points[i + 1] - self.points[i])

    def update(self, sensor: SensorFrame) -> IkCommand:
        p = sensor.base_pos[:2].astype(np.float64)
        s, dist_path = self._project_s(p)
        self.closest_distance = min(self.closest_distance, dist_path)
        goal_dist = float(np.linalg.norm(self.points[-1] - p))
        if self.total_len - s < self.args.arrive_radius and goal_dist < self.args.arrive_radius:
            self.finished = True
            return IkCommand(0.0, 0.0, len(self.points) - 1, goal_dist)

        lookahead = np.clip(
            self.args.lookahead_base + self.args.lookahead_time * abs(self.args.speed),
            self.args.lookahead_min,
            self.args.lookahead_max,
        )
        target = self._point_at_s(s + lookahead)
        dx, dy = target - p
        yaw = float(sensor.rpy[2])
        c, sn = math.cos(yaw), math.sin(yaw)
        x_body = c * dx + sn * dy
        y_body = -sn * dx + c * dy
        ld2 = max(lookahead * lookahead, 1e-6)
        curvature = 2.0 * y_body / ld2

        abs_curv = abs(curvature)
        if abs_curv > 1e-6:
            v_curve = math.sqrt(max(self.args.max_lat_acc, 1e-6) / abs_curv)
        else:
            v_curve = self.args.speed
        speed = min(self.args.speed, v_curve)
        speed = max(self.args.min_path_speed, speed)
        if x_body < -0.05 and self.args.allow_reverse:
            speed = -speed
        yaw_rate = float(np.clip(speed * curvature, -self.args.max_yaw_rate, self.args.max_yaw_rate))
        return IkCommand(float(speed), yaw_rate, self.progress, dist_path)


def args_sign(x: float) -> float:
    return -1.0 if x < -0.05 else 1.0


def test_command(test_name: str, elapsed: float, sensor: SensorFrame, follower: PathFollower | None, args: argparse.Namespace) -> IkCommand:
    if test_name == "forward":
        return IkCommand(args.speed, 0.0)
    if test_name == "yaw":
        return IkCommand(0.0, args.yaw_rate)
    if test_name == "arc":
        return IkCommand(args.speed, args.yaw_rate)
    if follower is None:
        raise RuntimeError(f"Test {test_name} needs a path follower")
    del elapsed
    return follower.update(sensor)


def waypoints_for_test(test_name: str, args: argparse.Namespace) -> list[tuple[float, float]]:
    sx = float(args.start_x)
    sy = float(args.start_y)
    if test_name == "waypoint":
        return [
            (sx + 0.3, sy),
            (sx + 0.65, sy),
            (sx + 1.0, sy),
        ]
    if test_name == "slalom":
        local = [
            (0.0, 0.0),
            (0.8, -0.5),
            (1.9, -0.95),
            (2.8, -1.0),
            (2.85, -1.8),
            (2.75, -2.05),
            (1.05, -2.0),
            (0.9, -2.8),
            (1.05, -3.0),
            (2.75, -3.0),
            (2.75, -3.72),
            (2.25, -3.85),
            (1.4, -4.25),
            (1.4, -2.55),
            (0.45, -3.42),
            (0.0, 0.0),
        ]
        return [(sx + x, sy + y) for x, y in local]
    return []


@dataclass
class Metrics:
    start_pos: np.ndarray
    start_yaw: float
    start_time: float = 0.0
    max_abs_roll: float = 0.0
    max_abs_pitch: float = 0.0
    max_tilt: float = 0.0
    max_leg_error: float = 0.0
    max_wheel_air_height: float = -1e9
    min_wheel_air_height: float = 1e9
    wheel_speed_error_sum: float = 0.0
    wheel_speed_error_count: int = 0
    max_wheel_speed: float = 0.0
    max_gyro_z: float = 0.0
    body_vx_sum: float = 0.0
    gyro_z_sum: float = 0.0
    yaw_unwrapped: float = 0.0
    velocity_count: int = 0
    last_yaw: float = field(init=False)

    def __post_init__(self) -> None:
        self.last_yaw = float(self.start_yaw)

    def update(
        self,
        sensor: SensorFrame,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        ids: ModelIds,
        geometry: WheelGeometry,
        wheel_target: np.ndarray,
        leg_target: np.ndarray,
    ) -> None:
        roll, pitch, _ = sensor.rpy
        self.max_abs_roll = max(self.max_abs_roll, abs(float(roll)))
        self.max_abs_pitch = max(self.max_abs_pitch, abs(float(pitch)))
        self.max_tilt = max(self.max_tilt, math.hypot(float(roll), float(pitch)))
        self.max_leg_error = max(
            self.max_leg_error,
            float(np.max(np.abs(sensor.joint_pos[:12] - leg_target))),
        )

        wheel_center_z = data.xipos[ids.wheel_body, 2]
        floor_air_height = wheel_center_z - geometry.radius
        self.max_wheel_air_height = max(self.max_wheel_air_height, float(np.max(floor_air_height)))
        self.min_wheel_air_height = min(self.min_wheel_air_height, float(np.min(floor_air_height)))

        actual = sensor.wheel_vel
        self.wheel_speed_error_sum += float(np.mean(np.abs(actual - wheel_target)))
        self.wheel_speed_error_count += 1
        self.max_wheel_speed = max(self.max_wheel_speed, float(np.max(np.abs(actual))))
        self.max_gyro_z = max(self.max_gyro_z, abs(float(sensor.gyro[2])))
        rot = quat_to_mat_wxyz(sensor.quat_wxyz)
        body_vel = rot.T @ data.qvel[:3]
        yaw_now = float(sensor.rpy[2])
        self.yaw_unwrapped += wrap_pi(yaw_now - self.last_yaw)
        self.last_yaw = yaw_now
        self.body_vx_sum += float(body_vel[0])
        self.gyro_z_sum += float(sensor.gyro[2])
        self.velocity_count += 1
        del model

    def as_dict(self, sensor: SensorFrame, reached: int, total: int, reason: str, success: bool) -> dict:
        yaw_change = wrap_pi(float(sensor.rpy[2]) - self.start_yaw)
        delta = sensor.base_pos[:2] - self.start_pos[:2]
        mean_wheel_err = self.wheel_speed_error_sum / max(1, self.wheel_speed_error_count)
        mean_body_vx = self.body_vx_sum / max(1, self.velocity_count)
        elapsed = max(1e-6, float(sensor.time) - float(self.start_time))
        mean_yaw_rate = self.yaw_unwrapped / elapsed
        mean_gyro_z = self.gyro_z_sum / max(1, self.velocity_count)
        return {
            "success": bool(success),
            "reason": reason,
            "sim_time": round(sensor.time, 4),
            "start_xy": [round(float(v), 4) for v in self.start_pos[:2]],
            "final_xy": [round(float(v), 4) for v in sensor.base_pos[:2]],
            "delta_xy": [round(float(v), 4) for v in delta],
            "final_z": round(float(sensor.base_pos[2]), 4),
            "yaw_change_deg": round(math.degrees(yaw_change), 3),
            "final_yaw_deg": round(math.degrees(float(sensor.rpy[2])), 3),
            "max_roll_deg": round(math.degrees(self.max_abs_roll), 3),
            "max_pitch_deg": round(math.degrees(self.max_abs_pitch), 3),
            "max_tilt_deg": round(math.degrees(self.max_tilt), 3),
            "max_leg_encoder_error_rad": round(self.max_leg_error, 5),
            "mean_wheel_speed_error_rad_s": round(mean_wheel_err, 5),
            "max_wheel_speed_rad_s": round(self.max_wheel_speed, 5),
            "mean_body_vx_mps": round(mean_body_vx, 5),
            "mean_yaw_rate_rad_s": round(mean_yaw_rate, 5),
            "mean_imu_gyro_z_rad_s": round(mean_gyro_z, 5),
            "min_wheel_air_height_m": round(self.min_wheel_air_height, 5),
            "max_wheel_air_height_m": round(self.max_wheel_air_height, 5),
            "max_imu_gyro_z_rad_s": round(self.max_gyro_z, 5),
            "reached_waypoints": int(reached),
            "total_waypoints": int(total),
        }


class InteractivePanel:
    def __init__(self, args: argparse.Namespace) -> None:
        try:
            import tkinter as tk
            from tkinter import ttk
        except ImportError as exc:
            raise RuntimeError("Interactive mode needs tkinter, but it is not available in this Python.") from exc

        self.tk = tk
        self.alive = True
        self.last_update = 0.0
        self.root = tk.Tk()
        self.root.title("Wheel-Leg IK Control")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.vx = tk.DoubleVar(value=float(args.speed))
        self.yaw = tk.DoubleVar(value=float(args.yaw_rate))
        self.status_vars: dict[str, object] = {}

        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)

        self._add_slider(main, 0, "linear x m/s", self.vx, args.interactive_vx_limit)
        self._add_slider(main, 1, "yaw z rad/s", self.yaw, args.interactive_yaw_limit)

        buttons = ttk.Frame(main)
        buttons.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(6, 10))
        ttk.Button(buttons, text="Zero", command=self.zero).pack(side="left", padx=(0, 6))
        ttk.Button(buttons, text="Quit", command=self.close).pack(side="left")

        metrics = ttk.LabelFrame(main, text="State", padding=8)
        metrics.grid(row=3, column=0, columnspan=3, sticky="nsew")
        metrics.columnconfigure(1, weight=1)
        for row, key in enumerate(
            [
                "cmd",
                "pos",
                "rpy",
                "body_vel",
                "imu",
                "wheel",
                "wheel_air",
                "leg",
                "limits",
            ]
        ):
            ttk.Label(metrics, text=key).grid(row=row, column=0, sticky="w", padx=(0, 8))
            var = tk.StringVar(value="-")
            self.status_vars[key] = var
            ttk.Label(metrics, textvariable=var, width=48).grid(row=row, column=1, sticky="w")

    def _add_slider(self, parent: object, row: int, label: str, var: object, limit: float) -> None:
        ttk = __import__("tkinter.ttk").ttk
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        scale = ttk.Scale(parent, from_=-limit, to=limit, variable=var, orient="horizontal")
        scale.grid(row=row, column=1, sticky="ew", padx=8)
        value = ttk.Label(parent, width=8)
        value.grid(row=row, column=2, sticky="e")

        def refresh_value(*_: object) -> None:
            value.configure(text=f"{float(var.get()): .2f}")

        var.trace_add("write", refresh_value)
        refresh_value()

    def zero(self) -> None:
        self.vx.set(0.0)
        self.yaw.set(0.0)

    def close(self) -> None:
        self.alive = False

    def command(self) -> IkCommand:
        return IkCommand(float(self.vx.get()), float(self.yaw.get()))

    def update_events(self) -> bool:
        if not self.alive:
            return False
        try:
            self.root.update_idletasks()
            self.root.update()
        except self.tk.TclError:
            self.alive = False
        return self.alive

    def update_metrics(
        self,
        sensor: SensorFrame,
        data: mujoco.MjData,
        ids: ModelIds,
        geometry: WheelGeometry,
        wheel_target: np.ndarray,
        leg_target: np.ndarray,
        metrics: Metrics,
        args: argparse.Namespace,
    ) -> None:
        if sensor.time - self.last_update < args.ui_update_dt:
            return
        self.last_update = sensor.time

        rot = quat_to_mat_wxyz(sensor.quat_wxyz)
        body_vel = rot.T @ data.qvel[:3]
        wheel_air = data.xipos[ids.wheel_body, 2] - geometry.radius
        wheel_err = float(np.mean(np.abs(sensor.wheel_vel - wheel_target)))
        leg_err = float(np.max(np.abs(sensor.joint_pos[:12] - leg_target)))
        tilt = math.hypot(float(sensor.rpy[0]), float(sensor.rpy[1]))

        values = {
            "cmd": f"x={self.vx.get(): .2f} m/s, yaw={self.yaw.get(): .2f} rad/s",
            "pos": f"x={sensor.base_pos[0]: .3f}, y={sensor.base_pos[1]: .3f}, z={sensor.base_pos[2]: .3f}",
            "rpy": (
                f"roll={math.degrees(sensor.rpy[0]): .2f}, "
                f"pitch={math.degrees(sensor.rpy[1]): .2f}, "
                f"yaw={math.degrees(sensor.rpy[2]): .2f} deg"
            ),
            "body_vel": f"vx={body_vel[0]: .3f}, vy={body_vel[1]: .3f}, vz={body_vel[2]: .3f} m/s",
            "imu": f"gyro_z={sensor.gyro[2]: .3f} rad/s, tilt={math.degrees(tilt): .2f} deg",
            "wheel": (
                f"target=[{', '.join(f'{v: .1f}' for v in wheel_target)}], "
                f"err={wheel_err: .3f} rad/s"
            ),
            "wheel_air": f"min={np.min(wheel_air): .4f}, max={np.max(wheel_air): .4f} m",
            "leg": f"encoder_err={leg_err: .4f} rad, max={metrics.max_leg_error: .4f} rad",
            "limits": (
                f"max_tilt={math.degrees(metrics.max_tilt): .2f} deg, "
                f"max_wheel_speed={metrics.max_wheel_speed: .2f} rad/s"
            ),
        }
        for key, value in values.items():
            self.status_vars[key].set(value)

    def destroy(self) -> None:
        try:
            self.root.destroy()
        except self.tk.TclError:
            pass


def run_one(test_name: str, args: argparse.Namespace) -> dict:
    tmp_root = Path(args.tmp_dir) if args.tmp_dir else Path(tempfile.gettempdir())
    with tempfile.TemporaryDirectory(prefix="ik_slalom_", dir=str(tmp_root)) as tmp:
        model, data, ids = build_model(args, Path(tmp))
        default_dof_pos = reset_robot(model, data, ids, args)
        geometry = infer_wheel_geometry(model, data, ids)
        wheel_signs = np.array(args.wheel_signs, dtype=np.float64)
        ik = DifferentialWheelIk(geometry, args.max_wheel_speed, wheel_signs, args.yaw_wheel_gain, args)
        leg_ik = LegStanceIk(model, data, ids, args) if args.leg_ik else None
        posture = TablePostureController(args, default_dof_pos) if leg_ik is None else None
        path = waypoints_for_test(test_name, args)
        if path and args.path_follower == "pure":
            follower = PurePursuitFollower(path, args)
        else:
            follower = PathFollower(path, args.speed, args.max_yaw_rate, args.arrive_radius, args.allow_reverse) if path else None

        sensor0 = read_sensors(model, data, ids)
        metrics = Metrics(
            start_pos=sensor0.base_pos.copy(),
            start_yaw=float(sensor0.rpy[2]),
            start_time=float(sensor0.time),
        )

        control_steps = max(1, int(round(args.control_dt / model.opt.timestep)))
        total_steps = int(round(args.duration / model.opt.timestep))
        wheel_target = np.zeros(4, dtype=np.float64)
        leg_target = default_dof_pos[:12].copy()
        reason = "timeout"
        success = False

        viewer = mujoco.viewer.launch_passive(model, data) if args.viewer else None
        panel = InteractivePanel(args) if args.interactive else None
        try:
            for step in range(total_steps):
                if panel is not None and not panel.update_events():
                    reason = "interactive panel closed"
                    break
                if viewer is not None and not viewer.is_running():
                    reason = "viewer closed"
                    break

                if step % control_steps == 0:
                    sensor = read_sensors(model, data, ids)
                    cmd = panel.command() if panel is not None else test_command(test_name, step * model.opt.timestep, sensor, follower, args)
                    raw_wheel_target = ik.solve(cmd, sensor)
                    if leg_ik is not None:
                        leg_target = leg_ik.compute(data, sensor)
                    elif posture is not None:
                        leg_target = posture.compute(sensor)
                    raw_wheel_target *= sensor_command_scale(sensor, leg_target, args)
                    max_wheel_delta = args.wheel_accel_limit * args.control_dt
                    wheel_target = wheel_target + np.clip(raw_wheel_target - wheel_target, -max_wheel_delta, max_wheel_delta)
                    data.ctrl[ids.ctrl[:12]] = leg_target
                    data.ctrl[ids.ctrl[12:]] = wheel_target

                    if follower is not None and follower.done:
                        reason = "path complete"
                        success = True
                        break

                mujoco.mj_step(model, data)
                if step % control_steps == 0:
                    sensor = read_sensors(model, data, ids)
                    metrics.update(sensor, model, data, ids, geometry, wheel_target, leg_target)
                    if panel is not None:
                        panel.update_metrics(sensor, data, ids, geometry, wheel_target, leg_target, metrics, args)
                    if viewer is not None:
                        viewer.sync()

                if viewer is not None and args.realtime:
                    time.sleep(model.opt.timestep * args.realtime_scale)
        finally:
            if panel is not None:
                panel.destroy()
            if viewer is not None:
                viewer.close()

        final_sensor = read_sensors(model, data, ids)
        reached = follower.idx if follower is not None else 0
        total = len(path)

        stable = (
            metrics.max_tilt < math.radians(args.max_tilt_deg)
            and metrics.max_leg_error < args.max_leg_error
            and metrics.max_wheel_air_height < args.max_wheel_air_height
        )

        if args.interactive:
            success = stable
            reason = reason if reason != "timeout" else "interactive duration elapsed"
        elif test_name == "forward":
            success = final_sensor.base_pos[0] - metrics.start_pos[0] > max(0.25, 0.35 * args.speed * args.duration) and stable
            reason = "forward displacement ok" if success else reason
        elif test_name == "yaw":
            success = abs(wrap_pi(final_sensor.rpy[2] - metrics.start_yaw)) > max(0.15, 0.14 * abs(args.yaw_rate) * args.duration) and stable
            reason = "yaw change ok" if success else reason
        elif test_name == "arc":
            delta = np.linalg.norm(final_sensor.base_pos[:2] - metrics.start_pos[:2])
            success = (
                delta > max(0.25, 0.25 * args.speed * args.duration)
                and abs(wrap_pi(final_sensor.rpy[2] - metrics.start_yaw)) > 0.15
                and stable
            )
            reason = "arc displacement and yaw ok" if success else reason
        elif test_name in {"waypoint", "slalom"}:
            success = follower is not None and follower.done and stable
            reason = "path complete" if success else reason

        out = metrics.as_dict(final_sensor, reached, total, reason, success)
        out.update(
            {
                "test": test_name,
                "wheel_radius_m": round(geometry.radius, 5),
                "track_width_m": round(geometry.track_width, 5),
                "wheel_body_y_m": [round(float(v), 5) for v in geometry.wheel_y],
                "wheel_signs": [float(v) for v in wheel_signs],
                "wheel_model": args.wheel_model,
                "linear_wheel_gain": float(args.linear_wheel_gain),
                "direct_yaw_wheel_gain": float(args.direct_yaw_wheel_gain),
                "yaw_wheel_gain": float(args.yaw_wheel_gain),
                "stable": bool(stable),
                "leg_ik": bool(args.leg_ik),
                "posture": args.posture,
                "body_height": float(args.body_height),
                "imu_posture": bool(args.imu_posture),
                "yaw_rate_kp": float(args.yaw_rate_kp),
                "encoder_posture_kp": float(args.encoder_posture_kp),
                "encoder_guard": bool(args.encoder_guard),
                "imu_guard": bool(args.imu_guard),
                "posture_gain": float(args.posture_gain),
                "path_follower": args.path_follower,
                "default_pose": (
                    "rough_rl"
                    if args.posture == "rough"
                    else "custom"
                    if args.posture == "custom"
                    else "mujoco_sim_height_table"
                ),
            }
        )
        return out


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--test", choices=("forward", "yaw", "arc", "waypoint", "slalom", "all"), default="forward")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--settle", type=float, default=1.5)
    parser.add_argument("--control-dt", type=float, default=0.02)
    parser.add_argument("--speed", type=float, default=0.35)
    parser.add_argument("--yaw-rate", type=float, default=0.25)
    parser.add_argument("--max-yaw-rate", type=float, default=0.45)
    parser.add_argument("--max-wheel-speed", type=float, default=10.0)
    parser.add_argument("--wheel-model", choices=("diff", "direct"), default="diff")
    parser.add_argument("--linear-wheel-gain", type=float, default=12.5)
    parser.add_argument("--direct-yaw-wheel-gain", type=float, default=8.0)
    parser.add_argument("--yaw-wheel-gain", type=float, default=2.2)
    parser.add_argument("--yaw-rate-kp", type=float, default=0.0)
    parser.add_argument("--max-yaw-wheel-speed", type=float, default=1.65)
    parser.add_argument("--wheel-accel-limit", type=float, default=28.0)
    parser.add_argument("--max-tilt-deg", type=float, default=25.0)
    parser.add_argument("--max-leg-error", type=float, default=0.65)
    parser.add_argument("--max-wheel-air-height", type=float, default=0.08)
    parser.add_argument("--posture", choices=("rough", "table", "custom"), default="rough")
    parser.add_argument("--body-height", type=float, default=0.37)
    parser.add_argument("--custom-abduction", type=float, default=0.0)
    parser.add_argument("--custom-hip", type=float, default=0.55)
    parser.add_argument("--custom-knee", type=float, default=-1.30)
    parser.add_argument("--imu-posture", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--roll-comp-gain", type=float, default=0.35)
    parser.add_argument("--pitch-comp-gain", type=float, default=0.35)
    parser.add_argument("--encoder-posture-kp", type=float, default=0.15)
    parser.add_argument("--encoder-posture-max", type=float, default=0.03)
    parser.add_argument("--encoder-guard", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--encoder-guard-start", type=float, default=0.28)
    parser.add_argument("--encoder-guard-stop", type=float, default=0.65)
    parser.add_argument("--imu-guard", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--imu-guard-start-deg", type=float, default=12.0)
    parser.add_argument("--imu-guard-stop-deg", type=float, default=28.0)
    parser.add_argument("--leg-ik", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--posture-gain", type=float, default=0.65)
    parser.add_argument("--leg-ik-iterations", type=int, default=3)
    parser.add_argument("--leg-ik-damping", type=float, default=0.02)
    parser.add_argument("--leg-ik-step", type=float, default=0.035)
    parser.add_argument("--max-leg-offset", type=float, default=0.28)
    parser.add_argument("--leg-target-rate", type=float, default=0.035)
    parser.add_argument("--leg-target-filter", type=float, default=0.45)
    parser.add_argument("--arrive-radius", type=float, default=0.20)
    parser.add_argument("--allow-reverse", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--path-follower", choices=("waypoint", "pure"), default="pure")
    parser.add_argument("--lookahead-base", type=float, default=0.45)
    parser.add_argument("--lookahead-time", type=float, default=0.35)
    parser.add_argument("--lookahead-min", type=float, default=0.35)
    parser.add_argument("--lookahead-max", type=float, default=0.90)
    parser.add_argument("--max-lat-acc", type=float, default=0.45)
    parser.add_argument("--min-path-speed", type=float, default=0.25)
    parser.add_argument("--pure-search-segments", type=int, default=4)
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--start-z", type=float, default=0.6)
    parser.add_argument(
        "--terrain-xml",
        type=Path,
        default=None,
        help="Optional terrain scene XML. Omit it to use a generated flat ground scene.",
    )
    parser.add_argument("--robot-xml", type=Path, default=PROJECT_ROOT / "mjcf" / "wheelleg.xml")
    parser.add_argument("--tmp-dir", type=Path, default=Path("D:/tmp") if Path("D:/tmp").exists() else None)
    parser.add_argument("--interactive", action="store_true", help="Open a live command/metrics panel and drive manually.")
    parser.add_argument("--interactive-vx-limit", type=float, default=1.0)
    parser.add_argument("--interactive-yaw-limit", type=float, default=1.0)
    parser.add_argument("--ui-update-dt", type=float, default=0.10)
    parser.add_argument("--viewer", action="store_true", help="Open MuJoCo passive viewer while running the test.")
    parser.add_argument("--realtime", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--realtime-scale", type=float, default=1.0)
    parser.add_argument(
        "--wheel-signs",
        type=float,
        nargs=4,
        default=[1.0, 1.0, 1.0, 1.0],
        metavar=("FL", "FR", "RL", "RR"),
        help="Per-wheel velocity sign multipliers in joint order.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.interactive:
        args.viewer = True
        args.duration = max(args.duration, 3600.0)
    tests = ["forward", "yaw", "arc", "waypoint", "slalom"] if args.test == "all" else [args.test]
    results = [run_one(name, args) for name in tests]
    print(json.dumps(results[0] if len(results) == 1 else results, indent=2, ensure_ascii=False))
    return 0 if all(r["success"] for r in results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
