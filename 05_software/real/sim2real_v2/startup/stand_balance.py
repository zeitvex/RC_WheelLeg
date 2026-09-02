from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict

import numpy as np


@dataclass
class StandBalanceDebug:
    roll: float
    pitch: float
    roll_rate: float
    pitch_rate: float
    hip_base: float
    knee_base: float
    roll_corr: float
    pitch_corr: float
    pitch_compensation_enabled: bool
    target: list[float]
    stable: bool


class StandBalanceController:
    def __init__(self, cfg: Dict[str, Any], control_dt: float):
        self.enabled = bool(cfg.get("enabled", True))
        self.control_dt = float(control_dt)
        self.height = float(cfg.get("height", 0.33))
        self.kp_roll = float(cfg.get("kp_roll", 0.85))
        self.pitch_compensation_enabled = bool(cfg.get("pitch_compensation_enabled", True))
        self.kp_pitch = float(cfg.get("kp_pitch", 0.70))
        self.kd_roll_rate = float(cfg.get("kd_roll_rate", 0.03))
        self.kd_pitch_rate = float(cfg.get("kd_pitch_rate", 0.025))
        self.pitch_deadband = float(np.radians(cfg.get("pitch_deadband_deg", 0.0)))
        self.pitch_corr_clip = float(cfg.get("pitch_corr_clip", 0.12))
        self.pitch_corr_filter_alpha = float(np.clip(cfg.get("pitch_corr_filter_alpha", 1.0), 0.0, 1.0))
        self.pitch_front_sign = float(cfg.get("pitch_front_sign", -1.0))
        self.lateral_lean_gain = float(cfg.get("lateral_lean_gain", 0.0))
        self.hip_abduction_clip = float(cfg.get("hip_abduction_clip", 0.45))
        self.hip_pitch_clip = tuple(cfg.get("hip_pitch_clip", [-1.0, 2.5]))
        self.knee_clip = tuple(cfg.get("knee_clip", [-2.6, -0.3]))
        self.stable_roll_deg = float(cfg.get("stable_roll_deg", 6.0))
        self.stable_pitch_deg = float(cfg.get("stable_pitch_deg", 8.0))
        self.stable_gyro_deg_s = float(cfg.get("stable_gyro_deg_s", 45.0))
        self.enter_hold_s = float(cfg.get("enter_hold_s", 1.0))

        self.profile_h = np.asarray(
            cfg.get("profile_h", [0.157, 0.248, 0.311, 0.366, 0.411, 0.448]),
            dtype=np.float32,
        )
        self.profile_hip = np.asarray(
            cfg.get("profile_hip", [1.5, 1.2, 1.0, 0.8, 0.6, 0.4]),
            dtype=np.float32,
        )
        self.profile_knee = np.asarray(
            cfg.get("profile_knee", [-2.5, -2.1, -1.8, -1.5, -1.2, -0.9]),
            dtype=np.float32,
        )
        self._stable_time = 0.0
        self._pitch_corr_filtered = 0.0
        self._last_debug = StandBalanceDebug(0.0, 0.0, 0.0, 0.0, 0.9, -1.8, 0.0, 0.0, False, [], False)

    @property
    def last_debug(self) -> StandBalanceDebug:
        return self._last_debug

    def reset(self) -> None:
        self._stable_time = 0.0
        self._pitch_corr_filtered = 0.0

    def _estimate_roll_pitch(self, projected_gravity: np.ndarray) -> tuple[float, float]:
        gx, gy, gz = [float(v) for v in projected_gravity]
        roll = float(np.arctan2(-gy, max(1e-6, -gz)))
        pitch = float(np.arctan2(gx, np.sqrt(max(1e-6, gy * gy + gz * gz))))
        return roll, pitch

    def _base_leg_pose(self) -> tuple[float, float]:
        h_clamp = float(np.clip(self.height, float(self.profile_h[0]), float(self.profile_h[-1])))
        hip = float(np.interp(h_clamp, self.profile_h, self.profile_hip))
        knee = float(np.interp(h_clamp, self.profile_h, self.profile_knee))
        return hip, knee

    def compute_target(self, state: Dict[str, Any], command: np.ndarray | None = None) -> np.ndarray:
        projected_gravity = np.asarray(state["projected_gravity"], dtype=np.float32)
        imu_gyro = np.asarray(state["imu_gyro"], dtype=np.float32)
        cmd = np.zeros(3, dtype=np.float32) if command is None else np.asarray(command, dtype=np.float32)

        hip_base, knee_base = self._base_leg_pose()
        roll, pitch = self._estimate_roll_pitch(projected_gravity)
        roll_rate = float(imu_gyro[0])
        pitch_rate = float(imu_gyro[1])

        roll_corr = -self.kp_roll * roll - self.kd_roll_rate * roll_rate
        if self.pitch_compensation_enabled:
            pitch_for_ctrl = 0.0 if abs(pitch) < self.pitch_deadband else pitch
            pitch_corr_raw = -self.kp_pitch * pitch_for_ctrl - self.kd_pitch_rate * pitch_rate
            pitch_corr_raw = float(np.clip(pitch_corr_raw, -self.pitch_corr_clip, self.pitch_corr_clip))
            alpha = self.pitch_corr_filter_alpha
            pitch_corr = (1.0 - alpha) * self._pitch_corr_filtered + alpha * pitch_corr_raw
            self._pitch_corr_filtered = pitch_corr
        else:
            pitch_corr = 0.0
            self._pitch_corr_filtered = 0.0
        lateral_lean = self.lateral_lean_gain * float(cmd[1])

        target = np.zeros(16, dtype=np.float32)
        for leg_idx in range(4):
            side = 1.0 if leg_idx in (0, 2) else -1.0
            fore_aft = self.pitch_front_sign if leg_idx in (0, 1) else -self.pitch_front_sign
            target[leg_idx * 3 + 0] = float(
                np.clip(side * roll_corr + lateral_lean, -self.hip_abduction_clip, self.hip_abduction_clip)
            )
            target[leg_idx * 3 + 1] = float(
                np.clip(hip_base + fore_aft * pitch_corr, self.hip_pitch_clip[0], self.hip_pitch_clip[1])
            )
            target[leg_idx * 3 + 2] = float(np.clip(knee_base, self.knee_clip[0], self.knee_clip[1]))
        target[12:] = 0.0

        stable = (
            abs(np.degrees(roll)) <= self.stable_roll_deg
            and abs(np.degrees(pitch)) <= self.stable_pitch_deg
            and max(abs(np.degrees(roll_rate)), abs(np.degrees(pitch_rate))) <= self.stable_gyro_deg_s
        )
        self._stable_time = self._stable_time + self.control_dt if stable else 0.0
        self._last_debug = StandBalanceDebug(
            roll=roll,
            pitch=pitch,
            roll_rate=roll_rate,
            pitch_rate=pitch_rate,
            hip_base=hip_base,
            knee_base=knee_base,
            roll_corr=roll_corr,
            pitch_corr=pitch_corr,
            pitch_compensation_enabled=self.pitch_compensation_enabled,
            target=target.tolist(),
            stable=stable,
        )
        return target

    def is_stable(self) -> bool:
        return self._stable_time >= self.enter_hold_s
