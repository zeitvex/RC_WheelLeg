"""Logging helpers for sim2real runs.

Each session writes:
- `state.csv`: high-rate state stream
- `events.jsonl`: event / milestone stream
"""

import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np


class LogBundle:
    """One session directory containing state CSV and event JSONL."""

    JOINT_LABELS = (
        "fl_hip_abd", "fl_hip_pitch", "fl_knee",
        "fr_hip_abd", "fr_hip_pitch", "fr_knee",
        "rl_hip_abd", "rl_hip_pitch", "rl_knee",
        "rr_hip_abd", "rr_hip_pitch", "rr_knee",
        "fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel",
    )

    def __init__(self, log_root: str = "logs"):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir = Path(log_root) / timestamp
        self.dir.mkdir(parents=True, exist_ok=True)

        self.state_path = self.dir / "state.csv"
        self.events_path = self.dir / "events.jsonl"

        self._state_fp = open(self.state_path, "w", encoding="utf-8")
        self._events_fp = open(self.events_path, "w", encoding="utf-8")
        self._t0 = time.time()
        self._closed = False

        self._write_state_header()
        self.event("LOG_START", session_dir=str(self.dir))
        print(f"[Log] {self.dir}")

    def _write_state_header(self):
        cols = ["t", "t_rel", "phase"]
        cols += [f"{joint}_pos" for joint in self.JOINT_LABELS]
        cols += [f"{joint}_vel" for joint in self.JOINT_LABELS]
        cols += [f"{joint}_tau" for joint in self.JOINT_LABELS]
        cols += [f"{joint}_tgt" for joint in self.JOINT_LABELS]
        cols += [f"{joint}_raw" for joint in self.JOINT_LABELS]
        cols += ["gyro_x", "gyro_y", "gyro_z"]
        cols += ["accel_x", "accel_y", "accel_z"]
        cols += ["quat_w", "quat_x", "quat_y", "quat_z"]
        cols += ["pgrav_x", "pgrav_y", "pgrav_z"]
        cols += ["cmd_vx", "cmd_vy", "cmd_yaw"]
        cols += ["imu_age_ms", "loop_dt_ms"]
        cols += ["safety_level", "guard_level"]
        cols += ["holdover", "stale_max", "fresh_count"]
        cols += ["kp_scale", "nan_flag"]
        cols += ["kp_leg_cmd", "kd_leg_cmd", "kd_wheel_cmd"]
        cols += ["runtime_release_alpha", "runtime_release_hold_s", "runtime_blend_ratio"]
        cols += ["hold_target_max_err", "policy_target_max_err", "hold_policy_max_gap"]
        cols += ["target_source_code"]
        cols += [
            "clip_primary_joint_index",
            "clip_primary_joint",
            "clip_primary_target",
            "clip_primary_measured",
            "clip_primary_default",
            "clip_primary_pos_err",
            "clip_primary_raw",
            "clip_primary_scaled",
        ]
        cols += ["safety_reason", "guard_reason"]
        self._state_fp.write(",".join(cols) + "\n")
        self._state_fp.flush()

    def state(
        self,
        phase: str,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        joint_torque: np.ndarray,
        target_pose: np.ndarray,
        raw_action: Optional[np.ndarray],
        gyro: np.ndarray,
        accel: np.ndarray,
        quat: np.ndarray,
        proj_gravity: np.ndarray,
        command: np.ndarray,
        imu_age_ms: float,
        loop_dt_ms: float,
        safety_level: int = 0,
        guard_level: int = 0,
        holdover: int = 0,
        stale_max: int = 0,
        fresh_count: int = 16,
        kp_scale: float = 1.0,
        nan_flag: int = 0,
        kp_leg_cmd: float = 0.0,
        kd_leg_cmd: float = 0.0,
        kd_wheel_cmd: float = 0.0,
        runtime_release_alpha: float = 0.0,
        runtime_release_hold_s: float = 0.0,
        runtime_blend_ratio: float = 0.0,
        hold_target_max_err: float = 0.0,
        policy_target_max_err: float = 0.0,
        hold_policy_max_gap: float = 0.0,
        target_source: str = "",
        clip_primary_joint: str = "",
        clip_primary_target: float = 0.0,
        clip_primary_measured: float = 0.0,
        clip_primary_default: float = 0.0,
        clip_primary_pos_err: float = 0.0,
        clip_primary_raw: float = 0.0,
        clip_primary_scaled: float = 0.0,
        safety_reason: str = "",
        guard_reason: str = "",
    ):
        if self._closed:
            return
        if target_pose is None:
            target_pose = np.zeros(16, dtype=np.float32)
        if raw_action is None:
            raw_action = np.zeros(16, dtype=np.float32)

        now = time.time()
        numeric_values = []
        numeric_values += joint_pos.tolist()
        numeric_values += joint_vel.tolist()
        numeric_values += joint_torque.tolist()
        numeric_values += target_pose.tolist()
        numeric_values += raw_action.tolist()
        numeric_values += gyro.tolist()
        numeric_values += accel.tolist()
        numeric_values += quat.tolist()
        numeric_values += proj_gravity.tolist()
        numeric_values += command.tolist()
        numeric_values += [imu_age_ms, loop_dt_ms]
        numeric_values += [safety_level, guard_level, holdover, stale_max, fresh_count, kp_scale, nan_flag]
        numeric_values += [kp_leg_cmd, kd_leg_cmd, kd_wheel_cmd]
        numeric_values += [runtime_release_alpha, runtime_release_hold_s, runtime_blend_ratio]
        numeric_values += [hold_target_max_err, policy_target_max_err, hold_policy_max_gap]
        numeric_values += [_target_source_code(target_source)]
        numeric_values += [_csv_numeric_joint_index(clip_primary_joint)]
        numeric_values += [
            clip_primary_target,
            clip_primary_measured,
            clip_primary_default,
            clip_primary_pos_err,
            clip_primary_raw,
            clip_primary_scaled,
        ]

        parts = [f"{now:.6f}", f"{now - self._t0:.6f}", phase]
        parts += [f"{value:.6f}" for value in numeric_values]
        parts += [_csv_escape(clip_primary_joint), _csv_escape(safety_reason), _csv_escape(guard_reason)]
        self._state_fp.write(",".join(parts) + "\n")

    def event(self, kind: str, **fields: Any):
        if self._closed:
            return
        record = {"t": time.time(), "t_rel": time.time() - self._t0, "kind": kind}
        for key, value in fields.items():
            if isinstance(value, np.ndarray):
                record[key] = value.tolist()
            elif isinstance(value, (np.integer, np.floating)):
                record[key] = value.item()
            else:
                record[key] = value
        self._events_fp.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._events_fp.flush()
        if kind != "STATE_TICK":
            print(f"[Event {record['t_rel']:7.2f}s] {kind} {_short_fields(fields)}")

    def flush(self):
        if not self._closed:
            self._state_fp.flush()
            self._events_fp.flush()

    def close(self):
        if self._closed:
            return
        self.event("LOG_END")
        self._state_fp.flush()
        self._state_fp.close()
        self._events_fp.flush()
        self._events_fp.close()
        self._closed = True
        print(f"[Log] saved -> {self.dir}")


def _csv_escape(text: str) -> str:
    if not text:
        return ""
    return text.replace(",", ";").replace("\n", " ").replace("\r", " ")


def _csv_numeric_joint_index(joint_name: str) -> float:
    if not joint_name:
        return -1.0
    try:
        return float(LogBundle.JOINT_LABELS.index(joint_name))
    except ValueError:
        return -1.0


def _target_source_code(target_source: str) -> float:
    mapping = {
        "": -1.0,
        "startup_hold": 0.0,
        "stand_balance": 1.0,
        "runtime_hold": 2.0,
        "runtime_blend": 3.0,
        "runtime_policy": 4.0,
    }
    return mapping.get(target_source, 99.0)


def _short_fields(fields: Dict[str, Any]) -> str:
    parts = []
    for key, value in fields.items():
        if isinstance(value, (list, tuple, np.ndarray)):
            arr = np.asarray(value).ravel()
            if arr.size > 4:
                continue
            try:
                parts.append(f"{key}=[{','.join(f'{float(x):.2f}' for x in arr)}]")
            except (TypeError, ValueError):
                parts.append(f"{key}={list(arr)[:4]}")
        elif isinstance(value, float):
            parts.append(f"{key}={value:.3f}")
        else:
            parts.append(f"{key}={value}")
    return " ".join(parts)


SimpleLogger = LogBundle
