"""Web-facing session state machine for current sim2real deployment."""

from __future__ import annotations

import threading
import time
import traceback
import queue
from collections import deque
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, Optional

import numpy as np

from tools.logger import LogBundle
from input_dev.remote_uart import RemoteCommandSource


class Stage(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"
    ENABLING = "ENABLING"
    ENABLED = "ENABLED"
    JOINT_TEST = "JOINT_TEST"
    CALIBRATING = "CALIBRATING"
    STARTING_UP = "STARTING_UP"
    STAND_HOLD = "STAND_HOLD"
    RUNTIME = "RUNTIME"
    FAULTED = "FAULTED"
    ESTOPPED = "ESTOPPED"


@dataclass
class SessionStatus:
    stage: str = Stage.DISCONNECTED.value
    detail: str = ""
    last_event: str = ""
    busy: bool = False
    cmd: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    input_mode: str = "WEB"
    remote_takeover_active: bool = False
    remote_takeover_allowed: bool = False
    remote_soft_estop: bool = False
    remote_status: Dict[str, Any] = field(default_factory=dict)
    last_state: Optional[Dict[str, Any]] = None
    log_dir: Optional[str] = None
    fault_reason: Optional[str] = None
    last_error: Optional[str] = None
    last_traceback: Optional[str] = None
    diagnostics: Dict[str, Any] = field(default_factory=dict)


class RobotSession:
    JOINT_LABELS = LogBundle.JOINT_LABELS
    DEFAULT_ACTION_SCALE = np.array(
        [
            0.125, 0.25, 0.25,
            0.125, 0.25, 0.25,
            0.125, 0.25, 0.25,
            0.125, 0.25, 0.25,
            5.0, 5.0, 5.0, 5.0,
        ],
        dtype=np.float32,
    )

    def __init__(
        self,
        cfg: Dict[str, Any],
        cfg_path: Path,
        driver_factory_real: Callable,
        driver_factory_dry: Callable,
    ):
        self.cfg = cfg
        self.cfg_path = cfg_path
        self.driver_factory_real = driver_factory_real
        self.driver_factory_dry = driver_factory_dry

        self.lock = threading.RLock()
        self.status = SessionStatus()

        self.io = None
        self.runner = None
        self.guard = None
        self.safety = None
        self.initializer = None
        self.stand_balance = None
        self.logger = None

        self._stop_runtime = threading.Event()
        self._cmd_lock = threading.Lock()
        self._cmd = np.zeros(3, dtype=np.float32)
        self._filtered_cmd = np.zeros(3, dtype=np.float32)
        self._last_raw_cmd = np.zeros(3, dtype=np.float32)
        self._estop = False
        self._busy_thread: Optional[threading.Thread] = None
        self._stand_target = None

        self._remote_source: Optional[RemoteCommandSource] = None
        self._remote_takeover_active: bool = False
        self._remote_soft_estop: bool = False
        self._remote_poll_error_count: int = 0
        self._remote_last_error: Optional[str] = None

        self._stop_poll = threading.Event()
        self._poll_thread: Optional[threading.Thread] = None

        self._event_listeners: list = []
        self._recent_events = deque(maxlen=300)
        self._listener_lock = threading.Lock()

        self._last_runtime_ts: float = 0.0
        self._last_poll_ts: float = 0.0
        self._last_command_ts: float = 0.0
        self._runtime_loop_count: int = 0
        self._runtime_overrun_count: int = 0
        self._runtime_overrun_max_ms: float = 0.0
        self._runtime_policy_stale_count: int = 0
        self._runtime_policy_stale_max_ms: float = 0.0
        self._last_loop_profile: Dict[str, float] = {}
        self._latest_target_info: Dict[str, Any] = {}
        self._latest_target: Optional[np.ndarray] = None
        self._poll_error_count: int = 0
        self._api_error_count: int = 0
        self._disconnecting: bool = False

        self._target_queue = queue.Queue(maxsize=1)
        self._state_lock = threading.Lock()
        self._latest_hardware_state = None
        self._policy_exception = None
        self._motor_exception = None
        self._status_exception = None
        self._runtime_error_event = threading.Event()

    def _policy_action_scale(self) -> np.ndarray:
        values = self.cfg.get("policy", {}).get("action_scale", self.DEFAULT_ACTION_SCALE.tolist())
        action_scale = np.asarray(values, dtype=np.float32)
        if action_scale.shape != (16,):
            raise ValueError(f"policy.action_scale must be 16 values, got shape {action_scale.shape}")
        return action_scale

    def _diag_snapshot(self) -> Dict[str, Any]:
        return {
            "runtime_active": bool(self.status.stage == Stage.RUNTIME.value and not self._stop_runtime.is_set()),
            "runtime_loop_count": int(self._runtime_loop_count),
            "runtime_overrun_count": int(self._runtime_overrun_count),
            "runtime_overrun_max_ms": round(float(self._runtime_overrun_max_ms), 3),
            "runtime_policy_stale_count": int(self._runtime_policy_stale_count),
            "runtime_policy_stale_max_ms": round(float(self._runtime_policy_stale_max_ms), 3),
            "last_loop_profile": dict(self._last_loop_profile),
            "latest_target": self._latest_target_snapshot(),
            "filtered_cmd": self._filtered_cmd.tolist(),
            "raw_cmd": self._last_raw_cmd.tolist(),
            "last_runtime_age_s": round(time.time() - self._last_runtime_ts, 3) if self._last_runtime_ts else None,
            "last_poll_age_s": round(time.time() - self._last_poll_ts, 3) if self._last_poll_ts else None,
            "last_command_age_s": round(time.time() - self._last_command_ts, 3) if self._last_command_ts else None,
            "poll_thread_alive": bool(self._poll_thread and self._poll_thread.is_alive()),
            "busy_thread_alive": bool(self._busy_thread and self._busy_thread.is_alive()),
            "estop": bool(self._estop),
            "poll_error_count": int(self._poll_error_count),
            "api_error_count": int(self._api_error_count),
            "remote_poll_error_count": int(self._remote_poll_error_count),
            "remote_last_error": self._remote_last_error,
            "remote_takeover_active": bool(self._remote_takeover_active),
            "remote_soft_estop": bool(self._remote_soft_estop),
            "zero_cmd_suppression": (
                bool(getattr(self.runner, "enable_zero_cmd_suppression", False))
                if self.runner is not None
                else None
            ),
            "policy_path": str(getattr(self.runner, "policy_path", "")) if self.runner is not None else None,
            "stand_balance": self._stand_balance_snapshot(),
        }

    def _stand_balance_snapshot(self) -> Dict[str, Any]:
        if self.stand_balance is None:
            return {"enabled": False}
        debug = self.stand_balance.last_debug
        return {
            "enabled": bool(self.stand_balance.enabled),
            "pitch_compensation_enabled": bool(getattr(debug, "pitch_compensation_enabled", False)),
            "roll_deg": float(np.degrees(debug.roll)),
            "pitch_deg": float(np.degrees(debug.pitch)),
            "roll_rate_deg_s": float(np.degrees(debug.roll_rate)),
            "pitch_rate_deg_s": float(np.degrees(debug.pitch_rate)),
            "hip_base": float(debug.hip_base),
            "knee_base": float(debug.knee_base),
            "roll_corr": float(debug.roll_corr),
            "pitch_corr": float(debug.pitch_corr),
            "stable": bool(debug.stable),
        }

    def _policy_release_cfg(self) -> Dict[str, float]:
        policy_cfg = self.cfg.get("policy", {})
        return {
            "command_hold_s": max(float(policy_cfg.get("release_command_hold_s", 0.12)), 0.0),
            "posture_max_err": max(float(policy_cfg.get("release_posture_max_err", 0.35)), 0.0),
            "target_blend_s": max(float(policy_cfg.get("release_target_blend_s", 0.30)), 1e-3),
        }

    def _compute_release_metrics(self, state: Dict[str, Any], hold_target: np.ndarray, cmd: np.ndarray) -> Dict[str, float]:
        joint_pos = np.asarray(state["joint_pos"], dtype=np.float32)
        default_pose = np.asarray(self.runner.default_dof_pos, dtype=np.float32)
        hold_target = np.asarray(hold_target, dtype=np.float32)
        planar_cmd, yaw_cmd = self.runner.command_activation_metrics(cmd)
        return {
            "planar_cmd": float(planar_cmd),
            "yaw_cmd": float(yaw_cmd),
            "max_hold_err": float(np.max(np.abs(joint_pos[:12] - hold_target[:12]))),
            "max_default_err": float(np.max(np.abs(joint_pos[:12] - default_pose[:12]))),
            "max_hold_default_gap": float(np.max(np.abs(hold_target[:12] - default_pose[:12]))),
        }

    def _blend_runtime_target(
        self,
        hold_target: np.ndarray,
        policy_target: np.ndarray,
        release_alpha: float,
        target_blend_s: float,
        control_dt: float,
    ) -> np.ndarray:
        blend = min(1.0, release_alpha * (self.runner.command_release_s / max(target_blend_s, control_dt)))
        return ((1.0 - blend) * hold_target + blend * policy_target).astype(np.float32)

    def _compute_target_error_metrics(
        self,
        state: Dict[str, Any],
        hold_target: np.ndarray,
        policy_target: np.ndarray,
    ) -> Dict[str, float]:
        joint_pos = np.asarray(state["joint_pos"], dtype=np.float32)
        hold_target = np.asarray(hold_target, dtype=np.float32)
        policy_target = np.asarray(policy_target, dtype=np.float32)
        return {
            "hold_target_max_err": float(np.max(np.abs(joint_pos[:12] - hold_target[:12]))),
            "policy_target_max_err": float(np.max(np.abs(joint_pos[:12] - policy_target[:12]))),
            "hold_policy_max_gap": float(np.max(np.abs(hold_target[:12] - policy_target[:12]))),
        }

    def _record_loop_profile(self, profile: Dict[str, float]) -> Dict[str, float]:
        compact = {k: round(float(v), 3) for k, v in profile.items()}
        self._last_loop_profile = compact
        return compact

    def _note_overrun(self, over_ms: float) -> None:
        self._runtime_overrun_count += 1
        self._runtime_overrun_max_ms = max(self._runtime_overrun_max_ms, float(over_ms))

    def _note_policy_stale(self, age_ms: float) -> None:
        self._runtime_policy_stale_count += 1
        self._runtime_policy_stale_max_ms = max(self._runtime_policy_stale_max_ms, float(age_ms))

    def _build_last_state(
        self,
        *,
        state: Dict[str, Any],
        target: np.ndarray,
        raw: np.ndarray,
        projected_gravity: np.ndarray,
        cmd: np.ndarray,
        loop_dt_ms: float,
        phase: str,
        safety_level: int = 0,
        guard_level: int = 0,
        safety_reason: str = "",
        guard_reason: str = "",
        zero_command: bool = True,
        runtime_released: bool = False,
        release_alpha: float = 0.0,
        release_active_hold_s: float = 0.0,
        release_max_hold_err: float = 0.0,
        loop_profile: Optional[Dict[str, float]] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        motor_diag = state.get("motor_stale", {}) or {}
        extra_dict = extra or {}
        raw_cmd_list = extra_dict.get("raw_cmd", [0.0, 0.0, 0.0])
        filtered_cmd_list = cmd.tolist()
        payload: Dict[str, Any] = {
            "joint_pos": state["joint_pos"].tolist(),
            "joint_vel": state["joint_vel"].tolist(),
            "joint_torque": state["joint_torque"].tolist(),
            "target": target.tolist(),
            "raw": raw.tolist(),
            "gyro": state["imu_gyro"].tolist(),
            "proj_gravity": projected_gravity.tolist(),
            "imu_age_ms": float(state["imu_age_ms"]),
            "imu_fresh": bool(state.get("imu_fresh", False)),
            "odom": state.get("odom"),
            "loop_dt_ms": float(loop_dt_ms),
            "loop_profile": loop_profile or {},
            "holdover_total": int(self.io.hw.holdover_total),
            "safety_level": int(safety_level),
            "guard_level": int(guard_level),
            "phase": phase,
            "cmd": cmd.tolist(),
            "raw_cmd": raw_cmd_list,
            "filtered_cmd": filtered_cmd_list,
            "safety_reason": safety_reason,
            "guard_reason": guard_reason,
            "zero_command": bool(zero_command),
            "runtime_released": bool(runtime_released),
            "release_alpha": float(release_alpha),
            "release_active_hold_s": float(release_active_hold_s),
            "release_max_hold_err": float(release_max_hold_err),
            "per_motor_stale": motor_diag.get("per_motor_stale", [0] * 16),
            "motor_fresh_count": int(motor_diag.get("fresh_count", 16)),
            "motor_fresh_by_update_count": int(motor_diag.get("fresh_by_update_count", 0)),
            "motor_fresh_by_value_change": int(motor_diag.get("fresh_by_value_change", 0)),
            "motor_update_counts": motor_diag.get("update_counts", [0] * 16),
            "motor_temperatures": motor_diag.get("temperatures", [0.0] * 16),
            "motor_fault_codes": motor_diag.get("fault_codes", [0] * 16),
            "motor_mode_states": motor_diag.get("mode_states", [0] * 16),
            "latest_target": self._latest_target_snapshot(),
        }
        if extra:
            payload.update(extra)
        return payload

    def _signal_stats(self, obs: Optional[np.ndarray], raw: Optional[np.ndarray], scaled: Optional[np.ndarray]) -> Dict[str, float]:
        stats: Dict[str, float] = {}
        if obs is not None and np.asarray(obs).size:
            obs_arr = np.asarray(obs, dtype=np.float32)
            stats.update(
                {
                    "obs_min": float(np.min(obs_arr)),
                    "obs_max": float(np.max(obs_arr)),
                    "obs_abs_max": float(np.max(np.abs(obs_arr))),
                }
            )
        if raw is not None and np.asarray(raw).size:
            raw_arr = np.asarray(raw, dtype=np.float32)
            stats.update(
                {
                    "raw_min": float(np.min(raw_arr)),
                    "raw_max": float(np.max(raw_arr)),
                    "raw_abs_max": float(np.max(np.abs(raw_arr))),
                }
            )
        if scaled is not None and np.asarray(scaled).size:
            scaled_arr = np.asarray(scaled, dtype=np.float32)
            stats.update(
                {
                    "scaled_min": float(np.min(scaled_arr)),
                    "scaled_max": float(np.max(scaled_arr)),
                    "scaled_abs_max": float(np.max(np.abs(scaled_arr))),
                }
            )
        return stats

    def _update_latest_target(self, target: np.ndarray, source: str) -> None:
        target = np.asarray(target, dtype=np.float32)
        now = time.time()
        delta_max = 0.0
        if self._latest_target is not None and self._latest_target.shape == target.shape:
            delta_max = float(np.max(np.abs(target - self._latest_target)))
        self._latest_target = target.copy()
        self._latest_target_info = {
            "source": source,
            "t": now,
            "delta_max": delta_max,
            "target_abs_max": float(np.max(np.abs(target))) if target.size else 0.0,
        }

    def _latest_target_snapshot(self) -> Dict[str, Any]:
        if not self._latest_target_info:
            return {}
        snap = dict(self._latest_target_info)
        snap["age_ms"] = (time.time() - float(snap.get("t", time.time()))) * 1000.0
        return snap

    def _filter_command(self, raw_cmd: np.ndarray, dt: float) -> np.ndarray:
        raw_cmd = np.asarray(raw_cmd, dtype=np.float32)
        self._last_raw_cmd = raw_cmd.copy()
        cfg = self.cfg.get("command_filter", {}) or {}
        if not bool(cfg.get("enabled", False)):
            self._filtered_cmd = raw_cmd.copy()
            return raw_cmd

        limits = np.array(
            [
                float(cfg.get("max_vx_acc", 1.0)),
                float(cfg.get("max_vy_acc", 1.0)),
                float(cfg.get("max_yaw_acc", 1.5)),
            ],
            dtype=np.float32,
        )
        max_delta = np.maximum(limits * max(float(dt), 1e-3), 0.0)
        delta = np.clip(raw_cmd - self._filtered_cmd, -max_delta, max_delta)
        self._filtered_cmd = (self._filtered_cmd + delta).astype(np.float32)
        return self._filtered_cmd.copy()

    def _remote_cfg(self) -> Dict[str, Any]:
        return dict(self.cfg.get("remote", {}) or {})

    def _build_remote_source(self) -> Optional[RemoteCommandSource]:
        remote_cfg = self._remote_cfg()
        port = str(remote_cfg.get("port") or "").strip()
        if not remote_cfg.get("enabled", False) or not port:
            return None
        return RemoteCommandSource(
            port=port,
            baudrate=int(remote_cfg.get("baudrate", 100000)),
            timeout=float(remote_cfg.get("timeout", 0.02)),
            axis_deadzone=int(remote_cfg.get("axis_deadzone", 50)),
            active_threshold=int(remote_cfg.get("active_threshold", 50)),
            axis_full_scale=float(remote_cfg.get("axis_full_scale", 660.0)),
            max_vx=float(remote_cfg.get("max_vx", self.cfg["controller"]["max_vx"])),
            max_vy=float(remote_cfg.get("max_vy", self.cfg["controller"]["max_vy"])),
            max_yaw=float(remote_cfg.get("max_yaw_rate", self.cfg["controller"]["max_yaw_rate"])),
            invert_vx=bool(remote_cfg.get("invert_vx", False)),
            invert_vy=bool(remote_cfg.get("invert_vy", False)),
            invert_yaw=bool(remote_cfg.get("invert_yaw", False)),
        )

    def _remote_takeover_allowed(self) -> bool:
        return bool(
            self.status.stage == Stage.RUNTIME.value
            and self._remote_source is not None
        )

    def _refresh_input_status(self) -> None:
        remote_status = {}
        if self._remote_source is not None:
            remote_status = self._remote_source.get_status()
            remote_status["available"] = True
            remote_status["last_error"] = self._remote_last_error
        else:
            remote_status = {"available": False, "last_error": self._remote_last_error}
        self.status.input_mode = "REMOTE" if self._remote_takeover_active else "WEB"
        self.status.remote_takeover_active = bool(self._remote_takeover_active)
        self.status.remote_takeover_allowed = bool(self._remote_takeover_allowed())
        self.status.remote_soft_estop = bool(self._remote_soft_estop)
        self.status.remote_status = remote_status

    def _poll_remote(self) -> None:
        if self._remote_source is None:
            self._remote_soft_estop = False
            self._refresh_input_status()
            return
        try:
            remote_state = self._remote_source.poll()
            self._remote_last_error = None
            self._remote_soft_estop = bool(remote_state.estop_requested)
        except Exception as exc:
            self._remote_poll_error_count += 1
            self._remote_last_error = f"{type(exc).__name__}: {exc}"
            self._remote_soft_estop = False
            if self._remote_poll_error_count <= 3 or self._remote_poll_error_count % 20 == 0:
                self._broadcast({"kind": "REMOTE_POLL_ERROR", "error": self._remote_last_error})
        self._refresh_input_status()

    def _update_diag_locked(self) -> None:
        self._refresh_input_status()
        self.status.diagnostics = self._diag_snapshot()

    def note_api_error(self) -> None:
        with self.lock:
            self._api_error_count += 1
            self._update_diag_locked()

    def _set_fault(self, exc: Exception, tb: str) -> None:
        error_text = f"{type(exc).__name__}: {exc}"
        with self.lock:
            self.status.fault_reason = error_text
            self.status.last_error = error_text
            self.status.last_traceback = tb
            self._update_diag_locked()

    def get_status(self) -> Dict[str, Any]:
        with self.lock:
            self._update_diag_locked()
            return asdict(self.status)

    def get_debug_snapshot(self) -> Dict[str, Any]:
        return {"status": self.get_status(), "recent_events": list(self._recent_events)}

    def _set(self, **kwargs):
        with self.lock:
            for key, value in kwargs.items():
                setattr(self.status, key, value)
            self._update_diag_locked()
            snapshot = asdict(self.status)
        self._broadcast({"kind": "STATUS", **snapshot})

    def _set_stage(self, stage: Stage, detail: str = ""):
        self._set(stage=stage.value, detail=detail)

    def _build_action_diag(
        self,
        *,
        state: Dict[str, Any],
        raw: np.ndarray,
        scaled: np.ndarray,
        tentative: np.ndarray,
        cmd: np.ndarray,
        zero_command: bool,
        runtime_released: bool,
        release_alpha: float,
        safety_details: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        details = dict(safety_details or {})
        joint_indices = list(details.get("joint_indices", []))
        joint_pos = np.asarray(state["joint_pos"], dtype=np.float32)
        default_pose = np.asarray(self.runner.default_dof_pos, dtype=np.float32)
        pos_err = tentative - joint_pos
        leg_offset = tentative[:12] - default_pose[:12]

        diag: Dict[str, Any] = {
            "joint_indices": joint_indices,
            "joint_names": [self.JOINT_LABELS[i] for i in joint_indices if 0 <= i < len(self.JOINT_LABELS)],
            "cmd": cmd.tolist(),
            "zero_command": bool(zero_command),
            "runtime_released": bool(runtime_released),
            "release_alpha": float(release_alpha),
            "max_raw": float(np.max(np.abs(raw))) if raw.size else 0.0,
            "max_scaled": float(np.max(np.abs(scaled[:12]))) if scaled.size else 0.0,
            "max_target": float(np.max(np.abs(tentative[:12]))) if tentative.size else 0.0,
        }

        if joint_indices:
            primary = int(joint_indices[0])
            diag.update(
                {
                    "primary_joint_index": primary,
                    "primary_joint_name": self.JOINT_LABELS[primary],
                    "primary_target": float(tentative[primary]),
                    "primary_default": float(default_pose[primary]),
                    "primary_measured": float(joint_pos[primary]),
                    "primary_pos_err": float(pos_err[primary]),
                    "primary_raw": float(raw[primary]),
                    "primary_scaled": float(scaled[primary]),
                }
            )
            if primary < 12:
                diag["primary_leg_offset"] = float(leg_offset[primary])

        details.update(diag)
        return details

    def add_listener(self, q: "queue.Queue"):
        with self._listener_lock:
            self._event_listeners.append(q)
            for event in list(self._recent_events):
                try:
                    q.put_nowait(event)
                except Exception:
                    pass

    def remove_listener(self, q: "queue.Queue"):
        with self._listener_lock:
            if q in self._event_listeners:
                self._event_listeners.remove(q)

    def _broadcast(self, event: Dict[str, Any]):
        payload = dict(event)
        payload["t"] = time.time()
        self._recent_events.append(payload)
        with self._listener_lock:
            for listener in list(self._event_listeners):
                try:
                    listener.put_nowait(payload)
                except Exception:
                    pass

    def _run_async(self, fn, *args, **kwargs) -> bool:
        with self.lock:
            if self.status.busy:
                return False
            self.status.busy = True
            self.status.fault_reason = None
            self.status.last_error = None
            self.status.last_traceback = None
            self._update_diag_locked()

        def _wrap():
            try:
                fn(*args, **kwargs)
            except Exception as exc:
                tb = traceback.format_exc()
                print(f"\n[Background Task Error] {fn.__name__}")
                print(tb)
                self._set_fault(exc, tb)
                self._broadcast({"kind": "BG_TASK_ERROR", "fn": fn.__name__, "error": str(exc), "traceback": tb})
                self._set_stage(Stage.FAULTED, detail=str(exc))
                try:
                    if self.io:
                        self.io.damping_brake()
                except Exception:
                    pass
            finally:
                with self.lock:
                    self.status.busy = False
                    self._update_diag_locked()
                self._broadcast({"kind": "BG_TASK_DONE", "fn": fn.__name__})

        self._busy_thread = threading.Thread(target=_wrap, daemon=True)
        self._busy_thread.start()
        return True

    def connect(self, dry_run: bool = False):
        return self._run_async(self._do_connect, dry_run)

    def _do_connect(self, dry_run: bool):
        if self.status.stage != Stage.DISCONNECTED.value:
            self._broadcast({"kind": "WARN", "msg": "already connected"})
            return
        self._set_stage(Stage.CONNECTING, "connecting hardware")

        from interface.real_io import RealIO
        from safety.runtime_guard import RuntimeGuard
        from safety.safety_monitor import SafetyMonitor
        from startup.pose_initializer import PoseInitializer
        from startup.stand_balance import StandBalanceController
        from tools.logger import LogBundle

        cfg = self.cfg
        control_dt = 1.0 / float(cfg["control_freq"])
        motor_dt = 1.0 / float(cfg.get("motor_freq", 200))
        driver_factory = self.driver_factory_dry() if dry_run else self.driver_factory_real()

        self.logger = LogBundle(cfg["log_dir"])
        self._set(log_dir=str(self.logger.dir))
        self.logger.event(
            "CONFIG_LOADED",
            config_path=str(self.cfg_path),
            dry_run=dry_run,
            control_freq=cfg["control_freq"],
            motor_model=cfg["motor_model"],
        )

        self.io = RealIO(
            driver_factory=driver_factory,
            motor_model=cfg["motor_model"],
            can1_port=cfg["can1_port"],
            can2_port=cfg["can2_port"],
            imu_lib_path=cfg.get("imu_lib_path"),
            control_dt=control_dt,
            motor_dt=motor_dt,
            kp_leg=cfg["controller"]["kp_leg"],
            kd_leg=cfg["controller"]["kd_leg"],
            hold_kp_leg=cfg["controller"].get("hold_kp_leg", cfg["controller"]["kp_leg"]),
            hold_kd_leg=cfg["controller"].get("hold_kd_leg", cfg["controller"]["kd_leg"]),
            kd_wheel=cfg["controller"]["kd_wheel"],
            debug=cfg.get("debug", False),
            dry_run=dry_run,
        )
        self.guard = RuntimeGuard(
            max_ang_vel=cfg["safety"]["max_ang_vel"],
            max_tilt_z=cfg["safety"]["max_tilt_z"],
            imu_age_warn_ms=cfg["safety"].get("imu_age_warn_ms", 60.0),
            imu_age_stop_ms=cfg["safety"].get("imu_age_stop_ms", 200.0),
        )
        self.safety = SafetyMonitor(
            max_target_offset=cfg["safety"]["max_target_offset"],
            max_ang_vel=cfg["safety"]["max_ang_vel"],
            max_tilt_z=cfg["safety"]["max_tilt_z"],
            clip_to_brake=cfg["safety"].get("clip_to_brake", 0),
            hard_target_offset=cfg["safety"].get("hard_target_offset", 1.2),
        )
        self.initializer = PoseInitializer(
            self.io,
            control_dt=control_dt,
            transition_time_min=cfg["startup"].get("transition_time_min", 2.0),
            transition_time_max=cfg["startup"].get("transition_time_max", 6.0),
            transition_seconds_per_rad=cfg["startup"].get("transition_seconds_per_rad", 1.5),
            hold_time=cfg["startup"]["hold_time"],
            settle_pos_threshold=cfg["startup"]["settle_pos_threshold"],
            settle_vel_threshold=cfg["startup"]["settle_vel_threshold"],
            timeout_extra=cfg["startup"].get("timeout_extra", 3.0),
            imu_fresh_wait_s=cfg["startup"].get("imu_fresh_wait_s", 1.0),
            progress_log_interval=cfg["startup"]["progress_log_interval"],
            ramp_kp_time=cfg["startup"].get("ramp_kp_time", 1.0),
            soft_hold_duration=cfg["startup"].get("soft_hold_duration", 1.0),
            max_dev_warn=cfg["startup"].get("max_dev_warn", 1.5),
            max_dev_abort=cfg["startup"].get("max_dev_abort", 3.0),
        )
        self.stand_balance = StandBalanceController(cfg.get("stand_balance", {}), control_dt=control_dt)
        self._remote_source = self._build_remote_source()
        self._remote_takeover_active = False
        self._remote_soft_estop = False
        self._remote_last_error = None

        class _WebEstop:
            def __init__(self, owner):
                self.owner = owner

            def is_estop_triggered(self):
                return self.owner._estop

        self.initializer.attach(self.logger, self.guard, _WebEstop(self))
        self.io.connect(imu_timeout_ms=cfg.get("imu_start_timeout_ms", 8000))
        if self._remote_source is not None:
            try:
                self._remote_source.open()
                self.logger.event("REMOTE_CONNECTED", port=self._remote_source.port)
                self._poll_remote()
            except Exception as exc:
                print(f"[RobotSession] 警告: 无法打开遥控器串口 {self._remote_source.port} ({exc})。已自动禁用遥控。")
                self._remote_source = None
        self.logger.event("CAN_IMU_CONNECTED", initial_gravity=self.io.imu.initial_gravity)
        self._set_stage(Stage.CONNECTED, "hardware connected")

    def disconnect(self):
        return self._run_async(self._do_disconnect)

    def _do_disconnect(self):
        if self._disconnecting:
            return
        self._disconnecting = True
        self._stop_runtime.set()
        self._stop_state_poll()
        time.sleep(0.05)
        try:
            if self.io:
                self.io.damping_brake()
        except Exception:
            pass
        try:
            if self._remote_source:
                self._remote_source.close()
        except Exception:
            pass
        try:
            if self._remote_source:
                self._remote_source.close()
        except Exception:
            pass
        try:
            if self.io:
                self.io.disconnect()
        except Exception:
            pass
        if self.logger:
            self.logger.event("HARDWARE_DISCONNECTED")
            self.logger.close()
        self.io = None
        self.runner = None
        self.logger = None
        self._stand_target = None
        self._remote_source = None
        self._remote_takeover_active = False
        self._remote_soft_estop = False
        self._remote_last_error = None
        self._set_stage(Stage.DISCONNECTED, "hardware disconnected")
        self._disconnecting = False

    def enable_motors(self):
        return self._run_async(self._do_enable)

    def _do_enable(self):
        if self.status.stage not in (Stage.CONNECTED.value, Stage.STAND_HOLD.value, Stage.FAULTED.value):
            return
        self._set_stage(Stage.ENABLING, "enabling motors")
        self.io.enable_motors()
        self.logger.event("MOTORS_ENABLED")
        time.sleep(0.5)
        self._set_stage(Stage.ENABLED, "motors enabled")
        self._start_state_poll()

    def _start_state_poll(self):
        self._stop_poll.clear()
        if self._poll_thread and self._poll_thread.is_alive():
            return

        def _poll_loop():
            while not self._stop_poll.is_set():
                if self.status.stage == Stage.RUNTIME.value:
                    time.sleep(0.2)
                    continue
                try:
                    stage = self.status.stage
                    self._poll_remote()
                    if stage == Stage.ENABLED.value:
                        self.io.hw.passive_poll()

                    state = self.io.read_state()
                    if stage == Stage.STAND_HOLD.value and self.stand_balance is not None and self.stand_balance.enabled:
                        self._stand_target = self.stand_balance.compute_target(state, np.zeros(3, dtype=np.float32))
                        self.io.hold_pose(self._stand_target, kp_scale=1.0)
                        self._update_latest_target(self._stand_target, "stand_balance")
                    elif stage == Stage.STAND_HOLD.value and self._stand_target is not None:
                        self.io.hold_pose(self._stand_target, kp_scale=1.0)
                        self._update_latest_target(self._stand_target, "stand_hold")

                    motor_diag = state.get("motor_stale", {})
                    self._last_poll_ts = time.time()
                    self._set(
                        last_state={
                            "joint_pos": state["joint_pos"].tolist(),
                            "joint_vel": state["joint_vel"].tolist(),
                            "joint_torque": state["joint_torque"].tolist(),
                            "target": self._stand_target.tolist() if self._stand_target is not None else [0.0] * 16,
                            "raw": [0.0] * 16,
                            "gyro": state["imu_gyro"].tolist(),
                            "proj_gravity": state["projected_gravity"].tolist(),
                            "imu_age_ms": float(state["imu_age_ms"]),
                            "loop_dt_ms": 0.0,
                            "holdover_total": int(getattr(self.io.hw, "holdover_total", 0)),
                            "safety_level": 0,
                            "guard_level": 0,
                            "phase": "POLL",
                            "stand_balance": self._stand_balance_snapshot(),
                            "latest_target": self._latest_target_snapshot(),
                            "per_motor_stale": motor_diag.get("per_motor_stale", [0] * 16),
                        }
                    )
                except Exception as exc:
                    self._poll_error_count += 1
                    self._broadcast({"kind": "POLL_ERROR", "error": str(exc), "traceback": traceback.format_exc()})
                time.sleep(0.2)

        self._poll_thread = threading.Thread(target=_poll_loop, daemon=True)
        self._poll_thread.start()

    def _stop_state_poll(self):
        self._stop_poll.set()
        thread = self._poll_thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=0.5)
        self._poll_thread = None

    def disable_motors(self):
        return self._run_async(self._do_disable)

    def _do_disable(self):
        self._stop_state_poll()
        try:
            self.io.damping_brake()
        except Exception:
            pass
        time.sleep(0.05)
        self.io.disable_motors()
        self.logger.event("MOTORS_DISABLED")
        self._set_stage(Stage.CONNECTED, "motors disabled")

    def startup(self):
        return self._run_async(self._do_startup)

    def _do_startup(self):
        from startup.pose_initializer import PoseInitFailed, STAND_POSE

        if self.status.stage != Stage.ENABLED.value:
            raise RuntimeError("startup requires ENABLED")

        self._set_stage(Stage.STARTING_UP, detail="transition to stand pose")
        try:
            target = self.initializer.transition_to_stand_from_current(target_pose=STAND_POSE)
            self._stand_target = target
            if self.stand_balance is not None and self.stand_balance.enabled:
                self.logger.event("STAND_BALANCE_BEGIN")
                self.stand_balance.reset()
                stable_deadline = time.perf_counter() + 6.0
                while time.perf_counter() < stable_deadline:
                    state = self.io.read_state()
                    self._stand_target = self.stand_balance.compute_target(state, np.zeros(3, dtype=np.float32))
                    self.io.hold_pose(self._stand_target, kp_scale=1.0)
                    if self.stand_balance.is_stable():
                        debug = self.stand_balance.last_debug
                        self.logger.event(
                            "STAND_BALANCE_STABLE",
                            roll_deg=float(np.degrees(debug.roll)),
                            pitch_deg=float(np.degrees(debug.pitch)),
                            pitch_corr=float(debug.pitch_corr),
                            pitch_compensation_enabled=bool(debug.pitch_compensation_enabled),
                        )
                        break
                    time.sleep(1.0 / float(self.cfg["control_freq"]))
                self.logger.event("STAND_BALANCE_END")
                self._set_stage(Stage.STAND_HOLD, detail="stand-balance hold active")
            else:
                self._set_stage(Stage.STAND_HOLD, detail="holding stand pose with PD")
        except PoseInitFailed as exc:
            self.logger.event("POSE_INIT_FAILED", error=str(exc))
            try:
                self.io.damping_brake()
            except Exception:
                pass
            self._set_stage(Stage.FAULTED, detail=str(exc))
            raise

    def _run_policy_loop_wrap(self, control_dt: float, require_active_command: bool):
        from safety.runtime_guard import GuardLevel
        from safety.safety_monitor import SafetyLevel
        try:
            next_exec = time.perf_counter()
            runtime_released = not require_active_command
            release_cfg = self._policy_release_cfg()
            release_active_time = 0.0
            release_block_reason = "active command required" if require_active_command else ""

            while not self._stop_runtime.is_set() and not self._runtime_error_event.is_set():
                if getattr(self, "_debug_hang_policy", False):
                    time.sleep(0.1)
                    continue
                with self._cmd_lock:
                    web_cmd = self._cmd.copy()
                self._poll_remote()
                raw_cmd = self._remote_source.get_command() if self._remote_takeover_active and self._remote_source is not None else web_cmd
                cmd = self._filter_command(raw_cmd, control_dt)

                with self._state_lock:
                    state = self._latest_hardware_state

                if state is None:
                    next_exec += control_dt
                    slack = next_exec - time.perf_counter()
                    if slack > 0:
                        time.sleep(slack)
                    continue

                obs = self.io.get_obs_policy(state, cmd, self.runner.default_dof_pos, self.runner.last_actions)
                zero_command = self.runner._is_zero_command(cmd, state["imu_gyro"])
                
                if np.any(np.isnan(obs)) or np.any(np.isinf(obs)):
                    raise ValueError("Observation vector contains NaN or Inf values")

                if self._remote_takeover_active and self._remote_soft_estop:
                    raise RuntimeError("Remote soft estop triggered")

                if not runtime_released and zero_command:
                    raw = np.zeros(16, dtype=np.float32)
                    scaled = np.zeros(16, dtype=np.float32)
                    target_hold = self.stand_balance.compute_target(state, np.zeros(3, dtype=np.float32)) if self.stand_balance is not None and self.stand_balance.enabled else self.runner.default_dof_pos.copy()
                    tentative = target_hold.copy().astype(np.float32)
                    policy_target = self.runner.default_dof_pos.copy()
                    release_active_time = 0.0
                    release_block_reason = "zero command"
                    runtime_blend_ratio = 0.0
                else:
                    target_hold = self.stand_balance.compute_target(state, np.zeros(3, dtype=np.float32)) if self.stand_balance is not None and self.stand_balance.enabled else self.runner.default_dof_pos.copy()
                    release_metrics = self._compute_release_metrics(state, target_hold, cmd)
                    if not runtime_released:
                        release_active_time += control_dt if self.runner.is_command_active(cmd) else 0.0
                        active_ready = release_active_time >= release_cfg["command_hold_s"]
                        posture_ready = release_metrics["max_hold_err"] <= release_cfg["posture_max_err"]
                        if active_ready and posture_ready:
                            runtime_released = True
                            self.logger.event(
                                "RUNTIME_COMMAND_RELEASED",
                                cmd=cmd.tolist(),
                                active_hold_s=release_active_time,
                                max_hold_err=release_metrics["max_hold_err"],
                                max_default_err=release_metrics["max_default_err"],
                                max_hold_default_gap=release_metrics["max_hold_default_gap"],
                            )
                        else:
                            reasons = []
                            if not active_ready:
                                reasons.append(f"cmd_hold<{release_cfg['command_hold_s']:.2f}s")
                            if not posture_ready:
                                reasons.append(f"hold_err>{release_cfg['posture_max_err']:.3f}")
                            release_block_reason = ",".join(reasons)

                    if not runtime_released:
                        raw = np.zeros(16, dtype=np.float32)
                        scaled = np.zeros(16, dtype=np.float32)
                        tentative = target_hold.copy().astype(np.float32)
                        policy_target = self.runner.default_dof_pos.copy()
                        runtime_blend_ratio = 0.0
                    else:
                        scaled, raw = self.runner.step(obs, control_dt)
                        if np.any(np.isnan(raw)) or np.any(np.isinf(raw)):
                            raise ValueError("Policy action contains NaN or Inf values")
                        policy_target = (scaled + self.runner.default_dof_pos).astype(np.float32)
                        tentative = self._blend_runtime_target(
                            target_hold,
                            policy_target,
                            float(getattr(self.runner, "_command_release_alpha", 0.0)),
                            release_cfg["target_blend_s"],
                            control_dt,
                        )
                        scaled = tentative - self.runner.default_dof_pos
                        runtime_blend_ratio = min(
                            1.0,
                            float(getattr(self.runner, "_command_release_alpha", 0.0))
                            * (self.runner.command_release_s / max(release_cfg["target_blend_s"], control_dt)),
                        )

                joint_cmd = {
                    "time": time.perf_counter(),
                    "target": tentative,
                    "raw": raw,
                    "scaled": scaled,
                    "policy_target": policy_target,
                    "zero_command": zero_command,
                    "runtime_released": runtime_released,
                    "release_alpha": float(getattr(self.runner, "_command_release_alpha", 0.0)),
                    "release_active_time": release_active_time,
                    "release_block_reason": release_block_reason,
                    "runtime_blend_ratio": runtime_blend_ratio,
                    "obs": obs,
                    "cmd": cmd.copy(),
                    "raw_cmd": raw_cmd.copy(),
                }
                
                try:
                    self._target_queue.put_nowait(joint_cmd)
                except queue.Full:
                    try:
                        self._target_queue.get_nowait()
                        self._target_queue.put_nowait(joint_cmd)
                    except Exception:
                        pass

                next_exec += control_dt
                slack = next_exec - time.perf_counter()
                if slack > 0:
                    time.sleep(slack)
        except Exception as exc:
            self._policy_exception = exc
            self._policy_traceback = traceback.format_exc()
            self._runtime_error_event.set()

    def _run_motor_loop_wrap(self, motor_dt: float, policy_timeout_s: float, policy_stale_warn_s: float, log_every: int):
        from safety.runtime_guard import GuardLevel
        from safety.safety_monitor import SafetyLevel
        try:
            next_exec = time.perf_counter()
            loop_count = 0
            last_target = None
            last_target_time = 0.0
            last_stale_log_time = 0.0
            last_overrun_log_time = 0.0

            self._last_runtime_ts = time.time()
            self._runtime_loop_count = 0
            self._last_loop_dt_ms = 0.0
            self._last_loop_profile_snapshot = {}

            log_decimation = max(1, int(round(0.02 / motor_dt))) if motor_dt < 0.02 else 1

            while not self._stop_runtime.is_set() and not self._runtime_error_event.is_set():
                loop_t0 = time.perf_counter()
                loop_profile = {}

                def mark_profile(name: str) -> None:
                    nonlocal profile_last
                    now_profile = time.perf_counter()
                    loop_profile[name] = (now_profile - profile_last) * 1000.0
                    profile_last = now_profile

                profile_last = loop_t0

                state = self.io.read_state()
                mark_profile("read_state_ms")

                with self._state_lock:
                    self._latest_hardware_state = state

                try:
                    joint_cmd = self._target_queue.get_nowait()
                    last_target = joint_cmd
                    last_target_time = joint_cmd["time"]
                except queue.Empty:
                    pass

                current_time = time.perf_counter()
                if last_target is None:
                    actual_target = self.io.hold_pose(self._stand_target, kp_scale=1.0)
                    self._update_latest_target(actual_target, "runtime_zero_hold")
                    mark_profile("hold_pose_ms")

                    if loop_count * motor_dt > 2.0:
                        raise TimeoutError("Initial policy target wait timeout")

                    raw = np.zeros(16, dtype=np.float32)
                    scaled = np.zeros(16, dtype=np.float32)
                    projected_gravity = state["projected_gravity"]
                    safety_level = 0
                    guard_level = 0
                    safety_reason = ""
                    guard_reason = ""
                    zero_command = True
                    runtime_released = False
                    release_alpha = 0.0
                    release_active_time = 0.0
                    release_max_hold_err = 0.0
                    cmd = np.zeros(3, dtype=np.float32)
                    raw_cmd = np.zeros(3, dtype=np.float32)
                    extra = {
                        "raw_cmd": raw_cmd.tolist(),
                    }
                else:
                    age_s = current_time - last_target_time
                    if age_s > policy_stale_warn_s:
                        age_ms = age_s * 1000.0
                        self._note_policy_stale(age_ms)
                        if current_time - last_stale_log_time > 1.0:
                            self.logger.event("POLICY_TARGET_STALE", age_ms=age_ms, timeout_ms=policy_timeout_s * 1000.0)
                            last_stale_log_time = current_time
                    if age_s > policy_timeout_s:
                        self.logger.event("POLICY_TIMEOUT", age_ms=age_s * 1000.0)
                        raise TimeoutError(f"Policy target age {age_s*1000.0:.1f}ms exceeds safety limit {policy_timeout_s*1000.0:.1f}ms")

                    tentative = last_target["target"]
                    raw = last_target["raw"]
                    scaled = last_target["scaled"]
                    policy_target = last_target["policy_target"]
                    zero_command = last_target["zero_command"]
                    runtime_released = last_target["runtime_released"]
                    release_alpha = last_target["release_alpha"]
                    release_active_time = last_target["release_active_time"]
                    obs = last_target["obs"]
                    cmd = last_target["cmd"]
                    raw_cmd = last_target["raw_cmd"]
                    runtime_blend_ratio = last_target["runtime_blend_ratio"]

                    projected_gravity = state["projected_gravity"]
                    guard_decision = self.guard.check(
                        imu_gyro=state["imu_gyro"],
                        projected_gravity=projected_gravity,
                        imu_age_ms=float(state["imu_age_ms"]),
                        estop_triggered=self._estop,
                        extra_nan_arrays=(raw, tentative),
                    )
                    mark_profile("safety_ms")

                    if guard_decision.level == GuardLevel.STOP:
                        self.logger.event("GUARD_STOP", phase="RUNTIME", reason=guard_decision.reason)
                        raise RuntimeError(f"RuntimeGuard STOP: {guard_decision.reason}")

                    safety_decision = self.safety.check(
                        target_pose=tentative,
                        default_pose=self.runner.default_dof_pos,
                        imu_gyro=state["imu_gyro"],
                        projected_gravity=projected_gravity,
                        estop_triggered=self._estop,
                    )

                    if safety_decision.level == SafetyLevel.ESTOP:
                        self.logger.event("SAFETY_ESTOP", reason=safety_decision.message)
                        raise RuntimeError(f"SafetyMonitor ESTOP: {safety_decision.message}")
                    if safety_decision.level == SafetyLevel.BRAKE:
                        safety_diag = self._build_action_diag(
                            state=state,
                            raw=raw,
                            scaled=scaled,
                            tentative=tentative,
                            cmd=cmd,
                            zero_command=zero_command,
                            runtime_released=runtime_released,
                            release_alpha=release_alpha,
                            safety_details=safety_decision.details,
                        )
                        self.logger.event("SAFETY_BRAKE", reason=safety_decision.message, details=safety_diag)
                        raise RuntimeError(f"SafetyMonitor BRAKE: {safety_decision.message}")

                    if safety_decision.level == SafetyLevel.CLIP and safety_decision.clipped_target is not None:
                        scaled = safety_decision.clipped_target - self.runner.default_dof_pos
                        tentative = safety_decision.clipped_target

                    if runtime_released or not zero_command:
                        actual_target = self.io.send_actions(scaled, self.runner.default_dof_pos)
                        self._update_latest_target(actual_target, "runtime_policy")
                        mark_profile("send_actions_ms")
                    else:
                        actual_target = self.io.hold_pose(tentative, kp_scale=1.0)
                        self._update_latest_target(actual_target, "runtime_zero_hold")
                        mark_profile("hold_pose_ms")

                    loop_dt_ms = (time.perf_counter() - loop_t0) * 1000.0
                    self._last_runtime_ts = time.time()
                    self._runtime_loop_count += 1
                    self._last_loop_dt_ms = loop_dt_ms
                    self._last_loop_profile_snapshot = loop_profile.copy()

                    release_max_hold_err = float(np.max(np.abs(state["joint_pos"][:12] - tentative[:12])))
                    target_metrics = self._compute_target_error_metrics(state, tentative, policy_target)

                    extra = self._signal_stats(obs, raw, scaled)
                    extra.update({
                        "raw_cmd": raw_cmd.tolist(),
                        "max_raw": float(np.max(np.abs(raw))),
                        "clip_joint_indices": (safety_decision.details or {}).get("joint_indices", []),
                        "clip_joint_names": (safety_decision.details or {}).get("joint_names", []),
                        "clip_primary_joint": (safety_decision.details or {}).get("primary_joint_name"),
                        "clip_primary_target": (safety_decision.details or {}).get("primary_target"),
                        "clip_primary_measured": (safety_decision.details or {}).get("primary_measured"),
                        "clip_primary_default": (safety_decision.details or {}).get("primary_default"),
                        "clip_primary_pos_err": (safety_decision.details or {}).get("primary_pos_err"),
                        "clip_primary_raw": (safety_decision.details or {}).get("primary_raw"),
                        "clip_primary_scaled": (safety_decision.details or {}).get("primary_scaled"),
                    })

                    safety_level = int(safety_decision.level)
                    guard_level = int(guard_decision.level)
                    safety_reason = safety_decision.message
                    guard_reason = guard_decision.reason

                    if log_every and (loop_count % (log_every * log_decimation) == 0):
                        motor_diag = state.get("motor_stale", {}) or {}
                        stand_diag = self._stand_balance_snapshot()
                        self.logger.state(
                            phase="RUNTIME",
                            joint_pos=state["joint_pos"],
                            joint_vel=state["joint_vel"],
                            joint_torque=state["joint_torque"],
                            target_pose=actual_target,
                            raw_action=raw,
                            gyro=state["imu_gyro"],
                            accel=state["imu_accel"],
                            quat=state["quat_wxyz"],
                            proj_gravity=projected_gravity,
                            command=cmd,
                            imu_age_ms=float(state["imu_age_ms"]),
                            loop_dt_ms=loop_dt_ms,
                            safety_level=safety_level,
                            guard_level=guard_level,
                            holdover=int(motor_diag.get("holdover_this_frame", 0)),
                            stale_max=int(motor_diag.get("stale_max", 0)),
                            fresh_count=int(motor_diag.get("fresh_count", 16)),
                            kp_leg_cmd=float(self.io.kp_leg),
                            kd_leg_cmd=float(self.io.kd_leg),
                            kd_wheel_cmd=float(self.io.kd_wheel),
                            runtime_release_alpha=release_alpha,
                            runtime_release_hold_s=release_active_time,
                            runtime_blend_ratio=runtime_blend_ratio,
                            hold_target_max_err=target_metrics["hold_target_max_err"],
                            policy_target_max_err=target_metrics["policy_target_max_err"],
                            hold_policy_max_gap=target_metrics["hold_policy_max_gap"],
                            stand_roll_deg=float(stand_diag.get("roll_deg", 0.0)),
                            stand_pitch_deg=float(stand_diag.get("pitch_deg", 0.0)),
                            stand_roll_corr=float(stand_diag.get("roll_corr", 0.0)),
                            stand_pitch_corr=float(stand_diag.get("pitch_corr", 0.0)),
                            stand_pitch_comp_enabled=bool(stand_diag.get("pitch_compensation_enabled", False)),
                            target_source="runtime_blend" if runtime_blend_ratio < 0.999 else "runtime_policy",
                            clip_primary_joint=str((safety_decision.details or {}).get("primary_joint_name", "")),
                            clip_primary_target=float((safety_decision.details or {}).get("primary_target", 0.0) or 0.0),
                            clip_primary_measured=float((safety_decision.details or {}).get("primary_measured", 0.0) or 0.0),
                            clip_primary_default=float((safety_decision.details or {}).get("primary_default", 0.0) or 0.0),
                            clip_primary_pos_err=float((safety_decision.details or {}).get("primary_pos_err", 0.0) or 0.0),
                            clip_primary_raw=float((safety_decision.details or {}).get("primary_raw", 0.0) or 0.0),
                            clip_primary_scaled=float((safety_decision.details or {}).get("primary_scaled", 0.0) or 0.0),
                            safety_reason=safety_reason,
                            guard_reason=guard_reason,
                        )
                        mark_profile("log_ms")

                with self._state_lock:
                    self._latest_motor_diagnostics = {
                        "actual_target": actual_target,
                        "raw": raw,
                        "scaled": scaled,
                        "projected_gravity": projected_gravity,
                        "safety_level": safety_level,
                        "guard_level": guard_level,
                        "safety_reason": safety_reason,
                        "guard_reason": guard_reason,
                        "zero_command": zero_command,
                        "runtime_released": runtime_released,
                        "release_alpha": release_alpha,
                        "release_active_time": release_active_time,
                        "release_max_hold_err": release_max_hold_err,
                        "extra": extra,
                        "cmd": cmd,
                    }

                loop_profile["total_ms"] = (time.perf_counter() - loop_t0) * 1000.0
                self._record_loop_profile(loop_profile)

                next_exec += motor_dt
                slack = next_exec - time.perf_counter()
                if slack > 0:
                    coarse = slack - 0.002
                    if coarse > 0:
                        time.sleep(coarse)
                    micro_slack = next_exec - time.perf_counter()
                    if micro_slack > 0:
                        time.sleep(min(micro_slack, 0.001))
                elif slack < -motor_dt:
                    over_ms = -slack * 1000.0
                    self._note_overrun(over_ms)
                    if time.perf_counter() - last_overrun_log_time > 0.5:
                        self.logger.event("LOOP_OVERRUN", over_ms=over_ms)
                        last_overrun_log_time = time.perf_counter()
                    next_exec = time.perf_counter()

                loop_count += 1
        except Exception as exc:
            self._motor_exception = exc
            self._motor_traceback = traceback.format_exc()
            self._runtime_error_event.set()

    def _run_status_loop_wrap(self, status_dt: float):
        try:
            while not self._stop_runtime.is_set() and not self._runtime_error_event.is_set():
                start_time = time.perf_counter()

                with self._state_lock:
                    state = self._latest_hardware_state
                    diag = getattr(self, "_latest_motor_diagnostics", None)

                if state is not None and diag is not None:
                    loop_profile = getattr(self, "_last_loop_profile_snapshot", {})
                    loop_dt_ms = getattr(self, "_last_loop_dt_ms", 0.0)

                    self._set(
                        cmd=diag["cmd"].tolist(),
                        last_state=self._build_last_state(
                            state=state,
                            target=diag["actual_target"],
                            raw=diag["raw"],
                            projected_gravity=diag["projected_gravity"],
                            cmd=diag["cmd"],
                            loop_dt_ms=loop_dt_ms,
                            phase="RUNTIME",
                            safety_level=diag["safety_level"],
                            guard_level=diag["guard_level"],
                            safety_reason=diag["safety_reason"],
                            guard_reason=diag["guard_reason"],
                            zero_command=diag["zero_command"],
                            runtime_released=diag["runtime_released"],
                            release_alpha=diag["release_alpha"],
                            release_active_hold_s=diag["release_active_time"],
                            release_max_hold_err=diag["release_max_hold_err"],
                            loop_profile=loop_profile,
                            extra=diag["extra"],
                        )
                    )

                elapsed = time.perf_counter() - start_time
                sleep_time = max(0.0, status_dt - elapsed)
                time.sleep(sleep_time)
        except Exception as exc:
            self._status_exception = exc
            self._status_traceback = traceback.format_exc()
            self._runtime_error_event.set()

    def runtime_start(self, policy_path: Optional[str] = None):
        return self._run_async(self._do_runtime_start, policy_path)

    def _do_runtime_start(self, policy_path: Optional[str]):
        from policy.policy_runner import PolicyRunner, resolve_policy_path
        from safety.runtime_guard import GuardLevel
        from safety.safety_monitor import SafetyLevel

        if self.status.stage != Stage.STAND_HOLD.value:
            raise RuntimeError("runtime start requires STAND_HOLD")

        sim2real_root = Path(__file__).resolve().parents[1]
        resolved_policy = resolve_policy_path(policy_path, sim2real_root)
        if not resolved_policy.exists():
            raise RuntimeError(f"policy not found: {resolved_policy}")

        self.runner = PolicyRunner(
            resolved_policy,
            enable_zero_cmd_suppression=self.cfg.get("policy", {}).get("enable_zero_cmd_suppression", True),
            hold_zero_command_pose=self.cfg.get("policy", {}).get("hold_zero_command_pose", True),
            command_release_s=self.cfg.get("policy", {}).get("command_release_s", 0.35),
            action_scale=self._policy_action_scale(),
            zero_cmd_use_yaw_rate=self.cfg.get("policy", {}).get("zero_cmd_use_yaw_rate", False),
            clip_obs=self.cfg.get("policy", {}).get("clip_obs", 100.0),
        )
        self.safety.reset()
        require_active_command = self.cfg.get("policy", {}).get("require_active_command_to_release", True)
        self.logger.event("POLICY_LOADED", path=str(resolved_policy))
        self._stop_state_poll()

        control_dt = 1.0 / float(self.cfg.get("policy_freq", 50))
        target = self._stand_target

        self.logger.event("PRIME_BEGIN")
        zero_cmd = np.zeros(3, dtype=np.float32)
        for index in range(1):
            if self.stand_balance is not None and self.stand_balance.enabled:
                state = self.io.read_state()
                target = self.stand_balance.compute_target(state, zero_cmd)
                self.io.hold_pose(target, kp_scale=1.0)
            else:
                self.io.hold_pose(target, kp_scale=1.0)
                state = self.io.read_state()
            obs = self.io.get_obs_policy(state, zero_cmd, self.runner.default_dof_pos, self.runner.last_actions)
            if index == 0:
                self.runner.reset(prime_obs=obs)
            time.sleep(control_dt)
        self.logger.event("PRIME_END")

        state = self.io.read_state()
        projected_gravity = state["projected_gravity"]
        if abs(projected_gravity[0]) > 0.5 or abs(projected_gravity[1]) > 0.5:
            error_message = (
                f"IMU frame mismatch or body tilt too large: "
                f"gravity projection X={projected_gravity[0]:.2f}, Y={projected_gravity[1]:.2f}"
            )
            self.logger.event("GUARD_STOP", phase="STARTUP", reason=error_message)
            self._set_stage(Stage.FAULTED, error_message)
            return

        self.logger.event("HISTORY_PRIMED", initial_obs=obs)
        self._stop_runtime.clear()
        self._runtime_loop_count = 0
        self._runtime_overrun_count = 0
        self._runtime_overrun_max_ms = 0.0
        self._runtime_policy_stale_count = 0
        self._runtime_policy_stale_max_ms = 0.0
        self._filtered_cmd[:] = 0.0
        self._last_raw_cmd[:] = 0.0
        self._latest_target = None
        self._latest_target_info = {}
        self._remote_takeover_active = False
        self._remote_soft_estop = False
        self._set_stage(Stage.RUNTIME, detail="Decoupled policy & motor loop active")
        self.logger.event("RUNTIME_BEGIN")

        # 启动解耦的多 Loop
        # 1. 清空解耦队列与状态缓存
        while not self._target_queue.empty():
            try:
                self._target_queue.get_nowait()
            except Exception:
                break
        
        self._latest_hardware_state = state
        self._policy_exception = None
        self._motor_exception = None
        self._status_exception = None
        self._runtime_error_event.clear()

        # 2. 读取配置频率
        motor_dt = 1.0 / float(self.cfg.get("motor_freq", 200))
        status_dt = 1.0 / float(self.cfg.get("status_freq", 10))
        policy_timeout_s = float(self.cfg.get("policy_timeout_ms", 150.0)) / 1000.0
        policy_stale_warn_s = float(self.cfg.get("policy_stale_warn_ms", 60.0)) / 1000.0
        log_every = int(self.cfg.get("log_every", 1))

        # 3. 创建并启动线程
        policy_thread = threading.Thread(
            target=self._run_policy_loop_wrap,
            args=(control_dt, require_active_command),
            name="PolicyLoop",
            daemon=True
        )
        motor_thread = threading.Thread(
            target=self._run_motor_loop_wrap,
            args=(motor_dt, policy_timeout_s, policy_stale_warn_s, log_every),
            name="MotorLoop",
            daemon=True
        )
        status_thread = threading.Thread(
            target=self._run_status_loop_wrap,
            args=(status_dt,),
            name="StatusLoop",
            daemon=True
        )

        policy_thread.start()
        motor_thread.start()
        status_thread.start()

        # 4. 主工作监控忙等，直到用户触发 stop 或是线程抛出异常
        try:
            while not self._stop_runtime.is_set() and not self._runtime_error_event.is_set():
                time.sleep(0.05)
                
            if self._runtime_error_event.is_set():
                for exc, tb, name in [
                    (self._motor_exception, getattr(self, "_motor_traceback", ""), "MotorLoop"),
                    (self._policy_exception, getattr(self, "_policy_traceback", ""), "PolicyLoop"),
                    (self._status_exception, getattr(self, "_status_traceback", ""), "StatusLoop"),
                ]:
                    if exc is not None:
                        raise exc
        finally:
            # 5. 确保通知所有工作线程退出并等待它们
            self._stop_runtime.set()
            policy_thread.join(timeout=1.0)
            motor_thread.join(timeout=1.0)
            status_thread.join(timeout=1.0)

        if self.status.stage == Stage.RUNTIME.value:
            self.logger.event("RUNTIME_STOP")
            self._remote_takeover_active = False
            self._remote_soft_estop = False
            self._filtered_cmd[:] = 0.0
            self._last_raw_cmd[:] = 0.0
            self._latest_target = None
            self._latest_target_info = {}
            self._set_stage(Stage.STAND_HOLD, detail="runtime stopped, back to stand-balance hold")
            self._start_state_poll()

    def runtime_stop(self):
        self._stop_runtime.set()
        return True

    def estop(self):
        self._estop = True
        try:
            if self.io:
                self.io.damping_brake()
        except Exception:
            pass
        self._stop_runtime.set()
        if self.logger:
            self.logger.event("USER_ESTOP_WEB")
        self._set_stage(Stage.ESTOPPED, detail="web emergency stop")
        return True

    def reset_estop(self):
        self._estop = False
        self._set(detail="estop cleared")
        return True

    def set_remote_takeover(self, enabled: bool):
        enabled = bool(enabled)
        if enabled:
            if self.status.stage != Stage.RUNTIME.value:
                raise RuntimeError("remote takeover requires RUNTIME")
            if self._remote_source is None:
                raise RuntimeError("remote controller not configured")
            self._poll_remote()
            self._remote_takeover_active = True
            self._filtered_cmd[:] = 0.0
            self._last_raw_cmd[:] = 0.0
            if self.logger:
                self.logger.event("REMOTE_TAKEOVER_ENABLED", port=self._remote_source.port)
            self._set(detail="remote takeover enabled")
            return True

        self._remote_takeover_active = False
        self._remote_soft_estop = False
        self._filtered_cmd[:] = 0.0
        self._last_raw_cmd[:] = 0.0
        if self.logger:
            self.logger.event("REMOTE_TAKEOVER_DISABLED")
        self._set(detail="remote takeover disabled")
        return True

    def set_command(self, vx: float, vy: float, yaw: float):
        with self._cmd_lock:
            self._cmd[0] = float(vx)
            self._cmd[1] = float(vy)
            self._cmd[2] = float(yaw)
        self._last_command_ts = time.time()
        self._set(cmd=self._cmd.tolist())
        return True

    def test_motor(self, leg: str, joint: str, delta_rad: float, kp: float, kd: float, duration_s: float):
        return self._run_async(self._do_test_motor, leg, joint, delta_rad, kp, kd, duration_s)

    def _do_test_motor(self, leg: str, joint: str, delta_rad: float, kp: float, kd: float, duration_s: float):
        if self.io is None:
            raise RuntimeError("hardware not connected")
        if self.status.stage not in (Stage.ENABLED.value, Stage.STAND_HOLD.value, Stage.FAULTED.value):
            raise RuntimeError("test motor requires ENABLED/STAND_HOLD/FAULTED")

        joint_key = (str(leg), str(joint))
        if joint_key not in self.io.hw.mapper.SIM_INDEX_MAP:
            raise RuntimeError(f"unknown joint: {leg}_{joint}")

        idx = self.io.hw.mapper.SIM_INDEX_MAP[joint_key]
        base_pose = self.io.read_measured_pose().astype(np.float32)
        target_pose = base_pose.copy()
        target_pose[idx] += float(delta_rad)

        prev_stage = self.status.stage
        self._set_stage(Stage.JOINT_TEST, detail=f"testing {leg}_{joint}")
        if self.logger:
            self.logger.event(
                "JOINT_TEST_BEGIN",
                joint=f"{leg}_{joint}",
                joint_index=idx,
                delta_rad=float(delta_rad),
                kp=float(kp),
                kd=float(kd),
                duration_s=float(duration_s),
                start_pos=float(base_pose[idx]),
                target_pos=float(target_pose[idx]),
            )

        self._stop_state_poll()
        next_exec = time.perf_counter()
        deadline = next_exec + max(float(duration_s), 0.1)
        samples = []
        while time.perf_counter() < deadline:
            state = self.io.read_state()
            measured = float(state["joint_pos"][idx])
            error = float(target_pose[idx] - measured)
            torque = float(state["joint_torque"][idx])
            vel = float(state["joint_vel"][idx])
            samples.append((measured, error, vel, torque))
            self.io.hw.send_control(target_pose, float(kp), float(kd), self.io.kd_wheel)
            next_exec += 1.0 / float(self.cfg["control_freq"])
            slack = next_exec - time.perf_counter()
            if slack > 0:
                time.sleep(slack)

        self.io.hold_pose(base_pose, kp_scale=1.0)
        final_state = self.io.read_state()
        final_measured = float(final_state["joint_pos"][idx])
        if self.logger:
            self.logger.event(
                "JOINT_TEST_END",
                joint=f"{leg}_{joint}",
                joint_index=idx,
                final_pos=final_measured,
                final_err=float(target_pose[idx] - final_measured),
                max_abs_err=float(max(abs(s[1]) for s in samples) if samples else 0.0),
                max_abs_vel=float(max(abs(s[2]) for s in samples) if samples else 0.0),
                max_abs_tau=float(max(abs(s[3]) for s in samples) if samples else 0.0),
            )
        self._set(stage=prev_stage, detail=f"joint test {leg}_{joint} done")
        if prev_stage in (Stage.ENABLED.value, Stage.STAND_HOLD.value):
            self._start_state_poll()

    def list_logs(self):
        log_root = Path(self.cfg.get("log_dir", "logs"))
        if not log_root.exists():
            return []
        out = []
        for directory in sorted(log_root.iterdir(), reverse=True):
            if not directory.is_dir():
                continue
            state_path = directory / "state.csv"
            events_path = directory / "events.jsonl"
            out.append(
                {
                    "id": directory.name,
                    "state_csv": state_path.exists(),
                    "events_jsonl": events_path.exists(),
                    "size_kb": (
                        (state_path.stat().st_size + events_path.stat().st_size) // 1024
                        if state_path.exists() and events_path.exists()
                        else 0
                    ),
                }
            )
        return out
