"""Sim-to-real control/IK API extracted from mature mujoco_sim controller.

This module provides a deployment-friendly wrapper around:
- wheel mode posture control
- trot swing-leg IK + wheel assist
- differential wheel speed mapping

No MuJoCo runtime is required for using the API itself.
For trot IK, Pinocchio model is used via Dynamics.
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple
import numpy as np

from config import (
    LEG_NAMES,
    LEG_JOINTS,
    WHEEL_JOINT,
    DEFAULT_JOINT_ANGLES,
    WHEEL_RADIUS,
    WHEEL_TRACK,
    WHEEL_VEL_MAX,
    KP_ROLL,
    KP_PITCH,
    GAIT_FREQ,
    GAIT_DUTY,
    SWING_HEIGHT,
    PHASE_OFFSETS,
)
from dynamics import Dynamics


@dataclass
class DeployState:
    """Minimal state for deployment control."""

    rpy: np.ndarray  # (3,) roll, pitch, yaw


class Sim2RealControlAPI:
    """Deployment-friendly control and IK API.

    Supported modes:
    - wheel: wheel differential drive + leg posture hold
    - trot: swing foot IK + stance posture + wheel assist
    """

    def __init__(self):
        self.mode = "wheel"
        self.prone = False

        self.vel_x = 0.0
        self.vel_y = 0.0
        self.yaw_rate = 0.0
        self.height = 0.33

        self._gait_phase = 0.0
        self._smooth_vx = 0.0
        self._smooth_vy = 0.0
        self._smooth_yaw = 0.0

        self._default_q = np.array([
            DEFAULT_JOINT_ANGLES["hip_abduction"],
            DEFAULT_JOINT_ANGLES["hip_pitch"],
            DEFAULT_JOINT_ANGLES["knee"],
        ])

        self.dynamics = Dynamics()
        self._swing_start_foot = {leg: np.zeros(3) for leg in LEG_NAMES}
        self._last_contact = {leg: True for leg in LEG_NAMES}

    def set_mode(self, mode: str):
        if mode not in ("wheel", "trot"):
            raise ValueError("mode must be one of: wheel, trot")
        self.mode = mode

    def set_command(self, vel_x: float, vel_y: float, yaw_rate: float, height: Optional[float] = None):
        self.vel_x = float(vel_x)
        self.vel_y = float(vel_y)
        self.yaw_rate = float(yaw_rate)
        if height is not None:
            self.height = float(height)

    def compute(
        self,
        state: DeployState,
        dt: float,
        q_pin: Optional[np.ndarray] = None,
        dq_pin: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute leg and wheel commands.

        Returns:
            leg_targets: (12,) [fl(3), fr(3), rl(3), rr(3)]
            wheel_targets: (4,) [fl, fr, rl, rr] in rad/s

        Notes:
            - wheel mode does not require q_pin/dq_pin
            - trot mode requires q_pin/dq_pin for IK/FK through Pinocchio
        """
        alpha = min(float(dt) * 3.0, 1.0)
        self._smooth_vx += alpha * (self.vel_x - self._smooth_vx)
        self._smooth_vy += alpha * (self.vel_y - self._smooth_vy)
        self._smooth_yaw += alpha * (self.yaw_rate - self._smooth_yaw)

        if self.prone:
            return self._prone_mode()

        if self.mode == "wheel":
            return self._wheel_mode(state)

        if q_pin is None or dq_pin is None:
            raise ValueError("trot mode requires q_pin and dq_pin")
        return self._trot_mode(state, float(dt), q_pin, dq_pin)

    def to_joint_dict(self, leg_targets: np.ndarray, wheel_targets: np.ndarray) -> Dict[str, float]:
        """Convert array commands to named joint-command dictionary."""
        out: Dict[str, float] = {}
        for i, leg in enumerate(LEG_NAMES):
            out[f"{leg}_{LEG_JOINTS[0]}"] = float(leg_targets[i * 3 + 0])
            out[f"{leg}_{LEG_JOINTS[1]}"] = float(leg_targets[i * 3 + 1])
            out[f"{leg}_{LEG_JOINTS[2]}"] = float(leg_targets[i * 3 + 2])
            out[f"{leg}_{WHEEL_JOINT}"] = float(wheel_targets[i])
        return out

    def _prone_mode(self):
        leg_targets = np.zeros(12)
        for i, leg in enumerate(LEG_NAMES):
            side = 1.0 if leg[1] == "l" else -1.0
            leg_targets[i * 3 + 0] = side * 0.3
            leg_targets[i * 3 + 1] = 1.5
            leg_targets[i * 3 + 2] = -2.65
        return leg_targets, np.zeros(4)

    def _wheel_mode(self, state: DeployState):
        wheel_targets = self._differential_drive(self._smooth_vx, self._smooth_yaw)
        leg_targets = self._posture_control(state)
        return leg_targets, wheel_targets

    def _posture_control(self, state: DeployState) -> np.ndarray:
        leg_targets = np.zeros(12)

        _H = [0.157, 0.248, 0.311, 0.366, 0.411, 0.448]
        _HIP = [1.5, 1.2, 1.0, 0.8, 0.6, 0.4]
        _KNEE = [-2.5, -2.1, -1.8, -1.5, -1.2, -0.9]

        h_clamp = np.clip(self.height, _H[0], _H[-1])
        q_hip_base = float(np.interp(h_clamp, _H, _HIP))
        q_knee_base = float(np.interp(h_clamp, _H, _KNEE))

        roll_corr = -KP_ROLL * float(state.rpy[0])
        pitch_corr = -KP_PITCH * float(state.rpy[1])
        lateral_lean = 0.3 * self.vel_y

        for i, leg in enumerate(LEG_NAMES):
            side = 1.0 if leg[1] == "l" else -1.0
            leg_targets[i * 3 + 0] = np.clip(side * roll_corr + lateral_lean, -0.5, 0.5)
            leg_targets[i * 3 + 1] = np.clip(q_hip_base + pitch_corr, -1.0, 2.5)
            leg_targets[i * 3 + 2] = np.clip(q_knee_base, -2.6, -0.3)

        return leg_targets

    def _trot_mode(self, state: DeployState, dt: float, q_pin: np.ndarray, dq_pin: np.ndarray):
        self._gait_phase = (self._gait_phase + dt * GAIT_FREQ) % 1.0

        contacts: Dict[str, bool] = {}
        for leg in LEG_NAMES:
            phase = (self._gait_phase + PHASE_OFFSETS[leg]) % 1.0
            contacts[leg] = bool(phase < GAIT_DUTY)

        self.dynamics.update(q_pin, dq_pin)

        leg_targets = np.zeros(12)
        wheel_targets = np.zeros(4)

        for i, leg in enumerate(LEG_NAMES):
            if contacts[leg]:
                leg_targets[i * 3:(i + 1) * 3] = self._stance_leg_target(state, leg)
                self._swing_start_foot[leg] = self.dynamics.get_foot_pos(leg)
                self._last_contact[leg] = True
                wheel_targets[i] = self._differential_drive_single(self._smooth_vx, self._smooth_yaw, leg)
            else:
                swing_phase = self._get_swing_phase(leg)
                target_foot = self._compute_swing_target(leg, state, swing_phase)
                q_ik = self.dynamics.inverse_kinematics(leg, target_foot, q_pin)
                leg_targets[i * 3:(i + 1) * 3] = q_ik
                self._last_contact[leg] = False
                wheel_targets[i] = 0.0

        return leg_targets, wheel_targets

    def _stance_leg_target(self, state: DeployState, leg: str) -> np.ndarray:
        _H = [0.157, 0.248, 0.311, 0.366, 0.411, 0.448]
        _HIP = [1.5, 1.2, 1.0, 0.8, 0.6, 0.4]
        _KNEE = [-2.5, -2.1, -1.8, -1.5, -1.2, -0.9]

        h_clamp = np.clip(self.height, _H[0], _H[-1])
        q_hip = float(np.interp(h_clamp, _H, _HIP))
        q_knee = float(np.interp(h_clamp, _H, _KNEE))

        roll_corr = -KP_ROLL * float(state.rpy[0])
        pitch_corr = -KP_PITCH * float(state.rpy[1])
        side = 1.0 if leg[1] == "l" else -1.0
        lateral_lean = 0.3 * self.vel_y

        return np.array([
            np.clip(side * roll_corr + lateral_lean, -0.5, 0.5),
            np.clip(q_hip + pitch_corr, -1.0, 2.5),
            np.clip(q_knee, -2.6, -0.3),
        ])

    def _differential_drive(self, vel_x: float, yaw_rate: float) -> np.ndarray:
        vel_left = (vel_x - 0.5 * WHEEL_TRACK * yaw_rate) / WHEEL_RADIUS
        vel_right = (vel_x + 0.5 * WHEEL_TRACK * yaw_rate) / WHEEL_RADIUS
        targets = np.zeros(4)
        for i, leg in enumerate(LEG_NAMES):
            targets[i] = vel_left if leg[1] == "l" else vel_right
        return np.clip(targets, -WHEEL_VEL_MAX, WHEEL_VEL_MAX)

    def _differential_drive_single(self, vel_x: float, yaw_rate: float, leg: str) -> float:
        if leg[1] == "l":
            v = (vel_x - 0.5 * WHEEL_TRACK * yaw_rate) / WHEEL_RADIUS
        else:
            v = (vel_x + 0.5 * WHEEL_TRACK * yaw_rate) / WHEEL_RADIUS
        return float(np.clip(v, -WHEEL_VEL_MAX, WHEEL_VEL_MAX))

    def _get_swing_phase(self, leg: str) -> float:
        phase = (self._gait_phase + PHASE_OFFSETS[leg]) % 1.0
        if phase < GAIT_DUTY:
            return 0.0
        return (phase - GAIT_DUTY) / (1.0 - GAIT_DUTY)

    def _compute_swing_target(self, leg: str, state: DeployState, swing_phase: float) -> np.ndarray:
        p_start = self._swing_start_foot[leg]
        p_end = self._compute_touchdown(leg, state)

        s = swing_phase
        s_mj = 10 * s**3 - 15 * s**4 + 6 * s**5

        pos = p_start + (p_end - p_start) * s_mj

        z_lift = 64.0 * s**3 * (1.0 - s)**3
        pos[2] = p_start[2] + SWING_HEIGHT * z_lift

        return pos

    def _compute_touchdown(self, leg: str, state: DeployState) -> np.ndarray:
        td = self._swing_start_foot[leg].copy()

        t_stance = (1.0 / GAIT_FREQ) * GAIT_DUTY
        yaw = float(state.rpy[2])
        c, s = np.cos(yaw), np.sin(yaw)
        R_z = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        cmd_vel_world = R_z @ np.array([self._smooth_vx, self._smooth_vy, 0.0])

        td[0] += cmd_vel_world[0] * t_stance * 0.5
        td[1] += cmd_vel_world[1] * t_stance * 0.5
        td[2] = WHEEL_RADIUS
        return td


if __name__ == "__main__":
    api = Sim2RealControlAPI()
    api.set_mode("wheel")
    api.set_command(vel_x=0.3, vel_y=0.0, yaw_rate=0.0, height=0.33)

    state = DeployState(rpy=np.array([0.0, 0.0, 0.0]))
    leg, wheel = api.compute(state=state, dt=0.004)
    cmd = api.to_joint_dict(leg, wheel)

    print("Example wheel-mode command:")
    for k, v in sorted(cmd.items()):
        print(f"{k}: {v:.6f}")
