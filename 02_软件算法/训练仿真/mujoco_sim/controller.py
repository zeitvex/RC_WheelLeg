"""Main controller: wheel mode + trot mode for wheeled-legged robot.

Wheel mode: differential drive + leg posture hold (height/roll/pitch compensation)
Trot mode: quadruped gait with wheel-assisted propulsion

Actuator interface:
  - Leg joints: ctrl = target angle (PD: kp=60, kd=3)
  - Wheel joints: ctrl = target velocity in rad/s (gain=2.0)
"""

import numpy as np
from robot import Robot, RobotState
from dynamics import Dynamics
from mpc_controller import MPCController
from config import (
    LEG_NAMES, DEFAULT_JOINT_ANGLES, WHEEL_RADIUS, WHEEL_TRACK,
    WHEEL_VEL_MAX, KP_ROLL, KP_PITCH, HEIGHT_TABLE,
    GAIT_FREQ, GAIT_DUTY, SWING_HEIGHT, PHASE_OFFSETS,
)

RL_ROUGH_Q = np.array([0.0, 0.550, -1.125], dtype=float)
LEG_STATE_IDX = np.array([0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14], dtype=int)


class Controller:
    """Wheeled-legged robot controller."""

    def __init__(self, robot: Robot):
        self.robot = robot
        self.dynamics = Dynamics()

        # User commands
        self.vel_x = 0.0       # m/s forward
        self.vel_y = 0.0       # m/s lateral
        self.yaw_rate = 0.0    # rad/s
        self.height = 0.37     # m desired body height (wheel center under hip)
        self.wheel_posture = "table"  # "table" follows height slider; "rl" matches src/robot default

        # Mode: "wheel", "trot", or "mpc"
        self.mode = "wheel"

        # Prone (lie down) state
        self.prone = False

        # MPC controller
        self._mpc_ctrl = MPCController(robot)
        self._mpc_active = False  # track torque mode state

        # Gait state
        self._gait_phase = 0.0

        # Smoothed commands for trot mode (avoid sudden jumps)
        self._smooth_vx = 0.0
        self._smooth_vy = 0.0
        self._smooth_yaw = 0.0
        self._last_leg_targets = np.tile(
            np.array(
                [
                    DEFAULT_JOINT_ANGLES["hip_abduction"],
                    DEFAULT_JOINT_ANGLES["hip_pitch"],
                    DEFAULT_JOINT_ANGLES["knee"],
                ],
                dtype=float,
            ),
            4,
        )

        # Wheel-mode sensor feedback.
        self.yaw_rate_kp = 0.45
        self.roll_comp_gain = KP_ROLL
        self.pitch_comp_gain = KP_PITCH
        self.encoder_posture_kp = 0.12
        self.encoder_posture_max = 0.025
        self.encoder_guard_start = 0.28
        self.encoder_guard_stop = 0.65
        self.imu_guard_start = np.deg2rad(12.0)
        self.imu_guard_stop = np.deg2rad(28.0)
        self.yaw_wheel_gain = 1.0
        self.max_yaw_wheel_speed = 4.0

        # Default leg angles
        self._default_q = np.array([
            DEFAULT_JOINT_ANGLES["hip_abduction"],
            DEFAULT_JOINT_ANGLES["hip_pitch"],
            DEFAULT_JOINT_ANGLES["knee"],
        ])

        # Swing leg memory
        self._swing_start_foot = {leg: np.zeros(3) for leg in LEG_NAMES}
        self._last_contact = {leg: True for leg in LEG_NAMES}

    def compute(self, state: RobotState, dt: float) -> tuple[np.ndarray, np.ndarray]:
        # Smooth all velocity commands (both modes)
        alpha = min(dt * 3.0, 1.0)  # ~0.33s time constant
        self._smooth_vx += alpha * (self.vel_x - self._smooth_vx)
        self._smooth_vy += alpha * (self.vel_y - self._smooth_vy)
        self._smooth_yaw += alpha * (self.yaw_rate - self._smooth_yaw)

        if self.prone:
            self._ensure_position_mode()
            return self._prone_mode()
        if self.mode == "mpc":
            return self._mpc_mode(state, dt)
        if self.mode == "wheel":
            self._ensure_position_mode()
            return self._wheel_mode(state, dt)
        else:
            self._ensure_position_mode()
            return self._trot_mode(state, dt)

    def _mpc_mode(self, state: RobotState, dt: float):
        """MPC locomotion: MIT motor protocol (PD + MPC feedforward torque)."""
        # Switch to torque mode if not already
        if not self._mpc_active:
            self.robot.enable_torque_mode()
            self._mpc_active = True

        # Sync commands to MPC controller
        self._mpc_ctrl.vel_x = self.vel_x
        self._mpc_ctrl.vel_y = self.vel_y
        self._mpc_ctrl.yaw_rate = self.yaw_rate
        self._mpc_ctrl.height = self.height

        # Compute and apply (sets ctrl directly via set_ctrl_mit)
        self._mpc_ctrl.compute(state, dt)
        # Return dummy - ctrl already set
        return np.zeros(12), np.zeros(4)

    def _ensure_position_mode(self):
        """Switch back to position PD mode if coming from MPC."""
        if self._mpc_active:
            self.robot.enable_position_mode()
            self._mpc_active = False

    def _prone_mode(self):
        """Lie down: actual prone pose from real robot."""
        leg_targets = np.zeros(12)
        for i, leg in enumerate(LEG_NAMES):
            side = 1.0 if leg[1] == "l" else -1.0
            leg_targets[i*3] = side * 0.3     # fl/rl: +0.3, fr/rr: -0.3
            leg_targets[i*3+1] = 1.5          # hip pitch
            leg_targets[i*3+2] = -2.65        # knee hard limit from MJCF
        return leg_targets, np.zeros(4)

    # ─────────────────────────────────────────────────────────────────────
    # WHEEL MODE
    # ─────────────────────────────────────────────────────────────────────

    def _wheel_mode(self, state: RobotState, dt: float):
        """Wheel drive + leg posture hold.

        vel_y: limited effect in wheel mode (differential drive cannot produce
        pure lateral motion). Uses hip_abduction lean for small lateral force.
        For significant lateral motion, use trot mode.
        """
        leg_targets = self._posture_control(state)
        safe_vx, safe_yaw = self._wheel_velocity_envelope(self._smooth_vx, self._smooth_yaw)
        yaw_feedback = safe_yaw + self.yaw_rate_kp * (safe_yaw - float(state.ang_vel[2]))
        wheel_targets = self._differential_drive(safe_vx, yaw_feedback)
        wheel_targets *= self._sensor_command_scale(state, leg_targets)
        self._last_leg_targets = leg_targets.copy()
        return leg_targets, wheel_targets

    def _posture_control(self, state: RobotState) -> np.ndarray:
        """Leg joint targets from the soft wheel-X height table."""
        leg_targets = np.zeros(12)

        # Calibrated height→angle lookup (minimizes motor torque at each height)
        _H  = [r[0] for r in HEIGHT_TABLE]
        _HIP = [r[1] for r in HEIGHT_TABLE]
        _KNEE = [r[2] for r in HEIGHT_TABLE]

        h_clamp = np.clip(self.height, _H[0], _H[-1])
        q_hip_base = float(np.interp(h_clamp, _H, _HIP))
        q_knee_base = float(np.interp(h_clamp, _H, _KNEE))
        q_ab_base = 0.0
        if self.wheel_posture == "rl":
            q_ab_base, q_hip_base, q_knee_base = RL_ROUGH_Q

        roll_corr = -self.roll_comp_gain * state.rpy[0]
        pitch_corr = -self.pitch_comp_gain * state.rpy[1]
        lateral_lean = 0.3 * self.vel_y

        for i, leg in enumerate(LEG_NAMES):
            side = 1.0 if leg[1] == "l" else -1.0
            leg_targets[i*3] = np.clip(q_ab_base + side * roll_corr + lateral_lean, -0.5, 0.5)
            leg_targets[i*3+1] = np.clip(q_hip_base + pitch_corr, -1.0, 2.5)
            leg_targets[i*3+2] = np.clip(q_knee_base, -2.65, -0.3)

        encoder_err = self._last_leg_targets - state.joint_pos[LEG_STATE_IDX]
        leg_targets += np.clip(
            self.encoder_posture_kp * encoder_err,
            -self.encoder_posture_max,
            self.encoder_posture_max,
        )
        leg_targets[0::3] = np.clip(leg_targets[0::3], -0.5, 0.5)
        leg_targets[1::3] = np.clip(leg_targets[1::3], -1.0, 2.5)
        leg_targets[2::3] = np.clip(leg_targets[2::3], -2.65, -0.3)
        return leg_targets

    def _wheel_velocity_envelope(self, vel_x: float, yaw_rate: float) -> tuple[float, float]:
        """Limit x/yaw combinations that are unsafe for the RL posture."""
        ax = abs(vel_x)
        if ax >= 0.8:
            yaw_lim = 0.35
        elif ax >= 0.5:
            yaw_lim = 0.55
        elif ax >= 0.25:
            yaw_lim = 0.75
        else:
            yaw_lim = 1.0
        return float(vel_x), float(np.clip(yaw_rate, -yaw_lim, yaw_lim))

    def _sensor_command_scale(self, state: RobotState, leg_targets: np.ndarray) -> float:
        """Back off wheels when IMU or encoder feedback says posture is degrading."""
        leg_error = float(np.max(np.abs(state.joint_pos[LEG_STATE_IDX] - leg_targets)))
        tilt = float(np.hypot(state.rpy[0], state.rpy[1]))
        scale = 1.0

        if leg_error >= self.encoder_guard_stop:
            scale = 0.0
        elif leg_error > self.encoder_guard_start:
            span = max(1e-6, self.encoder_guard_stop - self.encoder_guard_start)
            scale *= 1.0 - (leg_error - self.encoder_guard_start) / span

        if tilt >= self.imu_guard_stop:
            scale = 0.0
        elif tilt > self.imu_guard_start:
            span = max(1e-6, self.imu_guard_stop - self.imu_guard_start)
            scale *= 1.0 - (tilt - self.imu_guard_start) / span

        return float(np.clip(scale, 0.0, 1.0))

    # ─────────────────────────────────────────────────────────────────────
    # TROT MODE
    # ─────────────────────────────────────────────────────────────────────

    def _trot_mode(self, state: RobotState, dt: float):
        """Trot gait with wheel assist."""
        # Advance gait phase
        self._gait_phase = (self._gait_phase + dt * GAIT_FREQ) % 1.0

        # Contact state
        contacts = {}
        for leg in LEG_NAMES:
            phase = (self._gait_phase + PHASE_OFFSETS[leg]) % 1.0
            contacts[leg] = phase < GAIT_DUTY

        # Pinocchio update
        q_pin, dq_pin = self.robot.get_qpos_qvel_for_pinocchio()
        self.dynamics.update(q_pin, dq_pin)

        leg_targets = np.zeros(12)
        wheel_targets = np.zeros(4)

        for i, leg in enumerate(LEG_NAMES):
            if contacts[leg]:
                # Stance: posture hold
                leg_targets[i*3:(i+1)*3] = self._stance_leg_target(state, leg)
                self._swing_start_foot[leg] = self.dynamics.get_foot_pos(leg)
                self._last_contact[leg] = True
                # Wheel: drive with smoothed velocity
                wheel_targets[i] = self._differential_drive_single(
                    self._smooth_vx, self._smooth_yaw, leg)
            else:
                # Swing: IK trajectory
                swing_phase = self._get_swing_phase(leg)
                target_foot = self._compute_swing_target(leg, state, swing_phase)
                q_ik = self.dynamics.inverse_kinematics(leg, target_foot, q_pin)
                leg_targets[i*3:(i+1)*3] = q_ik
                self._last_contact[leg] = False
                # Wheel: zero (free during swing)
                wheel_targets[i] = 0.0

        return leg_targets, wheel_targets

    def _stance_leg_target(self, state: RobotState, leg: str) -> np.ndarray:
        """Stance leg: table-interpolated height + attitude compensation."""
        _H  = [r[0] for r in HEIGHT_TABLE]
        _HIP = [r[1] for r in HEIGHT_TABLE]
        _KNEE = [r[2] for r in HEIGHT_TABLE]

        h_clamp = np.clip(self.height, _H[0], _H[-1])
        q_hip = float(np.interp(h_clamp, _H, _HIP))
        q_knee = float(np.interp(h_clamp, _H, _KNEE))

        roll_corr = -KP_ROLL * state.rpy[0]
        pitch_corr = -KP_PITCH * state.rpy[1]
        side = 1.0 if leg[1] == "l" else -1.0
        lateral_lean = 0.3 * self.vel_y

        return np.array([
            np.clip(side * roll_corr + lateral_lean, -0.5, 0.5),
            np.clip(q_hip + pitch_corr, -1.0, 2.5),
            np.clip(q_knee, -2.65, -0.3),
        ])

    # ─────────────────────────────────────────────────────────────────────
    # DIFFERENTIAL DRIVE
    # ─────────────────────────────────────────────────────────────────────

    def _differential_drive(self, vel_x: float, yaw_rate: float) -> np.ndarray:
        """4 wheel velocities from body commands."""
        linear_wheel = vel_x / WHEEL_RADIUS
        yaw_wheel = self.yaw_wheel_gain * 0.5 * WHEEL_TRACK * yaw_rate / WHEEL_RADIUS
        yaw_wheel = float(np.clip(yaw_wheel, -self.max_yaw_wheel_speed, self.max_yaw_wheel_speed))
        vel_left = linear_wheel - yaw_wheel
        vel_right = linear_wheel + yaw_wheel
        targets = np.zeros(4)
        for i, leg in enumerate(LEG_NAMES):
            targets[i] = vel_left if leg[1] == "l" else vel_right
        return np.clip(targets, -WHEEL_VEL_MAX, WHEEL_VEL_MAX)

    def _differential_drive_single(self, vel_x: float, yaw_rate: float, leg: str) -> float:
        if leg[1] == "l":
            v = (vel_x - 0.5 * WHEEL_TRACK * yaw_rate) / WHEEL_RADIUS
        else:
            v = (vel_x + 0.5 * WHEEL_TRACK * yaw_rate) / WHEEL_RADIUS
        return np.clip(v, -WHEEL_VEL_MAX, WHEEL_VEL_MAX)

    # ─────────────────────────────────────────────────────────────────────
    # SWING TRAJECTORY
    # ─────────────────────────────────────────────────────────────────────

    def _get_swing_phase(self, leg: str) -> float:
        phase = (self._gait_phase + PHASE_OFFSETS[leg]) % 1.0
        if phase < GAIT_DUTY:
            return 0.0
        return (phase - GAIT_DUTY) / (1.0 - GAIT_DUTY)

    def _compute_swing_target(self, leg: str, state: RobotState,
                              swing_phase: float) -> np.ndarray:
        """Swing foot target with Raibert heuristic using COMMANDED velocity."""
        p_start = self._swing_start_foot[leg]
        p_end = self._compute_touchdown(leg, state)

        s = swing_phase
        s_mj = 10*s**3 - 15*s**4 + 6*s**5

        pos = p_start + (p_end - p_start) * s_mj

        # Z lift
        z_lift = 64.0 * s**3 * (1.0 - s)**3
        pos[2] = p_start[2] + SWING_HEIGHT * z_lift

        return pos

    def _compute_touchdown(self, leg: str, state: RobotState) -> np.ndarray:
        """Raibert heuristic using COMMANDED velocity.

        When commands are zero, foot lands at its takeoff position (no net motion).
        When commands are nonzero, foot placement is offset by commanded velocity.
        """
        # Base: land where the foot took off (zero net displacement)
        td = self._swing_start_foot[leg].copy()

        # Add commanded velocity offset (Raibert-style)
        t_stance = (1.0 / GAIT_FREQ) * GAIT_DUTY
        yaw = state.rpy[2]
        c, s = np.cos(yaw), np.sin(yaw)
        R_z = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        cmd_vel_world = R_z @ np.array([self._smooth_vx, self._smooth_vy, 0.0])

        td[0] += cmd_vel_world[0] * t_stance * 0.5
        td[1] += cmd_vel_world[1] * t_stance * 0.5
        td[2] = WHEEL_RADIUS  # ground level
        return td
