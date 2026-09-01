"""MPC controller integration for wheeled-legged robot.

Integrates: gait scheduler + reference trajectory + ConvexMPC solver +
            swing leg control + stance force mapping + wheel drive.

Architecture (following go2-convex-mpc):
  - MPC runs at ~50 Hz (every MPC_DECIMATION control steps)
  - Swing/stance leg controller runs at control rate (50 Hz)
  - Wheel drive: stance legs use differential drive, swing legs coast
"""

import numpy as np
from robot import Robot, RobotState
from dynamics import Dynamics
from mpc import ConvexMPC, MPC_DT
from config import (
    LEG_NAMES, WHEEL_RADIUS, WHEEL_TRACK, WHEEL_VEL_MAX,
    CTRL_DT, GAIT_FREQ, GAIT_DUTY, SWING_HEIGHT, PHASE_OFFSETS,
    DEFAULT_JOINT_ANGLES, ROBOT_MASS,
)

# MPC update decimation (relative to control loop)
MPC_DECIMATION = max(1, int(MPC_DT / CTRL_DT))  # 1 step at 50Hz


class MPCController:
    """Convex MPC locomotion controller for wheeled-legged robot."""

    def __init__(self, robot: Robot):
        self.robot = robot
        self.dynamics = Dynamics()
        self.mpc = ConvexMPC(mass=ROBOT_MASS)

        # User commands
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.yaw_rate = 0.0
        self.height = 0.35  # actual standing height with default joint angles

        # Gait state - start at phase 0 with all legs in stance (duty=0.6)
        self._gait_phase = 0.0
        self._step_count = 0
        self._initialized = False

        # MPC solution cache - initialize with gravity compensation
        self._mpc_forces = np.zeros(12)
        self._init_gravity_comp()

        # Swing trajectory state
        self._swing_start_foot = {leg: np.zeros(3) for leg in LEG_NAMES}
        self._swing_start_time = {leg: 0.0 for leg in LEG_NAMES}
        self._last_contact = {leg: True for leg in LEG_NAMES}

        # Smoothed commands
        self._smooth_vx = 0.0
        self._smooth_vy = 0.0
        self._smooth_yaw = 0.0

    def _init_gravity_comp(self):
        """Pre-fill MPC forces with static gravity compensation."""
        fz_per_leg = ROBOT_MASS * 9.81 / 4.0
        for i in range(4):
            self._mpc_forces[i*3 + 2] = fz_per_leg

    def compute(self, state: RobotState, dt: float):
        """Main MPC control loop.

        Uses MIT motor protocol: tau = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff
        where tau_ff comes from MPC force mapping via Jacobian transpose.

        Returns:
            tau_legs: (12,) feedforward torques for MIT mode
            wheel_targets: (4,) wheel velocity targets
        """
        # Smooth commands
        alpha = min(dt * 3.0, 1.0)
        self._smooth_vx += alpha * (self.vel_x - self._smooth_vx)
        self._smooth_vy += alpha * (self.vel_y - self._smooth_vy)
        self._smooth_yaw += alpha * (self.yaw_rate - self._smooth_yaw)

        # Update Pinocchio
        q_pin, dq_pin = self.robot.get_qpos_qvel_for_pinocchio()
        self.dynamics.update(q_pin, dq_pin)

        # Initialize foot positions on first call
        if not self._initialized:
            for leg in LEG_NAMES:
                self._swing_start_foot[leg] = self.dynamics.get_foot_pos(leg)
            self._initialized = True

        # Decide if we should trot or just stand
        moving = (abs(self._smooth_vx) > 0.02 or
                  abs(self._smooth_vy) > 0.02 or
                  abs(self._smooth_yaw) > 0.05)

        if moving:
            self._gait_phase = (self._gait_phase + dt * GAIT_FREQ) % 1.0
        else:
            self._gait_phase = 0.0  # all legs in stance

        # Contact schedule
        contacts = {}
        for leg in LEG_NAMES:
            phase = (self._gait_phase + PHASE_OFFSETS[leg]) % 1.0
            contacts[leg] = phase < GAIT_DUTY

        # Get foot positions relative to CoM
        foot_positions = np.zeros((4, 3))
        for i, leg in enumerate(LEG_NAMES):
            foot_positions[i] = self.dynamics.get_foot_pos(leg) - state.pos

        # --- Run MPC at lower rate ---
        if self._step_count % MPC_DECIMATION == 0:
            x0 = self._build_state_vector(state)
            x_ref = self._build_reference(state)
            contact_table = self._build_contact_table()
            self._mpc_forces = self.mpc.solve(x0, x_ref, foot_positions, contact_table)

        self._step_count += 1

        # --- Compute feedforward torques and desired joint positions ---
        tau_ff = np.zeros(12)
        q_des = np.zeros(12)
        dq_des = np.zeros(12)
        kp = np.zeros(12)
        kd = np.zeros(12)
        wheel_targets = np.zeros(4)

        for i, leg in enumerate(LEG_NAMES):
            if contacts[leg]:
                # Stance: MPC force → feedforward torque, PD holds posture
                f_leg = self._mpc_forces[i*3:(i+1)*3]
                J = self.dynamics.get_foot_jacobian_leg(leg)
                tau_ff[i*3:(i+1)*3] = J.T @ (-f_leg)

                # PD target: default standing angles (posture hold)
                q_des[i*3] = DEFAULT_JOINT_ANGLES["hip_abduction"]
                q_des[i*3+1] = DEFAULT_JOINT_ANGLES["hip_pitch"]
                q_des[i*3+2] = DEFAULT_JOINT_ANGLES["knee"]
                kp[i*3:(i+1)*3] = [40.0, 40.0, 40.0]
                kd[i*3:(i+1)*3] = [3.0, 3.0, 3.0]

                # Record foot position
                self._swing_start_foot[leg] = self.dynamics.get_foot_pos(leg)
                self._last_contact[leg] = True

                # Wheel drive
                wheel_targets[i] = self._wheel_cmd(leg)
            else:
                # Swing: IK target position, strong PD, no feedforward
                if self._last_contact[leg]:
                    self._swing_start_foot[leg] = self.dynamics.get_foot_pos(leg)
                    self._swing_start_time[leg] = state.time
                    self._last_contact[leg] = False

                q_ik = self._swing_leg_ik(leg, state, q_pin)
                q_des[i*3:(i+1)*3] = q_ik
                kp[i*3:(i+1)*3] = [60.0, 60.0, 60.0]  # strong PD for swing
                kd[i*3:(i+1)*3] = [3.0, 3.0, 3.0]
                # tau_ff stays 0 for swing

                wheel_targets[i] = 0.0

        # Use MIT protocol via robot interface
        self.robot.set_ctrl_mit(q_des, dq_des, kp, kd, tau_ff, wheel_targets)
        # Return dummy (actual ctrl is set directly above)
        return None, None

    def _build_state_vector(self, state: RobotState):
        """Build MPC state: [pos, rpy, vel, omega]."""
        return np.concatenate([state.pos, state.rpy, state.lin_vel, state.ang_vel])

    def _build_reference(self, state: RobotState):
        """Build reference trajectory over MPC horizon."""
        N = self.mpc.N
        x_ref = np.zeros((12, N))

        yaw = state.rpy[2]
        cy, sy = np.cos(yaw), np.sin(yaw)
        R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        vel_world = R_z @ np.array([self._smooth_vx, self._smooth_vy, 0.0])

        for k in range(N):
            t = (k + 1) * self.mpc.dt
            # Position: integrate from current
            x_ref[0, k] = state.pos[0] + vel_world[0] * t
            x_ref[1, k] = state.pos[1] + vel_world[1] * t
            x_ref[2, k] = self.height
            # RPY: keep roll/pitch zero, integrate yaw
            x_ref[3, k] = 0.0
            x_ref[4, k] = 0.0
            x_ref[5, k] = yaw + self._smooth_yaw * t
            # Velocity
            x_ref[6, k] = vel_world[0]
            x_ref[7, k] = vel_world[1]
            x_ref[8, k] = 0.0
            # Angular velocity
            x_ref[9, k] = 0.0
            x_ref[10, k] = 0.0
            x_ref[11, k] = self._smooth_yaw

        return x_ref

    def _build_contact_table(self):
        """Build contact schedule over MPC horizon."""
        N = self.mpc.N
        table = np.zeros((4, N), dtype=int)
        for k in range(N):
            future_phase = (self._gait_phase + (k + 1) * self.mpc.dt * GAIT_FREQ) % 1.0
            for i, leg in enumerate(LEG_NAMES):
                leg_phase = (future_phase + PHASE_OFFSETS[leg]) % 1.0
                table[i, k] = 1 if leg_phase < GAIT_DUTY else 0
        return table

    def _swing_leg_ik(self, leg: str, state: RobotState, q_pin: np.ndarray):
        """Swing leg: compute IK target joint angles for trajectory."""
        swing_phase = self._get_swing_phase(leg)

        p_start = self._swing_start_foot[leg]
        p_end = self._compute_touchdown(leg, state)

        s = swing_phase
        s_mj = 10*s**3 - 15*s**4 + 6*s**5

        pos_des = p_start + (p_end - p_start) * s_mj
        # Z lift
        z_lift = 64.0 * s**3 * (1.0 - s)**3
        pos_des[2] = p_start[2] + SWING_HEIGHT * z_lift

        # IK to get joint angles
        q_ik = self.dynamics.inverse_kinematics(leg, pos_des, q_pin)
        return q_ik

    def _get_swing_phase(self, leg: str) -> float:
        phase = (self._gait_phase + PHASE_OFFSETS[leg]) % 1.0
        if phase < GAIT_DUTY:
            return 0.0
        return (phase - GAIT_DUTY) / (1.0 - GAIT_DUTY)

    def _compute_touchdown(self, leg: str, state: RobotState) -> np.ndarray:
        """Raibert heuristic for touchdown position."""
        td = self._swing_start_foot[leg].copy()
        t_stance = GAIT_DUTY / GAIT_FREQ

        yaw = state.rpy[2]
        cy, sy = np.cos(yaw), np.sin(yaw)
        R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        cmd_vel_world = R_z @ np.array([self._smooth_vx, self._smooth_vy, 0.0])

        td[0] += cmd_vel_world[0] * t_stance * 0.5
        td[1] += cmd_vel_world[1] * t_stance * 0.5
        td[2] = WHEEL_RADIUS
        return td

    def _wheel_cmd(self, leg: str) -> float:
        """Differential drive for a single wheel."""
        if leg[1] == "l":
            v = (self._smooth_vx - 0.5 * WHEEL_TRACK * self._smooth_yaw) / WHEEL_RADIUS
        else:
            v = (self._smooth_vx + 0.5 * WHEEL_TRACK * self._smooth_yaw) / WHEEL_RADIUS
        return np.clip(v, -WHEEL_VEL_MAX, WHEEL_VEL_MAX)
