"""MuJoCo interface for the wheeled-legged robot.

Configures actuators as proper PD controllers at runtime:
  - Leg joints: force = kp*(ctrl - qpos) - kd*qvel, ctrl = target angle
  - Wheel joints: force = gain*(ctrl - qvel), ctrl = target velocity (rad/s)
"""

import numpy as np
import mujoco
from dataclasses import dataclass
from config import (SCENE_XML, LEG_NAMES, LEG_JOINTS, WHEEL_JOINT,
                    DEFAULT_JOINT_ANGLES, WHEEL_RADIUS, WHEEL_TRACK)


@dataclass
class RobotState:
    """Robot state from MuJoCo."""
    pos: np.ndarray       # (3,) world position
    quat: np.ndarray      # (4,) quaternion (w,x,y,z) MuJoCo convention
    rot: np.ndarray       # (3,3) body→world rotation
    rpy: np.ndarray       # (3,) roll, pitch, yaw
    lin_vel: np.ndarray   # (3,) world frame linear velocity
    ang_vel: np.ndarray   # (3,) body frame angular velocity
    joint_pos: np.ndarray # (16,) all joint positions [fl3+wheel, fr3+wheel, rl3+wheel, rr3+wheel]
    joint_vel: np.ndarray # (16,) all joint velocities
    time: float


class Robot:
    """MuJoCo simulation interface with proper PD actuator configuration."""

    # Leg PD gains (tuned for 12.3kg robot)
    LEG_KP = 60.0
    LEG_KD = 3.0
    # Wheel velocity gain
    WHEEL_KP = 2.0

    def __init__(self, xml_path=None):
        self.model = mujoco.MjModel.from_xml_path(str(xml_path or SCENE_XML))
        self.data = mujoco.MjData(self.model)

        # Cache IDs
        self._base_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self._actuator_ids = {}  # name → actuator index
        self._joint_qpos_adr = {}  # name → qpos address
        self._joint_qvel_adr = {}  # name → qvel address

        # Build joint/actuator maps
        self._ctrl_order = []
        for leg in LEG_NAMES:
            for jt in (*LEG_JOINTS, WHEEL_JOINT):
                name = f"{leg}_{jt}"
                aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                self._actuator_ids[name] = aid
                self._joint_qpos_adr[name] = self.model.jnt_qposadr[jid]
                self._joint_qvel_adr[name] = self.model.jnt_dofadr[jid]
                self._ctrl_order.append(name)

        # Configure actuators as proper PD controllers
        self._configure_actuators()

    def _configure_actuators(self):
        """Set actuators to proper PD mode.

        Leg joints: force = kp*(ctrl - qpos) - kd*qvel
        Wheels: force = gain*(ctrl - qvel)  (velocity tracking)
        """
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            self.model.actuator_biastype[i] = 1  # affine bias
            self.model.actuator_gaintype[i] = 0  # fixed gain
            self.model.actuator_forcelimited[i] = 0  # no force clamp (17Nm is in actuatorfrcrange)

            if 'wheel' not in name:
                self.model.actuator_gainprm[i, 0] = self.LEG_KP
                self.model.actuator_biasprm[i, 0] = 0.0
                self.model.actuator_biasprm[i, 1] = -self.LEG_KP
                self.model.actuator_biasprm[i, 2] = -self.LEG_KD
                self.model.actuator_ctrlrange[i] = [-3.14, 3.14]
            else:
                self.model.actuator_gainprm[i, 0] = self.WHEEL_KP
                self.model.actuator_biasprm[i, 0] = 0.0
                self.model.actuator_biasprm[i, 1] = 0.0
                self.model.actuator_biasprm[i, 2] = -self.WHEEL_KP
                self.model.actuator_ctrlrange[i] = [-20.0, 20.0]

    @property
    def dt(self):
        return self.model.opt.timestep

    def reset(self):
        """Reset to standing pose at correct height for default joint angles."""
        mujoco.mj_resetData(self.model, self.data)

        # Set default leg angles
        for leg in LEG_NAMES:
            for jt, key in zip(LEG_JOINTS, ("hip_abduction", "hip_pitch", "knee")):
                name = f"{leg}_{jt}"
                adr = self._joint_qpos_adr[name]
                self.data.qpos[adr] = DEFAULT_JOINT_ANGLES[key]

        # Compute correct base height from default angles using 2R FK
        # leg_length = sqrt(L1^2 + L2^2 - 2*L1*L2*cos(pi + knee))
        import math
        L1, L2 = 0.25, 0.20
        knee = DEFAULT_JOINT_ANGLES["knee"]
        leg_length = math.sqrt(L1**2 + L2**2 - 2*L1*L2*math.cos(math.pi + knee))
        # base_z = wheel_radius + leg_length - hip_z_offset
        base_z = 0.10 + leg_length - 0.054
        self.data.qpos[2] = base_z
        self.data.qpos[3] = 1.0  # quat w

        mujoco.mj_forward(self.model, self.data)

        # Set ctrl to match initial pose (so PD doesn't jerk)
        for leg in LEG_NAMES:
            for jt, key in zip(LEG_JOINTS, ("hip_abduction", "hip_pitch", "knee")):
                name = f"{leg}_{jt}"
                self.data.ctrl[self._actuator_ids[name]] = DEFAULT_JOINT_ANGLES[key]
            # Wheels: zero velocity
            self.data.ctrl[self._actuator_ids[f"{leg}_{WHEEL_JOINT}"]] = 0.0

    def get_state(self) -> RobotState:
        """Extract robot state."""
        pos = self.data.xpos[self._base_bid].copy()
        quat = self.data.xquat[self._base_bid].copy()  # (w,x,y,z)
        rot = self.data.xmat[self._base_bid].reshape(3, 3).copy()

        rpy = np.array([
            np.arctan2(rot[2, 1], rot[2, 2]),
            np.arctan2(-rot[2, 0], np.sqrt(rot[2, 1]**2 + rot[2, 2]**2)),
            np.arctan2(rot[1, 0], rot[0, 0]),
        ])

        # Base velocity (world frame)
        lin_vel = self.data.qvel[0:3].copy()
        ang_vel = self.data.qvel[3:6].copy()

        # Joint states (16 joints: 4 legs × 4 joints each)
        joint_pos = np.zeros(16)
        joint_vel = np.zeros(16)
        for i, name in enumerate(self._ctrl_order):
            joint_pos[i] = self.data.qpos[self._joint_qpos_adr[name]]
            joint_vel[i] = self.data.qvel[self._joint_qvel_adr[name]]

        return RobotState(
            pos=pos, quat=quat, rot=rot, rpy=rpy,
            lin_vel=lin_vel, ang_vel=ang_vel,
            joint_pos=joint_pos, joint_vel=joint_vel,
            time=self.data.time,
        )

    def set_ctrl(self, leg_targets: np.ndarray, wheel_targets: np.ndarray):
        """Set actuator commands (position PD mode).

        Args:
            leg_targets: (12,) target joint angles for legs [fl3, fr3, rl3, rr3]
            wheel_targets: (4,) target wheel velocities [fl, fr, rl, rr] in rad/s
        """
        for i, leg in enumerate(LEG_NAMES):
            for j, jt in enumerate(LEG_JOINTS):
                name = f"{leg}_{jt}"
                self.data.ctrl[self._actuator_ids[name]] = leg_targets[i * 3 + j]
            name = f"{leg}_{WHEEL_JOINT}"
            self.data.ctrl[self._actuator_ids[name]] = wheel_targets[i]

    def set_ctrl_mit(self, q_des: np.ndarray, dq_des: np.ndarray,
                     kp: np.ndarray, kd: np.ndarray, tau_ff: np.ndarray,
                     wheel_targets: np.ndarray):
        """MIT motor protocol: tau = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff.

        Computes torque in software, sends to actuators in torque mode.
        Call enable_torque_mode() first.

        Args:
            q_des: (12,) desired joint angles
            dq_des: (12,) desired joint velocities
            kp: (12,) position gains (0 for pure torque)
            kd: (12,) velocity gains
            tau_ff: (12,) feedforward torques
            wheel_targets: (4,) wheel velocity targets
        """
        for i, leg in enumerate(LEG_NAMES):
            for j, jt in enumerate(LEG_JOINTS):
                name = f"{leg}_{jt}"
                aid = self._actuator_ids[name]
                idx = i * 3 + j
                q = self.data.qpos[self._joint_qpos_adr[name]]
                dq = self.data.qvel[self._joint_qvel_adr[name]]
                tau = (kp[idx] * (q_des[idx] - q)
                       + kd[idx] * (dq_des[idx] - dq)
                       + tau_ff[idx])
                self.data.ctrl[aid] = np.clip(tau, -17.0, 17.0)
            name = f"{leg}_{WHEEL_JOINT}"
            self.data.ctrl[self._actuator_ids[name]] = wheel_targets[i]

    def enable_torque_mode(self):
        """Switch leg actuators to direct torque mode (for MPC/MIT)."""
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if 'wheel' not in name:
                self.model.actuator_gainprm[i, 0] = 1.0
                self.model.actuator_biasprm[i, :3] = [0, 0, 0]
                self.model.actuator_biastype[i] = 0
                self.model.actuator_ctrlrange[i] = [-17.0, 17.0]

    def enable_position_mode(self):
        """Switch leg actuators back to position PD mode."""
        self._configure_actuators()

    def step(self):
        """Advance one simulation timestep."""
        mujoco.mj_step(self.model, self.data)

    def get_qpos_qvel_for_pinocchio(self):
        """Get full qpos/qvel for Pinocchio (reorder quaternion)."""
        qpos = self.data.qpos.copy()
        qvel = self.data.qvel.copy()
        # MuJoCo quat: (w,x,y,z) → Pinocchio: (x,y,z,w)
        w, x, y, z = qpos[3], qpos[4], qpos[5], qpos[6]
        q_pin = np.concatenate([qpos[0:3], [x, y, z, w], qpos[7:]])
        # MuJoCo vel is already [lin_world(3), ang_body(3), joints(16)]
        # Pinocchio wants [lin_body(3), ang_body(3), joints(16)]
        from scipy.spatial.transform import Rotation
        R = Rotation.from_quat([x, y, z, w]).as_matrix()
        v_body = R.T @ qvel[0:3]
        dq_pin = np.concatenate([v_body, qvel[3:]])
        return q_pin, dq_pin
