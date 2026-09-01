"""Pinocchio dynamics: FK, Jacobian, IK for the wheeled-legged robot."""

import numpy as np
import pinocchio as pin
from config import MJCF_PATH, LEG_NAMES

# Foot frame names in Pinocchio model (wheel link centers)
FOOT_FRAMES = {leg: f"{leg}_wheel_Link" for leg in LEG_NAMES}

# Leg joint names for each leg
_LEG_JOINT_NAMES = {
    leg: [f"{leg}_{jt}" for jt in ("hip_abduction_joint", "hip_pitch_joint", "knee_joint")]
    for leg in LEG_NAMES
}


class Dynamics:
    """Pinocchio-based kinematics/dynamics. Deployable on real hardware."""

    def __init__(self):
        self.model = pin.buildModelFromMJCF(str(MJCF_PATH))
        self.data = self.model.createData()

        # Cache frame IDs
        self._foot_fids = {}
        for leg, fname in FOOT_FRAMES.items():
            self._foot_fids[leg] = self.model.getFrameId(fname)

        # Cache joint velocity indices for each leg (3 joints)
        self._leg_v_indices = {}
        for leg, jnames in _LEG_JOINT_NAMES.items():
            indices = []
            for jn in jnames:
                jid = self.model.getJointId(jn)
                indices.append(self.model.joints[jid].idx_v)
            self._leg_v_indices[leg] = indices

        # Cache joint config indices for each leg
        self._leg_q_indices = {}
        for leg, jnames in _LEG_JOINT_NAMES.items():
            indices = []
            for jn in jnames:
                jid = self.model.getJointId(jn)
                indices.append(self.model.joints[jid].idx_q)
            self._leg_q_indices[leg] = indices

    def update(self, q: np.ndarray, dq: np.ndarray):
        """Forward kinematics + Jacobians.

        Args:
            q: Pinocchio config (nq=23: pos3, quat_xyzw4, joints16)
            dq: Pinocchio velocity (nv=22: v_body3, w_body3, joints16)
        """
        pin.forwardKinematics(self.model, self.data, q, dq)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data, q)

    def get_foot_pos(self, leg: str) -> np.ndarray:
        """Foot (wheel center) position in world frame (3,)."""
        return self.data.oMf[self._foot_fids[leg]].translation.copy()

    def get_foot_jacobian_leg(self, leg: str) -> np.ndarray:
        """3x3 linear Jacobian of foot w.r.t. 3 leg joints (world frame)."""
        fid = self._foot_fids[leg]
        J_full = pin.getFrameJacobian(
            self.model, self.data, fid, pin.LOCAL_WORLD_ALIGNED)[:3, :]
        cols = self._leg_v_indices[leg]
        return J_full[:, cols]

    def inverse_kinematics(self, leg: str, target_pos: np.ndarray,
                           q_current: np.ndarray, max_iter=30, eps=1e-4) -> np.ndarray:
        """Numerical IK for one leg. Returns (3,) joint angles.

        Args:
            leg: Leg name
            target_pos: Desired foot position in world frame (3,)
            q_current: Current full Pinocchio config (nq=23)
        """
        q = q_current.copy()
        fid = self._foot_fids[leg]
        q_indices = self._leg_q_indices[leg]

        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            err = target_pos - self.data.oMf[fid].translation
            if np.linalg.norm(err) < eps:
                break
            pin.computeJointJacobians(self.model, self.data, q)
            J = pin.getFrameJacobian(
                self.model, self.data, fid, pin.LOCAL_WORLD_ALIGNED)[:3, :]
            J_leg = J[:, self._leg_v_indices[leg]]
            dq = np.linalg.solve(J_leg.T @ J_leg + 1e-6 * np.eye(3), J_leg.T @ err)
            for i, idx in enumerate(q_indices):
                q[idx] += dq[i]

        return np.array([q[idx] for idx in q_indices])
