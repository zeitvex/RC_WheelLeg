"""Convex MPC solver for wheeled-legged robot.

Centroidal dynamics: single rigid body model with 4 contact forces.
State: x = [pos(3), rpy(3), vel(3), omega(3)] = 12
Input: u = [f1(3), f2(3), f3(3), f4(3)] = 12
Friction pyramid constraints on each foot.

Reference: MIT Cheetah 3 Convex MPC (Di Carlo et al.)
"""

import numpy as np
from scipy import sparse
from scipy.linalg import block_diag
import osqp

from config import ROBOT_MASS, LEG_NAMES

# MPC parameters
MPC_HORIZON = 10       # prediction steps
MPC_DT = 0.02         # 50 Hz MPC update
MU = 0.6              # friction coefficient
FZ_MAX = 200.0        # max vertical force per leg
FZ_MIN = 10.0         # min vertical force (stance)
NX = 12               # state dim
NU = 12               # input dim (4 legs × 3D force)

# Cost weights: [pos_x, pos_y, pos_z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
Q_WEIGHTS = np.array([2.0, 2.0, 50.0, 50.0, 50.0, 10.0, 2.0, 2.0, 1.0, 1.0, 1.0, 1.0])
R_WEIGHTS = np.array([1e-6] * 12)


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


class ConvexMPC:
    """Convex MPC: solves QP for optimal ground reaction forces."""

    def __init__(self, mass=ROBOT_MASS, inertia=None):
        self.mass = mass
        # Approximate body inertia (diagonal, world-aligned)
        if inertia is None:
            self.I_body = np.diag([0.07, 0.26, 0.24])
        else:
            self.I_body = np.array(inertia).reshape(3, 3)

        self.N = MPC_HORIZON
        self.dt = MPC_DT
        self.Q = np.diag(Q_WEIGHTS)
        self.R = np.diag(R_WEIGHTS)
        self.gravity = np.array([0, 0, -9.81])

        self._last_forces = np.zeros(NU)

    def solve(self, x0, x_ref, foot_positions, contact_schedule):
        """Solve MPC QP.

        Args:
            x0: (12,) current state [pos, rpy, vel, omega]
            x_ref: (12, N) reference trajectory over horizon
            foot_positions: (4, 3) foot positions in world frame (relative to CoM)
            contact_schedule: (4, N) binary contact table (1=stance)

        Returns:
            forces: (12,) optimal forces for current timestep [f1x,f1y,f1z,...,f4x,f4y,f4z]
        """
        N = self.N

        # Build dynamics matrices
        Ad, Bd_list, gd = self._discretize_dynamics(x0, foot_positions)

        # Build QP: min 0.5 z'Hz + f'z  s.t. lb <= Az <= ub, lbx <= z <= ubx
        # Decision variables: z = [x1,...,xN, u0,...,uN-1]
        nvars = N * NX + N * NU

        # --- Hessian ---
        H_diag = np.concatenate([np.tile(2 * Q_WEIGHTS, N), np.tile(2 * R_WEIGHTS, N)])
        H = sparse.diags(H_diag, format='csc')

        # --- Gradient ---
        g = np.zeros(nvars)
        for k in range(N):
            g[k*NX:(k+1)*NX] = -2 * self.Q @ x_ref[:, k]

        # --- Dynamics equality constraints ---
        # x_{k+1} = Ad @ x_k + Bd_k @ u_k + gd
        # Rewrite: x_{k+1} - Ad @ x_k - Bd_k @ u_k = gd (for k>0)
        #           x_1 - Bd_0 @ u_0 = Ad @ x0 + gd (for k=0)
        n_eq = N * NX
        A_eq = np.zeros((n_eq, nvars))
        b_eq = np.zeros(n_eq)

        # k=0: x_1 = Ad @ x0 + Bd_0 @ u_0 + gd
        A_eq[0:NX, 0:NX] = np.eye(NX)  # x_1
        A_eq[0:NX, N*NX:N*NX+NU] = -Bd_list[0]  # -Bd_0 @ u_0
        b_eq[0:NX] = Ad @ x0 + gd

        for k in range(1, N):
            row = k * NX
            # x_{k+1}
            A_eq[row:row+NX, k*NX:(k+1)*NX] = np.eye(NX)
            # -Ad @ x_k
            A_eq[row:row+NX, (k-1)*NX:k*NX] = -Ad
            # -Bd_k @ u_k
            A_eq[row:row+NX, N*NX+k*NU:N*NX+(k+1)*NU] = -Bd_list[k]
            b_eq[row:row+NX] = gd

        # --- Friction pyramid inequality constraints ---
        # For each stance leg at each timestep: 4 faces
        # fx - mu*fz <= 0, -fx - mu*fz <= 0, fy - mu*fz <= 0, -fy - mu*fz <= 0
        n_ineq_max = 4 * 4 * N
        A_ineq = np.zeros((n_ineq_max, nvars))
        u_ineq = np.zeros(n_ineq_max)

        row = 0
        for k in range(N):
            u_base = N * NX + k * NU
            for leg in range(4):
                if contact_schedule[leg, k] == 1:
                    fx_idx = u_base + leg * 3
                    fy_idx = u_base + leg * 3 + 1
                    fz_idx = u_base + leg * 3 + 2

                    # Friction pyramid: stance leg
                    A_ineq[row, fx_idx] = 1.0
                    A_ineq[row, fz_idx] = -MU
                    row += 1
                    A_ineq[row, fx_idx] = -1.0
                    A_ineq[row, fz_idx] = -MU
                    row += 1
                    A_ineq[row, fy_idx] = 1.0
                    A_ineq[row, fz_idx] = -MU
                    row += 1
                    A_ineq[row, fy_idx] = -1.0
                    A_ineq[row, fz_idx] = -MU
                    row += 1

        A_ineq = A_ineq[:row]
        u_ineq = u_ineq[:row]

        # Stack constraints
        A_full = np.vstack([A_eq, A_ineq])
        l_full = np.concatenate([b_eq, -np.inf * np.ones(row)])
        u_full = np.concatenate([b_eq, u_ineq])

        # --- Box constraints on forces (as identity rows in A) ---
        A_box = np.zeros((N * NU, nvars))
        l_box = -np.inf * np.ones(N * NU)
        u_box = np.inf * np.ones(N * NU)

        for k in range(N):
            u_base = N * NX + k * NU
            for leg in range(4):
                idx = u_base + leg * 3
                box_row = k * NU + leg * 3
                # Identity rows for fx, fy, fz
                for j in range(3):
                    A_box[box_row + j, idx + j] = 1.0

                if contact_schedule[leg, k] == 1:
                    # Stance: fz bounded
                    l_box[box_row + 2] = FZ_MIN
                    u_box[box_row + 2] = FZ_MAX
                else:
                    # Swing: all forces = 0
                    l_box[box_row:box_row+3] = 0.0
                    u_box[box_row:box_row+3] = 0.0

        # Final constraint matrix
        A_full = np.vstack([A_full, A_box])
        l_full = np.concatenate([l_full, l_box])
        u_full = np.concatenate([u_full, u_box])

        # --- Solve with OSQP ---
        A_sparse = sparse.csc_matrix(A_full)
        H_sparse = sparse.triu(H, format='csc')

        solver = osqp.OSQP()
        solver.setup(H_sparse, g, A_sparse, l_full, u_full,
                     eps_abs=1e-4, eps_rel=1e-4,
                     max_iter=500, polish=True, verbose=False,
                     warm_start=True)

        # Warm start with previous solution
        if self._last_forces is not None:
            x_warm = np.zeros(nvars)
            x_warm[N*NX:N*NX+NU] = self._last_forces
            solver.warm_start(x=x_warm)

        result = solver.solve()

        if result.info.status == 'solved' or result.info.status == 'solved_inaccurate':
            # Extract first timestep forces
            forces = result.x[N*NX:N*NX+NU]
            self._last_forces = forces.copy()
        else:
            forces = self._last_forces

        return forces

    def _discretize_dynamics(self, x0, foot_positions):
        """Build discrete-time centroidal dynamics.

        State: [pos, rpy, vel, omega] (12)
        Continuous: dx/dt = Ac @ x + Bc @ u + gc
        Discrete: x_{k+1} = Ad @ x + Bd @ u + gd
        """
        m = self.mass
        I_inv = np.linalg.inv(self.I_body)
        dt = self.dt
        yaw = x0[5]
        cy, sy = np.cos(yaw), np.sin(yaw)

        # Rotation for rpy rate ≈ R_z^T @ omega
        R_zT = np.array([[cy, sy, 0], [-sy, cy, 0], [0, 0, 1]])

        # Ac (12×12)
        Ac = np.zeros((NX, NX))
        Ac[0:3, 6:9] = np.eye(3)       # pos_dot = vel
        Ac[3:6, 9:12] = R_zT           # rpy_dot ≈ R_z^T @ omega

        # Ad = I + Ac*dt (first-order)
        Ad = np.eye(NX) + Ac * dt

        # Bc varies per timestep (foot positions change contact point)
        Bd_list = []
        for k in range(self.N):
            Bc = np.zeros((NX, NU))
            for leg in range(4):
                r = foot_positions[leg]
                # vel_dot += f/m
                Bc[6:9, leg*3:(leg+1)*3] = np.eye(3) / m
                # omega_dot += I^{-1} @ (r × f)
                Bc[9:12, leg*3:(leg+1)*3] = I_inv @ _skew(r)
            Bd = Bc * dt
            Bd_list.append(Bd)

        # Gravity contribution
        gd = np.zeros(NX)
        gd[6:9] = self.gravity * dt  # vel += g*dt

        return Ad, Bd_list, gd
