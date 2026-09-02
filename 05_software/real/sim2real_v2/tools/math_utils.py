"""数学工具 — 与 rc_mjlab/sim2sim/tools/math_utils.py 数值完全一致。"""
from typing import Optional
import numpy as np


def get_gravity_orientation(quat_wxyz: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = quat_wxyz
    gx = 2.0 * (-qz * qx + qw * qy)
    gy = -2.0 * (qz * qy + qw * qx)
    gz = 1.0 - 2.0 * (qw * qw + qz * qz)
    return np.array([gx, gy, gz], dtype=np.float32)


def quat_rotate_inverse(quat_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    q_w = quat_wxyz[0]
    q_vec = quat_wxyz[1:]
    a = v * (2.0 * q_w * q_w - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c


def quat_from_accel(accel: np.ndarray) -> np.ndarray:
    """用静止重力方向初始化机身姿态四元数。

    思想：仿真启动时 quat = [1,0,0,0] 隐含"机身完全水平"，但真机摆在地面上
    pitch/roll 通常各自有几度偏差，会让 projected_gravity 一开始就错。
    用加速度计读数与 [0,0,-1] 的最短旋转作为初值，可以把首步重力误差
    降到 IMU 噪声级。
    """
    g_meas = accel / (np.linalg.norm(accel) + 1e-9)
    g_ref = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    cross = np.cross(g_ref, g_meas)
    dot = float(np.dot(g_ref, g_meas))
    if dot < -0.999999:
        return np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float32)
    s = float(np.sqrt((1.0 + dot) * 2.0))
    q = np.array([s * 0.5, cross[0] / s, cross[1] / s, cross[2] / s], dtype=np.float32)
    return q / (np.linalg.norm(q) + 1e-9)


class LowPassFilter:
    """一阶 IIR 低通，alpha 公式与训练侧 rc_mjlab/src/robot/mdp/lowpass_actions.py
    `_lowpass_weights` 完全一致：

        alpha = 1 - exp(-2π · cutoff_freq / control_freq)
              = 1 - exp(-2π · cutoff_freq · dt)

    注意：这与 rc_mjlab/sim2sim/interface/mujoco_io.py 用的近似公式
    (dt / (dt + 1/(2π·fc))) 数值上不同，在 15Hz 截止时差约 30%。
    我们以训练侧为准，因为策略是在那个滤波下学的。
    """

    def __init__(self, cutoff_freq: float, dt: float, dim: int):
        self.alpha = float(1.0 - np.exp(-2.0 * np.pi * cutoff_freq * dt))
        self.y_prev = None

    def filter(self, x: np.ndarray) -> np.ndarray:
        if self.y_prev is None:
            self.y_prev = x.copy()
        y = self.alpha * x + (1.0 - self.alpha) * self.y_prev
        self.y_prev = y.copy()
        return y

    def reset(self):
        self.y_prev = None


class MahonyFilter:
    """互补滤波器：高频用陀螺仪积分，低频用加速度计修正。"""

    def __init__(self, kp: float = 2.0, ki: float = 0.0, dt: float = 0.02):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.e_int = np.zeros(3, dtype=np.float32)

    def reset_with_accel(self, accel: np.ndarray):
        self.q = quat_from_accel(accel)
        self.e_int.fill(0.0)

    def update(self, accel: np.ndarray, gyro: np.ndarray, dt: Optional[float] = None) -> np.ndarray:
        if dt is None:
            dt = self.dt
        norm_a = float(np.linalg.norm(accel))
        if norm_a > 1e-6:
            a = accel / norm_a
            q = self.q
            v = np.array([
                2.0 * (q[1] * q[3] - q[0] * q[2]),
                2.0 * (q[0] * q[1] + q[2] * q[3]),
                q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3],
            ], dtype=np.float32)
            e = np.cross(a, v)
            if self.ki > 0.0:
                self.e_int += e * dt
            else:
                self.e_int.fill(0.0)
            gyro = gyro + self.kp * e + self.ki * self.e_int

        q = self.q
        q_dot = 0.5 * np.array([
            -q[1] * gyro[0] - q[2] * gyro[1] - q[3] * gyro[2],
             q[0] * gyro[0] + q[2] * gyro[2] - q[3] * gyro[1],
             q[0] * gyro[1] - q[1] * gyro[2] + q[3] * gyro[0],
             q[0] * gyro[2] + q[1] * gyro[1] - q[2] * gyro[0],
        ], dtype=np.float32)
        self.q += q_dot * dt
        self.q /= (np.linalg.norm(self.q) + 1e-9)
        return self.q
