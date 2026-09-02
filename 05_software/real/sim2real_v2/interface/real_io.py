import time
from typing import Callable, Dict, Tuple

import numpy as np

from interface.imu_client import IMUClient
from interface.motor_driver import HardwareIO
from tools.math_utils import LowPassFilter, MahonyFilter, get_gravity_orientation


def _quat_yaw_wxyz(quat) -> float:
    w, x, y, z = [float(v) for v in quat]
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _wrap_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


class OdomTracker:
    def __init__(self, jump_distance_m: float = 0.5, jump_yaw_rad: float = 0.8):
        self.jump_distance_m = float(jump_distance_m)
        self.jump_yaw_rad = float(jump_yaw_rad)
        self.origin_pos = None
        self.origin_yaw = 0.0
        self.last_local_pos = None
        self.last_local_yaw = 0.0

    def reset(self):
        self.origin_pos = None
        self.origin_yaw = 0.0
        self.last_local_pos = None
        self.last_local_yaw = 0.0

    def update(self, odom):
        if odom is None:
            return None
        pos = np.asarray(odom.get("pos", [0.0, 0.0, 0.0]), dtype=np.float32)
        yaw = _quat_yaw_wxyz(odom.get("quat_wxyz", [1.0, 0.0, 0.0, 0.0]))
        if self.origin_pos is None:
            self.origin_pos = pos.copy()
            self.origin_yaw = yaw
        local_pos = pos - self.origin_pos
        local_yaw = _wrap_pi(yaw - self.origin_yaw)
        jump_detected = False
        jump_distance = 0.0
        jump_yaw = 0.0
        if self.last_local_pos is not None:
            jump_distance = float(np.linalg.norm(local_pos[:2] - self.last_local_pos[:2]))
            jump_yaw = abs(_wrap_pi(local_yaw - self.last_local_yaw))
            jump_detected = jump_distance > self.jump_distance_m or jump_yaw > self.jump_yaw_rad
        self.last_local_pos = local_pos.copy()
        self.last_local_yaw = local_yaw
        tracked = dict(odom)
        tracked.update(
            {
                "local_pos": local_pos.tolist(),
                "local_yaw": local_yaw,
                "jump_detected": bool(jump_detected),
                "jump_distance_m": jump_distance,
                "jump_yaw_rad": jump_yaw,
            }
        )
        return tracked


class RealIO:
    def __init__(
        self,
        driver_factory: Callable[[str, str, bool], Tuple[object, object]],
        motor_model: str,
        can1_port: str,
        can2_port: str,
        imu_lib_path: str,
        control_dt: float = 0.02,
        motor_dt: float = 0.005,
        kp_leg: float = 80.0,
        kd_leg: float = 2.5,
        hold_kp_leg: float | None = None,
        hold_kd_leg: float | None = None,
        kd_wheel: float = 2.0,
        debug: bool = False,
        dry_run: bool = False,
    ):
        self.control_dt = control_dt
        self.motor_dt = motor_dt
        self.kp_leg = kp_leg
        self.kd_leg = kd_leg
        self.hold_kp_leg = kp_leg if hold_kp_leg is None else float(hold_kp_leg)
        self.hold_kd_leg = kd_leg if hold_kd_leg is None else float(hold_kd_leg)
        self.kd_wheel = kd_wheel

        print("[RealIO] 初始化电机驱动...")
        self.hw = HardwareIO(driver_factory, motor_model, can1_port, can2_port, debug)
        print("[RealIO] 初始化 IMU...")
        self.imu = IMUClient(lib_path=imu_lib_path, dry_run=dry_run)

        # 使用 motor_dt 初始化滤波器，因为它们都在 200Hz 电机控制循环中更新
        self.imu_filter = MahonyFilter(kp=2.0, ki=0.0, dt=motor_dt)
        self.quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

        self.lpf_legs = LowPassFilter(cutoff_freq=5.0, dt=motor_dt, dim=12)
        self.lpf_wheels = LowPassFilter(cutoff_freq=15.0, dt=motor_dt, dim=4)

        self._last_imu_age_ms = -1.0
        self._last_imu_fresh = False
        self.odom_tracker = OdomTracker()
        self._last_read_time = None

    def connect(self, imu_timeout_ms: int = 8000):
        self.hw.connect()
        self.imu.start(timeout_ms=imu_timeout_ms)
        if self.imu.initial_gravity is not None:
            self.imu_filter.reset_with_accel(self.imu.initial_gravity)
            self.quat_wxyz = self.imu_filter.q.copy()
        self.odom_tracker.reset()
        self._last_read_time = None

    def disconnect(self):
        try:
            self.hw.disable_all()
        finally:
            self.imu.stop()
            self.hw.disconnect()

    def enable_motors(self):
        self.hw.enable_all()

    def disable_motors(self):
        self.hw.disable_all()

    def damping_brake(self):
        self.hw.damping_brake(self.kd_leg, self.kd_wheel)

    def wait_feedback_ready(self, max_attempts: int = 20, poll_interval: float = 0.05):
        return self.hw.wait_feedback_ready(max_attempts=max_attempts, poll_interval=poll_interval)

    def read_measured_pose(self) -> np.ndarray:
        return self.hw.read_measured_pose()

    def read_state(self) -> Dict[str, object]:
        joint_pos, joint_vel, joint_torque, motor_diag = self.hw.read_state()
        gyro, accel, age_ms, fresh = self.imu.get_latest()
        odom = self.odom_tracker.update(self.imu.get_latest_odom())
        self._last_imu_age_ms = age_ms
        self._last_imu_fresh = fresh

        # 动态测量 dt，以适应 POLL (5Hz) 与 RUNTIME (200Hz) 的不同频率切换
        t_now = time.perf_counter()
        if self._last_read_time is not None:
            dt = t_now - self._last_read_time
            if dt <= 0.0 or dt > 0.5:
                dt = self.motor_dt
        else:
            dt = self.motor_dt
        self._last_read_time = t_now

        self.quat_wxyz = self.imu_filter.update(accel, gyro, dt=dt)
        projected_gravity = get_gravity_orientation(self.quat_wxyz)

        return {
            "joint_pos": joint_pos,
            "joint_vel": joint_vel,
            "joint_torque": joint_torque,
            "imu_gyro": gyro,
            "imu_accel": accel,
            "quat_wxyz": self.quat_wxyz.copy(),
            "projected_gravity": projected_gravity,
            "imu_age_ms": age_ms,
            "imu_fresh": fresh,
            "odom": odom,
            "motor_stale": motor_diag,
        }

    def get_obs_policy(
        self,
        state: Dict[str, object],
        command: np.ndarray,
        default_dof_pos: np.ndarray,
        last_actions_raw: np.ndarray,
    ) -> np.ndarray:
        gyro = state["imu_gyro"]
        joint_pos = state["joint_pos"]
        joint_vel = state["joint_vel"]
        projected_gravity = state["projected_gravity"]

        base_ang_vel = (gyro * 0.25).astype(np.float32)
        joint_pos_rel = (joint_pos[:12] - default_dof_pos[:12]).astype(np.float32)
        joint_vel_leg = (joint_vel[:12] * 0.05).astype(np.float32)
        wheel_vel = (joint_vel[12:] * 0.05).astype(np.float32)

        return np.concatenate(
            [
                base_ang_vel,
                projected_gravity,
                command.astype(np.float32),
                joint_pos_rel,
                joint_vel_leg,
                wheel_vel,
                last_actions_raw,
            ]
        ).astype(np.float32)

    def send_actions(self, scaled_actions: np.ndarray, default_dof_pos: np.ndarray):
        act = (scaled_actions + default_dof_pos).astype(np.float32)
        act = np.clip(act, -100.0, 100.0)
        act[:12] = self.lpf_legs.filter(act[:12])
        act[12:] = self.lpf_wheels.filter(act[12:])
        self.hw.send_control(act, self.kp_leg, self.kd_leg, self.kd_wheel)
        return act

    def hold_pose(self, sim_target_pose: np.ndarray, kp_scale: float = 1.0):
        target = np.clip(sim_target_pose.astype(np.float32), -100.0, 100.0)
        kp_scale = float(np.clip(kp_scale, 0.0, 1.0))
        self.hw.send_control(target, self.hold_kp_leg * kp_scale, self.hold_kd_leg, self.kd_wheel)
        return target
