import time
from typing import Callable, Dict, Tuple

import numpy as np

from interface.imu_client import IMUClient
from interface.motor_driver import HardwareIO
from tools.math_utils import LowPassFilter, MahonyFilter, get_gravity_orientation


class RealIO:
    def __init__(
        self,
        driver_factory: Callable[[str, str, bool], Tuple[object, object]],
        motor_model: str,
        can1_port: str,
        can2_port: str,
        imu_lib_path: str,
        control_dt: float = 0.02,
        kp_leg: float = 80.0,
        kd_leg: float = 2.5,
        kd_wheel: float = 2.0,
        debug: bool = False,
    ):
        self.control_dt = control_dt
        self.kp_leg = kp_leg
        self.kd_leg = kd_leg
        self.kd_wheel = kd_wheel

        print("[RealIO] 初始化电机驱动...")
        self.hw = HardwareIO(driver_factory, motor_model, can1_port, can2_port, debug)
        print("[RealIO] 初始化 IMU...")
        self.imu = IMUClient(lib_path=imu_lib_path)

        self.imu_filter = MahonyFilter(kp=2.0, ki=0.0, dt=control_dt)
        self.quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

        self.lpf_legs = LowPassFilter(cutoff_freq=5.0, dt=control_dt, dim=12)
        self.lpf_wheels = LowPassFilter(cutoff_freq=15.0, dt=control_dt, dim=4)

        self._last_imu_age_ms = -1.0
        self._last_imu_fresh = False

    def connect(self, imu_timeout_ms: int = 8000):
        self.hw.connect()
        self.imu.start(timeout_ms=imu_timeout_ms)
        if self.imu.initial_gravity is not None:
            self.imu_filter.reset_with_accel(self.imu.initial_gravity)
            self.quat_wxyz = self.imu_filter.q.copy()

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
        self._last_imu_age_ms = age_ms
        self._last_imu_fresh = fresh
        self.quat_wxyz = self.imu_filter.update(accel, gyro)
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
        self.hw.send_control(target, self.kp_leg * kp_scale, self.kd_leg, self.kd_wheel)
        return target
