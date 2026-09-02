"""起立姿态初始化器（实测起点版本）。

设计：
  - 不再假设机器人的物理起始姿态（不再有 CRAWL_POSE / GROUND_POSE 起点）
  - enable 后从 io.read_measured_pose() 读 16 关节实测，直接作为插值起点
  - 余弦插值到 STAND_POSE，transition_time 根据最大偏差自适应
  - 全程 RuntimeGuard 守护（空格急停/倾倒/翻滚/NaN/IMU 陈旧）
  - 50Hz 写 LogBundle CSV（phase 字段标识阶段）

Phase 流程：
  STARTUP_SOFT_HOLD  — 软起步保持实测姿态，kp 从 0.125 渐升到 1.0
  STARTUP_TRANSITION — 实测起点 → STAND 余弦插值
  STARTUP_HOLD_AFTER — 站稳后保持 1 秒
"""
import time
from typing import Optional

import numpy as np

from safety.runtime_guard import GuardLevel, RuntimeGuard
from tools.logger import LogBundle
from tools.math_utils import get_gravity_orientation


# 仅作为目标姿态使用（训练侧 default_dof_pos）
STAND_POSE = np.array([
    0.0, 0.9, -1.8,
    0.0, 0.9, -1.8,
    0.0, 0.9, -1.8,
    0.0, 0.9, -1.8,
    0.0, 0.0, 0.0, 0.0,
], dtype=np.float32)


def _periodic_leg_delta(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Return shortest signed leg-joint delta from src to dst."""
    delta = np.asarray(dst[:12], dtype=np.float32) - np.asarray(src[:12], dtype=np.float32)
    return ((delta + np.pi) % (2.0 * np.pi) - np.pi).astype(np.float32)


class PoseInitFailed(RuntimeError):
    """起立流程触发安全停止。main.py 捕获后立即 damping_brake。"""


class PoseInitializer:
    def __init__(self, real_io, control_dt: float = 0.02,
                 transition_time_min: float = 2.0,
                 transition_time_max: float = 6.0,
                 transition_seconds_per_rad: float = 1.5,
                 hold_time: float = 1.0,
                 settle_pos_threshold: float = 0.12,
                 settle_vel_threshold: float = 0.6,
                 timeout_extra: float = 3.0,
                 imu_fresh_wait_s: float = 1.0,
                 progress_log_interval: float = 0.5,
                 ramp_kp_time: float = 1.0,
                 soft_hold_duration: float = 1.0,
                 max_dev_warn: float = 1.5,
                 max_dev_abort: float = 3.0):
        """
        Args:
            transition_time_min/max/_per_rad: 自适应公式
                t = clip(min, max, max_dev * seconds_per_rad)
            timeout_extra: 起立超时 = transition_time + timeout_extra
            soft_hold_duration: 起立前先在实测姿态保持几秒，期间 kp ramp-up
            max_dev_warn: 最大偏差超过此值打警告（仅日志）
            max_dev_abort: 最大偏差超过此值直接 PoseInitFailed（拒绝起立）
        """
        self.io = real_io
        self.control_dt = control_dt
        self.transition_time_min = transition_time_min
        self.transition_time_max = transition_time_max
        self.transition_seconds_per_rad = transition_seconds_per_rad
        self.hold_time = hold_time
        self.settle_pos_threshold = settle_pos_threshold
        self.settle_vel_threshold = settle_vel_threshold
        self.timeout_extra = timeout_extra
        self.imu_fresh_wait_s = max(float(imu_fresh_wait_s), 0.0)
        self.progress_log_interval = progress_log_interval
        self.ramp_kp_time = ramp_kp_time
        self.soft_hold_duration = soft_hold_duration
        self.max_dev_warn = max_dev_warn
        self.max_dev_abort = max_dev_abort

        self.logger: Optional[LogBundle] = None
        self.guard: Optional[RuntimeGuard] = None
        self.keyboard = None

    def attach(self, logger: LogBundle, guard: RuntimeGuard, keyboard):
        self.logger = logger
        self.guard = guard
        self.keyboard = keyboard

    # ---- 通用每周期工作 ----
    def _tick(self, phase: str, sim_target: np.ndarray, kp_scale: float, next_exec: float):
        """读状态 → guard 检查 → 写日志 → 锁帧。返回 (state_dict, next_exec)。
        若 guard.STOP，立即抛 PoseInitFailed。"""
        loop_t0 = time.perf_counter()

        state = self.io.read_state()
        proj_g = get_gravity_orientation(state["quat_wxyz"])

        guard_dec = None
        if self.guard is not None:
            estop = bool(self.keyboard and self.keyboard.is_estop_triggered())
            guard_dec = self.guard.check(
                imu_gyro=state["imu_gyro"],
                projected_gravity=proj_g,
                imu_age_ms=float(state["imu_age_ms"]),
                estop_triggered=estop,
                extra_nan_arrays=(sim_target, state["joint_pos"], state["joint_vel"]),
            )

        if self.logger is not None:
            motor_diag = state.get("motor_stale", {})
            self.logger.state(
                phase=phase,
                joint_pos=state["joint_pos"],
                joint_vel=state["joint_vel"],
                joint_torque=state.get("joint_torque", np.zeros(16, dtype=np.float32)),
                target_pose=sim_target,
                raw_action=None,
                gyro=state["imu_gyro"],
                accel=state["imu_accel"],
                quat=state["quat_wxyz"],
                proj_gravity=proj_g,
                command=np.zeros(3, dtype=np.float32),
                imu_age_ms=float(state["imu_age_ms"]),
                loop_dt_ms=(time.perf_counter() - loop_t0) * 1000.0,
                safety_level=0,
                guard_level=int(guard_dec.level) if guard_dec else 0,
                holdover=int(motor_diag.get("holdover_this_frame", 0)),
                stale_max=int(motor_diag.get("stale_max", 0)),
                fresh_count=int(motor_diag.get("fresh_count", 16)),
                kp_scale=kp_scale,
                nan_flag=int(np.any(np.isnan(state["joint_pos"]))),
                kp_leg_cmd=float(getattr(self.io, "hold_kp_leg", self.io.kp_leg) * kp_scale),
                kd_leg_cmd=float(getattr(self.io, "hold_kd_leg", self.io.kd_leg)),
                kd_wheel_cmd=float(self.io.kd_wheel),
                target_source="startup_hold",
                guard_reason=guard_dec.reason if guard_dec else "",
            )

        if guard_dec is not None and guard_dec.level == GuardLevel.STOP:
            if self.logger:
                self.logger.event("GUARD_STOP", phase=phase, reason=guard_dec.reason)
            raise PoseInitFailed(f"[{phase}] {guard_dec.reason}")

        next_exec += self.control_dt
        slack = next_exec - time.perf_counter()
        if slack > 0:
            coarse = slack - 0.002
            if coarse > 0:
                time.sleep(coarse)
            micro_slack = next_exec - time.perf_counter()
            if micro_slack > 0:
                time.sleep(min(micro_slack, 0.001))
        else:
            next_exec = time.perf_counter()
        return state, next_exec

    # ---- 主入口：从实测姿态起立到 STAND ----
    def transition_to_stand_from_current(self,
                                          target_pose: Optional[np.ndarray] = None
                                          ) -> np.ndarray:
        """完整起立流程：
          1. 读实测起点
          2. 偏差检查（warn / abort）
          3. SOFT_HOLD：保持实测姿态 + kp ramp-up
          4. TRANSITION：余弦插值到 target，transition_time 自适应
          5. HOLD_AFTER：保持 1 秒
        返回最终 target_pose（供主循环使用）。
        """
        if target_pose is None:
            target_pose = STAND_POSE.copy()
        target_pose = target_pose.astype(np.float32).copy()
        target_pose[12:] = 0.0

        # === 1. 读实测起点（要求电机反馈完整）===
        ok, missing = self.io.wait_feedback_ready(max_attempts=20, poll_interval=0.05)
        if not ok:
            msg = f"feedback incomplete: {len(missing)} motors no response: {missing[:4]}"
            if self.logger:
                self.logger.event("STARTUP_NO_FEEDBACK",
                                  missing=[m[2] for m in missing])
            raise PoseInitFailed(msg)

        last_imu_age = -1.0
        imu_deadline = time.perf_counter() + self.imu_fresh_wait_s
        while self.imu_fresh_wait_s > 0.0 and time.perf_counter() < imu_deadline:
            state = self.io.read_state()
            last_imu_age = float(state.get("imu_age_ms", 1e9))
            if last_imu_age <= 60.0:
                break
            time.sleep(self.control_dt)
        else:
            if self.imu_fresh_wait_s > 0.0 and self.logger:
                self.logger.event("STARTUP_IMU_STALE_WARN", imu_age_ms=last_imu_age)

        start_pose = self.io.read_measured_pose().astype(np.float32).copy()
        start_pose[12:] = 0.0   # 轮子起点固定为 0 速度

        # === 2. 偏差检查 ===
        startup_delta = _periodic_leg_delta(start_pose, target_pose)
        diff = np.abs(startup_delta)
        max_dev = float(np.max(diff))
        max_dev_joint = int(np.argmax(diff))
        transition_time = float(np.clip(
            max_dev * self.transition_seconds_per_rad,
            self.transition_time_min, self.transition_time_max
        ))
        timeout = transition_time + self.timeout_extra

        if self.logger:
            self.logger.event(
                "STARTUP_PLAN",
                start_pose_leg=start_pose[:12].tolist(),
                target_pose_leg=target_pose[:12].tolist(),
                max_dev=max_dev,
                max_dev_joint_idx=max_dev_joint,
                transition_time=transition_time,
                timeout=timeout,
            )
        print(f"[PoseInit] 实测起点最大偏差 {max_dev:.3f} rad (关节 idx={max_dev_joint}); "
              f"transition_time={transition_time:.2f}s")

        if max_dev > self.max_dev_abort:
            raise PoseInitFailed(
                f"实测起点偏差过大 ({max_dev:.2f} rad > abort 阈值 "
                f"{self.max_dev_abort})；请检查电机是否在合理姿势"
            )
        if max_dev > self.max_dev_warn:
            print(f"[PoseInit] WARNING 偏差 {max_dev:.2f} rad > {self.max_dev_warn}; "
                  f"起立可能比较剧烈")
            if self.logger:
                self.logger.event("STARTUP_LARGE_DEV", max_dev=max_dev)

        # === 3. SOFT_HOLD：实测姿态 + kp ramp-up ===
        if self.logger:
            self.logger.event("STARTUP_SOFT_HOLD_BEGIN",
                              duration=self.soft_hold_duration,
                              ramp_kp_time=self.ramp_kp_time,
                              ramp_kp_min=0.125)
        n = max(1, int(self.soft_hold_duration / max(self.control_dt, 1e-3)))
        next_exec = time.perf_counter()
        t0 = next_exec
        ramp_min = 0.125
        for i in range(n):
            elapsed = time.perf_counter() - t0
            if elapsed < self.ramp_kp_time:
                kp_scale = ramp_min + (1.0 - ramp_min) * (elapsed / self.ramp_kp_time)
            else:
                kp_scale = 1.0
            self.io.hold_pose(start_pose, kp_scale=kp_scale)
            _s, next_exec = self._tick("STARTUP_SOFT_HOLD", start_pose, kp_scale, next_exec)
        if self.logger:
            self.logger.event("STARTUP_SOFT_HOLD_END")

        # === 4. TRANSITION：余弦插值 ===
        if self.logger:
            self.logger.event("STARTUP_TRANSITION_BEGIN",
                              transition_time=transition_time, timeout=timeout)
        print(f"[PoseInit] 起立: transition={transition_time:.2f}s, "
              f"hold={self.hold_time}s, timeout={timeout:.2f}s")

        t0 = time.perf_counter()
        last_log = t0
        reached = False
        hold_start: Optional[float] = None
        next_exec = t0

        while True:
            now = time.perf_counter()
            elapsed = now - t0
            phase = min(1.0, elapsed / max(transition_time, 1e-3))

            if elapsed > timeout:
                if self.logger:
                    self.logger.event("STARTUP_TIMEOUT", elapsed=elapsed)
                raise PoseInitFailed(
                    f"transition timeout after {elapsed:.2f}s, target not reached"
                )

            blend = 0.5 - 0.5 * np.cos(np.pi * phase)
            blended = start_pose.astype(np.float32).copy()
            blended[:12] = start_pose[:12] + blend * startup_delta
            blended[12:] = 0.0
            self.io.hold_pose(blended, kp_scale=1.0)
            state, next_exec = self._tick("STARTUP_TRANSITION", blended, 1.0, next_exec)

            joint_pos = state["joint_pos"]
            joint_vel = state["joint_vel"]
            pos_err = float(np.max(np.abs(_periodic_leg_delta(joint_pos, target_pose))))
            vel_err = float(np.max(np.abs(joint_vel[:12])))

            if now - last_log >= self.progress_log_interval:
                msg = (f"[PoseInit] phase={phase*100:5.1f}% | "
                       f"max_pos_err={pos_err:.3f} | max_vel={vel_err:.3f}")
                print(msg)
                if self.logger:
                    self.logger.event("STARTUP_PROGRESS",
                                      phase=phase, pos_err=pos_err, vel_err=vel_err)
                last_log = now

            if (phase >= 1.0
                    and pos_err <= self.settle_pos_threshold
                    and vel_err <= self.settle_vel_threshold):
                if not reached:
                    reached = True
                    hold_start = now
                    if self.logger:
                        self.logger.event("STARTUP_REACHED",
                                          pos_err=pos_err, vel_err=vel_err)
                    print(f"[PoseInit] 已到位，保持 {self.hold_time:.2f}s")
                elif hold_start is not None and now - hold_start >= self.hold_time:
                    break
            elif phase >= 1.0:
                reached = False
                hold_start = None

        # === 5. HOLD_AFTER ===
        if self.logger:
            self.logger.event("STARTUP_HOLD_AFTER_BEGIN", duration=self.hold_time)
        n_hold = max(1, int(self.hold_time / max(self.control_dt, 1e-3)))
        next_exec = time.perf_counter()
        for _ in range(n_hold):
            self.io.hold_pose(target_pose, kp_scale=1.0)
            _s, next_exec = self._tick("STARTUP_HOLD_AFTER", target_pose, 1.0, next_exec)

        if self.logger:
            self.logger.event("STARTUP_TRANSITION_END")
        print("[PoseInit] 默认站姿初始化完成")
        return target_pose

    # ---- 等用户回车（外部调用，期间持续保持） ----
    def hold_until_user_confirm(self, target_pose: np.ndarray, evt) -> bool:
        """阻塞循环到 evt.is_set()，期间持续 PD 保持站姿、跑 guard、写日志。
        返回 True 正常确认，False 因 guard.STOP 中止。"""
        if self.logger:
            self.logger.event("WAIT_USER_BEGIN")
        next_exec = time.perf_counter()
        while not evt.is_set():
            self.io.hold_pose(target_pose, kp_scale=1.0)
            try:
                _s, next_exec = self._tick("WAIT_USER", target_pose, 1.0, next_exec)
            except PoseInitFailed as e:
                print(f"[PoseInit] WAIT_USER 期间触发停止: {e}")
                return False
        if self.logger:
            self.logger.event("WAIT_USER_END")
        return True
