"""通用运行期守护：每个控制周期调用一次，无副作用，只做检查。

设计原则：
- 守护函数本身不下发动作、不打印（除非 verbose），只返回判定
- 调用方决定收到 GuardStop 时怎么办（damping_brake 或 raise）
- 起立期 / 等待期 / 主循环都共用同一组检查
"""
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

import numpy as np


class GuardLevel(IntEnum):
    OK = 0
    WARN = 1       # 仅记录，不停
    STOP = 2       # 主调方应立刻 damping_brake + 退出当前阶段


@dataclass
class GuardDecision:
    level: GuardLevel
    reason: str  # 触发时人类可读说明，OK 时为空


class RuntimeGuard:
    """启动/起立/主循环共用的安全守护。

    不监控目标位置范围（那是 SafetyMonitor 的职责）。这里只关心
    机身整体状态：是否倾倒、是否翻滚、是否检测到 NaN、用户是否按急停。
    """

    def __init__(self,
                 max_ang_vel: float = 12.0,
                 max_tilt_z: float = -0.30,
                 imu_age_warn_ms: float = 60.0,
                 imu_age_stop_ms: float = 200.0):
        self.max_ang_vel = max_ang_vel
        self.max_tilt_z = max_tilt_z
        self.imu_age_warn_ms = imu_age_warn_ms
        self.imu_age_stop_ms = imu_age_stop_ms

    def check(self,
              imu_gyro: np.ndarray,
              projected_gravity: np.ndarray,
              imu_age_ms: float,
              estop_triggered: bool,
              extra_nan_arrays: tuple = ()) -> GuardDecision:
        # 1) 用户急停
        if estop_triggered:
            return GuardDecision(GuardLevel.STOP, "user E-stop")

        # 2) NaN 检查（任意输入数组中出现 NaN）
        for arr in (imu_gyro, projected_gravity, *extra_nan_arrays):
            if arr is None:
                continue
            if np.any(np.isnan(arr)) or np.any(np.isinf(arr)):
                return GuardDecision(GuardLevel.STOP, "NaN/Inf detected in observation/action")

        # 3) IMU 数据陈旧
        if imu_age_ms > self.imu_age_stop_ms:
            return GuardDecision(GuardLevel.STOP, f"IMU stale {imu_age_ms:.0f}ms")
        warned_imu = imu_age_ms > self.imu_age_warn_ms

        # 4) 倾倒
        if projected_gravity[2] > self.max_tilt_z:
            return GuardDecision(GuardLevel.STOP,
                                 f"tilt: g_z={projected_gravity[2]:.3f}")

        # 5) 角速度爆表
        ang_norm = float(np.linalg.norm(imu_gyro))
        if ang_norm > self.max_ang_vel:
            return GuardDecision(GuardLevel.STOP, f"ang_vel overflow: |w|={ang_norm:.2f}")

        if warned_imu:
            return GuardDecision(GuardLevel.WARN, f"IMU age {imu_age_ms:.0f}ms")
        return GuardDecision(GuardLevel.OK, "")
