"""三级安全监控（对应方法论 97.11）。

Level 0: 正常
Level 1: 限幅（位置/速度异常）— 截断目标位置幅值，记录连续触发次数
Level 2: 刹车（连续限幅 N 次 / IMU 角速度过大 / 倾倒）— 卸载刚度只留阻尼
Level 3: 急停（用户触发）— 让上层断电

设计原则：监控只判定，不直接关电机；返回 SafetyDecision 由上层决策。
"""
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Optional

import numpy as np


class SafetyLevel(IntEnum):
    NORMAL = 0
    CLIP = 1
    BRAKE = 2
    ESTOP = 3


@dataclass
class SafetyDecision:
    level: SafetyLevel
    message: str
    clipped_target: Optional[np.ndarray]
    details: Optional[dict[str, Any]] = None


class SafetyMonitor:
    """安全监控（按 50Hz 控制频率调用）。

    Args:
        max_target_offset: 单关节相对默认位姿的最大偏离 (rad)
        max_ang_vel: IMU 角速度模 (rad/s)
        max_tilt_rad: 机身重力 z 轴投影低于该值认为已严重倾倒
        clip_to_brake: 连续 clip 多少帧升级为刹车
    """

    def __init__(self,
                 max_target_offset: float = 0.6,
                 max_ang_vel: float = 10.0,
                 max_tilt_z: float = -0.3,
                 clip_to_brake: int = 0,
                 hard_target_offset: float = 1.2):
        self.max_target_offset = max_target_offset
        self.max_ang_vel = max_ang_vel
        self.max_tilt_z = max_tilt_z  # projected_gravity z 应当 ~ -1，明显小于 -0.3 视作倾倒
        self.clip_to_brake = clip_to_brake
        self.hard_target_offset = hard_target_offset
        self.consecutive_clips = 0

    def check(self,
              target_pose: np.ndarray,
              default_pose: np.ndarray,
              imu_gyro: np.ndarray,
              projected_gravity: np.ndarray,
              estop_triggered: bool) -> SafetyDecision:
        if estop_triggered:
            return SafetyDecision(SafetyLevel.ESTOP, "user E-stop", None, None)

        # 倾倒（projected_gravity[2] 应在 -1 附近，越接近 0 越倾斜）
        if projected_gravity[2] > self.max_tilt_z:
            return SafetyDecision(
                SafetyLevel.BRAKE,
                f"tilt detected: g_z={projected_gravity[2]:.3f}",
                None,
                {"g_z": float(projected_gravity[2])},
            )

        # 角速度爆表（猛烈翻滚）
        if np.linalg.norm(imu_gyro) > self.max_ang_vel:
            return SafetyDecision(
                SafetyLevel.BRAKE,
                f"angular velocity overflow: |w|={np.linalg.norm(imu_gyro):.2f}",
                None,
                {"ang_vel_norm": float(np.linalg.norm(imu_gyro))},
            )

        # 目标位置偏离过大 → 截断到允许范围
        offset_leg = target_pose[:12] - default_pose[:12]
        clipped_offset = np.clip(offset_leg, -self.max_target_offset, self.max_target_offset)
        if not np.allclose(offset_leg, clipped_offset):
            self.consecutive_clips += 1
            clipped = target_pose.copy()
            clipped[:12] = default_pose[:12] + clipped_offset
            exceeded = np.where(np.abs(offset_leg) > self.max_target_offset)[0].tolist()
            max_offset = float(np.max(np.abs(offset_leg)))
            details = {
                "joint_indices": exceeded,
                "max_leg_offset": max_offset,
                "consecutive_clips": int(self.consecutive_clips),
            }
            if self.hard_target_offset > 0.0 and max_offset > self.hard_target_offset:
                return SafetyDecision(
                    SafetyLevel.BRAKE,
                    f"target leg offset exceeds hard limit: {max_offset:.3f}",
                    clipped,
                    details,
                )
            if self.clip_to_brake > 0 and self.consecutive_clips >= self.clip_to_brake:
                return SafetyDecision(
                    SafetyLevel.BRAKE,
                    f"clipped {self.consecutive_clips} frames in a row",
                    clipped,
                    details,
                )
            return SafetyDecision(SafetyLevel.CLIP, "target leg offset out of range", clipped, details)

        self.consecutive_clips = 0
        return SafetyDecision(SafetyLevel.NORMAL, "", None, None)

    def reset(self):
        self.consecutive_clips = 0
