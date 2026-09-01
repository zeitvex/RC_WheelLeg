"""仿真→实机电机映射。

数据来源：sim_rl/ik_real/sim_to_real_deploy_beifen.py 和
sim_rl/sim2real/motor_mapping.py 中的 sign / offset / can_id 表（已在实机上验证）。
关节顺序与 rc_mjlab/sim2sim 完全一致：[12 个腿关节] + [4 个轮子]。
"""
from typing import Dict, Tuple

import numpy as np


class MotorMapping:
    LEG_NAMES = ("fl", "fr", "rl", "rr")
    JOINT_NAMES = ("hip_abduction", "hip_pitch", "knee", "wheel")

    SIM_JOINT_ORDER = (
        ("fl", "hip_abduction"), ("fl", "hip_pitch"), ("fl", "knee"),
        ("fr", "hip_abduction"), ("fr", "hip_pitch"), ("fr", "knee"),
        ("rl", "hip_abduction"), ("rl", "hip_pitch"), ("rl", "knee"),
        ("rr", "hip_abduction"), ("rr", "hip_pitch"), ("rr", "knee"),
        ("fl", "wheel"), ("fr", "wheel"), ("rl", "wheel"), ("rr", "wheel"),
    )
    SIM_INDEX_MAP = {jk: i for i, jk in enumerate(SIM_JOINT_ORDER)}

    CAN_ID_MAP: Dict[Tuple[str, str], Tuple[int, int]] = {
        ("fl", "hip_abduction"): (1, 1), ("fl", "hip_pitch"): (1, 2),
        ("fl", "knee"): (1, 3),          ("fl", "wheel"): (1, 4),
        ("fr", "hip_abduction"): (1, 5), ("fr", "hip_pitch"): (1, 6),
        ("fr", "knee"): (1, 7),          ("fr", "wheel"): (1, 8),
        ("rl", "hip_abduction"): (2, 1), ("rl", "hip_pitch"): (2, 2),
        ("rl", "knee"): (2, 3),          ("rl", "wheel"): (2, 4),
        ("rr", "hip_abduction"): (2, 5), ("rr", "hip_pitch"): (2, 6),
        ("rr", "knee"): (2, 7),          ("rr", "wheel"): (2, 8),
    }

    DIRECTION_MAP: Dict[Tuple[str, str], int] = {
        ("fl", "hip_abduction"): -1, ("fl", "hip_pitch"): -1,
        ("fl", "knee"): -1,          ("fl", "wheel"): -1,
        ("fr", "hip_abduction"): -1, ("fr", "hip_pitch"):  1,
        ("fr", "knee"):  1,          ("fr", "wheel"):  1,
        ("rl", "hip_abduction"):  1, ("rl", "hip_pitch"): -1,
        ("rl", "knee"): -1,          ("rl", "wheel"): -1,
        ("rr", "hip_abduction"):  1, ("rr", "hip_pitch"):  1,
        ("rr", "knee"):  1,          ("rr", "wheel"):  1,
    }

    ZERO_OFFSET_MAP: Dict[Tuple[str, str], float] = {
        ("fl", "hip_abduction"):  0.003, ("fl", "hip_pitch"):  0.030,
        ("fl", "knee"):           0.028, ("fl", "wheel"):      0.000,
        ("fr", "hip_abduction"):  0.004, ("fr", "hip_pitch"):  0.038,
        ("fr", "knee"):           0.011, ("fr", "wheel"):      0.000,
        ("rl", "hip_abduction"):  0.019, ("rl", "hip_pitch"): -0.034,
        ("rl", "knee"):           0.025, ("rl", "wheel"):      0.000,
        ("rr", "hip_abduction"): -0.001, ("rr", "hip_pitch"):  0.039,
        ("rr", "knee"):           0.018, ("rr", "wheel"):      0.000,
    }

    def __init__(self):
        self.num_motors = len(self.SIM_JOINT_ORDER)
        self._sign = np.array([self.DIRECTION_MAP[jk] for jk in self.SIM_JOINT_ORDER], dtype=np.float32)
        self._offset = np.array([self.ZERO_OFFSET_MAP[jk] for jk in self.SIM_JOINT_ORDER], dtype=np.float32)

    def sim_to_real(self, sim_angles: np.ndarray) -> Dict[Tuple[int, int], float]:
        if len(sim_angles) != 16:
            raise ValueError(f"expected 16 sim angles, got {len(sim_angles)}")
        out: Dict[Tuple[int, int], float] = {}
        for i, jk in enumerate(self.SIM_JOINT_ORDER):
            real = float(self._sign[i] * sim_angles[i] + self._offset[i])
            out[self.CAN_ID_MAP[jk]] = real
        return out

    def sim_vel_to_real(self, sim_vels: np.ndarray) -> Dict[Tuple[int, int], float]:
        # 速度只受方向影响，不应用 offset。
        out: Dict[Tuple[int, int], float] = {}
        for i, jk in enumerate(self.SIM_JOINT_ORDER):
            out[self.CAN_ID_MAP[jk]] = float(self._sign[i] * sim_vels[i])
        return out

    def real_to_sim(self, real_pos: Dict[Tuple[int, int], float]) -> np.ndarray:
        out = np.zeros(16, dtype=np.float32)
        for i, jk in enumerate(self.SIM_JOINT_ORDER):
            v = real_pos.get(self.CAN_ID_MAP[jk])
            if v is None:
                continue
            out[i] = (v - self._offset[i]) / self._sign[i]
        return out

    def real_vel_to_sim(self, real_vel: Dict[Tuple[int, int], float]) -> np.ndarray:
        out = np.zeros(16, dtype=np.float32)
        for i, jk in enumerate(self.SIM_JOINT_ORDER):
            v = real_vel.get(self.CAN_ID_MAP[jk])
            if v is None:
                continue
            out[i] = v / self._sign[i]
        return out

    def joint_name_at(self, idx: int) -> str:
        leg, joint = self.SIM_JOINT_ORDER[idx]
        return f"{leg}_{joint}_joint"
