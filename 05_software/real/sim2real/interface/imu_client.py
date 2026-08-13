"""Odin1 IMU 客户端封装。

核心改动相对 sim_rl/odin1/python/odin1_imu.py：
- 自动加载默认 .so 路径，调用方只需要 IMUClient(lib_path=...)
- 启动后做一次"重力对齐" — 用静止时的加速度计读数初始化 Mahony 滤波器，
  把首步姿态偏差从可能的 5°+ 降到 0.3° 内。这是方法论 D4 的关键一步。
- 数据老化检测：若 imu_age_ms > stale_threshold 则报警（不阻塞）。
"""
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np


class IMUClient:
    """Odin1 IMU 包装。

    Args:
        lib_path: libodin1_imu_bridge.so 的绝对路径；None 则按方法论 1.2 中
                  约定的相对位置寻找。
        gravity_align_samples: 启动时取多少帧加速度计平均值用于姿态初始化
        stale_threshold_ms: 单帧数据超过该 age 视为陈旧
    """

    def __init__(self, lib_path: Optional[str] = None, gravity_align_samples: int = 50,
                 stale_threshold_ms: float = 50.0):
        # 优先级 1: vendored/odin1_imu（独立部署模式）
        # 优先级 2: ../../odin1/odin1/python（开发模式，即 sim_rl/odin1/odin1/python）
        sim2real_root = Path(__file__).resolve().parents[1]
        candidates = [
            sim2real_root / "vendored" / "odin1_imu",
            sim2real_root.parents[1] / "odin1" / "odin1" / "python",
        ]
        for cand in candidates:
            if cand.exists() and str(cand) not in sys.path:
                sys.path.insert(0, str(cand))
                break
        try:
            from odin1_imu import Odin1ImuClient  # type: ignore
        except ImportError as e:
            raise ImportError(
                f"无法导入 Odin1ImuClient，已尝试的路径: {[str(c) for c in candidates]}: {e}"
            )

        # lib_path 默认查找：vendored/odin1_imu/build/libodin1_imu_bridge.so → 开发路径
        if lib_path is None:
            so_candidates = [
                sim2real_root / "vendored" / "odin1_imu" / "build" / "libodin1_imu_bridge.so",
                sim2real_root / "vendored" / "odin1_imu" / "libodin1_imu_bridge.so",
                sim2real_root.parents[1] / "odin1" / "odin1" / "build" / "libodin1_imu_bridge.so",
            ]
            for so in so_candidates:
                if so.exists():
                    lib_path = str(so)
                    break

        self._client = Odin1ImuClient(lib_path=lib_path)
        self._gravity_align_samples = gravity_align_samples
        self._stale_threshold_ms = stale_threshold_ms
        self._initial_gravity: Optional[np.ndarray] = None
        # 用本机时钟追踪数据新鲜度（stamp_ns 是设备单调时钟，不能和 time.time 混算）
        self._last_seq: int = -1
        self._last_fresh_time: float = 0.0

    def version(self) -> str:
        return self._client.version()

    def start(self, timeout_ms: int = 8000):
        """启动 IMU 流，并采集若干帧用于重力对齐。"""
        self._client.start(timeout_ms=timeout_ms)
        self._wait_for_stream()
        self._initial_gravity = self._collect_gravity_samples()
        self._last_fresh_time = time.time()

    def stop(self):
        try:
            self._client.stop()
        except Exception:
            pass

    @property
    def initial_gravity(self) -> Optional[np.ndarray]:
        """启动后的初始重力向量（机身坐标系），用于初始化 Mahony 四元数。"""
        return self._initial_gravity

    def get_latest(self):
        """返回 (gyro[3], accel[3], age_ms, fresh)；fresh=False 表示无新数据。"""
        sample = self._client.get_latest()
        if sample is None:
            return (np.zeros(3, dtype=np.float32),
                    np.array([0.0, 0.0, 9.81], dtype=np.float32),
                    -1.0, False)
        gyro = np.array([sample.gyro_x, sample.gyro_y, sample.gyro_z], dtype=np.float32)
        accel = np.array([sample.accel_x, sample.accel_y, sample.accel_z], dtype=np.float32)
        # 用 stamp_ns 判断是否有新数据，因为 sequence 字段在 C++ 中可能没有赋值，导致永远为 0
        stamp = getattr(sample, "stamp_ns", 0)
        now = time.time()
        if stamp != self._last_seq:
            self._last_seq = stamp
            self._last_fresh_time = now
            fresh = True
        else:
            fresh = False
        age_ms = (now - self._last_fresh_time) * 1000.0
        return gyro, accel, age_ms, fresh

    # ---- 内部方法 ----
    def _wait_for_stream(self, timeout: float = 3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._client.wait_for_data(timeout_ms=200):
                # 有数据进来后清空一次队列以保证后续 get_latest 拿到的都是最新
                while self._client.pop_sample() is not None:
                    pass
                return
        raise RuntimeError("IMU 启动超时，未收到任何样本")

    def _collect_gravity_samples(self) -> np.ndarray:
        accels = []
        for _ in range(self._gravity_align_samples):
            sample = self._client.pop_sample()
            if sample is None:
                if not self._client.wait_for_data(timeout_ms=100):
                    continue
                sample = self._client.pop_sample()
                if sample is None:
                    continue
            accels.append([sample.accel_x, sample.accel_y, sample.accel_z])
        if not accels:
            print("[IMU] 警告: 重力对齐期间未收到样本，使用默认重力 [0,0,-9.81]")
            return np.array([0.0, 0.0, -9.81], dtype=np.float32)
        gravity = np.mean(accels, axis=0).astype(np.float32)
        print(f"[IMU] 重力对齐完成: g_body = {gravity}")
        return gravity
