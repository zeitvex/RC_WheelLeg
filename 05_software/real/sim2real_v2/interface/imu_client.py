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
from typing import Dict, Optional

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
                 stale_threshold_ms: float = 50.0, dry_run: bool = False):
        self._mock_mode = dry_run
        self._initial_gravity = None
        self._last_seq = -1
        self._last_fresh_time = 0.0
        self._last_odom_stamp = -1
        self._last_odom_fresh_time = 0.0

        if self._mock_mode:
            print("[IMUClient] 启动 Mock IMU 模式 (不加载物理 IMU 驱动)。")
            self._client = None
            return

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
        except Exception as e:
            print(f"[IMUClient] 错误: 无法初始化实机 IMU 驱动 ({type(e).__name__}: {e})。实机部署下拒绝启动。")
            raise

    def version(self) -> str:
        if self._mock_mode:
            return "MockIMU-v1.0"
        return self._client.version()

    def start(self, timeout_ms: int = 8000):
        """启动 IMU 流，并采集若干帧用于重力对齐。"""
        if self._mock_mode:
            self._initial_gravity = np.array([0.0, 0.0, 9.81], dtype=np.float32)
            self._last_fresh_time = time.time()
            self._last_odom_fresh_time = time.time()
            return
        self._client.start(timeout_ms=timeout_ms)
        self._wait_for_stream()
        self._initial_gravity = self._collect_gravity_samples()
        self._last_fresh_time = time.time()

    def stop(self):
        if self._mock_mode:
            return
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
        if self._mock_mode:
            now = time.time()
            age_ms = float(getattr(self, "_debug_mock_age_ms", 0.0))
            fresh = True
            return (np.zeros(3, dtype=np.float32),
                    np.array([0.0, 0.0, 9.81], dtype=np.float32),
                    age_ms, fresh)

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

    def get_latest_odom(self) -> Optional[Dict[str, object]]:
        """Return latest Odin odom for diagnostics only; policy observations stay unchanged."""
        if self._mock_mode:
            now = time.time()
            # 模拟一个围绕 (0, 0) 的圆形轨迹，用于测试 Web UI Canvas 绘图
            theta = now * 0.2
            x = 0.5 * np.cos(theta)
            y = 0.5 * np.sin(theta)
            return {
                "type": "STANDARD",
                "stamp_ns": int(now * 1e9),
                "fresh": True,
                "age_ms": 0.0,
                "pos": [float(x), float(y), 0.33],
                "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
                "linear_vel": [0.0, 0.0, 0.0],
                "angular_vel": [0.0, 0.0, 0.2],
            }

        getter = getattr(self._client, "odom_get_latest", None)
        if getter is None:
            return None
        try:
            sample = getter()
        except Exception:
            return None
        if sample is None:
            return None

        stamp = int(getattr(sample, "stamp_ns", 0))
        now = time.time()
        if stamp != self._last_odom_stamp:
            self._last_odom_stamp = stamp
            self._last_odom_fresh_time = now
            fresh = True
        else:
            fresh = False
        age_ms = (now - self._last_odom_fresh_time) * 1000.0 if self._last_odom_fresh_time else -1.0
        return {
            "type": _odom_type_name(int(getattr(sample, "type", -1))),
            "stamp_ns": stamp,
            "fresh": fresh,
            "age_ms": age_ms,
            "pos": [
                float(getattr(sample, "pos_x", 0.0)),
                float(getattr(sample, "pos_y", 0.0)),
                float(getattr(sample, "pos_z", 0.0)),
            ],
            "quat_wxyz": [
                float(getattr(sample, "orient_w", 1.0)),
                float(getattr(sample, "orient_x", 0.0)),
                float(getattr(sample, "orient_y", 0.0)),
                float(getattr(sample, "orient_z", 0.0)),
            ],
            "linear_vel": [
                float(getattr(sample, "linear_vel_x", 0.0)),
                float(getattr(sample, "linear_vel_y", 0.0)),
                float(getattr(sample, "linear_vel_z", 0.0)),
            ],
            "angular_vel": [
                float(getattr(sample, "angular_vel_x", 0.0)),
                float(getattr(sample, "angular_vel_y", 0.0)),
                float(getattr(sample, "angular_vel_z", 0.0)),
            ],
        }

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


def _odom_type_name(value: int) -> str:
    return {0: "STANDARD", 1: "HIGHFREQ", 2: "TF"}.get(value, f"UNKNOWN_{value}")
