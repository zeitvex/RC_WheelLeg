#!/usr/bin/python3
"""ODIN1 IMU ctypes 封装."""

from __future__ import annotations

import ctypes
from pathlib import Path
from typing import Iterator, Optional


class Odin1ImuSample(ctypes.Structure):
    """输入: 无; 输出: Odin1ImuSample; 作用: 映射 C++ bridge 的 IMU 结构体."""

    _fields_ = [
        ("accel_x", ctypes.c_float),
        ("accel_y", ctypes.c_float),
        ("accel_z", ctypes.c_float),
        ("gyro_x", ctypes.c_float),
        ("gyro_y", ctypes.c_float),
        ("gyro_z", ctypes.c_float),
        ("stamp_ns", ctypes.c_uint64),
        ("sequence", ctypes.c_uint64),
    ]


class Odin1ImuClient:
    """输入: lib_path[Optional[str|Path]]; 输出: Odin1ImuClient; 作用: 提供 Python 对 ODIN1 IMU bridge 的访问接口."""

    def __init__(self, lib_path: Optional[str | Path] = None) -> None:
        self._project_root = Path(__file__).resolve().parents[1]
        resolved_path = Path(lib_path) if lib_path else self._project_root / "build" / "libodin1_imu_bridge.so"
        self._lib = ctypes.CDLL(str(resolved_path))
        self._configure_signatures()

    def _configure_signatures(self) -> None:
        """输入: 无; 输出: 无; 作用: 配置 ctypes 函数签名."""

        self._lib.odin1_imu_version.restype = ctypes.c_char_p

        self._lib.odin1_imu_start.argtypes = [ctypes.c_int]
        self._lib.odin1_imu_start.restype = ctypes.c_int

        self._lib.odin1_imu_stop.argtypes = []
        self._lib.odin1_imu_stop.restype = None

        self._lib.odin1_imu_is_running.argtypes = []
        self._lib.odin1_imu_is_running.restype = ctypes.c_int

        self._lib.odin1_imu_wait_for_data.argtypes = [ctypes.c_int]
        self._lib.odin1_imu_wait_for_data.restype = ctypes.c_int

        self._lib.odin1_imu_pop_sample.argtypes = [ctypes.POINTER(Odin1ImuSample)]
        self._lib.odin1_imu_pop_sample.restype = ctypes.c_int

        self._lib.odin1_imu_get_latest.argtypes = [ctypes.POINTER(Odin1ImuSample)]
        self._lib.odin1_imu_get_latest.restype = ctypes.c_int

        self._lib.odin1_imu_last_error.argtypes = []
        self._lib.odin1_imu_last_error.restype = ctypes.c_char_p

    def version(self) -> str:
        """输入: 无; 输出: str; 作用: 获取 C++ bridge 版本号."""

        return self._lib.odin1_imu_version().decode("utf-8")

    def last_error(self) -> str:
        """输入: 无; 输出: str; 作用: 获取最近一次 bridge 错误信息."""

        return self._lib.odin1_imu_last_error().decode("utf-8")

    def start(self, timeout_ms: int = 5000) -> None:
        """输入: timeout_ms[int]; 输出: 无; 作用: 启动 IMU 数据接收."""

        result = self._lib.odin1_imu_start(timeout_ms)
        if result != 0:
            raise RuntimeError(f"启动 ODIN1 IMU 失败: {self.last_error()} (code={result})")

    def stop(self) -> None:
        """输入: 无; 输出: 无; 作用: 停止 IMU 数据接收."""

        self._lib.odin1_imu_stop()

    def is_running(self) -> bool:
        """输入: 无; 输出: bool; 作用: 返回 bridge 是否仍在运行."""

        return bool(self._lib.odin1_imu_is_running())

    def wait_for_data(self, timeout_ms: int = 1000) -> bool:
        """输入: timeout_ms[int]; 输出: bool; 作用: 等待 IMU 数据到达."""

        result = self._lib.odin1_imu_wait_for_data(timeout_ms)
        if result < 0:
            raise RuntimeError(f"等待 IMU 数据失败: {self.last_error()} (code={result})")
        return bool(result)

    def pop_sample(self) -> Optional[Odin1ImuSample]:
        """输入: 无; 输出: Optional[Odin1ImuSample]; 作用: 从队列中取出一帧 IMU 数据."""

        sample = Odin1ImuSample()
        result = self._lib.odin1_imu_pop_sample(ctypes.byref(sample))
        if result < 0:
            raise RuntimeError(f"读取 IMU 队列失败: {self.last_error()} (code={result})")
        return sample if result == 1 else None

    def get_latest(self) -> Optional[Odin1ImuSample]:
        """输入: 无; 输出: Optional[Odin1ImuSample]; 作用: 获取最近一帧 IMU 数据."""

        sample = Odin1ImuSample()
        result = self._lib.odin1_imu_get_latest(ctypes.byref(sample))
        if result < 0:
            raise RuntimeError(f"读取最新 IMU 数据失败: {self.last_error()} (code={result})")
        return sample if result == 1 else None

    def iter_samples(self, timeout_ms: int = 1000) -> Iterator[Odin1ImuSample]:
        """输入: timeout_ms[int]; 输出: Iterator[Odin1ImuSample]; 作用: 连续迭代输出 IMU 数据."""

        while self.is_running():
            if not self.wait_for_data(timeout_ms):
                continue
            while True:
                sample = self.pop_sample()
                if sample is None:
                    break
                yield sample
