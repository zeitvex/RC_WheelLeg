from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import serial

SBUS_FRAME_SIZE = 25
SBUS_RC_MID = 1024
SBUS_AXIS_SCALE = 660.0

SWITCH_LOW = -1
SWITCH_MID = 0
SWITCH_HIGH = 1


@dataclass
class RemoteSwitchState:
    ch7: int = SWITCH_MID


@dataclass
class RemoteControlState:
    ch1: int = 0
    ch2: int = 0
    ch3: int = 0
    ch4: int = 0
    switches: RemoteSwitchState = field(default_factory=RemoteSwitchState)
    frame_ok: bool = False

    def active_axes(self, threshold: int = 50) -> dict[str, bool]:
        return {
            "ch1": abs(self.ch1) > threshold,
            "ch2": abs(self.ch2) > threshold,
            "ch3": abs(self.ch3) > threshold,
            "ch4": abs(self.ch4) > threshold,
        }

    @property
    def estop_requested(self) -> bool:
        return self.switches.ch7 == SWITCH_HIGH

    def as_dict(self) -> dict:
        return {
            "ch1": int(self.ch1),
            "ch2": int(self.ch2),
            "ch3": int(self.ch3),
            "ch4": int(self.ch4),
            "switches": {"ch7": int(self.switches.ch7)},
            "frame_ok": bool(self.frame_ok),
            "estop_requested": bool(self.estop_requested),
        }


class RemoteUartReceiver:
    def __init__(self, port: str, baudrate: int = 100000, timeout: float = 0.02, axis_deadzone: int = 50):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.axis_deadzone = int(axis_deadzone)
        self.serial: Optional[serial.Serial] = None
        self._buffer = bytearray()
        self.state = RemoteControlState()

    def open(self) -> None:
        if self.serial and self.serial.is_open:
            return
        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
        )

    def close(self) -> None:
        if self.serial and self.serial.is_open:
            self.serial.close()

    def poll(self) -> RemoteControlState:
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("remote uart is not open")

        waiting = self.serial.in_waiting
        if waiting:
            self._buffer.extend(self.serial.read(waiting))

        while len(self._buffer) >= SBUS_FRAME_SIZE:
            start_idx = self._buffer.find(0x0F)
            if start_idx < 0:
                self._buffer.clear()
                break
            if start_idx > 0:
                del self._buffer[:start_idx]
            if len(self._buffer) < SBUS_FRAME_SIZE:
                break
            frame = bytes(self._buffer[:SBUS_FRAME_SIZE])
            del self._buffer[:SBUS_FRAME_SIZE]
            parsed = self._parse_frame(frame)
            if parsed is not None:
                self.state = parsed
        return self.state

    def _parse_frame(self, frame: bytes) -> Optional[RemoteControlState]:
        if len(frame) != SBUS_FRAME_SIZE or frame[0] != 0x0F:
            return None

        channels = [0] * 16
        channels[0] = (frame[1] | (frame[2] << 8)) & 0x07FF
        channels[1] = ((frame[2] >> 3) | (frame[3] << 5)) & 0x07FF
        channels[2] = ((frame[3] >> 6) | (frame[4] << 2) | (frame[5] << 10)) & 0x07FF
        channels[3] = ((frame[5] >> 1) | (frame[6] << 7)) & 0x07FF
        channels[4] = ((frame[6] >> 4) | (frame[7] << 4)) & 0x07FF
        channels[5] = ((frame[7] >> 7) | (frame[8] << 1) | (frame[9] << 9)) & 0x07FF
        channels[6] = ((frame[9] >> 2) | (frame[10] << 6)) & 0x07FF
        channels[7] = ((frame[10] >> 5) | (frame[11] << 3)) & 0x07FF
        channels[8] = (frame[12] | (frame[13] << 8)) & 0x07FF
        channels[9] = ((frame[13] >> 3) | (frame[14] << 5)) & 0x07FF

        if channels[0] < 100:
            return None

        state = RemoteControlState(
            ch1=self._normalize_axis(channels[0]),
            ch2=self._normalize_axis(channels[1]),
            ch3=self._normalize_axis(channels[3]),
            ch4=self._normalize_axis(channels[2]),
            switches=RemoteSwitchState(ch7=self._decode_switch(channels[6])),
            frame_ok=True,
        )
        if any(abs(value) > 800 for value in (state.ch1, state.ch2, state.ch3, state.ch4)):
            return None
        return state

    def _normalize_axis(self, value: int) -> int:
        mapped = int(round((value - SBUS_RC_MID) * SBUS_AXIS_SCALE / 800.0))
        return 0 if abs(mapped) <= self.axis_deadzone else mapped

    @staticmethod
    def _decode_switch(value: int) -> int:
        if value < 500:
            return SWITCH_LOW
        if value > 1500:
            return SWITCH_HIGH
        return SWITCH_MID


class RemoteCommandMapper:
    def __init__(
        self,
        *,
        max_vx: float,
        max_vy: float,
        max_yaw: float,
        active_threshold: int = 50,
        axis_full_scale: float = SBUS_AXIS_SCALE,
        invert_vx: bool = False,
        invert_vy: bool = False,
        invert_yaw: bool = False,
    ):
        self.max_vx = float(max_vx)
        self.max_vy = float(max_vy)
        self.max_yaw = float(max_yaw)
        self.active_threshold = int(active_threshold)
        self.axis_full_scale = max(float(axis_full_scale), 1.0)
        self.invert_vx = bool(invert_vx)
        self.invert_vy = bool(invert_vy)
        self.invert_yaw = bool(invert_yaw)

    def map_command(self, state: RemoteControlState) -> np.ndarray:
        vx = self._axis_to_velocity(state.ch2, self.max_vx, self.invert_vx)
        vy = self._axis_to_velocity(state.ch4, self.max_vy, self.invert_vy)
        yaw = self._axis_to_velocity(state.ch1, self.max_yaw, self.invert_yaw)
        return np.array([vx, vy, yaw], dtype=np.float32)

    def is_command_active(self, state: RemoteControlState) -> bool:
        return any(abs(value) > self.active_threshold for value in (state.ch1, state.ch2, state.ch4))

    def _axis_to_velocity(self, raw_value: int, limit: float, invert: bool) -> float:
        if abs(raw_value) <= self.active_threshold:
            return 0.0
        scaled = max(-1.0, min(1.0, raw_value / self.axis_full_scale))
        if invert:
            scaled = -scaled
        return float(scaled * limit)


class RemoteCommandSource:
    def __init__(
        self,
        *,
        port: str,
        max_vx: float,
        max_vy: float,
        max_yaw: float,
        baudrate: int = 100000,
        timeout: float = 0.02,
        axis_deadzone: int = 50,
        active_threshold: int = 50,
        axis_full_scale: float = SBUS_AXIS_SCALE,
        invert_vx: bool = False,
        invert_vy: bool = False,
        invert_yaw: bool = False,
    ):
        self.receiver = RemoteUartReceiver(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            axis_deadzone=axis_deadzone,
        )
        self.mapper = RemoteCommandMapper(
            max_vx=max_vx,
            max_vy=max_vy,
            max_yaw=max_yaw,
            active_threshold=active_threshold,
            axis_full_scale=axis_full_scale,
            invert_vx=invert_vx,
            invert_vy=invert_vy,
            invert_yaw=invert_yaw,
        )
        self.last_state = RemoteControlState()
        self.last_command = np.zeros(3, dtype=np.float32)

    @property
    def port(self) -> str:
        return self.receiver.port

    def open(self) -> None:
        self.receiver.open()

    def close(self) -> None:
        self.receiver.close()

    def poll(self) -> RemoteControlState:
        self.last_state = self.receiver.poll()
        self.last_command = self.mapper.map_command(self.last_state)
        return self.last_state

    def get_command(self) -> np.ndarray:
        return self.last_command.copy()

    def is_command_active(self) -> bool:
        return self.mapper.is_command_active(self.last_state)

    def get_status(self) -> dict:
        status = self.last_state.as_dict()
        status.update(
            {
                "port": self.port,
                "cmd": self.last_command.tolist(),
                "command_active": bool(self.is_command_active()),
            }
        )
        return status
