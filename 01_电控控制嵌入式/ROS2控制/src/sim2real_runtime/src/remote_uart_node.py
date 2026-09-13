#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import serial

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

SBUS_FRAME_SIZE = 25
SBUS_RC_MID = 1024
SBUS_AXIS_SCALE = 660.0

SWITCH_LOW = -1
SWITCH_MID = 0
SWITCH_HIGH = 1


@dataclass
class RemoteSwitchState:
    ch5: int = SWITCH_MID
    ch6: int = SWITCH_MID
    ch7: int = SWITCH_MID
    ch8: int = SWITCH_MID
    ch9: int = SWITCH_MID
    ch10: int = SWITCH_MID

    def get(self, channel: int) -> Optional[int]:
        return {
            5: self.ch5,
            6: self.ch6,
            7: self.ch7,
            8: self.ch8,
            9: self.ch9,
            10: self.ch10,
        }.get(int(channel))


@dataclass
class RemoteControlState:
    ch1: int = 0
    ch2: int = 0
    ch3: int = 0
    ch4: int = 0
    switches: RemoteSwitchState = field(default_factory=RemoteSwitchState)
    frame_ok: bool = False

    @property
    def estop_requested(self) -> bool:
        return self.switches.ch7 == SWITCH_HIGH


class SbusUartReceiver:
    def __init__(self, port: str, baudrate: int, timeout: float, axis_deadzone: int):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.axis_deadzone = int(axis_deadzone)
        self.serial: Optional[serial.Serial] = None
        self.buffer = bytearray()
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
            self.buffer.extend(self.serial.read(waiting))

        while len(self.buffer) >= SBUS_FRAME_SIZE:
            start_idx = self.buffer.find(0x0F)
            if start_idx < 0:
                self.buffer.clear()
                break
            if start_idx > 0:
                del self.buffer[:start_idx]
            if len(self.buffer) < SBUS_FRAME_SIZE:
                break
            frame = bytes(self.buffer[:SBUS_FRAME_SIZE])
            del self.buffer[:SBUS_FRAME_SIZE]
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
            switches=RemoteSwitchState(
                ch5=self._decode_switch(channels[4]),
                ch6=self._decode_switch(channels[5]),
                ch7=self._decode_switch(channels[6]),
                ch8=self._decode_switch(channels[7]),
                ch9=self._decode_switch(channels[8]),
                ch10=self._decode_switch(channels[9]),
            ),
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


class RemoteUartNode(Node):
    def __init__(self) -> None:
        super().__init__("sim2real_remote_uart_node", allow_undeclared_parameters=True)

        self.enabled = bool(self.declare_parameter("remote_enabled", True).value)
        self.port = str(self.declare_parameter("remote_port", "/dev/ttyACM0").value)
        self.baudrate = int(self.declare_parameter("remote_baudrate", 100000).value)
        self.timeout = float(self.declare_parameter("remote_timeout", 0.02).value)
        self.axis_deadzone = int(self.declare_parameter("remote_axis_deadzone", 40).value)
        self.active_threshold = int(self.declare_parameter("remote_active_threshold", 40).value)
        self.axis_full_scale = max(float(self.declare_parameter("remote_axis_full_scale", 660.0).value), 1.0)
        self.max_vx = float(self.declare_parameter("remote_max_vx", 0.8).value)
        self.max_vy = float(self.declare_parameter("remote_max_vy", 0.3).value)
        self.max_yaw = float(self.declare_parameter("remote_max_yaw_rate", 0.5).value)
        self.invert_vx = bool(self.declare_parameter("remote_invert_vx", True).value)
        self.invert_vy = bool(self.declare_parameter("remote_invert_vy", False).value)
        self.invert_yaw = bool(self.declare_parameter("remote_invert_yaw", True).value)
        self.publish_inactive_zero = bool(self.declare_parameter("remote_publish_inactive_zero", True).value)
        self.estop_latch = bool(self.declare_parameter("remote_estop_latch", True).value)
        self.poll_hz = float(self.declare_parameter("remote_poll_hz", 50.0).value)
        self.default_mode = str(self.declare_parameter("cmd_mux_default_mode", "REMOTE").value).strip().upper()
        self.model_switch_enabled = bool(self.declare_parameter("remote_model_switch_enabled", True).value)
        self.model_switch_channel = int(self.declare_parameter("remote_model_switch_channel", 10).value)
        self.model_switch_debounce_frames = max(int(self.declare_parameter("remote_model_switch_debounce_frames", 3).value), 1)
        self.model_switch_rough_level = self.parse_switch_level(
            str(self.declare_parameter("remote_model_switch_rough_level", "low").value)
        )
        legacy_ik_level = str(self.declare_parameter("remote_model_switch_crawl_level", "").value).strip()
        ik_level_default = legacy_ik_level if legacy_ik_level else "high"
        self.model_switch_ik_level = self.parse_switch_level(
            str(self.declare_parameter("remote_model_switch_ik_level", ik_level_default).value)
        )

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_remote", 10)
        self.estop_pub = self.create_publisher(Bool, "/safety/estop", 10)
        self.model_cmd_pub = self.create_publisher(String, "runtime/model_cmd", 10)
        self.receiver: Optional[SbusUartReceiver] = None
        self.estop_published = False
        self.open_error_logged = False
        self.remote_mode_active = self.default_mode == "REMOTE"
        self.model_switch_candidate: Optional[int] = None
        self.model_switch_candidate_count = 0
        self.model_switch_stable: Optional[int] = None

        self.create_subscription(String, "control/mode_state", self.on_mode_state, 10)

        if self.enabled:
            self.receiver = SbusUartReceiver(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                axis_deadzone=self.axis_deadzone,
            )
            try:
                self.receiver.open()
                self.get_logger().info(f"Remote UART opened on {self.port} at {self.baudrate} baud")
            except Exception as exc:
                self.get_logger().error(f"Failed to open remote UART {self.port}: {exc}")
                self.open_error_logged = True
        else:
            self.get_logger().warn("Remote UART node is disabled by parameter")

        period = 1.0 / self.poll_hz if self.poll_hz > 0.0 else 0.02
        self.timer = self.create_timer(period, self.on_timer)

    def destroy_node(self) -> bool:
        if self.receiver is not None:
            self.receiver.close()
        return super().destroy_node()

    def on_timer(self) -> None:
        if not self.enabled or self.receiver is None:
            return

        try:
            if not self.receiver.serial or not self.receiver.serial.is_open:
                self.receiver.open()
            state = self.receiver.poll()
        except Exception as exc:
            if not self.open_error_logged:
                self.get_logger().error(f"Remote UART poll failed: {exc}")
                self.open_error_logged = True
            return

        self.open_error_logged = False

        if state.estop_requested:
            if not self.estop_published or not self.estop_latch:
                self.estop_pub.publish(Bool(data=True))
                self.get_logger().warn("Remote E-stop requested by CH7 high")
                self.estop_published = True
            self.publish_zero_cmd()
            return

        if not self.estop_latch and self.estop_published:
            self.estop_pub.publish(Bool(data=False))
            self.estop_published = False

        self.handle_model_switch(state)

        active = any(abs(value) > self.active_threshold for value in (state.ch1, state.ch2, state.ch4))
        if active or self.publish_inactive_zero:
            cmd = Twist()
            cmd.linear.x = self.axis_to_velocity(state.ch2, self.max_vx, self.invert_vx)
            cmd.linear.y = self.axis_to_velocity(state.ch4, self.max_vy, self.invert_vy)
            cmd.angular.z = self.axis_to_velocity(state.ch1, self.max_yaw, self.invert_yaw)
            self.cmd_pub.publish(cmd)

    def publish_zero_cmd(self) -> None:
        self.cmd_pub.publish(Twist())

    def on_mode_state(self, msg: String) -> None:
        mode = str(msg.data).strip().upper()
        remote_mode_active = mode == "REMOTE"
        if remote_mode_active == self.remote_mode_active:
            return

        self.remote_mode_active = remote_mode_active
        self.reset_model_switch_tracking()

    def axis_to_velocity(self, raw_value: int, limit: float, invert: bool) -> float:
        if abs(raw_value) <= self.active_threshold:
            return 0.0
        scaled = max(-1.0, min(1.0, raw_value / self.axis_full_scale))
        if invert:
            scaled = -scaled
        return float(scaled * limit)

    @staticmethod
    def parse_switch_level(value: str) -> int:
        normalized = value.strip().lower()
        if normalized == "low":
            return SWITCH_LOW
        if normalized == "high":
            return SWITCH_HIGH
        return SWITCH_MID

    def handle_model_switch(self, state: RemoteControlState) -> None:
        if not self.model_switch_enabled or not self.remote_mode_active:
            return

        switch_level = state.switches.get(self.model_switch_channel)
        if switch_level is None:
            return

        if switch_level == self.model_switch_candidate:
            self.model_switch_candidate_count += 1
        else:
            self.model_switch_candidate = switch_level
            self.model_switch_candidate_count = 1

        if self.model_switch_candidate_count < self.model_switch_debounce_frames:
            return

        if switch_level == self.model_switch_stable:
            return

        self.model_switch_stable = switch_level

        if switch_level == self.model_switch_rough_level:
            self.model_cmd_pub.publish(String(data="rough"))
            self.get_logger().info(
                f"Remote model switch: CH{self.model_switch_channel} -> rough"
            )
        elif switch_level == self.model_switch_ik_level:
            self.model_cmd_pub.publish(String(data="ik"))
            self.get_logger().info(
                f"Remote model switch: CH{self.model_switch_channel} -> ik"
            )

    def reset_model_switch_tracking(self) -> None:
        self.model_switch_candidate = None
        self.model_switch_candidate_count = 0
        self.model_switch_stable = None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RemoteUartNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
