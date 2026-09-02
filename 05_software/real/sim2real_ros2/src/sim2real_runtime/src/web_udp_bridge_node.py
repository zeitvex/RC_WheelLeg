#!/usr/bin/env python3
from __future__ import annotations

import json
import socket
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sim2real_interfaces.msg import RuntimeState, RuntimeTarget
from std_msgs.msg import Bool, String


class WebUdpBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("sim2real_web_udp_bridge_node", allow_undeclared_parameters=True)

        self.enabled = bool(self.declare_parameter("web_bridge_enabled", True).value)
        self.listen_host = str(self.declare_parameter("web_udp_listen_host", "0.0.0.0").value)
        self.listen_port = int(self.declare_parameter("web_udp_listen_port", 15000).value)
        self.remote_host = str(self.declare_parameter("web_udp_remote_host", "").value)
        self.remote_port = int(self.declare_parameter("web_udp_remote_port", 15001).value)
        self.state_hz = float(self.declare_parameter("web_udp_state_hz", 20.0).value)
        self.cmd_timeout_ms = float(self.declare_parameter("web_udp_cmd_timeout_ms", 300.0).value)
        self.max_packet_bytes = int(self.declare_parameter("web_udp_max_packet_bytes", 8192).value)
        self.max_vx = float(self.declare_parameter("web_udp_max_vx", 0.8).value)
        self.max_vy = float(self.declare_parameter("web_udp_max_vy", 0.3).value)
        self.max_yaw = float(self.declare_parameter("web_udp_max_yaw_rate", 0.5).value)
        self.estop_on_timeout = bool(self.declare_parameter("web_udp_estop_on_timeout", False).value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((self.listen_host, self.listen_port))

        self.client_addr: Optional[tuple[str, int]] = None
        if self.remote_host:
            self.client_addr = (self.remote_host, self.remote_port)

        self.latest_target: Optional[RuntimeTarget] = None
        self.latest_state: Optional[RuntimeState] = None
        self.latest_cmd = Twist()
        self.latest_mode = "UNKNOWN"
        self.latest_mux_status = ""
        self.estop = False
        self.web_enabled = False
        self.last_cmd_time = self.get_clock().now()
        self.timeout_estop_sent = False

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_web", 10)
        self.estop_pub = self.create_publisher(Bool, "/safety/estop", 10)
        self.web_enabled_pub = self.create_publisher(Bool, "web/enabled", 10)
        self.remote_enabled_pub = self.create_publisher(Bool, "remote/enabled", 10)
        self.nav_enabled_pub = self.create_publisher(Bool, "nav/enabled", 10)
        self.mode_pub = self.create_publisher(String, "control/mode", 10)

        self.create_subscription(RuntimeTarget, "runtime/target", self.on_target, 10)
        self.create_subscription(RuntimeState, "runtime/state", self.on_state, 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Bool, "/safety/estop", self.on_estop, 10)
        self.create_subscription(String, "control/mode_state", self.on_mode_state, 10)
        self.create_subscription(String, "control/mux_status", self.on_mux_status, 10)

        self.rx_timer = self.create_timer(0.01, self.on_rx_timer)
        self.state_timer = self.create_timer(1.0 / self.state_hz if self.state_hz > 0.0 else 0.05, self.on_state_timer)
        self.guard_timer = self.create_timer(0.05, self.on_guard_timer)

        self.get_logger().info(f"Web UDP bridge listening on {self.listen_host}:{self.listen_port}")

    def on_target(self, msg: RuntimeTarget) -> None:
        self.latest_target = msg

    def on_state(self, msg: RuntimeState) -> None:
        self.latest_state = msg

    def on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = msg

    def on_estop(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    def on_mode_state(self, msg: String) -> None:
        self.latest_mode = msg.data

    def on_mux_status(self, msg: String) -> None:
        self.latest_mux_status = msg.data

    def on_rx_timer(self) -> None:
        if not self.enabled:
            return
        while True:
            try:
                data, addr = self.sock.recvfrom(self.max_packet_bytes)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().warn(f"UDP receive failed: {exc}")
                break
            self.client_addr = addr
            try:
                payload = json.loads(data.decode("utf-8"))
                self.handle_packet(payload)
            except Exception as exc:
                self.send_packet({"type": "error", "message": str(exc)})

    def handle_packet(self, payload: dict[str, Any]) -> None:
        msg_type = str(payload.get("type", "")).lower()
        if msg_type == "cmd_vel":
            cmd = self.parse_twist(payload)
            self.cmd_pub.publish(cmd)
            self.last_cmd_time = self.get_clock().now()
            self.timeout_estop_sent = False
        elif msg_type == "zero":
            self.cmd_pub.publish(Twist())
            self.last_cmd_time = self.get_clock().now()
        elif msg_type == "estop":
            self.estop_pub.publish(Bool(data=bool(payload.get("data", True))))
        elif msg_type == "mode":
            mode = str(payload.get("mode", "DISABLED")).upper()
            self.mode_pub.publish(String(data=mode))
            self.web_enabled = mode == "WEB"
            self.web_enabled_pub.publish(Bool(data=mode == "WEB"))
            self.remote_enabled_pub.publish(Bool(data=mode == "REMOTE"))
            self.nav_enabled_pub.publish(Bool(data=mode == "NAV"))
        elif msg_type == "web_enable":
            self.web_enabled = bool(payload.get("data", False))
            self.web_enabled_pub.publish(Bool(data=self.web_enabled))
            if self.web_enabled:
                self.mode_pub.publish(String(data="WEB"))
        elif msg_type == "remote_enable":
            enabled = bool(payload.get("data", False))
            self.remote_enabled_pub.publish(Bool(data=enabled))
            if enabled:
                self.mode_pub.publish(String(data="REMOTE"))
        elif msg_type == "nav_enable":
            enabled = bool(payload.get("data", False))
            self.nav_enabled_pub.publish(Bool(data=enabled))
            if enabled:
                self.mode_pub.publish(String(data="NAV"))
        elif msg_type == "ping":
            self.send_packet({"type": "pong", "stamp": self.now_sec()})
        else:
            self.send_packet({"type": "error", "message": f"unknown packet type: {msg_type}"})

    def parse_twist(self, payload: dict[str, Any]) -> Twist:
        cmd = Twist()
        linear = payload.get("linear", {}) or {}
        angular = payload.get("angular", {}) or {}
        cmd.linear.x = self.clamp(float(linear.get("x", 0.0)), -self.max_vx, self.max_vx)
        cmd.linear.y = self.clamp(float(linear.get("y", 0.0)), -self.max_vy, self.max_vy)
        cmd.angular.z = self.clamp(float(angular.get("z", 0.0)), -self.max_yaw, self.max_yaw)
        return cmd

    def on_guard_timer(self) -> None:
        age_ms = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1.0e6
        if age_ms > self.cmd_timeout_ms:
            self.cmd_pub.publish(Twist())
            if self.estop_on_timeout and not self.timeout_estop_sent:
                self.estop_pub.publish(Bool(data=True))
                self.timeout_estop_sent = True

    def on_state_timer(self) -> None:
        if not self.enabled:
            return
        self.send_packet(self.build_state_packet())

    def build_state_packet(self) -> dict[str, Any]:
        target = self.latest_target
        state = self.latest_state
        packet: dict[str, Any] = {
            "type": "state",
            "stamp": self.now_sec(),
            "mode": self.latest_mode,
            "mux_status": self.latest_mux_status,
            "estop": self.estop,
            "web_enabled": self.web_enabled,
            "cmd_vel": self.twist_to_dict(self.latest_cmd),
            "runtime": {},
            "robot": {},
        }
        if target is not None:
            packet["runtime"] = {
                "target_source": str(target.target_source),
                "zero_command": bool(target.zero_command),
                "runtime_released": bool(target.runtime_released),
                "release_alpha": self._f(target.release_alpha),
                "command": [self._f(v) for v in target.command],
                "raw_command": [self._f(v) for v in target.raw_command],
            }
        if state is not None:
            packet["robot"] = {
                "joint_pos": [self._f(v) for v in state.joint_pos],
                "joint_vel": [self._f(v) for v in state.joint_vel],
                "joint_torque": [self._f(v) for v in state.joint_torque],
                "imu_gyro": [self._f(v) for v in state.imu_gyro],
                "imu_accel": [self._f(v) for v in state.imu_accel],
                "projected_gravity": [self._f(v) for v in state.projected_gravity],
                "quat_wxyz": [self._f(v) for v in state.quat_wxyz],
                "imu_age_ms": self._f(state.imu_age_ms),
                "imu_fresh": bool(state.imu_fresh),
                "odom_age_ms": self._f(state.odom_age_ms),
                "odom_fresh": bool(state.odom_fresh),
                "odom_local_pos": [self._f(v) for v in state.odom_local_pos],
                "odom_local_yaw": self._f(state.odom_local_yaw),
                "fresh_count": int(state.fresh_count),
                "holdover_count": int(state.holdover_count),
                "stale_max": int(state.stale_max),
                "update_counts": [int(v) for v in state.update_counts],
            }
        return packet

    def send_packet(self, payload: dict[str, Any]) -> None:
        if self.client_addr is None:
            return
        try:
            data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.sock.sendto(data, self.client_addr)
        except OSError as exc:
            self.get_logger().warn(f"UDP send failed: {exc}")

    @staticmethod
    def _f(v: Any) -> float:
        try:
            f = float(v)
            if f != f:
                return 0.0
            return round(f, 6)
        except (TypeError, ValueError):
            return 0.0

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def twist_to_dict(msg: Twist) -> dict[str, Any]:
        return {
            "linear": {"x": msg.linear.x, "y": msg.linear.y, "z": msg.linear.z},
            "angular": {"x": msg.angular.x, "y": msg.angular.y, "z": msg.angular.z},
        }

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WebUdpBridgeNode()
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
