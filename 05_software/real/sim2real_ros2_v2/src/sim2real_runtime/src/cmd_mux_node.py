#!/usr/bin/env python3
from __future__ import annotations

from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String


class ControlMode(str, Enum):
    DISABLED = "DISABLED"
    REMOTE = "REMOTE"
    WEB = "WEB"
    NAV = "NAV"


class CmdMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("sim2real_cmd_mux_node", allow_undeclared_parameters=True)

        self.default_mode = str(self.declare_parameter("cmd_mux_default_mode", "REMOTE").value).upper()
        self.output_hz = float(self.declare_parameter("cmd_mux_output_hz", 50.0).value)
        self.remote_timeout_ms = float(self.declare_parameter("cmd_mux_remote_timeout_ms", 250.0).value)
        self.web_timeout_ms = float(self.declare_parameter("cmd_mux_web_timeout_ms", 300.0).value)
        self.nav_timeout_ms = float(self.declare_parameter("cmd_mux_nav_timeout_ms", 500.0).value)
        self.max_vx = float(self.declare_parameter("cmd_mux_max_vx", 0.8).value)
        self.max_vy = float(self.declare_parameter("cmd_mux_max_vy", 0.3).value)
        self.max_yaw = float(self.declare_parameter("cmd_mux_max_yaw_rate", 0.5).value)
        self.max_vx_acc = float(self.declare_parameter("cmd_mux_max_vx_acc", 1.0).value)
        self.max_vy_acc = float(self.declare_parameter("cmd_mux_max_vy_acc", 1.0).value)
        self.max_yaw_acc = float(self.declare_parameter("cmd_mux_max_yaw_acc", 1.5).value)

        self.mode = self.parse_mode(self.default_mode)
        self.estop = False
        self.remote_enabled = self.mode == ControlMode.REMOTE
        self.web_enabled = self.mode == ControlMode.WEB
        self.nav_enabled = self.mode == ControlMode.NAV

        self.latest_remote = Twist()
        self.latest_web = Twist()
        self.latest_nav = Twist()
        self.remote_stamp: Optional[rclpy.time.Time] = None
        self.web_stamp: Optional[rclpy.time.Time] = None
        self.nav_stamp: Optional[rclpy.time.Time] = None
        self.last_output = Twist()
        self.last_pub_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, "control/mode_state", 10)
        self.status_pub = self.create_publisher(String, "control/mux_status", 10)

        self.create_subscription(Twist, "cmd_vel_remote", self.on_remote, 10)
        self.create_subscription(Twist, "cmd_vel_web", self.on_web, 10)
        self.create_subscription(Twist, "cmd_vel_nav", self.on_nav, 10)
        self.create_subscription(String, "control/mode", self.on_mode, 10)
        self.create_subscription(Bool, "remote/enabled", self.on_remote_enabled, 10)
        self.create_subscription(Bool, "web/enabled", self.on_web_enabled, 10)
        self.create_subscription(Bool, "nav/enabled", self.on_nav_enabled, 10)
        self.create_subscription(Bool, "/safety/estop", self.on_estop, 10)

        period = 1.0 / self.output_hz if self.output_hz > 0.0 else 0.02
        self.timer = self.create_timer(period, self.on_timer)
        self.get_logger().info(f"Command mux started in mode {self.mode.value}")

    def parse_mode(self, value: str) -> ControlMode:
        try:
            return ControlMode(value.upper())
        except ValueError:
            self.get_logger().warn(f"Unknown control mode '{value}', using DISABLED")
            return ControlMode.DISABLED

    def on_remote(self, msg: Twist) -> None:
        self.latest_remote = msg
        self.remote_stamp = self.get_clock().now()

    def on_web(self, msg: Twist) -> None:
        self.latest_web = msg
        self.web_stamp = self.get_clock().now()

    def on_nav(self, msg: Twist) -> None:
        self.latest_nav = msg
        self.nav_stamp = self.get_clock().now()

    def on_mode(self, msg: String) -> None:
        new_mode = self.parse_mode(msg.data)
        if new_mode != self.mode:
            self.mode = new_mode
            self.remote_enabled = self.mode == ControlMode.REMOTE
            self.web_enabled = self.mode == ControlMode.WEB
            self.nav_enabled = self.mode == ControlMode.NAV
            self.get_logger().info(f"Control mode changed to {self.mode.value}")

    def on_remote_enabled(self, msg: Bool) -> None:
        self.remote_enabled = bool(msg.data)
        if self.remote_enabled:
            self.mode = ControlMode.REMOTE

    def on_web_enabled(self, msg: Bool) -> None:
        self.web_enabled = bool(msg.data)
        if self.web_enabled:
            self.mode = ControlMode.WEB

    def on_nav_enabled(self, msg: Bool) -> None:
        self.nav_enabled = bool(msg.data)
        if self.nav_enabled:
            self.mode = ControlMode.NAV

    def on_estop(self, msg: Bool) -> None:
        self.estop = bool(msg.data)
        if self.estop:
            self.mode = ControlMode.DISABLED

    def on_timer(self) -> None:
        now = self.get_clock().now()
        target = Twist()
        source = "zero"

        if not self.estop:
            if self.mode == ControlMode.REMOTE and self.remote_enabled and self.is_fresh(self.remote_stamp, self.remote_timeout_ms, now):
                target = self.latest_remote
                source = "remote"
            elif self.mode == ControlMode.WEB and self.web_enabled and self.is_fresh(self.web_stamp, self.web_timeout_ms, now):
                target = self.latest_web
                source = "web"
            elif self.mode == ControlMode.NAV and self.nav_enabled and self.is_fresh(self.nav_stamp, self.nav_timeout_ms, now):
                target = self.latest_nav
                source = "nav"

        target = self.limit_twist(target)
        target = self.accel_limit(target, now)
        self.cmd_pub.publish(target)
        self.mode_pub.publish(String(data=self.mode.value))
        self.status_pub.publish(String(data=f"mode={self.mode.value},source={source},estop={self.estop}"))

    def is_fresh(self, stamp: Optional[rclpy.time.Time], timeout_ms: float, now: rclpy.time.Time) -> bool:
        if stamp is None:
            return False
        age_ms = (now - stamp).nanoseconds / 1.0e6
        return age_ms <= timeout_ms

    def limit_twist(self, msg: Twist) -> Twist:
        out = Twist()
        out.linear.x = self.clamp(msg.linear.x, -self.max_vx, self.max_vx)
        out.linear.y = self.clamp(msg.linear.y, -self.max_vy, self.max_vy)
        out.angular.z = self.clamp(msg.angular.z, -self.max_yaw, self.max_yaw)
        return out

    def accel_limit(self, target: Twist, now: rclpy.time.Time) -> Twist:
        dt = max((now - self.last_pub_time).nanoseconds / 1.0e9, 1.0e-3)
        out = Twist()
        out.linear.x = self.step(self.last_output.linear.x, target.linear.x, self.max_vx_acc * dt)
        out.linear.y = self.step(self.last_output.linear.y, target.linear.y, self.max_vy_acc * dt)
        out.angular.z = self.step(self.last_output.angular.z, target.angular.z, self.max_yaw_acc * dt)
        self.last_output = out
        self.last_pub_time = now
        return out

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, float(value)))

    @staticmethod
    def step(current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CmdMuxNode()
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
