#!/usr/bin/env python3
"""
Two-axis gimbal stabilisation and manual control.

Reads drone attitude from odometry and publishes roll/pitch joint commands
to keep the camera pointing at a fixed world angle.
"""

import math
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


def quat_to_roll_pitch(q: Any) -> tuple[float, float]:
    """Return (roll, pitch) in radians from a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w

    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    return roll, pitch


class LowPassFilter:
    """First-order low-pass filter for smoothing gimbal setpoints."""

    def __init__(self, cutoff_hz: float, dt: float):
        self._passthrough = cutoff_hz <= 0.0
        if not self._passthrough:
            rc = 1.0 / (2.0 * math.pi * cutoff_hz)
            self._alpha = dt / (rc + dt)
        else:
            self._alpha = 1.0
        self._state: float | None = None

    def update(self, x: float) -> float:
        if self._passthrough:
            return x
        if self._state is None:
            self._state = x  # initialise on first sample
        self._state += self._alpha * (x - self._state)
        return self._state

    def reset(self, value: float = 0.0) -> None:
        self._state = value


class GimbalController(Node):
    def __init__(self) -> None:
        super().__init__("gimbal_controller")

        self.declare_parameter("model_name", "x500_0")
        self.declare_parameter("default_pitch", -0.7854)  # -45 deg
        self.declare_parameter("stabilize", True)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("mode", "lock")
        self.declare_parameter("deadband", 0.0)
        self.declare_parameter("input_filter_hz", 0.0)
        self.declare_parameter("follow_smoothing", 0.0)

        p = self.get_parameter
        self._model = p("model_name").value
        self._def_pitch = p("default_pitch").value
        self._stabilize = p("stabilize").value
        rate = p("publish_rate").value
        self._mode = p("mode").value
        self._deadband = p("deadband").value
        input_filter_hz = p("input_filter_hz").value
        follow_smoothing = p("follow_smoothing").value

        dt = 1.0 / rate

        self._roll_setpt_filter = LowPassFilter(input_filter_hz, dt)
        self._pitch_setpt_filter = LowPassFilter(input_filter_hz, dt)

        if follow_smoothing > 0.0:
            self._smooth_alpha = dt / (follow_smoothing + dt)
        else:
            self._smooth_alpha = 1.0  # no smoothing, instant update

        self._out_roll = 0.0
        self._out_pitch = self._def_pitch

        self._drone_roll = 0.0
        self._drone_pitch = 0.0
        self._set_roll = 0.0
        self._set_pitch = self._def_pitch

        odom_topic = f"/model/{self._model}/odometry"
        pitch_topic = f"/model/{self._model}/command/gimbal_pitch"
        roll_topic = f"/model/{self._model}/command/gimbal_roll"

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(Float64, "/gimbal/set_pitch", self._pitch_cb, 10)
        self.create_subscription(Float64, "/gimbal/set_roll", self._roll_cb, 10)

        self._pub_roll = self.create_publisher(Float64, roll_topic, 10)
        self._pub_pitch = self.create_publisher(Float64, pitch_topic, 10)

        self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f"Gimbal controller started\n"
            f"  Model        : {self._model}\n"
            f"  Mode         : {self._mode}\n"
            f"  Stabilize    : {self._stabilize}\n"
            f"  Default pitch: {math.degrees(self._def_pitch):.1f} deg\n"
            f"  Deadband     : {math.degrees(self._deadband):.2f} deg\n"
            f"  Input filter : {input_filter_hz:.1f} Hz\n"
            f"  Smoothing    : {follow_smoothing:.2f} s\n"
            f"  Odometry     : {odom_topic}\n"
            f"  Pitch cmd    : {pitch_topic}\n"
            f"  Roll cmd     : {roll_topic}\n"
            f"  Set pitch    : /gimbal/set_pitch  (Float64, rad)\n"
            f"  Set roll     : /gimbal/set_roll   (Float64, rad)\n"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._drone_roll, self._drone_pitch = quat_to_roll_pitch(q)

    def _pitch_cb(self, msg: Float64) -> None:
        self._set_pitch = float(msg.data)
        self.get_logger().info(f"Gimbal pitch target: {math.degrees(self._set_pitch):.1f} deg")

    def _roll_cb(self, msg: Float64) -> None:
        self._set_roll = float(msg.data)
        self.get_logger().info(f"Gimbal roll target: {math.degrees(self._set_roll):.1f} deg")

    def _control_loop(self) -> None:
        roll_target = self._roll_setpt_filter.update(self._set_roll)
        pitch_target = self._pitch_setpt_filter.update(self._set_pitch)

        if self._stabilize:
            roll_cmd = roll_target - self._drone_roll
            pitch_cmd = pitch_target - self._drone_pitch
        else:
            roll_cmd = roll_target
            pitch_cmd = pitch_target

        if self._deadband > 0.0:
            if abs(roll_cmd) < self._deadband:
                roll_cmd = 0.0
            if abs(pitch_cmd) < self._deadband:
                pitch_cmd = 0.0

        self._out_roll += self._smooth_alpha * (roll_cmd - self._out_roll)
        self._out_pitch += self._smooth_alpha * (pitch_cmd - self._out_pitch)

        self._pub_roll.publish(Float64(data=self._out_roll))
        self._pub_pitch.publish(Float64(data=self._out_pitch))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GimbalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
