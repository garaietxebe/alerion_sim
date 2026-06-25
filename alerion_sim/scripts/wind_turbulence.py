#!/usr/bin/env python3
"""
Dryden atmospheric turbulence node.

Injects wind noise into Gazebo by publishing directly to the world wind topic
via gz-transport Python bindings.
"""

import math
import random

import gz.transport13 as gz_transport
import rclpy
from gz.msgs10.wind_pb2 import Wind
from rclpy.node import Node


class WindTurbulenceNode(Node):
    def __init__(self) -> None:
        super().__init__("wind_turbulence")

        self.declare_parameter("mean_x", 0.0)
        self.declare_parameter("mean_y", 0.0)
        self.declare_parameter("mean_z", 0.0)
        self.declare_parameter("intensity", 0.8)
        self.declare_parameter("correlation_time", 3.0)
        self.declare_parameter("update_rate", 10.0)
        self.declare_parameter("world_name", "inspection")

        p = self.get_parameter
        self._mean_x = p("mean_x").value
        self._mean_y = p("mean_y").value
        self._mean_z = p("mean_z").value
        sigma = p("intensity").value
        tau = p("correlation_time").value
        rate = p("update_rate").value
        self._world = p("world_name").value

        dt = 1.0 / rate

        # discrete Dryden filter coefficients, clamped to [0, 1] for numerical stability
        self._alpha = max(0.0, min(1.0, 1.0 - dt / tau))
        self._beta = sigma * math.sqrt(2.0 * dt / tau)

        # per-axis turbulence state (zero-mean)
        self._u = [0.0, 0.0, 0.0]

        # gz-transport publisher, kept alive for the lifetime of the node
        gz_topic = f"/world/{self._world}/wind"
        self._gz_node = gz_transport.Node()
        self._pub = self._gz_node.advertise(gz_topic, Wind)

        self.create_timer(dt, self._tick)

        self.get_logger().info(
            f"Wind turbulence started\n"
            f"  Mean wind : ({self._mean_x:.2f}, {self._mean_y:.2f}, "
            f"{self._mean_z:.2f}) m/s\n"
            f"  Intensity : sigma = {sigma:.2f} m/s per axis\n"
            f"  Corr. time: tau = {tau:.1f} s  "
            f"(alpha={self._alpha:.4f}, beta={self._beta:.4f})\n"
            f"  Rate      : {rate:.1f} Hz\n"
            f"  Gz topic  : {gz_topic}\n"
        )

    def _dryden_step(self) -> tuple[float, float, float]:
        """Advance the filter one step and return (ux, uy, uz) turbulence."""
        for i in range(3):
            w = random.gauss(0.0, 1.0)
            self._u[i] = self._alpha * self._u[i] + self._beta * w
        return (self._u[0], self._u[1], self._u[2])

    def _tick(self) -> None:
        ux, uy, uz = self._dryden_step()

        msg = Wind()
        msg.linear_velocity.x = self._mean_x + ux
        msg.linear_velocity.y = self._mean_y + uy
        msg.linear_velocity.z = self._mean_z + uz

        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WindTurbulenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
