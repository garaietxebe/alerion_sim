#!/usr/bin/env python3
"""
Dryden atmospheric turbulence node.

Injects wind noise into Gazebo by publishing directly to the world wind topic
via gz-transport Python bindings.

Also publishes two ROS 2 topics for monitoring and visualisation:

  /wind/vector  (geometry_msgs/Vector3Stamped)
      Instantaneous wind vector (mean + turbulence) in the world frame.
      Useful for data recording and analysis.

  /wind/marker  (visualization_msgs/Marker)
      Cyan arrow in RViz showing wind direction and magnitude at the drone's
      current position.  Length scales with speed (scale_factor m per m/s).
      The arrow "breathes" with the Gaussian turbulence — fast oscillations
      are Dryden turbulence, slow drift is the baseline wind.
"""

import math
import random

import gz.transport13 as gz_transport
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Vector3Stamped
from gz.msgs10.wind_pb2 import Wind
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker


# Arrow scale: metres of arrow length per m/s of wind speed.
# At the default mean wind of 2 m/s + intensity 0.8 the arrow is ~1 m long.
_SCALE_FACTOR = 0.4
# Shaft / head diameters
_SHAFT_D = 0.08
_HEAD_D  = 0.18
# Cyan colour — distinct from the velocity arrow (green/yellow/red)
_COLOR   = ColorRGBA(r=0.0, g=0.85, b=1.0, a=0.9)


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
        self.declare_parameter("odom_topic", "/model/x500_0/odometry")

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

        # latest drone position for marker placement (world frame)
        self._drone_pos: Point = Point(x=0.0, y=0.0, z=2.0)
        self._odom_frame: str = "x500_0/odom"

        # gz-transport publisher, kept alive for the lifetime of the node
        gz_topic = f"/world/{self._world}/wind"
        self._gz_node = gz_transport.Node()
        self._gz_pub = self._gz_node.advertise(gz_topic, Wind)

        # ROS 2 publishers
        self._vec_pub = self.create_publisher(Vector3Stamped, "/wind/vector", 10)
        self._marker_pub = self.create_publisher(Marker, "/wind/marker", 10)

        # subscribe to odometry to know where to draw the marker
        self.create_subscription(
            Odometry,
            p("odom_topic").value,
            self._odom_cb,
            10,
        )

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
            f"  RViz      : /wind/marker (cyan arrow at drone position)\n"
        )

    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        self._drone_pos = msg.pose.pose.position
        self._odom_frame = msg.header.frame_id

    def _dryden_step(self) -> tuple[float, float, float]:
        """Advance the filter one step and return (ux, uy, uz) turbulence."""
        for i in range(3):
            w = random.gauss(0.0, 1.0)
            self._u[i] = self._alpha * self._u[i] + self._beta * w
        return (self._u[0], self._u[1], self._u[2])

    def _tick(self) -> None:
        ux, uy, uz = self._dryden_step()

        wx = self._mean_x + ux
        wy = self._mean_y + uy
        wz = self._mean_z + uz
        speed = math.sqrt(wx * wx + wy * wy + wz * wz)

        now = self.get_clock().now().to_msg()

        # ── Gazebo wind ────────────────────────────────────────────────
        gz_msg = Wind()
        gz_msg.linear_velocity.x = wx
        gz_msg.linear_velocity.y = wy
        gz_msg.linear_velocity.z = wz
        self._gz_pub.publish(gz_msg)

        # ── ROS vector topic ───────────────────────────────────────────
        vec = Vector3Stamped()
        vec.header = Header(stamp=now, frame_id=self._odom_frame)
        vec.vector.x = wx
        vec.vector.y = wy
        vec.vector.z = wz
        self._vec_pub.publish(vec)

        # ── RViz marker: cyan arrow at drone position ──────────────────
        if speed < 0.01:
            return

        origin = self._drone_pos
        end = Point(
            x=origin.x + wx * _SCALE_FACTOR,
            y=origin.y + wy * _SCALE_FACTOR,
            z=origin.z + wz * _SCALE_FACTOR,
        )

        m = Marker()
        m.header = Header(stamp=now, frame_id=self._odom_frame)
        m.ns = "wind"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.points = [origin, end]
        m.scale.x = _SHAFT_D
        m.scale.y = _HEAD_D
        m.scale.z = 0.0
        m.color = _COLOR
        m.lifetime = Duration(sec=0, nanosec=300_000_000)   # 0.3 s auto-hide
        self._marker_pub.publish(m)


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
