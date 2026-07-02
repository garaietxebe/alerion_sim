#!/usr/bin/env python3
"""
Odometry → velocity arrow marker.

Reads nav_msgs/Odometry, rotates the body-frame twist.linear into the
world frame using the drone's orientation quaternion, then publishes a
visualization_msgs/Marker (ARROW) on /drone/velocity_marker.

The arrow:
  - starts at the drone's current position
  - points in the world-frame velocity direction
  - length is proportional to speed (1 m/s → scale_factor metres)
  - colour transitions green → yellow → red as speed rises to max_speed

Parameters
----------
topic        : odometry input topic  (default: /model/x500_0/odometry)
scale_factor : arrow length per m/s  (default: 0.4  → 5 m/s = 2 m arrow)
max_speed    : speed at which colour is fully red, in m/s (default: 5.0)
min_speed    : speeds below this are not published (noise floor, default: 0.05)
"""

import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def _speed_color(speed: float, max_speed: float) -> ColorRGBA:
    """
    Green (0 m/s) → Yellow (max/2) → Red (max_speed).
    Returns a ColorRGBA with alpha=1.
    """
    t = min(speed / max(max_speed, 1e-6), 1.0)   # 0 → 1
    c = ColorRGBA()
    c.a = 1.0
    if t < 0.5:
        # green → yellow
        c.r = t * 2.0
        c.g = 1.0
        c.b = 0.0
    else:
        # yellow → red
        c.r = 1.0
        c.g = 1.0 - (t - 0.5) * 2.0
        c.b = 0.0
    return c


class OdomToVelMarker(Node):
    def __init__(self) -> None:
        super().__init__("odom_to_vel_marker")

        self.declare_parameter("topic", "/model/x500_0/odometry")
        self.declare_parameter("scale_factor", 0.4)
        self.declare_parameter("max_speed", 5.0)
        self.declare_parameter("min_speed", 0.05)

        self._topic = self.get_parameter("topic").value
        self._scale = float(self.get_parameter("scale_factor").value)
        self._max_spd = float(self.get_parameter("max_speed").value)
        self._min_spd = float(self.get_parameter("min_speed").value)

        self._pub = self.create_publisher(Marker, "/drone/velocity_marker", 10)
        self.create_subscription(Odometry, self._topic, self._cb, 10)
        self.get_logger().info(
            f"Publishing velocity arrow on /drone/velocity_marker  "
            f"(scale {self._scale} m per m/s, max colour speed {self._max_spd} m/s)"
        )

    def _cb(self, msg: Odometry) -> None:
        # Body-frame velocity from the twist field (already in child_frame_id)
        bvx = msg.twist.twist.linear.x
        bvy = msg.twist.twist.linear.y
        bvz = msg.twist.twist.linear.z
        speed = math.sqrt(bvx * bvx + bvy * bvy + bvz * bvz)

        if speed < self._min_spd:
            return

        # Publish in the drone body frame (child_frame_id = x500_0/base_footprint).
        # Start at (0,0,0) — the drone centre — end in the body-frame velocity
        # direction scaled by self._scale.  RViz resolves the frame via TF, so
        # no world-position copying is needed and there is no physics-jitter.
        start = Point(x=0.0, y=0.0, z=0.0)
        end = Point(
            x=bvx * self._scale,
            y=bvy * self._scale,
            z=bvz * self._scale,
        )

        m = Marker()
        m.header.stamp = msg.header.stamp
        m.header.frame_id = msg.child_frame_id   # x500_0/base_footprint
        m.ns = "velocity"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.points = [start, end]
        m.scale.x = 0.06              # shaft diameter
        m.scale.y = 0.12              # head diameter
        m.scale.z = 0.0               # auto head length
        m.color = _speed_color(speed, self._max_spd)
        m.lifetime = Duration(sec=0, nanosec=500_000_000)   # auto-hide after 0.5 s
        self._pub.publish(m)


def main() -> None:
    rclpy.init()
    node = OdomToVelMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
