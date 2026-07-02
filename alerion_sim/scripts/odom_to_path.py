#!/usr/bin/env python3
"""
Odometry → Path publisher.

Builds a nav_msgs/Path by accumulating drone poses over time and
publishes it on /drone/path so RViz can draw a continuous flight trail.

Parameters
----------
topic        : odometry input topic  (default: /model/x500_0/odometry)
max_points   : maximum poses kept in the path before oldest are dropped
               (default: 3000  ≈ 5 min at 10 Hz with min_distance=0.1)
min_distance : minimum distance in metres between consecutive poses;
               points closer than this are skipped to avoid flooding
               the path when the drone is hovering (default: 0.1 m)
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class OdomToPath(Node):
    def __init__(self) -> None:
        super().__init__("odom_to_path")

        self.declare_parameter("topic", "/model/x500_0/odometry")
        self.declare_parameter("max_points", 3000)
        self.declare_parameter("min_distance", 0.1)

        self._topic = self.get_parameter("topic").value
        self._max_pts = int(self.get_parameter("max_points").value)
        self._min_dist = float(self.get_parameter("min_distance").value)

        self._path = Path()
        self._last_pos: tuple[float, float, float] | None = None

        self._pub = self.create_publisher(Path, "/drone/path", 10)
        self.create_subscription(Odometry, self._topic, self._cb, 10)
        self.get_logger().info(
            f"Publishing path on /drone/path  (max {self._max_pts} pts, "
            f"min dist {self._min_dist} m)"
        )

    def _cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        p = (pos.x, pos.y, pos.z)

        # Skip if the drone hasn't moved enough since the last recorded point
        if self._last_pos is not None:
            dx = p[0] - self._last_pos[0]
            dy = p[1] - self._last_pos[1]
            dz = p[2] - self._last_pos[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) < self._min_dist:
                return

        self._last_pos = p

        # Append new pose
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self._path.poses.append(ps)

        # Trim oldest poses to cap memory use
        if len(self._path.poses) > self._max_pts:
            self._path.poses = self._path.poses[-self._max_pts :]

        # Path header should use the same frame as the poses
        self._path.header = msg.header
        self._pub.publish(self._path)


def main() -> None:
    rclpy.init()
    node = OdomToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
