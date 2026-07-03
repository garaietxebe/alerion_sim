#!/usr/bin/env python3
"""
Drone state visualiser.

Single node that combines three odometry consumers into one process and one
subscription, replacing the former odom_to_tf / odom_to_path /
odom_to_vel_marker scripts.

Publishes
---------
/tf                       (tf2_msgs/TFMessage)
    Vehicle pose as a dynamic TF transform.  Required because ros_gz_bridge's
    gz.msgs.Pose_V → tf2_msgs/TFMessage conversion leaves frame_id and
    child_frame_id empty in ROS 2 Jazzy + Gazebo Harmonic.

/drone/path               (nav_msgs/Path)
    Cumulative flight trail — pose list grown every time the drone moves more
    than `min_distance` metres from the last recorded point.

/drone/velocity_marker    (visualization_msgs/Marker)
    Speed-coloured ARROW in the drone body frame.
    Green (slow) → Yellow → Red (fast).

Parameters
----------
topic        : odometry input topic   (default: /model/x500_0/odometry)
max_points   : max poses kept in path (default: 3000 ≈ 5 min at 10 Hz)
min_distance : minimum pose spacing m (default: 0.1)
scale_factor : arrow length per m/s   (default: 0.4)
max_speed    : speed mapped to full red, m/s (default: 5.0)
min_speed    : speeds below this are not published, m/s (default: 0.05)
"""

import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


def _speed_color(speed: float, max_speed: float) -> ColorRGBA:
    """Green → Yellow → Red as speed goes from 0 to max_speed."""
    t = min(speed / max(max_speed, 1e-6), 1.0)
    c = ColorRGBA(a=1.0, b=0.0)
    if t < 0.5:
        c.r = t * 2.0
        c.g = 1.0
    else:
        c.r = 1.0
        c.g = 1.0 - (t - 0.5) * 2.0
    return c


class DroneVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("drone_visualizer")

        self.declare_parameter("topic",        "/model/x500_0/odometry")
        self.declare_parameter("max_points",   3000)
        self.declare_parameter("min_distance", 0.1)
        self.declare_parameter("scale_factor", 0.4)
        self.declare_parameter("max_speed",    5.0)
        self.declare_parameter("min_speed",    0.05)

        p = self.get_parameter
        topic          = p("topic").value
        self._max_pts  = int(p("max_points").value)
        self._min_dist = float(p("min_distance").value)
        self._scale    = float(p("scale_factor").value)
        self._max_spd  = float(p("max_speed").value)
        self._min_spd  = float(p("min_speed").value)

        self._tf_br   = TransformBroadcaster(self)
        self._path    = Path()
        self._last_pos: tuple[float, float, float] | None = None

        self._path_pub = self.create_publisher(Path,   "/drone/path",            10)
        self._vel_pub  = self.create_publisher(Marker, "/drone/velocity_marker", 10)

        # One subscription dispatches to all three publishers.
        self.create_subscription(Odometry, topic, self._cb, 10)

        self.get_logger().info(
            f"Drone visualizer started  ({topic})\n"
            f"  /tf                    : odom → base_footprint transform\n"
            f"  /drone/path            : trail  (max {self._max_pts} pts, "
            f"min dist {self._min_dist} m)\n"
            f"  /drone/velocity_marker : arrow  "
            f"(scale {self._scale} m per m/s, red at {self._max_spd} m/s)\n"
        )

    # ── TF ──────────────────────────────────────────────────────────────────

    def _publish_tf(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header         = msg.header            # frame_id  = odom frame
        t.child_frame_id = msg.child_frame_id    # child     = base_footprint
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation      = msg.pose.pose.orientation
        self._tf_br.sendTransform(t)

    # ── Flight trail ─────────────────────────────────────────────────────────

    def _publish_path(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        cur = (pos.x, pos.y, pos.z)

        if self._last_pos is not None:
            dx = cur[0] - self._last_pos[0]
            dy = cur[1] - self._last_pos[1]
            dz = cur[2] - self._last_pos[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) < self._min_dist:
                return          # haven't moved far enough — skip

        self._last_pos = cur

        ps = PoseStamped(header=msg.header, pose=msg.pose.pose)
        self._path.poses.append(ps)

        if len(self._path.poses) > self._max_pts:
            self._path.poses = self._path.poses[-self._max_pts:]

        self._path.header = msg.header
        self._path_pub.publish(self._path)

    # ── Velocity arrow ───────────────────────────────────────────────────────

    def _publish_vel_marker(self, msg: Odometry) -> None:
        # Twist is expressed in the child frame (body frame).
        bvx = msg.twist.twist.linear.x
        bvy = msg.twist.twist.linear.y
        bvz = msg.twist.twist.linear.z
        speed = math.sqrt(bvx * bvx + bvy * bvy + bvz * bvz)

        if speed < self._min_spd:
            return

        # Marker in the body frame at the drone centre (0, 0, 0).
        # RViz resolves the frame via TF — no world-position copying needed,
        # no physics-jitter artefacts.
        m = Marker()
        m.header.stamp    = msg.header.stamp
        m.header.frame_id = msg.child_frame_id  # x500_0/base_footprint
        m.ns     = "velocity"
        m.id     = 0
        m.type   = Marker.ARROW
        m.action = Marker.ADD
        m.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=bvx * self._scale, y=bvy * self._scale, z=bvz * self._scale),
        ]
        m.scale.x = 0.06    # shaft diameter
        m.scale.y = 0.12    # head diameter
        m.scale.z = 0.0     # auto head length
        m.color   = _speed_color(speed, self._max_spd)
        m.lifetime = Duration(sec=0, nanosec=500_000_000)   # 0.5 s
        self._vel_pub.publish(m)

    # ── Shared callback ──────────────────────────────────────────────────────

    def _cb(self, msg: Odometry) -> None:
        self._publish_tf(msg)
        self._publish_path(msg)
        self._publish_vel_marker(msg)


def main() -> None:
    rclpy.init()
    node = DroneVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
