#!/usr/bin/env python3
"""
Odometry → TF broadcaster.

Subscribes to a nav_msgs/Odometry topic and re-publishes the pose as a
TF transform so that RViz (and any other tf2 consumer) can locate the
vehicle in 3-D space.

Why this node exists
--------------------
ros_gz_bridge can bridge gz.msgs.Pose_V to tf2_msgs/TFMessage in principle,
but in ROS 2 Jazzy + Gazebo Harmonic the conversion leaves both frame_id and
child_frame_id empty, so the tf2 library discards every transform.
The odometry message already carries the correct frame names in its header,
so reading from there is simpler and more reliable.

Usage (inside simulation.launch.py, not meant to be run manually):
  python3 odom_to_tf.py --ros-args -p topic:=/model/x500_0/odometry
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self) -> None:
        super().__init__("odom_to_tf")
        self.declare_parameter("topic", "/model/x500_0/odometry")
        topic = self.get_parameter("topic").value

        self._br = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self._cb, 10)
        self.get_logger().info(f"Broadcasting TF from odometry topic: {topic}")

    def _cb(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header = msg.header                       # frame_id  = parent (e.g. "odom")
        t.child_frame_id = msg.child_frame_id       # child     (e.g. "base_link")
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
