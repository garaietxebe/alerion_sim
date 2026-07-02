"""
RViz2 visualisation launch file.

Loads the per-level RViz2 config from config/rviz/<level>.rviz so that
the visual layout automatically matches the fidelity of the running simulation.

Run alongside simulation.launch.py:

  # terminal 1 — simulation
  ros2 launch alerion_sim simulation.launch.py level:=full

  # terminal 2 — visualisation (same level)
  ros2 launch alerion_sim rviz.launch.py level:=full

Inside Docker (host must have RViz2 + ROS 2 sourced):
  ros2 launch /alerion_sim/launch/rviz.launch.py level:=full

Fixed frame note
----------------
The Fixed Frame is "inspection" (the Gazebo world name).  The simulation
launch file bridges Gazebo TF to ROS 2 /tf via two ros_gz_bridge nodes
(ros_gz_world_tf + ros_gz_model_tf), so the full transform chain is:

  inspection → x500_0 → x500_0/base_link → lidar_sensor_link / camera_link …

If RViz still warns "No transform from [X] to [Fixed Frame]", run:
  ros2 run tf2_tools view_frames
to see the actual tree and adjust Fixed Frame accordingly.
"""

from pathlib import Path

from launch import LaunchDescription  # type: ignore[attr-defined]
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # type: ignore[attr-defined]

_RVIZ_DIR = Path(__file__).resolve().parent.parent / "config" / "rviz"


def generate_launch_description() -> LaunchDescription:
    level = LaunchConfiguration("level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "level",
                default_value="full",
                choices=["minimal", "development", "full"],
                description=(
                    "Fidelity level — must match the running simulation. "
                    "Selects config/rviz/<level>.rviz."
                ),
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value="x500_0",
                description="Vehicle model name (used in topic paths like /model/<name>/odometry)",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    [str(_RVIZ_DIR) + "/", level, ".rviz"],
                ],
                output="screen",
            ),
        ]
    )
