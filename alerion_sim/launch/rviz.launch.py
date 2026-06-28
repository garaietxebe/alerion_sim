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
The default Fixed Frame is "odom".  If RViz shows a warning like
"No transform from [odom] to [Fixed Frame]", open the Global Options
panel in RViz and change Fixed Frame to match the frame_id published
by /model/x500_0/odometry (check with: ros2 topic echo --once /model/x500_0/odometry).
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
