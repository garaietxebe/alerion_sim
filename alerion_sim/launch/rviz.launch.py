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
The Fixed Frame in all .rviz configs is "x500_0/odom".  TF is published by
the drone_visualizer node (part of simulation.launch.py), which reads
nav_msgs/Odometry and rebroadcasts the pose as a /tf transform.  The full
transform chain is:

  x500_0/odom → x500_0/base_footprint → lidar_sensor_link
                                       → x500_0/camera_link

If RViz still warns "No transform from [X] to [Fixed Frame]", the
drone_visualizer node hasn't received its first odometry message yet — the
drone takes ~6 s to spawn.  Wait a moment and it resolves automatically.
To inspect the live tree: ros2 run tf2_tools view_frames
"""

from pathlib import Path
from typing import Any

from launch import LaunchDescription  # type: ignore[attr-defined]
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # type: ignore[attr-defined]

_CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
_RVIZ_DIR   = _CONFIG_DIR / "rviz"
_FALLBACK   = "minimal"

# Discover available levels from the filesystem so a new config/levels/*.yaml
# is automatically accepted without editing this file.
_available_levels = sorted(p.stem for p in (_CONFIG_DIR / "levels").glob("*.yaml"))


def _rviz_setup(context: Any, *args: Any, **kwargs: Any) -> list[Any]:
    level    = LaunchConfiguration("level").perform(context)
    rviz_cfg = _RVIZ_DIR / f"{level}.rviz"

    if not rviz_cfg.exists():
        fallback_cfg = _RVIZ_DIR / f"{_FALLBACK}.rviz"
        if not fallback_cfg.exists():
            raise FileNotFoundError(
                f"\n[alerion_sim] ERROR: no RViz config for level '{level}' "
                f"and fallback '{_FALLBACK}' is also missing.\n"
                f"  Create at least one of:\n"
                f"    {rviz_cfg}\n"
                f"    {fallback_cfg}\n"
            )
        print(
            f"\n[alerion_sim] WARNING: no RViz config found for level '{level}'.\n"
            f"  Expected : {rviz_cfg}\n"
            f"  Falling back to '{_FALLBACK}' config: {fallback_cfg}\n"
            f"  To add a dedicated config, create config/rviz/{level}.rviz\n"
        )
        rviz_cfg = fallback_cfg

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", str(rviz_cfg)],
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "level",
                default_value="full",
                choices=_available_levels,
                description=(
                    "Fidelity level — any YAML in config/levels/. "
                    "Selects config/rviz/<level>.rviz "
                    f"(falls back to '{_FALLBACK}' if missing)."
                ),
            ),
            OpaqueFunction(function=_rviz_setup),
        ]
    )
