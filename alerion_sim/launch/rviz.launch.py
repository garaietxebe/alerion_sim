"""
RViz2 visualisation launch file.

Config resolution order (first match wins):
  1. config/rviz/<profile>.rviz   — profile-specific layout (e.g. lidar_scan.rviz)
  2. config/rviz/<level>.rviz     — per-level default
  3. config/rviz/minimal.rviz     — hard fallback

Pass the same level and profile that were used for the simulation:

  # terminal 1 — simulation
  ros2 launch alerion_sim simulation.launch.py level:=minimal profile:=lidar_scan

  # terminal 2 — visualisation
  ros2 launch alerion_sim rviz.launch.py level:=minimal profile:=lidar_scan

Inside Docker:
  ros2 launch /alerion_sim/launch/rviz.launch.py level:=minimal profile:=lidar_scan

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
_available_levels   = sorted(p.stem for p in (_CONFIG_DIR / "levels").glob("*.yaml"))
_available_profiles = ["auto"] + sorted(p.stem for p in (_CONFIG_DIR / "profiles").glob("*.yaml"))


def _rviz_setup(context: Any, *args: Any, **kwargs: Any) -> list[Any]:
    level   = LaunchConfiguration("level").perform(context)
    profile = LaunchConfiguration("profile").perform(context)

    # Resolution order:
    #   1. config/rviz/<profile>.rviz  — profile has its own dedicated layout
    #   2. config/rviz/<level>.rviz    — level default
    #   3. config/rviz/<_FALLBACK>.rviz — hard fallback
    candidates = []
    if profile != "auto":
        candidates.append((_RVIZ_DIR / f"{profile}.rviz", f"profile '{profile}'"))
    candidates.append((_RVIZ_DIR / f"{level}.rviz", f"level '{level}'"))

    rviz_cfg = None
    chosen_label = None
    for path, label in candidates:
        if path.exists():
            rviz_cfg = path
            chosen_label = label
            break

    if rviz_cfg is None:
        fallback_cfg = _RVIZ_DIR / f"{_FALLBACK}.rviz"
        if not fallback_cfg.exists():
            tried = "\n    ".join(str(p) for p, _ in candidates)
            raise FileNotFoundError(
                f"\n[alerion_sim] ERROR: no RViz config found for "
                f"level='{level}' profile='{profile}', and fallback "
                f"'{_FALLBACK}' is also missing.\n"
                f"  Tried:\n    {tried}\n"
                f"    {fallback_cfg}\n"
            )
        tried = ", ".join(f"'{p.name}'" for p, _ in candidates)
        print(
            f"\n[alerion_sim] WARNING: no RViz config found "
            f"(tried {tried}).\n"
            f"  Falling back to '{_FALLBACK}' config: {fallback_cfg}\n"
        )
        rviz_cfg = fallback_cfg
        chosen_label = f"fallback '{_FALLBACK}'"

    print(f"[alerion_sim] RViz config   : {rviz_cfg.name}  ({chosen_label})")

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
                    "Used to select config/rviz/<level>.rviz if no profile config exists."
                ),
            ),
            DeclareLaunchArgument(
                "profile",
                default_value="auto",
                choices=_available_profiles,
                description=(
                    "Sensor/config profile. If config/rviz/<profile>.rviz exists "
                    "it takes priority over the level config."
                ),
            ),
            OpaqueFunction(function=_rviz_setup),
        ]
    )
