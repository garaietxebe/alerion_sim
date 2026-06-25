"""
Passive validation launch file.

Run alongside simulation.launch.py to monitor CPU and RAM usage per process.
"""

from pathlib import Path

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription  # type: ignore[attr-defined]

_LAUNCH_DIR = Path(__file__).resolve().parent
_PROJECT_DIR = _LAUNCH_DIR.parent
_PARAM_FILE = str(_PROJECT_DIR / "config" / "validation.yaml")


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_name", default_value="x500_0"),
            DeclareLaunchArgument(
                "world_name",
                default_value="inspection",
                description="Gazebo world name for RTF stats subscription",
            ),
            DeclareLaunchArgument(
                "status_interval",
                default_value="5.0",
                description="Seconds between console status lines",
            ),
            DeclareLaunchArgument(
                "cpu_sample_hz",
                default_value="0.2",
                description="Compute sampling rate in Hz (0.2 = every 5 s)",
            ),
            DeclareLaunchArgument(
                "compute_csv",
                default_value="/alerion_sim/logs/alerion_compute.csv",
                description="Output CSV path for compute load data",
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(_PROJECT_DIR / "scripts" / "validation_node.py"),
                    "--ros-args",
                    "--params-file",
                    _PARAM_FILE,
                    "-p",
                    ["model_name:=", LaunchConfiguration("model_name")],
                    "-p",
                    ["world_name:=", LaunchConfiguration("world_name")],
                    "-p",
                    ["status_interval:=", LaunchConfiguration("status_interval")],
                    "-p",
                    ["cpu_sample_hz:=", LaunchConfiguration("cpu_sample_hz")],
                    "-p",
                    ["compute_csv:=", LaunchConfiguration("compute_csv")],
                ],
                output="screen",
                name="validation_node",
            ),
        ]
    )
