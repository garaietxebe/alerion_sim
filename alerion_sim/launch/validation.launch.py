"""
Passive validation launch file.

Run alongside simulation.launch.py to monitor CPU and RAM usage per process
and verify that all expected topics are being published.

Accepts the same level / sensor_profile arguments as simulation.launch.py so
it can compute exactly which topics should be active for the current run.
"""

import tempfile
from pathlib import Path
from typing import Any

import yaml
from launch import LaunchDescription  # type: ignore[attr-defined]
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

_LAUNCH_DIR = Path(__file__).resolve().parent
_PROJECT_DIR = _LAUNCH_DIR.parent
_CONFIG_DIR = _PROJECT_DIR / "config"
_PARAM_FILE = str(_CONFIG_DIR / "validation.yaml")

# Re-use the same config loader as simulation.launch.py.
# importlib is needed because the filename contains dots (simulation.launch.py)
# which breaks normal Python import syntax.
import importlib.util as _ilu  # noqa: E402

_spec = _ilu.spec_from_file_location("simulation_launch", _LAUNCH_DIR / "simulation.launch.py")
_sim_launch = _ilu.module_from_spec(_spec)  # type: ignore[arg-type]
_spec.loader.exec_module(_sim_launch)  # type: ignore[union-attr]
load_config = _sim_launch.load_config


def _launch_setup(context: Any, *args: Any, **kwargs: Any) -> list[Any]:
    level = LaunchConfiguration("level").perform(context)
    sensor_profile = LaunchConfiguration("sensor_profile").perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    compute_csv = LaunchConfiguration("compute_csv").perform(context)
    status_interval = LaunchConfiguration("status_interval").perform(context)
    cpu_sample_hz = LaunchConfiguration("cpu_sample_hz").perform(context)

    # Load the merged config so we know which sensors are active
    cfg = load_config(level, sensor_profile)
    sim = cfg.get("simulation", {})
    px4 = sim.get("px4", {})
    sens = cfg.get("sensors", {})
    lidar_cfg = sens.get("lidar", {})
    camera_cfg = sens.get("camera", {})
    cam_full_cfg = cfg.get("camera", {})
    nadir_cfg = cfg.get("nadir_camera", {})

    _dist_ok = (
        px4.get("start_px4", True)
        and camera_cfg.get("enabled", False)
        and cam_full_cfg.get("distortion_enabled", False)
    )

    # Build the exact topic list that this level+profile combination publishes
    expected_topics: list[str] = ["/clock"]
    if px4.get("start_px4", True):
        expected_topics.append(f"/model/{model_name}/odometry")
        if lidar_cfg.get("enabled", False):
            expected_topics += ["/lidar", "/lidar/points"]
        if camera_cfg.get("enabled", False):
            expected_topics += [
                "/camera/image_raw",
                "/camera/image_raw/camera_info",
                f"/model/{model_name}/command/gimbal_pitch",
                f"/model/{model_name}/command/gimbal_roll",
            ]
        if _dist_ok:
            expected_topics.append("/camera/image_distorted")
        if nadir_cfg.get("enabled", False):
            expected_topics.append("/camera/nadir_raw")

    # Write dynamic parameters (including the computed expected_topics list) to a
    # temporary YAML file.  Using --params-file avoids command-line quoting issues
    # with string lists and doesn't require the alerion_sim package to be installed
    # in the container's ament index.
    dynamic_params = {
        "validation_node": {
            "ros__parameters": {
                "model_name": model_name,
                "world_name": world_name,
                "compute_csv": compute_csv,
                "status_interval": float(status_interval),
                "cpu_sample_hz": float(cpu_sample_hz),
                "expected_topics": expected_topics,
            }
        }
    }
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix="alerion_validation_", delete=False
    )
    yaml.dump(dynamic_params, tmp)
    tmp.flush()

    return [
        ExecuteProcess(
            cmd=[
                "python3",
                str(_PROJECT_DIR / "scripts" / "validation_node.py"),
                "--ros-args",
                "--params-file", _PARAM_FILE,
                "--params-file", tmp.name,
            ],
            additional_env={"PYTHONUNBUFFERED": "1"},
            output="screen",
            name="validation_node",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "level",
                default_value="full",
                choices=["minimal", "development", "full"],
                description="Fidelity level — must match the running simulation",
            ),
            DeclareLaunchArgument(
                "sensor_profile",
                default_value="auto",
                choices=["auto", "navigation", "vision", "hard_vision"],
                description="Sensor profile — must match the running simulation",
            ),
            DeclareLaunchArgument("model_name", default_value="x500_0"),
            DeclareLaunchArgument(
                "world_name",
                default_value="inspection",
                description="Gazebo world name for RTF stats subscription",
            ),
            DeclareLaunchArgument(
                "status_interval",
                default_value="10.0",
                description="Seconds between console status lines",
            ),
            DeclareLaunchArgument(
                "cpu_sample_hz",
                default_value="0.2",
                description="Compute sampling rate in Hz (0.2 = every 5 s)",
            ),
            DeclareLaunchArgument(
                "compute_csv",
                default_value="/tmp/alerion_compute.csv",
                description="Output CSV path for compute load data",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
