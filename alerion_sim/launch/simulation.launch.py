"""
Main simulation launch file.

Starts Gazebo, PX4 SITL, MicroXRCE agent, ROS bridges, and optional sensor
nodes according to the selected fidelity level and sensor profile.
"""

import copy
import os
import tempfile
from pathlib import Path
from typing import Any

import yaml
from jinja2 import Environment, FileSystemLoader
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription  # type: ignore[attr-defined]

# ---------------------------------------------------------------------------
# Project paths
# ---------------------------------------------------------------------------

_LAUNCH_DIR = Path(__file__).resolve().parent
_PROJECT_DIR = _LAUNCH_DIR.parent
_CONFIG_DIR = _PROJECT_DIR / "config"
_MODELS_DIR = _PROJECT_DIR / "models"
_WORLDS_DIR = _PROJECT_DIR / "worlds"

PX4_DIR = Path(os.environ.get("PX4_DIR", Path.home() / "Desktop" / "PX4-Autopilot"))
PX4_MODELS_DIR = PX4_DIR / "Tools" / "simulation" / "gz" / "models"
PX4_WORLDS_DIR = PX4_DIR / "Tools" / "simulation" / "gz" / "worlds"
# Gazebo-classic model resources (shelves, boxes, pallets)
PX4_CLASSIC_MODELS_DIR = (
    PX4_DIR / "Tools" / "simulation" / "gazebo-classic" / "sitl_gazebo-classic" / "models"
)


# Config loading and deep-merge utilities


def _deep_merge(base: dict, override: dict) -> dict:
    """merge para los diferentes yaml"""
    result = copy.deepcopy(base)
    for key, val in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(val, dict):
            result[key] = _deep_merge(result[key], val)
        else:
            result[key] = copy.deepcopy(val)
    return result


def load_config(level: str, sensor_profile: str = "auto") -> dict:
    """
    Config merge order (each layer overrides the previous):
      1. config/simulation.yaml
      2. config/sensors/*.yaml
      3. config/vehicle/x500.yaml
      4. config/levels/<level>.yaml
      5. config/profiles/<profile>.yaml   (if file exists)
      6. sensor_profile inline override   (navigation | vision | auto)
    """

    def _load(path: Path) -> dict:
        with open(path) as fh:
            return yaml.safe_load(fh) or {}

    cfg = _load(_CONFIG_DIR / "simulation.yaml")

    for sensor_file in sorted((_CONFIG_DIR / "sensors").glob("*.yaml")):
        cfg = _deep_merge(cfg, _load(sensor_file))

    vehicle_file = _CONFIG_DIR / "vehicle" / "x500.yaml"
    if vehicle_file.exists():
        cfg = _deep_merge(cfg, _load(vehicle_file))

    level_file = _CONFIG_DIR / "levels" / f"{level}.yaml"
    if not level_file.exists():
        raise FileNotFoundError(f"Level config not found: {level_file}")
    cfg = _deep_merge(cfg, _load(level_file))

    profile_file = _CONFIG_DIR / "profiles" / f"{sensor_profile}.yaml"
    if profile_file.exists():
        cfg = _deep_merge(cfg, _load(profile_file))

    if sensor_profile == "navigation":
        cfg.setdefault("sensors", {}).setdefault("lidar", {})["enabled"] = True
        cfg.setdefault("sensors", {}).setdefault("camera", {})["enabled"] = False
    elif sensor_profile == "vision":
        cfg.setdefault("sensors", {}).setdefault("lidar", {})["enabled"] = False
        cfg.setdefault("sensors", {}).setdefault("camera", {})["enabled"] = True

    return cfg


def render_template(template_path: Path, context: dict, suffix: str = ".sdf") -> str:
    """
    Renderizamos un temp file con el contexto que hayamos pedido usando un template, return de su path
    """
    env = Environment(
        loader=FileSystemLoader(str(template_path.parent)),
        trim_blocks=True,
        lstrip_blocks=True,
    )
    rendered = env.get_template(template_path.name).render(**context)
    fd = tempfile.NamedTemporaryFile(
        mode="w",
        suffix=suffix,
        prefix=f"{template_path.stem}_",
        delete=False,
    )
    fd.write(rendered)
    fd.flush()
    return fd.name


def launch_setup(context: Any, *args: Any, **kwargs: Any) -> list[Any]:
    level = LaunchConfiguration("level").perform(context)
    sensor_profile = LaunchConfiguration("sensor_profile").perform(context)

    cfg = load_config(level, sensor_profile)
    sim = cfg.get("simulation", {})
    phys = cfg.get("physics", {})
    rend = cfg.get("rendering", {})
    sens = cfg.get("sensors", {})
    wind = cfg.get("wind", {})
    px4 = sim.get("px4", {})
    dds = sim.get("dds", {})
    spawn = sim.get("vehicle", {}).get("spawn", {})

    lidar_cfg = sens.get("lidar", {})
    camera_cfg = sens.get("camera", {})
    cam_full_cfg = cfg.get("camera", {})  # full camera/gimbal params (camera.yaml + level overrides)

    # 1 render del mundo

    world_ctx = {
        "physics_enabled": phys.get("enabled", True),
        "physics_gravity_z": phys.get("gravity_z", -9.81),
        "physics_rtf": phys.get("real_time_factor", 1.0),
        "physics_step": phys.get("max_step_size", 0.004),
        "rendering_enabled": rend.get("enabled", True),
        "rendering_shadows": rend.get("shadows", True),
        "rendering_pbr": rend.get("pbr", False),
        "wind_enabled": wind.get("enabled", False),
        "wind_vel_x": wind.get("linear_velocity_x", 0.0),
        "wind_vel_y": wind.get("linear_velocity_y", 0.0),
        "wind_vel_z": wind.get("linear_velocity_z", 0.0),
    }
    rendered_world = render_template(_WORLDS_DIR / "inspection.sdf.j2", world_ctx)

    # 2 Render modelo SDF

    rendered_model = None
    if px4.get("start_px4", True):
        model_ctx = {
            "sensors_lidar_enabled": lidar_cfg.get("enabled", False),
            "sensors_lidar_noise": lidar_cfg.get("noise", False),
            "sensors_camera_enabled": camera_cfg.get("enabled", False),
            "sensors_camera_noise": camera_cfg.get("noise", False),
            "sensors_nadir_enabled": cfg.get("nadir_camera", {}).get("enabled", False),
            "physics_enabled": phys.get("enabled", True),
            "lidar": cfg.get("lidar", {}),
            "camera": cfg.get("camera", {}),
            "nadir_camera": cfg.get("nadir_camera", {}),
        }
        rendered_model = render_template(
            _MODELS_DIR / "x500_inspection" / "model.sdf.jinja", model_ctx
        )

    print(f"\n[alerion_sim] Level        : {level}")
    print(f"[alerion_sim] Sensor profile: {sensor_profile}")
    print(f"[alerion_sim] LiDAR active  : {lidar_cfg.get('enabled', False)}")
    print(f"[alerion_sim] Camera active : {camera_cfg.get('enabled', False)}")
    print(f"[alerion_sim] Physics       : {phys.get('enabled', True)}")
    print(f"[alerion_sim] Wind          : {wind.get('enabled', False)}")
    print(f"[alerion_sim] PX4 SITL      : {px4.get('start_px4', True)}\n")

    gz_args = ["-r"]
    if rend.get("headless", False):
        gz_args += ["-s"]

    actions = []

    # Gazebo environment variables: resource path, plugin path, server config

    gz_plugins_dir = str(
        PX4_DIR / "build" / "px4_sitl_default" / "src" / "modules" / "simulation" / "gz_plugins"
    )
    # Use our own server.config instead of PX4's to avoid loading ABI-fragile
    # custom plugins (OpticalFlowSystem, GstCameraSystem) that break when the
    # runtime gz-harmonic version differs from the one used to compile PX4 SITL.
    gz_server_cfg = str(_CONFIG_DIR / "gz_server.config")

    actions.append(
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=":".join(
                filter(
                    None,
                    [
                        str(PX4_MODELS_DIR),
                        str(PX4_WORLDS_DIR),
                        str(_MODELS_DIR),
                        str(PX4_CLASSIC_MODELS_DIR),  # shelves, big_box, europallet DAE meshes
                        str(_PROJECT_DIR),  # exposes media/materials/scripts/gazebo.material
                        # stub that silences Ogre1 URI errors from gazebo-classic DAE mesh references
                        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                    ],
                )
            ),
        )
    )
    actions.append(
        SetEnvironmentVariable(
            name="GZ_SIM_SYSTEM_PLUGIN_PATH",
            value=":".join(
                filter(
                    None,
                    [
                        gz_plugins_dir,
                        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                    ],
                )
            ),
        )
    )
    actions.append(
        SetEnvironmentVariable(
            name="GZ_SIM_SERVER_CONFIG_PATH",
            value=gz_server_cfg,
        )
    )

    # MicroXRCE-DDS agent: bridges PX4 uORB messages to ROS 2 DDS (skipped in minimal)

    if px4.get("start_px4", True):
        import shutil

        if shutil.which("MicroXRCEAgent") is None:
            raise RuntimeError(
                "\n\n  MicroXRCEAgent binary not found in PATH.\n"
                "  Install it with:\n"
                "    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent\n"
                "    cd ~/Micro-XRCE-DDS-Agent && mkdir build && cd build\n"
                "    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)\n"
                "    sudo make install && sudo ldconfig /usr/local/lib/\n"
            )
        actions.append(
            ExecuteProcess(
                cmd=[
                    "MicroXRCEAgent",
                    dds.get("transport", "udp4"),
                    "-p",
                    str(dds.get("port", 8888)),
                    "-v",
                    "4",
                ],
                output="screen",
                name="micro_xrce_dds",
            )
        )

    # Start Gazebo

    actions.append(
        ExecuteProcess(
            cmd=["gz", "sim", rendered_world] + gz_args,
            additional_env={"GZ_LOG_LEVEL": "3"},
            output="screen",
            name="gazebo",
        )
    )

    # Spawn vehicle and start PX4 with delays so Gazebo is ready to accept the connection

    if px4.get("start_px4", True):
        px4_firmware_dir = Path(px4.get("firmware_dir") or str(PX4_DIR))
        px4_bin = str(px4_firmware_dir / "build" / "px4_sitl_default" / "bin" / "px4")
        px4_rootfs = str(px4_firmware_dir / "build" / "px4_sitl_default" / "rootfs")
        px4_build = str(px4_firmware_dir / "build" / "px4_sitl_default")
        instance = str(px4.get("instance", 0))

        gz_model_instance = f"x500_{px4.get('instance', 0)}"

        # spawn model at t=3 s

        actions.append(
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        arguments=[
                            "-file",
                            rendered_model,
                            "-name",
                            gz_model_instance,
                            "-x",
                            str(spawn.get("x", 0.0)),
                            "-y",
                            str(spawn.get("y", 0.0)),
                            "-z",
                            str(spawn.get("z", 0.5)),
                            "-Y",
                            str(spawn.get("yaw", 0.0)),
                        ],
                        output="screen",
                        name="spawn_vehicle",
                    )
                ],
            )
        )

        # start PX4 SITL at t=6 s (model already spawned)

        actions.append(
            TimerAction(
                period=6.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            px4_bin,
                            px4_rootfs,
                            "-s",
                            "etc/init.d-posix/rcS",
                            "-i",
                            instance,
                            "-d",
                        ],
                        cwd=px4_build,
                        additional_env={
                            "PX4_SIM_MODEL": "gz_x500",
                            "PX4_GZ_MODEL_NAME": gz_model_instance,
                            "PX4_GZ_WORLD": "inspection",
                            "PX4_UXRCE_DDS_PORT": str(dds.get("port", 8888)),
                            "PX4_GZ_MODELS": str(PX4_MODELS_DIR),
                            "PX4_GZ_WORLDS": str(PX4_WORLDS_DIR),
                        },
                        output="screen",
                        name="px4_sitl",
                    )
                ],
            )
        )

    # ROS to Gazebo bridge
    gz_model_name = f"x500_{px4.get('instance', 0)}"
    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    if px4.get("start_px4", True):
        bridge_args.append(
            f"/model/{gz_model_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"
        )
        if lidar_cfg.get("enabled", False):
            bridge_args.append("/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan")
            bridge_args.append("/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked")
        if camera_cfg.get("enabled", False):
            bridge_args += [
                "/camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                f"/model/{gz_model_name}/command/gimbal_roll@std_msgs/msg/Float64]gz.msgs.Double",
                f"/model/{gz_model_name}/command/gimbal_pitch@std_msgs/msg/Float64]gz.msgs.Double",
            ]

    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=bridge_args,
            output="screen",
            name="ros_gz_bridge",
        )
    )

    # TF broadcasting + RViz helper publishers
    #
    # ros_gz_bridge's gz.msgs.Pose_V → tf2_msgs/TFMessage conversion does not
    # populate frame_id / child_frame_id in ROS 2 Jazzy + Gazebo Harmonic, so
    # every transform arrives with empty frame names and tf2 discards them.
    #
    # Instead we use reliable dedicated nodes:
    #
    #   odom_to_tf        – reads nav_msgs/Odometry and rebroadcasts as /tf.
    #   odom_to_path      – accumulates odometry poses into nav_msgs/Path
    #                       published on /drone/path (flight trail in RViz).
    #   odom_to_vel_marker– converts odometry twist into a coloured arrow
    #                       published on /drone/velocity_marker (speed & direction).
    #   static transforms – fixed sensor-frame offsets published on /tf_static.
    #
    if px4.get("start_px4", True):
        # Dynamic: odometry → TF (parent frame → base_link / base_footprint)
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(_PROJECT_DIR / "scripts" / "odom_to_tf.py"),
                    "--ros-args",
                    "-p", f"topic:=/model/{gz_model_name}/odometry",
                ],
                additional_env={"PYTHONUNBUFFERED": "1"},
                output="screen",
                name="odom_to_tf",
            )
        )

        # Flight trail: odometry → nav_msgs/Path on /drone/path
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(_PROJECT_DIR / "scripts" / "odom_to_path.py"),
                    "--ros-args",
                    "-p", f"topic:=/model/{gz_model_name}/odometry",
                ],
                additional_env={"PYTHONUNBUFFERED": "1"},
                output="screen",
                name="odom_to_path",
            )
        )

        # Velocity arrow: odometry twist → visualization_msgs/Marker on /drone/velocity_marker
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(_PROJECT_DIR / "scripts" / "odom_to_vel_marker.py"),
                    "--ros-args",
                    "-p", f"topic:=/model/{gz_model_name}/odometry",
                ],
                additional_env={"PYTHONUNBUFFERED": "1"},
                output="screen",
                name="odom_to_vel_marker",
            )
        )

        # Static: lidar sensor frame relative to base_footprint.
        # Gazebo Harmonic scopes all frame names with the model name (x500_0/…).
        # Pose from config/sensors/lidar.yaml: x=0 y=0 z=0.08 rpy=0 0 0
        # <gz_frame_id> in model SDF is "lidar_sensor_link" → scoped "x500_0/lidar_sensor_link"
        if lidar_cfg.get("enabled", False):
            lp = lidar_cfg.get("pose", {})
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    arguments=[
                        "--x",     str(lp.get("x", 0.0)),
                        "--y",     str(lp.get("y", 0.0)),
                        "--z",     str(lp.get("z", 0.08)),
                        "--roll",  str(lp.get("roll", 0.0)),
                        "--pitch", str(lp.get("pitch", 0.0)),
                        "--yaw",   str(lp.get("yaw", 0.0)),
                        "--frame-id",       f"{gz_model_name}/base_footprint",
                        "--child-frame-id", "lidar_sensor_link",
                    ],
                    output="screen",
                    name="tf_lidar_static",
                )
            )

        # Static: camera link relative to base_footprint.
        # Pose from config/sensors/camera.yaml: x=0.10 y=0 z=0.05 rpy=0 0 0
        if camera_cfg.get("enabled", False):
            cp = cam_full_cfg.get("pose", camera_cfg.get("pose", {}))
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    arguments=[
                        "--x",     str(cp.get("x", 0.10)),
                        "--y",     str(cp.get("y", 0.0)),
                        "--z",     str(cp.get("z", 0.05)),
                        "--roll",  str(cp.get("roll", 0.0)),
                        "--pitch", str(cp.get("pitch", 0.0)),
                        "--yaw",   str(cp.get("yaw", 0.0)),
                        "--frame-id",       f"{gz_model_name}/base_footprint",
                        "--child-frame-id", f"{gz_model_name}/camera_link",
                    ],
                    output="screen",
                    name="tf_camera_static",
                )
            )

    # Dedicated image bridge node (parameter_bridge has pixel format conversion issues with gz.msgs.Image in Gazebo Harmonic)

    if px4.get("start_px4", True) and camera_cfg.get("enabled", False):
        actions.append(
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                arguments=["/camera/image_raw"],
                output="screen",
                name="camera_image_bridge",
            )
        )

    # Brown-Conrady distortion post-processor, publishes /camera/image_distorted alongside the clean /camera/image_raw

    _dist_ok = (
        px4.get("start_px4", True)
        and camera_cfg.get("enabled", False)
        and cam_full_cfg.get("distortion_enabled", False)
    )
    print(
        f"[alerion_sim] Distortion node: {'YES' if _dist_ok else 'NO'} "
        f"(start_px4={px4.get('start_px4', True)}, "
        f"camera.enabled={camera_cfg.get('enabled', False)}, "
        f"distortion_enabled={cam_full_cfg.get('distortion_enabled', False)})"
    )
    if _dist_ok:
        dist = cam_full_cfg.get("distortion", {})
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(_PROJECT_DIR / "scripts" / "camera_distortion.py"),
                    "--ros-args",
                    "-p",
                    f"k1:={dist.get('k1', 0.0)}",
                    "-p",
                    f"k2:={dist.get('k2', 0.0)}",
                    "-p",
                    f"k3:={dist.get('k3', 0.0)}",
                    "-p",
                    f"p1:={dist.get('p1', 0.0)}",
                    "-p",
                    f"p2:={dist.get('p2', 0.0)}",
                ],
                output="screen",
                name="camera_distortion",
            )
        )

    # Nadir (downward-facing) camera bridge
    nadir_cfg = cfg.get("nadir_camera", {})
    if px4.get("start_px4", True) and nadir_cfg.get("enabled", False):
        actions.append(
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                arguments=["/camera/nadir_raw"],
                output="screen",
                name="nadir_image_bridge",
            )
        )

    # Gimbal controller

    if px4.get("start_px4", True) and camera_cfg.get("enabled", False):
        gimbal_cfg = cam_full_cfg.get("gimbal", {})
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(_PROJECT_DIR / "scripts" / "gimbal_controller.py"),
                    "--ros-args",
                    "-p",
                    f"model_name:={gz_model_name}",
                    "-p",
                    f"default_pitch:={gimbal_cfg.get('default_pitch', 0.7854)}",
                    "-p",
                    "stabilize:=true",
                    "-p",
                    "publish_rate:=50.0",
                    "-p",
                    f"mode:={gimbal_cfg.get('mode', 'lock')}",
                    "-p",
                    f"deadband:={gimbal_cfg.get('deadband', 0.0)}",
                    "-p",
                    f"input_filter_hz:={gimbal_cfg.get('input_filter_hz', 0.0)}",
                    "-p",
                    f"follow_smoothing:={gimbal_cfg.get('follow_smoothing', 0.0)}",
                ],
                output="screen",
                name="gimbal_controller",
            )
        )

    # Wind turbulence node

    turb_cfg = wind.get("turbulence", {})
    if wind.get("enabled", False) and turb_cfg.get("enabled", False):
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(_PROJECT_DIR / "scripts" / "wind_turbulence.py"),
                    "--ros-args",
                    "-p",
                    f"mean_x:={wind.get('linear_velocity_x', 0.0)}",
                    "-p",
                    f"mean_y:={wind.get('linear_velocity_y', 0.0)}",
                    "-p",
                    f"mean_z:={wind.get('linear_velocity_z', 0.0)}",
                    "-p",
                    f"intensity:={turb_cfg.get('intensity', 0.8)}",
                    "-p",
                    f"correlation_time:={turb_cfg.get('correlation_time', 3.0)}",
                    "-p",
                    f"update_rate:={turb_cfg.get('update_rate', 10.0)}",
                    "-p",
                    f"world_name:={turb_cfg.get('world_name', 'inspection')}",
                    "-p",
                    f"odom_topic:=/model/{gz_model_name}/odometry",
                ],
                output="screen",
                name="wind_turbulence",
            )
        )

    return actions


# Launch description


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "level",
                default_value="full",
                choices=["minimal", "development", "full"],
                description=(
                    "fidelity level:\n"
                    "  minimal     – headless, basic sensors only\n"
                    "  development – GUI, sensors active, no noise or wind\n"
                    "  full        – GUI, noise, wind, lens distortion, realistic gimbal"
                ),
            ),
            DeclareLaunchArgument(
                "sensor_profile",
                default_value="auto",
                choices=["auto", "navigation", "vision", "hard_vision"],
                description=(
                    "sensor profile:\n"
                    "  auto        – default sensor set\n"
                    "  navigation  – lidar on, camera off\n"
                    "  vision      – camera on, lidar off\n"
                    "  hard_vision – 1280x720@30Hz camera + distortion + realistic gimbal, no lidar"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
