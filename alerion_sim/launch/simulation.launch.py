"""
setup del simulador parametrizable

  ros2 launch alerion_sim simulation.launch.py                        maxima fidelidad
  ros2 launch alerion_sim simulation.launch.py level:=development     modo de desarrollo, la mayoria de los sistemas presentes pero no hay ningun plugin que genere sonido ni perturbaciones
  ros2 launch alerion_sim simulation.launch.py level:=minimal         modelo mas basico, computacionalmente el mas barato
  
  preset de diferentes tipos de sensores
  
  ros2 launch alerion_sim simulation.launch.py level:=development sensor_profile:=vision        
  ros2 launch alerion_sim simulation.launch.py level:=development sensor_profile:=navigation  

  1. Lee config/simulation.yaml  (configuracion base, sobreescrito por los config files sobre fidelidad)
  2. Deep-merge config/levels/<level>.yaml 
  3. Merge config/sensors/*.yaml / config/vehicle/x500.yaml
  4. Sobreescritura de sensor_profile si esta presente
  5. Renderiza worlds/inspection.sdf.j2   → /tmp/inspection_<level>.sdf
  6. Renderiza models/x500_inspection/model.sdf.jinja → /tmp/x500_<level>.sdf
  7. Inicializacion:
       - MicroXRCE-DDS agent     solo development + full (sive para capturar ciertos mensajes internos (uOrb) usados por PX4 para su tratamiento, no confundir con ros_gz_bridge)
       - Gazebo Sim              todos
       - ros_gz_sim create       spawnea "x500_0" en t=3 s (esto es para que no carge el dron antes que el mundo, seguramente se pueda quitar el timer)
       - PX4 SITL process        development + full, si start_px4: true en t=6 s ("")
         PX4_SIM_MODEL=gz_x500   usamos los param del modelo default para nuestro modelo x500_0
         PX4_GZ_MODEL_NAME=x500_0 conectamos PX4 con nuestro modelo custom _0
         → gz_bridge start -w inspection -n x500_0  sensores
       - ros_gz_bridge           (siempre mandando /clock + odometry; + sensors si el preset esta activo)
"""

import copy
import os
import tempfile
from pathlib import Path

import yaml
from jinja2 import Environment, FileSystemLoader

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ---------------------------------------------------------------------------
# rutas del proyecto
# ---------------------------------------------------------------------------

_LAUNCH_DIR  = Path(__file__).resolve().parent
_PROJECT_DIR = _LAUNCH_DIR.parent
_CONFIG_DIR  = _PROJECT_DIR / "config"
_MODELS_DIR  = _PROJECT_DIR / "models"
_WORLDS_DIR  = _PROJECT_DIR / "worlds"

PX4_DIR        = Path(os.environ.get("PX4_DIR", Path.home() / "Desktop" / "PX4-Autopilot"))
PX4_MODELS_DIR = PX4_DIR / "Tools" / "simulation" / "gz" / "models"
PX4_WORLDS_DIR = PX4_DIR / "Tools" / "simulation" / "gz" / "worlds"
# Recursos de modelos de Gazebo-classic (estantes, cajas, palés) 
PX4_CLASSIC_MODELS_DIR = (PX4_DIR / "Tools" / "simulation" /
                           "gazebo-classic" / "sitl_gazebo-classic" / "models")


#funciones para la carga de los config y el override correcto

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
    Como funcionan los configs en orden (el cada uno sobreescribe el anterior):
      1. config/simulation.yaml           – base default
      2. config/sensors/*.yaml            – base default hardware para los sensores (lidar, cam)
      3. config/vehicle/x500.yaml         – config del modelo
      4. config/levels/<level>.yaml       – sobreescritura dependiendo de la fidelidad ejecutada
      5. config/profiles/<profile>.yaml   – profile YAML override (if file exists)
      6. sensor_profile inline override   – navigation | vision | auto (legacy toggles)
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
        cfg.setdefault("sensors", {}).setdefault("lidar",  {})["enabled"] = True
        cfg.setdefault("sensors", {}).setdefault("camera", {})["enabled"] = False
    elif sensor_profile == "vision":
        cfg.setdefault("sensors", {}).setdefault("lidar",  {})["enabled"] = False
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



def launch_setup(context, *args, **kwargs):
    level          = LaunchConfiguration("level").perform(context)
    sensor_profile = LaunchConfiguration("sensor_profile").perform(context)

    cfg   = load_config(level, sensor_profile)
    sim   = cfg.get("simulation", {})
    phys  = cfg.get("physics",    {})
    rend  = cfg.get("rendering",  {})
    sens  = cfg.get("sensors",    {})
    wind  = cfg.get("wind",       {})
    px4   = sim.get("px4",        {})
    dds   = sim.get("dds",        {})
    spawn = sim.get("vehicle",    {}).get("spawn", {})

    lidar_cfg  = sens.get("lidar",  {})
    camera_cfg = sens.get("camera", {})

    # 1 render del mundo

    world_ctx = {
        "physics_enabled"  : phys.get("enabled", True),
        "physics_gravity_z": phys.get("gravity_z", -9.81),
        "physics_rtf"      : phys.get("real_time_factor", 1.0),
        "physics_step"     : phys.get("max_step_size", 0.004),
        "rendering_enabled": rend.get("enabled", True),
        "rendering_shadows": rend.get("shadows", True),
        "rendering_pbr"    : rend.get("pbr",     False),
        "wind_enabled"     : wind.get("enabled", False),
        "wind_vel_x"       : wind.get("linear_velocity_x", 0.0),
        "wind_vel_y"       : wind.get("linear_velocity_y", 0.0),
        "wind_vel_z"       : wind.get("linear_velocity_z", 0.0),
    }
    rendered_world = render_template(
        _WORLDS_DIR / "inspection.sdf.j2", world_ctx
    )

    # 2 Render modelo SDF  


    rendered_model = None
    if px4.get("start_px4", True):
        model_ctx = {
            "sensors_lidar_enabled" : lidar_cfg.get("enabled",  False),
            "sensors_lidar_noise"   : lidar_cfg.get("noise",    False),
            "sensors_camera_enabled": camera_cfg.get("enabled", False),
            "sensors_camera_noise"  : camera_cfg.get("noise",   False),
            "sensors_nadir_enabled" : cfg.get("nadir_camera", {}).get("enabled", False),
            "physics_enabled"       : phys.get("enabled", True),
            "lidar"                 : cfg.get("lidar",  {}),
            "camera"                : cfg.get("camera", {}),
            "nadir_camera"          : cfg.get("nadir_camera", {}),
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


    # 3 Inicializar variables para Gazebo

    """
    Variables de entorno que necesita Gazebo

    # GZ_SIM_RESOURCE_PATH  – modelos y mundos que busca
    # GZ_SIM_SYSTEM_PLUGIN_PATH – plugins customizados de px4
    # GZ_SIM_SERVER_CONFIG_PATH – PX4 server.config carga los sensores(???)

    Si no se establecen, los sensores no publican nada

    """

    gz_plugins_dir = str(PX4_DIR / "build" / "px4_sitl_default" /
                         "src" / "modules" / "simulation" / "gz_plugins")
    gz_server_cfg  = str(PX4_DIR / "src" / "modules" /
                         "simulation" / "gz_bridge" / "server.config")

    actions.append(SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join(filter(None, [
            str(PX4_MODELS_DIR),
            str(PX4_WORLDS_DIR),
            str(_MODELS_DIR),
            str(PX4_CLASSIC_MODELS_DIR),   # shelves, big_box, europallet DAE meshes
            str(_PROJECT_DIR),             # expone media/materials/scripts/gazebo.material
                                           # stub que silencia errores URI de Ogre1 de
                                           # referencias de mallas DAE de gazebo-classic
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ])),
    ))
    actions.append(SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=":".join(filter(None, [
            gz_plugins_dir,
            os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
        ])),
    ))
    actions.append(SetEnvironmentVariable(
        name="GZ_SIM_SERVER_CONFIG_PATH",
        value=gz_server_cfg,
    ))



    # 4 agente MicroXRCEAgent

    """
    MicroXRCE-DDS agent  (puente/publicador de mensajes PX4 uORB ↔ ROS 2 DDS)
    En minimal no existe (no hay PX4 SITL)
    """

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
        actions.append(ExecuteProcess(
            cmd=[
                "MicroXRCEAgent",
                dds.get("transport", "udp4"),
                "-p", str(dds.get("port", 8888)),
            ],
            output="screen",
            name="micro_xrce_dds",
        ))

    # 5 Activamos Gazebo sim

    actions.append(ExecuteProcess(
        cmd=["gz", "sim", rendered_world] + gz_args,
        additional_env={"GZ_LOG_LEVEL": "3"},
        output="screen",
        name="gazebo",
    ))

    # Inicializamos PX4, con delay para que gazebo y modelo dron esten preparados para hacer
    # conexion 

    if px4.get("start_px4", True):
        px4_firmware_dir = Path(px4.get("firmware_dir") or str(PX4_DIR))
        px4_bin    = str(px4_firmware_dir / "build" / "px4_sitl_default" / "bin" / "px4")
        px4_rootfs = str(px4_firmware_dir / "build" / "px4_sitl_default" / "rootfs")
        px4_build    = str(px4_firmware_dir / "build" / "px4_sitl_default")
        instance     = str(px4.get("instance", 0))

        gz_model_instance = f"x500_{px4.get('instance', 0)}"

        # A  Spawneamos nuestro modelo a t=3 para que este el sistema listo para captar la conexion

        actions.append(TimerAction(
            period=3.0,
            actions=[Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-file", rendered_model,
                    "-name", gz_model_instance,
                    "-x", str(spawn.get("x", 0.0)),
                    "-y", str(spawn.get("y", 0.0)),
                    "-z", str(spawn.get("z", 0.5)),
                    "-Y", str(spawn.get("yaw", 0.0)),
                ],
                output="screen",
                name="spawn_vehicle",
            )],
        ))

        # B  PX4 SITL a t=6  (modelo ya presente)

        actions.append(TimerAction(
            period=6.0,
            actions=[ExecuteProcess(
                cmd=[
                    px4_bin,
                    px4_rootfs,
                    "-s", "etc/init.d-posix/rcS",
                    "-i", instance,
                    "-d",
                ],
                cwd=px4_build,
                additional_env={
                    "PX4_SIM_MODEL":      "gz_x500",
                    "PX4_GZ_MODEL_NAME":  gz_model_instance,
                    "PX4_GZ_WORLD":       "inspection",
                    "PX4_UXRCE_DDS_PORT": str(dds.get("port", 8888)),
                    "PX4_GZ_MODELS":      str(PX4_MODELS_DIR),
                    "PX4_GZ_WORLDS":      str(PX4_WORLDS_DIR),
                },
                output="screen",
                name="px4_sitl",
            )],
        ))

    
    # 6. ROS ↔ Gazebo puente

    #TODO: check con otro sys para ver si los puentes funcionan o hay que estandarizar(?) el puente
    gz_model_name = f"x500_{px4.get('instance', 0)}"
    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    if px4.get("start_px4", True):
        bridge_args.append(
            f"/model/{gz_model_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"
        )
        if lidar_cfg.get("enabled", False):
            bridge_args.append("/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan")
            bridge_args.append(
                "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
            )
        if camera_cfg.get("enabled", False):
            bridge_args += [
                "/camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                f"/model/{gz_model_name}/command/gimbal_roll"
                f"@std_msgs/msg/Float64]gz.msgs.Double",
                f"/model/{gz_model_name}/command/gimbal_pitch"
                f"@std_msgs/msg/Float64]gz.msgs.Double",
            ]

    actions.append(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_args,
        output="screen",
        name="ros_gz_bridge",
    ))

    
    # Puente de imagen – nodo dedicado para los píxeles de la cámara
    # parameter_bridge tiene problemas de conversión de formato de píxel
    # con gz.msgs.Image en Gazebo Harmonic; ros_gz_image/image_bridge gestiona


    if px4.get("start_px4", True) and camera_cfg.get("enabled", False):
        actions.append(Node(
            package="ros_gz_image",
            executable="image_bridge",
            arguments=["/camera/image_raw"],
            output="screen",
            name="camera_image_bridge",
        ))

    # Postprocesador de distorsión de cámara
    # Este nodo aplica distorsión Brown-Conrady via remap de OpenCV después de que
    # el fotograma limpio se puentea a ROS 2.
    # Publica /camera/image_distorted junto al /camera/image_raw limpio.
    # QT_QPA_PLATFORM=xcb ros2 run rqt_image_view rqt_image_view /camera/image_raw       para ver la imagen limpia
    # QT_QPA_PLATFORM=xcb ros2 run rqt_image_view rqt_image_view /camera/image_distorted para ver la imagen con distorsión

    cam_full_cfg = cfg.get("camera", {})
    _dist_ok = (px4.get("start_px4", True)
                and camera_cfg.get("enabled", False)
                and cam_full_cfg.get("distortion_enabled", False))
    print(f"[alerion_sim] Distortion node: {'SÍ' if _dist_ok else 'NO'} "
          f"(start_px4={px4.get('start_px4', True)}, "
          f"camera.enabled={camera_cfg.get('enabled', False)}, "
          f"distortion_enabled={cam_full_cfg.get('distortion_enabled', False)})")
    if _dist_ok:
        dist = cam_full_cfg.get("distortion", {})
        actions.append(ExecuteProcess(
            cmd=[
                "python3",
                str(_PROJECT_DIR / "scripts" / "camera_distortion.py"),
                "--ros-args",
                "-p", f"k1:={dist.get('k1',  0.0)}",
                "-p", f"k2:={dist.get('k2',  0.0)}",
                "-p", f"k3:={dist.get('k3',  0.0)}",
                "-p", f"p1:={dist.get('p1',  0.0)}",
                "-p", f"p2:={dist.get('p2',  0.0)}",
            ],
            output="screen",
            name="camera_distortion",
        ))

    # Cámara nadir (orientada hacia abajo) – solo nivel full
    nadir_cfg = cfg.get("nadir_camera", {})
    if px4.get("start_px4", True) and nadir_cfg.get("enabled", False):
        actions.append(Node(
            package="ros_gz_image",
            executable="image_bridge",
            arguments=["/camera/nadir_raw"],
            output="screen",
            name="nadir_image_bridge",
        ))

    # 7 Inicializamos nodo de control para Gimbal

    if px4.get("start_px4", True) and camera_cfg.get("enabled", False):
        gimbal_cfg = camera_cfg.get("gimbal", {})
        actions.append(ExecuteProcess(
            cmd=[
                "python3",
                str(_PROJECT_DIR / "scripts" / "gimbal_controller.py"),
                "--ros-args",
                "-p", f"model_name:={gz_model_name}",
                "-p", f"default_pitch:={gimbal_cfg.get('default_pitch', 0.7854)}",
                "-p", "stabilize:=true",
                "-p", "publish_rate:=50.0",
                "-p", f"mode:={gimbal_cfg.get('mode', 'lock')}",
                "-p", f"deadband:={gimbal_cfg.get('deadband', 0.0)}",
                "-p", f"input_filter_hz:={gimbal_cfg.get('input_filter_hz', 0.0)}",
                "-p", f"follow_smoothing:={gimbal_cfg.get('follow_smoothing', 0.0)}",
            ],
            output="screen",
            name="gimbal_controller",
        ))

    # 8 turbulencia viento

    turb_cfg = wind.get("turbulence", {})
    if wind.get("enabled", False) and turb_cfg.get("enabled", False):
        actions.append(ExecuteProcess(
            cmd=[
                "python3",
                str(_PROJECT_DIR / "scripts" / "wind_turbulence.py"),
                "--ros-args",
                "-p", f"mean_x:={wind.get('linear_velocity_x', 0.0)}",
                "-p", f"mean_y:={wind.get('linear_velocity_y', 0.0)}",
                "-p", f"mean_z:={wind.get('linear_velocity_z', 0.0)}",
                "-p", f"intensity:={turb_cfg.get('intensity', 0.8)}",
                "-p", f"correlation_time:={turb_cfg.get('correlation_time', 3.0)}",
                "-p", f"update_rate:={turb_cfg.get('update_rate', 10.0)}",
                "-p", f"world_name:={turb_cfg.get('world_name', 'inspection')}",
            ],
            output="screen",
            name="wind_turbulence",
        ))

    return actions



# LaunchDescription


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "level",
            default_value="full",
            choices=["minimal", "development", "full"],
            description=(
                "fidelity level:\n"
                "  minimal     – fidelidad minima, ni fisicas ni renderizado\n"
                "  development – fidelidad general, sin sonido y modulos basicos\n"
                "  full        – fidelidad maxima, sonido + viento"
            ),
        ),
        DeclareLaunchArgument(
            "sensor_profile",
            default_value="auto",
            choices=["auto", "navigation", "vision", "hard_vision"],
            description=(
                "sensor profile:\n"
                "  auto        – basico\n"
                "  navigation  – con lidar, sin camara\n"
                "  vision      – con camara, sin lidar\n"
                "  hard_vision – camara 1280x720@30Hz + distorsion + gimbal realista "
                "(sin lidar, sobre development)"
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
