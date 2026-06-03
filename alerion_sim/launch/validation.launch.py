"""
validation.launch.py – Nodo de validación pasivo
=================================================
Ejecutar junto a simulation.launch.py mientras pilotas el dron manualmente.

Registra:
  - Desviación de pista cruzada desde la ruta de waypoints planificada  (por tick de odometría)
  - Carga computacional: CPU / RAM por proceso              (cada 5 s por defecto)

Uso
---
  ros2 launch alerion_sim validation.launch.py
  ros2 launch alerion_sim validation.launch.py compute_csv:=/tmp/run1_compute.csv
  ros2 launch alerion_sim validation.launch.py cpu_sample_hz:=0.1   # cada 10 s
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LAUNCH_DIR  = Path(__file__).resolve().parent
_PROJECT_DIR = _LAUNCH_DIR.parent
_PARAM_FILE  = str(_PROJECT_DIR / 'config' / 'validation.yaml')


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('model_name',      default_value='x500_0'),
        DeclareLaunchArgument('world_name',      default_value='inspection',
            description='Nombre del mundo Gazebo (para suscripción a stats RTF)'),
        DeclareLaunchArgument('status_interval', default_value='5.0',
            description='Segundos entre líneas de estado en consola'),
        DeclareLaunchArgument('log_file',        default_value='/alerion_sim/logs/alerion_validation.csv',
            description='CSV de telemetría por tick'),
        DeclareLaunchArgument('cpu_sample_hz',   default_value='0.2',
            description='Tasa de muestreo de cómputo (0.2 = cada 5 s)'),
        DeclareLaunchArgument('compute_csv',     default_value='/alerion_sim/logs/alerion_compute.csv',
            description='CSV de carga computacional (transmitido en tiempo real, volcado por fila)'),

        ExecuteProcess(
            cmd=[
                "python3",
                str(_PROJECT_DIR / "scripts" / "validation_node.py"),
                "--ros-args",
                "--params-file", _PARAM_FILE,
                "-p", ["model_name:=",      LaunchConfiguration("model_name")],
                "-p", ["world_name:=",      LaunchConfiguration("world_name")],
                "-p", ["status_interval:=", LaunchConfiguration("status_interval")],
                "-p", ["log_file:=",        LaunchConfiguration("log_file")],
                "-p", ["cpu_sample_hz:=",   LaunchConfiguration("cpu_sample_hz")],
                "-p", ["compute_csv:=",     LaunchConfiguration("compute_csv")],
            ],
            output="screen",
            name="validation_node",
        ),
    ])
