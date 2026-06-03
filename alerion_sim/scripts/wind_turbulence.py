#!/usr/bin/env python3
"""
wind_turbulence.py – Turbulencia atmosférica de Dryden para el viento de Gazebo
================================================================================
Inyecta ruido de viento  publicando directamente en el topic de viento del mundo
mediante los bindings Python de gz-transport.

Modelo de turbulencia
---------------------
Filtro conformador de Dryden

    u[k] = α · u[k-1]  +  β · w[k]

    α = 1 - dt/τ          
    β = σ · √(2·dt/τ)     
    w[k] ~ N(0, 1)


Parámetros
----------
  mean_x / mean_y / mean_z   float  Vector de viento medio (constante) (m/s, ENU)
  intensity                  float  Intensidad de turbulencia σ por eje (m/s)
  correlation_time           float  Tiempo de correlación de Dryden τ (s)
  update_rate                float  Hz a los que se actualiza el viento de Gazebo
  world_name                 str    Nombre del mundo de Gazebo
"""

import math
import random

import gz.transport13 as gz_transport
from gz.msgs10.wind_pb2 import Wind

import rclpy
from rclpy.node import Node


class WindTurbulenceNode(Node):

    def __init__(self):
        super().__init__('wind_turbulence')

        self.declare_parameter('mean_x',           0.0)
        self.declare_parameter('mean_y',           0.0)
        self.declare_parameter('mean_z',           0.0)
        self.declare_parameter('intensity',        0.8)
        self.declare_parameter('correlation_time', 3.0)
        self.declare_parameter('update_rate',      10.0)
        self.declare_parameter('world_name',       'inspection')

        p             = self.get_parameter
        self._mean_x  = p('mean_x').value
        self._mean_y  = p('mean_y').value
        self._mean_z  = p('mean_z').value
        sigma         = p('intensity').value
        tau           = p('correlation_time').value
        rate          = p('update_rate').value
        self._world   = p('world_name').value

        dt = 1.0 / rate

        # Coeficientes del filtro discreto de Dryden
        # α ajustada a [0, 1] para mantener estabilidad numérica si dt ≥ τ
        self._alpha = max(0.0, min(1.0, 1.0 - dt / tau))
        self._beta  = sigma * math.sqrt(2.0 * dt / tau)

        # Estado del filtro: componente de turbulencia por eje (media cero)
        self._u = [0.0, 0.0, 0.0]

        # Publicador gz-transport — persiste durante la vida del nodo
        gz_topic = f'/world/{self._world}/wind'
        self._gz_node = gz_transport.Node()
        self._pub     = self._gz_node.advertise(gz_topic, Wind)


        self.create_timer(dt, self._tick)

        self.get_logger().info(
            f'Wind turbulence started\n'
            f'  Mean wind : ({self._mean_x:.2f}, {self._mean_y:.2f}, '
            f'{self._mean_z:.2f}) m/s\n'
            f'  Intensity : σ = {sigma:.2f} m/s per axis\n'
            f'  Corr. time: τ = {tau:.1f} s  '
            f'(α={self._alpha:.4f}, β={self._beta:.4f})\n'
            f'  Rate      : {rate:.1f} Hz\n'
            f'  Gz topic  : {gz_topic}\n'
        )

    # ------------------------------------------------------------------
    def _dryden_step(self) -> tuple[float, float, float]:
        """Advance the filter one step; return (ux, uy, uz) turbulence."""
        for i in range(3):
            w = random.gauss(0.0, 1.0)
            self._u[i] = self._alpha * self._u[i] + self._beta * w
        return (self._u[0], self._u[1], self._u[2])

    # ------------------------------------------------------------------
    def _tick(self):
        ux, uy, uz = self._dryden_step()

        msg = Wind()
        msg.linear_velocity.x = self._mean_x + ux
        msg.linear_velocity.y = self._mean_y + uy
        msg.linear_velocity.z = self._mean_z + uz

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WindTurbulenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
