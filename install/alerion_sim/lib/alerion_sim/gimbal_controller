#!/usr/bin/env python3
"""
gimbal_controller.py – Estabilización de gimbal de 2 ejes y control manual


Lee la altitud del dron desde la odometría, calcula la corrección de roll/pitch necesaria para mantener la cámara apuntando a un ángulo fijo en el mundo,
publica comandos de posición de las articulaciones a los controladores del gimbal de Gazebo. Dos ejes de control: pitch (tilt) y roll (pan). El eje de yaw no se controla, lo que permite que la cámara gire libremente alrededor del eje vertical.


  nivel de desarrollo (mode=lock, deadband=0, no filters, follow_smoothing=0):
    Estabilizacion instantanea para el desarrollo sin sonido

  nivel completo (mode=follow, deadband=0.005, input_filter_hz=3, follow_smoothing=0.3):
    Imperfecciones realistas: latencia del canal RC, límite de resolución del encoder,
    movimiento de cámara orgánico suavizado.
    Casos realistas, latencia, limite de resolucion, y la camara se mueve de forma realista



Topics publicados (transimitido a puente con Gz a traves de simulation.launch.py  )
  /gimbal/roll_cmd   std_msgs/msg/Float64  → gz /model/x500_0/command/gimbal_roll
  /gimbal/pitch_cmd  std_msgs/msg/Float64  → gz /model/x500_0/command/gimbal_pitch

parametros modificados (en total 71 parametros pero estos son los que generan cambios sustanciales, la mayoria son alteraciones geometricas o finetuning del motor ODE)
  model_name         str    (default: x500_0)
  default_pitch      float  (default: -0.7854)
  stabilize          bool   (default: true) la funcion principal del gimbal
  publish_rate       float  (default: 50.0) publicacion cada cierta cantidad de hz
  mode               str    "lock" = hold abs. angle, "follow" = smooth tracking
                            (default: lock) lock → seguir angulo follow → trackear
  deadband           float  (default: 0.0) minimo de movimineto requierido para publicar una correcion
  input_filter_hz    float  frecuencia de corte del filtro paso bajo sobre el ángulo objetivo (0 = sin filtro)
  follow_smoothing   float  reducir la abruptez de la correcion generando un movimiento mas lento
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64



def quat_to_roll_pitch(q) -> tuple[float, float]:
    """devolver tupla (roll, pitch) en radianes deesde geometry_msgs → Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w

    
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    return roll, pitch



class LowPassFilter:
    """proceso usado con el gimbal para compensar movimientos causados por perturbaciones como vibraciones del motor o resistencia del aire"""

    def __init__(self, cutoff_hz: float, dt: float):
        
        self._passthrough = cutoff_hz <= 0.0
        if not self._passthrough:
            rc = 1.0 / (2.0 * math.pi * cutoff_hz)
            self._alpha = dt / (rc + dt)
        else:
            self._alpha = 1.0
        self._state: float | None = None   

    def update(self, x: float) -> float:
        if self._passthrough:
            return x
        if self._state is None:
            self._state = x          
        self._state += self._alpha * (x - self._state)
        return self._state

    def reset(self, value: float = 0.0):
        self._state = value




class GimbalController(Node):

    def __init__(self):
        super().__init__('gimbal_controller')

        self.declare_parameter('model_name',       'x500_0')
        self.declare_parameter('default_pitch',    -0.7854)   # -45°
        self.declare_parameter('stabilize',        True)
        self.declare_parameter('publish_rate',     50.0)
        self.declare_parameter('mode',             'lock')
        self.declare_parameter('deadband',         0.0)
        self.declare_parameter('input_filter_hz',  0.0)
        self.declare_parameter('follow_smoothing', 0.0)

        p = self.get_parameter
        self._model          = p('model_name').value
        self._def_pitch      = p('default_pitch').value
        self._stabilize      = p('stabilize').value
        rate                 = p('publish_rate').value
        self._mode           = p('mode').value
        self._deadband       = p('deadband').value
        input_filter_hz      = p('input_filter_hz').value
        follow_smoothing     = p('follow_smoothing').value

        dt = 1.0 / rate

        self._roll_setpt_filter  = LowPassFilter(input_filter_hz, dt)
        self._pitch_setpt_filter = LowPassFilter(input_filter_hz, dt)

        if follow_smoothing > 0.0:
            self._smooth_alpha = dt / (follow_smoothing + dt)
        else:
            self._smooth_alpha = 1.0   # sin suavizado → α=1 → actualización instantánea

        self._out_roll  = 0.0
        self._out_pitch = self._def_pitch

        self._drone_roll  = 0.0
        self._drone_pitch = 0.0
        self._set_roll    = 0.0
        self._set_pitch   = self._def_pitch

        odom_topic  = f'/model/{self._model}/odometry'
        pitch_topic = f'/model/{self._model}/command/gimbal_pitch'
        roll_topic  = f'/model/{self._model}/command/gimbal_roll'

        self.create_subscription(Odometry, odom_topic,          self._odom_cb,  10)
        self.create_subscription(Float64,  '/gimbal/set_pitch', self._pitch_cb, 10)
        self.create_subscription(Float64,  '/gimbal/set_roll',  self._roll_cb,  10)

        self._pub_roll  = self.create_publisher(Float64, roll_topic,  10)
        self._pub_pitch = self.create_publisher(Float64, pitch_topic, 10)

        self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f'Gimbal controller started\n'
            f'  Model        : {self._model}\n'
            f'  Mode         : {self._mode}\n'
            f'  Stabilize    : {self._stabilize}\n'
            f'  Default pitch: {math.degrees(self._def_pitch):.1f}°\n'
            f'  Deadband     : {math.degrees(self._deadband):.2f}°\n'
            f'  Input filter : {input_filter_hz:.1f} Hz\n'
            f'  Smoothing τ  : {follow_smoothing:.2f} s\n'
            f'  Odometry     : {odom_topic}\n'
            f'  Pitch cmd    : {pitch_topic}\n'
            f'  Roll cmd     : {roll_topic}\n'
            f'  Set pitch    : /gimbal/set_pitch  (Float64, rad)\n'
            f'  Set roll     : /gimbal/set_roll   (Float64, rad)\n'
        )


    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self._drone_roll, self._drone_pitch = quat_to_roll_pitch(q)

    def _pitch_cb(self, msg: Float64):
        self._set_pitch = float(msg.data)
        self.get_logger().info(f'Gimbal pitch target → {math.degrees(self._set_pitch):.1f}°')

    def _roll_cb(self, msg: Float64):
        self._set_roll = float(msg.data)
        self.get_logger().info(f'Gimbal roll target → {math.degrees(self._set_roll):.1f}°')



    def _control_loop(self):
        
        roll_target  = self._roll_setpt_filter.update(self._set_roll)
        pitch_target = self._pitch_setpt_filter.update(self._set_pitch)

        if self._stabilize:
            roll_cmd  = roll_target  - self._drone_roll
            pitch_cmd = pitch_target - self._drone_pitch
        else:
            roll_cmd  = roll_target
            pitch_cmd = pitch_target

        if self._deadband > 0.0:
            if abs(roll_cmd)  < self._deadband:
                roll_cmd  = 0.0
            if abs(pitch_cmd) < self._deadband:
                pitch_cmd = 0.0

        self._out_roll  += self._smooth_alpha * (roll_cmd  - self._out_roll)
        self._out_pitch += self._smooth_alpha * (pitch_cmd - self._out_pitch)

        self._pub_roll.publish(Float64(data=self._out_roll))
        self._pub_pitch.publish(Float64(data=self._out_pitch))



def main(args=None):
    rclpy.init(args=args)
    node = GimbalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
