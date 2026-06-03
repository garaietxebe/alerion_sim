#!/usr/bin/env python3
"""
validation_node.py — Instrumento de validación pasivo
======================================================
Observa un vuelo pilotado manualmente y registra:

  1. DESVIACIÓN DE RUTA
     Error de pista cruzada (m): distancia perpendicular desde la posición
     actual del dron hasta el punto más cercano de la polilínea de waypoints
     planificada. Se escribe en el CSV de telemetría en cada tick de odometría.

  2. CARGA COMPUTACIONAL
     CPU / RAM del sistema más desglose por proceso, muestreado cada
     `cpu_sample_hz` Hz (default 0.2 Hz = cada 5 s) y transmitido en tiempo
     real al CSV de cómputo para que el archivo esté completo aunque se cierre
     el nodo.

Sin control de vuelo, sin mensajes PX4, sin armado. Solo medición.

Suscripciones
-------------
  /model/<model_name>/odometry    nav_msgs/msg/Odometry   (posición real de Gazebo)

Archivos de salida
------------------
  log_file      – CSV por tick: velocidad, CTE
  compute_csv   – CSV por muestra: rtf, CTE, cpu%, mem_MB, proceso por columna
"""

import csv
import math
import re
import signal
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

try:
    import psutil
except ImportError:
    psutil = None

try:
    import gz.transport13 as gz_transport
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    _HAS_GZ = True
except ImportError:
    _HAS_GZ = False


# ---------------------------------------------------------------------------
# Geometría
# ---------------------------------------------------------------------------

def _speed(twist) -> float:
    v = twist.linear
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def _euclid(a, b) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def _point_segment_dist(p, a, b) -> float:
    """Distancia perpendicular del punto p al segmento a→b (extremos limitados)."""
    abx, aby, abz = b[0]-a[0], b[1]-a[1], b[2]-a[2]
    ab2 = abx*abx + aby*aby + abz*abz
    if ab2 < 1e-9:
        return _euclid(p, a)
    t = ((p[0]-a[0])*abx + (p[1]-a[1])*aby + (p[2]-a[2])*abz) / ab2
    t = max(0.0, min(1.0, t))
    cx, cy, cz = a[0]+t*abx, a[1]+t*aby, a[2]+t*abz
    return math.sqrt((p[0]-cx)**2 + (p[1]-cy)**2 + (p[2]-cz)**2)


# ---------------------------------------------------------------------------
# Muestreador de cómputo (psutil)
# ---------------------------------------------------------------------------

class ComputeSampler:
    def __init__(self, patterns: list[str]):
        self._patterns   = [re.compile(p) for p in patterns]
        self._labels     = patterns          # mantiene orden para columnas CSV
        self._proc_cache = {}               # pid -> (Process, etiqueta)
        self._last_scan  = 0.0
        if psutil:
            psutil.cpu_percent(interval=None)

    def _refresh(self):
        if not psutil:
            return
        seen = set()
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmd = ' '.join(proc.info['cmdline'] or [proc.info['name'] or ''])
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
            for pat in self._patterns:
                if pat.search(cmd):
                    pid = proc.info['pid']
                    if pid not in self._proc_cache:
                        try:
                            ps = psutil.Process(pid)
                            ps.cpu_percent(interval=None)   # inicializa contador
                            self._proc_cache[pid] = (ps, pat.pattern)
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            pass
                    seen.add(pid)
                    break
        for pid in list(self._proc_cache):
            if pid not in seen:
                del self._proc_cache[pid]

    def sample(self) -> dict | None:
        """Devuelve un dict snapshot o None si psutil no está disponible."""
        if not psutil:
            return None
        now = time.monotonic()
        if now - self._last_scan > 5.0:
            self._refresh()
            self._last_scan = now

        sys_cpu = psutil.cpu_percent(interval=None) * psutil.cpu_count()
        vm      = psutil.virtual_memory()

        # agrega por etiqueta (varios PIDs pueden coincidir con el mismo patrón)
        per_proc: dict[str, tuple[float, float]] = {}
        for _pid, (proc, label) in list(self._proc_cache.items()):
            try:
                cpu    = proc.cpu_percent(interval=None)
                mem_mb = proc.memory_info().rss / (1024 * 1024)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                del self._proc_cache[_pid]
                continue
            if label in per_proc:
                c, m = per_proc[label]
                per_proc[label] = (c + cpu, m + mem_mb)
            else:
                per_proc[label] = (cpu, mem_mb)

        return {
            'sys_cpu_pct': sys_cpu,
            'sys_mem_mb':  vm.used / (1024 * 1024),
            'per_proc':    per_proc,   # etiqueta -> (cpu%, mem_MB)
        }


# ---------------------------------------------------------------------------
# Nodo
# ---------------------------------------------------------------------------

class ValidationNode(Node):

    def __init__(self):
        super().__init__('validation_node')

        # -- Parámetros -------------------------------------------------------
        self.declare_parameter('model_name',      'x500_0')
        self.declare_parameter('world_name',      'inspection')
        self.declare_parameter('log_file',        '/alerion_sim/logs/alerion_validation.csv')
        self.declare_parameter('compute_csv',     '/alerion_sim/logs/alerion_compute.csv')
        self.declare_parameter('status_interval',  5.0)
        self.declare_parameter('waypoints',       [0.0])
        self.declare_parameter('cpu_sample_hz',    0.2)     # cada 5 s
        self.declare_parameter('target_processes', [
            'gz sim', 'px4', 'MicroXRCEAgent',
            'parameter_bridge', 'image_bridge',
            'gimbal_controller', 'validation_node',
        ])

        g = self.get_parameter
        self._model      = g('model_name').value
        self._world      = g('world_name').value
        self._log_file   = g('log_file').value
        self._compute_csv_path = g('compute_csv').value
        self._status_dt  = g('status_interval').value
        wp_flat          = list(g('waypoints').value)
        cpu_hz           = g('cpu_sample_hz').value
        patterns         = list(g('target_processes').value)

        # -- RTF desde gz stats -----------------------------------------------
        self._last_rtf  = 1.0
        self._rtf_lock  = threading.Lock()
        self._gz_node   = None
        if _HAS_GZ:
            self._gz_node = gz_transport.Node()
            self._gz_node.subscribe(
                WorldStatistics,
                f'/world/{self._world}/stats',
                self._gz_stats_cb,
            )
        else:
            self.get_logger().warn(
                'gz.transport13 no encontrado — columna RTF valdrá 1.0 (no disponible).'
            )

        # -- Parseo de waypoints (grupos [x, y, z, tol]) ---------------------
        self._waypoints: list[tuple] = []
        if len(wp_flat) >= 4 and len(wp_flat) % 4 == 0:
            for i in range(0, len(wp_flat), 4):
                x, y, z, tol = wp_flat[i:i+4]
                self._waypoints.append((float(x), float(y), float(z), float(tol)))
        else:
            self.get_logger().warn(
                'No hay waypoints válidos configurados — el CTE se medirá '
                'relativo a la posición inicial del dron.')

        # -- Puntos de la polilínea de ruta (construidos desde la primera odometría) --
        self._origin     = None          # (x,y,z) del primer sample de odom
        self._seg_pts: list[tuple] = []  # [origen, WP1, WP2, …]
        self._seg_idx    = 0             # índice del segmento actual
        self._target_idx = 0             # índice del próximo waypoint

        # -- Estado de telemetría ---------------------------------------------
        self._t0        = time.monotonic()
        self._odom_n    = 0
        self._last_pos  = (0.0, 0.0, 0.0)
        self._last_spd  = 0.0
        self._last_cte  = 0.0

        # -- Muestreador de cómputo -------------------------------------------
        self._sampler = ComputeSampler(patterns) if psutil else None
        self._labels  = patterns    # conserva para orden de columnas CSV
        if not psutil:
            self.get_logger().warn('psutil no instalado — monitorización de cómputo DESACTIVADA.')

        # -- CSV de telemetría (por tick) -------------------------------------
        tel_path = Path(self._log_file)
        tel_path.parent.mkdir(parents=True, exist_ok=True)
        self._tel_fh  = open(tel_path, 'w', newline='')
        self._tel_csv = csv.writer(self._tel_fh)
        self._tel_csv.writerow([
            'speed_mps', 'cte_m',
        ])

        # -- CSV de cómputo (por muestra, transmitido en tiempo real) --------
        cmp_path = Path(self._compute_csv_path)
        cmp_path.parent.mkdir(parents=True, exist_ok=True)
        self._cmp_fh  = open(cmp_path, 'w', newline='')
        self._cmp_csv = csv.writer(self._cmp_fh)
        # cabecera escrita una vez; columnas por proceso añadidas tras los patrones
        header = [
            'rtf',
            'cte_m',
            'sys_cpu_pct', 'sys_mem_mb',
        ]
        for lbl in self._labels:
            header += [f'{lbl}_cpu_pct', f'{lbl}_mem_mb']
        self._cmp_csv.writerow(header)
        self._cmp_fh.flush()

        # -- Suscripciones / timers ------------------------------------------
        odom_topic = f'/model/{self._model}/odometry'
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_timer(1.0 / max(cpu_hz, 1e-3), self._sample_compute)
        self.create_timer(self._status_dt,            self._print_status)

        self.get_logger().info(
            f'\n  Nodo de validación iniciado (pasivo — sin control de vuelo)\n'
            f'  Modelo       : {self._model}\n'
            f'  Odometría    : {odom_topic}\n'
            f'  Waypoints    : {len(self._waypoints)}\n'
            f'  Tasa cómputo : cada {1/max(cpu_hz,1e-3):.0f} s  →  {self._compute_csv_path}\n'
            f'  Telemetría   : {self._log_file}\n'
            f'  Vuela manualmente — Ctrl+C imprime informe final y cierra archivos.\n'
        )

    # -----------------------------------------------------------------------
    # Ayudantes de segmento
    # -----------------------------------------------------------------------

    def _build_route(self, origin):
        self._origin   = origin
        self._seg_pts  = [origin] + [(w[0], w[1], w[2]) for w in self._waypoints]
        self._seg_idx  = 0
        self._target_idx = 0

    def _current_seg(self):
        if len(self._seg_pts) < 2:
            return self._origin or (0.0, 0.0, 0.0), self._origin or (0.0, 0.0, 0.0)
        i = min(self._seg_idx, len(self._seg_pts) - 2)
        return self._seg_pts[i], self._seg_pts[i + 1]

    # -----------------------------------------------------------------------
    # Callback de odometría
    # -----------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        elapsed = time.monotonic() - self._t0
        self._odom_n += 1

        pos = msg.pose.pose.position
        p   = (pos.x, pos.y, pos.z)
        spd = _speed(msg.twist.twist)

        # construye la ruta en el primer mensaje (origen = punto real de despegue)
        if self._origin is None:
            self._build_route(p)

        # error de pista cruzada contra el segmento planificado actual
        a, b = self._current_seg()
        cte  = _point_segment_dist(p, a, b)

        # llegada a waypoint → avanza segmento
        if self._target_idx < len(self._waypoints):
            tx, ty, tz, tol = self._waypoints[self._target_idx]
            if _euclid(p, (tx, ty, tz)) <= tol:
                self.get_logger().info(
                    f'[{elapsed:.1f}s] WP{self._target_idx+1} alcanzado  '
                    f'(CTE en llegada: {cte:.2f} m)')
                self._target_idx += 1
                self._seg_idx = self._target_idx   # pasa al segmento siguiente

        self._last_pos = p
        self._last_spd = spd
        self._last_cte = cte

        self._tel_csv.writerow([
            f'{spd:.4f}', f'{cte:.4f}',
        ])

    # -----------------------------------------------------------------------
    # Muestra de cómputo → transmitida directamente al CSV
    # -----------------------------------------------------------------------

    def _gz_stats_cb(self, msg: 'WorldStatistics'):
        with self._rtf_lock:
            self._last_rtf = msg.real_time_factor

    def _sample_compute(self):
        if self._origin is None:
            return    # no muestrea hasta ver el dron
        snap = self._sampler.sample() if self._sampler else None
        with self._rtf_lock:
            rtf = self._last_rtf

        if snap is None:
            row = [
                f'{rtf:.3f}',
                f'{self._last_cte:.3f}', '', '']
            for _ in self._labels:
                row += ['', '']
        else:
            row = [
                f'{rtf:.3f}',
                f'{self._last_cte:.3f}',
                f'{snap["sys_cpu_pct"]:.1f}',
                f'{snap["sys_mem_mb"]:.0f}',
            ]
            for lbl in self._labels:
                cpu, mem = snap['per_proc'].get(lbl, (0.0, 0.0))
                row += [f'{cpu:.1f}', f'{mem:.0f}']

        self._cmp_csv.writerow(row)
        self._cmp_fh.flush()    # vuelca cada fila para que los datos sobrevivan a un cierre abrupto

    # -----------------------------------------------------------------------
    # Estado en tiempo real
    # -----------------------------------------------------------------------

    def _print_status(self):
        if self._origin is None:
            self.get_logger().info('Esperando primer mensaje de odometría…')
            return

        elapsed = time.monotonic() - self._t0
        x, y, z = self._last_pos
        snap     = self._sampler.sample() if self._sampler else None

        cpu_str = f'{snap["sys_cpu_pct"]:.1f}%' if snap else 'n/a'
        mem_str = f'{snap["sys_mem_mb"]:.0f} MB' if snap else 'n/a'

        wp_label = (f'→WP{self._target_idx+1}/{len(self._waypoints)}'
                    if self._target_idx < len(self._waypoints) else '✓ completado')

        procs = ''
        if snap and snap['per_proc']:
            top = sorted(snap['per_proc'].items(), key=lambda kv: -kv[1][0])[:4]
            procs = '  [' + '  '.join(f'{l}={c:.0f}%' for l, (c, _) in top) + ']'

        print(f'[{elapsed:7.1f}s] seg#{self._seg_idx} {wp_label:12s} '
              f'pos=({x:6.2f},{y:6.2f},{z:5.2f})  '
              f'v={self._last_spd:.1f}m/s  CTE={self._last_cte:.2f}m  '
              f'CPU={cpu_str}  MEM={mem_str}{procs}')

    # -----------------------------------------------------------------------
    # Informe final (en Ctrl+C / SIGTERM)
    # -----------------------------------------------------------------------

    def report(self):
        elapsed = time.monotonic() - self._t0
        sep = '=' * 76

        # lee el CSV de cómputo para estadísticas de resumen
        cte_all: list[float] = []
        try:
            with open(self._log_file, newline='') as fh:
                for row in csv.DictReader(fh):
                    try:
                        cte_all.append(float(row['cte_m']))
                    except (KeyError, ValueError):
                        pass
        except OSError:
            pass

        def _avg(xs): return sum(xs)/len(xs) if xs else 0.0
        def _rms(xs): return math.sqrt(sum(v*v for v in xs)/len(xs)) if xs else 0.0

        print(f'\n{sep}\n  INFORME DE VALIDACIÓN  {time.strftime("%Y-%m-%d %H:%M:%S")}\n{sep}')
        print(f'  Duración           : {elapsed:.1f} s')
        print(f'  Mensajes odometría : {self._odom_n}')
        print(f'  Waypoints alcanzados: {self._target_idx}/{len(self._waypoints)}')
        if cte_all:
            print(f'  DESVIACIÓN DE RUTA : '
                  f'media={_avg(cte_all):.3f} m  '
                  f'máx={max(cte_all):.3f} m  '
                  f'rms={_rms(cte_all):.3f} m  '
                  f'(n={len(cte_all)} muestras)')
        print(f'  CSV telemetría     : {self._log_file}')
        print(f'  CSV cómputo        : {self._compute_csv_path}')
        print(sep + '\n')

        for fh in (self._tel_fh, self._cmp_fh):
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Punto de entrada
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ValidationNode()

    def _shutdown(_sig, _frame):
        node.report()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
