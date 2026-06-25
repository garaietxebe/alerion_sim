#!/usr/bin/env python3
"""
Passive validation node.

Monitors a manually piloted flight and logs per-process CPU and RAM usage
to a CSV file in real time.
"""

import csv
import math
import re
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

try:
    import psutil
except ImportError:
    psutil = None  # type: ignore[assignment]

try:
    import gz.transport13 as gz_transport
    from gz.msgs10.world_stats_pb2 import WorldStatistics

    _HAS_GZ = True
except ImportError:
    _HAS_GZ = False


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _speed(twist: Any) -> float:
    v = twist.linear
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


# ---------------------------------------------------------------------------
# Compute sampler
# ---------------------------------------------------------------------------


class ComputeSampler:
    def __init__(self, patterns: list[str]):
        self._patterns = [re.compile(p) for p in patterns]
        self._labels = patterns  # preserves column order in CSV
        self._proc_cache: dict[int, tuple[Any, str]] = {}  # pid -> (Process, label)
        self._last_scan = 0.0
        if psutil:
            psutil.cpu_percent(interval=None)

    def _refresh(self) -> None:
        if not psutil:
            return
        seen = set()
        for proc in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                cmd = " ".join(proc.info["cmdline"] or [proc.info["name"] or ""])  # type: ignore[attr-defined]
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
            for pat in self._patterns:
                if pat.search(cmd):
                    pid = proc.info["pid"]  # type: ignore[attr-defined]
                    if pid not in self._proc_cache:
                        try:
                            ps = psutil.Process(pid)
                            ps.cpu_percent(interval=None)  # prime the counter
                            self._proc_cache[pid] = (ps, pat.pattern)
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            pass
                    seen.add(pid)
                    break
        # remove stale entries
        for pid in list(self._proc_cache):
            if pid not in seen:
                del self._proc_cache[pid]

    def sample(self) -> dict | None:
        """Return a snapshot dict, or None if psutil is unavailable."""
        if not psutil:
            return None
        now = time.monotonic()
        if now - self._last_scan > 5.0:
            self._refresh()
            self._last_scan = now

        sys_cpu = psutil.cpu_percent(interval=None) * psutil.cpu_count()
        vm = psutil.virtual_memory()

        # aggregate by label (multiple PIDs may match the same pattern)
        per_proc: dict[str, tuple[float, float]] = {}
        for _pid, (proc, label) in list(self._proc_cache.items()):
            try:
                cpu = proc.cpu_percent(interval=None)
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
            "sys_cpu_pct": sys_cpu,
            "sys_mem_mb": vm.used / (1024 * 1024),
            "per_proc": per_proc,  # label -> (cpu%, mem_MB)
        }


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------


class ValidationNode(Node):
    def __init__(self) -> None:
        super().__init__("validation_node")

        # parameters
        self.declare_parameter("model_name", "x500_0")
        self.declare_parameter("world_name", "inspection")
        self.declare_parameter("compute_csv", "/alerion_sim/logs/alerion_compute.csv")
        self.declare_parameter("status_interval", 5.0)
        self.declare_parameter("cpu_sample_hz", 0.2)  # 0.2 Hz = every 5 s
        self.declare_parameter(
            "target_processes",
            [
                "gz sim",
                "px4",
                "MicroXRCEAgent",
                "parameter_bridge",
                "image_bridge",
                "gimbal_controller",
                "validation_node",
            ],
        )

        g = self.get_parameter
        self._model = g("model_name").value
        self._world = g("world_name").value
        self._compute_csv_path = g("compute_csv").value
        self._status_dt = g("status_interval").value
        cpu_hz = g("cpu_sample_hz").value
        patterns = list(g("target_processes").value)

        # RTF from Gazebo world stats
        self._last_rtf = 1.0
        self._rtf_lock = threading.Lock()
        self._gz_node = None
        if _HAS_GZ:
            self._gz_node = gz_transport.Node()
            self._gz_node.subscribe(
                WorldStatistics,
                f"/world/{self._world}/stats",
                self._gz_stats_cb,
            )
        else:
            self.get_logger().warn(
                "gz.transport13 not found, RTF column will read 1.0."
            )

        # telemetry state
        self._t0 = time.monotonic()
        self._odom_n = 0
        self._origin: tuple[float, float, float] | None = None  # first odom sample

        # compute sampler
        self._sampler = ComputeSampler(patterns) if psutil else None
        if not psutil:
            self.get_logger().warn("psutil not installed, compute monitoring disabled.")

        # compute CSV, flushed on every row so data survives an abrupt shutdown
        cmp_path = Path(self._compute_csv_path)
        cmp_path.parent.mkdir(parents=True, exist_ok=True)
        self._cmp_fh = open(cmp_path, "w", newline="")
        self._cmp_csv = csv.writer(self._cmp_fh)
        self._cmp_csv.writerow(
            [
                "rtf",
                "sys_cpu_pct",
                "sys_mem_mb",
                "gz_cpu_pct",
                "gz_mem_mb",
                "px4_cpu_pct",
                "px4_mem_mb",
                "xrce_cpu_pct",
                "xrce_mem_mb",
                "bridge_cpu_pct",
                "bridge_mem_mb",
            ]
        )
        self._cmp_fh.flush()

        # subscriptions and timers
        odom_topic = f"/model/{self._model}/odometry"
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_timer(1.0 / max(cpu_hz, 1e-3), self._sample_compute)
        self.create_timer(self._status_dt, self._print_status)

        self.get_logger().info(
            f"  Model       : {self._model}\n"
            f"  Odometry    : {odom_topic}\n"
            f"  Compute CSV : {self._compute_csv_path}  (every {1 / max(cpu_hz, 1e-3):.0f} s)\n"
        )

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_n += 1
        pos = msg.pose.pose.position
        p = (pos.x, pos.y, pos.z)
        if self._origin is None:
            self._origin = p

    def _gz_stats_cb(self, msg: "WorldStatistics") -> None:
        with self._rtf_lock:
            self._last_rtf = msg.real_time_factor

    # -----------------------------------------------------------------------
    # Compute sampling
    # -----------------------------------------------------------------------

    def _sample_compute(self) -> None:
        if self._origin is None:
            return  # wait until the drone is visible
        snap = self._sampler.sample() if self._sampler else None
        with self._rtf_lock:
            rtf = self._last_rtf

        def _cpu(key: str) -> float:
            return snap["per_proc"].get(key, (0.0, 0.0))[0] if snap else 0.0

        def _mem(key: str) -> float:
            return snap["per_proc"].get(key, (0.0, 0.0))[1] if snap else 0.0

        self._cmp_csv.writerow(
            [
                f"{rtf:.3f}",
                f"{snap['sys_cpu_pct']:.1f}" if snap else "",
                f"{snap['sys_mem_mb']:.0f}" if snap else "",
                f"{_cpu('gz sim'):.1f}",
                f"{_mem('gz sim'):.0f}",
                f"{_cpu('px4'):.1f}",
                f"{_mem('px4'):.0f}",
                f"{_cpu('MicroXRCEAgent'):.1f}",
                f"{_mem('MicroXRCEAgent'):.0f}",
                f"{_cpu('parameter_bridge'):.1f}",
                f"{_mem('parameter_bridge'):.0f}",
            ]
        )
        self._cmp_fh.flush()

    # -----------------------------------------------------------------------
    # Status output
    # -----------------------------------------------------------------------

    def _print_status(self) -> None:
        if self._origin is None:
            self.get_logger().info("Waiting for first odometry message...")
            return

        elapsed = time.monotonic() - self._t0
        snap = self._sampler.sample() if self._sampler else None

        with self._rtf_lock:
            rtf = self._last_rtf

        if snap:
            pp = snap["per_proc"]

            def _fmt(key: str) -> str:
                cpu, mem = pp.get(key, (0.0, 0.0))
                return f"{cpu:5.1f}%  {mem:6.0f}MB"

            print(
                f"[{elapsed:7.1f}s]  RTF={rtf:.2f}\n"
                f"  SYS   cpu={snap['sys_cpu_pct']:5.1f}%  mem={snap['sys_mem_mb']:6.0f}MB\n"
                f"  gz    cpu={_fmt('gz sim')}\n"
                f"  px4   cpu={_fmt('px4')}\n"
                f"  xrce  cpu={_fmt('MicroXRCEAgent')}\n"
                f"  bridg cpu={_fmt('parameter_bridge')}\n"
            )
        else:
            print(f"[{elapsed:7.1f}s]  RTF={rtf:.2f}  (no compute data)")

    # -----------------------------------------------------------------------
    # Final report
    # -----------------------------------------------------------------------

    def report(self) -> None:
        elapsed = time.monotonic() - self._t0
        sep = "=" * 76

        print(f"\n{sep}\n  VALIDATION REPORT  {time.strftime('%Y-%m-%d %H:%M:%S')}\n{sep}")
        print(f"  Duration      : {elapsed:.1f} s")
        print(f"  Compute CSV   : {self._compute_csv_path}")
        print(sep + "\n")

        try:
            self._cmp_fh.flush()
            self._cmp_fh.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ValidationNode()

    def _shutdown(_sig: int, _frame: Any) -> None:
        node.report()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
