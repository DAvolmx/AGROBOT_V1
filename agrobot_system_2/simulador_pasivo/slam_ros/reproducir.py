#!/usr/bin/env python3
"""Reproduce una grabación de campo (variables.jsonl: scan + IMU a ~5 Hz) dentro de ROS con reloj simulado y guarda lo que sale del SLAM.

  (lo lanza simulador_pasivo/simular_slam.py)  python3 reproducir.py <grabacion/variables.jsonl> <salida.npz> [--velocidad 2] [--desde 0] [--hasta 1e9]

Publica /clock, /scan (LaserScan, marco `laser` = el del robot) y /imu/data_raw (como ros_bridge). Escucha el TF map->base_link (pose de
slam_toolbox), /odometry/filtered (EKF) y /map. Al final guarda poses, odometría y el último mapa en un .npz para medir_slam.py.
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

N_RAYOS = 720
IMU_VAR_GIRO = math.radians(2.4) ** 2


def cargar(ruta, desde, hasta):
    ev, visto, t0 = [], set(), None
    with open(ruta, errors="ignore") as f:
        for linea in f:
            try:
                d = json.loads(linea)
            except ValueError:
                continue
            t = d.get("t_unix")
            if t is None:
                continue
            t0 = t if t0 is None else t0
            if not (desde <= t - t0 <= hasta):
                continue
            e = d.get("env") or {}
            if e.get("gyroZ") is not None:
                ev.append((t, "imu", e))
            lt, sc = d.get("lidar_timestamp"), d.get("lidar_scan")
            if sc and lt not in visto:
                visto.add(lt)
                r = np.full(N_RAYOS, np.inf, dtype=np.float32)
                for p in sc:
                    a, dist = p.get("angle_deg"), p.get("distance_m")
                    if a is None or dist is None:
                        continue
                    r[int(round(((a + 180.0) % 360.0) * 2)) % N_RAYOS] = dist
                ev.append((t - float(d.get("lidar_age_s") or 0.0), "scan", r))
    ev.sort(key=lambda x: x[0])
    return ev


class Reproductor(Node):
    def __init__(self):
        super().__init__("reproductor_grabacion")
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.pub_scan = self.create_publisher(LaserScan, "/scan", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu/data_raw", 50)
        self.tf_buf = Buffer()
        self.tf_lis = TransformListener(self.tf_buf, self)
        qmap = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.mapa = None
        self.create_subscription(OccupancyGrid, "/map", self._mapa, qmap)
        self.ekf = []
        self.create_subscription(Odometry, "/odometry/filtered", self._ekf, 20)

    def _mapa(self, m):
        self.mapa = m

    def _ekf(self, m):
        q = m.pose.pose.orientation
        self.ekf.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
                         math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))

    def stamp(self, t):
        from builtin_interfaces.msg import Time
        s = Time()
        s.sec, s.nanosec = int(t), int((t - int(t)) * 1e9)
        return s

    def clock(self, t):
        c = Clock()
        c.clock = self.stamp(t)
        self.pub_clock.publish(c)

    def scan(self, t, r):
        m = LaserScan()
        m.header.stamp, m.header.frame_id = self.stamp(t), "laser"
        m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / N_RAYOS
        m.angle_max = m.angle_min + m.angle_increment * (N_RAYOS - 1)
        m.scan_time, m.time_increment = 0.1, 0.1 / N_RAYOS
        m.range_min, m.range_max = 0.05, 12.0
        m.ranges = [float(x) for x in r]
        self.pub_scan.publish(m)

    def imu(self, t, e):
        m = Imu()
        m.header.stamp, m.header.frame_id = self.stamp(t), "imu_link"
        m.linear_acceleration.x = float(e.get("accelX", 0.0)) * 9.80665
        m.linear_acceleration.y = float(e.get("accelY", 0.0)) * 9.80665
        m.linear_acceleration.z = float(e.get("accelZ", 1.0)) * 9.80665
        m.angular_velocity.x = math.radians(float(e.get("gyroX", 0.0)))
        m.angular_velocity.y = math.radians(float(e.get("gyroY", 0.0)))
        m.angular_velocity.z = math.radians(float(e.get("gyroZ", 0.0)))
        m.orientation_covariance[0] = -1.0
        for i in (0, 4, 8):
            m.angular_velocity_covariance[i] = IMU_VAR_GIRO
            m.linear_acceleration_covariance[i] = 0.1
        self.pub_imu.publish(m)

    def pose_slam(self, t):
        from rclpy.time import Time as RT
        try:
            tr = self.tf_buf.lookup_transform("map", "base_link", RT(seconds=t))
        except Exception:
            try:
                tr = self.tf_buf.lookup_transform("map", "base_link", RT())
            except Exception:
                return None
        q = tr.transform.rotation
        return (t, tr.transform.translation.x, tr.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def esperar_hasta(nodo, limite):
    """Atiende callbacks hasta la hora de pared `limite`. spin_once(timeout) vuelve en cuanto llega un mensaje, y usarlo como espera hacía
    que la reproducción fuera ~10x más rápida de lo pedido (slam_toolbox se saltaba scans)."""
    while True:
        falta = limite - time.monotonic()
        if falta <= 0:
            return
        rclpy.spin_once(nodo, timeout_sec=min(0.01, falta))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("grabacion")
    ap.add_argument("salida")
    ap.add_argument("--velocidad", type=float, default=2.0)
    ap.add_argument("--desde", type=float, default=0.0)
    ap.add_argument("--hasta", type=float, default=1e9)
    a = ap.parse_args()
    ev = cargar(a.grabacion, a.desde, a.hasta)
    n_scan = sum(1 for e in ev if e[1] == "scan")
    print(f"[reproducir] {len(ev)} eventos · {n_scan} scans · {ev[-1][0] - ev[0][0]:.0f} s grabados", flush=True)
    rclpy.init()
    nodo = Reproductor()
    t_ini = ev[0][0] - 2.0
    for k in range(40):                                   # 2 s de reloj antes de empezar: los nodos se enganchan al /clock
        nodo.clock(t_ini + k * 0.05)
        rclpy.spin_once(nodo, timeout_sec=0.05)
    poses = []
    pared0, sim0 = time.monotonic(), ev[0][0]
    ultimo_clock = sim0
    for i, (t, tipo, dato) in enumerate(ev):
        # el reloj avanza en pasos de 50 ms aunque no haya mensajes: el EKF (20 Hz) y slam_toolbox necesitan un /clock continuo
        while ultimo_clock + 0.05 < t:
            ultimo_clock += 0.05
            esperar_hasta(nodo, pared0 + (ultimo_clock - sim0) / a.velocidad)
            nodo.clock(ultimo_clock)
        esperar_hasta(nodo, pared0 + (t - sim0) / a.velocidad)
        nodo.clock(t)
        ultimo_clock = t
        if tipo == "scan":
            nodo.scan(t, dato)
            p = nodo.pose_slam(t - 0.2)
            if p is not None:
                poses.append(p)
        else:
            nodo.imu(t, dato)
        rclpy.spin_once(nodo, timeout_sec=0.0)
        if i % 2000 == 0:
            print(f"[reproducir] {100 * i / len(ev):3.0f} % · poses SLAM {len(poses)}", flush=True)
    t_fin = ev[-1][0]
    for k in range(int(6 / 0.1)):                         # deja que slam_toolbox actualice el mapa
        nodo.clock(t_fin + k * 0.1)
        esperar_hasta(nodo, time.monotonic() + 0.1)
    m = nodo.mapa
    np.savez_compressed(a.salida, poses=np.array(poses), ekf=np.array(nodo.ekf),
                        mapa=np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width) if m else np.zeros((0, 0), np.int8),
                        resolucion=m.info.resolution if m else 0.05,
                        origen=np.array([m.info.origin.position.x, m.info.origin.position.y]) if m else np.zeros(2))
    print(f"[reproducir] guardado {a.salida}: {len(poses)} poses SLAM, {len(nodo.ekf)} del EKF, mapa {'sí' if m else 'NO'}", flush=True)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
