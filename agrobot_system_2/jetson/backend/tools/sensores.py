#!/usr/bin/env python3
"""
Registro unificado de TODOS los sensores del AgroBot, y análisis de la
correlación entre ellos.

Por qué existe
--------------
Cada sensor del robot se mira hoy en un sitio distinto: el IMU en un topic de
ROS, el ambiental en el dashboard, el LIDAR en el radar, el actuador en su
trama UART, el enlace LoRa en la Raspberry. Así se ve si UNA señal está mal,
pero no se ve la relación ENTRE ellas, que es donde están casi todos los
problemas de un robot: el mando que se pide y la velocidad que sale, el giro
comandado y el que mide el giróscopo, la vibración del chasis y el error de
la odometría.

Esta herramienta pone todo en una sola tabla con un único reloj, y después
mide cómo se relacionan las columnas, incluido el RETARDO entre ellas.

Uso
---
    python3 sensores.py grabar --segundos 120
    python3 sensores.py analizar registro_20260906_0230.csv

Para que el registro salga completo conviene tener sourceado ROS:
    source /opt/ros/jazzy/setup.bash
    source ~/Desktop/AGROBOT/agrobot_system_2/ros2_ws/install/setup.bash

Sin ROS funciona igual, pero sin las columnas de LIDAR, odometría y EKF.

Fuentes y por qué cada una
--------------------------
  ROS       IMU, /scan, rf2o, EKF y odometría comandada. Es la vía de menor
            latencia y la que ya usa la navegación.
  WebSocket El backend (puerto 8080) es el único que conoce el estado real
            del actuador, la salud de cada subsistema y las detecciones de
            la cámara. Se lee por su WebSocket en vez de duplicar lógica.
  UDP 5001  Solo si el backend NO está corriendo. Se comprueba antes de
            intentar abrir el puerto: si el backend está vivo, el puerto es
            suyo e intentar tomarlo rompería la telemetría del robot.
"""

import argparse
import asyncio
import csv
import json
import math
import os
import socket
import sys
import threading
import time
from datetime import datetime

# ---------------------------------------------------------------------------
# Estado compartido. Lo escriben varios hilos (ROS por un lado, asyncio por
# otro) y lo lee el muestreador. Un lock simple basta: son escrituras cortas
# sobre un dict y no hay ninguna sección crítica larga.
# ---------------------------------------------------------------------------
_lock = threading.Lock()
_estado: dict[str, float] = {}
_sellos: dict[str, float] = {}      # cuándo se actualizó cada grupo


def anotar(grupo: str, valores: dict):
    with _lock:
        _estado.update(valores)
        _sellos[grupo] = time.time()


def leer() -> tuple[dict, dict]:
    with _lock:
        return dict(_estado), dict(_sellos)


# ---------------------------------------------------------------------------
# ROS: corre en su propio hilo con su propio executor.
# ---------------------------------------------------------------------------
_hilos_ros: list = []


def hilo_ros(parar: threading.Event) -> bool:
    terminado = threading.Event()
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu, LaserScan
        from nav_msgs.msg import Odometry
    except Exception as e:
        print(f"[ros] no disponible ({type(e).__name__}). "
              f"Sourcea ROS si quieres LIDAR, odometría y EKF.")
        return False

    def yaw_de(q):
        return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                       1 - 2 * (q.y * q.y + q.z * q.z)))

    class Escucha(Node):
        def __init__(self):
            super().__init__("registrador_sensores")
            self.create_subscription(Imu, "/imu/data_raw", self.imu, qos_profile_sensor_data)
            self.create_subscription(LaserScan, "/scan", self.scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, "/odom_rf2o", self.rf2o, 10)
            self.create_subscription(Odometry, "/odometry/filtered", self.ekf, 10)
            self.create_subscription(Odometry, "/wheel_odom", self.cmd, 10)

        def imu(self, m):
            anotar("imu", {
                "imu_ax": m.linear_acceleration.x, "imu_ay": m.linear_acceleration.y,
                "imu_az": m.linear_acceleration.z, "imu_gx": m.angular_velocity.x,
                "imu_gy": m.angular_velocity.y,  "imu_gz": m.angular_velocity.z})

        def scan(self, m):
            # Se resume el scan en pocos números: guardar 360 rangos por fila
            # haría la tabla ilegible y no aporta para buscar correlaciones.
            # Lo que sí importa es a qué distancia está lo más cercano, y
            # cuántos haces devuelven algo — un scan que se queda sin retornos
            # válidos explica por sí solo un fallo de odometría láser.
            n = len(m.ranges)
            validos, dmin, amin, frontal = 0, float("inf"), 0.0, float("inf")
            for i, r in enumerate(m.ranges):
                if not (m.range_min <= r <= m.range_max) or math.isnan(r):
                    continue
                validos += 1
                a = math.degrees(m.angle_min + i * m.angle_increment)
                if r < dmin:
                    dmin, amin = r, a
                if abs((a + 180) % 360 - 180) <= 30 and r < frontal:
                    frontal = r
            anotar("scan", {
                "scan_validos": validos,
                "scan_pct_validos": round(100.0 * validos / n, 1) if n else 0.0,
                "scan_min_m": None if dmin == float("inf") else round(dmin, 3),
                "scan_min_ang": round(amin, 1),
                "scan_frontal_m": None if frontal == float("inf") else round(frontal, 3)})

        def rf2o(self, m):
            anotar("rf2o", {
                "rf2o_vx": m.twist.twist.linear.x, "rf2o_vyaw": m.twist.twist.angular.z,
                "rf2o_x": m.pose.pose.position.x, "rf2o_y": m.pose.pose.position.y,
                "rf2o_yaw": yaw_de(m.pose.pose.orientation)})

        def ekf(self, m):
            anotar("ekf", {
                "ekf_vx": m.twist.twist.linear.x, "ekf_vyaw": m.twist.twist.angular.z,
                "ekf_x": m.pose.pose.position.x, "ekf_y": m.pose.pose.position.y,
                "ekf_yaw": yaw_de(m.pose.pose.orientation)})

        def cmd(self, m):
            anotar("cmd", {"cmd_vx": m.twist.twist.linear.x,
                           "cmd_vyaw": m.twist.twist.angular.z})

    def correr():
        rclpy.init()
        nodo = Escucha()
        try:
            while not parar.is_set():
                rclpy.spin_once(nodo, timeout_sec=0.1)
        finally:
            nodo.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass
        terminado.set()

    hilo = threading.Thread(target=correr, daemon=True)
    hilo.start()
    _hilos_ros.append((hilo, terminado))
    return True


# ---------------------------------------------------------------------------
# Backend: WebSocket. Trae lo que solo él sabe (actuador, salud, cámara).
# ---------------------------------------------------------------------------
async def tarea_websocket(url: str, parar: asyncio.Event):
    import aiohttp
    avisado = False
    while not parar.is_set():
        try:
            async with aiohttp.ClientSession() as s:
                async with s.ws_connect(url, heartbeat=10) as ws:
                    print(f"[ws] conectado a {url}")
                    async for msg in ws:
                        if parar.is_set():
                            break
                        if msg.type != aiohttp.WSMsgType.TEXT:
                            continue
                        try:
                            m = json.loads(msg.data)
                        except Exception:
                            continue
                        procesar_ws(m)
        except Exception as e:
            if not parar.is_set():
                # Solo se avisa la primera vez. Reintentar cada 5 s es lo
                # correcto —el backend puede arrancar a mitad de la grabacion—
                # pero repetir el mensaje llena la pantalla y tapa el progreso.
                if not avisado:
                    print(f"[ws] backend no disponible ({type(e).__name__}). "
                          f"Se reintenta en segundo plano; el resto se graba igual.")
                    avisado = True
                await asyncio.sleep(5)


def procesar_ws(m: dict):
    tipo, d = m.get("type"), m.get("data") or {}
    if tipo in ("estado_inicial", "snapshot", "estado"):
        st = d.get("actuator_status") or {}
        if st:
            anotar("actuador", {
                "act_v": st.get("v_cmd"), "act_omega": st.get("omega_cmd"),
                "act_failsafe": int(bool(st.get("failsafe"))),
                "act_servo": st.get("servo_angle"), "act_led": int(bool(st.get("led_state")))})
        sy = d.get("system_stats") or {}
        if sy:
            anotar("sistema", {"cpu_pct": sy.get("cpu"), "ram_pct": sy.get("ram"),
                               "temp_jetson": sy.get("temp")})
    elif tipo == "actuator_status":
        anotar("actuador", {
            "act_v": d.get("v_cmd"), "act_omega": d.get("omega_cmd"),
            "act_failsafe": int(bool(d.get("failsafe"))),
            "act_servo": d.get("servo_angle"), "act_led": int(bool(d.get("led_state")))})
    elif tipo == "env_telemetry":
        anotar("ambiente", {"co2": d.get("co2"), "temp_scd": d.get("tempSCD"),
                            "humedad": d.get("humidity")})
    elif tipo == "detection":
        anotar("camara", {"det_n": len(d.get("detections") or []),
                          "det_conf_max": max((x.get("conf", 0) for x in
                                               (d.get("detections") or [])), default=0.0)})


# ---------------------------------------------------------------------------
# UDP directo. SOLO si el backend no está usando el puerto.
# ---------------------------------------------------------------------------
def hilo_udp(parar: threading.Event) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("0.0.0.0", 5001))
    except OSError:
        # Lo normal cuando el robot está funcionando: el backend tiene el
        # puerto. No es un error, es que el dato ya llega por ROS.
        print("[udp] puerto 5001 ocupado (backend vivo) — se usa ROS para el IMU")
        s.close()
        return False
    s.settimeout(0.5)
    print("[udp] backend apagado: se leen las tramas del nodo LoRa directamente")

    def correr():
        while not parar.is_set():
            try:
                data, _ = s.recvfrom(1024)
                d = json.loads(data)
            except Exception:
                continue
            anotar("imu", {"imu_ax": d.get("accelX"), "imu_ay": d.get("accelY"),
                           "imu_az": d.get("accelZ"),
                           "imu_gx": math.radians(d.get("gyroX", 0.0)),
                           "imu_gy": math.radians(d.get("gyroY", 0.0)),
                           "imu_gz": math.radians(d.get("gyroZ", 0.0))})
            anotar("ambiente", {"co2": d.get("co2"), "temp_scd": d.get("tempSCD"),
                                "humedad": d.get("humidity")})
        s.close()

    threading.Thread(target=correr, daemon=True).start()
    return True


# ---------------------------------------------------------------------------
COLUMNAS = [
    "t", "hora",
    "imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz",
    "co2", "temp_scd", "humedad",
    "scan_validos", "scan_pct_validos", "scan_min_m", "scan_min_ang", "scan_frontal_m",
    "rf2o_vx", "rf2o_vyaw", "rf2o_x", "rf2o_y", "rf2o_yaw",
    "ekf_vx", "ekf_vyaw", "ekf_x", "ekf_y", "ekf_yaw",
    "cmd_vx", "cmd_vyaw",
    "act_v", "act_omega", "act_failsafe", "act_servo", "act_led",
    "det_n", "det_conf_max",
    "cpu_pct", "ram_pct", "temp_jetson",
]
# Edad de cada grupo: sin esto no se distingue "el sensor mide siempre lo
# mismo" de "el sensor está muerto y se repite el último valor conocido".
GRUPOS = ["imu", "ambiente", "scan", "rf2o", "ekf", "cmd", "actuador", "camara", "sistema"]


async def grabar(args):
    ruta = args.salida or f"registro_{datetime.now():%Y%m%d_%H%M%S}.csv"
    parar_hilos = threading.Event()
    parar_async = asyncio.Event()

    hay_ros = hilo_ros(parar_hilos)
    hay_udp = hilo_udp(parar_hilos)
    tarea = asyncio.create_task(tarea_websocket(args.ws, parar_async))

    print(f"\nGrabando {args.segundos} s a {args.hz} Hz -> {ruta}")
    print("Mueve el robot, haz las maniobras que quieras comparar.\n")

    cabecera = COLUMNAS + [f"edad_{g}" for g in GRUPOS]
    t0 = time.time()
    periodo = 1.0 / args.hz
    n = 0
    with open(ruta, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cabecera)
        siguiente = t0
        while time.time() - t0 < args.segundos:
            siguiente += periodo
            await asyncio.sleep(max(0.0, siguiente - time.time()))
            ahora = time.time()
            est, sellos = leer()
            fila = [round(ahora - t0, 3), datetime.now().strftime("%H:%M:%S.%f")[:-3]]
            fila += [est.get(c) for c in COLUMNAS[2:]]
            # Redondeo de edades a ms; None si ese grupo no ha hablado nunca.
            fila += [None if g not in sellos else round(ahora - sellos[g], 3) for g in GRUPOS]
            w.writerow(fila)
            n += 1
            if n % (args.hz * 10) == 0:
                vivos = [g for g in GRUPOS if g in sellos and ahora - sellos[g] < 2.0]
                print(f"  {ahora - t0:5.0f}s  {n} filas  |  activos: {', '.join(vivos) or 'ninguno'}")

    parar_hilos.set()
    parar_async.set()
    tarea.cancel()
    # Se le dan unos segundos a ROS para cerrar sus nodos por su cuenta.
    for hilo, terminado in _hilos_ros:
        terminado.wait(timeout=3.0)
    print(f"\nListo: {n} filas en {ruta}")
    print(f"Analízalo con:  python3 {os.path.basename(__file__)} analizar {ruta}")
    return ruta


# ---------------------------------------------------------------------------
# Análisis
# ---------------------------------------------------------------------------
def analizar(args):
    import numpy as np

    filas = list(csv.DictReader(open(args.csv)))
    if len(filas) < 20:
        print("Registro demasiado corto para sacar nada.")
        return
    t = np.array([float(r["t"]) for r in filas])
    dt = float(np.median(np.diff(t)))
    print(f"\n{args.csv}: {len(filas)} filas, {t[-1]:.0f} s, ~{1/dt:.1f} Hz\n")

    # --- salud de cada columna ---------------------------------------------
    cols = {}
    print("=" * 74)
    print("ESTADO DE CADA SEÑAL")
    print("=" * 74)
    print(f"{'señal':<18}{'muestras':>9}{'min':>11}{'max':>11}{'desv':>11}  observación")
    for c in COLUMNAS[2:]:
        crudo = [r.get(c) for r in filas]
        v = np.array([float(x) for x in crudo if x not in (None, "", "None")])
        if v.size == 0:
            print(f"{c:<18}{0:>9}{'—':>11}{'—':>11}{'—':>11}  SIN DATOS")
            continue
        nota = ""
        if v.std() == 0:
            nota = "CONSTANTE — sensor parado o desconectado"
        elif v.size < len(filas) * 0.5:
            nota = f"solo {100*v.size/len(filas):.0f}% de las filas"
        print(f"{c:<18}{v.size:>9}{v.min():>11.3f}{v.max():>11.3f}{v.std():>11.3f}  {nota}")
        if v.std() > 0 and v.size > len(filas) * 0.5:
            cols[c] = np.array([float(x) if x not in (None, "", "None") else np.nan
                                for x in crudo])

    # --- correlaciones ------------------------------------------------------
    nombres = list(cols)
    print("\n" + "=" * 74)
    print("PARES MÁS CORRELACIONADOS")
    print("Correlación cerca de +1 o -1 = las dos señales se mueven juntas.")
    print("=" * 74)
    pares = []
    for i in range(len(nombres)):
        for j in range(i + 1, len(nombres)):
            a, b = cols[nombres[i]], cols[nombres[j]]
            m = ~(np.isnan(a) | np.isnan(b))
            if m.sum() < 20 or a[m].std() == 0 or b[m].std() == 0:
                continue
            r = float(np.corrcoef(a[m], b[m])[0, 1])
            if not math.isnan(r):
                pares.append((abs(r), r, nombres[i], nombres[j]))
    pares.sort(reverse=True)
    for _, r, x, y in pares[:args.top]:
        print(f"  {r:+.3f}   {x:<18} vs {y}")
    print("\n  ⚠️  Dos señales que sólo DERIVAN despacio correlacionan alto sin")
    print("      tener nada que ver: la temperatura del SCD40 y la posición")
    print("      integrada del EKF suben las dos poco a poco y salen a 0.7 sin")
    print("      relación física ninguna. Fíate de los pares que además")
    print("      comparten forma cuando el robot se mueve, no de los que sólo")
    print("      comparten pendiente. Los retardos de abajo son más fiables:")
    print("      necesitan que las dos señales cambien de verdad.")

    # --- retardos -----------------------------------------------------------
    # Esto es lo que de verdad sirve para ajustar el robot: no si dos señales
    # se parecen, sino CUÁNTO TARDA una en seguir a la otra. Un retardo entre
    # lo que se comanda y lo que el sensor mide es latencia de la cadena de
    # control, y se puede reducir.
    PAREJAS = [("cmd_vx", "rf2o_vx"), ("cmd_vx", "ekf_vx"), ("cmd_vx", "act_v"),
               ("cmd_vyaw", "imu_gz"), ("cmd_vyaw", "rf2o_vyaw"),
               ("act_omega", "imu_gz"), ("imu_gz", "ekf_vyaw"),
               ("rf2o_vx", "ekf_vx")]
    hechas = [(a, b) for a, b in PAREJAS if a in cols and b in cols]
    if hechas:
        print("\n" + "=" * 74)
        print("RETARDO ENTRE SEÑALES  (positivo = la segunda va por detrás)")
        print("=" * 74)
        for a, b in hechas:
            x, y = cols[a], cols[b]
            m = ~(np.isnan(x) | np.isnan(y))
            x, y = x[m] - x[m].mean(), y[m] - y[m].mean()
            if x.std() == 0 or y.std() == 0:
                continue
            maxlag = int(2.0 / dt)
            mejor_r, mejor_k = 0.0, 0
            for k in range(-maxlag, maxlag + 1):
                xa = x[max(0, -k):len(x) - max(0, k)]
                ya = y[max(0, k):len(y) - max(0, -k)]
                if len(xa) < 20 or xa.std() == 0 or ya.std() == 0:
                    continue
                r = float(np.corrcoef(xa, ya)[0, 1])
                if abs(r) > abs(mejor_r):
                    mejor_r, mejor_k = r, k
            print(f"  {a:<12} -> {b:<12} retardo {mejor_k*dt:+.2f} s   corr {mejor_r:+.3f}")

    print("\nSugerencia: si 'cmd_vx -> rf2o_vx' sale con correlación baja, el robot")
    print("no está haciendo lo que se le pide, o la odometría no lo está viendo.\n")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("grabar", help="captura todos los sensores a un CSV")
    g.add_argument("--segundos", type=int, default=120)
    g.add_argument("--hz", type=int, default=10, help="filas por segundo (def. 10)")
    g.add_argument("--salida", help="ruta del CSV")
    g.add_argument("--ws", default="http://127.0.0.1:8080/ws",
                   help="WebSocket del backend")

    a = sub.add_parser("analizar", help="correlaciones y retardos de un CSV")
    a.add_argument("csv")
    a.add_argument("--top", type=int, default=15)

    args = p.parse_args()
    if args.cmd == "grabar":
        try:
            asyncio.run(grabar(args))
        except KeyboardInterrupt:
            print("\ninterrumpido")
    else:
        analizar(args)


if __name__ == "__main__":
    main()
