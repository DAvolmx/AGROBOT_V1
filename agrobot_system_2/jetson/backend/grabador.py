"""
Grabador de sesión: TODO lo que hace el robot, para modelarlo después.
======================================================================

Pedido del usuario el 2026-10-02 ("guarda todos los datos para modelar
mejor todo"). Hasta entonces solo quedaba la pose a 1 Hz
(`navigation_logger`), los pasos robóticos y la evasión: de una sesión de
manejo a mano no quedaba con qué calibrar el simulador ni entrenar.

Una carpeta por arranque del backend en `~/.agrobot/datos/AAAAMMDD_HHMMSS/`:

  telemetria.csv   10 Hz: orden (pulso, pedida, tras la rampa), eco de la
                   C3, pose del EKF, huecos del LIDAR, suelo por cámara,
                   IMU, servo, modo de manejo, nivel de seguridad.
  scans_NNNN.npz   scans COMPLETOS del LIDAR (ángulos fijos + distancias en
                   float16 + instante), en bloques de 15 s. Todos mientras
                   el robot se mueve (y 3 s después, por el deslizamiento);
                   1 por segundo en reposo.
  camara/          JPEG de la cámara: 2 por segundo en movimiento, 1 cada
                   10 s en reposo, y uno en cada frenada.
  eventos.jsonl    todo lo que el backend anuncia: frenadas (y si fue el
                   LIDAR o la cámara), emergencias, seguridad, modo de
                   manejo, pasos robóticos, evasión, toasts.
  sesion.json      versión del código, configuración al empezar, resumen.

Si queda poco disco (< 5 GB) deja de grabar scans y fotos y lo anota.
Leerla: `tools/leer_sesion.py`.
"""
from __future__ import annotations

import threading
from functools import wraps

def serialized(fn):
    @wraps(fn)
    def wrapper(self, *args, **kwargs):
        with self._io_lock:
            if self._closed:
                return
            return fn(self, *args, **kwargs)
    return wrapper

import csv
import json
import math
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np

RAIZ = Path.home() / ".agrobot" / "datos"
TELEMETRIA_HZ = 10.0
SCAN_REPOSO_S = 1.0
FOTO_MOVIMIENTO_S = 0.5
FOTO_REPOSO_S = 10.0
DESPUES_DE_MOVER_S = 3.0          # el robot sigue deslizando tras la orden
BLOQUE_SCANS_S = 15.0           # corto: los apagones bruscos pierden el bloque abierto
DISCO_MINIMO_GB = 5.0
EVENTOS_IGNORADOS = {"lidar_scan", "lidar_distancias", "pose", "actuator_status",
                     "env_telemetry", "system_stats", "map", "odom_webcam", "snapshot",
                     "health", "costmap", "camera_frame"}

COLUMNAS = [
    "t", "t_unix", "pulso_accion", "modo_manejo",
    "v_pedida", "w_pedida", "v_aplicada", "w_aplicada",
    "v_c3", "w_c3", "c3_failsafe", "c3_via", "servo_deg",
    "x", "y", "rumbo_deg", "vx_ekf", "vyaw_ekf",
    "lidar_adelante", "lidar_atras", "lidar_izquierda", "lidar_derecha", "edad_scan_s",
    "suelo_hueco", "suelo_pct", "suelo_obj_ref",
    "acc_x", "acc_y", "acc_z", "giro_x", "giro_y", "giro_z",
    "seguridad", "patinaje",
]


def _version_codigo() -> str:
    try:
        return subprocess.run(["git", "-C", str(Path(__file__).parent), "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True, timeout=3).stdout.strip()
    except Exception:
        return "?"


def _num(v, nd=4):
    if v is None:
        return ""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return v
    return "" if math.isnan(f) else round(f, nd)


class Grabador:
    def __init__(self, *, leer: dict, obtener_frame=None, cv2=None, config=None,
                 raiz: Path = RAIZ, reloj=time.monotonic):
        """`leer` = {nombre: callable} para cada fuente (ver `_fila`). Las que
        falten o fallen quedan vacías: grabar nunca debe tumbar al backend."""
        self._io_lock = threading.RLock()
        self._closed = False
        self._leer = leer
        self._frame = obtener_frame
        self._cv2 = cv2
        self._reloj = reloj
        self.dir = raiz / time.strftime("%Y%m%d_%H%M%S")
        (self.dir / "camara").mkdir(parents=True, exist_ok=True)
        self._csv_f = open(self.dir / "telemetria.csv", "w", newline="")
        self._csv = csv.writer(self._csv_f)
        self._csv.writerow(COLUMNAS)
        self._eventos = open(self.dir / "eventos.jsonl", "a")
        self._scans_t, self._scans_d = [], []
        self._angulos = None
        self._bloque = 0
        self._t_bloque = reloj()
        self._ultimo_scan_t = None
        self._t_scan_guardado = 0.0
        self._t_foto = 0.0
        self._ultimo_mov = -1e9
        self._ultimo_seq_foto = None
        self.sin_disco = False
        self.contadores = {"filas": 0, "scans": 0, "fotos": 0, "eventos": 0}
        (self.dir / "sesion.json").write_text(json.dumps({
            "inicio": time.strftime("%Y-%m-%d %H:%M:%S"), "codigo": _version_codigo(),
            "config": config or {}}, indent=2, ensure_ascii=False))

    # ------------------------------------------------------------ fuentes
    def _l(self, nombre, defecto=None):
        fn = self._leer.get(nombre)
        if fn is None:
            return defecto
        try:
            return fn()
        except Exception:
            return defecto

    def moviendose(self, fila: dict) -> bool:
        return any(abs(float(fila.get(k) or 0)) > 0.01 for k in ("v_pedida", "w_pedida", "v_c3", "w_c3"))

    def _fila(self) -> dict:
        vel = self._l("velocidades", {}) or {}
        c3 = self._l("c3", {}) or {}
        pose = self._l("pose", {}) or {}
        d = self._l("distancias", {}) or {}
        suelo = self._l("suelo", None) or {}
        imu = self._l("imu", {}) or {}
        seg = self._l("seguridad", {}) or {}
        return {
            "t": round(self._reloj(), 3), "t_unix": round(time.time(), 3),
            "pulso_accion": self._l("pulso_accion", "") or "", "modo_manejo": self._l("modo", ""),
            "v_pedida": vel.get("v_pedida"), "w_pedida": vel.get("omega_pedida"),
            "v_aplicada": vel.get("v_aplicada"), "w_aplicada": vel.get("omega_aplicada"),
            "v_c3": c3.get("v_cmd"), "w_c3": c3.get("omega_cmd"), "c3_failsafe": c3.get("failsafe"),
            "c3_via": c3.get("via_estado"), "servo_deg": c3.get("servo_angle"),
            "x": pose.get("x"), "y": pose.get("y"), "rumbo_deg": pose.get("theta_deg"),
            "vx_ekf": pose.get("vx"), "vyaw_ekf": pose.get("vyaw"),
            "lidar_adelante": d.get("adelante"), "lidar_atras": d.get("atras"),
            "lidar_izquierda": d.get("izquierda"), "lidar_derecha": d.get("derecha"),
            "edad_scan_s": self._l("edad_scan"),
            "suelo_hueco": suelo.get("hueco"), "suelo_pct": suelo.get("suelo_pct"),
            "suelo_obj_ref": suelo.get("objeto_en_referencia"),
            "acc_x": imu.get("accelX"), "acc_y": imu.get("accelY"), "acc_z": imu.get("accelZ"),
            "giro_x": imu.get("gyroX"), "giro_y": imu.get("gyroY"), "giro_z": imu.get("gyroZ"),
            "seguridad": seg.get("nivel"),
            "patinaje": (seg.get("deslizamiento") or {}).get("patinaje"),
        }

    # ------------------------------------------------------------ ciclo
    @serialized
    def tick(self):
        """Llamar a TELEMETRIA_HZ. Escribe una fila y decide scan/foto."""
        ahora = self._reloj()
        fila = self._fila()
        self._csv.writerow([_num(fila.get(c)) if not isinstance(fila.get(c), str) else fila.get(c)
                            for c in COLUMNAS])
        self.contadores["filas"] += 1
        if self.moviendose(fila):
            self._ultimo_mov = ahora
        activo = ahora - self._ultimo_mov < DESPUES_DE_MOVER_S
        if self.contadores["filas"] % 50 == 0:
            self._csv_f.flush()
            self.sin_disco = shutil.disk_usage(self.dir).free / 1e9 < DISCO_MINIMO_GB
        if self.sin_disco:
            return
        self._tomar_scan(ahora, activo)
        if ahora - self._t_foto >= (FOTO_MOVIMIENTO_S if activo else FOTO_REPOSO_S):
            self._tomar_foto(ahora)
        if ahora - self._t_bloque >= BLOQUE_SCANS_S:
            self.guardar_scans()

    def _tomar_scan(self, ahora, activo):
        t_scan = self._l("t_scan")
        if t_scan is None or t_scan == self._ultimo_scan_t:
            return
        if not activo and ahora - self._t_scan_guardado < SCAN_REPOSO_S:
            return
        puntos = self._l("scan", []) or []
        if not puntos:
            return
        ang = np.array([p["angle_deg"] for p in puntos], dtype=np.float32)
        dist = np.array([p["distance_m"] for p in puntos], dtype=np.float32)
        # Rejilla fija de 0,5° para que todos los scans tengan la misma forma.
        if self._angulos is None:
            self._angulos = np.arange(-180.0, 180.0, 0.5, dtype=np.float32)
        idx = np.clip(np.round((ang + 180.0) / 0.5).astype(int), 0, len(self._angulos) - 1)
        fila = np.full(len(self._angulos), np.nan, dtype=np.float16)
        fila[idx] = dist.astype(np.float16)
        self._scans_d.append(fila)
        self._scans_t.append(ahora)
        self._ultimo_scan_t = t_scan
        self._t_scan_guardado = ahora
        self.contadores["scans"] += 1

    def _tomar_foto(self, ahora, motivo=""):
        if self._frame is None or self._cv2 is None:
            return
        try:
            dato = self._frame()
        except Exception:
            return
        if dato is None or dato[2] == self._ultimo_seq_foto:
            return
        self._ultimo_seq_foto = dato[2]
        ok, jpg = self._cv2.imencode(".jpg", dato[0], [int(self._cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            nombre = f"{ahora:012.3f}{('_' + motivo) if motivo else ''}.jpg"
            (self.dir / "camara" / nombre).write_bytes(jpg.tobytes())
            self._t_foto = ahora
            self.contadores["fotos"] += 1

    def guardar_scans(self):
        if self._scans_d:
            np.savez_compressed(self.dir / f"scans_{self._bloque:04d}.npz",
                                t=np.array(self._scans_t, dtype=np.float64),
                                angulos_deg=self._angulos, distancias_m=np.stack(self._scans_d))
            self._bloque += 1
        self._scans_t, self._scans_d = [], []
        self._t_bloque = self._reloj()

    # ------------------------------------------------------------ eventos
    @serialized
    def evento(self, mensaje: dict):
        """Todo lo que el backend anuncia (state.broadcast), menos el flujo continuo."""
        tipo = mensaje.get("type")
        if tipo in EVENTOS_IGNORADOS:
            return
        try:
            self._eventos.write(json.dumps({"t": round(self._reloj(), 3), "t_unix": round(time.time(), 3),
                                            "tipo": tipo, "datos": mensaje.get("data")},
                                           ensure_ascii=False, default=str) + "\n")
            self._eventos.flush()
            self.contadores["eventos"] += 1
        except (OSError, TypeError, ValueError):
            return
        if tipo == "obstacle_warning" and not self.sin_disco:
            self._tomar_foto(self._reloj(), "freno")

    @serialized
    def cerrar(self):
        self._closed = True
        self.guardar_scans()
        for f in (self._csv_f, self._eventos):
            try:
                f.close()
            except OSError:
                pass
        try:
            datos = json.loads((self.dir / "sesion.json").read_text())
            datos.update(fin=time.strftime("%Y-%m-%d %H:%M:%S"), contadores=self.contadores,
                         sin_disco=self.sin_disco)
            (self.dir / "sesion.json").write_text(json.dumps(datos, indent=2, ensure_ascii=False))
        except (OSError, ValueError):
            pass
