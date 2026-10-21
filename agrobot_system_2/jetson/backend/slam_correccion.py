"""Corrección de la odometría por scans con el MLP entrenado en Kaggle (NumPy puro; sin torch ni SciPy).

Ciclo cerrado: simulador_pasivo/generar_dataset_slam.py -> tools/kaggle/agrobot_slam_correccion.ipynb (entrena y exporta
pesos_mlp.npz) -> este módulo en el robot. NO está conectado a ningún nodo: es una pieza lista para que un nodo de odometría
(rf2o/ICP + EKF) sume la corrección al incremento estimado, y solo cuando el operador decida desplegarla.

Convenciones (las mismas del generador): marco ROS (x adelante, y izquierda, yaw antihorario), SI, scan de 360 rayos con el 0° al
frente del robot (ya corregido el yaw=π del montaje del LIDAR), rangos válidos 0,18–8,0 m.
Por fotograma (12 valores): v_cmd, w_cmd, gyro_z, icp_residual, dmin_frontal, dmin_izq, dmin_der, prev_dx, prev_dy, prev_dth,
scan_ok, dt. El modelo usa los K=5 últimos. Devuelve (dx, dy, dθ) A SUMAR al incremento estimado (marco del robot).
"""
from __future__ import annotations

from collections import deque
from pathlib import Path

import numpy as np

R_MIN, R_MAX = 0.18, 8.0
N_BASE = 12
LIMITE_CORR = np.array([0.05, 0.05, 0.05])     # m, m, rad por paso: tope de seguridad (el modelo nunca mueve la pose más que esto)
Z_MAX = 8.0                                    # |z-score| máximo de una entrada; más allá, fuera de distribución -> sin corrección


def dmin_sectores(rangos) -> tuple[float, float, float]:
    """Distancia mínima frontal (±15°), izquierda (60–120°) y derecha (240–300°) de un scan de 360 rayos (None = sin scan)."""
    if rangos is None:
        return R_MAX, R_MAX, R_MAX
    r = np.asarray(rangos, float)
    r = np.where(np.isfinite(r) & (r >= R_MIN) & (r <= R_MAX), r, R_MAX)
    return (float(r[np.r_[0:16, 345:360]].min()), float(r[60:121].min()), float(r[240:301].min()))


class CorreccionSlam:
    def __init__(self, ruta_pesos: str | Path):
        p = np.load(ruta_pesos, allow_pickle=False)
        self.W = [(p["W1"], p["b1"]), (p["W2"], p["b2"]), (p["W3"], p["b3"]), (p["W4"], p["b4"])]
        self.x_mean, self.x_scale = p["x_mean"], p["x_scale"]
        self.y_mean, self.y_scale = p["y_mean"], p["y_scale"]
        self.K = int(p["K"])
        if self.x_mean.shape[0] != self.K * N_BASE:
            raise ValueError("pesos incompatibles: D != K*12")
        self.ventana: deque = deque(maxlen=self.K)
        self.ultimo_ood = False

    def reset(self):
        self.ventana.clear()

    def _forward(self, x):
        z = (x - self.x_mean) / self.x_scale
        for i, (W, b) in enumerate(self.W):
            z = z @ W.T + b
            if i < 3:
                z = np.maximum(z, 0.0)
        return z * self.y_scale + self.y_mean, np.abs((x - self.x_mean) / self.x_scale).max()

    def actualizar(self, v_cmd, w_cmd, gyro_z, icp_residual, rangos, prev_inc, scan_ok, dt) -> np.ndarray:
        """Añade un fotograma y devuelve la corrección (dx, dy, dθ). Con la ventana incompleta repite el primer fotograma (como el
        entrenamiento). Si algo es no finito o está fuera de distribución, devuelve ceros (el robot sigue con su estimación)."""
        f = [v_cmd, w_cmd, gyro_z, icp_residual, *dmin_sectores(rangos), prev_inc[0], prev_inc[1], prev_inc[2],
             1.0 if scan_ok else 0.0, dt]
        f = np.asarray(f, float)
        if not np.all(np.isfinite(f)):
            self.ultimo_ood = True
            return np.zeros(3)
        self.ventana.append(f)
        fot = list(self.ventana)
        while len(fot) < self.K:
            fot.insert(0, fot[0])
        corr, zmax = self._forward(np.concatenate(fot))
        self.ultimo_ood = bool(zmax > Z_MAX)
        if self.ultimo_ood or not np.all(np.isfinite(corr)):
            return np.zeros(3)
        return np.clip(corr, -LIMITE_CORR, LIMITE_CORR)

    def corregir_incremento(self, inc_est, **kw) -> np.ndarray:
        return np.asarray(inc_est, float) + self.actualizar(prev_inc=kw.pop("prev_inc", inc_est), **kw)


def desde_config(slam_cfg: dict, base: str | Path | None = None):
    """CorreccionSlam si `slam.correccion_ml_activa` y existe el archivo de pesos; si no, None (el robot sigue con su estimación)."""
    if not slam_cfg.get("correccion_ml_activa", False):
        return None
    ruta = Path(slam_cfg.get("correccion_ml_pesos", "models/slam_correccion/pesos_mlp.npz"))
    if not ruta.is_absolute():
        ruta = Path(base or Path(__file__).resolve().parent) / ruta
    return CorreccionSlam(ruta) if ruta.is_file() else None
