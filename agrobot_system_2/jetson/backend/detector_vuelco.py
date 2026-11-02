"""DETECTOR DE VUELCO ROBUSTO A LOS GOLPES (2026-11-02). Sustituye a la regla instantánea de modo_seguro (inclinación del acelerómetro >= 38°, o >= 22° con
balanceo >= 90 °/s), que con el robot en marcha saltaba sola: en 102 h de telemetría real SIN vuelcos el acelerómetro marca inclinaciones de 50–120° en el p99
de la conducción y el giroscopio picos de 190 °/s. Son GOLPES de 0,1–0,3 s al arrancar cada pulso (motores al 78–100 %), no inclinación: a los 0,5 s el robot
vuelve a dar ~4°. Así se enclavó el modo seguro el 19-oct («VUELCO 77°» con el robot quieto) y por eso está apagado.

Lo que distingue un vuelco de un golpe es que la GRAVEDAD queda inclinada de forma sostenida. El detector:
  1. descarta las muestras que no son gravedad pura: |a| lejos de 1 g (`g_tol`) o el giroscopio girando rápido (`w_gate_dps`);
  2. estima la gravedad con la MEDIANA de las muestras aceptadas en los últimos `ventana_s` (mínimo `n_min`), y de ahí balanceo y cabeceo;
  3. VUELCO si el balanceo pasa `lim_lateral_deg` o el cabeceo `lim_frontal_deg` durante `persistencia_s` seguidos (el robot ya está tumbado o cayendo:
     se cortan los motores para que no siga empujando);
  4. PRECAUCIÓN si la inclinación supera `prec_deg` durante `prec_s` (talud fuerte: ir lento);
  5. se LIBERA solo cuando la inclinación vuelve por debajo de `liberar_deg` durante `liberar_s` (el operador lo enderezó): no queda enclavado para siempre.
Los parámetros por defecto salen del entrenamiento con vuelcos simulados + 102 h reales (simulador_pasivo/entrenar_vuelco.py, docs/VUELCO_DETECTOR_20261102).

  6. ignora los paquetes REPETIDOS idénticos (el nodo del IMU se congela segundos con el último valor, a veces en mitad de un golpe).
Lógica pura: `paso(t, acc_g, gyro_dps)` por cada paquete NUEVO del IMU (acc en g, gyro en °/s, ya nivelados con imu_nivel). Ejes del AGROBOT: X lateral,
Y adelante, Z arriba. Balanceo = giro alrededor de Y (se ve en accX), cabeceo = alrededor de X (se ve en accY)."""
from __future__ import annotations

import math
from collections import deque

NORMAL, PRECAUCION, VUELCO = "normal", "precaucion", "vuelco"

# Entrenados el 2026-11-02 (CEM, 10 × 18; prueba en datos no vistos: 110/128 vuelcos simulados, 0/232 falsas simuladas, 0 falsas en 0,7 h de marcha
# real de exteriores e interiores del 27-oct en adelante). Los de la mano de antes (0.15/60/0.8/3/35/40/0.6) detectaban 121/128 pero daban 7,4 falsas/h.
PARAMS_DEFECTO = {
    "g_tol": 0.24, "w_gate_dps": 23.0, "ventana_s": 0.9, "n_min": 4,
    "lim_lateral_deg": 41.0, "lim_frontal_deg": 39.0, "persistencia_s": 1.07,
    "prec_deg": 18.0, "prec_s": 2.0, "liberar_deg": 15.0, "liberar_s": 3.0,
}


def inclinaciones(g) -> tuple[float, float, float]:
    """(balanceo, cabeceo, total) en grados desde un vector de gravedad (x lateral, y adelante, z arriba)."""
    x, y, z = g
    n = math.sqrt(x * x + y * y + z * z) or 1e-9
    total = math.degrees(math.acos(max(-1.0, min(1.0, z / n))))
    balanceo = math.degrees(math.atan2(x, math.sqrt(y * y + z * z) if z >= 0 else -math.sqrt(y * y + z * z)))
    cabeceo = math.degrees(math.atan2(y, math.sqrt(x * x + z * z) if z >= 0 else -math.sqrt(x * x + z * z)))
    return balanceo, cabeceo, total


def _mediana(v):
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class DetectorVuelco:
    def __init__(self, **params):
        self.p = {**PARAMS_DEFECTO, **params}
        self._buf: deque = deque()          # (t, ax, ay, az) aceptadas
        self._ultima = None
        self.estado = NORMAL
        self.balanceo = self.cabeceo = self.total = None
        self._sobre_desde = None            # inclinación sobre el límite de vuelco desde...
        self._prec_desde = None
        self._bajo_desde = None
        self.motivo = ""
        self.vuelcos = 0

    def paso(self, t: float, acc, gyro=None) -> str:
        p = self.p
        try:
            ax, ay, az = (float(v) for v in acc)
        except (TypeError, ValueError):
            return self.estado
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        gx, gy = (0.0, 0.0) if gyro is None else (float(gyro[0]), float(gyro[1]))
        firma = (ax, ay, az, gx, gy)
        if firma == self._ultima:
            return self.estado              # paquete REPETIDO: el nodo del IMU se congeló (visto en la telemetría real: 2–5 s con el mismo valor
        self._ultima = firma                # en mitad de un golpe). Con ruido real dos lecturas idénticas no existen.
        if abs(mag - 1.0) <= p["g_tol"] and math.hypot(gx, gy) <= p["w_gate_dps"]:
            self._buf.append((t, ax, ay, az))
        while self._buf and t - self._buf[0][0] > p["ventana_s"]:
            self._buf.popleft()
        if len(self._buf) < p["n_min"]:
            return self.estado              # sin gravedad fiable ahora: se mantiene lo que había (un golpe no cambia nada)
        g = (_mediana([b[1] for b in self._buf]), _mediana([b[2] for b in self._buf]), _mediana([b[3] for b in self._buf]))
        self.balanceo, self.cabeceo, self.total = inclinaciones(g)
        sobre = abs(self.balanceo) >= p["lim_lateral_deg"] or abs(self.cabeceo) >= p["lim_frontal_deg"] or self.total >= max(p["lim_lateral_deg"], p["lim_frontal_deg"]) + 10
        if sobre:
            self._sobre_desde = t if self._sobre_desde is None else self._sobre_desde
        else:
            self._sobre_desde = None
        if self.total >= p["prec_deg"]:
            self._prec_desde = t if self._prec_desde is None else self._prec_desde
        else:
            self._prec_desde = None
        if self.total < p["liberar_deg"]:
            self._bajo_desde = t if self._bajo_desde is None else self._bajo_desde
        else:
            self._bajo_desde = None

        if self.estado == VUELCO:
            if self._bajo_desde is not None and t - self._bajo_desde >= p["liberar_s"]:
                self.estado, self.motivo = NORMAL, "enderezado"
            return self.estado
        if self._sobre_desde is not None and t - self._sobre_desde >= p["persistencia_s"]:
            self.estado = VUELCO
            self.vuelcos += 1
            self.motivo = f"VUELCO: balanceo {self.balanceo:.0f}°, cabeceo {self.cabeceo:.0f}° sostenidos {p['persistencia_s']:.1f} s"
        elif self._prec_desde is not None and t - self._prec_desde >= p["prec_s"]:
            self.estado, self.motivo = PRECAUCION, f"inclinación sostenida {self.total:.0f}°"
        else:
            self.estado, self.motivo = NORMAL, ""
        return self.estado

    def snapshot(self) -> dict:
        r = lambda v: None if v is None else round(v, 1)
        return {"estado": self.estado, "motivo": self.motivo, "balanceo_deg": r(self.balanceo), "cabeceo_deg": r(self.cabeceo),
                "inclinacion_deg": r(self.total), "vuelcos": self.vuelcos}
