"""Servo de la cámara hacia ABAJO cuando la maceta está baja (2026-10-23, «las macetas bajas las piso»).

La cámara mira ~15° bajo el horizonte (servo 98°). Una maceta baja o muy cercana se sale por el borde inferior de la imagen y el
robot deja de verla justo antes de tocarla. Regla (sobre las cajas de maceta del último cuadro):
  · caja cortada por abajo (y2 >= 0,93·h) o centro por debajo del 62 % de la imagen  -> BAJAR el servo (hacia 62°, ~39° bajo el horizonte)
  · todas las cajas altas (centro por encima del 38 %) y completas                   -> SUBIR un poco (hacia el reposo)
  · sin macetas durante `reposo_s`                                                  -> volver poco a poco al reposo (98°)
El servo sube de ángulo = mira más arriba; baja de ángulo = mira más abajo (memoria: 76° ≈ 33° bajo el horizonte).
Lógica pura (sin red ni reloj propio): `paso()` recibe lo observado y devuelve el ángulo nuevo, o None si no hay que moverlo.
"""
from __future__ import annotations

REPOSO_DEG = 98
MIN_DEG = 66                    # tope hacia abajo (66° ≈ 30° bajo el horizonte)
PASO_BAJA_DEG = 6               # por decisión (cada ~0,5 s)
PASO_SUBE_DEG = 3
PASO_REPOSO_DEG = 4
CORTADA = 0.96                  # y2/h a partir de aquí la caja toca el borde inferior
CENTRO_BAJO = 0.70              # y_centro/h a partir de aquí la maceta está «baja» en la imagen
PERSISTENCIA = 2                # decisiones seguidas con maceta baja antes de bajar el servo (sobre la grabación real: 0,93/0,62 sin persistencia
                                # dejaba el servo abajo el 49 % del tiempo; 0,96/0,70 con persistencia 3, el 6 %)
CENTRO_ALTO = 0.38


def maceta_baja(caja_xyxy, h: float) -> bool:
    y1, y2 = float(caja_xyxy[1]), float(caja_xyxy[3])
    return y2 >= CORTADA * h or (y1 + y2) / 2 >= CENTRO_BAJO * h


def maceta_alta(caja_xyxy, h: float) -> bool:
    y1, y2 = float(caja_xyxy[1]), float(caja_xyxy[3])
    return (y1 + y2) / 2 <= CENTRO_ALTO * h and y2 < CORTADA * h


class ServoBajo:
    def __init__(self, reposo_s: float = 4.0, min_deg: int = MIN_DEG, reposo_deg: int = REPOSO_DEG, persistencia: int = PERSISTENCIA):
        self.persistencia = persistencia
        self._n_baja = 0
        self.reposo_s = reposo_s
        self.min_deg = min_deg
        self.reposo_deg = reposo_deg
        self._t_visto = -1e9

    def paso(self, cajas: list, h: float, servo_deg: float, ahora: float):
        """cajas: lista de xyxy de macetas; h: alto de la imagen; servo_deg: ángulo actual; ahora: segundos."""
        baja = bool(cajas) and any(maceta_baja(c, h) for c in cajas)
        self._n_baja = self._n_baja + 1 if baja else 0
        if cajas:
            self._t_visto = ahora
            if baja:
                if self._n_baja < self.persistencia:
                    return None                                 # aún puede ser un cuadro suelto
                nuevo = max(self.min_deg, servo_deg - PASO_BAJA_DEG)
            elif all(maceta_alta(c, h) for c in cajas) and servo_deg < self.reposo_deg:
                nuevo = min(self.reposo_deg, servo_deg + PASO_SUBE_DEG)
            else:
                return None
        elif ahora - self._t_visto >= self.reposo_s and abs(servo_deg - self.reposo_deg) > 0.5:
            nuevo = servo_deg + max(-PASO_REPOSO_DEG, min(PASO_REPOSO_DEG, self.reposo_deg - servo_deg))
        else:
            return None
        return None if abs(nuevo - servo_deg) < 0.5 else int(round(nuevo))
