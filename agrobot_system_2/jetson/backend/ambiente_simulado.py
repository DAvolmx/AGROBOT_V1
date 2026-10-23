"""Ambiente SIMULADO mientras el SCD40 está quemado (2026-10-23, para la presentación del 2026-10-24).

Cuando `calibration.ambiente_simulado` está activo y el SCD40 no entrega datos (campos en `_sensors_missing`), se rellenan CO₂,
temperatura y humedad con una caminata lenta alrededor de la última medida real del sensor (1349 ppm, 24,16 °C, 44,2 %,
grabación 19700101_000554). Cambian cada 5 s como el SCD40 real, con temperatura y humedad en sentido opuesto.
El payload lleva `_sensors_simulated` con los campos inventados: el dashboard los marca «sim» y las grabaciones quedan
distinguibles de medidas reales (no usar para análisis ni entrenamiento). Si el sensor vuelve a medir, se usan sus datos.
"""
from __future__ import annotations

import random
import time

BASE = {"co2": 1349.0, "tempSCD": 24.16, "humidity": 44.2}
LIMITES = {"co2": (1150.0, 1550.0), "tempSCD": (23.4, 25.0), "humidity": (41.0, 47.5)}
PERIODO_S = 5.0


class AmbienteSimulado:
    def __init__(self, semilla=None):
        self._rng = random.Random(semilla)
        self._v = dict(BASE)
        self._t = 0.0

    def _avanzar(self):
        r = self._rng
        dt = r.gauss(0, 0.04)                                    # °C por paso
        cambios = {"co2": r.gauss(0, 9.0), "tempSCD": dt, "humidity": -2.5 * dt + r.gauss(0, 0.08)}
        for k, d in cambios.items():
            lo, hi = LIMITES[k]
            v = self._v[k] + d + 0.03 * (BASE[k] - self._v[k])  # vuelve despacio a la base
            self._v[k] = min(hi, max(lo, v))

    def aplicar(self, payload: dict) -> dict:
        faltan = [k for k in BASE if k in payload.get("_sensors_missing", [])]
        if not faltan:
            return payload
        ahora = time.monotonic()
        if ahora - self._t >= PERIODO_S:
            self._t = ahora
            self._avanzar()
        for k in faltan:
            payload[k] = round(self._v[k]) if k == "co2" else round(self._v[k], 2 if k == "tempSCD" else 1)
        payload["_sensors_missing"] = [k for k in payload["_sensors_missing"] if k not in faltan]
        payload["_sensors_simulated"] = faltan
        return payload
