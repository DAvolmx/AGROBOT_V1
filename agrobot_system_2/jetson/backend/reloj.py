"""Detector de saltos del reloj de pared (2026-10-27). La Jetson no tiene RTC con pila: al arrancar el reloj vuelve al último valor guardado y NTP lo corrige minutos después con un
SALTO (el 27-oct fue de +711,5 s en plena misión). El reloj monótono no salta, así que el salto es el cambio de (pared − monótono) entre dos instantes.
Lógica pura: `paso(pared, mono)` devuelve los segundos que saltó el reloj de pared desde la última llamada (o None si no hubo salto)."""
from __future__ import annotations


class DetectorSalto:
    def __init__(self, umbral_s: float = 2.0):
        self.umbral = float(umbral_s)
        self._desfase: float | None = None
        self.saltos: list[dict] = []

    def paso(self, pared: float, mono: float) -> float | None:
        desfase = pared - mono
        if self._desfase is None:
            self._desfase = desfase
            return None
        delta = desfase - self._desfase
        self._desfase = desfase
        if abs(delta) > self.umbral:
            self.saltos.append({"mono": round(mono, 1), "pared": round(pared, 1), "delta_s": round(delta, 1)})
            return delta
        return None
