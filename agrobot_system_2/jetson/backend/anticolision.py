"""Anticolisión CON RETROCESO (2026-10-21, pedido del operador: «que el robot retroceda cuando vaya a chocar»).

Es una capa aparte del freno por LIDAR autónomo (`safety.freno_lidar_autonomo`, que el operador dejó APAGADO a propósito):
no frena «a distancia» ni bloquea el manejo; solo actúa cuando el robot VA A TOCAR algo delante.

  1. Va hacia adelante (v > 0,02 m/s) y el hueco libre delante es menor que `disparo_m` -> se cancela el avance.
  2. Si detrás hay sitio (hueco > retroceso_m + margen_atras_m) retrocede `retroceso_m` (a v_retroceso, por tiempo: no hay
     encoders); si no hay sitio, solo se detiene. Si durante el retroceso aparece algo detrás a menos de `parar_atras_m`, para.
  3. Después queda BLOQUEADO el avance contra ese obstáculo hasta que el operador suelte el mando (v ≈ 0 durante
     `soltar_s`) o el hueco delante sea claramente amplio: así no hay golpe-retroceso-golpe con la tecla apretada.
El giro nunca se bloquea. Sin scan fresco (más viejo que `scan_max_s`) no actúa (no puede ver).

LÍMITE HONESTO: solo ve lo que el LIDAR ve. El plano del LIDAR está a 37,5 cm: una caja, una bolsa, una manta o una maceta baja
NO se detectan y el robot los toca igual. Para eso haría falta un sensor bajo o conectar la segmentación de obstáculos de la cámara.

Es lógica pura (sin red ni reloj propio salvo el inyectable): `filtrar()` recibe lo que va a salir y devuelve lo que sale.
"""
from __future__ import annotations

import time

DEFECTOS = {
    "anticolision_activa": True,
    "anticolision_disparo_m": 0.12,        # hueco delante (hasta tocar) por debajo del cual se dispara
    "anticolision_retroceso_m": 0.15,
    "anticolision_v_retroceso": 0.15,      # m/s (el conductor por pulsos retrocede a ~0,19 como máximo)
    "anticolision_margen_atras_m": 0.20,   # hueco extra detrás exigido para retroceder
    "anticolision_parar_atras_m": 0.08,    # si detrás queda menos que esto, se corta el retroceso
    "anticolision_soltar_s": 0.3,
    "anticolision_scan_max_s": 1.0,
}

LIBRE, RETROCEDIENDO, BLOQUEADO = "libre", "retrocediendo", "bloqueado"


class Anticolision:
    def __init__(self, reloj=time.monotonic):
        self.reloj = reloj
        self.estado = LIBRE
        self._t_fin = 0.0
        self._t_quieto = None            # desde cuándo el operador no pide avanzar (en BLOQUEADO)
        self.disparos = 0
        self.retrocesos = 0
        self.alto_sin_espacio = 0

    def _cfg(self, safety: dict, k: str):
        return safety.get(k, DEFECTOS[k])

    def filtrar(self, v: float, omega: float, dist: dict, edad_scan_s, safety: dict):
        """Devuelve (v, omega, evento|None, reiniciar_rampa). `evento` es un dict para avisar al operador."""
        c = lambda k: self._cfg(safety, k)
        if not c("anticolision_activa"):
            self.estado = LIBRE
            return v, omega, None, False
        ahora = self.reloj()
        adelante, atras = dist.get("adelante"), dist.get("atras")
        scan_ok = edad_scan_s is not None and edad_scan_s <= c("anticolision_scan_max_s")

        if self.estado == RETROCEDIENDO:
            peligro_atras = scan_ok and atras is not None and atras < c("anticolision_parar_atras_m")
            if ahora >= self._t_fin or peligro_atras or not scan_ok:
                self.estado = BLOQUEADO
                self._t_quieto = None
                ev = {"tipo": "fin_retroceso", "motivo": "algo detrás" if peligro_atras else ("sin scan" if not scan_ok else "completo")}
                return 0.0, omega, ev, True
            return -c("anticolision_v_retroceso"), 0.0, None, False

        if self.estado == BLOQUEADO:
            pide_avanzar = v > 0.01
            if not pide_avanzar:
                if self._t_quieto is None:
                    self._t_quieto = ahora
                if ahora - self._t_quieto >= c("anticolision_soltar_s"):
                    self.estado = LIBRE
            else:
                self._t_quieto = None
                if scan_ok and (adelante is None or adelante > c("anticolision_disparo_m") + 0.40):
                    self.estado = LIBRE
            if self.estado == BLOQUEADO and pide_avanzar:
                return 0.0, omega, None, False                 # el avance contra el obstáculo no sale
            if self.estado == BLOQUEADO:
                return v, omega, None, False

        # LIBRE
        if v > 0.02 and scan_ok and adelante is not None and adelante < c("anticolision_disparo_m"):
            self.disparos += 1
            hay_sitio = atras is not None and atras >= c("anticolision_retroceso_m") + c("anticolision_margen_atras_m")
            if hay_sitio or atras is None:                     # None = nada a la vista detrás: hay sitio
                self.estado = RETROCEDIENDO
                self.retrocesos += 1
                self._t_fin = ahora + c("anticolision_retroceso_m") / max(c("anticolision_v_retroceso"), 0.05)
                return (-c("anticolision_v_retroceso"), 0.0,
                        {"tipo": "retroceso", "delante_m": round(float(adelante), 3), "detras_m": None if atras is None else round(float(atras), 3)}, True)
            self.estado = BLOQUEADO
            self._t_quieto = None
            self.alto_sin_espacio += 1
            return 0.0, omega, {"tipo": "alto_sin_espacio", "delante_m": round(float(adelante), 3), "detras_m": round(float(atras), 3)}, True
        return v, omega, None, False

    def resumen(self) -> dict:
        return {"estado": self.estado, "disparos": self.disparos, "retrocesos": self.retrocesos, "alto_sin_espacio": self.alto_sin_espacio}
