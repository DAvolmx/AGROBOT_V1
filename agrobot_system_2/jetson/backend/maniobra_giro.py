"""MANIOBRA DE GIRO COMPLETO (2026-10-27): media vuelta (180°) o vuelta entera (360°) en el sitio, sin que el robot se vaya por el arco.

Un giro de este robot (eje pasivo, tracción delantera) no es un pivote: cada pulso gira ~6° y AVANZA ~5 cm en arco (ver mx_giro_comp). Un 180° hecho
de golpe acaba a 20–30 cm del punto de partida y puede tocar la pared. Esta maniobra lo descompone en TRAMOS de lazo cerrado con el giroscopio
(giro_cerrado.py, ángulo exacto por tramo) y entre tramo y tramo corrige la deriva:

  · tras cada tramo mide cuánto se ha ido el robot del punto de partida (pose del SLAM) y cuánto ha girado en total (rumbo del SLAM, que manda sobre
    la integral del giroscopio, que deriva);
  · si la deriva supera `deriva_max_m` y hay hueco detrás, un pulso atrás (el arco del giro se paga con marcha atrás); si el hueco está delante y
    detrás hay pared, un pulso adelante no se hace (el robot sigue girando con la deriva que tenga mientras nada toque);
  · antes de cada tramo exige hueco: por delante y por detrás >= `hueco_min_m`; si no lo hay, intenta un pulso atrás o adelante hacia el lado libre
    y, si no cabe ninguno, PARA («sin espacio»);
  · si `tramos_sin_avance` tramos seguidos giran menos de `avance_min_deg` (robot atascado: se vio en el robot real el 27-oct con 18 pulsos de pivote
    sin mover el rumbo) PARA («sin avance de rumbo») en vez de seguir intentándolo.

Lógica pura, sin hardware ni reloj: `siguiente()` recibe el estado y devuelve la orden; el bucle que la ejecuta está en jetson_server.py
(`/api/maniobra_giro`). Convención: grados > 0 = izquierda (antihorario, ROS), igual que giro_cerrado.py. 180 y 360 son los casos de uso; admite cualquier
múltiplo (±720° como tope)."""
from __future__ import annotations

import math


def _dif(a_deg: float, b_deg: float) -> float:
    """a - b normalizado a (-180, 180]."""
    d = (a_deg - b_deg + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


class ManiobraGiro:
    def __init__(self, grados: float = 180.0, *, tramo_deg: float = 30.0, hueco_min_m: float = 0.25, deriva_max_m: float = 0.10,
                 avance_min_deg: float = 4.0, tramos_sin_avance: int = 3, tol_deg: float = 6.0, tramos_max: int = 40):
        self.objetivo = float(grados)
        if not 1.0 <= abs(self.objetivo) <= 720.0:
            raise ValueError("grados en [1, 720] (en valor absoluto)")
        self.signo = 1.0 if self.objetivo > 0 else -1.0
        self.tramo = abs(float(tramo_deg))
        self.hueco_min = float(hueco_min_m)
        self.deriva_max = float(deriva_max_m)
        self.avance_min = float(avance_min_deg)
        self.sin_avance_max = int(tramos_sin_avance)
        self.tol = float(tol_deg)
        self.tramos_max = int(tramos_max)
        # estado
        self.theta0: float | None = None
        self.xy0: tuple[float, float] | None = None
        self._theta_prev: float | None = None
        self.girado = 0.0                  # ° acumulados en el sentido pedido (el rumbo del SLAM, sumando los incrementos de cada tramo)
        self.tramos = 0
        self.atras = 0
        self.adelante = 0
        self._sin_avance = 0
        self._atras_seguidos = 0           # pulsos de corrección seguidos: tope 2, luego se gira aunque quede deriva (evita un bucle atrás-atrás)
        self._esperando_tramo = False
        self.terminado = False
        self.motivo: str | None = None

    # -- helpers -----------------------------------------------------------------------------------------------------------------------------
    def _deriva(self, xy) -> float:
        """Cuánto se ha ido del punto de partida (m), en cualquier dirección."""
        return math.hypot(xy[0] - self.xy0[0], xy[1] - self.xy0[1])

    def _adelantado(self, xy, theta_deg: float) -> float:
        """Cuánto está el robot POR DELANTE de su punto de partida, medido sobre su rumbo actual (m; negativo = por detrás). Es lo que un pulso atrás puede corregir."""
        t = math.radians(theta_deg)
        return (xy[0] - self.xy0[0]) * math.cos(t) + (xy[1] - self.xy0[1]) * math.sin(t)

    def _fin(self, motivo: str) -> dict:
        self.terminado = True
        self.motivo = motivo
        return {"tipo": "fin", "motivo": motivo}

    def _actualiza_giro(self, theta_deg: float):
        """Suma lo girado desde la última medida (en el sentido pedido) y descarta saltos de otro sentido mayores que un tramo (ruido del SLAM)."""
        inc = _dif(theta_deg, self._theta_prev) * self.signo
        self._theta_prev = theta_deg
        if self._esperando_tramo:
            self._esperando_tramo = False
            if inc < self.avance_min:
                self._sin_avance += 1
            else:
                self._sin_avance = 0
        if inc > -2.0 * self.tramo:                      # un retroceso grande de golpe es un salto del SLAM: no se resta
            self.girado += inc

    # -- API ---------------------------------------------------------------------------------------------------------------------------------
    def siguiente(self, theta_deg: float, xy: tuple[float, float], frente_m: float | None, atras_m: float | None) -> dict:
        """Una decisión. theta_deg y xy = pose del SLAM; frente_m / atras_m = huecos libres (None = sin retorno, es decir libre).
        Devuelve {"tipo": "girar", "grados": ±x} | {"tipo": "atras"} | {"tipo": "adelante"} | {"tipo": "fin", "motivo": str}."""
        if self.terminado:
            return {"tipo": "fin", "motivo": self.motivo}
        if self.theta0 is None:
            self.theta0, self.xy0, self._theta_prev = theta_deg, (float(xy[0]), float(xy[1])), theta_deg
        else:
            self._actualiza_giro(theta_deg)
        resto = abs(self.objetivo) - self.girado
        if resto <= self.tol:
            return self._fin("completa")
        if self._sin_avance >= self.sin_avance_max:
            return self._fin("sin avance de rumbo (atascado)")
        if self.tramos >= self.tramos_max:
            return self._fin("demasiados tramos")
        libre_f = frente_m is None or frente_m >= self.hueco_min
        libre_a = atras_m is None or atras_m >= self.hueco_min
        if self._adelantado(xy, theta_deg) > self.deriva_max and self._atras_seguidos < 2:
            if libre_a:
                self.atras += 1
                self._atras_seguidos += 1
                return {"tipo": "atras"}
            if not libre_f:                              # ni detrás ni delante hay hueco para corregir
                return self._fin("sin espacio")
        if not libre_f and not libre_a:
            return self._fin("sin espacio")
        if not libre_f and self._atras_seguidos < 2:     # pared delante: se gana sitio hacia atrás antes de seguir girando
            self.atras += 1
            self._atras_seguidos += 1
            return {"tipo": "atras"}
        if not libre_a and self._atras_seguidos < 2:
            self.adelante += 1
            self._atras_seguidos += 1
            return {"tipo": "adelante"}
        self._atras_seguidos = 0
        self.tramos += 1
        self._esperando_tramo = True
        return {"tipo": "girar", "grados": self.signo * min(self.tramo, resto)}

    def resumen(self) -> dict:
        return {"pedido_deg": self.objetivo, "girado_deg": round(self.girado, 1), "tramos": self.tramos, "atras": self.atras, "adelante": self.adelante,
                "motivo": self.motivo}
