"""GIRO DE LAZO CERRADO con el giroscopio (2026-10-22): «gira N grados», no «gira un pulso de 0,3 s».

Medido en el robot real (586 pulsos de 8 sesiones, sin los contaminados por failsafe): un pulso de giro rota |Δrumbo| mediana 5,8° (cuartiles 1,4°–10,6°,
p90 19°) y el **29 % de los pulsos casi no gira (<2°)**; los rectos se desvían 2,4° de mediana (p90 9°). Con pulsos a tiempo fijo el robot es poco
controlable: sale lo que salga del suelo, la zona muerta de los motores y la carga.

Este controlador manda omega CONSTANTE y corta cuando el ángulo integrado del giroscopio, MÁS lo que todavía girará por inercia
(velocidad filtrada × `anticipo_s`), alcanza el objetivo. Si el robot no responde (pulso «muerto»: <1,5° tras 0,5 s) sube la fuerza un 15 % por paso hasta
`w_max`. Tope de tiempo `t_max_s`. En el simulador (40 robots con física sorteada) acierta 15°, 20° y 30° con error medio ≤ 1,2° y p90 3–4° (con pulsos fijos:
11,7° siempre, p90 del error 5,8° a 15° y 20° a 30°). La física del simulador es MÁS regular que la real: la mejora real hay que medirla con
`tools/probar_giro_cerrado.py`.

Convención: grados > 0 = izquierda (ROS, antihorario), igual que el giroscopio Z; omega devuelto en rad/s con ese mismo signo. Lógica pura, sin hardware.
"""
from __future__ import annotations


class GiroCerrado:
    def __init__(self, objetivo_deg: float, w_rps: float = 0.6, anticipo_s: float = 0.30, t_max_s: float = 2.0, w_max_rps: float = 1.2):
        self.objetivo = float(objetivo_deg)
        self.signo = 1.0 if self.objetivo >= 0 else -1.0
        self.w = abs(float(w_rps))
        self.w_max = float(w_max_rps)
        self.anticipo = float(anticipo_s)
        self.t_max = float(t_max_s)
        self.t = 0.0
        self.integ = 0.0               # ° girados (integral del giroscopio)
        self.rate_f = 0.0              # °/s filtrada
        self.terminado = False
        self.motivo: str | None = None
        self.integ_al_parar = 0.0
        self.rate_al_parar = 0.0
        self.final_deg: float | None = None

    def paso(self, gz_dps: float, dt: float) -> float:
        """Un tick: recibe la velocidad angular medida (°/s, + izquierda) y devuelve el omega a mandar (rad/s). 0 = ya terminó."""
        if self.terminado:
            return 0.0
        dt = max(0.0, min(dt, 0.3))
        self.integ += gz_dps * dt
        self.rate_f = 0.6 * self.rate_f + 0.4 * gz_dps
        self.t += dt
        prevision = self.integ + self.rate_f * self.anticipo
        if abs(self.objetivo) < 0.5:
            self._fin("nada que girar")
        elif prevision * self.signo >= abs(self.objetivo):
            self._fin("alcanzado")
        elif self.t >= self.t_max:
            self._fin("tiempo")
        if self.terminado:
            return 0.0
        if self.t > 0.5 and abs(self.integ) < 1.5:               # pulso muerto: más fuerza
            self.w = min(self.w_max, self.w * 1.15)
        return self.signo * self.w

    def _fin(self, motivo: str):
        self.terminado = True
        self.motivo = motivo
        self.integ_al_parar, self.rate_al_parar = self.integ, self.rate_f

    def observar_tras_parar(self, gz_dps: float, dt: float):
        """Sigue integrando el giroscopio con el motor ya parado: da el ángulo FINAL (inercia incluida)."""
        self.final_deg = (self.integ if self.final_deg is None else self.final_deg) + gz_dps * max(0.0, min(dt, 0.3))

    def anticipo_medido(self):
        """Constante de inercia medida en este giro: lo que giró después de parar / la velocidad con la que paró. None si no es fiable."""
        if self.final_deg is None or abs(self.rate_al_parar) < 5.0 or self.motivo != "alcanzado":
            return None
        c = (self.final_deg - self.integ_al_parar) / self.rate_al_parar
        return c if 0.0 <= c <= 1.0 else None

    def resumen(self) -> dict:
        return {"pedido_deg": self.objetivo, "girado_deg": round(self.integ, 1), "final_deg": None if self.final_deg is None else round(self.final_deg, 1),
                "duracion_s": round(self.t, 2), "motivo": self.motivo, "w_final": round(self.w, 2), "anticipo_s": self.anticipo}
