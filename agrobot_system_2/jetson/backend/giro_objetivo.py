"""GIRO HASTA UN RUMBO (2026-11-02): «gira hasta mirar a 135°», no «gira N pulsos de 12°».

Medido en el robot real: los pulsos de giro de la exploración rinden lo que el suelo deje (L del 28-oct: pulsos de 6° daban ≈1°; exterior del 27-oct: mediana 1,5°
por pulso). El explorador calculaba «n_giro = hueco / grados_por_pulso» y daba por hecho que cada pulso giraba lo pedido: si el suelo se comía el giro, el robot
quedaba cruzado sin saberlo.

Esta clase cierra el lazo sobre el RUMBO de la pose (SLAM / vigía de la pose), que no deriva como la integral del giroscopio:
  · cada tramo es un giro de lazo cerrado con el giroscopio (giro_cerrado.py) de como mucho `tramo_max_deg`;
  · después de cada tramo se mide el rumbo real y se pide sólo lo que falta (si se pasó, el siguiente tramo vuelve);
  · un tramo que gira menos de `avance_min_deg` es un FALLO (falta de par, rueda en el aire, enganchado); `fallos_max` seguidos = «sin avance», que el
    explorador cuenta como atasco en vez de seguir insistiendo;
  · termina con el rumbo dentro de `tol_deg`, o tras `tramos_max` tramos.

Lógica pura, sin hardware ni reloj: `siguiente(theta_deg)` devuelve la orden. Convención ROS: grados > 0 = izquierda."""
from __future__ import annotations

import asyncio


SALTO_POSE_DEG = 25.0          # pose y giroscopio en desacuerdo por más de esto en un tramo = la pose saltó


def dif_deg(a: float, b: float) -> float:
    """a - b normalizado a (-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


class GiroObjetivo:
    def __init__(self, rumbo_deg: float, *, tol_deg: float = 5.0, tramo_max_deg: float = 45.0, avance_min_deg: float = 2.0,
                 fallos_max: int = 2, tramos_max: int = 8):
        self.rumbo = float(rumbo_deg) % 360.0
        self.tol = abs(float(tol_deg))
        self.tramo_max = abs(float(tramo_max_deg))
        self.avance_min = abs(float(avance_min_deg))
        self.fallos_max = int(fallos_max)
        self.tramos_max = int(tramos_max)
        self.theta0: float | None = None
        self._theta_prev: float | None = None
        self._pedido_prev = 0.0
        self.tramos = 0
        self.fallos = 0
        self.girado = 0.0               # ° girados en total (con signo, + izquierda)
        self.terminado = False
        self.motivo: str | None = None
        self.error_final: float | None = None

    def _fin(self, motivo: str, error: float) -> dict:
        self.terminado, self.motivo, self.error_final = True, motivo, error
        return {"tipo": "fin", "motivo": motivo}

    def siguiente(self, theta_deg: float) -> dict:
        """Una decisión con el rumbo medido ahora. {"tipo": "girar", "grados": ±x} | {"tipo": "fin", "motivo": str}."""
        if self.terminado:
            return {"tipo": "fin", "motivo": self.motivo}
        theta = float(theta_deg) % 360.0
        if self.theta0 is None:
            self.theta0 = theta
        else:
            hecho = dif_deg(theta, self._theta_prev)
            self.girado += hecho
            if self._pedido_prev:
                avance = hecho * (1.0 if self._pedido_prev > 0 else -1.0)      # lo girado EN EL SENTIDO pedido
                self.fallos = self.fallos + 1 if avance < min(self.avance_min, 0.5 * abs(self._pedido_prev)) else 0
        self._theta_prev = theta
        error = dif_deg(self.rumbo, theta)
        if abs(error) <= self.tol:
            return self._fin("alcanzado", error)
        if self.fallos >= self.fallos_max:
            return self._fin("sin avance", error)
        if self.tramos >= self.tramos_max:
            return self._fin("demasiados tramos", error)
        grados = max(-self.tramo_max, min(self.tramo_max, error))
        self._pedido_prev = grados
        self.tramos += 1
        return {"tipo": "girar", "grados": grados}

    def resumen(self) -> dict:
        return {"rumbo_deg": round(self.rumbo, 1), "girado_deg": round(self.girado, 1), "tramos": self.tramos, "fallos": self.fallos,
                "motivo": self.motivo, "error_deg": None if self.error_final is None else round(self.error_final, 1)}


async def girar_a_rumbo(conductor, rumbo_deg: float, get_pose, get_gyro_dps, cfg: dict, seguir=lambda: True) -> dict | None:
    """Bucle de `GiroObjetivo` sobre un `ConductorPulsos`: tramos de lazo cerrado con el giroscopio midiendo el rumbo con `get_pose()` tras cada uno.
    Lo usan el backend (jetson_server._girar_a_rumbo) y el simulador (correr.py), así el simulador prueba el mismo código que el robot.
    None si no se puede (sin IMU fresco, sin pose o el conductor ocupado): quien llama usa los pulsos de siempre. `seguir()` False corta entre tramos."""
    if get_gyro_dps() is None or get_pose() is None or conductor.activo:
        return None
    g = GiroObjetivo(rumbo_deg, tol_deg=float(cfg.get("giro_objetivo_tol_deg", 6.0)), tramo_max_deg=float(cfg.get("giro_objetivo_tramo_deg", 30.0)))
    w = None if cfg.get("giro_fuerza_aprendida", False) else 0.6
    theta_antes = giro_imu = None
    while True:
        pose = get_pose()
        if pose is None:
            g.motivo = "sin pose"
            break
        if theta_antes is not None and giro_imu is not None and abs(dif_deg(pose["theta_deg"], theta_antes) - giro_imu) > SALTO_POSE_DEG:
            g.motivo = "pose saltó"          # el rumbo de la pose cambió muy distinto de lo que midió el giroscopio (cierre de bucle del SLAM, salto):
            break                            # perseguir ese rumbo hizo dar 390° en el simulador; se para y el explorador vuelve a decidir
        orden = g.siguiente(pose["theta_deg"])
        if orden["tipo"] == "fin":
            break
        if not conductor.girar_grados(orden["grados"], get_gyro_dps, w_rps=w, t_max_s=max(2.0, abs(orden["grados"]) / 12.0)):
            g.motivo = "conductor ocupado"
            break
        theta_antes = pose["theta_deg"]
        while conductor.activo:
            await asyncio.sleep(0.05)
        await asyncio.sleep(0.3)                 # que la pose recoja el final del giro (inercia) antes de medir
        ug = getattr(conductor, "ultimo_giro", None) or {}
        giro_imu = ug.get("final_deg") if ug.get("final_deg") is not None else ug.get("girado_deg")
        if not seguir():
            g.motivo = "autonomía en pausa"
            break
    return {**g.resumen(), "w_giro": dict(conductor.w_giro)}
