"""
SLAM simulado: la pose que el robot CREE tener, y el mapa que construye.
=======================================================================

Hasta ahora el software del simulador leía la pose verdadera, así que nunca
se equivocaba de sitio. El robot real no tiene eso: tiene una estimación que
deriva y que, de vez en cuando, PEGA UN SALTO cuando el SLAM cierra un bucle.

Los números salen de los registros reales del robot (2026-09-19):

  · Deriva: 3,42 m tras 25 m recorridos (13,7 %) en una sesión y 5,38 m tras
    18 m (29,9 %) en otra. Casi toda aparece en los giros: es un skid-steer
    que patina, y el casamiento de scans no ve ese patinaje.
  · Saltos: 22 saltos de más de 25 cm en medio segundo en 154 s, que suman
    6,8 m de los 15 m "recorridos" según el registro. Eso no es el robot
    moviéndose: es slam_toolbox corrigiéndose de golpe al reconocer un sitio.

Con esto el simulador reproduce el problema de verdad: el robot planifica
sobre un mapa que no está donde él cree, y la ruta que grabó ayer ya no
coincide con la de hoy.
"""
import math

import numpy as np

# Ruido del casamiento de scans (rf2o + EKF), por metro recorrido y por radián
# girado. Los giros ensucian mucho más que los tramos rectos.
DERIVA_POR_METRO = 0.035
DERIVA_POR_RADIAN = 0.045
SESGO_GIRO = 0.02              # el giróscopo tiene un sesgo residual
# Cierres de bucle: al reconocer una zona ya mapeada, la pose salta hacia la
# verdadera. Corrige, pero de golpe, y eso rompe rutas y waypoints grabados.
CELDA_RECONOCIMIENTO_M = 0.6
ESPERA_RECONOCIMIENTO_S = 20.0
CORRECCION_BUCLE = 0.75        # cuánto del error se corrige en el salto
SALTO_MINIMO_M = 0.05


class SlamSimulado:
    """Estimación de pose + mapa de ocupación, con los vicios del de verdad."""

    def __init__(self, pose_inicial, limites, resolucion=0.05, semilla=0,
                 ruido=1.0, cierres=True):
        self.x, self.y, self.theta = pose_inicial
        self.resolucion = resolucion
        self.limites = limites
        self.ruido = float(ruido)          # 0 = SLAM perfecto, 1 = como el real
        self.cierres = cierres
        self.rng = np.random.default_rng(semilla)
        x0, y0, x1, y1 = limites
        self.ancho = int((x1 - x0) / resolucion)
        self.alto = int((y1 - y0) / resolucion)
        # log-odds de ocupación, como cualquier rejilla de ocupación.
        self.rejilla = np.zeros((self.alto, self.ancho), np.float32)
        self.visitadas = {}
        self.saltos = 0
        self.salto_total_m = 0.0
        self.recorrido_estimado = 0.0
        self.t = 0.0

    # ------------------------------------------------------------------ pose
    @property
    def pose(self) -> dict:
        return {"x": self.x, "y": self.y, "theta_deg": math.degrees(self.theta) % 360}

    def error(self, pose_real) -> float:
        return math.dist((self.x, self.y), (pose_real[0], pose_real[1]))

    def error_rumbo_deg(self, pose_real) -> float:
        d = math.degrees(self.theta - pose_real[2])
        return (d + 180) % 360 - 180

    def paso(self, dt, v, omega, pose_real):
        """Integra el movimiento con el ruido de la estimación."""
        self.t += dt
        k = self.ruido
        dist = abs(v) * dt
        giro = abs(omega) * dt
        # Error proporcional a lo andado y a lo girado, más el sesgo del giro.
        ev = v * (1 + k * self.rng.normal(0, DERIVA_POR_METRO)) if dist else 0.0
        eo = omega * (1 + k * self.rng.normal(0, DERIVA_POR_RADIAN)) if giro else 0.0
        eo += k * SESGO_GIRO * omega * dt

        th = self.theta + eo * dt / 2
        self.x += ev * math.cos(th) * dt
        self.y += ev * math.sin(th) * dt
        self.theta = (th + eo * dt / 2 + math.pi) % (2 * math.pi) - math.pi
        self.recorrido_estimado += abs(ev) * dt

        if self.cierres:
            self._quizas_cerrar_bucle(pose_real)

    def _quizas_cerrar_bucle(self, pose_real):
        """Al volver a una zona ya vista, el SLAM se recoloca DE GOLPE."""
        celda = (round(self.x / CELDA_RECONOCIMIENTO_M),
                 round(self.y / CELDA_RECONOCIMIENTO_M))
        visto = self.visitadas.get(celda)
        self.visitadas[celda] = self.t
        if visto is None or self.t - visto < ESPERA_RECONOCIMIENTO_S:
            return
        dx = (pose_real[0] - self.x) * CORRECCION_BUCLE
        dy = (pose_real[1] - self.y) * CORRECCION_BUCLE
        dth = ((pose_real[2] - self.theta + math.pi) % (2 * math.pi) - math.pi) * CORRECCION_BUCLE
        salto = math.hypot(dx, dy)
        if salto < SALTO_MINIMO_M:
            return
        self.x += dx
        self.y += dy
        self.theta += dth
        self.saltos += 1
        self.salto_total_m += salto
        return salto

    def corregir(self, x, y, theta):
        """El operador recoloca el robot en el mapa (como /initialpose)."""
        salto = math.dist((self.x, self.y), (x, y))
        self.x, self.y, self.theta = x, y, theta
        self.saltos += 1
        self.salto_total_m += salto
        return salto

    # ------------------------------------------------------------------ mapa
    def _a_celda(self, x, y):
        x0, y0, _, _ = self.limites
        return int((x - x0) / self.resolucion), int((y - y0) / self.resolucion)

    def ingerir_scan(self, puntos, alcance=8.0):
        """Pinta el scan en el mapa DESDE LA POSE ESTIMADA.

        Aquí es donde se ve el daño de la deriva: si la pose está mal, las
        paredes se pintan corridas y el mapa sale con las paredes dobles o
        emborronadas, igual que en el robot real."""
        if not puntos:
            return
        cx, cy = self._a_celda(self.x, self.y)
        for p in puntos[::3]:                     # 1 de cada 3 rayos: sobra
            ang = self.theta + math.radians(p["angle_deg"])
            d = min(p["distance_m"], alcance)
            hx, hy = self.x + d * math.cos(ang), self.y + d * math.sin(ang)
            gx, gy = self._a_celda(hx, hy)
            # Libre por el camino (Bresenham barato: muestreo del segmento).
            n = max(2, int(d / self.resolucion))
            xs = np.linspace(cx, gx, n).astype(int)
            ys = np.linspace(cy, gy, n).astype(int)
            dentro = (xs >= 0) & (xs < self.ancho) & (ys >= 0) & (ys < self.alto)
            self.rejilla[ys[dentro][:-1], xs[dentro][:-1]] -= 0.12
            if 0 <= gx < self.ancho and 0 <= gy < self.alto and p["distance_m"] < alcance - 1e-3:
                self.rejilla[gy, gx] += 0.9
        np.clip(self.rejilla, -4.0, 6.0, out=self.rejilla)

    def mapa_imagen(self):
        """El mapa tal como lo vería el operador (BGR, y arriba es +Y)."""
        p = 1 / (1 + np.exp(-self.rejilla))        # log-odds -> probabilidad
        img = np.full(p.shape + (3,), 128, np.uint8)
        img[p < 0.35] = (245, 245, 245)            # libre
        img[p > 0.65] = (40, 45, 55)               # ocupado
        return np.flipud(img)

    def calidad_mapa(self, mundo) -> dict:
        """Cuánto se parece el mapa construido a la verdad del terreno.

        Se mide sobre las celdas que el SLAM marcó como ocupadas: qué
        fracción cae de verdad sobre una pared (a menos de 15 cm)."""
        ocupadas = np.argwhere(self.rejilla > 1.0)
        if not len(ocupadas) or not len(mundo.paredes):
            return {"celdas_ocupadas": int(len(ocupadas)), "aciertos_pct": None}
        x0, y0, _, _ = self.limites
        ys, xs = ocupadas[:, 0], ocupadas[:, 1]
        px = x0 + (xs + 0.5) * self.resolucion
        py = y0 + (ys + 0.5) * self.resolucion
        ax, ay, bx, by = mundo.paredes.T
        ex, ey = bx - ax, by - ay
        largo2 = np.maximum(ex * ex + ey * ey, 1e-9)
        t = np.clip(((px[:, None] - ax) * ex + (py[:, None] - ay) * ey) / largo2, 0, 1)
        d = np.hypot(px[:, None] - (ax + t * ex), py[:, None] - (ay + t * ey)).min(axis=1)
        return {"celdas_ocupadas": int(len(ocupadas)),
                "aciertos_pct": round(100 * float((d < 0.15).mean()), 1),
                "error_medio_cm": round(float(d.mean()) * 100, 1)}

    def resumen(self, pose_real=None) -> dict:
        d = {"saltos": self.saltos, "salto_total_m": round(self.salto_total_m, 2),
             "recorrido_estimado_m": round(self.recorrido_estimado, 2),
             "pose": {k: round(v, 3) for k, v in self.pose.items()}}
        if pose_real is not None:
            d["error_m"] = round(self.error(pose_real), 3)
            d["error_rumbo_deg"] = round(self.error_rumbo_deg(pose_real), 1)
        return d
