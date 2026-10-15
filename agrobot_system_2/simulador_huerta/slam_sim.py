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
  · Caídas del LIDAR: el CP2102N (USB) se cayó 4 veces en 15 min; cada caída
    dejaba ~9 s sin scans (4 s hasta que el vigía mata el driver colgado,
    3 s de respawn y el arranque del motor del C1).

Y los que se ven en el mapa real (captura del dashboard, 2026-09-29): paredes
DUPLICADAS y giradas unos grados, y rayos de "libre" que atraviesan paredes.
Es el casamiento de scans fallando al girar: el C1 da una vuelta cada 100 ms
y el chasis gira en el sitio a ~1-2 rad/s, así que entre dos scans el mundo
ha girado 6-11°. Pasada su ventana de búsqueda, slam_toolbox casa con el
mínimo equivocado, el rumbo salta y desde ese momento todo se pinta girado
hasta que un cierre de bucle lo devuelve a su sitio.
"""
import math

import numpy as np

# Ruido del casamiento de scans (rf2o + EKF), por metro recorrido y por radián
# girado. Los giros ensucian mucho más que los tramos rectos.
# Recorte de alcance del SLAM real (ver mundo.LIDAR_RANGO_MIN_SLAM).
RANGO_MIN_M = 0.18
DERIVA_POR_METRO = 0.05
DERIVA_POR_RADIAN = 0.08
SESGO_GIRO = 0.02              # el giróscopo tiene un sesgo residual
REESCALA_S = 2.0               # cada cuánto cambia el error de escala
# Sin scans solo queda la odometría (IMU + comandos): deriva mucho más.
DERIVA_SIN_LIDAR = 3.0
# Cierres de bucle: al reconocer una zona ya mapeada, la pose salta hacia la
# verdadera. Corrige, pero de golpe, y eso rompe rutas y waypoints grabados.
CELDA_RECONOCIMIENTO_M = 0.6
ESPERA_RECONOCIMIENTO_S = 20.0
CORRECCION_BUCLE = 0.75        # cuánto del error se corrige en el salto
SALTO_MINIMO_M = 0.05
# Casamiento de scans al girar (ver el docstring del módulo).
LIDAR_PERIODO_S = 0.1
VENTANA_CASAMIENTO_RAD = math.radians(8.0)
P_FALLO_CASAMIENTO_MAX = 0.35  # por scan, girando al doble de la ventana
ERROR_FALLO_RUMBO_DEG = 6.0    # desviación típica del salto de rumbo
ERROR_FALLO_POS_M = 0.04
# Caídas del LIDAR según cómo esté conectado.
CAIDAS = {
    "usb":  {"cada_s": 225.0, "dura_s": (6.0, 12.0)},     # medido 2026-09-19
    "uart": {"cada_s": 3600.0, "dura_s": (0.3, 1.0)},     # sin USB: casi nunca
    "ninguna": None,
}


class FallosLidar:
    """Cuándo deja de llegar el scan. El resto del sistema tiene que
    sobrevivir a eso igual que en el robot: el freno por LIDAR bloquea el
    avance ("solo gira") y el SLAM navega a ciegas con la odometría."""

    def __init__(self, conexion="uart", semilla=0):
        self.rng = np.random.default_rng(semilla + 7)
        self.conexion = conexion
        self.caido_hasta = -1.0
        self.caidas = 0
        self.t_caido = 0.0
        self._proxima = self._sortear_proxima(0.0)

    def _sortear_proxima(self, t):
        p = CAIDAS.get(self.conexion)
        return math.inf if not p else t + float(self.rng.exponential(p["cada_s"]))

    def cambiar(self, conexion):
        self.conexion = conexion if conexion in CAIDAS else "uart"
        self._proxima = self._sortear_proxima(0.0 if math.isinf(self._proxima) else self._proxima)

    def caido(self, t) -> bool:
        """¿Está sin scans en el instante t? Sortea la siguiente caída."""
        if t < self.caido_hasta:
            return True
        if t >= self._proxima:
            lo, hi = CAIDAS[self.conexion]["dura_s"]
            self.caido_hasta = t + float(self.rng.uniform(lo, hi))
            self.t_caido += self.caido_hasta - t
            self.caidas += 1
            self._proxima = self._sortear_proxima(self.caido_hasta)
            return True
        return False


class SlamSimulado:
    """Estimación de pose + mapa de ocupación, con los vicios del de verdad."""

    def __init__(self, pose_inicial, limites, resolucion=0.05, semilla=0,
                 ruido=1.0, cierres=True):
        self.x, self.y, self.theta = pose_inicial
        self.resolucion = resolucion
        self.limites = limites
        self.ruido = float(ruido)          # 0 = SLAM perfecto, 1 = como el real
        self.cierres = cierres
        self.fallos_casamiento = True
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
        self.casamientos_fallidos = 0
        self.sin_lidar = False
        self.scans = 0
        self.t = 0.0
        self.eventos = []                  # los recoge el episodio
        self._escala_v = self._escala_w = 0.0
        self._resorteo = 0.0
        self._pasos_rayo = np.arange(0.0, 8.0, resolucion, dtype=np.float32)

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
        """Integra el movimiento con el ruido de la estimación.

        El error es de ESCALA y se mantiene durante cada maniobra (se sortea
        cada ~2 s): un giro de 90° medido un 4,5 % corto deja 4° de rumbo
        mal para siempre. Sorteado en cada paso de 20 ms se promediaba y el
        SLAM simulado apenas derivaba, lejos del 14-30 % medido."""
        self.t += dt
        k = self.ruido * (DERIVA_SIN_LIDAR if self.sin_lidar else 1.0)
        if self.t >= self._resorteo:
            self._escala_v = self.rng.normal(0, DERIVA_POR_METRO)
            self._escala_w = self.rng.normal(0, DERIVA_POR_RADIAN)
            self._resorteo = self.t + REESCALA_S
        ev = v * (1 + k * self._escala_v)
        eo = omega * (1 + k * self._escala_w)
        eo += k * SESGO_GIRO * omega

        th = self.theta + eo * dt / 2
        self.x += ev * math.cos(th) * dt
        self.y += ev * math.sin(th) * dt
        self.theta = (th + eo * dt / 2 + math.pi) % (2 * math.pi) - math.pi
        self.recorrido_estimado += abs(ev) * dt

        if self.cierres and not self.sin_lidar:
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
        self.eventos.append({"evento": "slam_cierre_bucle", "salto_cm": round(salto * 100),
                             "giro_deg": round(math.degrees(dth), 1)})
        return salto

    def _casamiento(self, omega):
        """Girando más rápido de lo que el casamiento sigue, a veces casa
        con el mínimo equivocado: el rumbo salta unos grados."""
        if not self.fallos_casamiento or self.ruido <= 0:
            return
        giro_scan = abs(omega) * LIDAR_PERIODO_S
        exceso = (giro_scan - VENTANA_CASAMIENTO_RAD) / VENTANA_CASAMIENTO_RAD
        if exceso <= 0:
            return
        if self.rng.random() > P_FALLO_CASAMIENTO_MAX * min(1.0, exceso) * self.ruido:
            return
        dth = math.radians(self.rng.normal(0, ERROR_FALLO_RUMBO_DEG)) * self.ruido
        self.theta = (self.theta + dth + math.pi) % (2 * math.pi) - math.pi
        self.x += self.rng.normal(0, ERROR_FALLO_POS_M) * self.ruido
        self.y += self.rng.normal(0, ERROR_FALLO_POS_M) * self.ruido
        self.casamientos_fallidos += 1
        self.eventos.append({"evento": "slam_casamiento_fallido",
                             "giro_por_scan_deg": round(math.degrees(giro_scan), 1),
                             "salto_rumbo_deg": round(math.degrees(dth), 1)})

    def corregir(self, x, y, theta):
        """El operador recoloca el robot en el mapa (como /initialpose)."""
        salto = math.dist((self.x, self.y), (x, y))
        self.x, self.y, self.theta = x, y, theta
        self.saltos += 1
        self.salto_total_m += salto
        return salto

    # ------------------------------------------------------------------ mapa
    def ingerir_scan(self, puntos, alcance=8.0, omega=0.0):
        """Pinta el scan en el mapa DESDE LA POSE ESTIMADA.

        Aquí es donde se ve el daño de la deriva: si la pose está mal, las
        paredes se pintan corridas y el mapa sale con las paredes dobles o
        emborronadas, igual que en el robot real. Vectorizado: antes era un
        linspace por rayo (21 ms por scan, un tercio del episodio)."""
        self.scans += 1
        if not puntos:
            return
        self._casamiento(omega)
        # Como el filtro_scan_slam del robot real: lo que está a menos de
        # 0,18 m es el propio chasis (mástil, cable) y no entra al mapa.
        sub = [p for p in puntos[::2] if p["distance_m"] >= RANGO_MIN_M]
        if not sub:
            return
        ang = np.radians(np.fromiter((p["angle_deg"] for p in sub), np.float32, len(sub)))
        d = np.minimum(np.fromiter((p["distance_m"] for p in sub), np.float32, len(sub)), alcance)
        ang += self.theta
        cos, sin = np.cos(ang), np.sin(ang)
        x0, y0, _, _ = self.limites
        res = self.resolucion
        ox, oy = (self.x - x0) / res, (self.y - y0) / res

        # Libre: muestras cada `res` a lo largo de cada rayo, hasta la celda
        # anterior al impacto.
        s = self._pasos_rayo / res
        dentro_rayo = s[None, :] < (d[:, None] / res - 1.0)
        gx = (ox + cos[:, None] * s[None, :])[dentro_rayo].astype(np.int32)
        gy = (oy + sin[:, None] * s[None, :])[dentro_rayo].astype(np.int32)
        ok = (gx >= 0) & (gx < self.ancho) & (gy >= 0) & (gy < self.alto)
        n = self.ancho * self.alto
        libres = np.bincount(gy[ok] * self.ancho + gx[ok], minlength=n)

        impacto = d < alcance - 1e-3
        hx = (ox + cos[impacto] * d[impacto] / res).astype(np.int32)
        hy = (oy + sin[impacto] * d[impacto] / res).astype(np.int32)
        ok = (hx >= 0) & (hx < self.ancho) & (hy >= 0) & (hy < self.alto)
        ocupadas = np.bincount(hy[ok] * self.ancho + hx[ok], minlength=n)

        plano = self.rejilla.reshape(-1)
        plano -= 0.12 * libres
        plano += 0.9 * ocupadas
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
             "casamientos_fallidos": self.casamientos_fallidos,
             "sin_lidar": self.sin_lidar,
             "pose": {k: round(v, 3) for k, v in self.pose.items()}}
        if pose_real is not None:
            d["error_m"] = round(self.error(pose_real), 3)
            d["error_rumbo_deg"] = round(self.error_rumbo_deg(pose_real), 1)
        return d
