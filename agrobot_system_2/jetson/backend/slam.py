"""
Estimador de pose por fusión de sensores + evasión reactiva de obstáculos.

Aviso honesto: esto NO es un SLAM completo (no construye un mapa de
ocupación ni cierra loops). Es la pieza que sí tiene sentido correr en
este backend con lo que hay disponible ahora mismo: sin encoders, sin
robot_localization/nav2 todavía instalados. Lo que hace:

  1. PoseEstimator — fusiona la velocidad COMANDADA (integración de
     vx/omega, ya que no hay encoder que mida la real) con la velocidad
     angular del giroscopio (MPU6050, vía telemetría LoRa) en un filtro
     complementario. El robot es DIFERENCIAL (skid-steer): se mueve solo
     sobre su eje longitudinal, no puede desplazarse de costado. Da una
     estimación de (x, y, θ) razonable para distancias cortas, con el
     error de deriva típico de dead-reckoning — sirve para el radar del
     dashboard, NO para navegación de precisión.

     ⚠️ En un skid-steer el giro se hace PATINANDO las ruedas, así que la
     omega comandada es peor predictor que en un robot con ruedas que
     ruedan limpio. Por eso `pose_gyro_trust` está alto (0.8): conviene
     creerle bastante más al giroscopio que al comando.
  2. LidarReactive — ingesta de un scan 2D (lista de puntos ángulo/
     distancia) y evasión reactiva simple: si hay un obstáculo en la
     DIRECCIÓN DEL MOVIMIENTO y más cerca que el umbral, recorta la
     velocidad comandada (frena antes de chocar) sin necesidad de un
     costmap ni un planner. El RPLIDAR C1 barre 360°, así que se vigila
     hacia adelante al avanzar y HACIA ATRÁS al retroceder — mirar siempre
     al frente dejaba el retroceso sin ninguna protección.

Todos los umbrales son parámetros, no constantes — ver config_store.py
sección "slam", editable desde la pestaña Parámetros del dashboard.

Para SLAM real (mapa + loop closure + planeación global) esto se
reemplaza por sllidar_ros2 + robot_localization (EKF) + nav2, tal como
ya quedó anotado en el README. Esta pieza deja de ser necesaria en ese
momento — o puede seguir usándose como fallback reactivo de bajo nivel
incluso con nav2 corriendo (doble capa de seguridad).
"""
import math
import time


class PoseEstimator:
    def __init__(self, gyro_weight: float = 0.8):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.gyro_weight = gyro_weight  # 0=solo dead-reckoning, 1=solo giroscopio
        self._last_t = None
        self._last_v_cmd = 0.0
        self._last_omega_cmd = 0.0

    def on_command(self, v_ref: float, omega_ref: float):
        self._last_v_cmd = v_ref
        self._last_omega_cmd = omega_ref

    def on_gyro(self, gyro_z_deg_s: float):
        """Se llama con cada lectura nueva del MPU6050 (gyroZ, °/s)."""
        now = time.time()
        if self._last_t is None:
            self._last_t = now
            return
        dt = now - self._last_t
        self._last_t = now
        if dt <= 0 or dt > 1.0:
            return  # descarta saltos raros (reconexión, primer paquete, etc.)

        omega_gyro = math.radians(gyro_z_deg_s)
        omega_fused = (self.gyro_weight * omega_gyro +
                       (1 - self.gyro_weight) * self._last_omega_cmd)

        # Integración diferencial: el robot solo avanza sobre su propio eje
        # longitudinal, así que la velocidad se proyecta con el rumbo. Se
        # usa el ángulo MEDIO del paso, no el inicial: integrar una curva
        # con el rumbo de entrada sesga la trayectoria hacia afuera.
        th = self.theta + omega_fused * dt / 2      # ángulo medio del paso
        self.x += self._last_v_cmd * math.cos(th) * dt
        self.y += self._last_v_cmd * math.sin(th) * dt
        self.theta += omega_fused * dt

    def reset(self):
        self.x = self.y = self.theta = 0.0

    def snapshot(self) -> dict:
        return {"x": self.x, "y": self.y, "theta_deg": math.degrees(self.theta) % 360}


# Sin scan fresco no se sabe qué hay delante ni detrás.
SCAN_VIEJO_S = 0.6
# Holgura del corredor a cada lado de la carrocería.
MARGEN_CORREDOR_M = 0.05


class LidarReactive:
    """Frenado reactivo a partir del scan 2D. No mapea, solo mide y frena.

    Mide dentro de un CORREDOR del ancho del robot, no dentro de un cono.
    Antes era un cono de ±21° y tenía los dos defectos de un cono:
      · de cerca es más estrecho que el robot: a 35 cm cubría ±13 cm, y las
        esquinas chocaban contra lo que el cono no veía;
      · de lejos se abre: en un pasillo estrecho ve las hileras de plantas
        de los lados como si estuvieran delante y frena sin motivo.
    Un corredor mira exactamente por donde va a pasar la carrocería.
    """

    def __init__(self):
        self.last_scan: list[dict] = []  # [{angle_deg, distance_m}, ...]
        self.last_scan_at = None
        self._cache = None

    def ingest_scan(self, points: list[dict]):
        self.last_scan = points
        self.last_scan_at = time.time()

    def edad_scan_s(self) -> float:
        return math.inf if self.last_scan_at is None else time.time() - self.last_scan_at

    def distancias(self, cfg: dict) -> dict:
        """Hueco libre desde cada cara de la carrocería hasta lo más cercano.

        Ejes del robot: x adelante, y a la izquierda (convención de ROS). El
        LIDAR está en el centro de base_link. None = nada a la vista en ese
        lado. Se calcula una vez por scan: el actuador lo pide a 20 Hz y el
        scan llega a 10.
        """
        clave = (self.last_scan_at, cfg["robot_largo_m"], cfg["robot_ancho_m"])
        if self._cache is not None and self._cache[0] == clave:
            return self._cache[1]

        ml = cfg["robot_largo_m"] / 2
        ma = cfg["robot_ancho_m"] / 2
        m = MARGEN_CORREDOR_M
        adelante = atras = izquierda = derecha = math.inf

        for p in self.last_scan:
            d = p["distance_m"]
            if not (0.05 < d < math.inf):
                continue
            a = math.radians(p["angle_deg"])
            x, y = d * math.cos(a), d * math.sin(a)
            if abs(x) <= ml and abs(y) <= ma:
                continue            # dentro de la carrocería: es el propio robot
            if abs(y) <= ma + m:
                if x > ml:
                    adelante = min(adelante, x - ml)
                elif x < -ml:
                    atras = min(atras, -x - ml)
            if abs(x) <= ml + m:
                if y > ma:
                    izquierda = min(izquierda, y - ma)
                elif y < -ma:
                    derecha = min(derecha, -y - ma)

        def _fin(v):
            return None if v == math.inf else round(v, 3)

        res = {"adelante": _fin(adelante), "atras": _fin(atras),
               "izquierda": _fin(izquierda), "derecha": _fin(derecha)}
        self._cache = (clave, res)
        return res

    def limitar_velocidad(self, v: float, cfg: dict) -> tuple[float, dict | None]:
        """Corta el avance (o el retroceso) cuando lo que hay en esa dirección
        está más cerca que lo que el robot necesita para detenerse.
        Devuelve (v, aviso|None).

        Es un CORTE a cero, no una reducción gradual, por dos límites del
        hardware medidos en el robot:
          · el firmware mapea cualquier velocidad pedida a PWM 200..255
            (78 % a 100 %): pedir la mitad de velocidad casi no lo frena;
          · al parar deja las ruedas sueltas y el robot desliza unos 80 cm
            (medido por el usuario el 2026-09-17).
        Así que hay que cortar con todo ese deslizamiento de antelación:
            parar si  hueco <= margen + deslizamiento

        Sin datos del LIDAR no se permite avanzar ni retroceder: con 80 cm de
        deslizamiento y sin ver, no hay velocidad segura.

        NO recorta el giro: girar sobre el propio eje no acerca el frente a
        nada, y quitarle el giro le quita justo la maniobra para zafar.
        """
        if abs(v) <= 1e-6:
            return v, None

        if self.edad_scan_s() > SCAN_VIEJO_S:
            return 0.0, {"motivo": "lidar_sin_datos"}

        lado = "adelante" if v > 0 else "atras"
        hueco = self.distancias(cfg)[lado]
        if hueco is None:
            return v, None

        umbral = cfg["obstacle_margen_m"] + cfg["obstacle_deslizamiento_m"]
        if hueco > umbral:
            return v, None
        return 0.0, {"motivo": "obstaculo", "lado": lado, "distancia_m": hueco,
                     "umbral_m": round(umbral, 2)}
