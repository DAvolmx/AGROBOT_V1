"""
Estimador de pose, evasión reactiva y costmap local.

  1. PoseEstimator — fusiona velocidad comandada + giroscopio MPU6050 en
     un filtro complementario. Dead-reckoning con skid-steer: sirve para
     el radar del dashboard, NO para navegación de precisión.

  2. LidarReactive — ingesta de un scan 2D y evasión por sectores: corta
     la velocidad cuando hay obstáculo más cerca que el umbral. Capa de
     seguridad que funciona con o sin nav2.

  3. CostmapLocal — rejilla de ocupación centrada en el robot, construida
     scan a scan (costmap_local.py). Alimenta la detección de senderos
     con un perfil lateral continuo y detecta huecos libres para navegar.

  4. MapManager — guardar/cargar la rejilla de ocupación a disco. Los
     mapas se guardan con nombre junto al agrobot_config.json y se
     restauran al reiniciar.
"""
import json
import math
import time
from pathlib import Path

from costmap_local import CostmapLocal


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
# Holgura sobre la carrocería para descartar retornos del propio robot.
HUELLA_TOLERANCIA_M = 0.02
# Margen a cada lado del camino que barre el robot (skid-steer: deriva algo).
CORREDOR_HOLGURA_M = 0.05


class LidarReactive:
    """Frenado reactivo a partir del scan 2D. No mapea, solo mide y frena.

    Mide por SECTORES angulares configurables (ver `distancias`). Historia,
    para no volver atrás: primero fue un cono fijo de ±21°, demasiado
    estrecho para cubrir el ancho del robot de cerca; luego un corredor del
    ancho de la carrocería, que en pasillos estrechos metía las paredes
    laterales dentro del sector frontal y paraba el robot sin tener nada
    delante. Ahora los ángulos los elige el operador desde el dashboard.
    """

    def __init__(self):
        self.last_scan: list[dict] = []  # [{angle_deg, distance_m}, ...]
        self.last_scan_at = None
        self._cache = None
        # Hueco delante visto por la CÁMARA (suelo_libre), m desde el centro
        # del robot o None. Cubre lo bajo que el LIDAR no ve (2026-10-02).
        self.hueco_camara = None          # callable -> float | None

    def ingest_scan(self, points: list[dict]):
        self.last_scan = points
        self.last_scan_at = time.monotonic()

    def edad_scan_s(self) -> float:
        return math.inf if self.last_scan_at is None else time.monotonic() - self.last_scan_at

    def distancias(self, cfg: dict) -> dict:
        """Hueco libre desde cada cara de la carrocería, en METROS.

        Por SECTORES angulares configurables, no por un corredor del ancho
        del robot. El corredor tenía un defecto que se notó en campo: en un
        pasillo estrecho las paredes laterales caían dentro de él (el robot
        mide 35 cm y el corredor 45), se contaban como obstáculo de FRENTE y
        el robot se paraba sin tener nada delante.

        Sectores (0° = adelante, + = izquierda, como en ROS):
          adelante  : ±`cono_frontal_deg`/2
          atras     : el espejo del frontal, alrededor de 180°
          izquierda : ±`cono_lateral_deg`/2 alrededor de +90°
          derecha   : lo mismo alrededor de -90°

        A cada distancia se le resta media carrocería, así el número es lo
        que falta para que TOQUE, no la distancia al centro del robot.
        None = nada a la vista en ese sector.
        """
        clave = (self.last_scan_at, cfg["robot_largo_cm"], cfg["robot_ancho_cm"],
                 cfg["cono_frontal_deg"], cfg["cono_lateral_deg"])
        if self._cache is not None and self._cache[0] == clave:
            return self._cache[1]

        ml = cfg["robot_largo_cm"] / 200.0      # media carrocería, en metros
        ma = cfg["robot_ancho_cm"] / 200.0
        semi_frontal = cfg["cono_frontal_deg"] / 2.0
        semi_lateral = cfg["cono_lateral_deg"] / 2.0
        adelante = atras = izquierda = derecha = math.inf

        for p in self.last_scan:
            d = p["distance_m"]
            if not (0.05 < d < math.inf):
                continue
            # Dentro de la huella del robot solo puede estar el propio robot
            # (chasis, cables, cámara): el 2026-09-19 esos reflejos a 5-15 cm
            # daban "0 cm libres" y bloqueaban adelante/atrás sin obstáculo.
            rad = math.radians(p["angle_deg"])
            lateral = abs(d * math.sin(rad))
            if (abs(d * math.cos(rad)) <= ml + HUELLA_TOLERANCIA_M
                    and lateral <= ma + HUELLA_TOLERANCIA_M):
                continue
            a = _wrap180(p["angle_deg"])
            abs_a = abs(a)
            # Adelante/atrás: solo lo que queda en el camino que barre la
            # carrocería yendo recto. Con cono de 45° una pared paralela a
            # 12 cm del costado caía en el sector frontal y bloqueaba el
            # avance sin estar en la trayectoria (2026-09-19).
            en_camino = lateral <= ma + CORREDOR_HOLGURA_M
            if abs_a <= semi_frontal:
                if en_camino:
                    adelante = min(adelante, d - ml)
            elif abs_a >= 180.0 - semi_frontal:
                if en_camino:
                    atras = min(atras, d - ml)
            elif abs(a - 90.0) <= semi_lateral:
                izquierda = min(izquierda, d - ma)
            elif abs(a + 90.0) <= semi_lateral:
                derecha = min(derecha, d - ma)

        def _fin(v):
            return None if v == math.inf else round(max(0.0, v), 3)

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
            parar si  hueco <= margen_parada_cm + deslizamiento_cm

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
        fuente = "lidar"
        if lado == "adelante" and self.hueco_camara is not None and cfg.get("freno_camara", True):
            cam = self.hueco_camara()
            if cam is not None:
                cam = max(0.0, cam - cfg["robot_largo_cm"] / 200.0)
                if hueco is None or cam < hueco:
                    hueco, fuente = cam, "camara"
        if hueco is None:
            return v, None

        umbral = (cfg["margen_parada_cm"] + cfg["deslizamiento_cm"]) / 100.0
        if hueco > umbral:
            return v, None
        return 0.0, {"motivo": "obstaculo", "lado": lado, "distancia_m": hueco,
                     "umbral_m": round(umbral, 2), "fuente": fuente}


def _wrap180(angulo_deg: float) -> float:
    """Ángulo a (-180, 180]."""
    return (angulo_deg + 180.0) % 360.0 - 180.0


MAP_DIR = Path(__file__).parent / "maps"


class MapManager:
    """Guardar y cargar mapas de ocupación local + metadatos."""

    def __init__(self, directory: Path = MAP_DIR):
        self.directory = directory
        self.directory.mkdir(exist_ok=True)

    def save(self, name: str, costmap: CostmapLocal, pose: dict | None = None) -> Path:
        import numpy as np
        path = self.directory / f"{name}.npz"
        meta = {
            "res_m": costmap.res,
            "size_m": costmap.size,
            "scans": costmap._scan_count,
            "saved_at": time.time(),
        }
        if pose:
            meta["pose"] = pose
        np.savez_compressed(path, grid=costmap.grid, meta=json.dumps(meta))
        return path

    def load(self, name: str, costmap: CostmapLocal) -> dict | None:
        import numpy as np
        path = self.directory / f"{name}.npz"
        if not path.exists():
            return None
        data = np.load(path, allow_pickle=False)
        grid = data["grid"]
        if grid.shape == (costmap.cells, costmap.cells):
            costmap.grid = grid.astype(np.float32)
        meta = json.loads(str(data["meta"]))
        costmap._scan_count = meta.get("scans", 0)
        return meta

    def list_maps(self) -> list[dict]:
        maps = []
        for p in sorted(self.directory.glob("*.npz")):
            try:
                import numpy as np
                data = np.load(p, allow_pickle=False)
                meta = json.loads(str(data["meta"]))
                meta["name"] = p.stem
                meta["size_bytes"] = p.stat().st_size
                maps.append(meta)
            except Exception:
                maps.append({"name": p.stem, "error": True})
        return maps

    def delete(self, name: str) -> bool:
        path = self.directory / f"{name}.npz"
        if path.exists():
            path.unlink()
            return True
        return False
