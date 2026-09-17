"""
Seguir senderos: centrado entre hileras (LIDAR) + sendero por color (cámara).
============================================================================

Dos medidas independientes del MISMO error —cuánto se ha desviado el robot
del centro del camino— que se combinan con un peso configurable:

  1. LIDAR. Si hay hilera o pared a los dos lados, el robot va centrado
     cuando la distancia izquierda y la derecha son iguales. El error es su
     diferencia, en metros, positivo si sobra sitio a la izquierda (hay que
     corregir hacia allá). Funciona de noche y con polvo, pero necesita algo
     a ambos lados: en campo abierto no dice nada.

  2. Cámara. En un cultivo el sendero es tierra y los lados son verdes. Se
     marca lo verde en HSV, se toma lo NO verde de la franja baja de la
     imagen (lo que el robot tiene justo delante) y su centro indica hacia
     dónde se va el camino. Error normalizado: -1 = el sendero está pegado
     al borde izquierdo de la imagen, +1 al derecho. Sirve donde no hay
     paredes, pero depende de la luz y de que el suelo no sea verde.

La corrección resultante es de tipo TODO-O-NADA (girar a un lado o avanzar),
no proporcional: los motores de este robot no bajan del 78 % de potencia
(PWM 200..255), así que "gira un poquito" no existe — se gira un pulso corto
y se vuelve a medir.
"""
import math

# Franja de la imagen que se analiza: la mitad baja es lo que el robot tiene
# delante. Más arriba es horizonte y mete cielo y plantas lejanas.
FRANJA_DESDE = 0.55
ANCHO_ANALISIS, ALTO_ANALISIS = 160, 120
# Si menos de esta fracción de la franja es "no verde", no hay sendero que
# seguir (todo vegetación, o de noche): la cámara se abstiene.
COBERTURA_MINIMA = 0.12


def error_lidar(distancias: dict, cfg: dict) -> float | None:
    """Desvío del centro entre hileras, en metros. None = no hay dos lados.

    Positivo: sobra sitio a la izquierda, hay que corregir a la izquierda.
    """
    izq, der = distancias.get("izquierda"), distancias.get("derecha")
    if izq is None or der is None:
        return None
    limite = cfg["sendero_pared_max_cm"] / 100.0
    if izq > limite or der > limite:
        return None            # un lado está lejísimos: no es un pasillo
    return izq - der


class VisionSendero:
    """Centro del sendero por color. Se le pasan fotogramas BGR."""

    def __init__(self):
        self.offset = None      # -1..1, o None si no hay sendero claro
        self.cobertura = 0.0
        self.estado = "sin datos"

    def analizar(self, cv2, frame, cfg) -> float | None:
        import numpy as np

        chico = cv2.resize(frame, (ANCHO_ANALISIS, ALTO_ANALISIS),
                           interpolation=cv2.INTER_AREA)
        franja = chico[int(ALTO_ANALISIS * FRANJA_DESDE):, :]
        hsv = cv2.cvtColor(franja, cv2.COLOR_BGR2HSV)

        verde = cv2.inRange(
            hsv,
            (cfg["sendero_verde_h_min"], cfg["sendero_verde_s_min"], 40),
            (cfg["sendero_verde_h_max"], 255, 255))
        # El sendero es lo que NO es vegetación.
        camino = cv2.bitwise_not(verde)
        # Cierra agujeros (piedras, hojas sueltas) para no partir el camino.
        camino = cv2.morphologyEx(camino, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        self.cobertura = float(camino.mean()) / 255.0
        if self.cobertura < COBERTURA_MINIMA:
            self.offset, self.estado = None, "sin sendero a la vista"
            return None
        if self.cobertura > 0.95:
            # Todo es "no verde": suelo desnudo sin hileras, o de noche. No
            # hay borde que seguir y el centroide sería ruido.
            self.offset, self.estado = None, "sin hileras que delimiten"
            return None

        columnas = camino.sum(axis=0).astype("float32")
        total = columnas.sum()
        if total <= 0:
            self.offset, self.estado = None, "sin sendero a la vista"
            return None
        centro = float((columnas * np.arange(len(columnas))).sum() / total)
        self.offset = (centro - (ANCHO_ANALISIS - 1) / 2) / ((ANCHO_ANALISIS - 1) / 2)
        self.estado = f"sendero visible ({self.cobertura*100:.0f} % de la franja)"
        return self.offset


class SeguidorSendero:
    """Junta las dos medidas y dice qué hacer: avanzar o corregir el rumbo."""

    def __init__(self, get_config, obtener_frame):
        self._get_config = get_config
        self._obtener_frame = obtener_frame
        self.vision = VisionSendero()
        self.error_lidar = None
        self.error_vision = None
        self.estado = "inactivo"

    def correccion(self, cv2, distancias: dict) -> str | None:
        """Devuelve "izquierda", "derecha" o None (seguir de frente)."""
        cfg = self._get_config()
        if not cfg["sendero_activo"]:
            self.estado = "inactivo"
            self.error_lidar = self.error_vision = None
            return None

        self.error_lidar = error_lidar(distancias, cfg)

        self.error_vision = None
        if cfg["sendero_vision"]:
            dato = self._obtener_frame()
            if dato is not None:
                try:
                    self.error_vision = self.vision.analizar(cv2, dato[0], cfg)
                except Exception as exc:
                    self.vision.estado = f"error: {type(exc).__name__}"

        # Tolerancia en metros para el LIDAR; la cámara trabaja normalizada,
        # así que su tolerancia se expresa como fracción del ancho.
        tol_m = cfg["sendero_tolerancia_cm"] / 100.0
        peso = cfg["sendero_vision_peso"]

        terminos = []
        if self.error_lidar is not None:
            terminos.append(((1.0 - peso), self.error_lidar / max(tol_m, 1e-3)))
        if self.error_vision is not None:
            # El sendero desplazado a la DERECHA de la imagen significa que el
            # robot se fue a la izquierda: el error cambia de signo respecto
            # del criterio del LIDAR.
            terminos.append((peso, -self.error_vision / 0.25))

        if not terminos:
            self.estado = "sin referencia (ni hileras ni sendero visible)"
            return None

        total_peso = sum(w for w, _ in terminos) or 1.0
        error = sum(w * e for w, e in terminos) / total_peso   # 1.0 = una tolerancia

        if abs(error) < 1.0:
            self.estado = f"centrado (error {error:+.2f})"
            return None
        self.estado = f"corrigiendo a la {'izquierda' if error > 0 else 'derecha'} (error {error:+.2f})"
        return "izquierda" if error > 0 else "derecha"

    def snapshot(self) -> dict:
        return {
            "estado": self.estado,
            "error_lidar_cm": None if self.error_lidar is None else round(self.error_lidar * 100),
            "error_vision": None if self.error_vision is None else round(self.error_vision, 2),
            "vision": self.vision.estado,
        }
