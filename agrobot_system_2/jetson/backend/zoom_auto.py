"""ZOOM AUTOMÁTICO al acercarse a una planta (2026-10-23): la cámara hace zoom sola para que la planta ocupe buena parte de la imagen
y el clasificador de plagas vea hojas, no un punto verde.

Contexto: la cámara es de foco fijo (UVC sin control de enfoque) y el zoom es DIGITAL por recorte del centro (camera.set_zoom, hasta 4×,
con la cámara a 1280×720: detalle real). «Enfocar» aquí = (1) llevar la planta al centro —lo hace la visión activa girando el robot y
con el tilt del servo— y (2) acercarla a la imagen con el zoom, midiendo la nitidez para quedarse con el mejor cuadro.

Regla (`paso`): mientras la visión activa está ENCUADRANDO/ANALIZANDO/ESCANEANDO (ya llegó a la planta) y hay una caja objetivo,
  zoom_deseado = zoom_actual · fracción_objetivo / fracción_actual    (fracción = lado mayor de la caja / lado de la imagen)
con zonas muertas para no temblar, pasos limitados (sube ×1,3, baja ×0,8 por decisión), tope `zoom_max` y la condición de que la caja
siga ENTERA dentro del recorte tras el zoom. Al salir de esos estados vuelve a 1× (SLAM, sendero y exploración esperan el campo completo).
Lógica pura: sin cámara ni reloj propio.
"""
from __future__ import annotations

ESTADOS_ZOOM = ("encuadrando", "analizando", "escaneando")   # NO al centrar ni al acercarse: el zoom llenaba la imagen a 1,7 m y el robot dejaba de avanzar (2026-10-23)
DEFECTOS = {
    "zoom_auto": True,
    "zoom_fraccion_objetivo": 0.80,     # la planta debe ocupar ~80 % del lado mayor de la imagen (validado: 0.85 > 0.72 > 0.55 en exactitud del clasificador)
    "zoom_max": 3.0,                    # tope (la cámara permite 4×, pero con 3× el recorte aún da 427×240 px reales a 1280×720)
    "zoom_banda": 0.15,                 # zona muerta: no se mueve si la fracción está a ±15 % del objetivo
    "zoom_paso_sube": 1.3,
    "zoom_paso_baja": 0.8,
    "zoom_periodo_s": 0.45,             # tiempo mínimo entre decisiones (cada cambio de zoom pide reencuadrar)
    "zoom_margen_caja": 0.08,           # fracción de imagen que debe sobrar alrededor de la caja tras el zoom
}


def fraccion(caja_xyxy, ancho: float, alto: float) -> float:
    x1, y1, x2, y2 = (float(v) for v in caja_xyxy)
    return max((x2 - x1) / ancho, (y2 - y1) / alto)


def zoom_maximo_que_cabe(caja_xyxy, ancho: float, alto: float, zoom_actual: float, margen: float) -> float:
    """Mayor zoom nuevo con el que la caja (centrada en su sitio actual respecto al CENTRO de la imagen) sigue entera en el recorte."""
    x1, y1, x2, y2 = (float(v) for v in caja_xyxy)
    ext_x = max(abs(x1 - ancho / 2), abs(x2 - ancho / 2)) / ancho          # semianchura de la caja respecto al centro, en fracción
    ext_y = max(abs(y1 - alto / 2), abs(y2 - alto / 2)) / alto
    ext = max(ext_x, ext_y, 1e-6)
    return zoom_actual * (0.5 - margen) / ext


class ZoomAuto:
    def __init__(self, cfg: dict | None = None):
        self.cfg = {**DEFECTOS, **(cfg or {})}
        self._t = -1e9

    def paso(self, estado: str, caja_xyxy, ancho: float, alto: float, zoom_actual: float, ahora: float):
        """Devuelve el zoom nuevo (float) o None si no hay que tocarlo."""
        c = self.cfg
        if not c["zoom_auto"]:
            return None
        if ahora - self._t < c["zoom_periodo_s"]:
            return None
        if estado not in ESTADOS_ZOOM or caja_xyxy is None:
            if zoom_actual > 1.02 and estado not in ESTADOS_ZOOM:            # fuera de la inspección: campo completo otra vez
                self._t = ahora
                return 1.0
            return None
        f = fraccion(caja_xyxy, ancho, alto)
        if f <= 0.01:
            return None
        deseado = zoom_actual * c["zoom_fraccion_objetivo"] / f
        deseado = min(deseado, c["zoom_max"], zoom_maximo_que_cabe(caja_xyxy, ancho, alto, zoom_actual, c["zoom_margen_caja"]))
        deseado = max(1.0, deseado)
        if abs(f / c["zoom_fraccion_objetivo"] - 1.0) <= c["zoom_banda"] and deseado >= zoom_actual * 0.9:
            return None
        nuevo = min(deseado, zoom_actual * c["zoom_paso_sube"]) if deseado > zoom_actual else max(deseado, zoom_actual * c["zoom_paso_baja"])
        if abs(nuevo - zoom_actual) < 0.05:
            return None
        self._t = ahora
        return round(nuevo, 2)


def nitidez(cv2, frame) -> float:
    """Varianza del laplaciano: mayor = más nítido (para quedarse con el mejor cuadro entre varios)."""
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gris, cv2.CV_64F).var())
