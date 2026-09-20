"""
Política aprendida: de lo que percibe el robot a qué pulso dar.
==============================================================

Es la pieza que convierte "horas de simulación" en algo que el robot real
puede usar: una red pequeña entrenada con ejemplos de conducción buena
(recorridos ideales) y con las correcciones del operador, guardada como
`politica.pt`.

Decisiones, y por qué:

  · Entrada de 14 números, no la imagen. Lo que hay son pocos miles de
    ejemplos: con una imagen haría falta mucho más y no cabría el tiempo de
    inferencia en el lazo de control. Los 14 números son lo que el robot ya
    calcula igual (LIDAR por sectores y la salida del detector de senderos),
    así que en simulación y en el campo la entrada es LA MISMA. Sin eso, lo
    aprendido en el simulador no serviría en el robot.
  · Salida: una de las cuatro acciones de siempre (adelante, izquierda,
    derecha, atrás). Este chasis no admite una curvatura continua.
  · Red diminuta (14→64→64→4, ~5000 parámetros): decide en menos de un
    milisegundo en la CPU de la Jetson, sin tocar la GPU que usa YOLO.
  · El freno por LIDAR y la capa de seguridad SIGUEN por encima: la política
    propone, no dispone. Un modelo mal entrenado no puede estrellar el robot.

El fichero .pt se guarda con todo lo necesario para usarlo (pesos, medias y
desviaciones de normalización, y los nombres de entradas y acciones), así que
el robot no necesita el código de entrenamiento para cargarlo.
"""
import math
from pathlib import Path

RUTA_POR_DEFECTO = Path.home() / ".agrobot" / "aprendizaje" / "politica.pt"

ACCIONES = ("adelante", "izquierda", "derecha", "atras")

# Nombres de las 14 entradas, en orden. Cambiar esto invalida los modelos ya
# entrenados, así que se versiona.
VERSION_ENTRADAS = 1
ENTRADAS = (
    "lidar_adelante", "lidar_atras", "lidar_izquierda", "lidar_derecha",
    "lidar_min", "hueco_relativo",
    "sendero_visible", "sendero_lateral", "sendero_rumbo", "sendero_ancho",
    "sendero_confianza", "v_actual", "omega_actual", "desvio_ruta",
)
LIDAR_TOPE_M = 4.0


def _norm(v, tope, por_defecto=1.0):
    """None (nada a la vista) = libre del todo."""
    if v is None:
        return por_defecto
    return max(-1.5, min(1.5, float(v) / tope))


def caracteristicas(sensores: dict, desvio_ruta_m=None) -> list[float]:
    """Lo que percibe el robot -> las 14 entradas de la red.

    Se calcula IGUAL en el simulador y en el robot: `sensores` es el mismo
    dict que ya produce el backend."""
    a = _norm(sensores.get("lidar_adelante_m"), LIDAR_TOPE_M)
    at = _norm(sensores.get("lidar_atras_m"), LIDAR_TOPE_M)
    i = _norm(sensores.get("lidar_izquierda_m"), LIDAR_TOPE_M)
    d = _norm(sensores.get("lidar_derecha_m"), LIDAR_TOPE_M)
    lateral = sensores.get("sendero_lateral_m")
    rumbo = sensores.get("sendero_rumbo_deg")
    ancho = sensores.get("sendero_ancho_m")
    visible = 1.0 if lateral is not None else 0.0
    return [
        a, at, i, d,
        min(a, at, i, d),
        (i - d),                                  # hacia qué lado hay más hueco
        visible,
        0.0 if lateral is None else max(-2.0, min(2.0, lateral / 0.5)),
        0.0 if rumbo is None else max(-2.0, min(2.0, rumbo / 30.0)),
        0.0 if ancho is None else min(2.0, ancho / 0.8),
        float(sensores.get("sendero_confianza") or 0.0),
        max(-1.5, min(1.5, float(sensores.get("v_actual") or 0.0) / 0.55)),
        max(-1.5, min(1.5, float(sensores.get("omega_actual") or 0.0) / 1.9)),
        0.0 if desvio_ruta_m is None else min(3.0, float(desvio_ruta_m) / 0.5),
    ]


def crear_red(entradas=len(ENTRADAS), salidas=len(ACCIONES), oculta=64):
    import torch.nn as nn

    return nn.Sequential(
        nn.Linear(entradas, oculta), nn.ReLU(),
        nn.Linear(oculta, oculta), nn.ReLU(),
        nn.Linear(oculta, salidas),
    )


class Politica:
    """Carga un `politica.pt` y decide. Si no hay modelo, no estorba."""

    def __init__(self, ruta=RUTA_POR_DEFECTO, umbral=0.55):
        self.ruta = Path(ruta)
        # Por debajo de esta seguridad, mejor que decida la regla de siempre:
        # una política dudosa es peor que el comportamiento conocido.
        self.umbral = umbral
        self.red = None
        self.medias = self.desv = None
        self.info = {}
        self.usos = 0
        self.dudas = 0
        self.cargar()

    @property
    def disponible(self) -> bool:
        return self.red is not None

    def cargar(self) -> bool:
        if not self.ruta.exists():
            return False
        try:
            import torch
        except ImportError:
            return False
        try:
            datos = torch.load(self.ruta, map_location="cpu", weights_only=False)
            if datos.get("version_entradas") != VERSION_ENTRADAS:
                print(f"[politica] {self.ruta.name} es de otra versión de entradas; se ignora")
                return False
            self.red = crear_red(len(datos["entradas"]), len(datos["acciones"]),
                                 datos.get("oculta", 64))
            self.red.load_state_dict(datos["pesos"])
            self.red.eval()
            self.medias = torch.tensor(datos["medias"], dtype=torch.float32)
            self.desv = torch.tensor(datos["desviaciones"], dtype=torch.float32)
            self.acciones = tuple(datos["acciones"])
            self.info = {k: v for k, v in datos.items()
                         if k in ("entrenado", "ejemplos", "acierto", "escenarios")}
            return True
        except Exception as exc:
            print(f"[politica] no se pudo cargar {self.ruta}: {type(exc).__name__}: {exc}")
            self.red = None
            return False

    def decidir(self, sensores: dict, desvio_ruta_m=None):
        """(accion, seguridad) o (None, 0) si no hay modelo o duda."""
        if self.red is None:
            return None, 0.0
        import torch

        with torch.no_grad():
            x = torch.tensor([caracteristicas(sensores, desvio_ruta_m)], dtype=torch.float32)
            x = (x - self.medias) / self.desv
            p = torch.softmax(self.red(x)[0], dim=0)
            seguridad, idx = float(p.max()), int(p.argmax())
        if seguridad < self.umbral:
            self.dudas += 1
            return None, seguridad
        self.usos += 1
        return self.acciones[idx], seguridad

    def resumen(self) -> dict:
        return {"disponible": self.disponible, "ruta": str(self.ruta),
                "usos": self.usos, "dudas": self.dudas, **self.info}
