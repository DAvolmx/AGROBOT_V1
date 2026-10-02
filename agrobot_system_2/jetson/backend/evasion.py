"""
Evasión de obstáculos: LIDAR + cámara (suelo libre) -> a dónde ir.
==================================================================

Histograma polar al estilo VFH (Borenstein y Koren, 1991), simplificado para
un robot que se mueve a pasos medidos (`paso_robotico.py`):

  1. Se juntan los obstáculos del LIDAR (todo alrededor, a ~20 cm del suelo)
     y los del suelo por cámara (solo delante, pero ve lo BAJO: la pesa y la
     cobija del pasillo del 2026-10-02 que el LIDAR no veía).
  2. Cada obstáculo se "engorda" con media anchura del robot más un margen:
     tapa todas las direcciones por las que el robot rozaría con él.
  3. Para cada dirección candidata (cada 10°) queda el recorrido libre.
  4. Se elige la dirección libre más parecida al objetivo; si es casi de
     frente, se avanza; si no, se gira hacia ella; si no hay ninguna, se
     retrocede (o se declara atascado si tampoco hay sitio detrás).

Devuelve una DECISIÓN explicable ("giro 40° a la izquierda: delante 0,45 m,
a la izquierda 1,8 m"), no un comando de motores: quien la ejecuta mide.
"""
from __future__ import annotations

import math

PASO_DEG = 10
MARGEN_M = 0.05            # además de media anchura del robot
ALCANCE_M = 3.0
FRENTE_DEG = 12            # dirección elegida dentro de ±esto = avanzar recto
AVANCE_MAX_M = 0.30        # un paso de evasión avanza como mucho esto
PESO_GIRO = 0.012          # penaliza alejarse del objetivo (por grado)
PESO_LIBRE = 0.5           # premia el recorrido libre (por metro, hasta ALCANCE)


def _envolver(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def libre_por_direccion(puntos, media_anchura_m: float, media_largo_m: float,
                        paso_deg: int = PASO_DEG) -> dict[int, float]:
    """{dirección_deg: hueco libre (m) desde el frontal, si el robot apuntara ahí}.

    `puntos` = [{angle_deg, distance_m}] en el marco del robot (0° = adelante,
    + = izquierda). Un punto bloquea la dirección d si, avanzando por d, el
    rectángulo engordado del robot pasaría a menos de media anchura de él."""
    radio = media_anchura_m + MARGEN_M
    libre = {d: ALCANCE_M for d in range(-180, 180, paso_deg)}
    for p in puntos:
        r = p["distance_m"]
        if not (0.05 < r < ALCANCE_M + radio):
            continue
        a = p["angle_deg"]
        for d in libre:
            rel = math.radians(_envolver(a - d))
            delante = r * math.cos(rel)
            lateral = abs(r * math.sin(rel))
            if delante > 0 and lateral < radio:
                # Hueco desde el FRONTAL (como distancias()["adelante"]).
                libre[d] = min(libre[d], max(0.0, delante - media_largo_m))
    return libre


def decidir(puntos_lidar, puntos_camara=(), *, objetivo_deg: float = 0.0,
            umbral_m: float = 0.57, robot_largo_m: float = 0.40,
            robot_ancho_m: float = 0.35) -> dict:
    """Qué hacer ahora. `umbral_m` = hueco que necesita el freno para parar
    (margen + deslizamiento): una dirección con menos recorrido libre que eso
    no sirve para avanzar."""
    puntos = list(puntos_lidar) + list(puntos_camara)
    ml, ma = robot_largo_m / 2, robot_ancho_m / 2
    libre = libre_por_direccion(puntos, ma, ml)
    necesario = umbral_m + 0.05

    # ¿Cabe girando sobre su eje? La esquina barre la media diagonal. En el
    # pasillo de 65 cm del 2026-10-02 NO cabía (rozaba el poste): ahí solo se
    # puede avanzar o retroceder hasta un sitio más ancho.
    media_diagonal = math.hypot(ml, ma)
    cercano = min((p["distance_m"] for p in puntos if p["distance_m"] > 0.05), default=math.inf)
    puede_girar = cercano > media_diagonal + MARGEN_M

    candidatas = []
    for d, l in libre.items():
        if not puede_girar and abs(_envolver(d)) > FRENTE_DEG:
            continue
        if l >= necesario:
            desvio = abs(_envolver(d - objetivo_deg))
            puntaje = PESO_LIBRE * min(l, ALCANCE_M) - PESO_GIRO * desvio
            candidatas.append((puntaje, d, l))
    frente = libre[0]
    atras = libre[-180]

    if not candidatas:
        if atras >= necesario:
            return {"accion": "retroceder", "cm": round(min(AVANCE_MAX_M, atras - umbral_m) * 100),
                    "motivo": (f"sin salida delante{'' if puede_girar else ' y sin sitio para girar'}; "
                               f"detrás {atras:.2f} m"),
                    "libre": libre}
        return {"accion": "atascado", "motivo": "ninguna dirección con hueco suficiente",
                "libre": libre}

    _, mejor, l = max(candidatas)
    if abs(_envolver(mejor - 0)) <= FRENTE_DEG and frente >= necesario:
        avance = min(AVANCE_MAX_M, frente - umbral_m)
        return {"accion": "avanzar", "cm": round(avance * 100), "direccion_deg": 0,
                "motivo": f"delante libre {frente:.2f} m", "puede_girar": puede_girar, "libre": libre}
    return {"accion": "girar", "grados": mejor,
            "motivo": (f"giro {abs(mejor)}° a la {'izquierda' if mejor > 0 else 'derecha'}: "
                       f"delante {frente:.2f} m, por ahí {l:.2f} m"),
            "libre": libre}
