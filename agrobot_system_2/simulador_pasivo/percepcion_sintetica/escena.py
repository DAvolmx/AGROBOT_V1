"""Generador de escenas de interior y de poses de cámara para el dataset sintético.

Una SESIÓN es una escena (disposición de macetas, obstáculos, suelo, luz) más una lista de
poses de cámara dentro de ella. El reparto entrenamiento/validación/prueba se hace POR
SESIÓN: fotogramas de la misma escena son casi idénticos y mezclarlos entre conjuntos
inflaría el resultado (justo lo que exigía el encargo original).
"""
from __future__ import annotations

import math

import numpy as np

from render import Caja, Cilindro, Escena, TIPOS_PLAGA

COL_MACETA = [(180, 95, 60), (235, 235, 230), (45, 45, 50), (70, 100, 170), (150, 150, 150), (200, 160, 90)]
COL_CAJA = [(190, 150, 100), (120, 120, 125), (40, 40, 45), (60, 90, 150), (170, 60, 60)]
COL_BORDE = [(150, 150, 150), (220, 200, 60), (40, 40, 40), (200, 200, 200)]
SUELOS = [
    dict(tipo="baldosa", color=(200, 195, 185), color2=(185, 180, 170), escala=0.4, junta=0.012),
    dict(tipo="baldosa", color=(150, 150, 155), color2=(135, 135, 140), escala=0.3, junta=0.015),
    dict(tipo="madera", color=(160, 110, 70), escala=0.12),
    dict(tipo="madera", color=(110, 80, 55), escala=0.15),
    dict(tipo="hormigon", color=(140, 140, 138)),
    dict(tipo="alfombra", color=(110, 100, 120)),
]


def _libre(x, y, objs, margen):
    for o in objs:
        if isinstance(o, Cilindro):
            if (x - o.x) ** 2 + (y - o.y) ** 2 < (max(o.r, o.copa_r) + margen) ** 2:
                return False
        else:
            c, s = math.cos(-o.ang), math.sin(-o.ang)
            lx, ly = (x - o.x) * c - (y - o.y) * s, (x - o.x) * s + (y - o.y) * c
            if abs(lx) < o.lx / 2 + margen and abs(ly) < o.ly / 2 + margen:
                return False
    return True


def escena_aleatoria(rng: np.random.Generator, tipo: str | None = None, con_plagas: bool = True) -> Escena:
    tipo = tipo or str(rng.choice(["sala", "pasillo", "vivero"], p=[0.35, 0.35, 0.30]))
    if tipo == "pasillo":
        W, Lg = float(rng.uniform(0.9, 1.7)), float(rng.uniform(4.5, 8.0))
    elif tipo == "vivero":
        W, Lg = float(rng.uniform(2.6, 4.2)), float(rng.uniform(4.0, 7.0))
    else:
        W, Lg = float(rng.uniform(3.0, 5.5)), float(rng.uniform(3.5, 7.0))
    x0, x1, y0, y1 = -0.8, Lg, -W / 2, W / 2
    paredes = [(x0, y0, x1, y0), (x0, y1, x1, y1), (x0, y0, x0, y1), (x1, y0, x1, y1)]
    if tipo == "sala" and rng.random() < 0.4:          # tabique parcial
        xp = float(rng.uniform(1.5, Lg - 1.5))
        paredes.append((xp, y0, xp, y0 + W * rng.uniform(0.3, 0.55)))
    objs: list = []
    oid = 0

    def anadir_maceta(x, y):
        nonlocal oid
        r = float(rng.uniform(0.07, 0.17))
        h = float(rng.uniform(0.12, 0.40))
        planta = rng.random() < 0.85
        cil = Cilindro(x, y, r, h, 2, COL_MACETA[int(rng.integers(len(COL_MACETA)))],
                       copa_r=r * float(rng.uniform(1.1, 1.8)) if planta else 0.0,
                       copa_h=float(rng.uniform(0.12, 0.35)) if planta else 0.0, id=oid)
        if planta and con_plagas and rng.random() < 0.40:
            for _ in range(int(rng.integers(1, 5))):
                cil.plagas.append((float(rng.uniform(0, 2 * math.pi)), float(rng.uniform(0.3, 0.95)),
                                   str(rng.choice(TIPOS_PLAGA))))
        oid += 1
        objs.append(cil)

    def probar(x, y, margen=0.22):
        return x0 + 0.3 < x < x1 - 0.3 and y0 + 0.2 < y < y1 - 0.2 and _libre(x, y, objs, margen)

    if tipo == "pasillo":                               # macetas a los lados, a veces alternadas
        lado = 1
        for x in np.arange(0.8, Lg - 0.5, float(rng.uniform(0.45, 0.9))):
            if rng.random() < 0.8:
                y = (W / 2 - float(rng.uniform(0.12, 0.28))) * (lado if rng.random() < 0.7 else -lado)
                if probar(x, y, 0.1):
                    anadir_maceta(float(x), float(y))
            lado *= -1
    elif tipo == "vivero":                              # hileras
        filas = int(rng.integers(2, 4))
        for k in range(filas):
            y = y0 + (k + 1) * W / (filas + 1) + float(rng.uniform(-0.1, 0.1))
            for x in np.arange(0.6, Lg - 0.4, float(rng.uniform(0.4, 0.7))):
                if rng.random() < 0.85 and probar(float(x), y + float(rng.uniform(-0.05, 0.05)), 0.12):
                    anadir_maceta(float(x), y)
    else:                                               # sala: macetas sueltas y grupos
        for _ in range(int(rng.integers(3, 10))):
            for _ in range(10):
                x, y = float(rng.uniform(x0 + 0.4, x1 - 0.4)), float(rng.uniform(y0 + 0.3, y1 - 0.3))
                if probar(x, y):
                    anadir_maceta(x, y)
                    break
    # obstáculos altos
    for _ in range(int(rng.integers(0, 4))):
        for _ in range(10):
            x, y = float(rng.uniform(0.6, x1 - 0.4)), float(rng.uniform(y0 + 0.25, y1 - 0.25))
            lx, ly = float(rng.uniform(0.2, 0.5)), float(rng.uniform(0.2, 0.5))
            cj = Caja(x, y, lx, ly, float(rng.uniform(0.15, 0.55)), float(rng.uniform(0, math.pi)), 4,
                      COL_CAJA[int(rng.integers(len(COL_CAJA)))], id=oid)
            if probar(x, y, max(lx, ly) / 2 + 0.1):
                oid += 1
                objs.append(cj)
                break
    # bordillos bajos: lo que el LIDAR (a 18 cm) no ve
    for _ in range(int(rng.integers(0, 3))):
        for _ in range(10):
            x, y = float(rng.uniform(0.8, x1 - 0.8)), float(rng.uniform(y0 + 0.3, y1 - 0.3))
            lg, an = float(rng.uniform(0.5, 1.8)), float(rng.uniform(0.05, 0.12))
            ang = float(rng.choice([0.0, math.pi / 2, rng.uniform(0, math.pi)]))
            cj = Caja(x, y, lg, an, float(rng.uniform(0.04, 0.08)), ang, 3,
                      COL_BORDE[int(rng.integers(len(COL_BORDE)))], id=oid)
            if probar(x, y, lg / 2 + 0.1):
                oid += 1
                objs.append(cj)
                break
    # patas de silla (4 cilindros finos)
    if rng.random() < 0.3:
        for _ in range(10):
            cx, cy = float(rng.uniform(1.0, x1 - 0.6)), float(rng.uniform(y0 + 0.4, y1 - 0.4))
            if probar(cx, cy, 0.4):
                for sx in (-0.18, 0.18):
                    for sy in (-0.18, 0.18):
                        objs.append(Cilindro(cx + sx, cy + sy, float(rng.uniform(0.012, 0.025)), 0.42, 4,
                                             (60, 45, 35), id=oid))
                        oid += 1
                break
    esc = Escena(
        paredes=paredes,
        cilindros=[o for o in objs if isinstance(o, Cilindro)],
        cajas=[o for o in objs if isinstance(o, Caja)],
        suelo=dict(SUELOS[int(rng.integers(len(SUELOS)))]),
        luz=dict(ambiente=float(rng.uniform(0.5, 0.85)),
                 dir=(float(rng.uniform(0, 360)), float(rng.uniform(35, 80))),
                 pared=tuple(int(v) for v in rng.integers(150, 235, 3)),
                 techo=tuple(int(v) for v in rng.integers(190, 240, 3))),
        nombre=tipo, limites=(x0, y0, x1, y1))
    if esc.suelo["tipo"] == "baldosa":
        base = np.array(esc.suelo["color"], float) * float(rng.uniform(0.75, 1.15))
        esc.suelo["color"] = tuple(np.clip(base, 40, 250))
        esc.suelo["color2"] = tuple(np.clip(base * float(rng.uniform(0.85, 0.97)), 40, 250))
    return esc


def _dist_adelante(esc: Escena, x, y, psi, tope=6.0):
    """Distancia (m) al primer obstáculo/pared en el plano, por delante. Aproxima cajas por círculos."""
    dx, dy = math.cos(psi), math.sin(psi)
    mejor = tope
    for (ax, ay, bx, by) in esc.paredes:
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        t = ((ax - x) * ey - (ay - y) * ex) / den
        u = ((ax - x) * dy - (ay - y) * dx) / den
        if t > 0 and 0 <= u <= 1:
            mejor = min(mejor, t)
    for o in esc.objetos():
        r = max(o.r, o.copa_r) if isinstance(o, Cilindro) else 0.5 * math.hypot(o.lx, o.ly)
        fx, fy = x - o.x, y - o.y
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - r * r)
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if t > 0:
                mejor = min(mejor, t)
    return mejor


def poses_sesion(esc: Escena, rng: np.random.Generator, n: int, prefiere_libre: float = 0.85):
    """n poses de cámara (x, y, psi, cabeceo_extra, balanceo, inclinacion) libres de objetos."""
    x0, y0, x1, y1 = esc.limites
    objs = esc.objetos()
    out = []
    intentos = 0
    sesgo_x = esc.nombre in ("pasillo", "vivero")
    while len(out) < n and intentos < n * 60:
        intentos += 1
        x = float(rng.uniform(x0 + 0.3, x1 - 0.6))
        y = float(rng.uniform(y0 + 0.2, y1 - 0.2))
        if not _libre(x, y, objs, 0.28):
            continue
        psi = float(rng.normal(0, 0.45)) if sesgo_x else float(rng.uniform(-math.pi, math.pi))
        # Un robot que avanza mira hacia espacio libre. Se prefiere (85 %) una orientación con
        # más de 0,9 m despejados; el 15 % restante se deja sin filtrar para no entrenar solo
        # con vistas cómodas (pegarse a una pared también pasa).
        if rng.random() < prefiere_libre:
            for _ in range(10):
                if _dist_adelante(esc, x, y, psi) > 0.9:
                    break
                psi = float(rng.normal(0, 0.45)) if sesgo_x else float(rng.uniform(-math.pi, math.pi))
        out.append((x, y, psi, float(rng.normal(0, 1.2)), float(rng.normal(0, 0.012)),
                    float(rng.uniform(9.0, 36.0))))
    return out
