"""Mapas de interior para simulación y entrenamiento: laboratorios, laberintos, pasillos con esquinas, sala con puerta y almacén.

Cada mapa es un `sim_logica_actual.Escenario` (paredes = segmentos reales vistos por el LIDAR; macetas = círculos bajo el plano del LIDAR
si no sobresalen). El robot arranca en (0,0) mirando a +x. Los laberintos son de UN solo camino (con vueltas) porque el controlador
sigue el pasillo con el LIDAR; no hace búsqueda de rutas.

`mapas()` -> {nombre: Escenario}.  `__main__` dibuja todos en `docs/mapas_interiores.png`.
"""
from __future__ import annotations

import math

import numpy as np

import sim_logica_actual as S

E = S.Escenario


def rect(x0, y0, x1, y1):
    """Contorno de un rectángulo (banco, estantería, columna) como 4 segmentos."""
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def _normal(a, b):
    tx, ty = b[0] - a[0], b[1] - a[1]
    L = math.hypot(tx, ty)
    return -ty / L, tx / L


def pasillo(centro, ancho, tapa_atras=True, tapa_fondo=1.6):
    """Paredes de un pasillo de `ancho` alrededor de la polilínea `centro` con esquinas a inglete; tapa atrás y un fondo
    de `tapa_fondo` m tras la meta (la meta queda lejos del fondo: si no, la regla de frente bloqueado se dispara)."""
    c = list(centro)
    d = c[-1][0] - c[-2][0], c[-1][1] - c[-2][1]
    L = math.hypot(*d)
    c.append((c[-1][0] + tapa_fondo * d[0] / L, c[-1][1] + tapa_fondo * d[1] / L))
    out = []
    for signo in (+1, -1):
        pts = []
        for i, p in enumerate(c):
            if i == 0:
                n = _normal(c[0], c[1])
                pts.append((p[0] + signo * ancho / 2 * n[0], p[1] + signo * ancho / 2 * n[1]))
            elif i == len(c) - 1:
                n = _normal(c[-2], c[-1])
                pts.append((p[0] + signo * ancho / 2 * n[0], p[1] + signo * ancho / 2 * n[1]))
            else:
                n1, n2 = _normal(c[i - 1], p), _normal(p, c[i + 1])
                m = (n1[0] + n2[0], n1[1] + n2[1])
                ml = math.hypot(*m)
                m = (m[0] / ml, m[1] / ml)
                k = (ancho / 2) / max(0.3, m[0] * n1[0] + m[1] * n1[1])       # inglete
                pts.append((p[0] + signo * k * m[0], p[1] + signo * k * m[1]))
        out += [(a[0], a[1], b[0], b[1]) for a, b in zip(pts, pts[1:])]
    n = _normal(c[-2], c[-1])
    out.append((c[-1][0] + ancho / 2 * n[0], c[-1][1] + ancho / 2 * n[1], c[-1][0] - ancho / 2 * n[0], c[-1][1] - ancho / 2 * n[1]))
    if tapa_atras:
        n0 = _normal(c[0], c[1])
        out.append((c[0][0] + ancho / 2 * n0[0], c[0][1] + ancho / 2 * n0[1], c[0][0] - ancho / 2 * n0[0], c[0][1] - ancho / 2 * n0[1]))
    return out


def _pasillo_esc(nombre, centro, ancho, descripcion, t_max=190.0):
    """Escenario de pasillo: la meta es el último punto de la línea central (no el fondo tapado)."""
    meta = centro[-1]
    return E(nombre, pasillo(centro, ancho), [], goal_x=meta[0], t_max=t_max, meta_xy=meta, linea_central=list(centro),
             descripcion=descripcion)


def _sala(x0, x1, y0, y1):
    return rect(x0, y0, x1, y1)


def mapas() -> dict:
    m = {}
    # ------------------------------------------------------------------ laberintos (un camino con vueltas)
    m["laberinto_serpiente"] = _pasillo_esc(
        "laberinto_serpiente", [(-0.7, 0), (1.6, 0), (1.6, 1.5), (-0.1, 1.5), (-0.1, 3.0), (1.8, 3.0)], 1.0,
        "Laberinto en serpiente: 4 esquinas de 90° (izquierda, izquierda, derecha, derecha) en un pasillo de 1,0 m.", 260.0)
    m["laberinto_estrecho"] = _pasillo_esc(
        "laberinto_estrecho", [(-0.7, 0), (1.5, 0), (1.5, -1.3), (3.0, -1.3), (3.0, 0.2)], 0.80,
        "Laberinto con pasillos de 80 cm y tres esquinas (derecha, izquierda, izquierda).", 240.0)
    m["pasillo_L"] = _pasillo_esc(
        "pasillo_L", [(-0.7, 0), (2.2, 0), (2.2, 2.2)], 1.0, "Pasillo con una esquina de 90° a la izquierda (1,0 m de ancho).", 170.0)
    m["pasillo_S"] = _pasillo_esc(
        "pasillo_S", [(-0.7, 0), (1.4, 0), (1.4, 1.2), (3.2, 1.2), (3.2, 2.4), (4.4, 2.4)], 0.95,
        "Pasillo en S de tres tramos (2 esquinas izquierda/derecha alternadas).", 230.0)
    m["pasillo_U"] = _pasillo_esc(
        "pasillo_U", [(-0.7, 0), (2.0, 0), (2.0, 1.7), (-0.2, 1.7)], 1.0, "Retorno en U de 180° (dos esquinas a la izquierda).", 200.0)
    # ------------------------------------------------------------------ laboratorio con bancos
    sala = _sala(-0.6, 6.6, -2.3, 2.3)
    bancos = rect(1.0, 0.75, 3.2, 1.35) + rect(1.0, -1.35, 3.2, -0.75) + rect(4.1, -0.20, 4.6, 2.3)
    m["laboratorio_bancos"] = E(
        "laboratorio_bancos", sala + bancos, [], goal_x=5.6, t_max=200.0, meta_xy=(5.6, 0.0),
        descripcion="Laboratorio 7,2 x 4,6 m con dos bancos laterales (pasillo de 1,5 m) y un tabique que obliga a pasar por el lado derecho.")
    mesas = []
    for x in (1.3, 3.1):
        for y in (-1.0, 1.0):
            mesas += rect(x - 0.45, y - 0.3, x + 0.45, y + 0.3)
    sillas = [(2.2, 0.2, 0.12), (2.2, -0.2, 0.12), (4.2, 0.0, 0.15), (4.8, 0.9, 0.14), (4.8, -0.9, 0.14)]
    m["laboratorio_mesas_sillas"] = E(
        "laboratorio_mesas_sillas", _sala(-0.6, 6.4, -2.0, 2.0) + mesas, sillas, goal_x=5.6, t_max=200.0, meta_xy=(5.6, 0.0),
        descripcion="Laboratorio con 4 mesas y sillas sueltas (patas visibles como círculos) entre las mesas.")
    # ------------------------------------------------------------------ sala con puerta
    muro = [(2.0, -2.2, 2.0, -0.45), (2.0, 0.45, 2.0, 2.2)]          # puerta de 90 cm
    m["sala_con_puerta"] = E(
        "sala_con_puerta", _sala(-0.6, 5.4, -2.2, 2.2) + muro, [], goal_x=4.2, t_max=170.0, meta_xy=(4.2, 0.0),
        descripcion="Dos salas unidas por una puerta de 90 cm: hay que centrarse y cruzar el vano.")
    muro2 = [(1.8, -2.2, 1.8, -0.95), (1.8, -0.25, 1.8, 2.2)]        # puerta descentrada, a la derecha del robot
    m["puerta_descentrada"] = E(
        "puerta_descentrada", _sala(-0.6, 5.0, -2.2, 2.2) + muro2, [], goal_x=4.0, t_max=190.0, meta_xy=(4.0, 0.0),
        descripcion="Puerta de 70 cm desplazada 60 cm a la derecha de la línea de salida.")
    # ------------------------------------------------------------------ almacén / invernadero cerrado con estanterías
    est = []
    for x in (1.2, 2.6):
        est += rect(x, -2.0, x + 0.4, -0.5) + rect(x, 0.5, x + 0.4, 2.0)
    est += rect(4.0, -2.0, 4.4, 1.0)
    m["almacen_estanterias"] = E(
        "almacen_estanterias", _sala(-0.6, 6.2, -2.0, 2.0) + est, [], goal_x=5.4, t_max=210.0, meta_xy=(5.4, 0.8),
        descripcion="Almacén con dos pares de estanterías (pasillo central de 1 m) y un muro que se rodea por arriba.")
    return m


def dibujar(ruta="docs/mapas_interiores.png"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from pathlib import Path
    mp = mapas()
    n = len(mp)
    cols = 3
    fig, ax = plt.subplots(math.ceil(n / cols), cols, figsize=(15, 4.2 * math.ceil(n / cols)))
    for a, (k, e) in zip(ax.flat, mp.items()):
        for (x0, y0, x1, y1) in e.paredes:
            a.plot([x0, x1], [y0, y1], "k-", lw=1.6)
        for (x, y, r) in e.macetas:
            a.add_patch(plt.Circle((x, y), r, color="tab:green", alpha=.6))
        if e.linea_central:
            a.plot(*zip(*e.linea_central), "c--", lw=.8)
        a.plot(0, 0, "go")
        if e.meta_xy:
            a.plot(*e.meta_xy, "r*", ms=12)
        a.set_aspect("equal")
        a.set_title(k, fontsize=10)
        a.grid(alpha=.2)
    for a in list(ax.flat)[n:]:
        a.axis("off")
    fig.tight_layout()
    p = Path(__file__).resolve().parents[1] / ruta
    p.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(p, dpi=110)
    print("Guardado", p)


if __name__ == "__main__":
    dibujar()
