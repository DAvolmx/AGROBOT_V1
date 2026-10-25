"""Escenarios de MISIÓN (2026-10-24): terrenos extensos, laberintos y salas con plantas que hay que recorrer, escanear y CONTAR.

`misiones(semilla)` -> {nombre: (entorno, Escenario)}. Las plantas cambian de sitio con la semilla (campo y laboratorio) o se desplazan unos
centímetros (hileras); la verdad terreno del conteo es `Escenario.plantas`. Todas las plantas también son obstáculos redondos de 12 cm de radio.
Costo de misión (menor = mejor): ver `costo()`; éxito = llegó, sin contactos, conteo con error <= 15 % y >= 80 % de las plantas vistas.
"""
from __future__ import annotations

import math

import numpy as np

import mapas_interiores as MI
import sim_logica_actual as S

E = S.Escenario
R = 0.12


def misiones(semilla: int = 0) -> dict:
    """Las 10 misiones de la semilla, SOLO con distribuciones factibles (hay un camino libre para un robot de 25 cm de radio): las que salen
    infactibles se vuelven a sortear con otro intento determinista."""
    out, ref = {}, None
    for intento in range(60):
        d = _misiones(semilla, intento)
        ref = ref or d
        for n, (ent, esc) in d.items():
            if n not in out and factible(esc):
                out[n] = (ent, esc)
        if len(out) == len(d):
            break
    for n, v in ref.items():                       # si alguna no se logró (no debería), se deja la primera con aviso en la descripción
        if n not in out:
            v[1].descripcion += " [INFACTIBLE]"
            out[n] = v
    return {n: out[n] for n in ref}


def _con_plantas(nombre, paredes, plantas, extra_obst, goal, t_max, desc, meta=None, centro=None):
    macetas = [(x, y, R) for x, y in plantas] + list(extra_obst)
    t_max = t_max + 12.0 * len(plantas)             # escanear cada planta cuesta tiempo (parada + acercamiento): el límite crece con el número de plantas
    return E(nombre, paredes, macetas, goal_x=goal, t_max=t_max, meta_xy=meta, linea_central=centro, descripcion=desc, plantas=list(plantas))


def _sobre_camino(centro, s_m, lateral):
    """Punto a `s_m` metros a lo largo de la polilínea `centro`, desplazado `lateral` m a la izquierda."""
    resto = s_m
    for (ax, ay), (bx, by) in zip(centro, centro[1:]):
        L = math.hypot(bx - ax, by - ay)
        if resto <= L:
            u = resto / L
            nx, ny = -(by - ay) / L, (bx - ax) / L
            return (ax + (bx - ax) * u + nx * lateral, ay + (by - ay) * u + ny * lateral)
        resto -= L
    return centro[-1]


def _misiones(semilla: int = 0, intento: int = 0) -> dict:
    rng = np.random.default_rng(31_000 + semilla + 7919 * intento)
    out = {}
    # 1) huerta larga: dos hileras de 14 m (≈50 plantas), paso libre 90 cm  [exterior]
    b = 0.45 + R
    pl, x = [], 0.9
    while x <= 14.0:
        pl += [(x + rng.normal(0, 0.03), b + rng.normal(0, 0.02)), (x + rng.normal(0, 0.03), -b + rng.normal(0, 0.02))]
        x += 0.5
    out["huerta_larga"] = ("exterior", _con_plantas("huerta_larga", [], pl, [], 14.6, 520.0, "Dos hileras de 14 m (~56 plantas) con 90 cm de paso: contar en marcha."))
    # 2) campo extenso: 14 x 3 m con plantas sueltas y rocas  [exterior]
    pl = []
    while len(pl) < 14:
        c = (float(rng.uniform(1.2, 13.0)), float(rng.uniform(-1.4, 1.4)))
        if all(math.hypot(c[0] - a, c[1] - b_) > 0.8 for a, b_ in pl):
            pl.append(c)
    rocas = []
    while len(rocas) < 7:
        c = (float(rng.uniform(1.5, 13.0)), float(rng.uniform(-2.0, 2.0)), float(rng.uniform(0.12, 0.22)))
        if all(math.hypot(c[0] - a, c[1] - b_) > 0.7 for a, b_ in pl) and all(math.hypot(c[0] - a, c[1] - b_) > 0.8 for a, b_, _r in rocas):
            rocas.append(c)
    out["campo_extenso"] = ("exterior", _con_plantas("campo_extenso", [], pl, rocas, 14.0, 520.0, "Campo abierto de 14 m con 14 plantas sueltas y 7 rocas."))
    # 3) camino curvo con plantas a los lados, 14 m  [exterior]
    pl = [(x_, 0.62 + 0.25 * math.sin(x_ / 1.4) + rng.normal(0, 0.02)) for x_ in np.arange(0.9, 13.5, 0.6)]
    pl += [(x_, -0.62 + 0.25 * math.sin(x_ / 1.4) + rng.normal(0, 0.02)) for x_ in np.arange(0.9, 13.5, 0.6)]
    cen = [(float(x_), 0.25 * math.sin(x_ / 1.4)) for x_ in np.arange(-0.5, 14.6, 0.25)]
    out["camino_curvo"] = ("exterior", _con_plantas("camino_curvo", [], pl, [], 14.0, 520.0, "Camino ondulado de 14 m con plantas a los dos lados (~42).", centro=cen))
    # 4) laberinto con macetas (un camino, 3 vueltas)  [interior]
    centro = [(-0.7, 0), (3.0, 0), (3.0, 1.9), (0.0, 1.9), (0.0, 3.8), (3.2, 3.8)]
    pared = MI.pasillo(centro, 1.2)
    longitud = sum(math.hypot(b_[0] - a[0], b_[1] - a[1]) for a, b_ in zip(centro, centro[1:]))
    pl = []
    for k, s_m in enumerate(np.linspace(1.6, longitud - 1.4, 7)):
        pl.append(_sobre_camino(centro, float(s_m + rng.normal(0, 0.12)), (0.38 if k % 2 == 0 else -0.38)))
    out["laberinto_macetas"] = ("interior", _con_plantas("laberinto_macetas", pared, pl, [], centro[-1][0], 600.0,
                                                         "Laberinto de 1,2 m con 3 vueltas y 7 macetas contra las paredes.", meta=centro[-1], centro=centro))
    # 5) laboratorio con macetas dispersas y patas de mesa  [interior]
    sala = MI._sala(-0.6, 8.2, -2.3, 2.3)
    patas = []
    for x_ in (2.0, 5.0):
        for y_ in (-1.5, 1.5):
            patas += MI.patas_mesa(x_, y_, 1.4, 0.7)
    pl = []
    while len(pl) < 8:
        c = (float(rng.uniform(1.0, 7.0)), float(rng.uniform(-1.2, 1.2)))
        if all(math.hypot(c[0] - a, c[1] - b_) > 0.9 for a, b_ in pl) and all(math.hypot(c[0] - a, c[1] - b_) > 0.5 for a, b_, _r in patas):
            pl.append(c)
    out["lab_macetas"] = ("interior", _con_plantas("lab_macetas", sala, pl, patas, 7.4, 560.0, "Laboratorio de 8,8 m con 8 macetas dispersas y 16 patas de mesa.", meta=(7.4, 0.0)))
    # 6) dos salas con puerta de 90 cm y macetas en las dos  [interior]
    muro = [(3.2, -2.2, 3.2, -0.45), (3.2, 0.45, 3.2, 2.2)]
    pl = []
    while len(pl) < 6:
        c = (float(rng.uniform(0.8, 7.4)), float(rng.uniform(-1.5, 1.5)))
        if abs(c[0] - 3.2) > 0.9 and all(math.hypot(c[0] - a, c[1] - b_) > 1.0 for a, b_ in pl) and math.hypot(c[0], c[1]) > 0.9:
            pl.append(c)
    out["salas_puerta"] = ("interior", _con_plantas("salas_puerta", MI._sala(-0.6, 8.2, -2.2, 2.2) + muro, pl, [], 7.4, 560.0,
                                                    "Dos salas unidas por una puerta de 90 cm con 6 macetas.", meta=(7.4, 0.0)))
    # 7) pasillo en L con plantas a lo largo de las paredes  [interior]
    centro = [(-0.7, 0), (5.0, 0), (5.0, 4.5)]
    pared = MI.pasillo(centro, 1.4)
    pl = []
    for k, s_m in enumerate(np.arange(1.0, 10.0, 0.9)):
        pl.append(_sobre_camino(centro, float(s_m), (0.5 if k % 2 == 0 else -0.5)))
    out["pasillo_L_plantas"] = ("interior", _con_plantas("pasillo_L_plantas", pared, pl, [], centro[-1][0], 560.0,
                                                        "Pasillo de 1,4 m en L con 10 plantas alternadas a lo largo de las paredes.", meta=centro[-1], centro=centro))
    # ------------------------------------------------------------------ MISIONES DIFÍCILES (2026-10-24, segunda ronda)
    # 8) laberinto grande: 8 esquinas, pasillo de 1,1 m, 10 macetas
    centro = [(-0.7, 0), (3.6, 0), (3.6, 1.8), (0.0, 1.8), (0.0, 3.6), (3.6, 3.6), (3.6, 5.4), (0.0, 5.4), (0.0, 7.2), (3.4, 7.2)]
    pared = MI.pasillo(centro, 1.1)
    longitud = sum(math.hypot(b_[0] - a[0], b_[1] - a[1]) for a, b_ in zip(centro, centro[1:]))
    pl = [_sobre_camino(centro, float(s_m + rng.normal(0, 0.12)), (0.34 if k % 2 == 0 else -0.34)) for k, s_m in enumerate(np.linspace(1.8, longitud - 1.2, 10))]
    out["laberinto_grande"] = ("interior", _con_plantas("laberinto_grande", pared, pl, [], centro[-1][0], 900.0,
                                                        "Laberinto de 1,1 m con 8 esquinas y 10 macetas contra las paredes.", meta=centro[-1], centro=centro))
    # 9) campo denso: 20 m, 26 plantas y 12 rocas
    pl = []
    while len(pl) < 26:
        c = (float(rng.uniform(1.2, 19.0)), float(rng.uniform(-1.6, 1.6)))
        if all(math.hypot(c[0] - a, c[1] - b_) > 0.9 for a, b_ in pl):
            pl.append(c)
    rocas = []
    while len(rocas) < 12:
        c = (float(rng.uniform(1.5, 19.0)), float(rng.uniform(-2.2, 2.2)), float(rng.uniform(0.12, 0.25)))
        if all(math.hypot(c[0] - a, c[1] - b_) > 0.75 for a, b_ in pl) and all(math.hypot(c[0] - a, c[1] - b_) > 0.9 for a, b_, _r in rocas):
            rocas.append(c)
    out["campo_denso"] = ("exterior", _con_plantas("campo_denso", [], pl, rocas, 20.0, 800.0, "Campo de 20 m con 26 plantas y 12 rocas."))
    # 10) laboratorio con sillas, patas de mesa, puerta de 90 cm y 8 macetas
    sala = MI._sala(-0.6, 9.2, -2.4, 2.4)
    muro = [(4.0, -2.4, 4.0, -0.45), (4.0, 0.45, 4.0, 2.4)]
    patas, segs = [], []
    for x_ in (1.6, 6.2):
        for y_ in (-1.6, 1.6):
            patas += MI.patas_mesa(x_, y_, 1.3, 0.7)
    for cx_, cy_, rum in ((2.8, 0.7, 20.0), (5.4, -0.8, 200.0), (7.4, 0.9, 90.0)):
        p_, sg_ = MI.silla(cx_, cy_, rum)
        patas += p_
        segs += sg_
    pl = []
    while len(pl) < 8:
        c = (float(rng.uniform(0.8, 8.2)), float(rng.uniform(-1.4, 1.4)))
        if abs(c[0] - 4.0) > 0.9 and all(math.hypot(c[0] - a, c[1] - b_) > 0.9 for a, b_ in pl) and all(math.hypot(c[0] - a, c[1] - b_) > 0.55 for a, b_, _r in patas) \
                and math.hypot(c[0], c[1]) > 0.9:
            pl.append(c)
    out["lab_sillas_plantas"] = ("interior", _con_plantas("lab_sillas_plantas", sala + muro + segs, pl, patas, 8.4, 700.0,
                                                         "Laboratorio de 9,8 m: puerta de 90 cm, 16 patas de mesa, 3 sillas y 8 macetas.", meta=(8.4, 0.0)))
    return out


def factible(esc, radio=0.25, res=0.05):
    """¿Existe un camino libre de inicio a meta para un robot de `radio` m? Rasteriza paredes y círculos, los ensancha por el radio y busca
    por inundación. Las misiones infactibles no miden nada del robot (se descubrió el 2026-10-25: macetas en la esquina de un laberinto tapaban el paso)."""
    import cv2
    xs = [v for w in esc.paredes for v in (w[0], w[2])] + [0.0, (esc.meta_xy or (esc.goal_x, 0.0))[0]] + [o[0] for o in esc.macetas]
    ys = [v for w in esc.paredes for v in (w[1], w[3])] + [0.0, (esc.meta_xy or (esc.goal_x, 0.0))[1]] + [o[1] for o in esc.macetas]
    x0, y0 = min(xs) - 1.0, min(ys) - 1.0
    W, H = int((max(xs) + 1.0 - x0) / res) + 1, int((max(ys) + 1.0 - y0) / res) + 1
    occ = np.zeros((H, W), np.uint8)
    for (a, b, c, d) in esc.paredes:
        cv2.line(occ, (int((a - x0) / res), int((b - y0) / res)), (int((c - x0) / res), int((d - y0) / res)), 1, 1)
    for (a, b, r) in esc.macetas:
        cv2.circle(occ, (int((a - x0) / res), int((b - y0) / res)), max(1, int(r / res)), 1, -1)
    k = int(radio / res)
    occ = cv2.dilate(occ, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))
    libre = (1 - occ).astype(np.uint8)
    ini = (int((esc.inicio[0] - x0) / res), int((esc.inicio[1] - y0) / res))
    meta = esc.meta_xy or (esc.goal_x, esc.centro_y)
    fin = (int((meta[0] - x0) / res), int((meta[1] - y0) / res))
    if not libre[ini[1], ini[0]] or not libre[fin[1], fin[0]]:
        return False
    n, lab = cv2.connectedComponents(libre, connectivity=8)
    return bool(lab[ini[1], ini[0]] == lab[fin[1], fin[0]])


def costo(r: dict) -> float:
    m = r["mision"]
    c = 35.0 * (1 - r["progreso"]) + 12.0 * min(r["contactos"], 5) + 20.0 * min(m["error_conteo"], 1.0) + 15.0 * (1 - m["recall"])
    c += 15.0 * (1 - m.get("cobertura_escaneo", 0.0)) + 0.008 * r["t_s"] + 10.0 * bool(r["atascado"]) + 10.0 * (not r["llego"])
    return round(c, 3)


def exito(r: dict) -> bool:
    """Misión cumplida: recorrió todo el camino y VOLVIÓ al inicio (con mx_regreso), sin un solo choque, escaneó >= 90 % de las plantas que vio y
    vio >= 80 % de las plantas. (El error de conteo cuenta en el costo y se informa, pero ya no decide el éxito.)"""
    m = r["mision"]
    return bool(r["llego"] and r["contactos"] == 0 and m.get("cobertura_escaneo", 0.0) >= 0.9 and m["recall"] >= 0.8)


if __name__ == "__main__":
    import json
    import time
    import entorno_calibrado as EC
    for nom, (ent, esc) in misiones(0).items():
        t0 = time.perf_counter()
        cfg = EC.config_base(ent, mision=True, guardian=True, guardian_usar_memoria=False)
        r = S.episodio(esc, S.Config(**cfg), 0, EC.hip(0))
        r["puntos"] = 0
        print(f"{nom:20s} {time.perf_counter()-t0:5.1f}s llegó={r['llego']} cont={r['contactos']} prog={r['progreso']} t={r['t_s']} costo={costo(r)} éxito={exito(r)} {json.dumps(r['mision'])}", flush=True)
