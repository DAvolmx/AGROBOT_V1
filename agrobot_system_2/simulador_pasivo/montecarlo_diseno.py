#!/usr/bin/env python3
"""Monte Carlo de DISEÑO 3D: busca el chasis que es seguro a velocidad útil.

El MC de firmware (montecarlo_firmware.py) encontró un SUELO de ~7% de vuelcos
que ningún parámetro quita: los bultos grandes vuelcan por GEOMETRÍA. Este MC
ataca esa geometría. Sortea el chasis (ancho delantero, ancho trasero, batalla,
altura del CG) y busca el que, a una velocidad útil objetivo, casi no vuelca,
cambiando lo MENOS posible respecto al diseño actual —sobre todo sin ensanchar
el frente de más, porque el frente estrecho (19 cm) era lo que mejoraba los
giros (menos arrastre lateral en skid-steer)—.

    python3 montecarlo_diseno.py                 # tabla (objetivo 0.75 m/s)
    python3 montecarlo_diseno.py --v 1.0 --json   # otra velocidad, JSON

'Menos puntos = mejor'. El vuelco manda; luego, mínimo cambio de diseño.
"""
import argparse
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))

from dinamica_pasiva import Parametros
import fallos_3d as F
from montecarlo_firmware import generar_situaciones

# Diseño actual (medido / confirmado).
ACTUAL = {"front_outer": 0.19, "rear_outer": 0.37, "wheelbase": 0.37, "cg_height": 0.20}

# Límites físicos del rediseño.
#  · rear_outer <= 0.44: el sendero mide ~50 cm; a 0.37 deja 6.5 cm/lado.
#  · front_outer <= rear_outer y >= 0.19 (no tiene sentido más estrecho).
#  · cg_height en [0.10, 0.22]: 0.10 es casi pegado a las ruedas (r=0.086).
RANGOS = {
    "front_outer": (0.19, 0.37),
    "rear_outer": (0.33, 0.44),
    "wheelbase": (0.32, 0.45),
    "cg_height": (0.10, 0.22),
}

CG_FACTIBLE = 0.12          # bajar de aquí cuesta rediseñar la caja: se penaliza

# Pesos (menos es mejor).
W_VUELCO = 60.0
W_FRENTE = 6.0              # ensanchar el frente penaliza (pierde giro)
W_CG = 3.0                 # bajar el CG por debajo de lo factible penaliza
W_BATALLA = 2.0            # cambiar la batalla es trabajo mecánico


def parametros_de(d):
    return replace(Parametros(),
                   front_outer=d["front_outer"], rear_outer=d["rear_outer"],
                   wheelbase=d["wheelbase"], cg_x=d["wheelbase"] / 2,
                   cg_height=d["cg_height"])


def sortear(rng):
    d = {k: float(rng.uniform(lo, hi)) for k, (lo, hi) in RANGOS.items()}
    d["front_outer"] = min(d["front_outer"], d["rear_outer"])   # frente <= trasero
    return d


def tasa_vuelco(est, sits, v_obj):
    """Fracción de tramos que vuelcan a la velocidad objetivo."""
    vuelcos = 0
    for h, es_giro, omega_dem, pendiente in sits:
        v = v_obj * (0.5 if pendiente >= 22.0 else 1.0)
        vuelca = F.vuelca_obstaculo_energia(h, v, est=est)
        if es_giro and F.vuelca_giro_rapido(v, min(omega_dem, 4.0), est=est):
            vuelca = True
        vuelcos += vuelca
    return vuelcos / len(sits)


def max_v_segura(est, sits, umbral=0.02):
    """Mayor velocidad con tasa de vuelco <= umbral (2% por defecto)."""
    v = 0.0
    seg = 0.0
    while v <= 1.5:
        if tasa_vuelco(est, sits, v) <= umbral:
            seg = v
        v += 0.05
    return round(seg, 2)


def evaluar(d, sits, v_obj, con_v_segura=False):
    """Coste de un diseño. `v_segura` (30x el terreno) solo se calcula si se
    pide, para no gastarla en las miles de variantes que no se muestran."""
    est = F.Estabilidad(parametros_de(d))
    tv = tasa_vuelco(est, sits, v_obj)
    frente_pen = (d["front_outer"] - 0.19) / 0.18
    cg_pen = max(0.0, (CG_FACTIBLE - d["cg_height"])) / CG_FACTIBLE
    batalla_pen = abs(d["wheelbase"] - 0.37) / 0.1
    coste = (W_VUELCO * tv + W_FRENTE * frente_pen
             + W_CG * cg_pen + W_BATALLA * batalla_pen)
    r = est.resumen()
    return {"coste": coste, "vuelco_rate": tv,
            "ang_vuelco_lateral_deg": r["ang_vuelco_lateral_deg"],
            "v_segura": max_v_segura(est, sits) if con_v_segura else None}


def buscar(n=4000, sits_n=500, semilla=0, v_obj=0.75, detalle=9):
    sits = generar_situaciones(sits_n, semilla)
    rng = np.random.default_rng(semilla + 7)
    cands = [("actual", dict(ACTUAL), evaluar(ACTUAL, sits, v_obj, con_v_segura=True))]
    for i in range(n):
        d = sortear(rng)
        cands.append((f"d{i}", d, evaluar(d, sits, v_obj)))
    cands.sort(key=lambda x: x[2]["coste"])
    # v_segura solo para las que se muestran (y la actual ya la tiene).
    for cid, d, m in cands[:detalle]:
        if m["v_segura"] is None:
            m["v_segura"] = max_v_segura(F.Estabilidad(parametros_de(d)), sits)
    return cands


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=4000)
    ap.add_argument("--situaciones", type=int, default=500)
    ap.add_argument("--v", type=float, default=0.75, help="velocidad útil objetivo (m/s)")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    cands = buscar(a.n, a.situaciones, v_obj=a.v)
    actual = next(c for c in cands if c[0] == "actual")
    mejor = next(c for c in cands if c[0] != "actual")

    if a.json:
        print(json.dumps({
            "objetivo_v": a.v,
            "actual": {"diseno": actual[1], **actual[2]},
            "mejor": {"diseno": mejor[1], **mejor[2]},
            "top": [{"id": c[0], "diseno": c[1], **c[2]} for c in cands[:10]],
        }, indent=2, ensure_ascii=False))
        return

    print(f"Objetivo: seguro a {a.v} m/s en el terreno de la huerta\n")
    print(f"{'diseño':>8} {'coste':>6} {'vuelco%':>8} {'áng.vuelco':>11} "
          f"{'v segura':>9}  (frente, trasero, batalla, CG) cm")
    for cid, d, m in [actual] + cands[:8]:
        marca = " <- ACTUAL" if cid == "actual" else ""
        print(f"{cid:>8} {m['coste']:>6.2f} {m['vuelco_rate']*100:>7.1f}% "
              f"{m['ang_vuelco_lateral_deg']:>9.1f}° {m['v_segura']:>7.2f} m/s  "
              f"({d['front_outer']*100:.0f}, {d['rear_outer']*100:.0f}, "
              f"{d['wheelbase']*100:.0f}, {d['cg_height']*100:.0f}){marca}")

    d = mejor[1]
    print("\n=== DISEÑO RECOMENDADO ===")
    print(f"  ancho delantero : {d['front_outer']*100:.0f} cm "
          f"(actual 19 · +{(d['front_outer']-0.19)*100:.0f} cm)")
    print(f"  ancho trasero   : {d['rear_outer']*100:.0f} cm (actual 37)")
    print(f"  batalla         : {d['wheelbase']*100:.0f} cm (actual 37)")
    print(f"  altura del CG   : {d['cg_height']*100:.0f} cm "
          f"(actual 20 · {(d['cg_height']-0.20)*100:+.0f} cm)")
    print(f"  -> ángulo de vuelco {actual[2]['ang_vuelco_lateral_deg']:.1f}° "
          f"-> {mejor[2]['ang_vuelco_lateral_deg']:.1f}°")
    print(f"  -> vuelco a {a.v} m/s: {actual[2]['vuelco_rate']*100:.1f}% "
          f"-> {mejor[2]['vuelco_rate']*100:.1f}%")
    print(f"  -> velocidad segura (<2% vuelco): {actual[2]['v_segura']:.2f} "
          f"-> {mejor[2]['v_segura']:.2f} m/s")


if __name__ == "__main__":
    main()
