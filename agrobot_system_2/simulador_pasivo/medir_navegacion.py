"""Mide la navegación en interiores con el firmware modelado en sim_logica_actual (suave_v4 por defecto).

Uso:  python3 medir_navegacion.py [--semillas 50-55] [--procs 6]
Compara unas cuantas configuraciones de la Jetson sobre los 4 escenarios y imprime éxito, contactos
y atascos. Es la línea base para decidir qué mejorar; NO despliega nada.
"""
import argparse
import sys
from dataclasses import asdict
from multiprocessing import Pool

import numpy as np

import sim_logica_actual as S

ESCS = ["sala_vacia", "pasillo_macetas", "eslalon_macetas", "curva_pasillo"]
SUB = 0.0025


def _job(a):
    cfgd, esc, sd = a
    S.SUBSTEP = SUB
    return S.episodio(S.escenarios_interior()[esc], S.Config(**cfgd), sd)


def configs():
    return {
        "actual (exterior .5)": S.Config(nombre="a"),
        "interior .2": S.Config(nombre="b", modo="interior", escala_velocidad=0.2),
        "preciso .3": S.Config(nombre="c", modo="preciso", escala_velocidad=0.3),
        "interior .5 av70 cono10": S.Config(nombre="d", modo="interior", escala_velocidad=0.5,
                                           avance_libre_cm=70, cono_frontal_deg=10),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--semillas", default="50-55")
    ap.add_argument("--procs", type=int, default=6)
    a = ap.parse_args()
    lo, hi = (int(x) for x in a.semillas.split("-"))
    seeds = range(lo, hi + 1)
    cf = configs()
    jobs = [(asdict(c), e, s) for c in cf.values() for e in ESCS for s in seeds]
    with Pool(a.procs) as p:
        res = p.map(_job, jobs, chunksize=2)
    k = len(ESCS) * len(seeds)
    for i, (n, c) in enumerate(cf.items()):
        rs = res[i * k:(i + 1) * k]
        por = {e: (round(100 * float(np.mean([r["exito"] for r in rs if r["escenario"] == e]))),
                   sum(r["contactos"] for r in rs if r["escenario"] == e)) for e in ESCS}
        print(f"{n:26s} éxito {100*np.mean([r['exito'] for r in rs]):3.0f}%  contactos {sum(r['contactos'] for r in rs):4d}  "
              f"atascos {sum(bool(r['atascado']) for r in rs):2d}  por escenario (éxito%,contactos) {por}", flush=True)


if __name__ == "__main__":
    main()
