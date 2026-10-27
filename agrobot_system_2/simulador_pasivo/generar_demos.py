"""Demostraciones del «recorrido ideal» (2026-10-27): un EXPERTO con el mapa verdadero (A* con oráculo, serpenteando por el sendero, predictor de colisión como
red de seguridad) recorre las 15 misiones (10 + 5 duras) y registra (sensores → acción). Se guardan solo los episodios SIN contactos que llegan a la meta.
Uso: PYTHONPATH=~/.agrobot/numba_pkg python3 generar_demos.py [--semillas 20] [--procs 6] [--zig 0.2]   Salida: ~/.agrobot/aprendizaje/demos_il.npz"""
import argparse, time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import comparar_estrategias as CE
import entorno_calibrado as EC
import entrenar_misiones as EM
import misiones_sim as MS
import sim_logica_actual as S

EXPERTO = dict(CE.PRED, mx_planner=True, mx_visitar=False, mx_plan_inflar_m=0.22, mx_plan_mirada_m=1.0, mx_meta=True, mx_anti_dither=True, mx_oraculo=True, mx_demos=True)
SAL = Path.home() / ".agrobot" / "aprendizaje" / "demos_il.npz"


def _job(a):
    mision, semilla, zig = a
    try:
        ent, esc = MS.misiones(semilla, duras=True)[mision]
        S.SUBSTEP = 0.005
        c = EM.campos(CE.BEST, ent, guardian=True, mx=False)
        c.update(EXPERTO, mx_zigzag_m=zig)
        r = S.episodio(esc, S.Config(**c), semilla, EC.hip(semilla))
        ok = bool(r["llego"]) and int(r["contactos"]) == 0
        return mision, semilla, ok, int(r["contactos"]), bool(r["ida_ok"]), r.get("demos", [])
    except Exception as e:                                                    # noqa: BLE001
        return mision, semilla, False, 9, False, repr(e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--semillas", type=int, default=20)
    ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--zig", type=float, default=0.2)
    a = ap.parse_args()
    nombres = list(MS.misiones(0, duras=True))
    tr = [(m, 300 + s, a.zig) for m in nombres for s in range(a.semillas)]
    t0 = time.time()
    with Pool(a.procs) as p:
        rs = p.map(_job, tr, chunksize=1)
    X, Y, G = [], [], []
    por = {m: [0, 0] for m in nombres}
    for m, s, ok, cont, ida, d in rs:
        por[m][1] += 1
        if isinstance(d, str):
            print("ERROR", m, s, d)
            continue
        if ok:
            por[m][0] += 1
            for f, y in d:
                X.append(f)
                Y.append(y)
                G.append(hash((m, s)) % 100000)
    np.savez_compressed(SAL, X=np.array(X, dtype=np.float32), Y=np.array(Y, dtype=np.int64), G=np.array(G))
    print(f"{len(X)} muestras de {sum(v[0] for v in por.values())}/{len(rs)} episodios limpios ({time.time()-t0:.0f} s)")
    for m, (g, n) in por.items():
        print(f"  {m:28s} {g}/{n}")


if __name__ == "__main__":
    main()
