"""Experimento de sensibilidad: ¿qué limita al robot en interiores?

Parte de la config ACTUAL y varía de una en una las palancas que importan, con el resto de
hipótesis FIJAS y controladas (a diferencia de la tanda de entrenamiento, que las aleatoriza).
Sirve para responder "¿qué cambia el resultado?", no para elegir una config.

  A) tope de duty de la Jetson (60 = real) x rampa del firmware x zona muerta del motor
  B) arranque mínimo `minp` x zona muerta, con rampa alta

Uso:  python3 que_pasaria_si.py [--procs 6]
"""
import argparse
import json
import time
from dataclasses import asdict
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import sim_logica_actual as S

SALIDA = Path.home() / ".agrobot" / "aprendizaje"
ESCS = ["sala_vacia", "pasillo_macetas", "eslalon_macetas", "curva_pasillo"]
SEMILLAS = list(range(0, 6))


def _job(args):
    cfg_d, esc, seed, mu, dead = args
    cfg = S.Config(**cfg_d)
    hip = S.Hip(mu=mu, dead=dead, bias=1.0, semilla_chasis=seed)
    r = S.episodio(S.escenarios_interior(0.60)[esc], cfg, seed, hip)
    r["puntos"] = round(S.puntuar(r), 3)
    return r


def _res(rs):
    return {"exito_pct": round(100 * float(np.mean([r["exito"] for r in rs])), 1),
            "llego_pct": round(100 * float(np.mean([r["llego"] for r in rs])), 1),
            "contactos": round(float(np.mean([r["contactos"] for r in rs])), 2),
            "t_s": round(float(np.mean([r["t_s"] for r in rs])), 1),
            "progreso": round(float(np.mean([r["progreso"] for r in rs])), 2),
            "atascado_pct": round(100 * float(np.mean([r["atascado"] for r in rs])), 1),
            "n": len(rs)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--procs", type=int, default=6)
    a = ap.parse_args()
    t0 = time.perf_counter()
    out = {"A": {}, "B": {}}
    with Pool(a.procs) as pool:
        print("A) tope de duty x rampa x zona muerta (mu 0,8)", flush=True)
        for tope in (60.0, 80.0, 100.0):
            for slew in (60.0, 300.0):
                for dead in (0.0, 25.0, 40.0):
                    cfg = S.Config(nombre=f"t{int(tope)}_s{int(slew)}", tope_duty=tope, fw_slew=slew)
                    jobs = [(asdict(cfg), e, sd, 0.8, dead) for e in ESCS for sd in SEMILLAS]
                    rs = pool.map(_job, jobs, chunksize=2)
                    out["A"][f"tope{int(tope)}_slew{int(slew)}_dead{int(dead)}"] = {
                        **_res(rs), "por_escenario": {e: _res([r for r in rs if r["escenario"] == e]) for e in ESCS}}
                    k = out["A"][f"tope{int(tope)}_slew{int(slew)}_dead{int(dead)}"]
                    print(f"   tope {tope:3.0f} slew {slew:3.0f} muerta {dead:2.0f}: exito {k['exito_pct']:5.1f}%  "
                          f"llego {k['llego_pct']:5.1f}%  t {k['t_s']:5.1f}s  contactos {k['contactos']}", flush=True)
        print("B) arranque minimo x zona muerta (tope 60, rampa 300)", flush=True)
        for minp in (0.0, 30.0, 45.0):
            for dead in (0.0, 25.0, 40.0):
                cfg = S.Config(nombre=f"minp{int(minp)}", fw_slew=300.0, fw_minp=minp)
                jobs = [(asdict(cfg), e, sd, 0.8, dead) for e in ESCS[:3] for sd in SEMILLAS]
                rs = pool.map(_job, jobs, chunksize=2)
                out["B"][f"minp{int(minp)}_dead{int(dead)}"] = _res(rs)
                k = out["B"][f"minp{int(minp)}_dead{int(dead)}"]
                print(f"   minp {minp:2.0f} muerta {dead:2.0f}: exito {k['exito_pct']:5.1f}%  llego {k['llego_pct']:5.1f}%  "
                      f"t {k['t_s']:5.1f}s  contactos {k['contactos']}", flush=True)
    out["segundos"] = round(time.perf_counter() - t0)
    SALIDA.mkdir(parents=True, exist_ok=True)
    f = SALIDA / f"interior_si_que_pasaria_{time.strftime('%Y%m%d')}.json"
    f.write_text(json.dumps(out, indent=1, ensure_ascii=False))
    print(f"Guardado en {f} ({out['segundos']} s)")


if __name__ == "__main__":
    main()
