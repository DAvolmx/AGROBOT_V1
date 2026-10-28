"""Simula y entrena el ESCENARIO EN L real (2026-10-28): L de 2,7 × 2,4 m (aquí: tramo largo x∈[-0,5; 2,2] de 0,70 m el primer metro y 0,90 m el resto; tramo corto x∈[1,3; 2,2] hasta y=1,95),
4 plantas (1 coliflor) pegadas a las paredes, el robot sale por el extremo estrecho y debe llegar al fondo del tramo corto, escanear las plantas y volver.
Qué se entrena: el giro. En el robot real un pulso de 0,4 s gira ~2,3° y los de lazo cerrado de ~20° giran 15–22° en 2–4 s; aquí se barre (duración del pulso de giro × grados que el explorador
CREE que gira un pulso) con la estrategia E8 (hueco + predictor suave + A* al volver) y se elige el par de menor costo de misión. Uso: PYTHONPATH=~/.agrobot/numba_pkg python3 entrenar_L.py [--semillas 6] [--procs 6]"""
import argparse, itertools, json, sys, time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import comparar_estrategias as CE
import entrenar_misiones as EM
import entorno_calibrado as EC
import misiones_sim as MS
import pulse_drive
import sim_logica_actual as S

SAL = Path(__file__).resolve().parents[1] / "docs" / "escenario_L_20261028"
ESTRATEGIA = CE.ESTRATEGIAS["E8 híbrido: hueco+predictor suave, A* al volver"]


def escenario_L(semilla=0, ancho1=0.70, ancho2=0.90):
    h1, h2 = ancho1 / 2, ancho2 / 2
    p = [(-0.5, -h1), (0.5, -h1), (0.5, -h2), (2.2, -h2), (2.2, 1.95), (1.3, 1.95), (1.3, h2), (0.5, h2), (0.5, h1), (-0.5, h1), (-0.5, -h1)]
    paredes = [(a[0], a[1], b[0], b[1]) for a, b in zip(p, p[1:])]
    rng = np.random.default_rng(900 + semilla)
    base = [(1.9, -0.30), (1.45, 0.30), (1.5, 1.30), (2.05, 1.75)]                  # coliflor + 3 plantas PEGADAS a las paredes (dejan >=0,6 m libres: dos macetas escalonadas en el centro cerraban el pasillo de 0,9 m en la 1.ª versión)
    plantas = [(x + float(rng.normal(0, 0.05)), y + float(rng.normal(0, 0.04))) for x, y in base]
    centro = [(-0.5, 0.0), (1.75, 0.0), (1.75, 1.6)]
    return MS._con_plantas("L_real", paredes, plantas, [], 1.75, 300.0, "L de 2,7 × 2,4 m, pasillo de 0,70 m el primer metro y 0,90 m después, 4 plantas.", meta=centro[-1], centro=centro)


def _job(a):
    dur, grados, semilla = a
    try:
        pulse_drive.MODOS["interior"]["pulso_giro_s"] = dur            # el simulador usa pulse_drive.segmentos() REAL: así cambia lo que dura cada pulso de giro
        S.SUBSTEP = 0.005
        c = EM.campos(CE.BEST, "interior", guardian=True, mx=False)
        c.update(ESTRATEGIA); c.update(mx_grados_pulso=float(grados))
        r = S.episodio(escenario_L(semilla), S.Config(**c), semilla, EC.hip(semilla))
        m = r["mision"]
        return {"dur": dur, "grados": grados, "seed": semilla, "exito": bool(MS.exito(r)), "ida": bool(r["ida_ok"]), "vuelta": bool(r["llego"]), "contactos": int(r["contactos"]),
                "cobertura": m["cobertura_escaneo"], "recall": m["recall"], "t_s": float(r["t_s"]), "costo": float(MS.costo(r)), "progreso": float(r["progreso"]), "atascado": bool(r["atascado"])}
    except Exception as e:                                                  # noqa: BLE001
        return {"dur": dur, "grados": grados, "seed": semilla, "error": repr(e), "exito": False, "ida": False, "vuelta": False, "contactos": 9, "cobertura": 0.0, "recall": 0.0, "t_s": 0.0, "costo": 150.0}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--semillas", type=int, default=6); ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--durs", default="0.4,1.2,2.0,2.8"); ap.add_argument("--grados", default="6,12,18")
    a = ap.parse_args(); SAL.mkdir(parents=True, exist_ok=True)
    durs, grs = [float(x) for x in a.durs.split(",")], [float(x) for x in a.grados.split(",")]
    trab = [(d, g, 300 + s) for d, g in itertools.product(durs, grs) for s in range(a.semillas)]
    t0 = time.time()
    with Pool(a.procs) as p:
        rs = p.map(_job, trab, chunksize=1)
    filas = []
    for d, g in itertools.product(durs, grs):
        l = [r for r in rs if r["dur"] == d and r["grados"] == g]
        n = len(l)
        filas.append({"dur_pulso_giro_s": d, "grados_supuestos": g, "n": n, "exito_pct": round(100 * sum(r["exito"] for r in l) / n, 1), "ida_pct": round(100 * sum(r["ida"] for r in l) / n, 1),
                      "vuelta_pct": round(100 * sum(r["vuelta"] for r in l) / n, 1), "contactos": round(sum(r["contactos"] for r in l) / n, 2), "cobertura": round(sum(r["cobertura"] for r in l) / n, 2),
                      "recall": round(sum(r["recall"] for r in l) / n, 2), "progreso": round(sum(r.get("progreso", 0) for r in l) / n, 2), "atascado_pct": round(100 * sum(r.get("atascado", False) for r in l) / n), "t_s": round(sum(r["t_s"] for r in l) / n), "costo": round(sum(r["costo"] for r in l) / n, 2), "errores": sum("error" in r for r in l)})
    filas.sort(key=lambda f: f["costo"])
    (SAL / "barrido_giro_L.json").write_text(json.dumps({"filas": filas, "episodios": rs, "segundos": round(time.time() - t0)}, indent=1, default=float, ensure_ascii=False))
    print("par (duración, grados) ordenados por costo (menor = mejor); base real = (0,4 s, 6°):")
    for f in filas:
        print(f)
    print("segundos", round(time.time() - t0))


if __name__ == "__main__":
    main()
