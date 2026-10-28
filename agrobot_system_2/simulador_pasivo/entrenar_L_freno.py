"""Segundo barrido del escenario en L (2026-10-28): con el giro fijo en el equivalente al de lazo cerrado (pulso de 2,0 s, 15°) se varían los frenos: cono lateral del LIDAR,
margen del predictor de colisión y hueco mínimo para avanzar. Uso: PYTHONPATH=~/.agrobot/numba_pkg python3 entrenar_L_freno.py [--semillas 4] [--procs 6]"""
import argparse, itertools, json, time
from multiprocessing import Pool

import entrenar_L as L
import entrenar_misiones as EM
import entorno_calibrado as EC
import misiones_sim as MS
import comparar_estrategias as CE
import pulse_drive
import sim_logica_actual as S


def _job(a):
    cono, margen, avance, semilla = a
    try:
        pulse_drive.MODOS["interior"]["pulso_giro_s"] = 2.0
        S.SUBSTEP = 0.005
        c = EM.campos(CE.BEST, "interior", guardian=True, mx=False)
        c.update(L.ESTRATEGIA); c.update(mx_grados_pulso=15.0, cono_lateral_deg=cono, mx_pred_margen_m=margen, avance_libre_cm=avance)
        r = S.episodio(L.escenario_L(semilla), S.Config(**c), semilla, EC.hip(semilla))
        m = r["mision"]
        return {"cono": cono, "margen": margen, "avance": avance, "seed": semilla, "ida": bool(r["ida_ok"]), "vuelta": bool(r["llego"]), "exito": bool(MS.exito(r)), "contactos": int(r["contactos"]),
                "progreso": float(r["progreso"]), "atascado": bool(r["atascado"]), "cobertura": m["cobertura_escaneo"], "recall": m["recall"], "t_s": float(r["t_s"]), "costo": float(MS.costo(r))}
    except Exception as e:                                                  # noqa: BLE001
        return {"cono": cono, "margen": margen, "avance": avance, "seed": semilla, "error": repr(e), "ida": False, "vuelta": False, "exito": False, "contactos": 9, "progreso": 0.0, "atascado": True,
                "cobertura": 0.0, "recall": 0.0, "t_s": 0.0, "costo": 150.0}


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--semillas", type=int, default=4); ap.add_argument("--procs", type=int, default=6); a = ap.parse_args()
    conos, margenes, avances = (40, 60, 75), (0.03, 0.05), (40, 60)
    trab = [(c, m, v, 300 + s) for c, m, v in itertools.product(conos, margenes, avances) for s in range(a.semillas)]
    t0 = time.time()
    with Pool(a.procs) as p:
        rs = p.map(_job, trab, chunksize=1)
    filas = []
    for c, m, v in itertools.product(conos, margenes, avances):
        l = [r for r in rs if (r["cono"], r["margen"], r["avance"]) == (c, m, v)]; n = len(l)
        filas.append({"cono_lateral_deg": c, "margen_pred_m": m, "avance_libre_cm": v, "n": n, "ida_pct": round(100 * sum(r["ida"] for r in l) / n), "exito_pct": round(100 * sum(r["exito"] for r in l) / n),
                      "contactos": round(sum(r["contactos"] for r in l) / n, 1), "progreso": round(sum(r["progreso"] for r in l) / n, 2), "atascado_pct": round(100 * sum(r["atascado"] for r in l) / n),
                      "cobertura": round(sum(r["cobertura"] for r in l) / n, 2), "t_s": round(sum(r["t_s"] for r in l) / n), "costo": round(sum(r["costo"] for r in l) / n, 1)})
    filas.sort(key=lambda f: f["costo"])
    (L.SAL / "barrido_frenos_L.json").write_text(json.dumps({"filas": filas, "episodios": rs, "segundos": round(time.time() - t0)}, indent=1, default=float, ensure_ascii=False))
    for f in filas:
        print(f)
    print("segundos", round(time.time() - t0))
