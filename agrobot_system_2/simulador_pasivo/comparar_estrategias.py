"""Compara ESTRATEGIAS de navegación en las 10 misiones de ida y vuelta (semillas nuevas, mismos parámetros de bajo nivel para todas).

Estrategias (de la literatura de navegación móvil con LIDAR 2D; la búsqueda web no estuvo disponible, así que es de conocimiento general, sin verificar en línea):
  E1 Reactiva clásica + guardián: avanza, al bloquearse gira hacia el lado más libre (bug/obstacle avoidance básico del robot de hoy).
  E2 Seguir el hueco (follow-the-gap / VFH): al bloquearse busca el hueco libre más cercano a la meta + rumbo a la meta + escape de nicho + visitar plantas.
  E3 E2 + predictor de colisión por pulso (barrido de la huella contra el scan antes de cada pulso).
  E4 Planificador global A* sobre el mapa acumulado + persecución pura + predictor (lo desconocido cuenta como libre; replanifica).
  E5 E3 + regreso por migas (enseñar y repetir: sigue al revés lo recorrido).
  E6 E4 con visitas a plantas desactivadas (solo ir y volver) para aislar cuánto cuesta escanear.
Uso: PYTHONPATH=~/.agrobot/numba_pkg python3 comparar_estrategias.py [--semillas 5]   Salida: ../docs/estrategias/{resultados.json,estrategias.png,CONCLUSION.md}"""
import argparse, glob, json, os, sys, time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import entrenar_misiones as EM
import entorno_calibrado as EC
import misiones_sim as MS
import sim_logica_actual as S

SAL = Path(__file__).resolve().parents[1] / "docs" / "estrategias"
_d = sorted(glob.glob(os.path.expanduser("~/.agrobot/aprendizaje/misiones_2026*/informe.json")))[-1]
BEST = dict(EM.EXTRA_HOY, **json.load(open(_d))["mejor"]["params"])
PRED = dict(mx_prediccion=True, mx_pred_giro_extra_m=0.04, mx_barrido=True, mx_pred_relajar_s=30.0)
HUECO = dict(mx_hueco=True, mx_meta=True, mx_anti_dither=True, mx_visitar=True)
ESTRATEGIAS = {
    "E1 reactiva clásica": dict(),
    "E2 seguir el hueco (VFH)": dict(HUECO),
    "E3 hueco + predictor": dict(HUECO, **PRED),
    "E4 planificador A* + predictor": dict(PRED, mx_planner=True, mx_visitar=True, mx_plan_inflar_m=0.22, mx_plan_mirada_m=1.0, mx_meta=True, mx_anti_dither=True),
    "E5 hueco + predictor + migas": dict(HUECO, **PRED, mx_migas=True),
    "E7 hueco + predictor suave": dict(HUECO, mx_prediccion=True, mx_pred_giro_extra_m=0.03, mx_barrido=True, mx_pred_margen_m=0.03, mx_pred_avance_m=0.22,
                                       mx_pred_atras_m=0.18, mx_pred_relajar_s=30.0),
    "E8 híbrido: hueco+predictor suave, A* al volver": dict(HUECO, mx_prediccion=True, mx_pred_giro_extra_m=0.03, mx_barrido=True, mx_pred_margen_m=0.03,
                                                         mx_pred_avance_m=0.22, mx_pred_atras_m=0.18, mx_pred_relajar_s=30.0, mx_planner=True,
                                                         mx_plan_solo_regreso=True, mx_plan_inflar_m=0.22, mx_plan_mirada_m=1.0),
    "E6 A* solo ida y vuelta (sin escanear)": dict(PRED, mx_planner=True, mx_visitar=False, mx_plan_inflar_m=0.22, mx_plan_mirada_m=1.0, mx_meta=True, mx_anti_dither=True),
}


def _job(a):
    nombre, mision, semilla = a
    try:
        ent, esc = MS.misiones(semilla)[mision]
        S.SUBSTEP = 0.005
        c = EM.campos(BEST, ent, guardian=True, mx=False)
        c.update(ESTRATEGIAS[nombre])
        t0 = time.time()
        r = S.episodio(esc, S.Config(**c), semilla, EC.hip(semilla))
        m = r["mision"]
        return {"estrategia": nombre, "mision": mision, "seed": semilla, "exito": MS.exito(r), "volvio": bool(r["llego"]), "ida": bool(r["ida_ok"]),
                "contactos": int(r["contactos"]), "cobertura": m["cobertura_escaneo"], "recall": m["recall"], "error_conteo": m["error_conteo"],
                "t_s": float(r["t_s"]), "progreso": float(r["progreso"]), "cpu_s": round(time.time() - t0, 1), "costo": MS.costo(r)}
    except Exception as e:                                                    # noqa: BLE001
        return {"estrategia": nombre, "mision": mision, "seed": semilla, "error": repr(e), "exito": False, "volvio": False, "ida": False, "contactos": 9,
                "cobertura": 0.0, "recall": 0.0, "error_conteo": 1.0, "t_s": 0.0, "progreso": 0.0, "cpu_s": 0.0, "costo": 150.0}


def resumen(rs):
    n = len(rs)
    f = lambda k: round(sum(r[k] for r in rs) / n, 3)
    return {"n": n, "exito_pct": round(100 * sum(r["exito"] for r in rs) / n, 1), "ida_pct": round(100 * sum(r["ida"] for r in rs) / n, 1),
            "vuelta_pct": round(100 * sum(r["volvio"] for r in rs) / n, 1), "sin_choque_pct": round(100 * sum(r["contactos"] == 0 for r in rs) / n, 1),
            "contactos": f("contactos"), "cobertura": f("cobertura"), "recall": f("recall"), "error_conteo": f("error_conteo"), "t_s": f("t_s"),
            "vuelta_sin_choque_pct": round(100 * sum(r["volvio"] and r["contactos"] == 0 for r in rs) / n, 1),
            "ida_sin_choque_pct": round(100 * sum(r["ida"] and r["contactos"] == 0 for r in rs) / n, 1), "progreso": f("progreso"), "costo": f("costo"), "cpu_s": f("cpu_s")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--semillas", type=int, default=5)
    ap.add_argument("--procs", type=int, default=6)
    a = ap.parse_args()
    SAL.mkdir(parents=True, exist_ok=True)
    semillas = list(range(100, 100 + a.semillas))
    nombres = list(MS.misiones(0))
    tr = [(e, m, s) for e in ESTRATEGIAS for m in nombres for s in semillas]
    t0 = time.time()
    with Pool(a.procs) as p:
        rs = p.map(_job, tr, chunksize=1)
    (SAL / "episodios.json").write_text(json.dumps(rs, default=float))
    out = {"semillas": semillas, "misiones": nombres, "estrategias": {}, "segundos": round(time.time() - t0)}
    for e in ESTRATEGIAS:
        l = [r for r in rs if r["estrategia"] == e]
        out["estrategias"][e] = {"todo": resumen(l), "por_mision": {m: resumen([r for r in l if r["mision"] == m]) for m in nombres}}
        print(f"{e:40s} {out['estrategias'][e]['todo']}", flush=True)
    (SAL / "resultados.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ks = list(ESTRATEGIAS)
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    for a_, (campo, tit) in zip(ax, (("ida_pct", "Llega a la meta (%)"), ("vuelta_pct", "Vuelve al inicio (%)"), ("contactos", "Contactos por episodio"))):
        a_.barh(range(len(ks)), [out["estrategias"][k]["todo"][campo] for k in ks], color="#3a7bd5")
        a_.set_yticks(range(len(ks)))
        a_.set_yticklabels([k.split(" ", 1)[0] for k in ks] if a_ is not ax[0] else ks, fontsize=8)
        a_.set_title(tit, fontsize=10)
        a_.invert_yaxis()
    fig.tight_layout()
    fig.savefig(SAL / "estrategias.png", dpi=130)
    # conclusión automática: vuelta% (misión de ida y vuelta) y luego menos contactos / costo
    orden = sorted(ks, key=lambda k: (-out["estrategias"][k]["todo"]["exito_pct"], -out["estrategias"][k]["todo"]["vuelta_sin_choque_pct"], -out["estrategias"][k]["todo"]["vuelta_pct"], out["estrategias"][k]["todo"]["contactos"],
                                      out["estrategias"][k]["todo"]["costo"]))
    lineas = ["# Conclusión: ¿cuál es la mejor estrategia?", "", f"{len(semillas)} semillas nuevas × {len(nombres)} misiones × {len(ks)} estrategias ({out['segundos']} s).", "",
              "| estrategia | éxito % | llega % | vuelve % | VUELVE SIN CHOQUE % | sin choque % | contactos/ep | cobertura escaneo | recall | tiempo (s) |", "|---|---|---|---|---|---|---|---|---|---|"]
    for k in orden:
        t = out["estrategias"][k]["todo"]
        lineas.append(f"| {k} | {t['exito_pct']} | {t['ida_pct']} | {t['vuelta_pct']} | {t['vuelta_sin_choque_pct']} | {t['sin_choque_pct']} | {t['contactos']} | {t['cobertura']} | {t['recall']} | {t['t_s']} |")
    mejor = orden[0]
    lineas += ["", f"**Mejor en esta medición: {mejor}.**", ""]
    (SAL / "CONCLUSION.md").write_text("\n".join(lineas))
    print("\n".join(lineas))


if __name__ == "__main__":
    main()
