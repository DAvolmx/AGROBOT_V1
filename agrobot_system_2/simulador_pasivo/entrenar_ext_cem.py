"""CEM sobre el SENDERO CORTO DE EXTERIOR (2026-10-28, prueba de exterior 2) con PRIORIDAD ADELANTE (sin marcha atrás). [Basado en entrenar_L_cem.py]
MONTE CARLO / CEM sobre el ESCENARIO EN L (2026-10-28): busca por entropía cruzada los parámetros de manejo (velocidad, suavidad, margen, cono, firmware, duración y grados del pulso de giro)
que minimizan el costo de misión en el L real de 2,7 × 2,4 m (tramo estrecho 0,70 m, luego 0,90 m, 4 plantas). Cada generación usa semillas NUEVAS y el escenario varía solo (ancho ±, plantas ±5 cm).
Valida al final los mejores contra HOY en semillas nunca vistas. NO despliega nada.
Uso: PYTHONPATH=~/.agrobot/numba_pkg python3 entrenar_L_cem.py [--horas 2.5] [--procs 5] [--pob 16] [--final-min 20]
Salida: ~/.agrobot/aprendizaje/ext_cem_<fecha>/{historial.json,informe.json,log.txt}"""
import argparse, json, math, time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import comparar_estrategias as CE
import entrenar_misiones as EM
import entorno_calibrado as EC
import misiones_sim as MS
import pulse_drive
import sim_logica_actual as S
import entrenar_intensivo as EI
from mc_exterior_sendero import sendero

BUSCA = ["escala_ext", "suavidad_duty", "suavidad_periodo_s", "sendero_tol_cm", "avance_libre_ext", "cono_ext", "fu_adel", "fu_pivote", "fu_tras", "fu_dif_del", "fu_acel", "fu_frena", "fu_freno_ms", "fu_comp_neg", "fu_comp_pos"]
ESP = {k: EI.ESPACIO[k] for k in BUSCA}
ESP["giro_dur"] = ("u", 0.4, 2.8)
ESP["giro_grados"] = ("u", 6, 18)
CL = list(ESP)
HOY = {k: EI.HOY[k] for k in BUSCA}
HOY = {k: EI.HOY[k] for k in BUSCA}
HOY.update(giro_dur=0.4, giro_grados=12)


def a_u(p):
    u = []
    for k in CL:
        t, a, b = ESP[k]
        v = min(max(p[k], a), b)
        u.append((math.log(v) - math.log(a)) / (math.log(b) - math.log(a)) if t == "log" else (v - a) / (b - a))
    return np.array(u)


def de_u(u):
    p = {}
    for k, x in zip(CL, np.clip(u, 0, 1)):
        t, a, b = ESP[k]
        p[k] = round(float(math.exp(math.log(a) + x * (math.log(b) - math.log(a))) if t == "log" else a + x * (b - a)), 3)
    return p


def _job(a):
    p, semilla = a
    try:
        rng = np.random.default_rng(semilla)
        ancho = float(rng.choice([0.70, 0.80, 0.90, 1.0]))
        pulse_drive.MODOS["exterior"]["pulso_giro_s"] = p["giro_dur"]
        S.SUBSTEP = 0.005
        c = EI.campos(dict(EI.HOY, escala_ext=p["escala_int"] if False else p["escala_ext"], suavidad_duty=p["suavidad_duty"], suavidad_periodo_s=p["suavidad_periodo_s"], sendero_tol_cm=p["sendero_tol_cm"],
                           avance_libre_ext=p["avance_libre_ext"], cono_ext=p["cono_ext"], fu_adel=p["fu_adel"], fu_pivote=p["fu_pivote"], fu_tras=p["fu_tras"], fu_dif_del=p["fu_dif_del"],
                           fu_acel=p["fu_acel"], fu_frena=p["fu_frena"], fu_freno_ms=p["fu_freno_ms"], fu_comp_neg=p["fu_comp_neg"], fu_comp_pos=p["fu_comp_pos"]), "exterior")
        c.update(man_retroceso_cm=0.0, man_atras_pulsos=1, mx_giro_comp=0, mx_grados_pulso=float(p["giro_grados"]))
        r = S.episodio(sendero(ancho), S.Config(**c), semilla, EC.hip(semilla))
        return {"seed": semilla, "ancho": ancho, "exito": bool(r["exito"]), "ida": bool(r["exito"]), "vuelta": bool(r["exito"]), "contactos": int(r["contactos"]),
                "progreso": float(r["progreso"]), "costo": float(S.puntuar_v2(r)), "t_s": float(r["t_s"])}
    except Exception as e:                                                  # noqa: BLE001
        return {"seed": semilla, "error": repr(e), "exito": False, "ida": False, "vuelta": False, "contactos": 9, "progreso": 0.0, "costo": 40.0, "t_s": 0.0}


def evalua(pool, cands, semillas):
    res = pool.map(_job, [(p, s) for p in cands for s in semillas], chunksize=1)
    n = len(semillas)
    return [res[i * n:(i + 1) * n] for i in range(len(cands))]


def resume(rs):
    n = len(rs)
    return {"n": n, "costo": round(sum(r["costo"] for r in rs) / n, 2), "exito_pct": round(100 * sum(r["exito"] for r in rs) / n, 1), "ida_pct": round(100 * sum(r["ida"] for r in rs) / n, 1),
            "vuelta_pct": round(100 * sum(r["vuelta"] for r in rs) / n, 1), "progreso": round(sum(r["progreso"] for r in rs) / n, 3), "contactos": round(sum(r["contactos"] for r in rs) / n, 2)}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--horas", type=float, default=2.5); ap.add_argument("--procs", type=int, default=5); ap.add_argument("--pob", type=int, default=16)
    ap.add_argument("--final-min", type=float, default=20.0); ap.add_argument("--semillas-gen", type=int, default=6); ap.add_argument("--sd", type=float, default=0.25); ap.add_argument("--val", type=int, default=14)
    a = ap.parse_args()
    sal = Path.home() / ".agrobot" / "aprendizaje" / ("ext_cem_" + time.strftime("%Y%m%d_%H%M")); sal.mkdir(parents=True, exist_ok=True)
    log = open(sal / "log.txt", "a")

    def say(*x):
        s = time.strftime("%H:%M:%S ") + " ".join(str(i) for i in x); print(s, flush=True); log.write(s + "\n"); log.flush()

    fin_busca = time.time() + a.horas * 3600 - a.final_min * 60
    media, sd = a_u(HOY), np.full(len(CL), a.sd)
    rng = np.random.default_rng(7)
    hist, mejores = [], []
    with Pool(a.procs) as pool:
        g = 0
        while time.time() < fin_busca:
            g += 1
            cands = [HOY] + [de_u(np.clip(rng.normal(media, sd), 0, 1)) for _ in range(a.pob - 1)]
            sem = [5000 + g * 10 + i for i in range(a.semillas_gen)]
            t0 = time.time(); rs = evalua(pool, cands, sem)
            fit = np.array([-sum(r["costo"] for r in x) / len(x) for x in rs])
            ord_ = np.argsort(-fit); elite = ord_[:max(3, a.pob // 4)]
            U = np.array([a_u(c) for c in cands])
            media = 0.6 * U[elite].mean(0) + 0.4 * media
            sd = np.maximum(0.7 * sd + 0.3 * U[elite].std(0), 0.04)
            mejores.append((float(fit[ord_[0]]), cands[ord_[0]]))
            hist.append({"gen": g, "costo_mejor": round(-float(fit[ord_[0]]), 2), "costo_hoy": round(-float(fit[0]), 2), "costo_medio": round(-float(fit.mean()), 2), "mejor": cands[ord_[0]], "seg": round(time.time() - t0)})
            say(f"gen {g}: mejor {hist[-1]['costo_mejor']} | HOY {hist[-1]['costo_hoy']} | medio {hist[-1]['costo_medio']} ({hist[-1]['seg']} s)")
            (sal / "historial.json").write_text(json.dumps({"historial": hist, "media": de_u(media)}, indent=1, ensure_ascii=False))
        # validación final con semillas nunca vistas
        top = [de_u(media)] + [c for _, c in sorted(mejores, key=lambda x: -x[0])[:4]]
        val = [9000 + i for i in range(a.val)]
        say("validación final con", len(val), "semillas nuevas")
        rv = evalua(pool, [HOY] + top, val)
    inf = {"hoy": resume(rv[0]), "candidatos": [{"params": c, **resume(r)} for c, r in zip(top, rv[1:])], "generaciones": g}
    inf["candidatos"].sort(key=lambda c: c["costo"])
    (sal / "informe.json").write_text(json.dumps(inf, indent=1, ensure_ascii=False))
    say("HOY", inf["hoy"]); say("MEJOR", {k: v for k, v in inf["candidatos"][0].items() if k != "params"})


if __name__ == "__main__":
    main()
