"""CEM de EXTERIOR con MISIÓN (2026-10-29): sendero corto con coles a los lados + campo abierto con plantas sueltas, PRIORIDAD ADELANTE (sin marcha atrás) y ESCANEO de cada planta.
Extiende entrenar_ext_cem.py: ahora el costo es el de misión (MS.costo: progreso, contactos, plantas vistas, cobertura de escaneo, tiempo) y se buscan también los parámetros
de escaneo (parada, distancia de inspección, distancia de visita). Cada generación usa semillas NUEVAS; al final valida HOY contra los mejores en semillas nunca vistas. NO despliega nada.
Uso: PYTHONPATH=~/.agrobot/numba_pkg python3 entrenar_ext_mision_cem.py [--horas 8] [--procs 5] [--pob 16] [--final-min 30]
Salida: ~/.agrobot/aprendizaje/ext_mision_<fecha>/{historial.json,informe.json,log.txt}"""
import argparse, json, math, time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import entorno_calibrado as EC
import misiones_sim as MS
import pulse_drive
import sim_logica_actual as S
import entrenar_intensivo as EI

BUSCA = ["escala_ext", "suavidad_duty", "suavidad_periodo_s", "sendero_tol_cm", "avance_libre_ext", "cono_ext", "fu_adel", "fu_pivote", "fu_tras", "fu_dif_del", "fu_acel", "fu_frena", "fu_freno_ms", "fu_comp_neg", "fu_comp_pos"]
ESP = {k: EI.ESPACIO[k] for k in BUSCA}
ESP["giro_dur"] = ("u", 0.4, 2.8)
ESP["giro_grados"] = ("u", 6, 18)
ESP["mis_escaneo_s"] = ("u", 3.0, 12.0)
ESP["mis_dist_insp_m"] = ("u", 0.35, 0.7)
ESP["mis_dist_visita_m"] = ("u", 0.6, 1.1)
ESP["mx_visitar_m"] = ("u", 2.0, 4.0)
CL = list(ESP)
HOY = {k: EI.HOY[k] for k in BUSCA}
HOY.update(giro_dur=0.4, giro_grados=12, mis_escaneo_s=8.0, mis_dist_insp_m=0.5, mis_dist_visita_m=0.9, mx_visitar_m=3.5)
ESCENARIOS = ["sendero_plantas", "campo_plantas"]


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


def escenario(nombre, semilla):
    rng = np.random.default_rng(semilla)
    if nombre == "sendero_plantas":                    # sendero de 0,70-1,0 m, 3,2 m, coles a ambos lados (las que hay que escanear)
        ancho = float(rng.choice([0.70, 0.80, 0.90, 1.0]))
        obst = EC._hileras(ancho, 3.2, 0.55, x_ini=0.6)
        pl = [(x + float(rng.normal(0, 0.03)), y) for x, y, _r in obst]
        return MS._con_plantas(f"sendero_{ancho:.2f}", [], pl, [], 4.0, 140.0, "Sendero corto con coles a los lados.")
    pl = []                                            # campo abierto 5 x 3 m con 6 plantas sueltas y 3 rocas
    while len(pl) < 6:
        c = (float(rng.uniform(1.0, 4.6)), float(rng.uniform(-1.2, 1.2)))
        if all(math.hypot(c[0] - a, c[1] - b) > 0.8 for a, b in pl):
            pl.append(c)
    rocas = []
    while len(rocas) < 3:
        c = (float(rng.uniform(1.2, 4.6)), float(rng.uniform(-1.3, 1.3)), float(rng.uniform(0.12, 0.2)))
        if all(math.hypot(c[0] - a, c[1] - b) > 0.55 + c[2] for a, b in pl) and all(math.hypot(c[0] - a, c[1] - b) > 0.6 for a, b, _r in rocas):
            rocas.append(c)
    return MS._con_plantas("campo_plantas", [], pl, rocas, 5.2, 150.0, "Campo abierto con plantas sueltas y rocas.")


def _job(a):
    p, nombre, semilla = a
    try:
        pulse_drive.MODOS["exterior"]["pulso_giro_s"] = p["giro_dur"]
        S.SUBSTEP = 0.005
        c = EI.campos(dict(EI.HOY, **{k: p[k] for k in BUSCA}), "exterior")
        c.update(man_retroceso_cm=0.0, man_atras_pulsos=1, mx_giro_comp=0, mx_grados_pulso=float(p["giro_grados"]),
                 mision=True, mx_visitar=True, mx_visitar_m=p["mx_visitar_m"], mis_dist_visita_m=p["mis_dist_visita_m"], mis_dist_insp_m=p["mis_dist_insp_m"], mis_escaneo_s=p["mis_escaneo_s"])
        r = S.episodio(escenario(nombre, semilla), S.Config(**c), semilla, EC.hip(semilla))
        m = r["mision"]
        return {"esc": nombre, "seed": semilla, "exito": bool(MS.exito(r)), "contactos": int(r["contactos"]), "progreso": float(r["progreso"]), "costo": float(MS.costo(r)),
                "cobertura": float(m.get("cobertura_escaneo", 0.0)), "recall": float(m["recall"]), "t_s": float(r["t_s"])}
    except Exception as e:                                                  # noqa: BLE001
        return {"esc": nombre, "seed": semilla, "error": repr(e), "exito": False, "contactos": 9, "progreso": 0.0, "costo": 150.0, "cobertura": 0.0, "recall": 0.0, "t_s": 0.0}


def evalua(pool, cands, semillas):
    tr = [(p, n, s) for p in cands for n in ESCENARIOS for s in semillas]
    res = pool.map(_job, tr, chunksize=1)
    n = len(ESCENARIOS) * len(semillas)
    return [res[i * n:(i + 1) * n] for i in range(len(cands))]


def resume(rs):
    n = len(rs)
    out = {"n": n, "costo": round(sum(r["costo"] for r in rs) / n, 2), "exito_pct": round(100 * sum(r["exito"] for r in rs) / n, 1), "progreso": round(sum(r["progreso"] for r in rs) / n, 3),
           "contactos": round(sum(r["contactos"] for r in rs) / n, 2), "cobertura": round(sum(r["cobertura"] for r in rs) / n, 3), "recall": round(sum(r["recall"] for r in rs) / n, 3),
           "t_s": round(sum(r["t_s"] for r in rs) / n), "errores": sum("error" in r for r in rs)}
    out["por_escenario"] = {e: {"exito_pct": round(100 * sum(r["exito"] for r in l) / len(l), 1), "costo": round(sum(r["costo"] for r in l) / len(l), 2), "cobertura": round(sum(r["cobertura"] for r in l) / len(l), 3)}
                            for e in ESCENARIOS for l in [[r for r in rs if r["esc"] == e]] if l}
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--horas", type=float, default=8.0); ap.add_argument("--procs", type=int, default=5); ap.add_argument("--pob", type=int, default=16)
    ap.add_argument("--final-min", type=float, default=30.0); ap.add_argument("--semillas-gen", type=int, default=4); ap.add_argument("--sd", type=float, default=0.25); ap.add_argument("--val", type=int, default=12)
    a = ap.parse_args()
    sal = Path.home() / ".agrobot" / "aprendizaje" / ("ext_mision_" + time.strftime("%Y%m%d_%H%M")); sal.mkdir(parents=True, exist_ok=True)
    log = open(sal / "log.txt", "a")

    def say(*x):
        s = time.strftime("%H:%M:%S ") + " ".join(str(i) for i in x); print(s, flush=True); log.write(s + "\n"); log.flush()

    fin_busca = time.time() + a.horas * 3600 - a.final_min * 60
    media, sd = a_u(HOY), np.full(len(CL), a.sd)
    rng = np.random.default_rng(11)
    hist, mejores = [], []
    with Pool(a.procs) as pool:
        g = 0
        while time.time() < fin_busca:
            g += 1
            cands = [HOY] + [de_u(np.clip(rng.normal(media, sd), 0, 1)) for _ in range(a.pob - 1)]
            sem = [6000 + g * 10 + i for i in range(a.semillas_gen)]
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
        top = [de_u(media)] + [c for _, c in sorted(mejores, key=lambda x: -x[0])[:4]]
        val = [9500 + i for i in range(a.val)]
        say("validación final con", len(val), "semillas nuevas x", len(ESCENARIOS), "escenarios")
        rv = evalua(pool, [HOY] + top, val)
    inf = {"hoy": resume(rv[0]), "candidatos": [{"params": c, **resume(r)} for c, r in zip(top, rv[1:])], "generaciones": g}
    inf["candidatos"].sort(key=lambda c: c["costo"])
    (sal / "informe.json").write_text(json.dumps(inf, indent=1, ensure_ascii=False))
    say("HOY", inf["hoy"]); say("MEJOR", {k: v for k, v in inf["candidatos"][0].items() if k != "params"})


if __name__ == "__main__":
    main()
