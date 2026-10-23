"""ENTRENAMIENTO INTENSIVO (2026-10-23) del sistema de manejo en simulación CALIBRADA, interior (70 %) y exterior (30 %).

Busca por entropía cruzada (CEM) los parámetros de la Jetson (velocidad, troceo de pulsos, espacio libre, cono) y del firmware fusión
(rampas, giro de delanteras, fuerza de traseras, freno activo, compensación del giro a izquierda/derecha) sobre el robot calibrado con los
pulsos reales (entorno_calibrado.py). Cada generación evalúa con semillas NUEVAS (no se sobreajusta); al final valida a los mejores
contra la config de hoy y contra la de ayer en semillas nunca vistas, con y sin el peor hardware plausible.

Uso: python3 entrenar_intensivo.py [--horas 4.0] [--procs 5] [--pob 20] [--final-min 35]
Salida: ~/.agrobot/aprendizaje/intensivo_<fecha>/{estado.json,historial.json,informe.json,log.txt}   (NO despliega nada)"""
import argparse, json, math, time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import entorno_calibrado as EC
import sim_logica_actual as S

# nombre: (tipo, min, max)
ESPACIO = {
    "escala_int": ("u", 0.08, 0.35), "escala_ext": ("u", 0.25, 0.80),
    "suavidad_duty": ("u", 0.25, 1.0), "suavidad_periodo_s": ("u", 0.15, 0.50),
    "sendero_tol_cm": ("u", 4, 15), "avance_libre_int": ("u", 30, 100), "avance_libre_ext": ("u", 60, 150),
    "cono_int": ("u", 25, 60), "cono_ext": ("u", 10, 45),
    "fu_adel": ("u", 60, 100), "fu_pivote": ("u", 60, 100), "fu_tras": ("u", 100, 200), "fu_dif_del": ("u", 0, 100),
    "fu_acel": ("log", 100, 600), "fu_frena": ("log", 300, 1000), "fu_freno_ms": ("u", 0, 250),
    "fu_comp_neg": ("u", 0, 60), "fu_comp_pos": ("u", 0, 40),
}
CLAVES = list(ESPACIO)
HOY = {"escala_int": 0.13, "escala_ext": 0.5, "suavidad_duty": 0.35, "suavidad_periodo_s": 0.2, "sendero_tol_cm": 10,
       "avance_libre_int": 40, "avance_libre_ext": 100, "cono_int": 45, "cono_ext": 20, "fu_adel": 100, "fu_pivote": 100,
       "fu_tras": 150, "fu_dif_del": 100, "fu_acel": 600, "fu_frena": 1000, "fu_freno_ms": 80, "fu_comp_neg": 25, "fu_comp_pos": 0}
AYER = dict(HOY, fu_freno_ms=0, fu_comp_neg=0, fu_frena=300, fu_acel=120)           # firmware de esta mañana: sin freno ni compensación
PESO = {"interior": 0.7, "exterior": 0.3}


def a_unidad(p):
    u = []
    for k in CLAVES:
        t, a, b = ESPACIO[k]
        v = min(max(p[k], a), b)
        u.append((math.log(v) - math.log(a)) / (math.log(b) - math.log(a)) if t == "log" else (v - a) / (b - a))
    return np.array(u)


def de_unidad(u):
    p = {}
    for k, x in zip(CLAVES, np.clip(u, 0, 1)):
        t, a, b = ESPACIO[k]
        p[k] = round(float(math.exp(math.log(a) + x * (math.log(b) - math.log(a))) if t == "log" else a + x * (b - a)), 3)
    return p


def campos(p, entorno):
    """Parámetros de la búsqueda -> campos de S.Config para ese entorno."""
    i = entorno == "interior"
    return EC.config_base(entorno, escala_velocidad=p["escala_int" if i else "escala_ext"], suavidad_duty=p["suavidad_duty"],
                          suavidad_periodo_s=p["suavidad_periodo_s"], sendero_tol_cm=p["sendero_tol_cm"],
                          avance_libre_cm=p["avance_libre_int" if i else "avance_libre_ext"],
                          cono_frontal_deg=p["cono_int" if i else "cono_ext"], fu_adel=p["fu_adel"], fu_pivote=p["fu_pivote"],
                          fu_tras=p["fu_tras"], fu_dif_del=p["fu_dif_del"], fu_acel=p["fu_acel"], fu_frena=p["fu_frena"],
                          fu_freno_ms=p["fu_freno_ms"], fu_comp_neg=p["fu_comp_neg"], fu_comp_pos=p["fu_comp_pos"])


def _job(a):
    p, entorno, esc, semilla, extremo = a
    try:
        r = EC.correr(entorno, esc, campos(p, entorno), semilla, extremo)
        return {"entorno": entorno, "esc": esc, "seed": semilla, "exito": bool(r["exito"]), "contactos": int(r["contactos"]),
                "progreso": float(r["progreso"]), "t_s": float(r["t_s"]), "pico_cm": float(r.get("pico_cm", 0)),
                "atascado": bool(r["atascado"]), "puntos": r["puntos"]}
    except Exception as e:
        return {"entorno": entorno, "esc": esc, "seed": semilla, "error": repr(e), "exito": False, "contactos": 9, "progreso": 0.0,
                "t_s": 99.0, "pico_cm": 0.0, "atascado": True, "puntos": 40.0}


def plan(semillas_int, semillas_ext):
    out = []
    for e, ss in (("interior", semillas_int), ("exterior", semillas_ext)):
        for esc in EC.escenarios(e, 0):
            out += [(e, esc, s) for s in ss]
    return out


def evaluar(pool, cands, semillas_int, semillas_ext, extremo=False):
    trabajos = [(p, e, esc, s, extremo) for p in cands for (e, esc, s) in plan(semillas_int, semillas_ext)]
    res = pool.map(_job, trabajos, chunksize=2)
    n = len(trabajos) // len(cands)
    return [res[i * n:(i + 1) * n] for i in range(len(cands))]


def fitness(rs):
    f = 0.0
    for e, w in PESO.items():
        pe = [r["puntos"] for r in rs if r["entorno"] == e]
        f += w * (sum(pe) / len(pe))
    return f


def resumen(rs):
    out = {}
    for e in ("interior", "exterior", "todo"):
        l = [r for r in rs if e == "todo" or r["entorno"] == e]
        n = len(l)
        out[e] = {"n": n, "puntos": round(sum(r["puntos"] for r in l) / n, 3), "exito_pct": round(100 * sum(r["exito"] for r in l) / n, 1),
                  "contactos_por_ep": round(sum(r["contactos"] for r in l) / n, 2),
                  "sin_contacto_pct": round(100 * sum(r["contactos"] == 0 for r in l) / n, 1),
                  "progreso": round(sum(r["progreso"] for r in l) / n, 3), "t_s": round(sum(r["t_s"] for r in l) / n, 1)}
    out["por_escenario"] = {}
    for e in ("interior", "exterior"):
        for esc in EC.escenarios(e, 0):
            l = [r for r in rs if r["entorno"] == e and r["esc"] == esc]
            out["por_escenario"][f"{e}/{esc}"] = {"exito_pct": round(100 * sum(r["exito"] for r in l) / len(l), 1),
                                                  "contactos_por_ep": round(sum(r["contactos"] for r in l) / len(l), 2),
                                                  "puntos": round(sum(r["puntos"] for r in l) / len(l), 2)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--horas", type=float, default=4.0)
    ap.add_argument("--procs", type=int, default=5)
    ap.add_argument("--pob", type=int, default=20)
    ap.add_argument("--final-min", type=float, default=35.0)
    ap.add_argument("--gens-max", type=int, default=10_000)
    a = ap.parse_args()
    t0 = time.time()
    fin_busqueda = t0 + a.horas * 3600 - a.final_min * 60
    d = Path.home() / ".agrobot" / "aprendizaje" / ("intensivo_" + time.strftime("%Y%m%d_%H%M"))
    d.mkdir(parents=True, exist_ok=True)
    log = open(d / "log.txt", "a")

    def say(*x):
        s = " ".join(str(i) for i in x)
        print(s, flush=True)
        log.write(s + "\n")
        log.flush()

    mu = a_unidad(HOY)
    sd = np.full(len(CLAVES), 0.22)
    rng = np.random.default_rng(20261023)
    hist, archivo = [], []
    gen = 0
    with Pool(a.procs) as pool:
        # referencia de partida (la config de hoy en las semillas de la gen 0) para ver cuánto se mejora
        while time.time() < fin_busqueda and gen < a.gens_max:
            gen += 1
            pob = [de_unidad(mu)] + ([HOY] if gen == 1 else []) + [de_unidad(mu + sd * rng.standard_normal(len(CLAVES))) for _ in range(a.pob)]
            pob = pob[:a.pob]
            si = [100 + gen * 7 + k for k in range(3)]         # semillas nuevas cada generación (con numba un episodio dura ~1 s)
            se = [200 + gen * 7 + k for k in range(2)]
            rs = evaluar(pool, pob, si, se)
            fit = np.array([fitness(r) for r in rs])
            orden = np.argsort(fit)
            elite = orden[:max(4, a.pob // 4)]
            U = np.array([a_unidad(pob[i]) for i in elite])
            mu = 0.5 * mu + 0.5 * U.mean(0)
            sd = np.maximum(0.05, 0.6 * sd + 0.4 * U.std(0) * 1.3)
            for i in orden:
                archivo.append({"gen": gen, "fit": round(float(fit[i]), 3), "params": pob[i], "res": resumen(rs[i])["todo"]})
            idx_hoy = next((i for i, p in enumerate(pob) if p == HOY), None)
            hist.append({"gen": gen, "min_h": round((time.time() - t0) / 3600, 2), "mejor": round(float(fit[orden[0]]), 3),
                         "media": round(float(fit.mean()), 3), "elite": round(float(fit[elite].mean()), 3),
                         "hoy": None if idx_hoy is None else round(float(fit[idx_hoy]), 3), "sd_medio": round(float(sd.mean()), 3),
                         "mejor_exito": resumen(rs[orden[0]])["todo"]["exito_pct"]})
            say(f"gen {gen:3d} · {(time.time()-t0)/60:5.0f} min · mejor {fit[orden[0]]:.2f} (éxito {hist[-1]['mejor_exito']}%) · media {fit.mean():.2f}"
                f" · élite {fit[elite].mean():.2f} · hoy {hist[-1]['hoy']} · sd {sd.mean():.3f}")
            json.dump({"gen": gen, "mu": mu.tolist(), "sd": sd.tolist()}, open(d / "estado.json", "w"))
            json.dump({"historial": hist, "archivo_top": sorted(archivo, key=lambda x: x["fit"])[:60]}, open(d / "historial.json", "w"), indent=1)

        say("== VALIDACIÓN FINAL en semillas nunca vistas ==")
        top = sorted(archivo, key=lambda x: x["fit"])
        cand, vistos = [], set()
        for m in top:
            k = json.dumps(m["params"], sort_keys=True)
            if k not in vistos:
                vistos.add(k)
                cand.append(m["params"])
            if len(cand) == 4:
                break
        cand.append(de_unidad(mu))
        nombres = [f"top{i+1}" for i in range(len(cand) - 1)] + ["media_final", "HOY", "AYER"]
        todos = cand + [HOY, AYER]
        si, se = list(range(5000, 5016)), list(range(6000, 6008))
        val = evaluar(pool, todos, si, se)
        filas = [{"nombre": n, "params": p, **resumen(r)} for n, p, r in zip(nombres, todos, val)]
        for f in filas:
            say(f"  {f['nombre']:12s} todo {f['todo']}")
        validos = [f for f in filas[:len(cand)]]
        mejor = min(validos, key=lambda f: f["todo"]["puntos"])
        say("mejor validado:", mejor["nombre"], mejor["todo"])
        say("== ESTRÉS (hardware más flojo: masa, drag y agarre) ==")
        est = evaluar(pool, [mejor["params"], HOY, AYER], list(range(7000, 7012)), list(range(8000, 8006)), extremo=True)
        estres = {n: resumen(r) for n, r in zip(["mejor", "HOY", "AYER"], est)}
        for n, v in estres.items():
            say(f"  {n:6s} {v['todo']}")
    json.dump({"duracion_min": round((time.time() - t0) / 60), "generaciones": gen, "historial": hist, "validacion": filas,
               "mejor": {"nombre": mejor["nombre"], "params": mejor["params"]}, "estres": estres,
               "archivo_top": sorted(archivo, key=lambda x: x["fit"])[:60]}, open(d / "informe.json", "w"), indent=1, ensure_ascii=False)
    say("Guardado", d / "informe.json", "· total", round((time.time() - t0) / 60), "min")


if __name__ == "__main__":
    main()
