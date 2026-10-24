"""ENTRENAMIENTO POR MISIONES (2026-10-24): CEM sobre 7 misiones largas (terrenos extensos, laberintos, laboratorio y salas con plantas) donde el
robot debe llegar a la meta SIN contactos, ESCANEAR las plantas y CONTARLAS, conduciendo con el guardián del sendero y las maniobras de esquina.

Busca (entropía cruzada) los parámetros de manejo (entrenar_intensivo.ESPACIO: velocidad, troceo, cono, firmware fusión, maniobras man_*) más los
de misión (distancia y duración del escaneo) y los del guardián (zona muerta). Costo por episodio: misiones_sim.costo(); éxito: misiones_sim.exito().
Cada generación usa semillas nuevas (las plantas cambian de sitio). Al final valida a los mejores contra HOY y contra el robot sin guardián/maniobras
en semillas nunca vistas, con y sin hardware flojo. NO despliega nada.

Uso: PYTHONPATH=~/.agrobot/numba_pkg python3 entrenar_misiones.py [--horas 5] [--procs 4] [--pob 16] [--final-min 40]
Salida: ~/.agrobot/aprendizaje/misiones_<fecha>/{log.txt,historial.json,informe.json}"""
import argparse, json, math, time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import entorno_calibrado as EC
import entrenar_intensivo as EI
import misiones_sim as MS
import sim_logica_actual as S

ESPACIO = dict(EI.ESPACIO)
ESPACIO.update({"mis_dist_insp_m": ("u", 0.30, 0.90), "mis_escaneo_s": ("u", 3.0, 12.0),
                "guardian_tol_cm": ("u", 3.0, 10.0), "guardian_tol_deg": ("u", 2.0, 8.0),
                # maniobras específicas (ir al hueco, rumbo a la meta, escape de nicho)
                "mx_hueco_m": ("u", 0.6, 1.5), "mx_grados_pulso": ("u", 3.0, 10.0), "mx_giro_largo": ("u", 3, 9), "mx_bordear_s": ("u", 5.0, 25.0),
                "mx_rumbo_tol_deg": ("u", 12.0, 40.0), "mx_ventana_s": ("u", 15.0, 40.0)})
ESPACIO_ENTEROS = ("mx_giro_largo",)
CLAVES = list(ESPACIO)
EI.ESPACIO, EI.CLAVES = ESPACIO, CLAVES               # a_unidad / de_unidad de entrenar_intensivo usan estas globales
EXTRA_HOY = {"mis_dist_insp_m": 0.5, "mis_escaneo_s": 8.0, "guardian_tol_cm": 6.0, "guardian_tol_deg": 4.0,
             "mx_hueco_m": 1.0, "mx_grados_pulso": 6.0, "mx_giro_largo": 5, "mx_bordear_s": 12.0, "mx_rumbo_tol_deg": 25.0, "mx_ventana_s": 25.0}
HOY = dict(EI.HOY, **EXTRA_HOY)
MAN = dict(EI.MAN, **EXTRA_HOY)
NOMBRES = list(MS.misiones(0))


def campos(p, entorno, guardian=True, mx=True):
    c = EI.campos(p, entorno)
    if mx:
        c.update(mx_hueco=True, mx_meta=True, mx_anti_dither=True, mx_hueco_m=p.get("mx_hueco_m", 1.0), mx_grados_pulso=p.get("mx_grados_pulso", 6.0),
                 mx_giro_largo=int(round(p.get("mx_giro_largo", 5))), mx_bordear_s=p.get("mx_bordear_s", 12.0),
                 mx_rumbo_tol_deg=p.get("mx_rumbo_tol_deg", 25.0), mx_ventana_s=p.get("mx_ventana_s", 25.0))
    c.update(mision=True, guardian=guardian, guardian_usar_memoria=False, guardian_tol_cm=p.get("guardian_tol_cm", 6.0),
             guardian_tol_deg=p.get("guardian_tol_deg", 4.0), mis_dist_insp_m=p.get("mis_dist_insp_m", 0.5), mis_escaneo_s=p.get("mis_escaneo_s", 8.0))
    return c


def _job(a):
    p, nombre, semilla, extremo, modo = a
    guardian, mx = modo if isinstance(modo, tuple) else (modo, modo)
    try:
        ent, esc = MS.misiones(semilla)[nombre]
        S.SUBSTEP = 0.005
        r = S.episodio(esc, S.Config(**campos(p, ent, guardian, mx)), semilla, EC.hip(semilla, extremo))
        m = r["mision"]
        return {"mision": nombre, "seed": semilla, "costo": MS.costo(r), "exito": MS.exito(r), "llego": bool(r["llego"]), "contactos": int(r["contactos"]),
                "progreso": float(r["progreso"]), "t_s": float(r["t_s"]), "rmse_cm": float(r["rmse_cm"]), "plantas": m["plantas"], "detectadas": m["detectadas"],
                "recall": m["recall"], "precision": m["precision"], "error_conteo": m["error_conteo"], "escaneos": m["escaneos"], "t_parado_s": m["t_parado_s"]}
    except Exception as e:                                           # noqa: BLE001
        return {"mision": nombre, "seed": semilla, "costo": 150.0, "exito": False, "llego": False, "contactos": 9, "progreso": 0.0, "t_s": 0.0, "rmse_cm": 99.0,
                "plantas": 0, "detectadas": 0, "recall": 0.0, "precision": 0.0, "error_conteo": 1.0, "escaneos": 0, "t_parado_s": 0.0, "error": repr(e)}


def evaluar(pool, cands, semillas, extremo=False, guardian=True):
    tr = [(p, n, s, extremo, guardian if not isinstance(guardian, list) else guardian[i]) for i, p in enumerate(cands) for n in NOMBRES for s in semillas]
    res = pool.map(_job, tr, chunksize=1)
    n = len(NOMBRES) * len(semillas)
    return [res[i * n:(i + 1) * n] for i in range(len(cands))]


def resumen(rs):
    def media(l, k):
        return round(sum(r[k] for r in l) / len(l), 3)
    out = {"todo": {"n": len(rs), "costo": media(rs, "costo"), "exito_pct": round(100 * sum(r["exito"] for r in rs) / len(rs), 1),
                    "llego_pct": round(100 * sum(r["llego"] for r in rs) / len(rs), 1), "contactos_por_ep": media(rs, "contactos"),
                    "recall": media(rs, "recall"), "precision": media(rs, "precision"), "error_conteo": media(rs, "error_conteo"), "t_s": media(rs, "t_s"),
                    "rmse_cm": media(rs, "rmse_cm"), "escaneos": media(rs, "escaneos")}, "por_mision": {}}
    for n in NOMBRES:
        l = [r for r in rs if r["mision"] == n]
        out["por_mision"][n] = {"exito_pct": round(100 * sum(r["exito"] for r in l) / len(l), 1), "llego_pct": round(100 * sum(r["llego"] for r in l) / len(l), 1),
                                "costo": media(l, "costo"), "contactos_por_ep": media(l, "contactos"), "recall": media(l, "recall"),
                                "error_conteo": media(l, "error_conteo"), "t_s": media(l, "t_s"), "rmse_cm": media(l, "rmse_cm")}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--horas", type=float, default=5.0)
    ap.add_argument("--procs", type=int, default=4)
    ap.add_argument("--pob", type=int, default=16)
    ap.add_argument("--final-min", type=float, default=40.0)
    ap.add_argument("--sd", type=float, default=0.22)
    ap.add_argument("--val-semillas", type=int, default=6)
    ap.add_argument("--estres-semillas", type=int, default=3)
    ap.add_argument("--semillas-gen", type=int, default=1)
    a = ap.parse_args()
    t0 = time.time()
    fin = t0 + a.horas * 3600 - a.final_min * 60
    d = Path.home() / ".agrobot" / "aprendizaje" / ("misiones_" + time.strftime("%Y%m%d_%H%M"))
    d.mkdir(parents=True, exist_ok=True)
    log = open(d / "log.txt", "a")

    def say(*x):
        s = " ".join(str(i) for i in x)
        print(s, flush=True)
        log.write(s + "\n")
        log.flush()
    for k, v in EXTRA_HOY.items():
        EI.HOY.setdefault(k, v)
    import glob as _g
    previos = sorted(_g.glob(str(Path.home() / ".agrobot/aprendizaje/misiones_2026*/historial.json")))
    arranque = dict(MAN)
    if previos:
        try:
            arranque = dict(EXTRA_HOY, **json.load(open(previos[-1]))["archivo_top"][0]["params"])    # mejor de la corrida anterior (sin maniobras nuevas)
        except (OSError, ValueError, KeyError, IndexError):
            pass
    mu = EI.a_unidad(arranque)
    sd = np.full(len(CLAVES), a.sd)
    rng = np.random.default_rng(20261024)
    hist, archivo, gen = [], [], 0
    with Pool(a.procs) as pool:
        while time.time() < fin:
            gen += 1
            pob = [EI.de_unidad(mu)] + ([MAN, arranque] if gen == 1 else [])
            pob += [EI.de_unidad(mu + sd * rng.standard_normal(len(CLAVES))) for _ in range(a.pob)]
            pob = pob[:a.pob]
            semillas = [300 + gen * 5 + k for k in range(a.semillas_gen)]
            rs = evaluar(pool, pob, semillas)
            fit = np.array([sum(r["costo"] for r in x) / len(x) for x in rs])
            orden = np.argsort(fit)
            elite = orden[:max(4, a.pob // 4)]
            U = np.array([EI.a_unidad(pob[i]) for i in elite])
            mu = 0.5 * mu + 0.5 * U.mean(0)
            sd = np.maximum(0.05, 0.6 * sd + 0.4 * U.std(0) * 1.3)
            for i in orden[:6]:
                archivo.append({"gen": gen, "fit": round(float(fit[i]), 3), "params": pob[i], "res": resumen(rs[i])["todo"]})
            hist.append({"gen": gen, "min": round((time.time() - t0) / 60), "mejor": round(float(fit[orden[0]]), 3), "media": round(float(fit.mean()), 3),
                         "elite": round(float(fit[elite].mean()), 3), "mejor_exito": resumen(rs[orden[0]])["todo"]["exito_pct"],
                         "mejor_error_conteo": resumen(rs[orden[0]])["todo"]["error_conteo"], "sd_medio": round(float(sd.mean()), 3)})
            say(f"gen {gen:3d} · {hist[-1]['min']:4d} min · mejor {hist[-1]['mejor']:.2f} (éxito {hist[-1]['mejor_exito']}% · err.conteo {hist[-1]['mejor_error_conteo']}) "
                f"· media {hist[-1]['media']:.2f} · élite {hist[-1]['elite']:.2f} · sd {hist[-1]['sd_medio']}")
            json.dump({"historial": hist, "mu": mu.tolist(), "sd": sd.tolist(), "archivo_top": sorted(archivo, key=lambda x: x["fit"])[:40]},
                      open(d / "historial.json", "w"), indent=1)
        say("== VALIDACIÓN FINAL (semillas nunca vistas) ==")
        top, cand, vistos = sorted(archivo, key=lambda x: x["fit"]), [], set()
        for m in top:
            k = json.dumps(m["params"], sort_keys=True)
            if k not in vistos:
                vistos.add(k)
                cand.append(m["params"])
            if len(cand) == 5:
                break
        cand.append(EI.de_unidad(mu))
        nombres = [f"top{i+1}" for i in range(len(cand) - 1)] + ["media_final", "HOY_con_guardian_y_maniobras", "HOY_con_guardian", "ROBOT_DE_HOY"]
        todos = cand + [HOY, HOY, HOY]
        gflags = [(True, True)] * len(cand) + [(True, True), (True, False), (False, False)]
        semillas = list(range(7000, 7000 + a.val_semillas))
        val = evaluar(pool, todos, semillas, guardian=gflags)
        filas = [{"nombre": n, "params": p, **resumen(r)} for n, p, r in zip(nombres, todos, val)]
        for f in filas:
            say(f"  {f['nombre']:18s} {f['todo']}")
        mejor = min(filas[:len(cand)], key=lambda f: f["todo"]["costo"])
        say("mejor validado:", mejor["nombre"])
        say("== ESTRÉS (hardware más flojo) ==")
        est = evaluar(pool, [mejor["params"], HOY, HOY], list(range(8000, 8000 + a.estres_semillas)), extremo=True, guardian=[(True, True), (True, False), (False, False)])
        estres = {n: resumen(r) for n, r in zip(["mejor", "HOY_con_guardian", "ROBOT_DE_HOY"], est)}
        for n, v in estres.items():
            say(f"  {n:18s} {v['todo']}")
    json.dump({"duracion_min": round((time.time() - t0) / 60), "generaciones": gen, "historial": hist, "validacion": filas,
               "mejor": {"nombre": mejor["nombre"], "params": mejor["params"]}, "estres": estres, "misiones": NOMBRES,
               "archivo_top": sorted(archivo, key=lambda x: x["fit"])[:40]}, open(d / "informe.json", "w"), indent=1, ensure_ascii=False)
    say("Guardado", d / "informe.json", "· total", round((time.time() - t0) / 60), "min")


if __name__ == "__main__":
    main()
