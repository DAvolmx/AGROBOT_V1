"""Ronda larga (por defecto 5 h) de entrenamiento del firmware FUSION con robot pesado y delanteras FUERTES y traseras flojas.

Método: búsqueda por entropía cruzada (CEM) sobre los 17 parámetros (10 del firmware + 7 del controlador
de pasillo), con ENTORNOS NUEVOS en cada generación (no se sobreajusta a unas semillas). Luego, con el
tiempo que reste, validación de los mejores en semillas que no se vieron + estrés + referencias.
NO despliega ni flashea nada. Reanudable: guarda el estado tras cada generación.

Uso: python3 entrenar_fusion_5h.py [--horas 5] [--procs 6] [--pob 24] [--draws 3]
Salida: ~/.agrobot/aprendizaje/fusion5h_<fecha>/{estado.json,informe.json,log.txt}
"""
import argparse, json, math, time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import montecarlo_firmware_pesado as M

CLAVES = list(M.RANGOS)
FINAL_MIN = 50            # minutos reservados para la validación final


def a_unidad(cfg):
    u = []
    for k in CLAVES:
        t, a, b = M.RANGOS[k]
        v = cfg[k]
        u.append((math.log(v) - math.log(a)) / (math.log(b) - math.log(a)) if t == "log" else (v - a) / (b - a))
    return np.clip(np.array(u), 0, 1)


def de_unidad(u):
    d = {}
    for k, x in zip(CLAVES, np.clip(u, 0, 1)):
        t, a, b = M.RANGOS[k]
        d[k] = round(float(math.exp(math.log(a) + x * (math.log(b) - math.log(a))) if t == "log" else a + x * (b - a)), 3)
    d["fu_minp"] = min(d["fu_minp"], d["fu_adel"], d["fu_atras"])
    d["firmware"] = "fusion"
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--horas", type=float, default=5.0)
    ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--pob", type=int, default=24)
    ap.add_argument("--draws", type=int, default=3)
    a = ap.parse_args()
    t0 = time.time()
    fin_busqueda = t0 + a.horas * 3600 - FINAL_MIN * 60
    d = Path.home() / ".agrobot" / "aprendizaje" / ("fusion5h_" + time.strftime("%Y%m%d_%H%M"))
    d.mkdir(parents=True, exist_ok=True)
    log = open(d / "log.txt", "a")

    def say(*x):
        s = " ".join(str(i) for i in x)
        print(s, flush=True); log.write(s + "\n"); log.flush()

    prev = json.load(open(sorted(M.SALIDA.glob("mc_pesado_2*.json"))[-1]))
    elegido = prev["elegido"]["config"]
    consenso = json.load(open(M.SALIDA / "mc_consenso.json"))["config_consenso"]
    mu = a_unidad(elegido) * 0.6 + a_unidad(consenso) * 0.4
    sd = np.full(len(CLAVES), 0.22)
    rng = np.random.default_rng(20261018)
    archivo = []                                        # (puntos medios, config) de cada candidato evaluado
    gen = 0
    with Pool(a.procs) as pool:
        while time.time() < fin_busqueda:
            gen += 1
            pob = [de_unidad(mu)] if gen == 1 else []   # la media actual siempre compite
            pob += [de_unidad(mu + sd * rng.standard_normal(len(CLAVES))) for _ in range(a.pob - len(pob))]
            semillas = list(range(1000 + gen * 10, 1000 + gen * 10 + a.draws))
            rs = M.evaluar(pool, pob, semillas)
            res = [M.resumen(r) for r in rs]
            pun = np.array([r["puntos"] for r in res])
            orden = np.argsort(pun)
            elite = orden[:max(4, a.pob // 4)]
            U = np.array([a_unidad(pob[i]) for i in elite])
            mu = 0.5 * mu + 0.5 * U.mean(0)
            sd = np.maximum(0.06, 0.6 * sd + 0.4 * U.std(0) * 1.3)
            for i in orden:
                archivo.append({"gen": gen, "puntos": res[i]["puntos"], "exito_pct": res[i]["exito_pct"],
                                "contactos_por_ep": res[i]["contactos_por_ep"], "config": pob[i]})
            say(f"gen {gen:2d} · {(time.time()-t0)/60:5.0f} min · mejor {pun[orden[0]]:.2f} (éxito {res[orden[0]]['exito_pct']}%) · "
                f"media {pun.mean():.2f} · élite {pun[elite].mean():.2f} · sd medio {sd.mean():.3f}")
            json.dump({"gen": gen, "mu": mu.tolist(), "sd": sd.tolist(), "archivo": archivo[-400:]},
                      open(d / "estado.json", "w"))

        say("== validación final en semillas nuevas ==")
        # candidatos: los 4 mejores del ARCHIVO por puntos medios (>=2 gens distintas no hace falta) + la media final
        mejores = sorted(archivo, key=lambda x: x["puntos"])
        cand, vistos = [], set()
        for m in mejores:
            clave = json.dumps(m["config"], sort_keys=True)
            if clave not in vistos:
                vistos.add(clave); cand.append(m["config"])
            if len(cand) == 4:
                break
        cand.append(de_unidad(mu))
        refs = {"ELEGIDO_MC": elegido, "CONSENSO": consenso, "SUAVE_PEGADO": M.REF["SUAVE_PEGADO"], "SUAVE_REPO": M.REF["SUAVE_REPO"]}
        nombres = [f"cand{i+1}" for i in range(len(cand) - 1)] + ["media_final"] + list(refs)
        todos = cand + list(refs.values())
        val = M.evaluar(pool, todos, list(range(2000, 2016)))
        filas = [{"nombre": n, "config": c, "total": M.resumen(r), "por_escenario": M.por_esc(r)}
                 for n, c, r in zip(nombres, todos, val)]
        for f in filas:
            say(f"  {f['nombre']:13s} {f['total']}")
        mejor = min(filas[:len(cand)], key=lambda f: f["total"]["puntos"])
        say("mejor validado:", mejor["nombre"], mejor["total"])
        say("== estrés (24 kg, traseras mínimas, zona muerta 55 %) ==")
        est = M.evaluar(pool, [mejor["config"], refs["SUAVE_PEGADO"], refs["ELEGIDO_MC"]], list(range(3000, 3008)), extremo=True)
        estres = {n: {"total": M.resumen(r), "por_escenario": M.por_esc(r)} for n, r in zip(["mejor", "SUAVE_PEGADO", "ELEGIDO_MC"], est)}
        for n, v in estres.items():
            say(f"  {n:13s} {v['total']}")
    json.dump({"duracion_min": round((time.time() - t0) / 60), "generaciones": gen, "validacion": filas,
               "mejor": {"nombre": mejor["nombre"], "config": mejor["config"]}, "estres": estres,
               "archivo_top": mejores[:30]}, open(d / "informe.json", "w"), indent=1, ensure_ascii=False)
    say("Guardado", d / "informe.json", "· total", round((time.time() - t0) / 60), "min")


if __name__ == "__main__":
    main()
