"""Montecarlo del firmware C3 «suave» v4 con un robot PESADO y DELANTERAS FLOJAS.

Uso:  python3 montecarlo_firmware_pesado.py [--n 300] [--draws 3] [--procs 4]
Salida: ~/.agrobot/aprendizaje/mc_pesado_<fecha>.json y docs/MC_FIRMWARE_PESADO_<fecha>.md
NO despliega ni flashea nada.

Modelo del robot (hipótesis, NO medidas salvo la masa total ~12 kg):
  · masa del chasis 14-24 kg (se mide 12 kg: se prueba más pesado a propósito);
  · delanteras flojas: front_drive_bias 0.3-0.9 (el sim base usaba 1-2, delanteras MÁS fuertes);
  · zona muerta 25/40/55 % (un robot pesado necesita más duty para arrancar), mu y rodadura como siempre.
Firmwares comparados: v3 (skid-steer, ráfagas), suave (pegado y de fábrica) y FUSION (v3 + suave).

Cada configuración sortea TODOS los parámetros: los 7 límites del firmware (adel, atras, pivote,
minp, acel, frena, tras) y los 7 del controlador de pasillo de la Jetson (v, kp, kd, kh, kl, wmax,
dstop). Se compara con dos referencias: el firmware TAL CUAL se pegó (pivote 150, minp 150, tras 150:
fuera de rango, ver informe) y los de fábrica del repo.
Etapas: 1) barrido aleatorio  2) sensibilidad por parámetro  3) finalistas vs referencias en
semillas nuevas  4) barrido de un parámetro cada vez alrededor del mejor  5) estrés.
"""
import argparse
import json
import math
import time
from dataclasses import asdict
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import sim_logica_actual as S

SALIDA = Path.home() / ".agrobot" / "aprendizaje"
DOCS = Path(__file__).resolve().parent.parent / "docs"
ESCS = ["sala_vacia", "pasillo_macetas", "eslalon_macetas", "curva_pasillo"]
SUB = 0.005
BASE_CONT = dict(control="continuo", pivote_en_sitio=True, modo="interior", escala_velocidad=0.3)

# Rangos de TODOS los parámetros del firmware FUSION (= limValido() de fusion_model.h) y del
# controlador de pasillo de la Jetson. ("u" uniforme, "log" logarítmico)
RANGOS = {
    "fu_adel": ("u", 40, 100), "fu_atras": ("u", 30, 100), "fu_pivote": ("u", 40, 100),
    "fu_minp": ("u", 0, 90), "fu_acel": ("log", 30, 500), "fu_frena": ("log", 100, 900),
    "fu_tras": ("u", 100, 200), "fu_dif_del": ("u", 0, 100), "fu_periodo": ("u", 40, 400),
    "fu_refuerzo": ("u", 100, 130),
    "cc_v": ("u", 0.15, 0.40), "cc_kp": ("log", 1.0, 6.0), "cc_kd": ("u", 0.2, 1.2),
    "cc_kh": ("u", 0.8, 4.5), "cc_kl": ("log", 0.4, 5.0), "cc_wmax": ("u", 0.4, 1.0),
    "cc_dstop": ("u", 0.22, 0.45),
}
CC_MEDIO = dict(cc_v=0.27, cc_kp=3.0, cc_kd=0.6, cc_kh=2.5, cc_kl=2.0, cc_wmax=0.8, cc_dstop=0.30)
# Referencias: el v3 pegado (skid-steer, rafagas de 200 ms a tension completa, boost 1,15), el suave
# TAL CUAL se pegó (pivote 150, minp 150, tras 150: fuera de rango), el suave de fábrica del repo y la
# FUSION con sus valores de fábrica.
REF = {
    "V3": dict(firmware="skidsteer_v3", **CC_MEDIO),
    "SUAVE_PEGADO": dict(firmware="suave_v4", su_adel=80, su_atras=60, su_pivote=150, su_minp=150,
                         su_acel=120, su_frena=300, su_tras=150, **CC_MEDIO),
    "SUAVE_REPO": dict(firmware="suave_v4", su_adel=80, su_atras=60, su_pivote=100, su_minp=50,
                       su_acel=120, su_frena=300, su_tras=130, **CC_MEDIO),
    "FUSION_FABRICA": dict(firmware="fusion", **CC_MEDIO),
}


def muestrear_config(rng):
    d = {}
    for k, (tipo, a, b) in RANGOS.items():
        d[k] = float(rng.uniform(a, b)) if tipo == "u" else float(math.exp(rng.uniform(math.log(a), math.log(b))))
    d["fu_minp"] = min(d["fu_minp"], d["fu_adel"], d["fu_atras"])      # regla del firmware
    d = {k: round(v, 3) for k, v in d.items()}
    d["firmware"] = "fusion"
    return d


def entorno(seed, extremo=False):
    """Hardware pesado de la semilla. extremo = el peor caso plausible (etapa de estrés)."""
    r = np.random.default_rng(31_000 + seed)
    if extremo:
        return dict(masa=24.0, bias=0.3, dead=55.0, mu=0.65, drag=0.22)
    return dict(masa=float(r.uniform(14, 24)), bias=float(r.uniform(0.3, 0.9)),
                dead=float(r.choice([25.0, 40.0, 55.0])), mu=float(r.choice([0.65, 0.8, 0.95])),
                drag=float(r.choice([0.13, 0.17, 0.22])))


def _job(a):
    cfg_d, esc, seed, extremo, hueco = a
    S.SUBSTEP = SUB
    cfg = S.Config(**{**BASE_CONT, **cfg_d})
    hip = S.hip_por_semilla(seed)
    for k, v in entorno(seed, extremo).items():
        setattr(hip, k, v)
    try:
        r = S.episodio(S.escenarios_interior(hueco)[esc], cfg, seed, hip)
    except Exception as e:                       # un episodio roto no tumba 2 h de cómputo
        return {"escenario": esc, "seed": seed, "error": repr(e), "exito": False, "contactos": 99,
                "progreso": 0.0, "t_s": 99.0, "atascado": True, "puntos": 99.0}
    return {"escenario": esc, "seed": seed, "exito": bool(r["exito"]), "contactos": int(r["contactos"]),
            "progreso": float(r["progreso"]), "t_s": float(r["t_s"]), "atascado": bool(r["atascado"]),
            "puntos": round(float(S.puntuar_v2(r)), 3)}


def evaluar(pool, cfgs, seeds, extremo=False, hueco=0.60):
    """cfgs: lista de dict. Devuelve lista de listas de resultados (una por config)."""
    jobs = [(c, e, s, extremo, hueco) for c in cfgs for e in ESCS for s in seeds]
    res = pool.map(_job, jobs, chunksize=4)
    por = len(ESCS) * len(seeds)
    return [res[i * por:(i + 1) * por] for i in range(len(cfgs))]


def resumen(rs):
    n = len(rs)
    return {"n": n, "puntos": round(float(np.mean([r["puntos"] for r in rs])), 3),
            "exito_pct": round(100.0 * sum(r["exito"] for r in rs) / n, 1),
            "contactos_por_ep": round(sum(r["contactos"] for r in rs) / n, 2),
            "sin_contacto_pct": round(100.0 * sum(r["contactos"] == 0 for r in rs) / n, 1),
            "atasco_pct": round(100.0 * sum(r["atascado"] for r in rs) / n, 1),
            "progreso": round(float(np.mean([r["progreso"] for r in rs])), 3),
            "t_s": round(float(np.mean([r["t_s"] for r in rs])), 1)}


def por_esc(rs):
    return {e: resumen([r for r in rs if r["escenario"] == e]) for e in ESCS}


def rangos_ordenados(x, y):
    """Spearman sin scipy."""
    rx = np.argsort(np.argsort(x)); ry = np.argsort(np.argsort(y))
    if rx.std() == 0 or ry.std() == 0:
        return 0.0
    return float(np.corrcoef(rx, ry)[0, 1])


def sensibilidad(cfgs, resumenes):
    """Por parámetro: Spearman con la puntuación y el éxito, y media por cuartil del valor."""
    out = {}
    pun = np.array([r["puntos"] for r in resumenes]); ex = np.array([r["exito_pct"] for r in resumenes])
    for k in RANGOS:
        v = np.array([c[k] for c in cfgs])
        q = np.quantile(v, [0.25, 0.5, 0.75])
        cuart = np.digitize(v, q)
        out[k] = {"spearman_puntos": round(rangos_ordenados(v, pun), 3),
                  "spearman_exito": round(rangos_ordenados(v, ex), 3),
                  "por_cuartil": [{"desde": round(float(v[cuart == i].min()), 3), "hasta": round(float(v[cuart == i].max()), 3),
                                   "puntos": round(float(pun[cuart == i].mean()), 2), "exito_pct": round(float(ex[cuart == i].mean()), 1)}
                                  for i in range(4) if (cuart == i).any()]}
    return out


def barrido_uno(pool, mejor, seeds, niveles=5):
    out = {}
    for k, (tipo, a, b) in RANGOS.items():
        n_niv = niveles if k.startswith("fu_") else 3
        if tipo == "u":
            vals = np.linspace(a, b, n_niv)
        else:
            vals = np.exp(np.linspace(math.log(a), math.log(b), n_niv))
        cfgs = []
        for v in vals:
            c = dict(mejor); c[k] = round(float(v), 3)
            if k in ("fu_adel", "fu_atras"):
                c["fu_minp"] = min(c["fu_minp"], c["fu_adel"], c["fu_atras"])
            cfgs.append(c)
        rs = evaluar(pool, cfgs, seeds)
        out[k] = [{"valor": c[k], **resumen(r)} for c, r in zip(cfgs, rs)]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--draws", type=int, default=3)
    ap.add_argument("--procs", type=int, default=4)
    ap.add_argument("--top", type=int, default=8)
    a = ap.parse_args()
    t0 = time.time()
    rng = np.random.default_rng(2026_10_18)
    cfgs = [muestrear_config(rng) for _ in range(a.n)]
    TRAIN, TEST, BARR = list(range(100, 100 + a.draws)), list(range(200, 210)), list(range(300, 306))
    informe = {"fecha": time.strftime("%Y-%m-%d %H:%M"), "n": a.n, "draws": a.draws}
    with Pool(a.procs) as pool:
        print("[0/5] comparacion de firmwares (mismos 8 entornos pesados)", flush=True)
        rc = evaluar(pool, list(REF.values()), list(range(500, 508)))
        informe["comparacion_firmwares"] = {k: {"total": resumen(r), "por_escenario": por_esc(r)}
                                            for k, r in zip(REF, rc)}
        for k, v in informe["comparacion_firmwares"].items():
            print("      ", k, v["total"], flush=True)
        (SALIDA / "mc_pesado_parcial.json").write_text(json.dumps(informe, indent=1, ensure_ascii=False))
        print(f"[1/5] barrido: {a.n} configs x {a.draws} entornos x {len(ESCS)} esc", flush=True)
        ref_tr = evaluar(pool, list(REF.values()), TRAIN)
        informe["ref_train"] = {k: resumen(r) for k, r in zip(REF, ref_tr)}
        rs = evaluar(pool, cfgs, TRAIN)
        res = [resumen(r) for r in rs]
        informe["mejor_train"] = sorted(res, key=lambda r: r["puntos"])[0]
        print(f"      mejor puntos {informe['mejor_train']['puntos']} · {time.time()-t0:.0f}s", flush=True)

        print("[2/5] sensibilidad por parametro", flush=True)
        informe["sensibilidad"] = sensibilidad(cfgs, res)
        orden = np.argsort([r["puntos"] for r in res])

        print(f"[3/5] top {a.top} + referencias en semillas nuevas ({len(TEST)})", flush=True)
        fin = [cfgs[i] for i in orden[:a.top]]
        todos = fin + list(REF.values())
        rt = evaluar(pool, todos, TEST)
        filas = [{"nombre": f"top{i+1}", "config": c, "total": resumen(r), "por_escenario": por_esc(r)}
                 for i, (c, r) in enumerate(zip(todos[:a.top], rt[:a.top]))]
        filas += [{"nombre": k, "config": c, "total": resumen(r), "por_escenario": por_esc(r)}
                  for k, c, r in zip(REF, todos[a.top:], rt[a.top:])]
        informe["finalistas_test"] = filas
        mejor_i = min(range(a.top), key=lambda i: filas[i]["total"]["puntos"])
        mejor = filas[mejor_i]
        informe["elegido"] = {"nombre": mejor["nombre"], "config": mejor["config"]}
        print("      elegido:", mejor["nombre"], mejor["total"], flush=True)

        print("[4/5] barrido de un parametro cada vez alrededor del elegido", flush=True)
        informe["barrido_un_parametro"] = barrido_uno(pool, mejor["config"], BARR)

        print("[5/5] estres: masa 24 kg, delanteras 0.3, zona muerta 55 %", flush=True)
        rest = evaluar(pool, [mejor["config"]] + list(REF.values()), list(range(400, 408)), extremo=True)
        informe["estres"] = {n: {"total": resumen(r), "por_escenario": por_esc(r)}
                             for n, r in zip(["elegido", *REF], rest)}
    informe["duracion_s"] = round(time.time() - t0)
    SALIDA.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M")
    (SALIDA / f"mc_pesado_{stamp}.json").write_text(json.dumps(informe, indent=1, ensure_ascii=False))
    print("Guardado", SALIDA / f"mc_pesado_{stamp}.json", flush=True)


if __name__ == "__main__":
    main()
