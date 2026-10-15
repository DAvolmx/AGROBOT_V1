"""Tanda de entrenamiento en INTERIORES con la lógica ACTUAL de movimiento.

Uso:   python3 entrenar_interior.py [--n 64] [--procs 6]
Salida: ~/.agrobot/aprendizaje/interior_tanda_<fecha>.json  (+ log por pantalla)

Protocolo (el mismo criterio que el banco anterior):
  · Las SEMILLAS se reparten así y no se mezclan:  train 0-3 · val 4-9 · test 10-19.
    La semilla fija el chasis rústico, el rozamiento, la zona muerta del motor y la
    fuerza relativa del módulo trasero (hip_por_semilla), y la perturbación inicial.
  · Etapa 1: búsqueda aleatoria sobre TRAIN.  Etapa 2: los mejores en VAL.
    La elección se hace SOLO con val. Etapa 3: la elegida contra la config actual y el
    perfil de interior en TEST, que no se ha mirado antes.
  · Se acepta un candidato solo si en TEST no empeora contactos ni éxito y mejora la
    puntuación. Si no, se dice que no y se deja la config actual.
  · NO se despliega nada al robot. Esto produce una recomendación, no un cambio.

QUÉ SE VARÍA: lo que el operador puede cambiar mañana sin tocar código — modo de pulsos,
escala de velocidad, troceado, parámetros del firmware 1.2 (/config: rampa, reducción
interior, arranque mínimo) y umbrales del explorador. NO se varía `fw_fwd`: la Jetson
limita el avance a 0,60·V_SCALE (60 % de duty) en pulse_drive y manual_demo, así que
subirlo en el firmware no tendría efecto.
"""
import argparse
import json
import math
import random
import time
from dataclasses import asdict
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import sim_logica_actual as S

SALIDA = Path.home() / ".agrobot" / "aprendizaje"
ESCS = ["sala_vacia", "pasillo_macetas", "eslalon_macetas", "curva_pasillo"]
TRAIN, VAL, TEST = list(range(0, 4)), list(range(4, 10)), list(range(10, 20))

ESPACIO = {
    "modo": ["interior", "preciso", "exterior"],
    "escala_velocidad": [0.13, 0.2, 0.5],
    "suavidad_duty": [1.0, 0.7],
    "fw_slew": [60, 120, 200, 300],
    "fw_inner": [65, 80, 90],
    "fw_minp": [0, 30, 45],
    "sendero_tol_cm": [7, 10, 15],
    "avance_libre_cm": [40, 70, 100],
    "cono_frontal_deg": [10, 30],
}


OBJETIVO = 1        # 1 = puntuar (ronda 1), 2 = puntuar_v2 (ronda 2)


def _puntos(r):
    return S.puntuar_v2(r) if OBJETIVO == 2 else S.puntuar(r)


def _job(args):
    cfg_d, esc_nombre, seed, hueco = args
    cfg = S.Config(**cfg_d)
    esc = S.escenarios_interior(hueco)[esc_nombre]
    r = S.episodio(esc, cfg, seed)
    r["puntos"] = round(_puntos(r), 3)
    r["hueco"] = hueco
    return r


def evaluar(pool, cfgs, seeds, escs=ESCS, hueco=0.60):
    trabajos = [(asdict(c), e, s, hueco) for c in cfgs for e in escs for s in seeds]
    res = pool.map(_job, trabajos, chunksize=2)
    por = {}
    for r in res:
        por.setdefault(r["config"], []).append(r)
    return por


def resumen(rs):
    return {
        "puntos": round(float(np.mean([r["puntos"] for r in rs])), 3),
        "exito_pct": round(100.0 * float(np.mean([r["exito"] for r in rs])), 1),
        "llego_pct": round(100.0 * float(np.mean([r["llego"] for r in rs])), 1),
        "contactos": round(float(np.mean([r["contactos"] for r in rs])), 3),
        "progreso": round(float(np.mean([r["progreso"] for r in rs])), 3),
        "t_s": round(float(np.mean([r["t_s"] for r in rs])), 1),
        "pico_cm": round(float(np.mean([r["pico_cm"] for r in rs])), 1),
        "atascado_pct": round(100.0 * float(np.mean([r["atascado"] for r in rs])), 1),
        "corr_descartadas_pct": round(100.0 * (1 - sum(r["corr_ejecutadas"] for r in rs) /
                                                max(1, sum(r["correcciones"] for r in rs))), 1),
        "n": len(rs),
    }


def por_escenario(rs):
    out = {}
    for e in ESCS:
        sub = [r for r in rs if r["escenario"] == e]
        if sub:
            out[e] = resumen(sub)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=64)
    ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--ronda", type=int, default=1, choices=(1, 2))
    a = ap.parse_args()
    global OBJETIVO, TRAIN, VAL, TEST, ESPACIO
    OBJETIVO = 1 if a.ronda == 1 else 2
    if a.ronda == 2:
        # SEMILLAS NUEVAS: las de la ronda 1 ya se miraron en su TEST.
        TRAIN, VAL, TEST = list(range(20, 24)), list(range(24, 30)), list(range(30, 40))
        # Espacio acotado con lo aprendido: la escala .13 (35 % de duty) dejó el robot
        # inmóvil, y la rampa de 60 %/s solo es buena para pulsos largos.
        ESPACIO = {
            "modo": ["interior", "preciso", "exterior"],
            "escala_velocidad": [0.2, 0.5],
            "suavidad_duty": [1.0, 0.7],
            "fw_slew": [60, 150, 300],
            "fw_inner": [65, 80, 90],
            "fw_minp": [0, 30, 45],
            "sendero_tol_cm": [5, 7, 10],
            "avance_libre_cm": [40, 70, 100],
            "cono_frontal_deg": [10, 30],
        }
    t0 = time.perf_counter()
    rng = random.Random(20261015)

    actual = S.config_actual()
    perfil = S.config_interior_perfil()
    candidatos = []
    vistos = set()
    while len(candidatos) < a.n:
        d = {k: rng.choice(v) for k, v in ESPACIO.items()}
        clave = tuple(d.values())
        if clave in vistos:
            continue
        vistos.add(clave)
        candidatos.append(S.Config(nombre=f"c{len(candidatos):02d}", **d))

    salida = {"fecha": time.strftime("%Y-%m-%d %H:%M"), "ronda": a.ronda,
              "objetivo": "puntuar_v2" if a.ronda == 2 else "puntuar", "espacio": ESPACIO,
              "semillas": {"train": TRAIN, "val": VAL, "test": TEST}, "etapas": {}}

    with Pool(a.procs) as pool:
        print(f"[1/3] búsqueda sobre TRAIN: {a.n} configs x {len(TRAIN)} semillas x {len(ESCS)} escenarios", flush=True)
        t1 = time.perf_counter()
        r1 = evaluar(pool, [actual, perfil] + candidatos, TRAIN)
        tabla1 = {n: resumen(rs) for n, rs in r1.items()}
        cfg_por_nombre = {c.nombre: c for c in [actual, perfil] + candidatos}
        orden = sorted((n for n in tabla1 if n not in ("actual", "perfil_interior")),
                       key=lambda n: tabla1[n]["puntos"])
        print(f"      {time.perf_counter()-t1:.0f}s · actual {tabla1['actual']['puntos']} · "
              f"perfil {tabla1['perfil_interior']['puntos']} · mejor {orden[0]} {tabla1[orden[0]]['puntos']}", flush=True)
        salida["etapas"]["train"] = {n: {"cfg": asdict(cfg_por_nombre[n]), **tabla1[n]} for n in tabla1}
        (SALIDA).mkdir(parents=True, exist_ok=True)
        fich = SALIDA / f"interior_tanda_{time.strftime('%Y%m%d')}{'_ronda2' if a.ronda == 2 else ''}.json"
        fich.write_text(json.dumps(salida, indent=1, ensure_ascii=False))

        top = [cfg_por_nombre[n] for n in orden[:a.top]]
        print(f"[2/3] VALIDACIÓN: top {len(top)} + actual + perfil x {len(VAL)} semillas", flush=True)
        t2 = time.perf_counter()
        r2 = evaluar(pool, [actual, perfil] + top, VAL)
        tabla2 = {n: resumen(rs) for n, rs in r2.items()}
        elegido_n = min((n for n in tabla2 if n not in ("actual", "perfil_interior")),
                        key=lambda n: tabla2[n]["puntos"])
        print(f"      {time.perf_counter()-t2:.0f}s · elegido por VAL: {elegido_n} {tabla2[elegido_n]['puntos']} "
              f"(actual {tabla2['actual']['puntos']})", flush=True)
        salida["etapas"]["val"] = {n: {"cfg": asdict(cfg_por_nombre[n]), **tabla2[n]} for n in tabla2}
        elegido = cfg_por_nombre[elegido_n]
        elegido.nombre = "candidato"
        fich.write_text(json.dumps(salida, indent=1, ensure_ascii=False))

        print(f"[3/3] TEST: candidato vs actual vs perfil x {len(TEST)} semillas", flush=True)
        t3 = time.perf_counter()
        r3 = evaluar(pool, [actual, perfil, elegido], TEST)
        tabla3 = {n: resumen(rs) for n, rs in r3.items()}
        print(f"      {time.perf_counter()-t3:.0f}s", flush=True)
        salida["etapas"]["test"] = {n: {"cfg": asdict({'actual': actual, 'perfil_interior': perfil,
                                                       'candidato': elegido}[n]),
                                        **tabla3[n], "por_escenario": por_escenario(r3[n])}
                                    for n in tabla3}
        salida["elegido"] = asdict(elegido)

        # criterio de aceptación
        c, b = tabla3["candidato"], tabla3["actual"]
        acepta = (c["puntos"] < b["puntos"] and c["contactos"] <= b["contactos"] + 1e-9
                  and c["exito_pct"] >= b["exito_pct"])
        salida["aceptado"] = bool(acepta)
        print(f"      candidato {c['puntos']} vs actual {b['puntos']} -> "
              f"{'ACEPTADO' if acepta else 'NO aceptado'}", flush=True)

        # robustez a lo que NO se ha medido: zona muerta del motor y rozamiento
        print("[+] sensibilidad a la zona muerta del motor y al rozamiento (actual vs candidato)", flush=True)
        sens = {}
        for nombre, cfg in (("actual", actual), ("candidato", elegido)):
            sens[nombre] = {}
            for dead in (0.0, 25.0, 40.0):
                jobs = []
                for esc in ESCS:
                    for sd in TEST[:6]:
                        jobs.append((asdict(cfg), esc, sd, 0.60, dead))
                res = pool.map(_job_dead, jobs, chunksize=2)
                sens[nombre][f"dead={int(dead)}"] = resumen(res)
        salida["sensibilidad_dead"] = sens

        # hueco entre macetas (estrés), solo con las dos macetas
        print("[+] hueco entre macetas 50 / 60 / 70 cm", flush=True)
        huecos = {}
        for nombre, cfg in (("actual", actual), ("candidato", elegido)):
            huecos[nombre] = {}
            for h in (0.50, 0.60, 0.70):
                res = []
                for esc in ("pasillo_macetas", "eslalon_macetas"):
                    res += evaluar(pool, [cfg], TEST[:6], escs=[esc], hueco=h)[cfg.nombre]
                huecos[nombre][f"{int(h*100)}cm"] = resumen(res)
        salida["huecos"] = huecos

    salida["segundos"] = round(time.perf_counter() - t0)
    fich.write_text(json.dumps(salida, indent=1, ensure_ascii=False))
    print(f"\nGuardado en {fich}  ({salida['segundos']} s)", flush=True)


def _job_dead(args):
    cfg_d, esc_nombre, seed, hueco, dead = args
    cfg = S.Config(**cfg_d)
    hip = S.hip_por_semilla(seed)
    hip.dead = dead
    esc = S.escenarios_interior(hueco)[esc_nombre]
    r = S.episodio(esc, cfg, seed, hip)
    r["puntos"] = round(_puntos(r), 3)
    return r


if __name__ == "__main__":
    main()
