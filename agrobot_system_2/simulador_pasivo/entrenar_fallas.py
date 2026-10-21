"""Aprendizaje ASISTIDO sobre fallas (2026-10-20): choques, atascos, cortes de sensores y enlace, terreno, hardware...

Bucle («ponle las mismas pruebas y mira si las pasó, hasta que lo logre»):

  ronda 0   Se corre la batería de pruebas (cada falla en sus escenarios, semillas de ENTRENAMIENTO) con el robot
            SIN lecciones. Anota cuáles NO pasa.
  ronda k   1) Cada prueba fallida se repite con MANEJO ASISTIDO: un «operador» con información privilegiada del mundo
               verdadero prueba maniobras cuando el robot se traba; la que lo saca del apuro se guarda como LECCIÓN
               (firma de lo que ve el LIDAR -> secuencia de pasos), el mismo formato que usa el robot real.
            2) Se vuelven a correr LAS MISMAS pruebas SIN asistencia, solo con las lecciones aprendidas.
            3) Se corre la batería con semillas NUEVAS (validación): ¿aprendió algo general o memorizó casos?
  Para al lograr ≥95 % en entrenamiento y ≥85 % en validación, o cuando dos rondas seguidas no mejoran.

El robot nunca ve la información privilegiada: solo las lecciones, que se buscan por la firma del LIDAR. Una lección que
falla 3 veces deja de proponerse (aprendizaje.MAX_FRACASOS), así que las malas se descartan solas.

Salida: ~/.agrobot/aprendizaje/fallas_<fecha>.json (todo el historial) y maniobras_sim_<fecha>.json (las lecciones,
formato de `aprendizaje.Aprendizaje`). NO toca el maniobras.json del robot real: fusionarlas es decisión del operador.

Uso: python3 entrenar_fallas.py [--config interior|actual|optimo] [--procs 6] [--rondas 6] [--semillas 3] [--rapido]
"""
import argparse
import json
import time
from dataclasses import asdict
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import sim_logica_actual as S
import episodio_fallas as E
from episodio_fallas import AP        # aprendizaje.py del backend (el mismo módulo que usa el robot real)

SALIDA = Path.home() / ".agrobot" / "aprendizaje"
SEMILLAS_TRAIN = (60, 61, 62, 63, 64)
SEMILLAS_VAL = (70, 71, 72, 73, 74)
OBJ_TRAIN, OBJ_VAL = 95.0, 85.0
VACIO = Path("/nonexistent/maniobras_sim.json")


def _job(a):
    cfg_d, fallo, esc, seed, ruta, asistir = a
    ap = AP.Aprendizaje(Path(ruta) if ruta else VACIO, autoguardar=False)
    return E.episodio_fallas(S.escenarios_interior(0.60)[esc], S.Config(**cfg_d), seed, fallo, ap, asistir)


def correr(pool, cfg, pares, semillas, ruta, asistir=False):
    jobs = [(asdict(cfg), f, e, s, ruta, asistir) for (f, e) in pares for s in semillas]
    return pool.map(_job, jobs, chunksize=1)


def tasa(rs):
    return 100.0 * sum(r["pasa"] for r in rs) / max(1, len(rs))


def por_fallo(rs):
    out = {}
    for r in rs:
        out.setdefault(r["fallo"], []).append(r)
    return {f: {"n": len(v), "pasa_pct": round(tasa(v), 1), "contactos": sum(r["contactos"] for r in v),
                "recuperaciones": sum(r["recuperaciones"] for r in v), "con_leccion": sum(r["con_leccion"] for r in v)}
            for f, v in out.items()}


def fusionar(ap, resultados):
    """Mete en el almacén las lecciones nuevas del asistente y los juicios (éxito/fracaso) de las usadas."""
    nuevas = 0
    for r in resultados:
        for l in r["lecciones_nuevas"]:
            lista = ap.maniobras.setdefault(l["firma"], [])
            acc = [p["accion"] for p in l["pasos"]]
            ig = next((m for m in lista if [p["accion"] for p in m.pasos] == acc), None)
            if ig is not None:
                ig.exitos += 1
            else:
                lista.append(AP.Maniobra(l["firma"], l["pasos"], origen="asistente_sim", exitos=1,
                                         nota=f"candidata {l.get('candidata')}"))
                nuevas += 1
        for j in r["juicios"]:
            for m in ap.maniobras.get(j["firma"], []):
                if [p["accion"] for p in m.pasos] == j["pasos"]:
                    if j["ok"]:
                        m.exitos += 1
                    else:
                        m.fracasos += 1
    return nuevas


def config_base(nombre):
    if nombre == "actual":
        return S.config_actual()
    if nombre == "optimo":
        fs = sorted(SALIDA.glob("modelo_manejo_optimo_*.json"))
        if fs:
            d = json.loads(fs[-1].read_text())["config"]
            print(f"config óptima de {fs[-1].name}", flush=True)
            return S.Config(**d)
        print("no hay modelo_manejo_optimo_*.json todavía: se usa el perfil de interior", flush=True)
    return S.config_interior_perfil()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", choices=("interior", "actual", "optimo"), default="optimo")
    ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--rondas", type=int, default=6)
    ap.add_argument("--semillas", type=int, default=3)
    ap.add_argument("--rapido", action="store_true", help="prueba de humo: 3 fallas, 1 semilla, 2 rondas")
    a = ap.parse_args()
    cfg = config_base(a.config)
    pares = E.combinaciones(("ninguno", "maceta_baja", "esquina_inicio") if a.rapido else None)
    ntr = 1 if a.rapido else a.semillas
    tr, va = SEMILLAS_TRAIN[:ntr], SEMILLAS_VAL[:ntr]
    rondas = 2 if a.rapido else a.rondas
    ts = time.strftime("%Y%m%d_%H%M")
    snap = SALIDA / f"maniobras_sim_{ts}.json"
    SALIDA.mkdir(parents=True, exist_ok=True)
    almacen = AP.Aprendizaje(snap, autoguardar=False)
    t_ini = time.time()
    hist = []
    print(f"[fallas] {len(pares)} pruebas (falla x escenario) x {ntr} semillas · config {cfg.nombre} · {S.GEOMETRIA}", flush=True)
    with Pool(a.procs) as pool:
        t0 = time.time()
        r_tr = correr(pool, cfg, pares, tr, None)
        r_va = correr(pool, cfg, pares, va, None)
        hist.append({"ronda": 0, "train_pct": round(tasa(r_tr), 1), "val_pct": round(tasa(r_va), 1),
                     "lecciones": 0, "por_fallo_train": por_fallo(r_tr), "por_fallo_val": por_fallo(r_va)})
        print(f"[ronda 0] sin lecciones: entrenamiento {tasa(r_tr):.1f} % · validación {tasa(r_va):.1f} % "
              f"({int(time.time() - t0)} s)", flush=True)
        sin_mejora, mejor_tr = 0, tasa(r_tr)
        for ronda in range(1, rondas + 1):
            t0 = time.time()
            fallidos = [(r["fallo"], r["escenario"], r["semilla"]) for r in r_tr if not r["pasa"]]
            if not fallidos:
                print("[fallas] el robot ya pasa TODAS las pruebas de entrenamiento", flush=True)
                break
            almacen.ruta = snap
            almacen.guardar()
            jobs = [(asdict(cfg), f, e, s, str(snap), True) for (f, e, s) in fallidos]
            asist = pool.map(_job, jobs, chunksize=1)
            nuevas = fusionar(almacen, asist)
            resueltas = sum(1 for r in asist if r["pasa"])
            almacen.guardar()
            # LAS MISMAS pruebas, sin asistencia, con lo aprendido; y las semillas nuevas
            r_tr = correr(pool, cfg, pares, tr, str(snap))
            r_va = correr(pool, cfg, pares, va, str(snap))
            fusionar(almacen, r_tr)                       # el robot también aprende de usar las lecciones
            almacen.guardar()
            res = almacen.resumen()
            hist.append({"ronda": ronda, "fallidas_antes": len(fallidos), "resueltas_con_asistente": resueltas,
                         "lecciones_nuevas": nuevas, "lecciones": res["maniobras"], "situaciones": res["situaciones"],
                         "train_pct": round(tasa(r_tr), 1), "val_pct": round(tasa(r_va), 1),
                         "por_fallo_train": por_fallo(r_tr), "por_fallo_val": por_fallo(r_va)})
            print(f"[ronda {ronda}] fallidas {len(fallidos)} -> con asistente resueltas {resueltas} · lecciones {res['maniobras']} "
                  f"({nuevas} nuevas) · MISMAS pruebas {tasa(r_tr):.1f} % · semillas nuevas {tasa(r_va):.1f} % "
                  f"({int(time.time() - t0)} s)", flush=True)
            (SALIDA / f"fallas_{ts}.json").write_text(json.dumps({"config": asdict(cfg), "geometria": S.GEOMETRIA,
                                                                  "historial": hist}, indent=1, ensure_ascii=False))
            if tasa(r_tr) >= OBJ_TRAIN and tasa(r_va) >= OBJ_VAL:
                print("[fallas] OBJETIVO LOGRADO", flush=True)
                break
            sin_mejora = 0 if tasa(r_tr) > mejor_tr + 0.5 else sin_mejora + 1
            mejor_tr = max(mejor_tr, tasa(r_tr))
            if sin_mejora >= 2:
                print("[fallas] dos rondas sin mejorar: se detiene", flush=True)
                break
    ult = hist[-1]
    peor = sorted(ult["por_fallo_val"].items(), key=lambda kv: kv[1]["pasa_pct"])[:5]
    print(f"\nRESULTADO: {hist[0]['train_pct']}% -> {ult['train_pct']}% (mismas pruebas) · {hist[0]['val_pct']}% -> {ult['val_pct']}% "
          f"(semillas nuevas) · {almacen.resumen()['maniobras']} lecciones", flush=True)
    print("Fallas que más cuestan (validación):", {k: v["pasa_pct"] for k, v in peor}, flush=True)
    almacen.ruta = snap
    almacen.guardar()
    (SALIDA / f"fallas_{ts}.json").write_text(json.dumps({"config": asdict(cfg), "geometria": S.GEOMETRIA, "historial": hist,
                                                          "lecciones": almacen.resumen()["aprendidas"],
                                                          "segundos": int(time.time() - t_ini)}, indent=1, ensure_ascii=False))
    print(f"Lecciones en {snap}\nInforme en {SALIDA / f'fallas_{ts}.json'} ({int(time.time() - t_ini)} s)", flush=True)


if __name__ == "__main__":
    main()
