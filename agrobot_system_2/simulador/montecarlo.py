#!/usr/bin/env python3
"""
Montecarlo: prueba cientos de configuraciones y se queda con la mejor.

    python3 simulador/montecarlo.py --variantes 60 --mundo exterior_2_sim_dificil
    python3 simulador/montecarlo.py --variantes 40 --aplicar     # y la instala en el robot

Qué hace, y por qué así:

  · SORTEA variantes de los parámetros que el operador toca a mano en el
    dashboard (conos del freno, margen de parada, velocidad, tolerancia del
    sendero, inclinación de la cámara…). Ajustarlos a ojo en el campo cuesta
    una tarde por prueba; aquí se prueban decenas en minutos.
  · Las compara con las MISMAS semillas y los MISMOS escenarios. Sin eso, una
    variante podría ganar por haberle tocado un terreno más fácil.
  · Usa ELIMINACIÓN POR RONDAS: todas corren un episodio, se quedan las
    mejores, esas corren más episodios, y así. Gastar el mismo tiempo en una
    variante mala que en una buena es tirar cómputo.
  · Corre en paralelo (un proceso por núcleo menos dos, para no dejar sin CPU
    al backend del robot).

Lo que sale (`~/.agrobot/aprendizaje/montecarlo.json`) es una configuración
para el ROBOT REAL: con `--aplicar` se escribe en `agrobot_config.json`
dejando copia de seguridad.
"""
import argparse
import copy
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))

CARPETA = Path.home() / ".agrobot" / "aprendizaje"
CONFIG_ROBOT = AQUI.parent / "jetson" / "backend" / "agrobot_config.json"
MUNDOS_DIR = AQUI / "mundos"
SALIDA = CARPETA / "montecarlo.json"

# Qué se sortea: (sección, clave, mínimo, máximo, decimales). Son los mismos
# parámetros que el operador ajusta en el dashboard del robot.
PARAMETROS = [
    ("slam", "cono_frontal_deg", 18, 60, 0),
    ("slam", "margen_parada_cm", 8, 30, 0),
    ("slam", "deslizamiento_cm", 20, 60, 0),
    ("autonomia", "escala_velocidad", 0.30, 0.90, 2),
    ("autonomia", "avance_libre_cm", 55, 140, 0),
    ("autonomia", "lado_libre_cm", 30, 80, 0),
    ("autonomia", "retroceso_libre_cm", 20, 60, 0),
    ("autonomia", "sendero_tolerancia_cm", 4, 25, 0),
    ("autonomia", "sendero_vision_peso", 0.3, 1.0, 2),
]
# La inclinación de la cámara no vive en la config del robot (la fija el
# servo), pero se sortea igual: es de lo que más cambia el resultado.
SERVO_MIN, SERVO_MAX = 54, 98

# Cuánto pesa cada cosa al puntuar. Pisar plantas y chocar son lo caro: el
# robot trabaja entre cultivo, y un choque en el campo es una avería.
PESOS = {"plantas_pisadas": 12.0, "colisiones": 25.0, "contacto_s": 1.5,
         "desvio_ruta_medio_cm": 0.35, "recorrido_m": -1.2}


def sortear(rng, base) -> dict:
    """Una variante al azar dentro de los límites de cada parámetro."""
    v = {"servo": int(rng.integers(SERVO_MIN, SERVO_MAX + 1))}
    for seccion, clave, lo, hi, dec in PARAMETROS:
        x = float(rng.uniform(lo, hi))
        v[f"{seccion}.{clave}"] = round(x, dec) if dec else int(round(x))
    return v


def aplicar_variante(cfg, variante) -> dict:
    cfg = copy.deepcopy(cfg)
    for k, valor in variante.items():
        if k == "servo":
            continue
        seccion, clave = k.split(".")
        cfg[seccion][clave] = valor
    return cfg


def puntuar(resultados) -> float:
    """Menos es mejor. Se promedian los episodios de la variante."""
    if not resultados:
        return 1e9
    total = 0.0
    for r in resultados:
        total += sum(peso * float(r.get(clave, 0) or 0) for clave, peso in PESOS.items())
    return total / len(resultados)


def _un_episodio(args) -> dict:
    """Un episodio en un proceso aparte (para el paralelo)."""
    variante, cfg_base, escenario, semilla, segundos = args
    os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
    import correr

    cfg = aplicar_variante(cfg_base, variante)
    correr.SERVO_SENDERO_DEG = int(variante.get("servo", correr.SERVO_SENDERO_DEG))
    try:
        ep = correr.Episodio(cargar_escenario(escenario, semilla), cfg, segundos, semilla)
        ep.calibrar_servo = False          # la inclinación la fija la variante
        return ep.correr()
    except Exception as exc:
        # Una variante que revienta no debe tumbar la búsqueda: se penaliza.
        return {"error": f"{type(exc).__name__}: {exc}", "colisiones": 99,
                "plantas_pisadas": 99, "desvio_ruta_medio_cm": 999, "recorrido_m": 0,
                "contacto_s": 99}


def cargar_escenario(escenario, semilla):
    """Nombre de escenario, o mapa guardado -> (mundo, pose)."""
    import mundo as M

    if not escenario.startswith("mundo:"):
        return escenario
    nombre = escenario.split(":", 1)[1]
    datos = json.loads((MUNDOS_DIR / f"{nombre}.json").read_text())
    m = M.Mundo.desde_dict(datos)
    ruta = np.asarray(m.sendero, np.float64)
    if len(ruta) >= 2:
        i = int(semilla) % (len(ruta) - 1)
        pose = (float(ruta[i][0]), float(ruta[i][1]),
                math.atan2(ruta[i + 1][1] - ruta[i][1], ruta[i + 1][0] - ruta[i][0]))
    else:
        x0, y0, x1, y1 = m.limites
        pose = ((x0 + x1) / 2, (y0 + y1) / 2, 0.0)
    return (m, pose)


def buscar(variantes=40, cfg_base=None, escenarios=("aleatorio",), segundos=90,
           rondas=3, semilla=0, procesos=None):
    """Sorteo + eliminación por rondas. Devuelve el ranking."""
    rng = np.random.default_rng(semilla)
    candidatas = [{"id": i, "variante": sortear(rng, cfg_base), "resultados": []}
                  for i in range(variantes)]
    # La configuración actual del robot entra como candidata: si ninguna
    # variante la supera, hay que saberlo.
    actual = {"servo": 72}
    for seccion, clave, *_ in PARAMETROS:
        actual[f"{seccion}.{clave}"] = cfg_base[seccion][clave]
    candidatas.append({"id": "actual", "variante": actual, "resultados": []})

    procesos = procesos or max(1, (os.cpu_count() or 4) - 2)
    print(f"{len(candidatas)} variantes · {rondas} rondas · {procesos} procesos en paralelo")
    for ronda in range(rondas):
        episodios = 2 ** ronda
        tareas, indice = [], []
        for c in candidatas:
            for k in range(episodios):
                esc = escenarios[(k + ronda) % len(escenarios)]
                tareas.append((c["variante"], cfg_base, esc, 1000 + ronda * 97 + k, segundos))
                indice.append(c)
        t0 = time.perf_counter()
        with ProcessPoolExecutor(max_workers=procesos) as pool:
            for c, r in zip(indice, pool.map(_un_episodio, tareas, chunksize=1)):
                c["resultados"].append(r)
        for c in candidatas:
            c["puntos"] = puntuar(c["resultados"])
        candidatas.sort(key=lambda c: c["puntos"])
        mejor = candidatas[0]
        print(f"  ronda {ronda+1}: {len(tareas)} episodios en "
              f"{time.perf_counter()-t0:.0f}s · mejor {mejor['id']} "
              f"({mejor['puntos']:.1f} puntos)", flush=True)
        if ronda < rondas - 1:
            sobreviven = candidatas[:max(3, len(candidatas) // 2)]
            # La configuración ACTUAL nunca se elimina: es la referencia con
            # la que hay que comparar, y sin ella no se sabe si la ganadora
            # mejora algo o solo gana entre variantes malas.
            actual_c = next((c for c in candidatas if c["id"] == "actual"), None)
            if actual_c is not None and actual_c not in sobreviven:
                sobreviven.append(actual_c)
            candidatas = sobreviven
    return candidatas


def resumen_variante(c) -> dict:
    prom = lambda k: round(float(np.mean([r.get(k, 0) or 0 for r in c["resultados"]])), 2)
    return {"id": c["id"], "puntos": round(c["puntos"], 2), "episodios": len(c["resultados"]),
            "plantas_pisadas": prom("plantas_pisadas"), "colisiones": prom("colisiones"),
            "desvio_ruta_cm": prom("desvio_ruta_medio_cm"), "recorrido_m": prom("recorrido_m"),
            "contacto_s": prom("contacto_s"), "variante": c["variante"]}


def aplicar_al_robot(variante, config=CONFIG_ROBOT):
    """Escribe la configuración ganadora en la del robot, con copia previa."""
    datos = json.loads(config.read_text())
    copia = config.with_suffix(f".bak_{time.strftime('%Y%m%d_%H%M')}")
    copia.write_text(json.dumps(datos, indent=2, ensure_ascii=False))
    cambios = []
    for k, valor in variante.items():
        if k == "servo":
            continue
        seccion, clave = k.split(".")
        antes = datos[seccion].get(clave)
        if antes != valor:
            cambios.append(f"  {seccion}.{clave}: {antes} -> {valor}")
            datos[seccion][clave] = valor
    config.write_text(json.dumps(datos, indent=2, ensure_ascii=False))
    print(f"copia de seguridad: {copia.name}")
    print("\n".join(cambios) if cambios else "  (sin cambios)")
    print(f"inclinación de cámara recomendada: servo {variante.get('servo')}°")
    print("reiniciá el backend para que la tome: sudo systemctl restart agrobot-backend")
    return cambios


def main():
    p = argparse.ArgumentParser(description="Búsqueda Montecarlo de la mejor configuración")
    p.add_argument("--variantes", type=int, default=40)
    p.add_argument("--rondas", type=int, default=3)
    p.add_argument("--segundos", type=float, default=90)
    p.add_argument("--mundo", action="append", default=[],
                   help="mapa guardado (se puede repetir); si no, escenarios al azar")
    p.add_argument("--escenarios", default="aleatorio,hileras",
                   help="escenarios de siempre, separados por comas")
    p.add_argument("--procesos", type=int)
    p.add_argument("--semilla", type=int, default=0)
    p.add_argument("--aplicar", action="store_true",
                   help="escribe la ganadora en agrobot_config.json (con copia)")
    p.add_argument("--config", default=str(CONFIG_ROBOT))
    a = p.parse_args()

    cfg = json.loads(Path(a.config).read_text())
    cfg["autonomia"].update(sendero_activo=True, sendero_paja=True)
    escenarios = [f"mundo:{m}" for m in a.mundo] + [e for e in a.escenarios.split(",") if e]

    t0 = time.perf_counter()
    ranking = buscar(a.variantes, cfg, tuple(escenarios), a.segundos, a.rondas,
                     a.semilla, a.procesos)
    CARPETA.mkdir(parents=True, exist_ok=True)
    salida = [resumen_variante(c) for c in ranking]
    SALIDA.write_text(json.dumps({"escenarios": escenarios, "segundos": a.segundos,
                                  "ranking": salida}, indent=1, ensure_ascii=False))

    print(f"\n=== mejores variantes ({time.perf_counter()-t0:.0f}s) ===")
    print(f"{'id':>7s} {'puntos':>8s} {'pisadas':>8s} {'choques':>8s} {'desvío':>8s} {'recorr':>8s}")
    for r in salida[:6]:
        print(f"{str(r['id']):>7s} {r['puntos']:8.1f} {r['plantas_pisadas']:8.2f} "
              f"{r['colisiones']:8.2f} {r['desvio_ruta_cm']:7.1f}cm {r['recorrido_m']:7.1f}m")
    mejor = salida[0]
    actual = next((r for r in salida if r["id"] == "actual"), None)
    if actual and mejor["id"] != "actual":
        d = actual["puntos"] - mejor["puntos"]
        print(f"\nfrente a la configuración ACTUAL del robot:")
        print(f"  puntos:  {actual['puntos']:.1f} -> {mejor['puntos']:.1f} "
              f"({'mejora' if d > 0 else 'EMPEORA'} {abs(d):.1f})")
        print(f"  pisadas: {actual['plantas_pisadas']:.2f} -> {mejor['plantas_pisadas']:.2f}")
        print(f"  choques: {actual['colisiones']:.2f} -> {mejor['colisiones']:.2f}")
        print(f"  desvío:  {actual['desvio_ruta_cm']:.1f} -> {mejor['desvio_ruta_cm']:.1f} cm")
        if d <= 0:
            print("  => conviene DEJAR la configuración actual")
    print("\nganadora:")
    for k, v in sorted(mejor["variante"].items()):
        print(f"  {k}: {v}")
    if a.aplicar:
        print("\n=== aplicando al robot ===")
        aplicar_al_robot(mejor["variante"], Path(a.config))
    else:
        print(f"\npara instalarla en el robot: python3 {Path(__file__).name} --aplicar ...")


if __name__ == "__main__":
    main()
