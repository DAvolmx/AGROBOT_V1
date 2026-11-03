"""CEM del EXPLORADOR REAL en el simulador fiel (2026-11-02).

Corre `correr.Episodio` (el mismo ExploradorReactivo, ConductorPulsos, atasco, giro a rumbo, suelo libre por cámara y cámara-como-LIDAR que el
backend) en 6 escenarios. Busca, a la vez, cuánto confiar en la cámara (camara_obstaculos, camara_como_lidar, umbral/textura del suelo por
apariencia) y los márgenes del explorador. Motivo: en la simulación la cámara marca obstáculo a ~0,42 m del centro (≈18 cm libres) y el
explorador pide `avance_libre_cm` = 100, así que casi no avanza.

Cada generación usa semillas NUEVAS; HOY (la config del robot) se evalúa en las mismas semillas. Al final valida HOY contra los 3 mejores en
semillas nunca vistas con episodios más largos. NO despliega nada.

Uso: python3 entrenar_explorador_7h.py --hasta <epoch> [--procs 4] [--pob 10] [--seg 180]
Salida: ~/.agrobot/aprendizaje/explorador_7h_<fecha>/{log.txt,historial.json,informe.json}
"""
import argparse, copy, json, math, os, time
from multiprocessing import get_context
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
CFG_ROBOT = AQUI.parent / "jetson" / "backend" / "agrobot_config.json"
ESCENARIOS = ["invernadero", "hileras", "callejon", "serpentina", "trampa_bucle", "campo_abierto"]

# nombre: (sección, tipo, min, max). tipo "b" = interruptor (u > 0,5)
ESP = {
    "camara_obstaculos": ("autonomia", "b", 0, 1),
    "camara_como_lidar": ("autonomia", "b", 0, 1),
    "suelo_umbral": ("autonomia", "u", 1.5, 5.0),
    "suelo_textura": ("autonomia", "u", 0.8, 2.5),
    "avance_libre_cm": ("autonomia", "u", 30, 120),
    "lado_libre_cm": ("autonomia", "u", 20, 70),
    "retroceso_libre_cm": ("autonomia", "u", 20, 50),
    "man_histeresis_cm": ("autonomia", "u", 5, 30),
    "man_pausa_s": ("autonomia", "u", 0.0, 2.0),
    "mx_grados_pulso": ("autonomia", "u", 6, 25),
    "e11_pred_margen_m": ("autonomia", "u", 0.02, 0.15),
    "atasco_reintento_s": ("autonomia", "u", 30, 150),
}
CL = list(ESP)


def leer_hoy(cfg):
    hoy = {}
    for k, (sec, t, a, b) in ESP.items():
        v = cfg[sec].get(k)
        if v is None:
            v = {"suelo_umbral": 2.6, "suelo_textura": 1.2, "atasco_reintento_s": 90.0}.get(k, (a + b) / 2)
        hoy[k] = float(bool(v)) if t == "b" else float(v)
    return hoy


def a_u(p):
    return np.array([min(max((p[k] - ESP[k][2]) / (ESP[k][3] - ESP[k][2]), 0.0), 1.0) for k in CL])


def de_u(u):
    p = {}
    for k, x in zip(CL, np.clip(u, 0, 1)):
        sec, t, a, b = ESP[k]
        p[k] = float(x > 0.5) if t == "b" else round(a + x * (b - a), 3)
    return p


def aplicar(cfg, p):
    c = copy.deepcopy(cfg)
    for k, v in p.items():
        sec, t = ESP[k][0], ESP[k][1]
        c[sec][k] = bool(v) if t == "b" else v
    return c


def costo(r):
    """Menos es mejor: avanzar y conocer celdas nuevas; pisar plantas y chocar castiga fuerte."""
    return (-1.0 * r["recorrido_m"] - 0.15 * r["celdas_visitadas"] + 4.0 * r["plantas_pisadas"] + 3.0 * r["colisiones"]
            + 0.3 * r["contacto_s"])


def episodio(tarea):
    cfg, esc, seg, semilla, ruta_apr = tarea
    import correr
    try:
        ep = correr.Episodio(esc, cfg, seg, semilla=semilla, ruta_aprendizaje=ruta_apr)
        r = ep.correr()
        errores = sum(1 for e in ep.eventos if e.get("evento") in ("error", "bucle_abortado"))
        return {k: r[k] for k in ("recorrido_m", "celdas_visitadas", "plantas_pisadas", "colisiones", "contacto_s", "pulsos",
                                  "desatascos", "segundos_reales", "estado_final")} | {"errores": errores}
    except Exception as exc:  # un episodio roto no tumba la generación; cuenta como malo
        return {"recorrido_m": 0.0, "celdas_visitadas": 0, "plantas_pisadas": 0, "colisiones": 0, "contacto_s": 0.0, "pulsos": 0,
                "desatascos": 0, "segundos_reales": 0, "estado_final": f"excepción {type(exc).__name__}: {exc}", "errores": 1}


def evaluar(pool, cfg_base, cands, seg, semillas, carpeta, etiqueta):
    tareas, idx = [], []
    for ci, p in enumerate(cands):
        for esc in ESCENARIOS:
            for s in semillas:
                ruta = carpeta / "aprendizaje" / f"{etiqueta}_{ci}_{esc}_{s}.json"   # sin contaminar entre candidatos
                tareas.append((aplicar(cfg_base, p), esc, seg, int(s), str(ruta)))
                idx.append((ci, esc, s))
    res = pool.map(episodio, tareas, chunksize=1)
    fallos = [r["estado_final"] for r in res if str(r["estado_final"]).startswith("excepción")]
    if fallos:                       # sin esto una generación entera de episodios rotos parecía «costo 0»
        print(time.strftime("%H:%M:%S"), f"AVISO {len(fallos)}/{len(res)} episodios fallaron:", fallos[0][:300], flush=True)
    out = [dict(costo=0.0, por_esc={}, fallos=len(fallos)) for _ in cands]
    for (ci, esc, s), r in zip(idx, res):
        out[ci]["costo"] += costo(r) / (len(ESCENARIOS) * len(semillas))
        out[ci]["por_esc"].setdefault(esc, []).append(r)
    return out


def resumen_por_esc(ev):
    return {e: {"recorrido_m": round(float(np.mean([r["recorrido_m"] for r in rs])), 2),
                "celdas": round(float(np.mean([r["celdas_visitadas"] for r in rs])), 1),
                "plantas_pisadas": round(float(np.mean([r["plantas_pisadas"] for r in rs])), 2),
                "colisiones": round(float(np.mean([r["colisiones"] for r in rs])), 2)} for e, rs in ev["por_esc"].items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hasta", type=float, required=True, help="epoch de fin (incluye la validación final)")
    ap.add_argument("--procs", type=int, default=4)
    ap.add_argument("--procs-antes", type=int, default=None, help="procesos hasta --procs-desde (p. ej. mientras corre el simulador visual)")
    ap.add_argument("--procs-desde", type=float, default=0.0, help="epoch a partir del cual se usan --procs")
    ap.add_argument("--pob", type=int, default=10)
    ap.add_argument("--seg", type=float, default=180)
    ap.add_argument("--final-min", type=float, default=45)
    ap.add_argument("--reanudar", default=None, help="carpeta de una corrida cortada (p. ej. por un corte de luz): sigue desde su historial.json")
    a = ap.parse_args()

    carpeta = Path(a.reanudar) if a.reanudar else Path.home() / ".agrobot" / "aprendizaje" / f"explorador_7h_{time.strftime('%Y%m%d_%H%M')}"
    (carpeta / "aprendizaje").mkdir(parents=True, exist_ok=True)
    log = open(carpeta / "log.txt", "a", buffering=1)

    def decir(*x):
        ln = time.strftime("%H:%M:%S ") + " ".join(str(v) for v in x)
        print(ln, flush=True); log.write(ln + "\n")

    cfg = json.loads(CFG_ROBOT.read_text())
    hoy = leer_hoy(cfg)
    rng = np.random.default_rng(int(time.time()))
    mu, sd = a_u(hoy), np.full(len(CL), 0.25)
    historial, elite_global = [], []
    ctx = get_context("spawn")
    fin_busqueda = a.hasta - a.final_min * 60
    decir("inicio", "HOY", json.dumps(hoy), "procs", a.procs, "pob", a.pob, "seg", a.seg, "fin búsqueda", time.strftime("%H:%M", time.localtime(fin_busqueda)))
    gen = 0
    if a.reanudar and (carpeta / "historial.json").exists():
        # mu y sd no se guardaban: se reconstruyen alrededor del mejor candidato visto, con una dispersión intermedia (0,15)
        historial = json.loads((carpeta / "historial.json").read_text())
        validas = [g for g in historial if not g.get("fallida")]
        elite_global = sorted(((g["mejor"], g["mejor_p"], g["gen"]) for g in validas), key=lambda x: x[0])[:8]
        if elite_global:
            mu, sd = a_u(elite_global[0][1]), np.full(len(CL), 0.15)
        gen = max((g["gen"] for g in historial), default=0)
        decir("REANUDADO tras corte", carpeta, "generaciones previas", gen, "mejor", round(elite_global[0][0], 3) if elite_global else None)
    while True:
        t_gen = time.time()
        if historial and t_gen + historial[-1]["duracion_s"] > fin_busqueda:
            break
        gen += 1
        us = [np.clip(mu + sd * rng.standard_normal(len(CL)), 0, 1) for _ in range(a.pob)]
        cands = [hoy] + [de_u(u) for u in us]
        semillas = [int(rng.integers(1, 10**6))]
        procs = a.procs if (a.procs_antes is None or time.time() >= a.procs_desde) else a.procs_antes
        with ctx.Pool(procs, maxtasksperchild=1) as pool:
            ev = evaluar(pool, cfg, cands, a.seg, semillas, carpeta, f"g{gen}")
        if ev[0]["fallos"] > len(cands) * len(ESCENARIOS) * len(semillas) // 2:
            decir(f"gen {gen}: más de la mitad de los episodios fallaron; no se actualiza la búsqueda")
            historial.append({"gen": gen, "fallida": True, "duracion_s": round(time.time() - t_gen)})
            time.sleep(60)
            continue
        costos = np.array([e["costo"] for e in ev])
        orden = np.argsort(costos[1:]) + 1
        n_el = max(2, a.pob // 3)
        elite = [us[i - 1] for i in orden[:n_el]]
        mu = 0.7 * np.mean(elite, axis=0) + 0.3 * mu
        sd = np.maximum(0.6 * np.std(elite, axis=0) + 0.4 * sd, 0.05)
        for i in orden[:n_el]:
            elite_global.append((float(costos[i]), cands[i], gen))
        elite_global = sorted(elite_global, key=lambda x: x[0])[:8]
        h = {"gen": gen, "semillas": semillas, "hoy": float(costos[0]), "mejor": float(costos[orden[0]]), "media": float(costos[1:].mean()),
             "mejor_p": cands[orden[0]], "mejor_por_esc": resumen_por_esc(ev[orden[0]]), "hoy_por_esc": resumen_por_esc(ev[0]),
             "duracion_s": round(time.time() - t_gen)}
        historial.append(h)
        (carpeta / "historial.json").write_text(json.dumps(historial, indent=1, ensure_ascii=False))
        decir(f"gen {gen:2d}  MEJOR {h['mejor']:7.2f}  HOY {h['hoy']:7.2f}  media {h['media']:7.2f}  ({h['duracion_s']} s)",
              "cam_obs", cands[orden[0]]["camara_obstaculos"], "cam_lidar", cands[orden[0]]["camara_como_lidar"],
              "avance", cands[orden[0]]["avance_libre_cm"], "umbral", cands[orden[0]]["suelo_umbral"])

    # Validación: HOY contra los 3 mejores distintos, semillas nunca vistas, episodios más largos
    top = []
    for c, p, g in elite_global:
        if all(any(abs(p[k] - q[k]) > 1e-6 for k in CL) for _, q, _ in top):
            top.append((c, p, g))
        if len(top) == 3:
            break
    cands = [hoy] + [p for _, p, _ in top]
    semillas = [int(rng.integers(10**6, 2 * 10**6)) for _ in range(2)]
    decir("validación final", len(cands), "candidatos", "semillas", semillas)
    with ctx.Pool(a.procs, maxtasksperchild=1) as pool:
        ev = evaluar(pool, cfg, cands, max(a.seg, 240), semillas, carpeta, "val")
    informe = {"escenarios": ESCENARIOS, "semillas_validacion": semillas, "hoy": hoy,
               "validacion": [{"nombre": "HOY" if i == 0 else f"top{i}", "params": p, "costo": round(e["costo"], 3),
                               "por_esc": resumen_por_esc(e), "gen": None if i == 0 else top[i - 1][2]} for i, (p, e) in enumerate(zip(cands, ev))],
               "generaciones": len(historial)}
    (carpeta / "informe.json").write_text(json.dumps(informe, indent=1, ensure_ascii=False))
    for v in informe["validacion"]:
        decir("VAL", v["nombre"], v["costo"], json.dumps({e: (r["recorrido_m"], r["plantas_pisadas"]) for e, r in v["por_esc"].items()}))
    decir("fin", carpeta)


if __name__ == "__main__":
    main()
