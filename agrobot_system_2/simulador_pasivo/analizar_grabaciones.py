"""Analiza las grabaciones de campo (variables.jsonl a 5 Hz + eventos/poses SLAM del lote que las solapa) y genera tablas y gráficas:
trayectoria, hueco delantero en el tiempo, contactos probables, desplazamiento por tipo de pulso, intervenciones y resultado de misiones.
Uso: python3 analizar_grabaciones.py [--campo <carpeta> ...]   (por defecto las de hoy)   Salida: ~/.agrobot/informes/analisis_<fecha>/"""
import argparse, bisect, glob, json, math, os, time
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HOME = Path.home() / ".agrobot"
LIDAR_ADELANTE = 0.185


def wrap(a):
    return (a + 180) % 360 - 180


def leer(path):
    out = []
    for l in open(path):
        try:
            out.append(json.loads(l))
        except ValueError:
            pass
    return out


def lote_que_solapa(t0, t1):
    mejor = None
    for d in sorted(glob.glob(str(HOME / "datos" / "2026*"))):
        f = Path(d) / "eventos.jsonl"
        if not f.exists():
            continue
        ts = []
        with open(f) as fh:
            for i, l in enumerate(fh):
                if i % 200 == 0:
                    try:
                        ts.append(json.loads(l)["t_unix"])
                    except Exception:
                        pass
        if ts and ts[0] <= t1 and ts[-1] >= t0:
            sol = min(ts[-1], t1) - max(ts[0], t0)
            if mejor is None or sol > mejor[0]:
                mejor = (sol, Path(d))
    return mejor[1] if mejor else None


def analizar(campo: Path, salida: Path):
    filas = [r for r in leer(campo / "variables.jsonl") if r.get("t_unix") and r.get("lidar_scan") is not None]
    if len(filas) < 50:
        return None
    t0, t1 = filas[0]["t_unix"], filas[-1]["t_unix"]
    lote = lote_que_solapa(t0, t1)
    ev = leer(lote / "eventos.jsonl") if lote else []
    slam = [(e["t_unix"], e["datos"]["pose"]["x"], e["datos"]["pose"]["y"], e["datos"]["pose"]["theta_deg"]) for e in ev if e["tipo"] == "scan_mapa"]
    ts = [s[0] for s in slam]
    res = {"sesion": campo.name, "lote": lote.name if lote else None, "duracion_s": round(t1 - t0), "muestras": len(filas)}
    # --- hueco delantero y contactos probables
    tt, gap = [], []
    for r in filas:
        sc = r["lidar_scan"]
        fr = [p["distance_m"] for p in sc if abs(wrap(p["angle_deg"])) <= 22 and p["distance_m"] > 0.05]
        tt.append(r["t_unix"] - t0)
        gap.append((min(fr) - LIDAR_ADELANTE) if fr else np.nan)
    gap = np.array(gap)
    # hueco mínimo del CUERPO (rectángulo del robot) a cualquier punto del scan; lo que cae dentro del rectángulo +2 cm se toma por chasis,
    # así que 2,0 cm es el suelo de la medida: «≤ 2,05 cm» = tocando o casi
    cer = []
    for r in filas:
        c = 1e9
        for p in r["lidar_scan"]:
            d = p["distance_m"]
            if d < 0.05:
                continue
            ang = math.radians(p["angle_deg"])
            x, y = d * math.cos(ang), d * math.sin(ang)
            if abs(x) <= (LIDAR_ADELANTE if x >= 0 else 0.305) + 0.02 and abs(y) <= 0.19 + 0.02:
                continue
            ex = max(0.0, (x - LIDAR_ADELANTE) if x >= 0 else (-x - 0.305))
            c = min(c, math.hypot(ex, max(0.0, abs(y) - 0.19)))
        cer.append(c * 100)
    cer = np.array(cer)
    toque = cer <= 2.05
    idx = np.where(toque)[0]                                          # episodios: se funden los toques separados < 1,5 s
    res["contactos_probables"] = int(0 if len(idx) == 0 else 1 + np.sum(np.diff(np.array(tt)[idx]) > 1.5))
    res["toques_por_min"] = round(res["contactos_probables"] / max(1.0, (t1 - t0) / 60.0), 2)
    res["hueco_min_cm"] = round(float(np.min(cer)), 1)
    res["pct_menos_5cm"] = round(100 * float(np.mean(cer < 5)), 1)
    res["pct_menos_12cm"] = round(100 * float(np.mean(cer < 12)), 1)
    res["hueco_mediano_cm"] = round(float(np.median(cer)), 1)
    res["_cercas"] = (tt, cer.tolist())
    # --- recorrido
    if slam:
        d = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(slam, slam[1:]) if b[0] - a[0] < 1.0)
        res["recorrido_m"] = round(d, 1)
    # --- pulsos por tipo
    por = defaultdict(list)
    cur = None
    runs = []
    for r in filas:
        v = r.get("velocities") or {}
        vp, wp = v.get("v_pedida") or 0, v.get("omega_pedida") or 0
        k = None
        if abs(wp) > 0.1 and vp > 0.05:
            k = "arco_izq" if wp > 0 else "arco_der"
        elif vp > 0.05:
            k = "adelante"
        elif vp < -0.05:
            k = "atras"
        if k and cur and cur[0] == k:
            cur[1].append(r["t_unix"])
        else:
            if cur:
                runs.append(cur)
            cur = (k, [r["t_unix"]]) if k else None
    if cur:
        runs.append(cur)
    if slam:
        for k, l in runs:
            ta, tb = l[0], l[-1] + 0.5
            i, j = (min(max(bisect.bisect_left(ts, t), 0), len(slam) - 1) for t in (ta, tb))
            A, B = slam[i], slam[j]
            if abs(A[0] - ta) > 1 or abs(B[0] - tb) > 1:
                continue
            th = math.radians(A[3])
            por[k].append((wrap(B[3] - A[3]), ((B[1] - A[1]) * math.cos(th) + (B[2] - A[2]) * math.sin(th)) * 100))
    res["pulsos"] = {k: {"n": len(v), "giro_deg": round(float(np.mean([x[0] for x in v])), 1), "avance_cm": round(float(np.mean([x[1] for x in v])), 1)}
                     for k, v in por.items() if v}
    # --- misión
    mis = [e for e in ev if e["tipo"] == "mision_plantas"]
    c = Counter(e["datos"].get("tipo") for e in mis)
    fallos = Counter(e["datos"].get("motivo") for e in mis if e["datos"].get("tipo") == "inspeccion_fallida")
    res["mision"] = {"inicios": c.get("mision_inicio", 0), "detectadas": c.get("planta_detectada", 0), "inspeccionadas": c.get("planta_inspeccionada", 0),
                     "fallidas": c.get("inspeccion_fallida", 0), "motivos_fallo": dict(fallos)}
    res["eventos"] = dict(Counter(e["tipo"] for e in ev))
    # --- gráficas
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
    if slam:
        xs, ys, tc = [s[1] for s in slam], [s[2] for s in slam], [s[0] - t0 for s in slam]
        sc = ax[0].scatter(xs, ys, c=tc, s=2, cmap="viridis")
        ax[0].set_aspect("equal"); ax[0].set_title("Trayectoria (SLAM, marco mapa)"); ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)")
        plt.colorbar(sc, ax=ax[0], label="s desde el inicio")
    ax[1].plot(tt, cer, lw=0.8)
    ax[1].axhline(2.05, color="r", ls="--", lw=0.8, label="tocando o casi (≤ 2 cm)")
    ax[1].axhline(5, color="orange", ls=":", lw=0.8, label="muy cerca (< 5 cm)")
    ax[1].set_ylim(0, 80); ax[1].set_title("Hueco mínimo del cuerpo a cualquier objeto (LIDAR)"); ax[1].set_xlabel("s"); ax[1].set_ylabel("cm"); ax[1].legend()
    fig.suptitle(f"{campo.name} · {res['duracion_s']} s · {res.get('recorrido_m', '?')} m · {res['contactos_probables']} toques probables · {res['pct_menos_5cm']} % del tiempo a < 5 cm")
    res.pop("_cercas", None)
    fig.tight_layout(); fig.savefig(salida / f"{campo.name}_trayectoria.png", dpi=110); plt.close(fig)
    return res, por


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--campo", nargs="*")
    a = ap.parse_args()
    campos = [Path(c) for c in a.campo] if a.campo else [Path(p) for p in sorted(glob.glob(str(HOME / "campo" / "20261023_*[0-9a-f]"))) if os.path.isdir(p)]
    salida = HOME / "informes" / ("analisis_" + time.strftime("%Y%m%d_%H%M"))
    salida.mkdir(parents=True, exist_ok=True)
    todo, pulsos = [], defaultdict(list)
    for c in campos:
        r = analizar(c, salida)
        if r:
            todo.append(r[0])
            for k, v in r[1].items():
                pulsos[(c.name, k)] = v
            print(json.dumps(r[0], ensure_ascii=False)[:400], flush=True)
    (salida / "analisis.json").write_text(json.dumps(todo, indent=1, ensure_ascii=False))
    # desplazamiento por tipo de pulso (todas las sesiones juntas)
    acum = defaultdict(list)
    for (_, k), v in pulsos.items():
        acum[k] += v
    if acum:
        fig, ax = plt.subplots(1, 2, figsize=(11, 4))
        nombres = sorted(acum)
        ax[0].boxplot([[x[1] for x in acum[k]] for k in nombres], tick_labels=nombres); ax[0].axhline(0, color="gray", lw=0.6)
        ax[0].set_title("Avance por pulso (cm, + adelante)"); ax[0].tick_params(axis="x", rotation=20)
        ax[1].boxplot([[x[0] for x in acum[k]] for k in nombres], tick_labels=nombres); ax[1].axhline(0, color="gray", lw=0.6)
        ax[1].set_title("Giro por pulso (°, + izquierda)"); ax[1].tick_params(axis="x", rotation=20)
        fig.tight_layout(); fig.savefig(salida / "pulsos.png", dpi=110); plt.close(fig)
    print("->", salida)


if __name__ == "__main__":
    main()
