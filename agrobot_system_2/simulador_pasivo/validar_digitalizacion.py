"""Valida `jetson/backend/digitalizar_mapa.py` contra mapas de verdad conocida (los de mapas_interiores.py).

Para cada mapa: recorre su línea central (o la recta a la meta) con el LIDAR simulado (ruido calibrado), acumula los puntos con una pose
que DERIVA (error de posición a σ_p por √m y de rumbo a σ_h °/√m, parecido al 1-1,5 % medido el 2026-10-22), arma la rejilla de
5 cm, la digitaliza y compara los muros extraídos con los reales:
  · recall    = % de la longitud real a <= 10 cm de algún muro extraído;
  · precisión = % de la longitud extraída a <= 10 cm de un muro real;
  · error     = distancia media (cm) de lo extraído a lo real.
Uso: python3 validar_digitalizacion.py [--semillas 3] [--salida ../docs/digitalizacion]   (escribe resultados.json y un PNG por mapa)"""
import argparse, json, math, sys
from pathlib import Path

import cv2
import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))
import digitalizar_mapa as DM          # noqa: E402
import mapas_interiores as MI          # noqa: E402
import sim_logica_actual as S          # noqa: E402


def recorrido(e, paso=0.1):
    """Puntos (x, y, rumbo) a lo largo de la línea central (ida y vuelta para ver las dos caras de las paredes)."""
    c = list(e.linea_central) if e.linea_central else [(0.0, 0.0), tuple(e.meta_xy or (e.goal_x, 0.0))]
    ida = []
    for (ax, ay), (bx, by) in zip(c, c[1:]):
        L = math.hypot(bx - ax, by - ay)
        n = max(1, int(L / paso))
        for i in range(n):
            ida.append((ax + (bx - ax) * i / n, ay + (by - ay) * i / n, math.atan2(by - ay, bx - ax)))
    ida.append((*c[-1], ida[-1][2]))
    return ida


def muestrear(segs, paso=0.04):
    out = []
    for s in segs:
        L = math.hypot(s[2] - s[0], s[3] - s[1])
        n = max(1, int(L / paso))
        out += [(s[0] + (s[2] - s[0]) * i / n, s[1] + (s[3] - s[1]) * i / n) for i in range(n + 1)]
    return np.array(out) if out else np.zeros((0, 2))


def dist_a_segmentos(P, segs):
    if len(P) == 0 or not segs:
        return np.full(len(P), np.inf)
    d = np.full(len(P), np.inf)
    for (x0, y0, x1, y1) in segs:
        ex, ey = x1 - x0, y1 - y0
        L2 = max(ex * ex + ey * ey, 1e-9)
        u = np.clip(((P[:, 0] - x0) * ex + (P[:, 1] - y0) * ey) / L2, 0, 1)
        d = np.minimum(d, np.hypot(P[:, 0] - (x0 + u * ex), P[:, 1] - (y0 + u * ey)))
    return d


def construir_rejilla(e, sp, sh, rng, res=0.05, margen=0.8):
    xs = [v for s in e.paredes for v in (s[0], s[2])]
    ys = [v for s in e.paredes for v in (s[1], s[3])]
    x0, y0, x1, y1 = min(xs) - margen, min(ys) - margen, max(xs) + margen, max(ys) + margen
    w, h = int((x1 - x0) / res) + 1, int((y1 - y0) / res) + 1
    g = np.full((h, w), -1, np.int8)
    libre = np.zeros((h, w), np.uint8)
    occ = np.zeros((h, w), np.uint16)
    ex = ey = eh = 0.0                                   # deriva acumulada
    trayecto = recorrido(e)
    trayecto = trayecto + trayecto[::-1][1:]
    prev = trayecto[0]
    for (x, y, th) in trayecto:
        paso = math.hypot(x - prev[0], y - prev[1])
        ex += rng.normal(0, sp * math.sqrt(paso)); ey += rng.normal(0, sp * math.sqrt(paso))
        eh += rng.normal(0, math.radians(sh) * math.sqrt(paso))
        prev = (x, y, th)
        psi = th + (math.pi if False else 0.0)
        scan = S.lidar_scan(e, x, y, psi, rng)
        px, py, pth = x + ex, y + ey, psi + eh              # pose ESTIMADA (con deriva)
        a = (int((px - x0) / res), int((y1 - py) / res))
        for r in scan:
            ang = math.radians(r["angle_deg"]) + pth
            hx, hy = px + r["distance_m"] * math.cos(ang), py + r["distance_m"] * math.sin(ang)
            b = (int((hx - x0) / res), int((y1 - hy) / res))
            if 0 <= b[0] < w and 0 <= b[1] < h:
                cv2.line(libre, a, b, 1, 1)
                occ[b[1], b[0]] += 1
    g[libre > 0] = 0
    g[occ >= 2] = 100                                    # una celda es muro si la tocan >= 2 rayos
    return g, res, (x0, y0), (x0, y0, x1, y1)


def evaluar(nombre, e, sp, sh, semilla, tol=0.10, salida=None):
    rng = np.random.default_rng(semilla)
    grid, res, org, _ = construir_rejilla(e, sp, sh, rng)
    r = DM.digitalizar((grid, res, org), None if salida is None else salida / f"{nombre}_s{semilla}")
    ext = r["muros"]
    real = [list(s) for s in e.paredes]
    Pr, Pe = muestrear(real), muestrear(ext)
    # solo se puede exigir lo que el recorrido llegó a ver: se cuenta como real lo que cae a <= 1,5 m del recorrido
    tray = np.array([(x, y) for x, y, _ in recorrido(e)])
    vis = np.array([np.min(np.hypot(tray[:, 0] - p[0], tray[:, 1] - p[1])) <= 1.5 for p in Pr]) if len(Pr) else np.array([])
    dr = dist_a_segmentos(Pr[vis], ext) if len(Pr) else np.array([])
    de = dist_a_segmentos(Pe, real)
    return {"mapa": nombre, "semilla": semilla, "deriva_pos": sp, "deriva_rumbo": sh,
            "recall_pct": round(100 * float((dr <= tol).mean()), 1) if len(dr) else 0.0,
            "precision_pct": round(100 * float((de <= tol).mean()), 1) if len(de) else 0.0,
            "error_medio_cm": round(100 * float(np.mean(np.minimum(de, 0.5))), 1) if len(de) else None,
            "n_muros_extraidos": len(ext), "n_muros_reales": len(real), "largo_extraido_m": r["largo_muros_m"],
            "area_libre_m2": r["area_libre_m2"], "espacios": len(r["espacios"])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--semillas", type=int, default=3)
    ap.add_argument("--salida", default=str(AQUI.parent / "docs" / "digitalizacion"))
    a = ap.parse_args()
    sal = Path(a.salida)
    sal.mkdir(parents=True, exist_ok=True)
    niveles = [("sin_deriva", 0.0, 0.0), ("deriva_1pct", 0.012, 0.5), ("deriva_3pct", 0.03, 1.5)]
    filas = []
    mapas = MI.mapas()
    for nivel, sp, sh in niveles:
        for nombre, e in mapas.items():
            for s in range(a.semillas):
                f = evaluar(nombre, e, sp, sh, s, salida=sal / nivel if s == 0 else None)
                f["nivel"] = nivel
                filas.append(f)
        l = [f for f in filas if f["nivel"] == nivel]
        print(f"{nivel:12s} recall {np.mean([f['recall_pct'] for f in l]):5.1f}%  precisión {np.mean([f['precision_pct'] for f in l]):5.1f}%  "
              f"error {np.mean([f['error_medio_cm'] for f in l if f['error_medio_cm'] is not None]):4.1f} cm", flush=True)
    (sal / "resultados.json").write_text(json.dumps(filas, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
