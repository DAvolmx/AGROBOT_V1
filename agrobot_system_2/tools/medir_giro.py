#!/usr/bin/env python3
"""Mide cuánto se DESPLAZA el robot al girar a la izquierda y a la derecha (calibración de `comp_neg` / `comp_pos`).

Usa la pose del SLAM (marco mapa, fiable) de ~/.agrobot/datos/<lote>/eventos.jsonl y las órdenes (omega_cmd) de la grabación de campo
~/.agrobot/campo/<sesion>/variables.jsonl, alineadas por t_unix. Un «toque de giro» = una racha de omega_cmd != 0 con v ~ 0.

  python3 tools/medir_giro.py [--campo <carpeta>] [--datos <carpeta>]      (por defecto la grabación y el lote más recientes)

Salida: por sentido, grados girados y centímetros avanzados (+ adelante / - atrás) POR TOQUE y por grado, y qué hacer con `comp_neg`
(giro a la izquierda: omega_cmd < 0 en este robot) y `comp_pos` (derecha). Objetivo: avance medio ≈ 0 cm sin perder giro.
"""
import argparse, bisect, glob, json, math, os, sys
from collections import defaultdict


def wrap(a):
    return (a + 180) % 360 - 180


def ultimo(patron):
    c = sorted(glob.glob(os.path.expanduser(patron)))
    return c[-1] if c else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--campo", default=None)
    ap.add_argument("--datos", default=None)
    a = ap.parse_args()
    campo = a.campo or ultimo("~/.agrobot/campo/2026*[0-9a-f]")
    campo = campo if campo and os.path.isdir(campo) else ultimo("~/.agrobot/campo/2026*")
    datos = a.datos or ultimo("~/.agrobot/datos/2026*")
    print("grabación de campo:", campo, "\nlote SLAM:", datos)
    slam = []
    for l in open(os.path.join(datos, "eventos.jsonl")):
        try:
            e = json.loads(l)
        except ValueError:
            continue
        if e.get("tipo") == "scan_mapa":
            p = e["datos"]["pose"]
            slam.append((e["t_unix"], p["x"], p["y"], p["theta_deg"]))
    if len(slam) < 50:
        sys.exit("no hay suficientes poses del SLAM en ese lote")
    ts = [s[0] for s in slam]
    rows = []
    for l in open(os.path.join(campo, "variables.jsonl")):
        try:
            r = json.loads(l)
        except ValueError:
            continue
        if (r.get("actuator_status") or {}).get("_received_at") and r.get("t_unix"):
            rows.append(r)
    cur, segs = None, []
    for r in rows:
        st = r["actuator_status"]
        w, v = st.get("omega_cmd") or 0, st.get("v_cmd") or 0
        tag = "izquierda (omega<0)" if w < -0.1 else "derecha (omega>0)" if w > 0.1 else None
        if tag and abs(v) < 0.05:
            if cur and cur[0] == tag:
                cur[1].append(r["t_unix"])
            else:
                if cur:
                    segs.append(cur)
                cur = (tag, [r["t_unix"]])
        else:
            if cur:
                segs.append(cur)
                cur = None
    if cur:
        segs.append(cur)
    res = defaultdict(list)
    for tag, l in segs:
        ta, tb = l[0], l[-1] + 0.4
        if tb - ta < 0.3:
            continue
        i, j = (min(max(bisect.bisect_left(ts, t), 0), len(slam) - 1) for t in (ta, tb))
        A, B = slam[i], slam[j]
        if abs(A[0] - ta) > 1 or abs(B[0] - tb) > 1:
            continue
        th = math.radians(A[3])
        dx, dy = B[1] - A[1], B[2] - A[2]
        res[tag].append((abs(wrap(B[3] - A[3])), (dx * math.cos(th) + dy * math.sin(th)) * 100))
    if not res:
        sys.exit("no se encontraron toques de giro en esa grabación")
    medias = {}
    for tag, l in res.items():
        n = len(l)
        g = sum(x[0] for x in l) / n
        d = sum(x[1] for x in l) / n
        medias[tag] = (g, d)
        print(f"{tag:22s} {n:3d} toques · gira {g:4.1f}° · avanza {d:+5.1f} cm por toque · {d / max(g, 0.1):+.2f} cm/°")
    print()
    for tag, (g, d) in medias.items():
        clave = "comp_neg" if tag.startswith("izq") else "comp_pos"
        if abs(d) < 1.5:
            print(f"{clave}: bien (|avance| < 1,5 cm por toque).")
        elif d < 0:
            print(f"{clave}: retrocede {-d:.1f} cm por toque -> SUBIR {clave} unos {min(25, int(-d * 2.5))} puntos (más ruedas que retroceden apagadas).")
        else:
            print(f"{clave}: avanza {d:.1f} cm por toque de más -> BAJAR {clave} (o 0); si ya es 0 no hay nada que hacer: avanzar al girar es inocuo.")


if __name__ == "__main__":
    main()
