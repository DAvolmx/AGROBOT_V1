#!/usr/bin/env python3
"""Prueba en el robot REAL de ir en LÍNEA RECTA: pulsos rectos sin corrección contra pulsos rectos con rumbo fusionado (2026-10-22).

  python3 tools/probar_recta.py --si-mover [--pulsos 12] [--rondas 3] [--zona-muerta 4] [--escala 0.5] [--url http://localhost:8080]

MUEVE el robot: en cada ronda avanza `pulsos` pulsos rectos primero SIN corrección (mide hacia dónde cae) y luego CON corrección de rumbo
(rumbo_fusion.CorrectorRumbo: zona muerta, histéresis y sesgo por pulso aprendido), alternando. Sitio abierto y plano, 4 m libres delante,
mano en STOP. Entre rondas el robot NO vuelve solo: reposiciónalo (o usa --atras para que retroceda los mismos pulsos).
El instrumento es la pose del SLAM/EKF (x, y, theta_deg); el rumbo deseado es el de arranque de cada tramo. Imprime desvío angular
final, desvío lateral final (m, respecto a la recta de arranque), pulsos de corrección dados y el sesgo aprendido (° por pulso, + = izquierda).
"""
import argparse
import json
import math
import statistics
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "jetson" / "backend"))
import rumbo_fusion as RF


def get(url, ruta):
    return json.load(urllib.request.urlopen(url + ruta, timeout=6))


def post(url, ruta, cuerpo):
    rq = urllib.request.Request(url + ruta, data=json.dumps(cuerpo).encode(), headers={"Content-Type": "application/json"})
    try:
        return json.load(urllib.request.urlopen(rq, timeout=6))
    except urllib.error.HTTPError as e:
        return {"error": e.read().decode()[:200]}


def pose(url):
    p = get(url, "/api/asistente/estado").get("pose_mapa")
    if not p:
        sys.exit("No hay pose del SLAM (¿ROS/LIDAR encendidos?).")
    return p["x"], p["y"], math.radians(p["theta_deg"])


def pulso(url, accion, escala):
    r = post(url, "/api/drive", {"accion": accion, "escala": escala, "maniobra": False})
    if "error" in r:
        sys.exit("el backend rechazó el pulso: " + str(r))
    time.sleep(1.6)                       # pulso + asentarse; la pose del SLAM llega a ~1 Hz


def tramo(url, n, escala, con_rumbo, zm):
    x0, y0, th0 = pose(url)
    corr = RF.CorrectorRumbo(zm) if con_rumbo else None
    dados = 0
    for _ in range(n):
        x, y, th = pose(url)
        e = RF._ang(th - th0)             # + = apunta a la izquierda del rumbo de arranque
        if corr is not None:
            a = corr.decidir(e)
            if a:
                pulso(url, a, escala)
                dados += 1
                x, y, th = pose(url)
                e = RF._ang(th - th0)
            corr.inicio_pulso_recto(e)
        pulso(url, "adelante", escala)
        if corr is not None:
            corr.fin_pulso_recto(RF._ang(pose(url)[2] - th0))
    x, y, th = pose(url)
    dx, dy = x - x0, y - y0
    avance = dx * math.cos(th0) + dy * math.sin(th0)
    lateral = -dx * math.sin(th0) + dy * math.cos(th0)         # + = a la izquierda de la recta de arranque
    return {"giro_deg": math.degrees(RF._ang(th - th0)), "lateral_m": lateral, "avance_m": avance, "correcciones": dados,
            "sesgo_deg": math.degrees(corr.sesgo) if corr else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8080")
    ap.add_argument("--pulsos", type=int, default=12)
    ap.add_argument("--rondas", type=int, default=3)
    ap.add_argument("--zona-muerta", type=float, default=4.0)
    ap.add_argument("--escala", type=float, default=0.5)
    ap.add_argument("--atras", action="store_true", help="tras cada tramo retrocede el mismo número de pulsos para volver al sitio")
    ap.add_argument("--si-mover", action="store_true", help="confirmas robot en el suelo, 4 m libres y mano en STOP")
    a = ap.parse_args()
    if not a.si_mover:
        sys.exit(__doc__)
    res = {"sin_corregir": [], "con_rumbo": []}
    for i in range(a.rondas):
        for nombre, con in (("sin_corregir", False), ("con_rumbo", True)):
            r = tramo(a.url, a.pulsos, a.escala, con, a.zona_muerta)
            res[nombre].append(r)
            sb = "" if r["sesgo_deg"] is None else f" · sesgo aprendido {r['sesgo_deg']:+.1f}°/pulso"
            print(f"{i + 1}. {nombre:12s} avance {r['avance_m']:.2f} m · lateral {r['lateral_m']:+.2f} m · giro {r['giro_deg']:+.1f}° · "
                  f"correcciones {r['correcciones']}{sb}", flush=True)
            if a.atras:
                for _ in range(a.pulsos):
                    pulso(a.url, "atras", a.escala)
    for nombre, L in res.items():
        lat = [abs(r["lateral_m"]) for r in L]
        gir = [abs(r["giro_deg"]) for r in L]
        print(f"\n{nombre:12s} n={len(L)} · |lateral| medio {statistics.mean(lat):.2f} m · |giro| medio {statistics.mean(gir):.1f}° · "
              f"avance medio {statistics.mean(r['avance_m'] for r in L):.2f} m")
    print("\nSi con_rumbo no mejora el lateral y el giro, NO lo actives en el explorador: pásame estas líneas.")


if __name__ == "__main__":
    main()
