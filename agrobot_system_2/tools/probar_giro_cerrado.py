#!/usr/bin/env python3
"""Compara en el robot REAL el giro de lazo cerrado (giroscopio) con los pulsos de giro de tiempo fijo (2026-10-22).

  python3 tools/probar_giro_cerrado.py --si-mover [--grados 15] [--n 8] [--url http://localhost:8080]

MUEVE el robot: N giros de +grados y N de -grados con cada método, alternados. Hazlo en el SUELO, en un sitio abierto, con la mano en STOP.
Mide el giro real con la pose del SLAM/EKF (theta_deg, el mismo instrumento para los dos métodos). Imprime error medio, desviación,
p90 del error y el porcentaje de giros que casi no giran (<2°). El objetivo del lazo cerrado: error medio ~0 y desviación pequeña.
"""
import argparse
import json
import statistics
import sys
import time
import urllib.request


def get(url, ruta):
    return json.load(urllib.request.urlopen(url + ruta, timeout=6))


def post(url, ruta, cuerpo):
    rq = urllib.request.Request(url + ruta, data=json.dumps(cuerpo).encode(), headers={"Content-Type": "application/json"})
    try:
        return json.load(urllib.request.urlopen(rq, timeout=6))
    except urllib.error.HTTPError as e:
        return {"error": e.read().decode()[:200]}


def theta(url):
    p = get(url, "/api/asistente/estado").get("pose_mapa")
    return None if not p else p["theta_deg"]


def delta(a, b):
    return ((b - a + 180) % 360) - 180


def esperar_quieto(url, s=2.0):
    time.sleep(s)


def medir(url, accion):
    t0 = theta(url)
    if t0 is None:
        sys.exit("No hay pose del SLAM (¿ROS/LIDAR encendidos?).")
    accion()
    esperar_quieto(url, 3.0)
    return delta(t0, theta(url))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8080")
    ap.add_argument("--grados", type=float, default=15.0)
    ap.add_argument("--n", type=int, default=8)
    ap.add_argument("--si-mover", action="store_true", help="confirmas robot en el suelo, espacio libre y mano en STOP")
    a = ap.parse_args()
    if not a.si_mover:
        sys.exit(__doc__)
    res = {"cerrado": [], "pulso": []}
    for i in range(a.n):
        for signo in (+1, -1):
            obj = signo * a.grados
            r = {}

            def cerrado():
                nonlocal r
                r = post(a.url, "/api/giro_cerrado", {"grados": obj})
                if "error" in r:
                    sys.exit("giro cerrado rechazado: " + str(r))
                time.sleep(3.0)
            d = medir(a.url, cerrado)
            res["cerrado"].append((obj, d))
            d2 = medir(a.url, lambda: post(a.url, "/api/drive", {"accion": "izquierda" if signo > 0 else "derecha", "escala": 0.5, "maniobra": False}))
            res["pulso"].append((obj, d2))
            print(f"{i + 1}.{'+' if signo > 0 else '-'} objetivo {obj:+.0f}° · cerrado {d:+6.1f}° · pulso {d2:+6.1f}°", flush=True)
    for nombre, L in res.items():
        err = [m - o for o, m in L]
        mov = [abs(m) for o, m in L]
        err_abs = sorted(abs(e) for e in err)
        print(f"\n{nombre:8s} n={len(L)} · giro medio con signo del objetivo {statistics.mean([m * (1 if o > 0 else -1) for o, m in L]):+.1f}° · error medio {statistics.mean(err):+.1f}° · "
              f"desv {statistics.pstdev(err):.1f}° · p90 |error| {err_abs[int(0.9 * (len(err_abs) - 1))]:.1f}° · sin girar (<2°): {100 * sum(1 for x in mov if x < 2) / len(mov):.0f} %")
    print("\n(en el giro de PULSOS el 'objetivo' es solo la dirección: no tiene ángulo pedido)")


if __name__ == "__main__":
    main()
