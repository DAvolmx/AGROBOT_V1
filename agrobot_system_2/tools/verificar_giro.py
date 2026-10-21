#!/usr/bin/env python3
"""Verifica con el robot REAL que «izquierda» gira a la izquierda y «derecha» a la derecha (control.invertir_giro).

  python3 tools/verificar_giro.py --si-mover [--escala 0.2] [--url http://localhost:8080]

MUEVE el robot: dos pulsos cortos de giro (izquierda y derecha). Hazlo en el SUELO, en un sitio abierto y con la
mano en el botón STOP (o la tecla Espacio). Mide el rumbo del SLAM/EKF (pose_mapa.theta_deg: aumenta al girar a
la IZQUIERDA, convención ROS) y el giroscopio (gyroZ: positivo = izquierda) durante cada pulso.

  PASS  los dos pulsos giran del lado pedido.
  FALLA ambos salen cruzados -> en el dashboard/config: control.invertir_giro = (lo contrario de lo actual).
"""
import argparse
import json
import sys
import time
import urllib.request


def get(url, ruta):
    return json.load(urllib.request.urlopen(url + ruta, timeout=6))


def post(url, ruta, body):
    rq = urllib.request.Request(url + ruta, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    try:
        return json.load(urllib.request.urlopen(rq, timeout=6))
    except urllib.error.HTTPError as e:
        return {"error": e.read().decode()[:200], "status": e.code}


def pulso(url, accion, escala):
    a = get(url, "/api/asistente/estado")
    th0 = a["pose_mapa"]["theta_deg"] if a.get("pose_mapa") else None
    r = post(url, "/api/drive", {"accion": accion, "escala": escala, "maniobra": False})
    if r.get("error"):
        sys.exit(f"No se pudo mandar '{accion}': {r}")
    gz = []
    t0 = time.time()
    while time.time() - t0 < 2.5:
        i = get(url, "/api/asistente/estado")["imu"]
        if i.get("fresco") and i.get("gyroZ") is not None:
            gz.append(float(i["gyroZ"]))
        time.sleep(0.15)
    time.sleep(1.0)
    b = get(url, "/api/asistente/estado")
    th1 = b["pose_mapa"]["theta_deg"] if b.get("pose_mapa") else None
    d = None if th0 is None or th1 is None else ((th1 - th0 + 180) % 360) - 180
    pico = max(gz, key=abs) if gz else None
    return d, pico


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8080")
    ap.add_argument("--escala", type=float, default=0.2)
    ap.add_argument("--si-mover", action="store_true", help="confirmas que el robot está en el suelo y que puedes pararlo")
    a = ap.parse_args()
    if not a.si_mover:
        sys.exit(__doc__)
    inv = get(a.url, "/api/params")["control"].get("invertir_giro")
    print(f"control.invertir_giro = {inv}\n")
    ok = []
    for accion, signo in (("izquierda", +1), ("derecha", -1)):
        d, pico = pulso(a.url, accion, a.escala)
        por_pose = None if d is None else (d * signo > 1.0)
        por_giro = None if pico is None else (pico * signo > 3.0)
        print(f"{accion:10s} Δrumbo SLAM = {'—' if d is None else f'{d:+.1f}°'} · pico gyroZ = {'—' if pico is None else f'{pico:+.1f} °/s'}"
              f"  -> {'lado correcto' if (por_pose or por_giro) and por_pose is not False and por_giro is not False else 'LADO CONTRARIO o sin giro medible'}")
        ok.append(bool((por_pose or por_giro) and por_pose is not False and por_giro is not False))
        time.sleep(1.5)
    if all(ok):
        print("\nPASS: izquierda gira a la izquierda y derecha a la derecha.")
        return 0
    print("\nFALLA: revisa. Si salen cruzados, pon control.invertir_giro = " + str(not inv).lower() + " (Parámetros → Control o agrobot_config.json) y repite.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
