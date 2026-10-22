#!/usr/bin/env python3
"""Elige exposición/enfoque de la cámara USB midiendo brillo y nitidez (2026-10-22). Para correr con el BACKEND PARADO (la cámara es de un solo dueño).

  sudo systemctl stop agrobot-backend.service
  python3 tools/ajustar_camara.py [--dispositivo 0] [--n 12] [--aplicar]

Apunta la cámara a la escena de siempre (suelo + pared, como en las pruebas) y déjala quieta. Prueba combinaciones de auto-exposición,
ganancia, brillo y enfoque; por cada una toma N fotogramas y mide brillo medio (objetivo ~110/255), % quemado/negro y nitidez (varianza del
Laplaciano, más alta = más nítida). Imprime la tabla y la mejor. Con --aplicar guarda la mejor en hardware.camara_ajustes de agrobot_config.json
(luego arranca el backend). Las propiedades que el driver no acepta se omiten.
"""
import argparse
import itertools
import json
import time
from pathlib import Path

import cv2
import numpy as np

CFG = Path(__file__).resolve().parent.parent / "jetson" / "backend" / "agrobot_config.json"


def medir(cap, n):
    b, n_, q, s = [], [], [], []
    for _ in range(n + 3):
        ok, f = cap.read()
        if not ok:
            continue
        if _ < 3:
            continue
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        b.append(g.mean()); q.append((g >= 250).mean()); n_.append((g <= 8).mean()); s.append(cv2.Laplacian(g, cv2.CV_64F).var())
    return (float(np.mean(b)), 100 * float(np.mean(q)), 100 * float(np.mean(n_)), float(np.median(s))) if b else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dispositivo", type=int, default=0)
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--aplicar", action="store_true")
    a = ap.parse_args()
    cap = cv2.VideoCapture(a.dispositivo, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit("No abre la cámara (¿backend parado y cámara conectada?).")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480); cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    rejilla = [{}] + [{"autofocus": af, "brightness": br, "gain": g, "backlight": bl}
                      for af, br, g, bl in itertools.product((1, 0), (0, 32, 64), (0, 64), (0, 1))]
    res = []
    for aj in rejilla:
        props = {"autofocus": cv2.CAP_PROP_AUTOFOCUS, "brightness": cv2.CAP_PROP_BRIGHTNESS, "gain": cv2.CAP_PROP_GAIN, "backlight": cv2.CAP_PROP_BACKLIGHT}
        for k, v in aj.items():
            cap.set(props[k], float(v))
        if aj.get("autofocus") == 0:
            cap.set(cv2.CAP_PROP_FOCUS, 40.0)
        time.sleep(0.6)
        m = medir(cap, a.n)
        if m is None:
            continue
        brillo, quem, neg, nit = m
        # puntuación: brillo cerca de 110, poco quemado/negro y la mayor nitidez posible
        pts = nit - 2.0 * abs(brillo - 110) - 30 * quem - 15 * neg
        res.append((pts, aj, m))
        print(f"{json.dumps(aj):80s} brillo {brillo:5.0f}  quemado {quem:4.1f}%  negro {neg:4.1f}%  nitidez {nit:6.0f}  puntos {pts:7.1f}", flush=True)
    cap.release()
    res.sort(key=lambda r: -r[0])
    print("\nMEJOR:", json.dumps(res[0][1]), "->", [round(x, 1) for x in res[0][2]])
    if a.aplicar:
        c = json.loads(CFG.read_text())
        c.setdefault("hardware", {})["camara_ajustes"] = res[0][1]
        CFG.write_text(json.dumps(c, indent=2, ensure_ascii=False))
        print("guardado en", CFG)


if __name__ == "__main__":
    main()
