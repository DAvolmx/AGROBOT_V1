"""Reproduce una grabación de campo en MapaScan y dibuja el mapa (PNG) con distintas variantes para COMPARAR cómo se ve:
  actual        parámetros y salida de hoy
  limpio        misma acumulación, salida con crestas finas (sin paredes dobles ni motas)
  limpio+olvido acumulación que olvida más rápido lo contradicho + salida limpia
Uso: python3 tools/render_mapa_scan.py <sesión> [cada=2] [salida_dir]"""
import json, math, sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jetson" / "backend"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import mapa_scan as MS
from analizar_slam_dia import cargar


def replay(v, cada, **par):
    ant = {k: getattr(MS, k) for k in par}
    for k, x in par.items():
        setattr(MS, k, x)
    try:
        m = MS.MapaScan()
        for r in v[::cada]:
            sc = r["lidar_scan"]
            m.agregar(r["env"]["_received_at"], r["pose"], [q["angle_deg"] for q in sc], [q["distance_m"] for q in sc])
        return m
    finally:
        for k, x in ant.items():
            setattr(MS, k, x)


def a_png(m, limpio, escala=4, trazo=None):
    d = m.como_mapa(limpio=limpio)
    w, h = d["width"], d["height"]
    g = np.frombuffer(__import__("base64").b64decode(d["data_b64"]), np.int8).reshape(h, w)
    g = np.flipud(g)
    img = np.full((h, w, 3), 205, np.uint8)
    img[g == 0] = (250, 250, 250)
    img[g == 100] = (45, 55, 50)
    img = cv2.resize(img, (w * escala, h * escala), interpolation=cv2.INTER_NEAREST if not limpio else cv2.INTER_CUBIC)
    return img, d


if __name__ == "__main__":
    s = sys.argv[1]
    cada = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    out = Path(sys.argv[3]) if len(sys.argv) > 3 else Path(__file__).resolve().parents[1] / "docs" / "mapa_bonito"
    out.mkdir(parents=True, exist_ok=True)
    v = cargar(s)
    base = replay(v, cada)
    olvido = replay(v, cada, LOG_MAX=3.5, LOG_LIBRE=-0.55)
    for nombre, m, limpio in (("actual", base, False), ("limpio", base, True), ("limpio_olvido", olvido, True)):
        img, d = a_png(m, limpio)
        cv2.putText(img, nombre, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 200), 2)
        cv2.imwrite(str(out / f"{s}_{nombre}.png"), img)
        occ = int((np.frombuffer(__import__("base64").b64decode(d["data_b64"]), np.int8) == 100).sum())
        print(nombre, "celdas ocupadas", occ, flush=True)
