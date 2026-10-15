"""Dataset de DETECCIÓN DE PLAGAS sintético (640x480), separado del de navegación.

Plagas sintéticas: pulgón (grupo de puntos), oruga (cuerpo a segmentos), escarabajo (óvalo oscuro).
Son SPRITES SENCILLOS pegados sobre la copa de las plantas, con señuelos (manchas de hoja y
motas de tierra) sin etiqueta.

LÍMITE FUNDAMENTAL: un detector entrenado con esto aprende a encontrar ESTOS sprites. No
demuestra nada sobre plagas reales, que se ven muy distintas. Sirve para (1) validar la cadena
"detección -> parada y escaneo" del robot y (2) medir cuánto depende la detección de la
distancia, el tamaño y el sesgo de cámara. El detector real de plagas sigue siendo el entrenado
con IP102/datos reales (`agrobot_plagas_det.pt`, `ip102_pest_yolo11s.pt`).

Uso: python3 generar_plagas.py [--train 90 --val 15 --test 24 --test_nuevo 12 --frames 20 --procs 6]
"""
import argparse
import math
import time
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

import escena as E
import render as R

AQUI = Path(__file__).resolve().parent
SALIDA = AQUI / "yolo_plagas"
BASE = {"train": 200_000, "val": 250_000, "test": 290_000, "test_nuevo": 330_000}


def _sesion(args):
    split, idx, n, nuevo = args
    rng = np.random.default_rng(BASE[split] + idx)
    esc = E.escena_aleatoria(rng, tipo=str(rng.choice(["vivero", "sala"])), con_plagas=True)
    # sin plagas el dataset no sirve: garantizar plantas con bichos
    for c in esc.cilindros:
        if c.copa_r > 0 and not c.plagas and rng.random() < 0.7:
            for _ in range(int(rng.integers(1, 4))):
                c.plagas.append((float(rng.uniform(0, 6.28)), float(rng.uniform(0.3, 0.95)),
                                 str(rng.choice(R.TIPOS_PLAGA))))
    if nuevo:                                    # dominio desplazado: otro suelo y otra luz
        esc.suelo = dict(tipo="baldosa", color=(90, 130, 90), color2=(70, 105, 75), escala=0.55, junta=0.02)
        esc.luz["ambiente"] = float(rng.choice([0.35, 0.95]))
    plantas = [c for c in esc.cilindros if c.copa_r > 0]
    (SALIDA / "images" / split).mkdir(parents=True, exist_ok=True)
    (SALIDA / "labels" / split).mkdir(parents=True, exist_ok=True)
    n_img = n_box = 0
    intentos = 0
    while n_img < n and plantas and intentos < n * 12:
        intentos += 1
        pl = plantas[int(rng.integers(len(plantas)))]
        dist = float(rng.uniform(0.5, 1.5))
        brg = float(rng.uniform(0, 2 * math.pi))
        x, y = pl.x + dist * math.cos(brg), pl.y + dist * math.sin(brg)
        if not E._libre(x, y, esc.objetos(), 0.2) or not (esc.limites[0] + 0.2 < x < esc.limites[2] - 0.2
                                                         and esc.limites[1] + 0.2 < y < esc.limites[3] - 0.2):
            continue
        psi = math.atan2(pl.y - y, pl.x - x) + float(rng.normal(0, 0.12))
        cam = dict(R.CAMARA, inclinacion=float(rng.uniform(8, 28)))
        r = R.render(esc, x, y, psi, rng, ancho=640, alto=480, cam=cam,
                     cabeceo_extra=float(rng.normal(0, 1.0)), balanceo=float(rng.normal(0, 0.01)))
        cajas = [(max(0, a), max(0, b), min(640, c), min(480, d), t) for (a, b, c, d, t) in r["plagas"]
                 if min(640, c) - max(0, a) >= 4 and min(480, d) - max(0, b) >= 4]
        if not cajas and rng.random() > 0.25:    # ~25 % de negativos (sin plaga visible)
            continue
        nombre = f"{split}_{idx:04d}_{n_img:02d}"
        cv2.imwrite(str(SALIDA / "images" / split / f"{nombre}.jpg"), r["rgb"][..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(SALIDA / "labels" / split / f"{nombre}.txt", "w") as fh:
            for (x0, y0, x1, y1, t) in cajas:
                fh.write(f"{t} {(x0+x1)/2/640:.6f} {(y0+y1)/2/480:.6f} {(x1-x0)/640:.6f} {(y1-y0)/480:.6f}\n")
        n_img += 1
        n_box += len(cajas)
    return split, n_img, n_box


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", type=int, default=90)
    ap.add_argument("--val", type=int, default=15)
    ap.add_argument("--test", type=int, default=24)
    ap.add_argument("--test_nuevo", type=int, default=12)
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--procs", type=int, default=5)
    a = ap.parse_args()
    t0 = time.perf_counter()
    trabajos = [(s, i, a.frames, s == "test_nuevo") for s, n in
                (("train", a.train), ("val", a.val), ("test", a.test), ("test_nuevo", a.test_nuevo)) for i in range(n)]
    tot = {}
    with Pool(a.procs) as pool:
        for k, (s, ni, nb) in enumerate(pool.imap_unordered(_sesion, trabajos)):
            t = tot.setdefault(s, [0, 0])
            t[0] += ni
            t[1] += nb
            if (k + 1) % 20 == 0:
                print(f"  {k+1}/{len(trabajos)} ({time.perf_counter()-t0:.0f} s)", flush=True)
    (SALIDA / "datos.yaml").write_text(
        f"path: {SALIDA}\ntrain: images/train\nval: images/val\ntest: images/test\n"
        "names:\n  0: pulgon\n  1: oruga\n  2: escarabajo\n")
    (SALIDA / "datos_nuevo.yaml").write_text(
        f"path: {SALIDA}\ntrain: images/train\nval: images/test_nuevo\n"
        "names:\n  0: pulgon\n  1: oruga\n  2: escarabajo\n")
    print({k: f"{v[0]} imágenes, {v[1]} cajas" for k, v in tot.items()}, f"{time.perf_counter()-t0:.0f} s")


if __name__ == "__main__":
    main()
