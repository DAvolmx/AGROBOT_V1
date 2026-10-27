"""Dataset sintético con la CÁMARA REAL del robot y suelos de TUS FOTOS (2026-10-27).

El dataset anterior (generar_dataset.py) asumía cámara a 0,40 m, 15,5° abajo; la del robot (agrobot_config.json: odometria_webcam) está a 0,24 m y a -2,3° con el servo
en 98° (+6,4° a 90°, +17,6° a 76°, +30° a 60°). Medido el 2026-10-27: el modelo entrenado con la geometría vieja daba el 80 % del suelo real como obstáculo.
Aquí: altura/adelante/hfov reales, inclinación = real + extra por servo (los ángulos que usa el robot), 55 % de las sesiones con suelo de parches de fotos reales
(datos/real/suelos.npz), paredes claras con más frecuencia (las reales son blancas/beige) y luz más variada.
Uso: python3 generar_dataset_real.py [--train 160 --val 30 --test 40 --procs 3]   Salida: datos_cam_real/<split>/sesion_XXXX.npz"""
import argparse, json, time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import escena as E
import render as R

AQUI = Path(__file__).resolve().parent
DATOS = AQUI / "datos_cam_real"
BASE = {"train": 7000, "val": 7500, "test": 7800}
CAM_REAL = dict(altura=0.24, inclinacion=1.9, hfov=55.0, adelante=0.245)      # calibrada con la caja de 16 cm a 0,5 y 1 m (2026-10-27); antes -2,3° y 71° (era la diagonal)
EXTRA_SERVO = (0.0, 0.0, 0.0, 6.4, 6.4, 17.6, 30.0)            # 98° es lo más usado en campo (≈ 60 %), luego 86–90, 76 y 60


def _sesion(a):
    split, idx, n = a
    rng = np.random.default_rng(BASE[split] + idx)
    esc = E.escena_aleatoria(rng)
    if rng.random() < 0.55:
        esc.suelo = dict(tipo="foto", color=(120, 115, 110), escala=float(rng.uniform(0.35, 0.8)), parche=int(rng.integers(0, 10 ** 6)))
    if rng.random() < 0.6:
        esc.pared = tuple(int(v) for v in np.clip(rng.normal(215, 14, 3) + rng.normal(0, 6, 3), 150, 250))
    esc.luz["ambiente"] = float(rng.choice([0.35, 0.5, 0.65, 0.8, 0.95]))
    poses = E.poses_sesion(esc, rng, n)
    rgbs, clases, insts, pl, incs = [], [], [], [], []
    for (x, y, psi, cab, bal, _inc) in poses:
        cam = dict(CAM_REAL)
        cam["inclinacion"] = CAM_REAL["inclinacion"] + float(rng.choice(EXTRA_SERVO)) + float(rng.normal(0, 1.0))
        r = R.render(esc, x, y, psi, rng, cam=cam, cabeceo_extra=cab, balanceo=bal)
        rgbs.append(r["rgb"]); clases.append(r["clase"]); insts.append(r["instancia"]); pl.append(r["plagas"]); incs.append(cam["inclinacion"])
    carpeta = DATOS / split
    carpeta.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(carpeta / f"sesion_{idx:04d}.npz", rgb=np.stack(rgbs), clase=np.stack(clases), instancia=np.stack(insts),
                        poses=np.array([p for p in poses], np.float32), plagas=json.dumps(pl),
                        meta=json.dumps({"tipo": esc.nombre, "suelo": esc.suelo["tipo"], "ambiente": esc.luz["ambiente"], "inclinaciones": incs}))
    return idx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", type=int, default=160)
    ap.add_argument("--val", type=int, default=30)
    ap.add_argument("--test", type=int, default=40)
    ap.add_argument("--frames", type=int, default=24)
    ap.add_argument("--procs", type=int, default=3)
    a = ap.parse_args()
    t0 = time.perf_counter()
    tr = [(s, i, a.frames) for s, n in (("train", a.train), ("val", a.val), ("test", a.test)) for i in range(n)]
    with Pool(a.procs) as p:
        for k, _ in enumerate(p.imap_unordered(_sesion, tr, chunksize=1)):
            if (k + 1) % 20 == 0:
                print(f"  {k+1}/{len(tr)} sesiones ({time.perf_counter()-t0:.0f} s)", flush=True)
    print("listo", {s: len(list((DATOS / s).glob("*.npz"))) for s in BASE}, f"{time.perf_counter()-t0:.0f} s")


if __name__ == "__main__":
    main()
