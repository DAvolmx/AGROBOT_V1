"""Genera el dataset sintético de interiores, repartido POR SESIÓN.

Uso:  python3 generar_dataset.py [--train 160] [--val 30] [--test 40] [--frames 24] [--procs 6]

Salida: percepcion_sintetica/datos/<split>/sesion_XXXX.npz  con
    rgb (n,H,W,3) uint8 · clase (n,H,W) uint8 · instancia (n,H,W) int16
    poses (n,6) x,y,psi,cabeceo,balanceo,inclinacion · plagas (lista de cajas por fotograma)
    meta: tipo de escena, suelo, luz
Semillas: cada split usa un rango de semillas DISTINTO, y una sesión nunca se reparte entre
splits. `test` no se mira hasta la evaluación final.

  +  `--dominio_nuevo` genera un test DESPLAZADO (otro suelo/luz/escala) para medir generalización.
"""
import argparse
import json
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import escena as E
import render as R

AQUI = Path(__file__).resolve().parent
DATOS = AQUI / "datos"
BASE_SEMILLA = {"train": 1_000, "val": 50_000, "test": 90_000, "test_nuevo": 130_000}


def _sesion(args):
    split, idx, n, dominio_nuevo = args
    rng = np.random.default_rng(BASE_SEMILLA[split] + idx)
    esc = E.escena_aleatoria(rng)
    if dominio_nuevo:
        # Dominio DESPLAZADO: suelos y luces que NO aparecen en train. Mide cuánto se cae el
        # modelo cuando cambia el entorno (sin tocar las etiquetas ni la geometría).
        if rng.random() < 0.5:
            esc.suelo = dict(tipo="baldosa", color=(90, 130, 90), color2=(70, 105, 75), escala=0.55, junta=0.02)
        else:
            esc.suelo = dict(tipo="madera", color=(225, 205, 160), escala=0.2)
        esc.luz["ambiente"] = float(rng.choice([0.32, 0.42, 0.95]))
    poses = E.poses_sesion(esc, rng, n)
    rgbs, clases, insts, pl = [], [], [], []
    for (x, y, psi, cab, bal, inc) in poses:
        cam = dict(R.CAMARA)
        cam["inclinacion"] = inc
        r = R.render(esc, x, y, psi, rng, cam=cam, cabeceo_extra=cab, balanceo=bal)
        rgbs.append(r["rgb"])
        clases.append(r["clase"])
        insts.append(r["instancia"])
        pl.append(r["plagas"])
    carpeta = DATOS / split
    carpeta.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        carpeta / f"sesion_{idx:04d}.npz",
        rgb=np.stack(rgbs), clase=np.stack(clases), instancia=np.stack(insts),
        poses=np.array([p for p in poses], np.float32),
        plagas=json.dumps(pl),
        meta=json.dumps({"tipo": esc.nombre, "suelo": esc.suelo["tipo"], "ambiente": esc.luz["ambiente"],
                         "n_macetas": len(esc.cilindros), "n_cajas": len(esc.cajas)}))
    return idx, len(poses)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", type=int, default=160)
    ap.add_argument("--val", type=int, default=30)
    ap.add_argument("--test", type=int, default=40)
    ap.add_argument("--test_nuevo", type=int, default=24)
    ap.add_argument("--frames", type=int, default=24)
    ap.add_argument("--procs", type=int, default=6)
    a = ap.parse_args()
    t0 = time.perf_counter()
    trabajos = []
    for split, n in (("train", a.train), ("val", a.val), ("test", a.test), ("test_nuevo", a.test_nuevo)):
        trabajos += [(split, i, a.frames, split == "test_nuevo") for i in range(n)]
    with Pool(a.procs) as pool:
        for k, _ in enumerate(pool.imap_unordered(_sesion, trabajos, chunksize=1)):
            if (k + 1) % 20 == 0:
                print(f"  {k+1}/{len(trabajos)} sesiones  ({time.perf_counter()-t0:.0f} s)", flush=True)
    resumen = {s: len(list((DATOS / s).glob("*.npz"))) for s in BASE_SEMILLA if (DATOS / s).exists()}
    print("sesiones por split:", resumen, f" · {time.perf_counter()-t0:.0f} s")


if __name__ == "__main__":
    main()
