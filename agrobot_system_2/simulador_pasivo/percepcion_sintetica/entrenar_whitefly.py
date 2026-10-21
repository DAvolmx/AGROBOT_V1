"""Entrena un detector de MOSCA BLANCA con fotos reales (Cassava Whitefly Detection, Project-AgML).

Uso: python3 entrenar_whitefly.py [--epocas 25] [--base yolo11n.pt]

Datos: 2 de los 8 ficheros parquet (≈750 fotos de móvil de yuca, 4000x1920, ~27x30 px por mosca).
Se parten en MOSAICOS de 640x640 a RESOLUCIÓN NATIVA: reducir la foto haría invisible al insecto.

Reparto por BLOQUES de fotos contiguas, no al azar: fotos consecutivas son casi la misma planta y
mezclarlas entre conjuntos inflaría el resultado. train = fichero 0; val/test = mitades del fichero 1.
LÍMITE: es OTRA planta (yuca) y otra cámara. Mide si el detector aprende a ver una mosca blanca real,
no que funcione sobre col o acelga: eso se comprueba con las fotos del usuario (`probar_whitefly.py`).
"""
import argparse
import io
import json
import random
import shutil
import time
from pathlib import Path

import cv2
import numpy as np
import pyarrow.parquet as pq
from PIL import Image

AQUI = Path(__file__).resolve().parent
DATOS = AQUI.parents[1] / "datos_plagas_reales" / "internet" / "cassava_whitefly" / "data"
SAL = AQUI / "yolo_whitefly"
T = 640


def mosaicos(img, cajas, rng, vacios=2, min_vis=0.5):
    h, w = img.shape[:2]
    cand = {}
    for (x, y, bw, bh) in cajas:
        gx, gy = min(int((x + bw / 2) // T), max(w // T - 1, 0)), min(int((y + bh / 2) // T), max(h // T - 1, 0))
        cand[(gx, gy)] = True
    ncols, nrows = max(w // T, 1), max(h // T, 1)
    libres = [(i, j) for i in range(ncols) for j in range(nrows) if (i, j) not in cand]
    for k in rng.sample(libres, min(vacios, len(libres))):
        cand[k] = True
    out = []
    for (gx, gy) in cand:
        x0, y0 = gx * T, gy * T
        trozo = img[y0:y0 + T, x0:x0 + T]
        if trozo.shape[0] < T or trozo.shape[1] < T:
            continue
        lab = []
        for (x, y, bw, bh) in cajas:
            ix0, iy0, ix1, iy1 = max(x, x0), max(y, y0), min(x + bw, x0 + T), min(y + bh, y0 + T)
            if ix1 <= ix0 or iy1 <= iy0 or (ix1 - ix0) * (iy1 - iy0) < min_vis * bw * bh:
                continue
            lab.append((0, ((ix0 + ix1) / 2 - x0) / T, ((iy0 + iy1) / 2 - y0) / T, (ix1 - ix0) / T, (iy1 - iy0) / T))
        out.append((trozo, lab))
    return out


def exportar(fichero, split, desde, hasta, rng, cont):
    pf = pq.ParquetFile(fichero)
    i = 0
    for g in range(pf.num_row_groups):
        for r in pf.read_row_group(g).to_pylist():
            if desde <= i < hasta:
                im = np.array(Image.open(io.BytesIO(r["image"]["bytes"])).convert("RGB"))[..., ::-1].copy()
                for k, (trozo, lab) in enumerate(mosaicos(im, r["objects"]["bbox"], rng)):
                    n = f"{split}_{i:04d}_{k}"
                    cv2.imwrite(str(SAL / "images" / split / f"{n}.jpg"), trozo, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    (SAL / "labels" / split / f"{n}.txt").write_text("".join(f"{c} {x:.6f} {y:.6f} {w:.6f} {h:.6f}\n" for c, x, y, w, h in lab))
                    cont[split][0] += 1
                    cont[split][1] += len(lab)
            i += 1
            if i >= hasta:
                return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epocas", type=int, default=25)
    ap.add_argument("--base", default="yolo11n.pt")
    ap.add_argument("--solo_datos", action="store_true")
    ap.add_argument("--reusar", action="store_true", help="no volver a exportar los mosaicos si ya existen")
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--nombre", default="whitefly", help="nombre de la corrida (no pisa las anteriores)")
    ap.add_argument("--patience", type=int, default=10, help="épocas sin mejorar antes de parar; subirlo = exigir más")
    a = ap.parse_args()
    t0 = time.perf_counter()
    cont = {s: [0, 0] for s in ("train", "val", "test")}
    if a.reusar and (SAL / "datos.yaml").exists():
        for s in cont:
            cont[s] = [len(list((SAL / "images" / s).glob("*.jpg"))),
                       sum(len(p.read_text().split("\n")) - 1 for p in (SAL / "labels" / s).glob("*.txt"))]
    else:
        if SAL.exists():
            shutil.rmtree(SAL)
        for s in ("train", "val", "test"):
            (SAL / "images" / s).mkdir(parents=True)
            (SAL / "labels" / s).mkdir(parents=True)
        rng = random.Random(0)
        f = sorted(DATOS.glob("*.parquet"))
        exportar(f[0], "train", 0, 10**6, rng, cont)
        exportar(f[1], "val", 0, 110, rng, cont)
        exportar(f[1], "test", 140, 10**6, rng, cont)       # hueco de 30 fotos entre val y test
    print({k: f"{v[0]} mosaicos, {v[1]} cajas" for k, v in cont.items()}, f"{time.perf_counter()-t0:.0f} s", flush=True)
    (SAL / "datos.yaml").write_text(f"path: {SAL}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: mosca_blanca\n")
    if a.solo_datos:
        return
    from ultralytics import YOLO
    base = AQUI.parents[1] / "jetson" / "backend" / "models" / a.base
    m = YOLO(str(base if base.exists() else a.base))
    m.train(data=str(SAL / "datos.yaml"), imgsz=T, epochs=a.epocas, batch=a.batch, device=0, workers=a.workers,
            project=str(AQUI / "modelos" / "yolo"), name=a.nombre, exist_ok=True, patience=a.patience, plots=False,
            degrees=0, flipud=0.5, fliplr=0.5, hsv_s=0.5, hsv_v=0.4, scale=0.3, mosaic=1.0, verbose=False, seed=0, amp=True)
    mejor = AQUI / "modelos" / "yolo" / a.nombre / "weights" / "best.pt"
    net = YOLO(str(mejor))
    res = {"modelo": str(mejor), "datos": {k: {"mosaicos": v[0], "cajas": v[1]} for k, v in cont.items()}}
    for split in ("val", "test"):
        r = net.val(data=str(SAL / "datos.yaml"), split=split, imgsz=T, batch=a.batch, device=0, plots=False, verbose=False, workers=0)
        res[split] = {"mAP50": round(float(r.box.map50), 4), "mAP50_95": round(float(r.box.map), 4),
                      "precision": round(float(r.box.mp), 4), "recall": round(float(r.box.mr), 4)}
        print(split, res[split], flush=True)
    res["NO_DEMOSTRADO"] = "Entrenado y medido con yuca (otra planta y cámara). No demuestra nada sobre col ni acelga."
    (AQUI / "modelos" / f"{a.nombre}_metricas.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    print("guardado", f"{time.perf_counter()-t0:.0f} s")


if __name__ == "__main__":
    main()
