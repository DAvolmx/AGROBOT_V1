"""Detector de COL: planta (0), cogollo (1) y daño de hoja de masticadores (2).

Datos: datos_plagas_reales/internet/col_dano (huggingface Arko007/white-cabbage-leaf-damage, CC-BY-4.0),
1200 fotos con cajas YOLO. Son fotos de OTRO campo y cámara (vista alta): la detección de la col y del
daño en tu robot (cámara baja) NO está demostrada hasta probarla con tus fotos.
NO detecta mosca blanca ni pulgón: solo daño de hoja. Uso: python3 entrenar_col.py [--epocas 25] [--batch 8]
"""
import argparse
import glob
import json
import os
import random
import time
from pathlib import Path

from ultralytics import YOLO

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parents[1]
DATOS = RAIZ / "datos_plagas_reales" / "internet" / "col_dano" / "data"
SAL = AQUI / "yolo_col"
NOMBRES = ["col_planta", "col_cogollo", "dano_hoja"]


def preparar():
    imgs = sorted(glob.glob(str(DATOS / "Raw images (JPEG format)" / "*.jpg")))
    random.Random(0).shuffle(imgs)
    n_val = max(40, len(imgs) // 10)
    reparto = {"val": imgs[:n_val], "train": imgs[n_val:]}
    for split, lista in reparto.items():
        (SAL / "images" / split).mkdir(parents=True, exist_ok=True)
        (SAL / "labels" / split).mkdir(parents=True, exist_ok=True)
        for im in lista:
            nombre = Path(im).stem
            lab = DATOS / "Annotated labels (YOLO format, TXT files)" / f"{nombre}.txt"
            if not lab.exists():
                continue
            for destino, origen in ((SAL / "images" / split / f"{nombre}.jpg", im),
                                    (SAL / "labels" / split / f"{nombre}.txt", str(lab))):
                if not destino.exists():
                    os.symlink(origen, destino)
    (SAL / "datos.yaml").write_text(
        f"path: {SAL}\ntrain: images/train\nval: images/val\nnames:\n" +
        "".join(f"  {i}: {n}\n" for i, n in enumerate(NOMBRES)))
    return {k: len(v) for k, v in reparto.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epocas", type=int, default=25)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--base", default="yolo11n.pt", help="yolo11n.pt | yolo11s.pt (más grande, más lento)")
    ap.add_argument("--nombre", default="col", help="nombre de la corrida (no pisa las anteriores)")
    ap.add_argument("--patience", type=int, default=8, help="épocas sin mejorar antes de parar; subirlo = exigir más")
    a = ap.parse_args()
    t0 = time.perf_counter()
    print("datos:", preparar(), flush=True)
    base = RAIZ / "jetson" / "backend" / "models" / a.base
    m = YOLO(str(base if base.exists() else a.base))
    m.train(data=str(SAL / "datos.yaml"), imgsz=a.imgsz, epochs=a.epocas, batch=a.batch, device=0, workers=a.workers,
            project=str(AQUI / "modelos" / "yolo"), name=a.nombre, exist_ok=True, patience=a.patience, plots=False,
            degrees=10, flipud=0.3, fliplr=0.5, hsv_s=0.5, hsv_v=0.4, scale=0.4, mosaic=1.0, verbose=False,
            seed=0, amp=True, cache=False)
    mejor = AQUI / "modelos" / "yolo" / a.nombre / "weights" / "best.pt"
    r = YOLO(str(mejor)).val(data=str(SAL / "datos.yaml"), imgsz=a.imgsz, batch=a.batch, device=0, plots=False,
                             verbose=False, workers=0)
    res = {"modelo": str(mejor), "mAP50": round(float(r.box.map50), 4), "mAP50_95": round(float(r.box.map), 4),
           "precision": round(float(r.box.mp), 4), "recall": round(float(r.box.mr), 4),
           "por_clase_mAP50": {NOMBRES[i]: round(float(v), 3) for i, v in enumerate(r.box.ap50)},
           "NO_DEMOSTRADO": "Validado con fotos del mismo dataset (otro campo/camara). Probar con fotos propias.",
           "segundos": round(time.perf_counter() - t0)}
    (AQUI / "modelos" / f"{a.nombre}_metricas.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    print(res, flush=True)


if __name__ == "__main__":
    main()
