"""Entrena un detector de plagas/enfermedad con FOTOS REALES ETIQUETADAS (cuando las haya).

Uso:  python3 entrenar_plagas_real.py --datos /ruta/datos_etiquetados [--epocas 60] [--tam 640]

Estructura esperada (formato YOLO, una carpeta por PLANTA o SESIÓN):
    datos_etiquetados/
        planta_01/  foto_001.jpg  foto_001.txt  ...      # txt: "clase x y w h" normalizados
        planta_02/  ...
        clases.txt                                       # una clase por línea: mosca_blanca, trips, minador, ...

POR QUÉ ASÍ
  · Los insectos que se buscan (mosca blanca, trips) miden unos píxeles en una foto de 640x480. Se
    parte cada foto en MOSAICOS solapados que se amplían: así el insecto ocupa muchos más píxeles.
  · El reparto entrenamiento/validación/prueba es POR PLANTA, nunca por foto: fotos de la misma
    planta son casi idénticas y mezclarlas inflaría el resultado (error clásico).
  · Se guarda el mejor por validación, y SIEMPRE junto al modelo anterior: el modelo nuevo NO se
    activa solo. Para volver atrás basta seleccionar el modelo previo en el dashboard.

QUÉ NO DEMUESTRA: con pocas plantas, el resultado vale para ESAS plantas. Si se entrena con la planta
de la demostración, hay que decirlo ("calibrado con fotos de esta planta").
"""
import argparse
import json
import random
import shutil
import time
from pathlib import Path

import cv2
import numpy as np

AQUI = Path(__file__).resolve().parent
BASE = AQUI.parents[1] / "jetson" / "backend" / "models"


def mosaicos(img, cajas, tam=640, solape=0.25, min_vis=0.5):
    """Trozos de `tam` px con solape; las cajas se recortan y se descartan si queda <min_vis visible."""
    h, w = img.shape[:2]
    paso = int(tam * (1 - solape))
    out = []
    for y0 in range(0, max(h - tam, 0) + paso, paso):
        for x0 in range(0, max(w - tam, 0) + paso, paso):
            x0 = min(x0, max(w - tam, 0))
            y0 = min(y0, max(h - tam, 0))
            x1, y1 = min(x0 + tam, w), min(y0 + tam, h)
            trozo = img[y0:y1, x0:x1]
            lab = []
            for (c, cx, cy, bw, bh) in cajas:
                bx0, by0 = (cx - bw / 2) * w, (cy - bh / 2) * h
                bx1, by1 = (cx + bw / 2) * w, (cy + bh / 2) * h
                ix0, iy0, ix1, iy1 = max(bx0, x0), max(by0, y0), min(bx1, x1), min(by1, y1)
                if ix1 <= ix0 or iy1 <= iy0:
                    continue
                if (ix1 - ix0) * (iy1 - iy0) < min_vis * (bx1 - bx0) * (by1 - by0):
                    continue
                tw, th_ = x1 - x0, y1 - y0
                lab.append((c, ((ix0 + ix1) / 2 - x0) / tw, ((iy0 + iy1) / 2 - y0) / th_,
                            (ix1 - ix0) / tw, (iy1 - iy0) / th_))
            out.append((trozo, lab))
    return out


def leer_cajas(txt):
    cajas = []
    if txt.exists():
        for ln in txt.read_text().split("\n"):
            p = ln.split()
            if len(p) == 5:
                cajas.append((int(p[0]), *map(float, p[1:])))
    return cajas


def preparar(datos: Path, salida: Path, tam, semilla=0):
    plantas = sorted(p for p in datos.iterdir() if p.is_dir())
    if len(plantas) < 3:
        raise SystemExit(f"Hay {len(plantas)} planta(s)/sesión(es): hacen falta al menos 3 para separar "
                         "entrenamiento, validación y prueba POR PLANTA. Si solo hay una, no hay forma honrada "
                         "de medir si generaliza.")
    random.Random(semilla).shuffle(plantas)
    n = len(plantas)
    reparto = {"train": plantas[:max(1, n - 2 * max(1, n // 5))],
               "val": plantas[max(1, n - 2 * max(1, n // 5)):n - max(1, n // 5)], "test": plantas[n - max(1, n // 5):]}
    if salida.exists():
        shutil.rmtree(salida)
    cuenta = {}
    for split, ps in reparto.items():
        (salida / "images" / split).mkdir(parents=True)
        (salida / "labels" / split).mkdir(parents=True)
        k = nb = 0
        for p in ps:
            for f in sorted(list(p.glob("*.jpg")) + list(p.glob("*.png")) + list(p.glob("*.webp"))):
                img = cv2.imread(str(f))
                if img is None:
                    continue
                for i, (trozo, lab) in enumerate(mosaicos(img, leer_cajas(f.with_suffix(".txt")), tam)):
                    if split == "train" or lab or random.Random(i).random() < 0.2:    # val/test: todos los con objeto
                        nombre = f"{p.name}_{f.stem}_{i}"
                        cv2.imwrite(str(salida / "images" / split / f"{nombre}.jpg"), trozo)
                        (salida / "labels" / split / f"{nombre}.txt").write_text(
                            "".join(f"{c} {x:.6f} {y:.6f} {w:.6f} {h:.6f}\n" for c, x, y, w, h in lab))
                        k += 1
                        nb += len(lab)
        cuenta[split] = {"plantas": [p.name for p in ps], "mosaicos": k, "cajas": nb}
    clases = (datos / "clases.txt").read_text().split()
    (salida / "datos.yaml").write_text(f"path: {salida}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n"
                                       + "".join(f"  {i}: {c}\n" for i, c in enumerate(clases)))
    return cuenta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--datos", required=True)
    ap.add_argument("--epocas", type=int, default=60)
    ap.add_argument("--tam", type=int, default=640)
    ap.add_argument("--base", default="yolo11s.pt", help="yolo11n.pt | ip102_pest_yolo11s.pt (parte de plagas reales) ")
    a = ap.parse_args()
    from ultralytics import YOLO
    t0 = time.perf_counter()
    trabajo = AQUI / "datos_plagas_real_tmp"
    cuenta = preparar(Path(a.datos), trabajo, a.tam)
    print(json.dumps(cuenta, indent=1, ensure_ascii=False))
    base = BASE / a.base if (BASE / a.base).exists() else a.base
    m = YOLO(str(base))
    nombre = time.strftime("plagas_real_%Y%m%d_%H%M")
    m.train(data=str(trabajo / "datos.yaml"), imgsz=a.tam, epochs=a.epocas, batch=8, device=0, workers=2,
            project=str(AQUI / "modelos" / "yolo"), name=nombre, exist_ok=True, patience=20, plots=False,
            degrees=10, fliplr=0.5, flipud=0.5, hsv_h=0.015, hsv_s=0.5, hsv_v=0.4, scale=0.4, mosaic=1.0,
            verbose=False, seed=0, amp=True)
    mejor = AQUI / "modelos" / "yolo" / nombre / "weights" / "best.pt"
    net = YOLO(str(mejor))
    res = {"modelo": str(mejor), "base": a.base, "reparto": cuenta, "segundos": round(time.perf_counter() - t0)}
    for split in ("val", "test"):
        r = net.val(data=str(trabajo / "datos.yaml"), split=split, imgsz=a.tam, batch=8, device=0, plots=False, verbose=False)
        res[split] = {"mAP50": round(float(r.box.map50), 4), "precision": round(float(r.box.mp), 4),
                      "recall": round(float(r.box.mr), 4),
                      "por_clase_mAP50": {net.names[int(i)]: round(float(v), 4) for i, v in zip(r.box.ap_class_index, r.box.ap50)}}
    res["NO_ACTIVADO"] = ("Este modelo NO se activa solo. Copiar a jetson/backend/models/ y elegirlo en el dashboard "
                          "(Gestor de Modelos IA); para volver atrás, elegir de nuevo el anterior (ai.active_model).")
    (AQUI / "modelos" / f"{nombre}_metricas.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    print(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
