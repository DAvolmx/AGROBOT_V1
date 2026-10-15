"""Exporta el dataset de OBJETOS (navegación) a formato YOLO desde las sesiones ya generadas.

Clases: 0 maceta_planta · 1 obstaculo · 2 borde_bajo. Las cajas salen de la máscara de
INSTANCIA (verdad del simulador, solo para etiquetar y evaluar). Una maceta y su planta forman
UNA instancia. Se descartan cajas con menos de 60 píxeles o de menos de 6 px de lado.

Separado a propósito de la detección de plagas (`generar_plagas.py`): son tareas y modelos distintos.
"""
import glob
import json
import sys
from pathlib import Path

import cv2
import numpy as np

AQUI = Path(__file__).resolve().parent
DATOS = AQUI / "datos"
SALIDA = AQUI / "yolo_obj"
CLASE_A_YOLO = {2: 0, 4: 1, 3: 2}
NOMBRES = ["maceta_planta", "obstaculo", "borde_bajo"]


def cajas_de(inst, clase, min_px=60, min_lado=6):
    out = []
    for i in np.unique(inst):
        if i < 0:
            continue
        m = inst == i
        n = int(m.sum())
        if n < min_px:
            continue
        cl = np.bincount(clase[m], minlength=5).argmax()
        if cl not in CLASE_A_YOLO:
            continue
        ys, xs = np.nonzero(m)
        x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
        if x1 - x0 < min_lado or y1 - y0 < min_lado:
            continue
        out.append((CLASE_A_YOLO[int(cl)], x0, y0, x1, y1))
    return out


def main():
    cont = {}
    for split in ("train", "val", "test", "test_nuevo"):
        (SALIDA / "images" / split).mkdir(parents=True, exist_ok=True)
        (SALIDA / "labels" / split).mkdir(parents=True, exist_ok=True)
        n_img = n_box = 0
        for f in sorted(glob.glob(str(DATOS / split / "*.npz"))):
            d = np.load(f, allow_pickle=True)
            base = Path(f).stem
            for k in range(len(d["rgb"])):
                h, w = d["clase"][k].shape
                cj = cajas_de(d["instancia"][k], d["clase"][k])
                nombre = f"{base}_{k:02d}"
                cv2.imwrite(str(SALIDA / "images" / split / f"{nombre}.jpg"), d["rgb"][k][..., ::-1],
                            [cv2.IMWRITE_JPEG_QUALITY, 95])
                with open(SALIDA / "labels" / split / f"{nombre}.txt", "w") as fh:
                    for (c, x0, y0, x1, y1) in cj:
                        fh.write(f"{c} {(x0+x1)/2/w:.6f} {(y0+y1)/2/h:.6f} {(x1-x0)/w:.6f} {(y1-y0)/h:.6f}\n")
                n_img += 1
                n_box += len(cj)
        cont[split] = (n_img, n_box)
    (SALIDA / "datos.yaml").write_text(
        f"path: {SALIDA}\ntrain: images/train\nval: images/val\ntest: images/test\n"
        f"names:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(NOMBRES)))
    (SALIDA / "datos_nuevo.yaml").write_text(
        f"path: {SALIDA}\ntrain: images/train\nval: images/test_nuevo\n"
        f"names:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(NOMBRES)))
    print({k: f"{a} imágenes, {b} cajas" for k, (a, b) in cont.items()})


if __name__ == "__main__":
    main()
