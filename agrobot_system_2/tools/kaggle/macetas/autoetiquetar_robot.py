"""Cajas de MACETA sobre fotos de la CÁMARA DEL ROBOT (2026-10-23): la sala del ensayo (callejón en L, macetas pegadas a la pared),
vista desde 24 cm del suelo. El detector entrenado el 2026-10-22 casi no había visto este punto de vista.

Acuerdo de dos modelos para no meter cajas malas: el detector propio del robot (agrobot_maceta_det.pt) y YOLO-World (vocabulario
abierto). Una caja se acepta si los dos coinciden (IoU >= 0,4) o si el detector propio pasa de 0,70. Imágenes sin ninguna caja de
ninguno de los dos = negativos (suelo, muebles, paredes del mismo cuarto), útiles contra falsos positivos.
Salida: ~/.agrobot/dataset_macetas/publicar/{images,labels} con prefijo `r_<sesion>_` (el notebook agrupa por sesión para el test)
y ~/.agrobot/dataset_macetas/revision_robot.jpg para revisarlas a ojo.
"""
import glob
import random
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO, YOLOWorld

random.seed(7)
OUT = Path.home() / ".agrobot/dataset_macetas/publicar"
for s in ("images", "labels"):
    (OUT / s).mkdir(parents=True, exist_ok=True)
MODELOS = Path("/home/jetson/Desktop/AGROBOT/agrobot_system_2/jetson/backend/models")
propio = YOLO(str(MODELOS / "agrobot_maceta_det.pt"))
mundo = YOLOWorld(str(MODELOS / "yolov8s-worldv2.pt"))
mundo.set_classes(["flower pot", "potted plant", "plant pot"])

CARPETAS = sorted(glob.glob(str(Path.home() / ".agrobot/campo/2026102[23]_2[12]*/fotos"))
                  + glob.glob(str(Path.home() / ".agrobot/campo/20261023_*/fotos"))
                  + glob.glob(str(Path.home() / ".agrobot/datos/20261023_*/camara")))
TOPE_POR_CARPETA, MAX_NEG = 90, 120


def iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    i = max(0, x2 - x1) * max(0, y2 - y1)
    return i / max(1e-6, (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i)


con, neg, ej = 0, 0, []
for carpeta in CARPETAS:
    fotos = sorted(glob.glob(carpeta + "/*.jpg"))
    if not fotos:
        continue
    paso = max(1, len(fotos) // TOPE_POR_CARPETA)
    sesion = Path(carpeta).parent.name.replace("_", "")[:15]
    for f in fotos[::paso]:
        im = cv2.imread(f)
        if im is None:
            continue
        h, w = im.shape[:2]
        rp = propio.predict(im, conf=0.15, imgsz=640, verbose=False)[0]
        rm = mundo.predict(im, conf=0.15, imgsz=640, verbose=False)[0]
        cp = [([float(v) for v in b.xyxy[0]], float(b.conf)) for b in rp.boxes]
        cm = [([float(v) for v in b.xyxy[0]], float(b.conf)) for b in rm.boxes
              if (b.xyxy[0][2] - b.xyxy[0][0]) * (b.xyxy[0][3] - b.xyxy[0][1]) < 0.6 * w * h]
        # Revisión del 2026-10-23: con «solo acuerdo» faltaba a menudo una de las 3 macetas, y una maceta sin caja enseña a NO verla.
        # Ahora: caja segura = propio >= 0,45, o YOLO-World >= 0,35, o los dos de acuerdo; duplicados fuera (IoU >= 0,5).
        seguras = [(c, p) for c, p in cp if p >= 0.45 or any(iou(c, m) >= 0.4 for m, _ in cm)] + [(c, p) for c, p in cm if p >= 0.35]
        cajas = []
        for c, _ in sorted(seguras, key=lambda t: -t[1]):
            if all(iou(c, k) < 0.5 for k in cajas):
                cajas.append(c)
        # Algo detectado con poca confianza que no coincide con ninguna caja segura = foto dudosa: no se usa (ni positiva ni negativa).
        if any(all(iou(c, k) < 0.3 for k in cajas) for c, p in cp + cm if p >= 0.20):
            continue
        if not cajas:
            if neg >= MAX_NEG or random.random() > 0.5:
                continue
            neg += 1
        else:
            con += 1
        nombre = f"r_{sesion}_{Path(f).stem.replace('.', '_')}"
        cv2.imwrite(str(OUT / "images" / f"{nombre}.jpg"), im, [cv2.IMWRITE_JPEG_QUALITY, 90])
        (OUT / "labels" / f"{nombre}.txt").write_text("".join(
            f"0 {(x1 + x2) / 2 / w:.5f} {(y1 + y2) / 2 / h:.5f} {(x2 - x1) / w:.5f} {(y2 - y1) / h:.5f}\n" for x1, y1, x2, y2 in cajas))
        if cajas and len(ej) < 300:
            o = im.copy()
            for x1, y1, x2, y2 in cajas:
                cv2.rectangle(o, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
            ej.append(cv2.resize(o, (240, 180)))
    print(f"{carpeta}: con maceta {con} · negativos {neg}", flush=True)

print("TOTAL con maceta:", con, "negativos:", neg, flush=True)
random.shuffle(ej)
ej = ej[:36]
while len(ej) % 6:
    ej.append(np.zeros((180, 240, 3), np.uint8))
cv2.imwrite(str(Path.home() / ".agrobot/dataset_macetas/revision_robot.jpg"), np.vstack([np.hstack(ej[r:r + 6]) for r in range(0, len(ej), 6)]))
