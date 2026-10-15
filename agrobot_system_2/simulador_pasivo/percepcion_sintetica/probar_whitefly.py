"""Prueba el detector de mosca blanca sobre FOTOS DEL USUARIO, por ventanas a resolución nativa.

Uso: python3 probar_whitefly.py [--conf 0.25] [--modelo modelos/yolo/whitefly/weights/best.pt]

No hay etiquetas en esas fotos: esto NO mide precisión, solo enseña qué marca el modelo para que se
revise a ojo. Cualquier foto con la planta de la demo que se añada a la carpeta entra sola.
"""
import argparse
import glob
from pathlib import Path

import cv2
import numpy as np

AQUI = Path(__file__).resolve().parent
FOTOS = AQUI.parents[1] / "datos_plagas_reales" / "usuario_20261015"
T = 640


def nms(cajas, iou=0.4):
    if not cajas:
        return []
    b = np.array([c[:4] for c in cajas], float)
    s = np.array([c[4] for c in cajas], float)
    orden = s.argsort()[::-1]
    keep = []
    while orden.size:
        i = orden[0]
        keep.append(i)
        xx0, yy0 = np.maximum(b[i, 0], b[orden[1:], 0]), np.maximum(b[i, 1], b[orden[1:], 1])
        xx1, yy1 = np.minimum(b[i, 2], b[orden[1:], 2]), np.minimum(b[i, 3], b[orden[1:], 3])
        inter = np.maximum(xx1 - xx0, 0) * np.maximum(yy1 - yy0, 0)
        a_i = (b[i, 2] - b[i, 0]) * (b[i, 3] - b[i, 1])
        a_o = (b[orden[1:], 2] - b[orden[1:], 0]) * (b[orden[1:], 3] - b[orden[1:], 1])
        orden = orden[1:][inter / (a_i + a_o - inter + 1e-9) < iou]
    return [cajas[i] for i in keep]


def detectar(net, bgr, conf, solape=0.25, escala=1.0):
    if escala != 1.0:
        bgr = cv2.resize(bgr, None, fx=escala, fy=escala, interpolation=cv2.INTER_CUBIC)
    h, w = bgr.shape[:2]
    paso = int(T * (1 - solape))
    cajas = []
    ys = list(range(0, max(h - T, 0) + 1, paso)) or [0]
    xs = list(range(0, max(w - T, 0) + 1, paso)) or [0]
    if ys[-1] + T < h:
        ys.append(h - T)
    if xs[-1] + T < w:
        xs.append(w - T)
    for y0 in ys:
        for x0 in xs:
            tr = bgr[y0:y0 + T, x0:x0 + T]
            r = net.predict(tr, imgsz=T, conf=conf, verbose=False)[0]
            for b in r.boxes:
                x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
                cajas.append((x1 + x0, y1 + y0, x2 + x0, y2 + y0, float(b.conf)))
    return [tuple(v / escala for v in c[:4]) + (c[4],) for c in nms(cajas)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--modelo", default=str(AQUI / "modelos" / "yolo" / "whitefly" / "weights" / "best.pt"))
    ap.add_argument("--escala", type=float, default=1.0, help="ampliar la foto antes de buscar (p. ej. 1.5 si la mosca es muy pequeña)")
    a = ap.parse_args()
    from ultralytics import YOLO
    net = YOLO(a.modelo)
    sal = AQUI.parents[1] / "docs" / "capturas_plagas_20261015"
    sal.mkdir(parents=True, exist_ok=True)
    for f in sorted(glob.glob(str(FOTOS / "*"))):
        bgr = cv2.imread(f)
        if bgr is None:
            from PIL import Image
            bgr = np.array(Image.open(f).convert("RGB"))[..., ::-1].copy()
        cj = detectar(net, bgr, a.conf, escala=a.escala)
        vis = bgr.copy()
        for (x1, y1, x2, y2, c) in cj:
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), max(2, bgr.shape[1] // 600))
        vis = cv2.resize(vis, (1200, int(1200 * vis.shape[0] / vis.shape[1])))
        cv2.imwrite(str(sal / f"whitefly_{Path(f).stem}.jpg"), vis)
        print(f"{Path(f).name:22s} {bgr.shape[1]}x{bgr.shape[0]}  -> {len(cj)} detecciones", [round(c[4], 2) for c in cj[:8]])


if __name__ == "__main__":
    main()
