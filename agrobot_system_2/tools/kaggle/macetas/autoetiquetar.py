"""Cajas automáticas de MACETA (YOLO-World, vocabulario abierto) sobre los datos propios del operador, para el detector de macetas.
Salida: ~/.agrobot/dataset_macetas/publicar/{images,labels} (clase 0 = maceta) + revision.jpg para verificar a ojo."""
import glob, os, cv2, numpy as np, random
from pathlib import Path
from ultralytics import YOLOWorld
random.seed(1)
OUT = Path.home() / ".agrobot/dataset_macetas/publicar"
for s in ("images", "labels"): (OUT / s).mkdir(parents=True, exist_ok=True)
m = YOLOWorld("/home/jetson/Desktop/AGROBOT/agrobot_system_2/jetson/backend/models/yolov8s-worldv2.pt")
m.set_classes(["flower pot", "potted plant", "plant pot"])
fuentes = []
for f in sorted(glob.glob(str(Path.home() / ".agrobot/datos_usuario/20261022_subida/*.jpeg"))): fuentes.append(("foto", f, None))
for v, cada in [(p, 1.0) for p in sorted(glob.glob(str(Path.home() / ".agrobot/datos_usuario/20261022_videos/*.mp4")))] + \
               [(p, 2.0) for p in sorted(glob.glob(str(Path.home() / ".agrobot/campo/*/video_*.avi")))]:
    c = cv2.VideoCapture(v); fps = c.get(5) or 10; n = int(c.get(7))
    for i in range(0, n, max(1, int(fps * cada))): fuentes.append(("video", v, i))
print("fotogramas a revisar:", len(fuentes), flush=True)
cap_actual, cap = None, None; conteo = 0; ej = []
for k, (tipo, f, i) in enumerate(fuentes):
    if tipo == "foto": im = cv2.imread(f)
    else:
        if f != cap_actual: cap, cap_actual = cv2.VideoCapture(f), f
        cap.set(1, i); ok, im = cap.read(); im = im if ok else None
    if im is None: continue
    h, w = im.shape[:2]; kk = 960 / max(h, w)
    if kk < 1: im = cv2.resize(im, (int(w * kk), int(h * kk)), interpolation=cv2.INTER_AREA); h, w = im.shape[:2]
    r = m.predict(im, conf=0.30, imgsz=640, verbose=False)[0]
    cajas = [[float(v) for v in b.xyxy[0]] for b in r.boxes if (b.xyxy[0][2] - b.xyxy[0][0]) * (b.xyxy[0][3] - b.xyxy[0][1]) < 0.85 * w * h]
    if not cajas: continue
    nombre = f"{'f' if tipo == 'foto' else 'v'}_{Path(f).stem[-14:]}_{i or 0:05d}".replace(" ", "_").replace("(", "").replace(")", "")
    cv2.imwrite(str(OUT / "images" / f"{nombre}.jpg"), im, [cv2.IMWRITE_JPEG_QUALITY, 90])
    (OUT / "labels" / f"{nombre}.txt").write_text("".join(f"0 {(x1+x2)/2/w:.5f} {(y1+y2)/2/h:.5f} {(x2-x1)/w:.5f} {(y2-y1)/h:.5f}\n" for x1, y1, x2, y2 in cajas))
    conteo += 1
    if len(ej) < 400:
        o = im.copy()
        for x1, y1, x2, y2 in cajas: cv2.rectangle(o, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
        ej.append(cv2.resize(o, (240, 180)))
    if k % 100 == 0: print(f"{k}/{len(fuentes)} · con maceta {conteo}", flush=True)
print("imágenes con maceta:", conteo, flush=True)
random.shuffle(ej); ej = ej[:30]
while len(ej) % 6: ej.append(np.zeros((180, 240, 3), np.uint8))
cv2.imwrite(str(Path.home() / ".agrobot/dataset_macetas/revision.jpg"), np.vstack([np.hstack(ej[r:r+6]) for r in range(0, len(ej), 6)]))
