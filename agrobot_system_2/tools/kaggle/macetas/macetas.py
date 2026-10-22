# AGROBOT · DETECTOR DE MACETAS (2026-10-22) — para que el robot las vea, NO las pise y las escanee con la cámara.
# Combina (1) COCO 2017 «potted plant» (cajas a mano, miles de fotos), (2) los datos propios del operador (fotos, videos de webcam y la
# cámara del robot) con cajas de YOLO-World revisadas, (3) negativos de interior (suelo, muebles, zapatos). Entrena YOLO11n (rápido en la
# Jetson) y mide aparte en las imágenes PROPIAS, que es lo que importa. Salida: /kaggle/working/maceta_det_best.pt + metricas.json.
import os, sys, json, random, shutil, subprocess, hashlib
from pathlib import Path
from collections import defaultdict
random.seed(42)
subprocess.run([sys.executable, "-m", "pip", "install", "-q", "ultralytics"], check=False)
import cv2
ENTRADA, RES = Path("/kaggle/input"), Path("/kaggle/working")
DS = RES / "macetas"
for sp in ("train", "val", "test_propio"):
    (DS / "images" / sp).mkdir(parents=True, exist_ok=True); (DS / "labels" / sp).mkdir(parents=True, exist_ok=True)

def buscar(pred):
    for p in sorted(ENTRADA.rglob("*")):
        if pred(p): return p
    return None

def guardar(img, sp, nombre, cajas):
    h, w = img.shape[:2]; k = 800 / max(h, w)
    if k < 1: img = cv2.resize(img, (int(w * k), int(h * k)), interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(DS / "images" / sp / f"{nombre}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    (DS / "labels" / sp / f"{nombre}.txt").write_text("".join(f"0 {x:.5f} {y:.5f} {bw:.5f} {bh:.5f}\n" for x, y, bw, bh in cajas))

cuenta = defaultdict(int)
# ---------------- 1) COCO 2017: «potted plant» (categoría 64) ----------------
ann_dir = buscar(lambda p: p.name == "instances_val2017.json")
if ann_dir is not None:
    raiz = ann_dir.parent.parent
    for jf, carpeta, tope, sp in (("instances_train2017.json", "train2017", 3000, "train"), ("instances_val2017.json", "val2017", 10**6, "val")):
        d = json.load(open(ann_dir.parent / jf))
        imgs = {i["id"]: i for i in d["images"]}
        por = defaultdict(list)
        for a in d["annotations"]:
            if a["category_id"] == 64 and not a.get("iscrowd"):
                por[a["image_id"]].append(a["bbox"])
        ids = list(por); random.shuffle(ids)
        neg = [i for i in imgs if i not in por]; random.shuffle(neg)
        for iid in ids[:tope]:
            im = imgs[iid]; img = cv2.imread(str(raiz / carpeta / im["file_name"]))
            if img is None: continue
            W, H = im["width"], im["height"]
            guardar(img, sp, f"coco_{iid}", [((x + w / 2) / W, (y + h / 2) / H, w / W, h / H) for x, y, w, h in por[iid]]); cuenta[f"coco_{sp}"] += 1
        for iid in neg[:min(600, tope // 5)]:                       # negativos de COCO: que no vea macetas en todo
            img = cv2.imread(str(raiz / carpeta / imgs[iid]["file_name"]))
            if img is not None: guardar(img, sp, f"coconeg_{iid}", []); cuenta[f"coco_neg_{sp}"] += 1
        del d
print("COCO:", dict(cuenta), flush=True)
# ---------------- 2) DATOS PROPIOS (cajas de YOLO-World revisadas) ----------------
propio = buscar(lambda p: p.is_dir() and "macetas-propias" in p.name.lower() and (p / "images").is_dir())
print("datos propios:", propio)
if propio is not None:
    for f in sorted((propio / "images").glob("*.jpg")):
        grupo = f.stem.rsplit("_", 1)[0]                              # mismo video = mismo grupo: nada de fotogramas vecinos en train y test
        h = int(hashlib.md5(grupo.encode()).hexdigest(), 16) % 10
        sp = "test_propio" if h < 2 else "train"
        lab = propio / "labels" / (f.stem + ".txt")
        cajas = [tuple(map(float, l.split()[1:5])) for l in lab.read_text().splitlines() if len(l.split()) == 5] if lab.exists() else []
        img = cv2.imread(str(f))
        if img is not None:
            for rep in range(3 if sp == "train" else 1):              # los propios pesan más: son la cámara y las macetas reales
                guardar(img, sp, f"prop_{f.stem}_{rep}", cajas)
            cuenta[f"propio_{sp}"] += 1
# ---------------- 3) NEGATIVOS DE INTERIOR ----------------
neg = buscar(lambda p: p.is_dir() and "negativos" in p.name.lower() and (p / "images").is_dir())
if neg is not None:
    for j, f in enumerate(sorted((neg / "images").glob("*.jpg"))):
        img = cv2.imread(str(f))
        if img is not None:
            guardar(img, "test_propio" if j % 5 == 0 else "train", f"negint_{f.stem}", []); cuenta["negativos_interior"] += 1
print("composición:", dict(cuenta), flush=True)
(DS / "datos.yaml").write_text(f"path: {DS}\ntrain: images/train\nval: images/val\ntest: images/test_propio\nnames:\n  0: maceta\n")
# ---------------- ENTRENAMIENTO ----------------
from ultralytics import YOLO
m = YOLO("yolo11n.pt")
m.train(data=str(DS / "datos.yaml"), epochs=60, imgsz=640, batch=32, patience=15, device=0, project=str(RES / "runs"), name="maceta",
        exist_ok=True, verbose=False, mosaic=1.0, mixup=0.05, hsv_v=0.5, degrees=5)
best = RES / "runs" / "maceta" / "weights" / "best.pt"
shutil.copy(best, RES / "maceta_det_best.pt")
mb = YOLO(str(best))
r_coco = mb.val(data=str(DS / "datos.yaml"), split="val", project=str(RES / "runs"), name="val_coco", exist_ok=True, verbose=False)
r_prop = mb.val(data=str(DS / "datos.yaml"), split="test", project=str(RES / "runs"), name="test_propio", exist_ok=True, verbose=False)
fp = sum(1 for f in (DS / "images" / "test_propio").glob("negint_*.jpg") if len(mb.predict(str(f), conf=0.4, verbose=False)[0].boxes))
n_neg = len(list((DS / "images" / "test_propio").glob("negint_*.jpg")))
met = {"composicion": dict(cuenta), "coco_val_map50": float(r_coco.box.map50), "propio_map50": float(r_prop.box.map50),
       "propio_precision": float(r_prop.box.mp), "propio_recall": float(r_prop.box.mr), "falsos_positivos_fondos_interior": f"{fp}/{n_neg}"}
(RES / "metricas.json").write_text(json.dumps(met, indent=1, ensure_ascii=False))
print("\nMÉTRICAS:", json.dumps(met, indent=1, ensure_ascii=False))
print("HECHO: /kaggle/working/maceta_det_best.pt (clase 0 = maceta)")
