# AGROBOT · MODELO DE CONTEXTO (2026-11-09). Capa 1 del diseño universal: encuentra plantas, coles, macetas, obstáculos y bordes bajos desde la vista baja del robot (solo agronomía y manejo).
# La especie concreta la da CLIP sobre el recorte. Compara el resultado con los detectores actuales en fotos del robot etiquetadas a mano y nunca entrenadas.
import hashlib, json, os, random, shutil, subprocess, sys
from collections import Counter, defaultdict
from pathlib import Path
DRY = bool(os.environ.get("CONTEXTO_DRY"))
ENTRADA = Path(os.environ.get("CONTEXTO_INPUT", "/kaggle/input"))
RES = Path(os.environ.get("CONTEXTO_SALIDA", "/kaggle/working"))
if not DRY:
    subprocess.run([sys.executable, "-m", "pip", "install", "-q", "ultralytics"], check=False)
import cv2
random.seed(42)
CLASES = ["planta", "col", "cogollo", "maceta", "obstaculo", "borde_bajo"]   # solo agronomía y lo que ayuda a manejar (2026-11-10): sin persona, animal ni vehículo
K = {c: i for i, c in enumerate(CLASES)}
DS = RES / "contexto"
for sp in ("train", "val"):
    (DS / "images" / sp).mkdir(parents=True, exist_ok=True); (DS / "labels" / sp).mkdir(parents=True, exist_ok=True)
registro = Counter()

def buscar(pred):
    for p in sorted(ENTRADA.rglob("*")):
        if pred(p): return p
    return None

def guardar(img, sp, nombre, cajas, origen, lado=800):
    h, w = img.shape[:2]; k = lado / max(h, w)
    if k < 1: img = cv2.resize(img, (int(w * k), int(h * k)), interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(DS / "images" / sp / f"{nombre}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    (DS / "labels" / sp / f"{nombre}.txt").write_text("".join(f"{c} {x:.5f} {y:.5f} {bw:.5f} {bh:.5f}\n" for c, x, y, bw, bh in cajas))
    registro[(sp, origen)] += 1
    for c, *_ in cajas: registro[("cajas_" + sp, CLASES[c])] += 1

def leer(lab, mapa):
    out = []
    for l in lab.read_text().splitlines() if lab.exists() else []:
        p = l.split()
        if len(p) == 5 and int(p[0]) in mapa: out.append((mapa[int(p[0])], *map(float, p[1:])))
    return out

def partir(nombre, resto=6):
    return "val" if int(hashlib.sha1(nombre.encode()).hexdigest(), 16) % resto == 0 else "train"

raiz = next(p for p in sorted(ENTRADA.rglob("*")) if p.is_dir() and (p / "mano_eval" / "images").is_dir() and (p / "pesos").is_dir())
# ---------------- 1) COCO: muebles/obstáculos y macetas, con cajas hechas a mano (personas, animales y vehículos quedan como fondo) ----------------
COCO = {15: "obstaculo", 27: "obstaculo", 33: "obstaculo", 62: "obstaculo", 63: "obstaculo", 65: "obstaculo", 67: "obstaculo", 64: "maceta"}
TOPE_COCO = 2500
ann = buscar(lambda p: p.name == "instances_val2017.json")
if ann is not None:
    for jf, carpeta, tope, sp in (("instances_train2017.json", "train2017", TOPE_COCO, "train"), ("instances_val2017.json", "val2017", 400, "val")):
        if not (ann.parent / jf).exists(): print("falta", jf); continue
        d = json.load(open(ann.parent / jf)); imgs = {i["id"]: i for i in d["images"]}; por = defaultdict(list)
        for a in d["annotations"]:
            if a["category_id"] in COCO and not a.get("iscrowd"): por[a["image_id"]].append(a)
        carp = buscar(lambda p: p.is_dir() and p.name == carpeta)
        ids = list(por); random.shuffle(ids); usados = Counter()
        for i in ids:
            clases = {COCO[a["category_id"]] for a in por[i]}
            if all(usados[c] >= tope for c in clases): continue
            img = cv2.imread(str(carp / imgs[i]["file_name"]))
            if img is None: continue
            W, H = imgs[i]["width"], imgs[i]["height"]; cajas = []
            for a in por[i]:
                x, y, w, h = a["bbox"]; cajas.append((K[COCO[a["category_id"]]], (x + w / 2) / W, (y + h / 2) / H, w / W, h / H))
            guardar(img, sp, f"coco_{i}", cajas, "coco")
            for c in clases: usados[c] += 1
# ---------------- 2) MACETAS PROPIAS (vista del robot y fotos del operador) ----------------
for f in sorted((raiz / "fuente_macetas" / "images").glob("*.jpg")):
    img = cv2.imread(str(f))
    if img is not None: guardar(img, partir(f.stem), f"mac_{f.stem}", leer(raiz / "fuente_macetas" / "labels" / f"{f.stem}.txt", {0: K["maceta"]}), "macetas")
# ---------------- 3) COLES ----------------
for sp in ("train", "val"):
    for f in sorted((raiz / "fuente_col" / "images" / sp).glob("*.jpg")):
        img = cv2.imread(str(f))
        if img is not None: guardar(img, sp, f"col_{f.stem}", leer(raiz / "fuente_col" / "labels" / sp / f"{f.stem}.txt", {0: K["col"], 1: K["cogollo"]}), "coles")
# ---------------- 4) OBSTÁCULOS Y BORDES BAJOS SINTÉTICOS (acotados: no deben dominar) ----------------
sin = buscar(lambda p: p.is_dir() and p.name == "yolo_obj")
if sin is not None:
    for sp in ("train", "val"):
        fs = sorted((sin / "images" / sp).glob("*.jpg")); random.shuffle(fs)
        for f in fs[:(1500 if sp == "train" else 200)]:
            img = cv2.imread(str(f))
            if img is not None: guardar(img, sp, f"sin_{f.stem}", leer(sin / "labels" / sp / f"{f.stem}.txt", {0: K["maceta"], 1: K["obstaculo"], 2: K["borde_bajo"]}), "sintetico")
# ---------------- 5) FOTOS DEL ROBOT CON CAJAS A MANO: se repiten para que pesen frente a COCO ----------------
REPETIR = 10
identidad = {i: i for i in range(len(CLASES))}
for f in sorted((raiz / "mano_train" / "images").glob("*.jpg")):
    img = cv2.imread(str(f))
    if img is None: continue
    cajas = leer(raiz / "mano_train" / "labels" / f"{f.stem}.txt", identidad)
    if partir(f.stem, 8) == "val":
        guardar(img, "val", f"mano_{f.stem}", cajas, "mano")
    else:
        for r in range(REPETIR): guardar(img, "train", f"mano{r}_{f.stem}", cajas, "mano")
# ---------------- 6) FOTOS DEL ROBOT ETIQUETADAS POR CLAUDE (2026-11-10): propuestas del modelo revisadas foto a foto; solo entrenan, pesan la mitad que las del operador ----------------
REPETIR_CLAUDE = 5
for f in sorted((raiz / "claude_train" / "images").glob("*.jpg")) if (raiz / "claude_train").is_dir() else []:
    img = cv2.imread(str(f))
    if img is None: continue
    cajas = leer(raiz / "claude_train" / "labels" / f"{f.stem}.txt", identidad)
    for r in range(REPETIR_CLAUDE): guardar(img, "train", f"claude{r}_{f.stem}", cajas, "claude")
# ---------------- 7) CULTIVO Y MALEZA PÚBLICO (cubeai/crop-weed-detection-for-yolov8, CC0): crop -> planta; la maleza no se marca (igual que en las fotos del robot) ----------------
yaml_cw = buscar(lambda p: p.name == "data.yaml" and "crop" in p.read_text(errors="ignore") and "weed" in p.read_text(errors="ignore"))
if yaml_cw is not None:
    for carpeta, sp, tope in (("train", "train", 1200), ("valid", "val", 150)):
        fs = sorted((yaml_cw.parent / carpeta / "images").glob("*.*")); random.shuffle(fs)
        for f in fs[:tope]:
            img = cv2.imread(str(f))
            if img is not None: guardar(img, sp, f"cw_{f.stem}", leer(yaml_cw.parent / carpeta / "labels" / f"{f.stem}.txt", {0: K["planta"]}), "cultivo_publico")
else:
    print("sin dataset de cultivo y maleza")
print(sorted(registro.items(), key=str))
(DS / "dataset.yaml").write_text(f"path: {DS}\ntrain: images/train\nval: images/val\nnc: {len(CLASES)}\nnames:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(CLASES)))
fotos_eval = sorted((raiz / "mano_eval" / "images").glob("*.jpg"))
assert not ({f.stem for f in fotos_eval} & {f.stem for f in (raiz / "mano_train" / "images").glob("*.jpg")})
assert not ({f.stem for f in fotos_eval} & {f.stem for f in (raiz / "claude_train" / "images").glob("*.jpg")})
if DRY: raise SystemExit("CONTEXTO_DRY: dataset armado, sin entrenar")

from ultralytics import YOLO
det = YOLO("yolo11s.pt")
det.train(data=str(DS / "dataset.yaml"), epochs=200, time=4.0, imgsz=800, batch=32, patience=30, close_mosaic=15, device=[0, 1], project=str(RES / "runs"), name="contexto",
          hsv_h=.015, hsv_s=.6, hsv_v=.4, fliplr=.5, flipud=0.0, mosaic=1.0, mixup=.05, exist_ok=True, verbose=False)
shutil.copy(RES / "runs" / "contexto" / "weights" / "best.pt", RES / "agrobot_contexto.pt")
mv = YOLO(str(RES / "agrobot_contexto.pt")).val(data=str(DS / "dataset.yaml"), split="val", imgsz=800, project=str(RES / "runs"), name="val", exist_ok=True)
por_clase = {CLASES[int(i)]: round(float(v), 3) for i, v in zip(mv.box.ap_class_index, mv.box.ap50)}
print("mAP50 por clase (validación mezclada):", por_clase)

def iou(a, b):
    ix = min(a[0] + a[2] / 2, b[0] + b[2] / 2) - max(a[0] - a[2] / 2, b[0] - b[2] / 2); iy = min(a[1] + a[3] / 2, b[1] + b[3] / 2) - max(a[1] - a[3] / 2, b[1] - b[3] / 2)
    return 0.0 if ix <= 0 or iy <= 0 else ix * iy / (a[2] * a[3] + b[2] * b[3] - ix * iy)
VERDES = {"planta", "col", "cogollo", "maceta", "col_planta", "col_cogollo"}
def predecir(modelos, conf):
    out = []
    for f in fotos_eval:
        p = []
        for m in modelos:
            r = m.predict(str(f), imgsz=800, conf=conf, verbose=False, device=0)[0]
            p += [tuple(map(float, b)) for c, b in zip(r.boxes.cls, r.boxes.xywhn) if r.names[int(c)] in VERDES]
        out.append(p)
    return out
reales = [[c[1:] for c in leer(raiz / "mano_eval" / "labels" / f"{f.stem}.txt", identidad) if CLASES[c[0]] in VERDES] for f in fotos_eval]
def medir(modelos):
    res = {}
    for conf in (0.15, 0.25, 0.4):
        pred = predecir(modelos, conf)
        n = sum(map(len, reales)); enc = sum(any(iou(p, b) >= 0.3 for p in ps) for bs, ps in zip(reales, pred) for b in bs)
        total = sum(map(len, pred)); ok = sum(any(iou(p, b) >= 0.3 for b in bs) for bs, ps in zip(reales, pred) for p in ps)
        res[str(conf)] = {"plantas_encontradas": enc, "plantas_totales": n, "exhaustividad": round(enc / max(n, 1), 3), "precision_cajas": round(ok / max(total, 1), 3), "cajas_por_foto": round(total / max(len(fotos_eval), 1), 2)}
    return res
resultado = {"fotos_evaluacion": len(fotos_eval), "actuales_maceta_y_col": medir([YOLO(str(raiz / "pesos" / n)) for n in ("agrobot_maceta_det.pt", "agrobot_col_yolo11n.pt")]),
             "contexto": medir([YOLO(str(RES / "agrobot_contexto.pt"))]), "map50_validacion": por_clase}
(RES / "comparacion_contexto.json").write_text(json.dumps(resultado, ensure_ascii=False, indent=1)); print(json.dumps(resultado, ensure_ascii=False, indent=1))
print("HECHO: agrobot_contexto.pt y comparacion_contexto.json")
