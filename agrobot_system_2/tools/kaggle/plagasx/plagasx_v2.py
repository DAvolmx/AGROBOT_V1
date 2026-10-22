# AGROBOT · PLAGASX v5 (negativos de interior + cajas reales de daño de hoja y mosca blanca) — base v2 — coles: plagas y enfermedades. Combina: (1) tu dataset de coles, (2) tres datasets externos de coliflor/papa,
# (3) tu dataset de SENDEROS ('navegacion') como imágenes de fondo (plantas y caminos SIN enfermedad: reducen falsos positivos).
# Entrena (a) un DETECTOR YOLO y (b) un CLASIFICADOR YOLO-cls. Las etiquetas de los datasets externos son de imagen completa, así que las
# métricas que valen son las del CLASIFICADOR; el detector sirve para localizar la planta. Todo se imprime con tablas y gráficas.
import os, sys, json, random, shutil, subprocess, re, time
from pathlib import Path
from collections import Counter, defaultdict
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import pandas as pd
import cv2
try:
    from IPython.display import display
except Exception:
    display = print

DRY = bool(os.environ.get("PLAGAS_DRY"))                       # solo arma el dataset (prueba local, sin entrenar)
ENTRADA = Path(os.environ.get("PLAGAS_INPUT", "/kaggle/input"))
SALIDA = Path(os.environ.get("PLAGAS_SALIDA", "/kaggle/working")) / "dataset_combinado"
RES = SALIDA.parent
random.seed(42); np.random.seed(42)
if not DRY:
    subprocess.run([sys.executable, "-m", "pip", "install", "-q", "ultralytics"], check=False)

CLASES = ["col_sana", "col_amarillamiento", "col_manchas_cafe", "col_dano_insecto", "col_mildiu_velloso", "col_malformacion", "col_pudricion", "col_mosca_blanca"]
# nombre de carpeta externo (normalizado: minúsculas y solo letras/números) -> clase
MAPA = {"healthy": 0, "nodisease": 0, "blackrot": 2, "alternarialeafspot": 2, "alternaria": 2, "bacterialspot": 2, "bacterialspotrot": 2,
        "ringspot": 2, "leafspot": 2, "downymildew": 4, "softrot": 6, "clubroot": 5,
        "cabbageaphidcolony": 3, "greenpeachaphid": 3, "potatoleafminer": 3, "leafminer": 3, "aphid": 3}
norm = lambda s: re.sub(r"[^a-z0-9]", "", s.lower())
MAX_POR_CLASE, MAX_FONDO = 200, 120

# ---------------- localizar los datasets montados (Kaggle cambia la ruta según la versión) ----------------
def buscar_dir(pred, base=ENTRADA):
    for p in sorted(base.rglob("*")):
        if p.is_dir() and pred(p):
            return p
    return None
mi = buscar_dir(lambda p: (p / "images").is_dir() and (p / "labels").is_dir() and (p / "dataset.yaml").exists()) \
     or buscar_dir(lambda p: (p / "images").is_dir() and (p / "labels").is_dir() and "coles" in str(p).lower())
nav = buscar_dir(lambda p: p.is_dir() and (p / "images").is_dir() and any(p.joinpath("images").glob("camino-*.jpg")))
neg = buscar_dir(lambda p: p.is_dir() and "negativos" in p.name.lower() and (p / "images").is_dir())
cajas = buscar_dir(lambda p: p.is_dir() and "cajas-plagas" in p.name.lower() and (p / "images" / "train").is_dir())
print("cajas reales (daño de hoja + mosca blanca):", cajas)
print("tu dataset de coles :", mi); print("navegación          :", nav); print("negativos de fondo  :", neg)
print("contenido de", ENTRADA, ":", [p.name for p in ENTRADA.iterdir()][:20] if ENTRADA.exists() else "no existe")

# ---------------- calidad de imagen ----------------
def apta(img, umbral_nitidez=60.0, umbral_sujeto=0.03):
    if img is None or min(img.shape[:2]) < 64: return False
    gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    nit = cv2.Laplacian(gris, cv2.CV_64F).var()
    _, m = cv2.threshold(gris, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    return nit >= umbral_nitidez and np.count_nonzero(m) / m.size >= umbral_sujeto

def leer_redim(ruta, lado=640):
    img = cv2.imread(str(ruta))
    if img is None: return None
    h, w = img.shape[:2]; k = lado / max(h, w)
    return cv2.resize(img, (int(w * k), int(h * k)), interpolation=cv2.INTER_AREA) if k < 1 else img

for sp in ("train", "val", "test"):
    (SALIDA / "images" / sp).mkdir(parents=True, exist_ok=True); (SALIDA / "labels" / sp).mkdir(parents=True, exist_ok=True)
registro = []                                              # (split, origen, clase o -1 (fondo), archivo)

def guardar(img, split, nombre, cajas):
    cv2.imwrite(str(SALIDA / "images" / split / f"{nombre}.jpg"), img)
    (SALIDA / "labels" / split / f"{nombre}.txt").write_text("".join(f"{c} {x} {y} {w} {h}\n" for c, x, y, w, h in cajas))

# ---------------- 1) TU DATASET (cajas automáticas por color: ver el análisis) ----------------
if mi is not None:
    for sp in ("train", "val", "test"):
        for f in sorted((mi / "images" / sp).glob("*.jpg")):
            lbl = mi / "labels" / sp / (f.stem + ".txt")
            cajas = [l.split() for l in lbl.read_text().splitlines() if l.strip()] if lbl.exists() else []
            if not cajas: continue
            img = leer_redim(f)
            if img is None: continue
            cajas = [(int(c[0]), *map(float, c[1:5])) for c in cajas]
            guardar(img, sp, "mio_" + f.stem, cajas)
            dom = Counter(c[0] for c in cajas).most_common(1)[0][0]
            registro.append((sp, "propio", dom, "mio_" + f.stem))
print("imágenes propias incluidas:", sum(1 for r in registro if r[1] == "propio"))

# ---------------- 2) EXTERNOS: se recorren todas las carpetas de /kaggle/input que no sean las propias ----------------
externas, no_mapeadas = [], Counter()
excluir = [p for p in (mi, nav) if p is not None]
for raiz, dirs, archivos in os.walk(ENTRADA, followlinks=True):
    rp = Path(raiz)
    if any(str(rp).startswith(str(e)) for e in excluir) or "agrobot-slam" in str(rp): continue
    imgs = [a for a in archivos if a.lower().endswith((".jpg", ".jpeg", ".png"))]
    if not imgs: continue
    k = MAPA.get(norm(rp.name))
    if k is None:
        no_mapeadas[rp.name] += len(imgs); continue
    externas += [(rp / a, k) for a in imgs]
print("carpetas externas sin mapear (se ignoran):", dict(no_mapeadas.most_common(12)))
por_clase = defaultdict(list)
for r, k in externas: por_clase[k].append(r)
sel = []
for k, rs in por_clase.items():
    random.shuffle(rs); n = 0
    for r in rs:
        if n >= MAX_POR_CLASE: break
        img = leer_redim(r)
        if apta(img): sel.append((img, k)); n += 1
random.shuffle(sel)
corte = [int(len(sel) * .7), int(len(sel) * .9)]
for i, (img, k) in enumerate(sel):
    sp = "train" if i < corte[0] else ("val" if i < corte[1] else "test")
    guardar(img, sp, f"ext_{i:05d}", [(k, .5, .5, 1.0, 1.0)])                 # imagen completa (clasificación, no detección real)
    registro.append((sp, "externo", k, f"ext_{i:05d}"))
print("imágenes externas incluidas:", len(sel))

# ---------------- 3) TU DATASET DE SENDEROS COMO FONDO (etiqueta vacía = «aquí no hay enfermedad») ----------------
if nav is not None:
    fotos = sorted((nav / "images").glob("camino-*.jpg"))
    paso = max(1, len(fotos) // MAX_FONDO)
    for j, f in enumerate(fotos[::paso][:MAX_FONDO]):
        img = leer_redim(f)
        if img is None: continue
        sp = "val" if j % 5 == 0 else "train"
        guardar(img, sp, f"fondo_{j:04d}", [])
        registro.append((sp, "fondo", -1, f"fondo_{j:04d}"))
print("imágenes de fondo (senderos):", sum(1 for r in registro if r[1] == "fondo"))

# ---------------- 3b) CAJAS REALES (v5, 2026-10-22): hojas de col dañadas/comidas (clase 3) y mosca blanca (clase 7) ----------------
# Cajas dibujadas a mano sobre la imagen (no de imagen completa): mejoran la localización. Las de mosca blanca son fotos de YUCA (fondo
# distinto a la col); las de daño, de col blanca (CC-BY-4.0, Arko007/white-cabbage-leaf-damage). Etiqueta vacía = hoja sin daño.
if cajas is not None:
    for sp in ("train", "val", "test"):
        for f in sorted((cajas / "images" / sp).glob("*.jpg")):
            lab = cajas / "labels" / sp / (f.stem + ".txt")
            img = cv2.imread(str(f))
            if img is None: continue
            filas = [l.split() for l in lab.read_text().splitlines() if len(l.split()) >= 5] if lab.exists() else []
            guardar(img, sp, f"caj_{f.stem}", [(int(c[0]), c[1], c[2], c[3], c[4]) for c in filas])
            registro.append((sp, "cajas", -2, f"caj_{f.stem}"))
print("imágenes con cajas reales:", sum(1 for r in registro if r[1] == "cajas"))

# ---------------- 4) NEGATIVOS DUROS DE INTERIOR (2026-10-22): fondos SIN planta que el detector marcó como plaga ----------------
# Pufs, puerta, parlante, zapatos y paredes que PLAGASX confundió con col_dano_insecto/col_malformacion al 77-95 % en la prueba real.
# Se usan TODOS y sin filtro de nitidez: lo borroso y oscuro de la cámara del robot es justo lo que debe aprender a ignorar.
if neg is not None:
    fotos = sorted((neg / "images").glob("*.jpg"))
    for j, f in enumerate(fotos):
        img = leer_redim(f)
        if img is None: continue
        sp = "val" if j % 5 == 0 else ("test" if j % 5 == 1 else "train")        # un test propio para medir los falsos positivos
        guardar(img, sp, f"negint_{j:04d}", [])
        registro.append((sp, "fondo_interior", -1, f"negint_{j:04d}"))
print("negativos duros de interior:", sum(1 for r in registro if r[1] == "fondo_interior"))

df = pd.DataFrame(registro, columns=["split", "origen", "clase", "archivo"])
df["clase_nombre"] = df.clase.map(lambda c: "cajas" if c == -2 else ("fondo" if c < 0 else CLASES[c]))
display(pd.crosstab(df.clase_nombre, [df.split, df.origen], margins=True))
(SALIDA / "dataset.yaml").write_text(f"path: {SALIDA}\ntrain: images/train\nval: images/val\ntest: images/test\nnc: {len(CLASES)}\nnames:\n" +
                                     "".join(f"  {i}: {n}\n" for i, n in enumerate(CLASES)))
fig, axs = plt.subplots(1, 2, figsize=(14, 4))
pd.crosstab(df.clase_nombre, df.origen).plot.bar(stacked=True, ax=axs[0]); axs[0].set_title("imágenes por clase y origen"); axs[0].grid(alpha=.3)
pd.crosstab(df.split, df.origen).plot.bar(stacked=True, ax=axs[1]); axs[1].set_title("imágenes por partición"); axs[1].grid(alpha=.3)
fig.tight_layout(); fig.savefig(RES / "01_composicion.png", dpi=110); plt.show()
if DRY: raise SystemExit("PLAGAS_DRY: dataset armado, sin entrenar")

# ---------------- 4) DETECTOR (YOLO) ----------------
from ultralytics import YOLO
det = YOLO("yolov8s.pt")
det.train(data=str(SALIDA / "dataset.yaml"), epochs=80, imgsz=800, batch=12, patience=15, close_mosaic=15, device=0, project=str(RES / "runs"), name="det",
          hsv_h=.015, hsv_s=.7, hsv_v=.4, flipud=.3, fliplr=.5, mosaic=1.0, mixup=.1, exist_ok=True, verbose=False)
mdet = det.val(data=str(SALIDA / "dataset.yaml"), split="test", project=str(RES / "runs"), name="det_test", exist_ok=True)
tabla_det = pd.DataFrame({"clase": [CLASES[int(i)] for i in mdet.box.ap_class_index], "mAP50": mdet.box.ap50.round(3), "mAP50-95": mdet.box.ap.round(3)})
print("\nDETECTOR (test) · mAP50 global:", round(float(mdet.box.map50), 3), "· mAP50-95:", round(float(mdet.box.map), 3), "(parte de las cajas son de imagen completa: orientativo; las clases 3 y 7 usan cajas reales)")
display(tabla_det)

shutil.copy(RES / "runs" / "det" / "weights" / "best.pt", RES / "coles_det_best.pt")      # el detector ya está a salvo
# Falsos positivos sobre los negativos de interior reservados para test (objetivo: 0 detecciones >= 0.45; el modelo anterior daba 77-95 %).
_neg_test = sorted((SALIDA / "images" / "test").glob("negint_*.jpg"))
if _neg_test:
    _fp = sum(1 for f in _neg_test if any(float(b) >= 0.45 for r in det.predict(str(f), verbose=False, conf=0.25) for b in r.boxes.conf))
    print(f"\nFALSOS POSITIVOS en {len(_neg_test)} fondos de interior (test): {_fp} ({100 * _fp / len(_neg_test):.1f} %)")
import traceback
try:
    # ---------------- 5) CLASIFICADOR (YOLO-cls): la métrica que vale ----------------
    CLS = RES / "cls"
    for sp in ("train", "val", "test"):
        for c in CLASES: (CLS / sp / c).mkdir(parents=True, exist_ok=True)
    for sp, org, k, a in registro:
        if k < 0: continue
        shutil.copy(SALIDA / "images" / sp / f"{a}.jpg", CLS / sp / CLASES[k] / f"{a}.jpg")
    for c in CLASES:                                             # clases vacías no admitidas por YOLO-cls: se quitan de las 3 particiones
        if not any((CLS / "train" / c).iterdir()):
            for sp in ("train", "val", "test"): shutil.rmtree(CLS / sp / c, ignore_errors=True)
            print("clase sin imágenes de entrenamiento, se omite en el clasificador:", c)
            continue
        for sp in ("val", "test"):
            if (CLS / sp / c).exists() and not any((CLS / sp / c).iterdir()): shutil.rmtree(CLS / sp / c, ignore_errors=True)
    clf = YOLO("yolov8s-cls.pt")
    clf.train(data=str(CLS), epochs=40, imgsz=224, batch=32, patience=10, device=0, project=str(RES / "runs"), name="cls", exist_ok=True, verbose=False)
    mcls = clf.val(data=str(CLS), split="test", project=str(RES / "runs"), name="cls_test", exist_ok=True)
    print("\nCLASIFICADOR (test) · top1:", round(float(mcls.top1), 3), "· top5:", round(float(mcls.top5), 3))
    nombres = [clf.names[i] for i in sorted(clf.names)]
    filas = []
    for c in nombres:
        fs = list((CLS / "test" / c).glob("*.jpg")) if (CLS / "test" / c).exists() else []
        if not fs: continue
        pr = [clf.predict(str(f), verbose=False)[0].probs.top1 for f in fs]
        filas.append({"clase": c, "imágenes_test": len(fs), "aciertos": sum(clf.names[p] == c for p in pr), "exactitud": round(sum(clf.names[p] == c for p in pr) / len(fs), 3)})
    tabla_cls = pd.DataFrame(filas); display(tabla_cls)
    fig, ax = plt.subplots(figsize=(8, 3.8)); ax.bar(tabla_cls.clase, tabla_cls.exactitud, color="#2a9d8f"); ax.set_ylim(0, 1)
    ax.set_title("exactitud por clase (test)"); ax.tick_params(axis="x", rotation=30); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(RES / "02_exactitud_clase.png", dpi=110); plt.show()
except Exception:
    (RES / "error_clasificador.txt").write_text(traceback.format_exc())
    print("ERROR EN EL CLASIFICADOR:\n", traceback.format_exc())
    clf = None
if clf is not None:
    shutil.copy(RES / "runs" / "cls" / "weights" / "best.pt", RES / "coles_cls_best.pt")
(RES / "resumen.json").write_text(json.dumps({"imagenes": df.groupby(["split", "origen"]).size().to_dict().__repr__(), "det_map50_test": float(mdet.box.map50),
                                              "cls_top1_test": (float(mcls.top1) if clf is not None else None), "por_clase": (filas if clf is not None else [])}, ensure_ascii=False, indent=1))
print("\nHECHO. Archivos en", RES, "(coles_det_best.pt, coles_cls_best.pt, resumen.json, gráficas)")
print("""LÍMITES: tus etiquetas son cajas automáticas por color (cubren ~50 % de la imagen) y los externos son de imagen completa: el DETECTOR no
aprende lesiones, solo la planta. Para detectar daños reales hacen falta cajas hechas a mano sobre las lesiones (empieza por las 348 imágenes
'rejected'). Las plantas de tu dataset de senderos entran como FONDO sano (sin etiqueta de enfermedad), no como 'col_sana' confirmada.""")
