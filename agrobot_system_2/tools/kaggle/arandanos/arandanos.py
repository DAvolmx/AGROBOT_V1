# AGROBOT · ARANDANOS v1 (2026-11-11): detector de frutos de arándano (maduro / inmaduro) y plagas que atacan al arándano.
# Fuentes (todas con CAJAS reales, no de imagen completa):
#   frutos  : zhengkunli3969/blueberry-detection-dataset (HANDset, nombres cruzados: madurez por color), raulcabrerasoles/visual-computer-blueberry
#             (ripe/unripe) y cnavarrete/blueberry_dataset de HuggingFace (Unripe/Pint/Ripe; se baja con internet).
#   plagas  : IP102 (cubeai/insect-pest-detection-for-yolo), solo especies que atacan arándano o se le parecen, agrupadas.
#   fondo   : hojas sanas de arándano de PlantVillage + senderos propios ('navegacion'): etiqueta vacía = "aquí no hay nada".
# LÍMITE: ninguna foto es de TU invernadero. Las plagas del IP102 son fotos de internet tomadas de cerca; la mosca de alas manchadas
# (Drosophila suzukii) no está: 'mosca_fruta' son tefrítidos (Bactrocera/Dacus), parientes de Ceratitis, no de D. suzukii.
import os, sys, json, random, shutil, subprocess, re, time, traceback
from pathlib import Path
from collections import Counter, defaultdict
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import cv2
import yaml
try:
    from IPython.display import display
except Exception:
    display = print

DRY = bool(os.environ.get("ARANDANOS_DRY"))                     # solo arma el dataset (prueba local, sin entrenar)
ENTRADA = Path(os.environ.get("ARANDANOS_INPUT", "/kaggle/input"))
RES = Path(os.environ.get("ARANDANOS_SALIDA", "/kaggle/working"))
SALIDA = RES / "dataset_arandanos"
random.seed(42); np.random.seed(42)
if not DRY:
    subprocess.run([sys.executable, "-m", "pip", "install", "-q", "ultralytics", "huggingface_hub"], check=False)

CLASES = ["arandano_maduro", "arandano_inmaduro", "pulgon", "trips", "aranita_roja", "chanchito_blanco",
          "conchuela", "oruga", "mosca_blanca", "mosca_fruta"]
# índice IP102 (orden del data.yaml de cubeai, verificado contra la lista en inglés el 2026-11-11) -> clase propia
IP = {}
for k, ids in {2: [24, 27, 28, 29, 52, 89, 90, 91], 3: [12, 33, 53, 54, 92], 4: [21, 32, 74], 5: [64, 76, 81],
               6: [77, 78, 79, 80], 7: [0, 1, 18, 19, 20, 23, 38, 39, 86, 97], 8: [71, 82], 9: [83, 84, 85]}.items():
    for i in ids: IP[i] = k
MAX_IP_TRAIN, MAX_IP_EVAL, MAX_HOJAS, MAX_FONDO = 600, 80, 400, 150
norm = lambda s: re.sub(r"[^a-z0-9]", "", str(s).lower())

def leer_redim(ruta, lado=1280):
    img = cv2.imread(str(ruta))
    if img is None: return None
    h, w = img.shape[:2]; k = lado / max(h, w)
    return cv2.resize(img, (int(w * k), int(h * k)), interpolation=cv2.INTER_AREA) if k < 1 else img

for sp in ("train", "val", "test"):
    (SALIDA / "images" / sp).mkdir(parents=True, exist_ok=True); (SALIDA / "labels" / sp).mkdir(parents=True, exist_ok=True)
registro = []                                                    # (split, origen, nombre, n_cajas por clase)

def guardar(img, split, nombre, cajas, origen):
    cv2.imwrite(str(SALIDA / "images" / split / f"{nombre}.jpg"), img)
    (SALIDA / "labels" / split / f"{nombre}.txt").write_text("".join(f"{c} {x} {y} {w} {h}\n" for c, x, y, w, h in cajas))
    registro.append((split, origen, nombre, Counter(c for c, *_ in cajas)))

def raices_yolo(base):
    """Carpetas con data.yaml (formato YOLO de Roboflow)."""
    return sorted({p.parent for p in base.rglob("data.yaml")}) if base.exists() else []

# ---------------- 0) cnavarrete/blueberry_dataset desde HuggingFace ----------------
HF_DIR = RES / "hf_cnavarrete"
def bajar_hf(repo, destino):
    """Descarga archivo por archivo con la API pública (sin depender de la versión de huggingface_hub)."""
    import urllib.request, urllib.parse
    arbol = json.load(urllib.request.urlopen(f"https://huggingface.co/api/datasets/{repo}/tree/main?recursive=1", timeout=60))
    n = 0
    for x in arbol:
        if x.get("type") != "file" or not re.search(r"\.(jpe?g|png|txt|yaml)$", x["path"], re.I): continue
        d = destino / x["path"]
        if d.exists(): continue
        d.parent.mkdir(parents=True, exist_ok=True)
        try:
            urllib.request.urlretrieve(f"https://huggingface.co/datasets/{repo}/resolve/main/{urllib.parse.quote(x['path'])}", d); n += 1
        except Exception as e:
            print("   falló", x["path"], e)
    return n
if not DRY or os.environ.get("ARANDANOS_HF"):
    try:
        print("cnavarrete/blueberry_dataset: archivos bajados", bajar_hf("cnavarrete/blueberry_dataset", HF_DIR))
    except Exception as e:
        print("no se pudo bajar cnavarrete/blueberry_dataset:", e)

# ---------------- 1) FRUTOS ----------------
# La MADUREZ se decide por el COLOR de cada clase de origen, no por su nombre: en HANDset la clase '0' resultó verde y la 'immature'
# azul pálido con pruina (nombres cruzados en Roboflow; revisado a ojo el 2026-11-11). Regla por tono mediano (H de OpenCV, 0-180):
# H <= 45 (verde/amarillo) o H > 150 (rosado, "pinta") -> inmaduro; 46-150 (cian pálido por la pruina, azul, violeta) -> maduro.
# La saturación NO sirve: los verdes desenfocados de HANDset salen tan poco saturados como los maduros con pruina. Las fuentes de una sola clase ('0', 'blueberry') no dicen madurez y se omiten.
# Las particiones de origen no se respetan: los subconjuntos de Roboflow repiten fotos entre sí. Se deduplica por hash de la imagen
# (8x8 en gris) y la partición sale del hash (10 % test, 10 % val), así una foto repetida nunca cae en train y en test a la vez.
def ahash(img):
    g = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (8, 8), interpolation=cv2.INTER_AREA)
    return "%016x" % int("".join("1" if v else "0" for v in (g > g.mean()).flatten()), 2)
def es_ip102(r):
    return "蚜虫" in (r / "data.yaml").read_text(encoding="utf-8", errors="ignore")
def leer_filas(lab):
    filas = [l.split() for l in lab.read_text().splitlines() if len(l.split()) >= 5] if lab.exists() else []
    out = []
    for c in filas:
        if len(c) > 5:                                                 # polígono -> caja
            xs, ys = list(map(float, c[1::2])), list(map(float, c[2::2]))
            out.append((int(float(c[0])), (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, max(xs) - min(xs), max(ys) - min(ys)))
        else:
            out.append((int(float(c[0])), *map(float, c[1:5])))
    return out
def pares(raiz):
    for dimg in sorted(raiz.rglob("images")):
        if not dimg.is_dir(): continue
        for f in sorted(dimg.iterdir()):
            if f.suffix.lower() in (".jpg", ".jpeg", ".png"):
                yield f, dimg.parent / "labels" / (f.stem + ".txt")
def tono_cajas(img, cajas, por_clase):
    h, w = img.shape[:2]; hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    for c, x, y, bw, bh in cajas[:8]:
        x0, y0, x1, y1 = int((x - bw / 4) * w), int((y - bh / 4) * h), int((x + bw / 4) * w), int((y + bh / 4) * h)
        roi = hsv[max(0, y0):max(y0 + 1, y1), max(0, x0):max(x0 + 1, x1)]
        if roi.size: por_clase[c].append((float(np.median(roi[..., 0])), float(np.median(roi[..., 1]))))

vistos, tabla_madurez = set(), []
fuentes_fruto = [r for r in raices_yolo(ENTRADA) + raices_yolo(HF_DIR) if not es_ip102(r) and "cluster" not in str(r).lower()]
for raiz in fuentes_fruto:
    nombres = yaml.safe_load((raiz / "data.yaml").read_text(encoding="utf-8"))["names"]
    nombres = list(nombres.values()) if isinstance(nombres, dict) else list(nombres)
    if len(nombres) < 2:
        print(f"frutos: {raiz.name} clases={nombres} -> se OMITE (sin madurez)"); continue
    hues = defaultdict(list)                                           # 1ª pasada: color de cada clase de origen
    for j, (f, lab) in enumerate(pares(raiz)):
        if j >= 80: break
        cajas = leer_filas(lab)
        if cajas: tono_cajas(leer_redim(f), cajas, hues)
    mapa = {}
    for i, n in enumerate(nombres):
        if len(hues[i]) < 10: mapa[i] = None; continue                 # sin muestras suficientes: imágenes con esa clase se descartan
        hm = float(np.median([a for a, _ in hues[i]])); sm = float(np.median([b for _, b in hues[i]]))
        mapa[i] = 1 if (hm <= 45 or hm > 150) else 0              # verde-amarillo o rosado (pinta) -> inmaduro; cian-azul-violeta -> maduro
        tabla_madurez.append({"fuente": raiz.name, "clase_origen": n, "cajas_medidas": len(hues[i]), "tono_H": round(hm, 1),
                              "saturación_S": round(sm, 1), "-> clase": CLASES[mapa[i]]})
    print(f"frutos: {raiz.name} clases={nombres} -> {[CLASES[v] if v is not None else 'DESCARTA' for v in mapa.values()]}")
    if all(v is None for v in mapa.values()):
        print("   sin clases medibles: se OMITE"); continue
    tag = re.sub(r"[^a-z0-9]", "", raiz.parent.name.lower() + raiz.name.lower())[-12:]
    n_ok = n_desc = n_dup = 0
    for f, lab in pares(raiz):                                         # 2ª pasada: guardar
        cajas = leer_filas(lab)
        if not cajas or any(mapa.get(c[0]) is None for c in cajas): n_desc += 1; continue   # sin etiqueta: puede tener frutos sin marcar
        img = leer_redim(f)
        if img is None: continue
        h = ahash(img)
        if h in vistos: n_dup += 1; continue
        vistos.add(h)
        sp = ("test", "val", "train", "train", "train", "train", "train", "train", "train", "train")[int(h[:8], 16) % 10]
        guardar(img, sp, f"fr_{tag}_{f.stem[:40]}", [(mapa[c], x, y, w, hh) for c, x, y, w, hh in cajas], "frutos:" + raiz.name[:24]); n_ok += 1
    print(f"   imágenes usadas {n_ok}, descartadas {n_desc}, duplicadas {n_dup}")
print("\nMADUREZ ASIGNADA POR COLOR (H 46-150 = maduro):"); display(pd.DataFrame(tabla_madurez))

# ---------------- 2) PLAGAS DEL IP102 ----------------
ip102 = next((r for r in raices_yolo(ENTRADA) if es_ip102(r)), None)
print("\nIP102:", ip102)
if ip102 is not None:
    for sp_src, destinos in (("train", ("train",)), ("valid", ("val", "test"))):
        dimg = next((d for d in (ip102 / sp_src / "images", ip102 / sp_src) if d.is_dir()), None)
        if dimg is None: print("IP102: falta", sp_src); continue
        dlab = dimg.parent / "labels" if dimg.name == "images" else dimg
        por_clase = defaultdict(list)
        for lab in sorted(dlab.glob("*.txt")):
            filas = [l.split() for l in lab.read_text().splitlines() if len(l.split()) >= 5]
            if not filas or any(int(c[0]) not in IP for c in filas): continue    # foto con otro insecto sin mapear: fuera
            por_clase[IP[int(filas[0][0])]].append((lab.stem, [(IP[int(c[0])], *map(float, c[1:5])) for c in filas]))
        tope = MAX_IP_TRAIN if sp_src == "train" else 2 * MAX_IP_EVAL
        for k, lst in por_clase.items():
            random.shuffle(lst)
            for j, (stem, cajas) in enumerate(lst[:tope]):
                f = next((dimg / (stem + e) for e in (".jpg", ".jpeg", ".png", ".JPG") if (dimg / (stem + e)).exists()), None)
                img = leer_redim(f, 960) if f is not None else None
                if img is None: continue
                guardar(img, destinos[j % len(destinos)], f"ip_{k}_{stem}", cajas, "ip102")

# ---------------- 3) FONDO: hojas sanas de arándano (PlantVillage) y senderos propios ----------------
hojas = [p for p in ENTRADA.rglob("*") if p.is_file() and p.suffix.lower() in (".jpg", ".jpeg", ".png")
         and "blueberry" in str(p.parent).lower() and "healthy" in str(p.parent).lower()]
random.shuffle(hojas)
for j, f in enumerate(hojas[:MAX_HOJAS]):
    img = leer_redim(f)
    if img is not None: guardar(img, ("val", "test")[j % 2] if j % 5 == 0 else "train", f"hoja_{j:04d}", [], "fondo:hoja_arandano")
nav = next((p for p in sorted(ENTRADA.rglob("images")) if p.is_dir() and any(p.glob("camino-*.jpg"))), None)
if nav is not None:
    fotos = sorted(nav.glob("camino-*.jpg")); paso = max(1, len(fotos) // MAX_FONDO)
    for j, f in enumerate(fotos[::paso][:MAX_FONDO]):
        img = leer_redim(f)
        if img is not None: guardar(img, "val" if j % 5 == 0 else "train", f"sendero_{j:04d}", [], "fondo:sendero")

# ---------------- resumen del dataset ----------------
df = pd.DataFrame([{"split": s, "origen": o, "imagen": n, **{CLASES[k]: v for k, v in c.items()}} for s, o, n, c in registro]).fillna(0)
print("\nimágenes por origen y partición:"); display(pd.crosstab(df.origen, df.split, margins=True))
cols = [c for c in CLASES if c in df]
print("\ncajas por clase y partición:"); display(df.groupby("split")[cols].sum().T.astype(int))
(SALIDA / "dataset.yaml").write_text(f"path: {SALIDA}\ntrain: images/train\nval: images/val\ntest: images/test\nnc: {len(CLASES)}\nnames:\n" +
                                     "".join(f"  {i}: {n}\n" for i, n in enumerate(CLASES)))
fig, ax = plt.subplots(figsize=(10, 4)); df.groupby("split")[cols].sum().T.plot.bar(ax=ax); ax.set_title("cajas por clase")
ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(RES / "01_composicion.png", dpi=110); plt.show()
if DRY: raise SystemExit("ARANDANOS_DRY: dataset armado, sin entrenar")

# ---------------- 4) DETECTOR ----------------
from ultralytics import YOLO
import torch
ngpu = torch.cuda.device_count(); dev = list(range(ngpu)) if ngpu > 1 else (0 if ngpu else "cpu")
print("GPUs:", ngpu)
det = YOLO("yolo11s.pt")
det.train(data=str(SALIDA / "dataset.yaml"), epochs=250, time=5.0, imgsz=960, batch=16 * max(1, ngpu), patience=40, close_mosaic=15,
          device=dev, project=str(RES / "runs"), name="det", exist_ok=True, verbose=False,
          hsv_h=.015, hsv_s=.6, hsv_v=.4, fliplr=.5, flipud=.0, mosaic=1.0, mixup=.05, scale=.6)
shutil.copy(RES / "runs" / "det" / "weights" / "best.pt", RES / "arandanos_det_best.pt")     # a salvo antes de evaluar
mdet = det.val(data=str(SALIDA / "dataset.yaml"), split="test", imgsz=960, project=str(RES / "runs"), name="det_test", exist_ok=True)
tabla = pd.DataFrame({"clase": [CLASES[int(i)] for i in mdet.box.ap_class_index], "mAP50": mdet.box.ap50.round(3),
                      "mAP50-95": mdet.box.ap.round(3), "precisión": np.round(mdet.box.p, 3), "recall": np.round(mdet.box.r, 3)})
print("\nDETECTOR (test) · mAP50:", round(float(mdet.box.map50), 3), "· mAP50-95:", round(float(mdet.box.map), 3)); display(tabla)

# falsos positivos sobre fondos de test (hojas sanas): objetivo 0 detecciones >= 0.45
fondos = sorted((SALIDA / "images" / "test").glob("hoja_*.jpg"))
fp = sum(1 for f in fondos if any(float(b) >= 0.45 for r in det.predict(str(f), verbose=False, conf=0.25, imgsz=960) for b in r.boxes.conf))
print(f"falsos positivos en {len(fondos)} hojas sanas de test: {fp}")
# muestra visual
muestras = [f for f in sorted((SALIDA / "images" / "test").glob("fr_*.jpg"))][:6] + [f for f in sorted((SALIDA / "images" / "test").glob("ip_*.jpg"))][:6]
fig, axs = plt.subplots(3, 4, figsize=(18, 12))
for ax, f in zip(axs.flat, muestras):
    ax.imshow(det.predict(str(f), verbose=False, conf=0.3, imgsz=960)[0].plot()[..., ::-1]); ax.axis("off"); ax.set_title(f.name[:30], fontsize=8)
fig.tight_layout(); fig.savefig(RES / "02_ejemplos_test.png", dpi=90); plt.show()

(RES / "resumen.json").write_text(json.dumps({"version": "arandanos_v1_20261111", "clases": CLASES, "imgsz": 960,
    "det_map50_test": float(mdet.box.map50), "det_map50_95_test": float(mdet.box.map), "por_clase": tabla.to_dict("records"),
    "fp_hojas_sanas": [fp, len(fondos)], "imagenes": {f"{s}|{o}": int(n) for (s, o), n in df.groupby(["split", "origen"]).size().items()}},
    ensure_ascii=False, indent=1))
print("\nHECHO. arandanos_det_best.pt, resumen.json y gráficas en", RES)
