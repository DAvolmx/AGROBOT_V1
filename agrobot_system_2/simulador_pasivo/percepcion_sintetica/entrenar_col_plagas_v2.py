"""Detector de COL v2 (2026-10-24): planta, cogollo, daño de hoja de masticadores y PULGÓN, con negativos de fondo.

Mejora sobre entrenar_col.py (dano_hoja mAP50 = 0,17, sin pulgón, falsos positivos en fondos de interior):
  · pulgón: cajas sacadas de las máscaras de `aphid_generalization` (Kaggle chahineeee/aphid-generalization, 626 fotos de áfidos en hojas);
  · 272 fondos sin planta del robot (negativos_fondo_20261022, etiqueta vacía) para bajar los falsos positivos;
  · col_dano (HF Arko007/white-cabbage-leaf-damage, CC-BY-4.0) como antes.
NO DEMOSTRADO en las coles reales del operador: se mide además cuántas cajas dibuja sobre sus 74 fotos y sobre los fondos reservados.
Uso: python3 entrenar_col_plagas_v2.py [--epocas 40] [--batch 16] [--modelo yolo11n.pt]   Salida: modelos/col_plagas_v2_metricas.json"""
import argparse, glob, json, random, shutil, time
from pathlib import Path

import cv2
import numpy as np

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parents[1]
INT = RAIZ / "datos_plagas_reales" / "internet"
SAL = Path.home() / ".agrobot" / "dataset_col_plagas_v2"
NOMBRES = ["col_planta", "col_cogollo", "dano_hoja", "pulgon"]
NEG = Path.home() / ".agrobot" / "dataset_planta" / "negativos_fondo_20261022"
FOTOS_OP = Path.home() / ".agrobot" / "datos_usuario" / "20261022_subida"


def _poner(split, nombre, img, lineas):
    (SAL / "images" / split).mkdir(parents=True, exist_ok=True)
    (SAL / "labels" / split).mkdir(parents=True, exist_ok=True)
    d = SAL / "images" / split / f"{nombre}.jpg"
    if not d.exists():
        shutil.copy(img, d)
    (SAL / "labels" / split / f"{nombre}.txt").write_text("\n".join(lineas) + ("\n" if lineas else ""))


def cajas_de_mascara(ruta, min_area=300, dil=9):
    m = cv2.imread(str(ruta), -1)
    if m is None:
        return []
    if m.ndim == 3:
        m = m[..., 0]
    h, w = m.shape
    mm = cv2.dilate((m > 127).astype(np.uint8), np.ones((dil, dil), np.uint8))
    cs, _ = cv2.findContours(mm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cs:
        x, y, bw, bh = cv2.boundingRect(c)
        if bw * bh < min_area or bw * bh > 0.6 * w * h:
            continue
        out.append(f"3 {(x + bw / 2) / w:.5f} {(y + bh / 2) / h:.5f} {bw / w:.5f} {bh / h:.5f}")
    return out


def preparar():
    rng = random.Random(0)
    n = {"col": 0, "pulgon": 0, "neg": 0}
    imgs = sorted(glob.glob(str(INT / "col_dano" / "data" / "Raw images (JPEG format)" / "*.jpg")))
    rng.shuffle(imgs)
    nv = max(40, len(imgs) // 10)
    for i, im in enumerate(imgs):
        lab = INT / "col_dano" / "data" / "Annotated labels (YOLO format, TXT files)" / f"{Path(im).stem}.txt"
        if not lab.exists():
            continue
        ls = [l.strip() for l in lab.read_text().splitlines() if len(l.split()) == 5]
        _poner("val" if i < nv else "train", "col_" + Path(im).stem, im, ls)
        n["col"] += 1
    base = INT / "aphid_generalization" / "data_generalization"
    pul = []
    for carpeta in ("Aphid Detection Dataset", "Pest Detection Dataset"):
        for im in sorted(glob.glob(str(base / carpeta / "image" / "*.jpg"))):
            pul.append((carpeta, im))
    rng.shuffle(pul)
    for i, (c, im) in enumerate(pul):
        ls = cajas_de_mascara(Path(im.replace("/image/", "/mask/").replace(".jpg", ".png")))
        if not ls:
            continue
        _poner("val" if i < len(pul) // 10 else "train", f"pul_{Path(im).stem}"[:80], im, ls)
        n["pulgon"] += 1
    neg = sorted(glob.glob(str(NEG / "images" / "*")))
    rng.shuffle(neg)
    for i, im in enumerate(neg):
        _poner("val" if i < len(neg) // 8 else "train", f"neg_{Path(im).stem}", im, [])
        n["neg"] += 1
    (SAL / "data.yaml").write_text(f"path: {SAL}\ntrain: images/train\nval: images/val\nnames:\n" +
                                   "".join(f"  {i}: {x}\n" for i, x in enumerate(NOMBRES)))
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epocas", type=int, default=40)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--modelo", default=str(RAIZ / "jetson" / "backend" / "models" / "yolo11n.pt"))
    ap.add_argument("--nombre", default="col_plagas_v2")
    a = ap.parse_args()
    from ultralytics import YOLO
    n = preparar()
    print("dataset:", n, flush=True)
    t0 = time.time()
    m = YOLO(a.modelo)
    m.train(data=str(SAL / "data.yaml"), epochs=a.epocas, imgsz=640, batch=a.batch, workers=2, device=0, project=str(AQUI / "modelos" / "yolo"),
            name=a.nombre, exist_ok=True, patience=12, cache=False, plots=False, verbose=False, amp=True)
    best = AQUI / "modelos" / "yolo" / a.nombre / "weights" / "best.pt"
    mb = YOLO(str(best))
    v = mb.val(data=str(SAL / "data.yaml"), split="val", imgsz=640, device=0, plots=False, verbose=False)
    por = {NOMBRES[int(c)]: round(float(x), 3) for c, x in zip(v.box.ap_class_index, v.box.ap50)}
    # falsos positivos: cajas por foto sobre fondos reservados (val de negativos) y sobre las coles reales del operador
    def cajas(carpeta, conf=0.35):
        fs = sorted(glob.glob(str(carpeta / "*.*")))[:120]
        tot, por_cls = 0, {k: 0 for k in NOMBRES}
        for r in mb.predict(fs, conf=conf, imgsz=640, device=0, verbose=False):
            for c in r.boxes.cls.tolist():
                tot += 1
                por_cls[NOMBRES[int(c)]] += 1
        return {"fotos": len(fs), "cajas": tot, "por_clase": por_cls}
    fp = {"fondos_val": cajas(SAL / "images" / "val", 0.35) if False else None}
    neg_val = [p for p in glob.glob(str(SAL / "images" / "val" / "neg_*.jpg"))]
    tot = sum(len(r.boxes) for r in mb.predict(neg_val, conf=0.35, imgsz=640, device=0, verbose=False)) if neg_val else 0
    fp = {"fondos_val": {"fotos": len(neg_val), "cajas": tot, "cajas_por_foto": round(tot / max(1, len(neg_val)), 3)},
          "coles_operador_74": cajas(FOTOS_OP)}
    res = {"modelo": str(best), "datos": n, "mAP50": round(float(v.box.map50), 3), "mAP50_95": round(float(v.box.map), 3),
           "precision": round(float(v.box.mp), 3), "recall": round(float(v.box.mr), 3), "por_clase_mAP50": por, "falsos_positivos": fp,
           "previo_col_10h": {"mAP50": 0.591, "dano_hoja": 0.171}, "segundos": round(time.time() - t0),
           "NO_DEMOSTRADO": "validación con fotos del mismo origen; las coles del operador no están etiquetadas (solo se cuentan cajas)."}
    (AQUI / "modelos" / f"{a.nombre}_metricas.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    print(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
