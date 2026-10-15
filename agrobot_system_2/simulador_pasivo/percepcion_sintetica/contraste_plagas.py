"""Contraste honesto de la detección de plagas: sintético vs real.

  A) El detector entrenado con SPRITES sintéticos, sobre fotos REALES de plagas (captures/).
  B) Los detectores REALES del proyecto (agrobot_plagas_det.pt, ip102_pest_yolo11s.pt) sobre las
     imágenes SINTÉTICAS de test.

Se cuenta en cuántas imágenes hay al menos una detección con confianza >= 0,25. NO hay etiquetas
en las fotos reales (solo el nombre de la especie en el fichero), así que esto no mide precisión:
mide si los modelos se reconocen entre dominios. Se espera que NO: eso es lo que demuestra que
el detector sintético no sirve para plagas reales y que hay que conservar el real.
"""
import glob
import json
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
MODELOS_REALES = AQUI.parents[1] / "jetson" / "backend" / "models"
REALES = AQUI.parents[1] / "jetson" / "backend" / "captures"


def contar(modelo, ficheros, imgsz, conf=0.25):
    from ultralytics import YOLO
    net = YOLO(str(modelo))
    con, total_det, clases = 0, 0, {}
    for f in ficheros:
        r = net.predict(f, imgsz=imgsz, conf=conf, verbose=False)[0]
        n = len(r.boxes)
        con += n > 0
        total_det += n
        for b in r.boxes:
            clases[r.names[int(b.cls)]] = clases.get(r.names[int(b.cls)], 0) + 1
    return {"imagenes": len(ficheros), "con_deteccion": int(con), "pct": round(100 * con / max(len(ficheros), 1), 1),
            "detecciones": int(total_det), "clases_top": dict(sorted(clases.items(), key=lambda kv: -kv[1])[:4])}


def main():
    rng = np.random.default_rng(0)
    reales = sorted(glob.glob(str(REALES / "*.jpg")))
    reales = [reales[i] for i in rng.choice(len(reales), 120, replace=False)]
    sint = sorted(glob.glob(str(AQUI / "yolo_plagas" / "images" / "test" / "*.jpg")))
    sint = [sint[i] for i in rng.choice(len(sint), 120, replace=False)]
    out = {"fotos_reales_usadas": len(reales), "sinteticas_usadas": len(sint)}
    sint_det = AQUI / "modelos" / "yolo" / "plagas" / "weights" / "best.pt"
    if sint_det.exists():
        out["A_detector_sintetico_sobre_fotos_reales"] = contar(sint_det, reales, 640)
        out["control_detector_sintetico_sobre_sinteticas"] = contar(sint_det, sint, 640)
    for nombre in ("agrobot_plagas_det.pt", "ip102_pest_yolo11s.pt"):
        m = MODELOS_REALES / nombre
        if m.exists():
            out[f"B_{nombre}_sobre_sinteticas"] = contar(m, sint, 640)
            out[f"control_{nombre}_sobre_fotos_reales"] = contar(m, reales, 640)
    (AQUI / "modelos" / "contraste_plagas.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
    for k, v in out.items():
        print(k, v)


if __name__ == "__main__":
    main()
