"""Entrena y evalúa un detector YOLO11n sobre el dataset sintético (objetos o plagas).

Uso:  python3 entrenar_yolo.py --tarea obj    [--epocas 40] [--imgsz 320] [--batch 32]
      python3 entrenar_yolo.py --tarea plagas [--epocas 40] [--imgsz 640] [--batch 16]

Son DOS modelos distintos (navegación vs plagas), como exige el encargo. Se parte de
`yolo11n.pt` (COCO). Se evalúa en val, test y test_nuevo (dominio desplazado). Se guarda el
mejor por mAP50 de validación (lo hace ultralytics en best.pt).

LÍMITE: todo es sintético. En el caso de plagas los 'insectos' son sprites sencillos: esto valida
la cadena y mide el efecto de la distancia, NO la detección de plagas reales.
"""
import argparse
import json
import time
from pathlib import Path

AQUI = Path(__file__).resolve().parent
BASE_YOLO = AQUI.parents[1] / "jetson" / "backend" / "models" / "yolo11n.pt"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tarea", choices=("obj", "plagas"), required=True)
    ap.add_argument("--epocas", type=int, default=40)
    ap.add_argument("--imgsz", type=int, default=None)
    ap.add_argument("--batch", type=int, default=None)
    a = ap.parse_args()
    from ultralytics import YOLO
    carpeta = AQUI / ("yolo_obj" if a.tarea == "obj" else "yolo_plagas")
    imgsz = a.imgsz or (320 if a.tarea == "obj" else 640)
    batch = a.batch or (32 if a.tarea == "obj" else 16)
    t0 = time.perf_counter()
    modelo = YOLO(str(BASE_YOLO))
    modelo.train(data=str(carpeta / "datos.yaml"), imgsz=imgsz, epochs=a.epocas, batch=batch, device=0,
                 workers=3, project=str(AQUI / "modelos" / "yolo"), name=a.tarea, exist_ok=True,
                 patience=15, plots=False, degrees=0.0, flipud=0.0, fliplr=0.5, verbose=False, seed=0,
                 amp=True, cache="ram" if a.tarea == "obj" else False)
    mejor = AQUI / "modelos" / "yolo" / a.tarea / "weights" / "best.pt"
    net = YOLO(str(mejor))
    res = {"tarea": a.tarea, "imgsz": imgsz, "epocas": a.epocas, "base": "yolo11n.pt (COCO)",
           "segundos_entrenamiento": round(time.perf_counter() - t0)}
    for nombre, yaml, split in (("val", "datos.yaml", "val"), ("test", "datos.yaml", "test"),
                                ("test_nuevo", "datos_nuevo.yaml", "val")):
        m = net.val(data=str(carpeta / yaml), split=split, imgsz=imgsz, batch=batch, device=0,
                    plots=False, verbose=False)
        res[nombre] = {"mAP50": round(float(m.box.map50), 4), "mAP50_95": round(float(m.box.map), 4),
                       "precision": round(float(m.box.mp), 4), "recall": round(float(m.box.mr), 4),
                       "por_clase_mAP50": {net.names[int(i)]: round(float(v), 4)
                                           for i, v in zip(m.box.ap_class_index, m.box.ap50)},
                       "ms_por_imagen": {k: round(float(v), 1) for k, v in m.speed.items()}}
        print(nombre, res[nombre]["mAP50"], res[nombre]["por_clase_mAP50"], flush=True)
    res["NO_DEMOSTRADO"] = ("Entrenado y evaluado solo con imágenes sintéticas."
                            + (" Los 'insectos' son sprites sencillos: no demuestra detección de plagas reales."
                               if a.tarea == "plagas" else ""))
    (AQUI / "modelos" / f"yolo_{a.tarea}_metricas.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    print("guardado", AQUI / "modelos" / f"yolo_{a.tarea}_metricas.json", f"({time.perf_counter()-t0:.0f} s)")


if __name__ == "__main__":
    main()
