"""Clasificador de PLANTAS MEDICINALES en la GPU (2026-10-24): une los dos datasets descargados en datos_plantas_medicinales/
(Oriental Medicinal Herb: 70 hierbas, nombres vietnamitas, CC-BY-NC-SA-4.0; Indian Medicinal Plant: carpeta por especie) y entrena yolo11-cls.

Validación con fotos RESERVADAS de los mismos datasets (15 % por clase): dice cuánto acierta el modelo en fotos como las del dataset, NO en tus plantas
con la cámara del robot. Uso: python3 entrenar_medicinales.py [--epocas 30] [--modelo yolo11n-cls.pt]
Salida: modelos/medicinales_metricas.json y modelos/yolo/medicinales/weights/best.pt"""
import argparse, json, random, re, shutil, time, unicodedata
from pathlib import Path

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parents[1]
DATOS = RAIZ / "datos_plantas_medicinales"
SAL = Path.home() / ".agrobot" / "dataset_medicinales"
EXT = {".jpg", ".jpeg", ".png", ".webp"}


def limpio(n: str) -> str:
    n = unicodedata.normalize("NFKD", n).encode("ascii", "ignore").decode()
    return re.sub(r"[^A-Za-z0-9_]+", "_", n).strip("_").lower() or "clase"


def clases():
    """{nombre_clase: [rutas]} de las dos fuentes."""
    out = {}
    for fuente, base, pref in (("oriental", DATOS / "oriental_medicinal_herb" / "herb_dataset", "or"),
                               ("indian", DATOS / "indian_medicinal_plant" / "Medicinal plant dataset", "in")):
        if not base.exists():
            continue
        for d in sorted(p for p in base.iterdir() if p.is_dir()):
            fs = [f for f in d.rglob("*") if f.suffix.lower() in EXT]
            if len(fs) >= 8:
                out[f"{pref}_{limpio(d.name)}"] = fs
    return out


def preparar(seed=0):
    rng = random.Random(seed)
    shutil.rmtree(SAL, ignore_errors=True)
    cl = clases()
    n = {"clases": len(cl), "train": 0, "val": 0}
    for nombre, fs in cl.items():
        rng.shuffle(fs)
        nv = max(2, int(0.15 * len(fs)))
        for i, f in enumerate(fs):
            sp = "val" if i < nv else "train"
            d = SAL / sp / nombre
            d.mkdir(parents=True, exist_ok=True)
            try:
                shutil.copy(f, d / f"{i:05d}{f.suffix.lower()}")
                n[sp] += 1
            except OSError:
                pass
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epocas", type=int, default=30)
    ap.add_argument("--modelo", default="yolo11n-cls.pt")
    ap.add_argument("--batch", type=int, default=64)
    a = ap.parse_args()
    from ultralytics import YOLO
    t0 = time.time()
    n = preparar()
    print("dataset:", n, flush=True)
    m = YOLO(a.modelo)
    m.train(data=str(SAL), epochs=a.epocas, imgsz=224, batch=a.batch, workers=2, device=0, project=str(AQUI / "modelos" / "yolo"), name="medicinales",
            exist_ok=True, patience=10, plots=False, verbose=False)
    best = YOLO(str(AQUI / "modelos" / "yolo" / "medicinales" / "weights" / "best.pt"))
    v = best.val(data=str(SAL), imgsz=224, device=0, plots=False, verbose=False)
    res = {"datos": n, "top1": round(float(v.top1), 4), "top5": round(float(v.top5), 4), "segundos": round(time.time() - t0), "modelo": a.modelo,
           "NO_DEMOSTRADO": "validación con fotos reservadas de los mismos datasets; no con tus plantas ni la cámara del robot.",
           "licencias": "Oriental Medicinal Herb: CC-BY-NC-SA-4.0 (solo uso no comercial)."}
    (AQUI / "modelos" / "medicinales_metricas.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    print(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
