#!/usr/bin/env python3
"""Entrena un clasificador de la huerta con YOLO11n-cls: tarea `especie` (col/coliflor/acelga/fondo)
o `estado` (sana/marchita/dano_hoja/minador/mosca_blanca/fondo).

  python3 tools/coliflor_entrenar.py especie [--epochs 40] [--imgsz 480]
  python3 tools/coliflor_entrenar.py estado

Lee ~/.agrobot/dataset_planta/<tarea> (ver tools/coliflor_dataset.py). La validación se separa por SESIÓN
(todas las fotos de una sesión van juntas a train o a val): fotos seguidas de la misma hoja son casi
idénticas y, mezcladas, inflarían la precisión (el mismo error que ya tuvo entrenar.py). Con menos de
3 sesiones por clase se separa por bloques de tiempo y se avisa.
NO despliega nada: escribe ~/.agrobot/aprendizaje/coliflor/candidato_coliflor_cls.pt.
Un solo entrenamiento a la vez (la Jetson tiene 7,5 GB compartidos con la GPU).
"""
import argparse
import random
import re
import shutil
import sys
import time
from pathlib import Path

BASE = Path.home() / ".agrobot" / "dataset_planta"
SALIDA = Path.home() / ".agrobot" / "aprendizaje" / "coliflor"
TAREAS = {
    "especie": (("col", "coliflor", "acelga", "fondo"), ("col", "coliflor", "acelga")),
    "estado": (("sana", "marchita", "dano_hoja", "minador", "mosca_blanca", "fondo"), ("sana", "marchita")),
}
MIN_FOTOS = 30


def dividir(fotos, rng):
    """(train, val, aviso) separando por sesión; por bloques de tiempo si hay pocas sesiones."""
    ses = {}
    for f in fotos:
        ses.setdefault(f.name.split("__")[0], []).append(f)
    if len(ses) >= 3:
        nombres = sorted(ses)
        rng.shuffle(nombres)
        nval = max(1, round(len(nombres) * 0.2))
        val = [f for s in nombres[:nval] for f in ses[s]]
        tr = [f for s in nombres[nval:] for f in ses[s]]
        return tr, val, None
    tr, val = [], []
    for s, fs in ses.items():
        fs = sorted(fs)
        corte = max(1, int(len(fs) * 0.8))
        tr += fs[:corte]
        val += fs[corte:]
    return tr, val, "menos de 3 sesiones: val = el último 20 % de cada sesión (hay fuga parcial; captura más sesiones)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tarea", choices=tuple(TAREAS))
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--imgsz", type=int, default=480)
    ap.add_argument("--batch", type=int, default=8)
    a = ap.parse_args()

    CLASES, OBLIGATORIAS = TAREAS[a.tarea]
    BASE_T = BASE / a.tarea
    por_clase = {c: sorted((BASE_T / c).glob("*.jpg")) for c in CLASES if (BASE_T / c).exists()}
    faltan = [c for c in OBLIGATORIAS if len(por_clase.get(c, [])) < MIN_FOTOS]
    if faltan:
        sys.exit(f"Faltan fotos (mínimo {MIN_FOTOS} por clase) en: {', '.join(faltan)}. Mira: coliflor_dataset.py contar")
    clases = [c for c in CLASES if len(por_clase.get(c, [])) >= 10]

    rng = random.Random(20261019)
    sello = time.strftime("%Y%m%d_%H%M")
    ds = SALIDA / f"dataset_{a.tarea}_{sello}"
    avisos = []
    for c in clases:
        tr, val, av = dividir(por_clase[c], rng)
        if av:
            avisos.append(f"{c}: {av}")
        for parte, fs in (("train", tr), ("val", val)):
            (ds / parte / c).mkdir(parents=True, exist_ok=True)
            for f in fs:
                (ds / parte / c / f.name).symlink_to(f)
        print(f"{c:13s} train {len(tr):4d}  val {len(val):4d}")
    for av in avisos:
        print("AVISO", av)

    from ultralytics import YOLO
    base = "yolo11n-cls.pt"          # se descarga solo la primera vez (3 MB)
    modelo = YOLO(base)
    modelo.train(data=str(ds), epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, workers=2, patience=15,
                 device=0, project=str(SALIDA), name=f"run_{a.tarea}_{sello}", exist_ok=True,
                 hsv_h=0.02, hsv_s=0.5, hsv_v=0.4, degrees=15, fliplr=0.5, flipud=0.2, verbose=False)
    mejor = SALIDA / f"run_{a.tarea}_{sello}" / "weights" / "best.pt"
    if not mejor.exists():
        sys.exit("El entrenamiento no produjo best.pt")
    cand = SALIDA / f"candidato_{a.tarea}_cls.pt"
    shutil.copy2(mejor, cand)

    # Evaluación por clase sobre val
    m = YOLO(str(cand))
    ok, tot = {c: 0 for c in clases}, {c: 0 for c in clases}
    conf = {c: {} for c in clases}
    for c in clases:
        for f in sorted((ds / "val" / c).glob("*.jpg")):
            r = m.predict(str(f), imgsz=a.imgsz, device=0, verbose=False)[0]
            p = r.names[int(r.probs.top1)]
            tot[c] += 1
            ok[c] += p == c
            if p != c:
                conf[c][p] = conf[c].get(p, 0) + 1
    print("\n=== PRECISIÓN POR CLASE (validación, sesiones nunca vistas) ===")
    for c in clases:
        pct = 100.0 * ok[c] / max(1, tot[c])
        print(f"{c:13s} {ok[c]:3d}/{tot[c]:3d} = {pct:5.1f} %   confunde con: {conf[c] or '-'}")
    print(f"\nCandidato: {cand}  (NO desplegado)")


if __name__ == "__main__":
    main()
