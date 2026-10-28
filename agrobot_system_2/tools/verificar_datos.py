#!/usr/bin/env python3
"""Verifica ~/.agrobot/datos (fotos vacías/ilegibles, scans corruptos, telemetría con ceros) y escribe el resumen en ~/.agrobot/aprendizaje/verificacion_datos.json."""
import glob, json, os
import cv2, numpy as np

R = os.path.expanduser("~/.agrobot/datos"); tot = dict(sesiones=0, fotos=0, fotos_vacias=0, scans=0, scans_malos=0, telemetria=0, telemetria_ceros=0)
for d in sorted(glob.glob(R + "/2026*_*")):
    fr = glob.glob(d + "/camara/*.jpg"); tot["sesiones"] += 1; tot["fotos"] += len(fr)
    tot["fotos_vacias"] += sum(1 for f in fr if os.path.getsize(f) == 0)
    for f in glob.glob(d + "/scans_*.npz"):
        tot["scans"] += 1
        try:
            if not len(np.load(f).files):
                tot["scans_malos"] += 1
        except Exception:                                                   # noqa: BLE001
            tot["scans_malos"] += 1
    p = d + "/telemetria.csv"
    if os.path.exists(p):
        for l in open(p, errors="ignore"):
            tot["telemetria"] += 1; tot["telemetria_ceros"] += "\x00" in l
print(json.dumps(tot, indent=1))
open(os.path.expanduser("~/.agrobot/aprendizaje/verificacion_datos.json"), "w").write(json.dumps(tot, indent=1))
