"""
Lee una sesión grabada por `jetson/backend/grabador.py` y la resume.

    python3 tools/leer_sesion.py                  # la última
    python3 tools/leer_sesion.py 20261002_053000  # una concreta

Como módulo: `cargar(carpeta)` -> dict con `telemetria` (lista de dicts),
`scans` (t, angulos_deg, distancias_m), `eventos` y `fotos` (rutas), para
calibrar el simulador o armar datasets.
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np

RAIZ = Path.home() / ".agrobot" / "datos"


def cargar(carpeta: Path) -> dict:
    tel = list(csv.DictReader(open(carpeta / "telemetria.csv")))
    bloques = [np.load(f) for f in sorted(carpeta.glob("scans_*.npz"))]
    scans = None
    if bloques:
        scans = {"t": np.concatenate([b["t"] for b in bloques]),
                 "angulos_deg": bloques[0]["angulos_deg"],
                 "distancias_m": np.concatenate([b["distancias_m"] for b in bloques]).astype(np.float32)}
    eventos = [json.loads(l) for l in open(carpeta / "eventos.jsonl")] if (carpeta / "eventos.jsonl").exists() else []
    fotos = sorted((carpeta / "camara").glob("*.jpg"))
    return {"telemetria": tel, "scans": scans, "eventos": eventos, "fotos": fotos,
            "sesion": json.loads((carpeta / "sesion.json").read_text())}


def resumen(d: dict) -> dict:
    tel = d["telemetria"]
    mov = [f for f in tel if any(abs(float(f.get(k) or 0)) > 0.01 for k in ("v_pedida", "w_pedida"))]
    tipos = {}
    for e in d["eventos"]:
        tipos[e["tipo"]] = tipos.get(e["tipo"], 0) + 1
    dur = (float(tel[-1]["t"]) - float(tel[0]["t"])) if len(tel) > 1 else 0
    return {"duracion_min": round(dur / 60, 1), "filas": len(tel), "filas_en_movimiento": len(mov),
            "scans": 0 if d["scans"] is None else len(d["scans"]["t"]), "fotos": len(d["fotos"]),
            "eventos": tipos, "codigo": d["sesion"].get("codigo")}


if __name__ == "__main__":
    if len(sys.argv) > 1:
        carpeta = RAIZ / sys.argv[1]
    else:
        carpetas = sorted(p for p in RAIZ.glob("*") if p.is_dir())
        if not carpetas:
            sys.exit("no hay sesiones en ~/.agrobot/datos")
        carpeta = carpetas[-1]
    print(carpeta)
    print(json.dumps(resumen(cargar(carpeta)), indent=2, ensure_ascii=False))
