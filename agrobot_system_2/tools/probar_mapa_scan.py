"""Reproduce una grabación de campo en MapaScan (el mapa del dashboard) con distintos parámetros y mide la NITIDEZ del mapa resultante.
Nitidez = celdas muy ocupadas (log-odds > 3) / celdas ocupadas (> 1): un mapa con paredes más finas y consistentes concentra la evidencia en menos
celdas. También cuenta las celdas ocupadas (menos = paredes más finas) y cuánto corrigió el alineador (m, °).
Uso: python3 tools/probar_mapa_scan.py <sesión> [cada=1]"""
import json, math, sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jetson" / "backend"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import mapa_scan as MS
from analizar_slam_dia import cargar


def reproducir(v, cada=1, **par):
    ant = {k: getattr(MS, k) for k in par}
    for k, x in par.items():
        setattr(MS, k, x)
    try:
        m = MS.MapaScan(manhattan=bool(par.get("_manhattan", False)))
        corr_max = 0.0
        t0 = time.time()
        for n, r in enumerate(v[::cada]):
            sc = r["lidar_scan"]
            ang = [q["angle_deg"] for q in sc]
            dist = [q["distance_m"] for q in sc]
            m.agregar(r["env"]["_received_at"], r["pose"], ang, dist)
            corr_max = max(corr_max, math.hypot(m.corr[0], m.corr[1]))
        occ = int((m.L > MS.UMBRAL_OCUPADO).sum())
        fuerte = int((m.L > 3.0).sum())
        return {"celdas_ocupadas": occ, "nitidez": round(fuerte / max(occ, 1), 3), "corr_final_cm": round(100 * math.hypot(m.corr[0], m.corr[1]), 1),
                "corr_final_deg": round(math.degrees(m.corr[2]), 2), "corr_max_cm": round(100 * corr_max, 1), "s": round(time.time() - t0, 1)}
    finally:
        for k, x in ant.items():
            setattr(MS, k, x)


if __name__ == "__main__":
    s = sys.argv[1]
    cada = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    v = cargar(s)
    variantes = {
        "sin alinear (pose SLAM tal cual)": dict(MIN_IMPACTOS_ALINEAR=10 ** 9),
        "actual (±6 cm, ±2°)": {},
        "búsqueda ±10 cm, ±3°": dict(BUSQUEDA_M=0.10, BUSQUEDA_DEG=3.0),
        "búsqueda fina ±6 cm paso 1,5 cm": dict(PASO_M=0.015),
        "±10 cm ±3° paso 2 cm 0,5°": dict(BUSQUEDA_M=0.10, BUSQUEDA_DEG=3.0, PASO_M=0.02, PASO_DEG=0.5),
        "giro permitido hasta 1,0 rad/s": dict(OMEGA_MAX_RPS=1.0),
    }
    out = {}
    for k, par in variantes.items():
        out[k] = reproducir(v, cada, **par)
        print(f"{k:38s}", out[k], flush=True)
    Path(__file__).resolve().parents[1].joinpath("docs/slam_dia", f"nitidez_{s}.json").write_text(json.dumps(out, ensure_ascii=False, indent=1))
