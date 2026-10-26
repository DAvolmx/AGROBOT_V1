"""¿La pose llega desfasada respecto al scan? Reproduce una grabación en MapaScan (sin alineador) usando la pose interpolada en t + δ y mide la nitidez del mapa.
Si el mejor δ no es 0, la pose y el scan no son simultáneos (latencia del TF / del LIDAR) y se puede compensar. Uso: python3 tools/probar_desfase_pose.py <sesión> [cada=2]"""
import json, math, sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jetson" / "backend"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import mapa_scan as MS
from analizar_slam_dia import cargar


def run(v, delta, cada):
    t = np.array([r["env"]["_received_at"] - r.get("pose_age_s", 0.0) for r in v])        # instante al que corresponde cada pose
    x = np.array([r["pose"]["x"] for r in v]); y = np.array([r["pose"]["y"] for r in v])
    th = np.unwrap(np.radians([r["pose"]["theta_deg"] for r in v]))
    ts = np.array([r["env"]["_received_at"] for r in v])                                    # instante del scan
    antes = MS.MIN_IMPACTOS_ALINEAR
    MS.MIN_IMPACTOS_ALINEAR = 10 ** 9
    try:
        m = MS.MapaScan()
        for i in range(0, len(v), cada):
            tq = ts[i] + delta
            p = {"x": float(np.interp(tq, t, x)), "y": float(np.interp(tq, t, y)), "theta_deg": float(math.degrees(np.interp(tq, t, th)))}
            sc = v[i]["lidar_scan"]
            m.agregar(ts[i], p, [q["angle_deg"] for q in sc], [q["distance_m"] for q in sc])
        occ = int((m.L > MS.UMBRAL_OCUPADO).sum()); fuerte = int((m.L > 3.0).sum())
        return occ, round(fuerte / max(occ, 1), 3)
    finally:
        MS.MIN_IMPACTOS_ALINEAR = antes


if __name__ == "__main__":
    s = sys.argv[1]; cada = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    v = cargar(s)
    res = {}
    for d in (-0.20, -0.10, -0.05, 0.0, 0.05, 0.10, 0.20, 0.30):
        res[d] = run(v, d, cada)
        print(f"{s} δ={d:+.2f} s  celdas ocupadas {res[d][0]:6d}  nitidez {res[d][1]}", flush=True)
    mejor = min(res, key=lambda d: res[d][0])
    print("mejor δ por celdas ocupadas:", mejor, "| por nitidez:", max(res, key=lambda d: res[d][1]))
