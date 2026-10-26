"""¿Una pose que combine el SLAM (ancla global) con el ICP scan-a-scan (precisión local) da un mapa más nítido que la pose del SLAM sola?
Reproduce una grabación de campo en MapaScan (sin su alineador, para medir solo la pose) con 4 flujos de pose:
  slam      la pose del SLAM tal cual (lo que hay hoy)
  icp       encadenar solo el ICP scan-a-scan (deriva, pero es local-exacto)
  fusion_k  ICP encadenado + tirón suave hacia la pose del SLAM (k por paso; 0,02 = el SLAM corrige la deriva en ~50 pasos = 10 s)
Mide: celdas ocupadas (menos = paredes más finas) y nitidez (celdas > 3 / celdas > 1). Uso: python3 tools/probar_pose_fusion.py <sesión> [cada=1]"""
import json, math, sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jetson" / "backend"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import mapa_scan as MS
from analizar_slam_dia import cargar, icp, paso_slam, puntos, rot


def flujos(v, ks=(0.02, 0.05, 0.15)):
    rng = np.random.default_rng(0)
    slam = [(r["pose"]["x"], r["pose"]["y"], math.radians(r["pose"]["theta_deg"])) for r in v]
    icp_chain = [slam[0]]
    fus = {k: [slam[0]] for k in ks}
    for i in range(len(v) - 1):
        a, b = v[i], v[i + 1]
        Pa, Pb = puntos(a["lidar_scan"]), puntos(b["lidar_scan"])
        R0, t0 = paso_slam(a["pose"], b["pose"])
        ok = len(Pa) >= 60 and len(Pb) >= 60
        if ok:
            sa = rng.choice(len(Pa), min(len(Pa), 220), replace=False)
            sb = rng.choice(len(Pb), min(len(Pb), 220), replace=False)
            R, t, inl, _ = icp(Pb[sb], Pa[sa], R0, t0)
            ok = inl > 0.5
        if not ok:
            R, t = R0, t0
        dth = math.atan2(R[1, 0], R[0, 0])
        for lista, k in [(icp_chain, None)] + [(fus[k_], k_) for k_ in ks]:
            x, y, th = lista[-1]
            d = rot(th) @ t
            x, y, th = x + d[0], y + d[1], th + dth
            if k is not None:                                          # tirón hacia el SLAM
                sx, sy, sth = slam[i + 1]
                x, y = x + k * (sx - x), y + k * (sy - y)
                th = th + k * ((sth - th + math.pi) % (2 * math.pi) - math.pi)
            lista.append((x, y, th))
    out = {"slam": slam, "icp": icp_chain}
    out.update({f"fusion_k={k}": fus[k] for k in ks})
    return out


def mapa(v, poses):
    antes = MS.MIN_IMPACTOS_ALINEAR
    MS.MIN_IMPACTOS_ALINEAR = 10 ** 9                                 # sin alineador: se mide solo la calidad de la pose
    try:
        m = MS.MapaScan()
        for r, (x, y, th) in zip(v, poses):
            sc = r["lidar_scan"]
            m.agregar(r["env"]["_received_at"], {"x": x, "y": y, "theta_deg": math.degrees(th)}, [q["angle_deg"] for q in sc], [q["distance_m"] for q in sc])
        occ, fuerte = int((m.L > MS.UMBRAL_OCUPADO).sum()), int((m.L > 3.0).sum())
        return {"celdas_ocupadas": occ, "nitidez": round(fuerte / max(occ, 1), 3)}
    finally:
        MS.MIN_IMPACTOS_ALINEAR = antes


if __name__ == "__main__":
    s = sys.argv[1]
    v = cargar(s)
    cada = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    v = v[::cada]
    fl = flujos(v)
    res = {k: mapa(v, p) for k, p in fl.items()}
    for k, r in res.items():
        print(f"{s} {k:16s} {r}", flush=True)
    Path(__file__).resolve().parents[1].joinpath("docs/slam_dia", f"fusion_{s}.json").write_text(json.dumps(res, indent=1))
