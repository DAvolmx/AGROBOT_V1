"""Mide el ERROR DEL SLAM con una grabación de campo (~/.agrobot/campo/<sesión>/variables.jsonl: pose + scan + giroscopio a 5 Hz).

Sin verdad terreno, se usa la propia geometría de los scans:
  1. ERROR POR PASO: entre dos scans consecutivos se parte del movimiento que dice el SLAM (pose a -> pose b) y se refina con ICP (punto a punto).
     La corrección que pide el ICP = lo que el SLAM se equivocó en ese paso (traslación en cm, rotación en °). Se agrupa por tipo de movimiento.
  2. GIROSCOPIO vs SLAM: el giro integrado del IMU contra el cambio de rumbo de la pose en ventanas de ~5 s.
  3. CONSISTENCIA DEL MAPA: porcentaje de puntos de cada scan que caen a < 4 cm de lo visto por scans de otro momento (> 4 s antes), con la pose del SLAM.
Uso: python3 tools/analizar_slam_dia.py <sesión> [<sesión> ...]   (por defecto las de hoy)   -> JSON en docs/slam_dia/"""
import json, math, sys, time
from pathlib import Path

import numpy as np

CAMPO = Path.home() / ".agrobot" / "campo"
OUT = Path(__file__).resolve().parents[1] / "docs" / "slam_dia"


def cargar(sesion):
    v = []
    for l in open(CAMPO / sesion / "variables.jsonl", errors="ignore"):
        try:
            v.append(json.loads(l))
        except ValueError:
            pass
    return [r for r in v if r.get("pose") and r.get("lidar_scan")]


def puntos(scan, rmin=0.15, rmax=6.0):
    a = np.radians([q["angle_deg"] for q in scan])
    d = np.array([q["distance_m"] for q in scan])
    ok = (d > rmin) & (d < rmax)
    p = np.stack([d[ok] * np.cos(a[ok]), d[ok] * np.sin(a[ok])], 1)
    chasis = (p[:, 0] > -0.32) & (p[:, 0] < 0.20) & (np.abs(p[:, 1]) < 0.22)          # el propio robot (marcos del LIDAR del backend)
    return p[~chasis]


def rot(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s], [s, c]])


def icp(P, Q, R0, t0, it=25):
    """Alinea P (scan b) sobre Q (scan a): Q ≈ R P + t. Devuelve R, t, fracción de inliers (< 8 cm) y rms de los inliers."""
    R, t = R0.copy(), t0.copy()
    for _ in range(it):
        X = P @ R.T + t
        d2 = ((X[:, None, :] - Q[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        dist = np.sqrt(d2[np.arange(len(X)), j])
        m = dist < max(0.25, 0.0)
        if m.sum() < 20:
            break
        A, B = X[m], Q[j[m]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        Rn = Vt.T @ U.T
        if np.linalg.det(Rn) < 0:
            Vt[-1] *= -1
            Rn = Vt.T @ U.T
        tn = cb - Rn @ ca
        R, t = Rn @ R, Rn @ t + tn
    X = P @ R.T + t
    d = np.sqrt(((X[:, None, :] - Q[None, :, :]) ** 2).sum(-1)).min(1)
    inl = d < 0.08
    return R, t, float(inl.mean()), float(np.sqrt((d[inl] ** 2).mean())) if inl.any() else 1.0


def paso_slam(a, b):
    """Movimiento de a -> b según el SLAM, en ejes de a: (R, t) tal que p_a = R p_b + t."""
    ta, tb = math.radians(a["theta_deg"]), math.radians(b["theta_deg"])
    d = np.array([b["x"] - a["x"], b["y"] - a["y"]])
    return rot(tb - ta), rot(-ta) @ d


def analizar(sesion, max_pares=1500):
    v = cargar(sesion)
    rng = np.random.default_rng(0)
    idx = np.arange(len(v) - 1)
    if len(idx) > max_pares:
        idx = np.sort(rng.choice(idx, max_pares, replace=False))
    filas = []
    for i in idx:
        a, b = v[i], v[i + 1]
        dt = b["env"]["_received_at"] - a["env"]["_received_at"] if a.get("env") and b.get("env") else 0.2
        if not (0.05 < dt < 0.6):
            continue
        Pa, Pb = puntos(a["lidar_scan"]), puntos(b["lidar_scan"])
        if len(Pa) < 60 or len(Pb) < 60:
            continue
        sa = rng.choice(len(Pa), min(len(Pa), 220), replace=False)
        sb = rng.choice(len(Pb), min(len(Pb), 220), replace=False)
        R0, t0 = paso_slam(a["pose"], b["pose"])
        R, t, inl0, rms0 = icp(Pb[sb], Pa[sa], R0, t0, it=0)                 # calidad del movimiento del SLAM tal cual
        R, t, inl, rms = icp(Pb[sb], Pa[sa], R0, t0)
        dth = math.degrees(math.atan2(R[1, 0], R[0, 0]) - math.atan2(R0[1, 0], R0[0, 0]))
        dth = (dth + 180) % 360 - 180
        n0 = float(np.hypot(*t0))
        u = t0 / n0 if n0 > 1e-6 else np.array([1.0, 0.0])
        dlt = t - t0
        filas.append({"i": int(i), "dt": dt, "along_cm": float(dlt @ u * 100), "lat_cm": float((u[0] * dlt[1] - u[1] * dlt[0]) * 100), "mov_cm": float(np.hypot(*t0) * 100), "giro_deg": math.degrees(math.atan2(R0[1, 0], R0[0, 0])),
                      "err_cm": float(np.hypot(*(t - t0)) * 100), "err_deg": dth, "inl0": inl0, "inl": inl, "rms_cm": rms * 100,
                      "gz": float(a["env"].get("gyroZ", 0.0))})
    return v, filas


def resumen(filas):
    f = [r for r in filas if r["inl"] > 0.5]                              # pares donde el ICP encontró estructura (excluye pasillos vacíos / scans malos)
    def grupo(sel):
        l = [r for r in f if sel(r)]
        if not l:
            return None
        e, g = np.array([r["err_cm"] for r in l]), np.array([r["err_deg"] for r in l])
        return {"pares": len(l), "err_cm_medio": round(float(e.mean()), 2), "err_cm_p95": round(float(np.percentile(e, 95)), 2),
                "err_deg_medio_abs": round(float(np.abs(g).mean()), 2), "err_deg_p95": round(float(np.percentile(np.abs(g), 95)), 2),
                "err_deg_sesgo": round(float(g.mean()), 2), "mov_cm_medio": round(float(np.mean([r['mov_cm'] for r in l])), 1),
                "a_lo_largo_cm_sesgo": round(float(np.mean([r['along_cm'] for r in l])), 2), "lateral_cm_sesgo": round(float(np.mean([r['lat_cm'] for r in l])), 2),
                "escala_mediana_%": round(float(100 * np.median([r['along_cm'] / max(r['mov_cm'], 0.5) for r in l])), 1)}
    return {"pares_usados": len(f), "pares_total": len(filas), "todo": grupo(lambda r: True),
            "parado (<0,5 cm y <0,5°)": grupo(lambda r: r["mov_cm"] < 0.5 and abs(r["giro_deg"]) < 0.5),
            "avanzando recto (>2 cm, <2°)": grupo(lambda r: r["mov_cm"] > 2 and abs(r["giro_deg"]) < 2),
            "girando (>3°)": grupo(lambda r: abs(r["giro_deg"]) > 3),
            "mov. rápido (>6 cm/paso)": grupo(lambda r: r["mov_cm"] > 6)}


def giroscopio(v):
    """Giro del IMU integrado vs cambio de rumbo de la pose, en ventanas de ~5 s (25 muestras)."""
    out = []
    for k in range(0, len(v) - 25, 25):
        w = v[k:k + 26]
        try:
            gz = sum(r["env"]["gyroZ"] for r in w[:-1]) * 0.2                 # °/s × 0,2 s por muestra
        except (KeyError, TypeError):
            continue
        dth = (w[-1]["pose"]["theta_deg"] - w[0]["pose"]["theta_deg"] + 180) % 360 - 180
        out.append((gz, dth))
    a = np.array(out)
    if len(a) < 5:
        return None
    gz, dth = a[:, 0], a[:, 1]
    mov = np.abs(dth) > 10
    return {"ventanas": len(a), "ventanas_con_giro_>10°": int(mov.sum()),
            "razon_pose/giroscopio (mediana, solo con giro)": round(float(np.median(dth[mov] / np.where(np.abs(gz[mov]) < 1e-6, 1e-6, gz[mov]))), 2) if mov.any() else None,
            "diferencia_media_abs_deg": round(float(np.abs(dth - gz).mean()), 2), "diferencia_p95_deg": round(float(np.percentile(np.abs(dth - gz), 95)), 2)}


def consistencia(v, n=200, hueco_s=4.0):
    """% de puntos que coinciden (< 4 cm) con scans de otro momento al ponerlos en el marco del mapa con la pose del SLAM."""
    rng = np.random.default_rng(1)
    def mundo(r):
        p = puntos(r["lidar_scan"])
        th = math.radians(r["pose"]["theta_deg"])
        return p @ rot(th).T + np.array([r["pose"]["x"], r["pose"]["y"]])
    res, esp = [], []
    for i in rng.choice(np.arange(60, len(v)), min(n, len(v) - 60), replace=False):
        A = mundo(v[i])
        ref = np.concatenate([mundo(v[j]) for j in range(max(0, i - 60), i - int(hueco_s * 5), 6)]) if i - int(hueco_s * 5) > 6 else None
        if ref is None or len(ref) < 50 or len(A) < 50:
            continue
        sa = A[rng.choice(len(A), min(len(A), 150), replace=False)]
        d = np.sqrt(((sa[:, None, :] - ref[None, ::2, :]) ** 2).sum(-1)).min(1)
        solapa = d < 0.30                                              # solo cuenta donde el mapa YA tenía algo cerca (no el territorio nuevo)
        if solapa.sum() >= 30:
            res.append(float((d[solapa] < 0.04).mean()))
            esp.append(float(np.median(d[solapa])))
    return {"scans": len(res), "coinciden_<4cm_en_zona_vista_%": round(100 * float(np.mean(res)), 1) if res else None,
            "p10_%": round(100 * float(np.percentile(res, 10)), 1) if res else None,
            "desfase_mediano_cm": round(100 * float(np.median(esp)), 1) if esp else None}


def main():
    sesiones = sys.argv[1:] or sorted(p.name for p in CAMPO.glob(time.strftime("%Y%m%d") + "_*") if p.is_dir())
    OUT.mkdir(parents=True, exist_ok=True)
    todo = {}
    for s in sesiones:
        v, filas = analizar(s)
        todo[s] = {"muestras": len(v), "error_por_paso": resumen(filas), "giroscopio": giroscopio(v), "consistencia": consistencia(v)}
        print(s, json.dumps(todo[s], ensure_ascii=False, indent=1), flush=True)
        (OUT / f"{s}_pares.json").write_text(json.dumps(filas))
    (OUT / "resumen.json").write_text(json.dumps(todo, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
