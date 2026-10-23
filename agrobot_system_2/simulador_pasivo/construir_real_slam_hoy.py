"""Dataset de corrección de SLAM con las grabaciones REALES de campo (5 Hz, scans completos) y la pose del SLAM como pseudo-referencia.

Mismo formato que generar_dataset_slam.py (K=5 fotogramas × 12 valores, targets_inc = incremento de REFERENCIA − incremento ICP) para poder
mezclarlo con el dataset simulado. HONESTIDAD: no hay verdad terreno externa; la referencia es la pose de slam_toolbox (casamiento de scans +
cierre de lazos sobre un grafo) y el incremento ICP es el de un casamiento fotograma a fotograma (como rf2o). El modelo aprende a acercar
el ICP crudo a lo que el SLAM acaba decidiendo: sirve para mejorar la odometría de entrada, no es «exactitud absoluta».
Uso: python3 construir_real_slam_hoy.py [--campo <carpeta> ...] [--salida ruta.npz]   (por defecto las grabaciones de 2026-10-23)"""
import argparse, bisect, glob, json, math, os
from pathlib import Path

import numpy as np

import generar_dataset_slam as G

HOME = Path.home() / ".agrobot"
K, N_BASE = G.K, G.N_BASE


def lote_que_solapa(t0, t1):
    mejor = None
    for d in sorted(glob.glob(str(HOME / "datos" / "2026*"))):
        f = Path(d) / "eventos.jsonl"
        if not f.exists():
            continue
        ts = []
        with open(f) as fh:
            for i, l in enumerate(fh):
                if i % 200 == 0:
                    try:
                        ts.append(json.loads(l)["t_unix"])
                    except Exception:
                        pass
        if ts and ts[0] <= t1 and ts[-1] >= t0:
            sol = min(ts[-1], t1) - max(ts[0], t0)
            if mejor is None or sol > mejor[0]:
                mejor = (sol, Path(d))
    return mejor[1] if mejor else None


def leer(path):
    out = []
    for l in open(path):
        try:
            out.append(json.loads(l))
        except ValueError:
            pass
    return out


def sesion(campo: Path):
    filas = [r for r in leer(campo / "variables.jsonl") if r.get("t_unix") and r.get("lidar_scan") and r.get("velocities")]
    if len(filas) < 60:
        return None
    lote = lote_que_solapa(filas[0]["t_unix"], filas[-1]["t_unix"])
    if lote is None:
        return None
    slam = [(e["t_unix"], e["datos"]["pose"]["x"], e["datos"]["pose"]["y"], math.radians(e["datos"]["pose"]["theta_deg"]))
            for e in leer(lote / "eventos.jsonl") if e["tipo"] == "scan_mapa"]
    ts = [s[0] for s in slam]

    def pose_slam(t):
        i = bisect.bisect_left(ts, t)
        if i <= 0 or i >= len(slam):
            return None
        a, b = slam[i - 1], slam[i]
        if b[0] - a[0] > 0.6 or b[0] - a[0] <= 0:
            return None
        u = (t - a[0]) / (b[0] - a[0])
        dth = G._wrap(b[3] - a[3])
        return np.array([a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2]), G._wrap(a[3] + u * dth)])

    est = None
    prev = None
    inc_prev = np.zeros(3)
    muestras = []
    rng = np.random.default_rng(0)
    for r in filas:
        t = r["t_unix"]
        real = pose_slam(t)
        if real is None:
            prev = None
            continue
        rr = G.a_rangos(r["lidar_scan"], rng, 0.0)
        ok_scan = np.isfinite(rr).sum() >= 25
        v = r["velocities"]
        v_c, w_c = float(v.get("v_aplicada") or 0.0), float(v.get("omega_aplicada") or 0.0)
        gz = math.radians(float((r.get("env") or {}).get("gyroZ") or 0.0))
        if est is None or prev is None:
            est = real.copy() if est is None else est
            prev = (t, real, G.a_xy(rr) if ok_scan else None, ok_scan)
            continue
        tj, real_j, xy_j, ok_j = prev
        dt = t - tj
        if dt <= 1e-3 or dt > 1.5:
            prev = (t, real, G.a_xy(rr) if ok_scan else None, ok_scan)
            continue
        inc_true = G._relativa(real_j, real)
        guess = np.array([v_c * dt, 0.0, (w_c if abs(gz) < 1e-9 else 0.5 * (w_c + gz)) * dt])
        if ok_scan and ok_j and xy_j is not None:
            inc_est, res, ok_icp = G.icp(xy_j, G.a_xy(rr), guess)
        else:
            inc_est, res, ok_icp = guess, 0.0, False
        est = G._componer(est, inc_est)
        fr = rr[np.r_[0:16, 345:360]]
        dmin = [float(np.nanmin(np.where(np.isfinite(x), x, G.R_MAX))) for x in (fr, rr[60:121], rr[240:301])]
        base = [v_c, w_c, gz, res, dmin[0], dmin[1], dmin[2], inc_prev[0], inc_prev[1], inc_prev[2], 1.0 if ok_icp else 0.0, dt]
        err = np.array([real[0] - est[0], real[1] - est[1], G._wrap(real[2] - est[2])])
        muestras.append((t, base, err, inc_true - inc_est, est.copy(), real.copy()))
        inc_prev = inc_est
        prev = (t, real, G.a_xy(rr) if ok_scan else None, ok_scan)
    if len(muestras) < K + 20:
        return None
    base = np.array([m[1] for m in muestras])
    feats = np.zeros((len(muestras), K * N_BASE))
    for i in range(len(muestras)):
        idx = [max(0, i - (K - 1) + j) for j in range(K)]
        feats[i] = base[idx].ravel()
    return {"features": feats, "targets": np.array([m[2] for m in muestras]), "targets_inc": np.array([m[3] for m in muestras]),
            "timestamp": np.array([m[0] for m in muestras]), "est_pose": np.array([m[4] for m in muestras]),
            "real_pose": np.array([m[5] for m in muestras])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--campo", nargs="*")
    ap.add_argument("--salida", default=str(HOME / "slam" / "real_hoy.npz"))
    a = ap.parse_args()
    campos = [Path(c) for c in a.campo] if a.campo else [Path(p) for p in sorted(glob.glob(str(HOME / "campo" / "20261023_*[0-9a-f]"))) if os.path.isdir(p)]
    partes, ids, nombres = [], [], []
    for k, c in enumerate(campos):
        r = sesion(c)
        print(c.name, "->", None if r is None else len(r["features"]), "muestras", flush=True)
        if r is None:
            continue
        partes.append(r)
        ids.append(np.full(len(r["features"]), k, dtype=np.int32))
        nombres.append(c.name)
    cat = {k: np.concatenate([p[k] for p in partes]) for k in partes[0]}
    n = len(cat["features"])
    np.savez_compressed(a.salida, **cat, episodio_id=np.concatenate(ids), escenario=np.array(["real_hoy"] * n), falla=np.array(["ninguno"] * n),
                        semilla=np.zeros(n, dtype=np.int32), nombres_features=np.array(G.NOMBRES), K=np.array(K),
                        meta=np.array(json.dumps({"fuente": "grabaciones de campo 2026-10-23 + SLAM (pseudo-referencia)", "sesiones": nombres})))
    print("->", a.salida, n, "muestras")


if __name__ == "__main__":
    main()
