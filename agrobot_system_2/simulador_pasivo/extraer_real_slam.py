"""Extrae de las sesiones REALES grabadas (~/.agrobot/datos/*/ con scans_*.npz + telemetria.csv) los mismos fotogramas que usa el
dataset simulado de corrección de SLAM, y mide parámetros reales del IMU y del LIDAR (2026-10-21).

IMPORTANTE (honestidad): las sesiones reales NO tienen pose de verdad terreno (no hay mocap ni referencia externa). Por eso
  · NO hay `targets`: estos datos no sirven para entrenar el corrector de forma supervisada;
  · SÍ sirven para (1) medir el sesgo y el ruido REALES del giroscopio y del LIDAR y sustituir las hipótesis del simulador,
    (2) comparar las distribuciones de entrada sim vs real (desfase de dominio) y (3) comprobar que el modelo, aplicado a datos
    reales, no inventa correcciones grandes. `ref_pose` es la pose que grabó el robot (rf2o + EKF + SLAM): es una REFERENCIA de
    consistencia, no una verdad terreno.
  · Los scans grabados van a ~1 Hz (dt ≈ 1 s), no a 10 Hz como en el simulador: es el desfase más importante.

Salida: ~/.agrobot/slam/real.npz y real_stats.json. Uso: python3 extraer_real_slam.py [--max-sesiones 80] [--salida RUTA]
"""
import argparse
import csv
import glob
import json
import math
import time
from pathlib import Path

import numpy as np

import generar_dataset_slam as G

RAIZ = Path.home() / ".agrobot" / "datos"
CAMPO = Path.home() / ".agrobot" / "campo"
SALIDA = Path.home() / ".agrobot" / "slam"
K, N_BASE = G.K, G.N_BASE


def cargar(ses: Path):
    txt = open(ses / "telemetria.csv", errors="ignore").read().replace("\x00", "")        # algunas sesiones tienen bytes nulos
    rows = [r for r in csv.DictReader(txt.splitlines()) if r.get("t")]

    def num(r, k):
        try:
            return float(r.get(k) or "nan")
        except (TypeError, ValueError):
            return float("nan")
    f = lambda k: np.array([num(r, k) for r in rows])
    tel = {k: f(k) for k in ("t", "v_aplicada", "w_aplicada", "giro_z", "x", "y", "rumbo_deg")}
    bl = [np.load(p) for p in sorted(ses.glob("scans_*.npz"))]
    if not bl:
        return None
    ts = np.concatenate([b["t"] for b in bl])
    ang = bl[0]["angulos_deg"]
    D = np.concatenate([b["distancias_m"] for b in bl]).astype(np.float32)
    return tel, ts, ang, D


def cargar_campo(ses: Path):
    """Grabación de CAMPO (recoleccion.py, 2026-10-22): variables.jsonl a 5 Hz con scan completo, pose, giroscopio y velocidades.
    Mismo formato de salida que cargar(). Los scans llegan a ~5 Hz (dt ≈ 0,2 s) en vez de ~1 Hz: mucho más cerca del simulador (10 Hz)."""
    tel = {k: [] for k in ("t", "v_aplicada", "w_aplicada", "giro_z", "x", "y", "rumbo_deg")}
    ts, D, visto = [], [], set()
    ang = np.arange(-180.0, 180.0, 0.5)
    with open(ses / "variables.jsonl", errors="ignore") as f:
        for linea in f:
            try:
                d = json.loads(linea)
            except ValueError:
                continue
            t = d.get("t_unix")
            p, v, env = d.get("pose") or {}, d.get("velocities") or {}, d.get("env") or {}
            if t is None:
                continue
            tel["t"].append(t)
            tel["v_aplicada"].append(v.get("v_aplicada", np.nan)); tel["w_aplicada"].append(v.get("omega_aplicada", np.nan))
            tel["giro_z"].append(env.get("gyroZ", np.nan))
            tel["x"].append(p.get("x", np.nan)); tel["y"].append(p.get("y", np.nan)); tel["rumbo_deg"].append(p.get("theta_deg", np.nan))
            lt, sc = d.get("lidar_timestamp"), d.get("lidar_scan")
            if not sc or lt in visto:                       # el mismo scan repetido en dos muestras se usa una vez
                continue
            visto.add(lt)
            fila = np.full(720, np.nan, dtype=np.float32)
            for r in sc:
                a, dist = r.get("angle_deg"), r.get("distance_m")
                if a is None or dist is None:
                    continue
                fila[int(round((((a + 180.0) % 360.0)) * 2)) % 720] = dist
            ts.append(t - float(d.get("lidar_age_s") or 0.0)); D.append(fila)
    if not D:
        return None
    tel = {k: np.array([np.nan if x is None else x for x in v], dtype=float) for k, v in tel.items()}
    return tel, np.array(ts), ang, np.array(D)


def a_360(ang, d):
    """720 rayos de 0,5° (-180..180) -> 360 rayos de 1° con índice = grado % 360 (0° al frente, + izquierda), inf si no válido."""
    r = np.full(360, np.inf)
    idx = np.round(ang).astype(int) % 360
    ok = np.isfinite(d) & (d >= G.R_MIN) & (d <= G.R_MAX)
    for i, v in zip(idx[ok], d[ok]):
        if v < r[i]:
            r[i] = v
    return r


def procesar_sesion(ses: Path, max_pares=2500):
    c = cargar_campo(ses) if (ses / "variables.jsonl").exists() else cargar(ses)
    if c is None:
        return None
    tel, ts, ang, D = c
    t = tel["t"]
    ok_t = np.isfinite(t)
    if ok_t.sum() < 50:
        return None
    for k in tel:                                   # solo filas con reloj válido y ordenado
        tel[k] = tel[k][ok_t]
    t = tel["t"]
    ok_t = np.ones(len(t), bool)
    orden = np.argsort(t, kind="stable")
    for k in tel:
        tel[k] = tel[k][orden]
    t = tel["t"]
    mov = np.nan_to_num(np.abs(tel["v_aplicada"]) + np.abs(tel["w_aplicada"])) > 0.01
    quieto = (~mov) & np.isfinite(tel["giro_z"])
    stats = {"gz_quieto": tel["giro_z"][quieto]}
    # porcentaje de rayos no válidos
    stats["rayos_invalidos_pct"] = float(100 * (~np.isfinite(D) | (D < G.R_MIN) | (D > G.R_MAX)).mean())
    muestras, est, prev, inc_prev = [], None, None, np.zeros(3)
    ref_fn = lambda tt: np.array([np.interp(tt, t[ok_t], np.nan_to_num(tel["x"])[ok_t]), np.interp(tt, t[ok_t], np.nan_to_num(tel["y"])[ok_t]),
                                  math.radians(float(np.interp(tt, t[ok_t], np.unwrap(np.deg2rad(np.nan_to_num(tel["rumbo_deg"])))[ok_t] * 180 / math.pi)))])
    for i in range(len(ts)):
        tk = float(ts[i])
        r = a_360(ang, D[i])
        okr = np.isfinite(r).sum() >= 25
        if prev is None:
            prev = (tk, G.a_xy(r) if okr else None, okr)
            est = np.zeros(3)
            continue
        tj, xy_j, ok_j = prev
        dt = tk - tj
        if dt <= 0.05 or dt > 3.0:
            prev = (tk, G.a_xy(r) if okr else None, okr)
            continue
        i0, i1 = int(np.searchsorted(t, tj)), int(np.searchsorted(t, tk))
        m = slice(i0, i1)
        if i1 - i0 < 2:
            continue
        v_c, w_c = float(np.nanmean(tel["v_aplicada"][m])), float(np.nanmean(tel["w_aplicada"][m]))
        if abs(v_c) + abs(w_c) < 0.01:                    # parado: no aporta a la corrección de movimiento
            prev = (tk, G.a_xy(r) if okr else None, okr)
            continue
        gyro = math.radians(float(np.nanmean(tel["giro_z"][m])))           # suposición: °/s (se comprueba en las estadísticas)
        guess = np.array([v_c * dt, 0.0, gyro * dt])
        if okr and ok_j and xy_j is not None:
            inc_est, res, ok_icp = G.icp(xy_j, G.a_xy(r), guess)
        else:
            inc_est, res, ok_icp = guess, 0.0, False
        est = G._componer(est, inc_est)
        dm = G.dmin_sectores_rangos(r) if hasattr(G, "dmin_sectores_rangos") else None
        if dm is None:
            fr, iz, de = r[np.r_[0:16, 345:360]], r[60:121], r[240:301]
            dm = [float(np.where(np.isfinite(x), x, G.R_MAX).min()) for x in (fr, iz, de)]
        base = [v_c, w_c, gyro, res, dm[0], dm[1], dm[2], inc_prev[0], inc_prev[1], inc_prev[2], 1.0 if ok_icp else 0.0, dt]
        muestras.append((tk, base, est.copy(), ref_fn(tk), float(res), float(np.hypot(*inc_est[:2]))))
        inc_prev = inc_est
        prev = (tk, G.a_xy(r) if okr else None, okr)
        if len(muestras) >= max_pares:
            break
    if len(muestras) < K + 3:
        return {"stats": stats, "muestras": None}
    base = np.array([m[1] for m in muestras])
    n = len(muestras)
    feats = np.zeros((n, K * N_BASE))
    for i in range(n):
        idx = [max(0, i - (K - 1) + j) for j in range(K)]
        feats[i] = base[idx].ravel()
    return {"stats": stats, "muestras": {"features": feats, "timestamp": np.array([m[0] for m in muestras]),
                                         "est_pose": np.array([m[2] for m in muestras]), "ref_pose": np.array([m[3] for m in muestras])}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-sesiones", type=int, default=80)
    ap.add_argument("--salida", default=None)
    a = ap.parse_args()
    ses = [Path(p) for p in sorted(glob.glob(str(RAIZ / "2026*")))
           if (Path(p) / "telemetria.csv").exists() and glob.glob(p + "/scans_*.npz")][-a.max_sesiones:]
    ses += [Path(p) for p in sorted(glob.glob(str(CAMPO / "2026*"))) if (Path(p) / "variables.jsonl").exists()]   # grabaciones de campo a 5 Hz
    print(f"[real] {len(ses)} sesiones con scans y telemetría", flush=True)
    t0 = time.time()
    feats, tss, ests, refs, sid, gz, inval = [], [], [], [], [], [], []
    usadas = []
    for i, s in enumerate(ses):
        try:
            r = procesar_sesion(s)
        except Exception as e:                      # una sesión corrupta no debe tirar las demás
            print(f"  {s.name}: error {e!r}", flush=True)
            continue
        if r is None:
            continue
        gz.append(r["stats"]["gz_quieto"]); inval.append(r["stats"]["rayos_invalidos_pct"])
        m = r["muestras"]
        if m is not None:
            feats.append(m["features"]); tss.append(m["timestamp"]); ests.append(m["est_pose"]); refs.append(m["ref_pose"])
            sid.append(np.full(len(m["features"]), len(usadas))); usadas.append(s.name)
            print(f"  {s.name}: {len(m['features'])} fotogramas con movimiento", flush=True)
    gzq = np.concatenate(gz) if gz else np.array([])
    gzq = gzq[np.isfinite(gzq)]
    stats = {"sesiones_revisadas": len(ses), "sesiones_con_movimiento": len(usadas),
             "gyro_z_quieto_dps": {"media_sesgo": float(gzq.mean()) if gzq.size else None, "desv_ruido": float(gzq.std()) if gzq.size else None,
                                   "n": int(gzq.size), "unidad_supuesta": "grados/s"},
             "rayos_invalidos_pct_medio": float(np.mean(inval)) if inval else None,
             "dt_entre_scans_en_movimiento_s": float(np.round(np.concatenate(feats)[:, (K - 1) * N_BASE + 11].mean(), 3)) if feats else None,
             "hipotesis_del_simulador": {"sesgo_imu_dps": [0.2, 1.0], "ruido_imu_dps": 0.3}}
    if not feats:
        raise SystemExit("ninguna sesión real con movimiento y scans")
    F = np.concatenate(feats)
    stats["fotogramas"] = int(len(F))
    stats["real_features_media"] = dict(zip(G.NOMBRES, np.round(F[:, (K - 1) * N_BASE:].mean(0), 4).tolist()))
    SALIDA.mkdir(parents=True, exist_ok=True)
    ruta = Path(a.salida) if a.salida else SALIDA / "real.npz"
    np.savez_compressed(ruta, features=F.astype(np.float32), timestamp=np.concatenate(tss), est_pose=np.concatenate(ests).astype(np.float32),
                        ref_pose=np.concatenate(refs).astype(np.float32), sesion_id=np.concatenate(sid).astype(np.int32),
                        sesiones=np.array(usadas), nombres_features=np.array(G.NOMBRES), K=np.array(K),
                        meta=np.array(json.dumps({"fecha": time.strftime("%Y-%m-%d %H:%M"), "origen": "real",
                                                  "aviso": "sin verdad terreno: ref_pose es la pose del propio robot (rf2o+EKF+SLAM)", **stats})))
    (SALIDA / "real_stats.json").write_text(json.dumps(stats, indent=1, ensure_ascii=False))
    print(json.dumps(stats, indent=1, ensure_ascii=False))
    print(f"[real] {len(F)} fotogramas · {len(usadas)} sesiones -> {ruta} ({int(time.time() - t0)} s)", flush=True)


if __name__ == "__main__":
    main()
