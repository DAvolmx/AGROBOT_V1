"""Genera el DATASET para entrenar (en Kaggle) un modelo de CORRECCIÓN DE SLAM, con el mismo simulador del robot (2026-10-21).

Ciclo cerrado:  este generador  ->  dataset.npz  ->  notebook de Kaggle (tools/kaggle/agrobot_slam_correccion.py/.ipynb)
                ->  modelo_correccion_pesos.npz + scaler  ->  jetson/backend/slam_correccion.py (NumPy puro, en el robot).

Qué se simula (todo del simulador de manejo: geometría del croquis, firmware, física con fallas, choques, paradas de escaneo):
  · cada episodio de `episodio_fallas` (GRABADOR) entrega el scan con las fallas aplicadas (cortes, fantasmas, vidrio, persona),
    los comandos reales (v, ω) y la pose verdadera del LIDAR;
  · al scan se le suma ruido gaussiano extra (σ total 1–3 cm, sorteado por episodio) y se simula el IMU: gyro_z = ω verdadero +
    sesgo (0,2–1 °/s, signo aleatorio, HIPÓTESIS) + ruido blanco (0,3 °/s);
  · la odometría ESTIMADA se hace como en el robot: casamiento de scans (ICP punto a punto, equivalente a rf2o) con la
    estimación inicial (v comandado, giroscopio), integrada en el tiempo. Sin scan válido avanza solo con la estimación inicial.

Salida (~/.agrobot/slam/dataset_<fecha>.npz, y dataset.npz si se pide --salida):
  features (N, 12*K)  K=5 fotogramas; por fotograma: v_cmd, w_cmd, gyro_z, icp_residual, dmin_frontal, dmin_izq, dmin_der,
                      prev_dx, prev_dy, prev_dth (incremento previo estimado), scan_ok, dt
  targets (N, 3)      pose_real − pose_estimada en el marco del mapa (dx, dy, dθ)  [acumulado: lo pide el documento]
  targets_inc (N, 3)  error del INCREMENTO (real − estimado, marco del robot): es lo observable a partir de la ventana; el notebook
                      entrena con esto y luego integra para medir ATE/RPE/cierre de lazo
  escenario, falla, semilla, timestamp, episodio_id, est_pose (N,3), real_pose (N,3), nombres_features, meta (json)

NO despliega nada. Uso:
  python3 generar_dataset_slam.py [--config optimo|actual|interior] [--procs 6] [--semillas 6] [--rapido] [--salida RUTA]
"""
import argparse
import json
import math
import time
from dataclasses import asdict
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import sim_logica_actual as S
import episodio_fallas as E
import pruebas_geometria as PG

SALIDA = Path.home() / ".agrobot" / "slam"
K = 5
N_BASE = 12
NOMBRES = ["v_cmd", "w_cmd", "gyro_z", "icp_residual", "dmin_front", "dmin_izq", "dmin_der",
           "prev_dx", "prev_dy", "prev_dth", "scan_ok", "dt"]
R_MIN, R_MAX = 0.18, 8.0
FALLOS_GENERALES = ("ninguno", "barro", "rueda_muerta", "motor_debil", "suelo_resbaloso", "lidar_cortes", "lidar_fantasmas",
                    "enlace_cortes")
IMU_RUIDO_RPS = math.radians(0.3)


class Grabador:
    def __init__(self):
        self.scans, self.pasos = [], []

    def scan(self, t, pts, lidar_xy, psi):
        self.scans.append((t, pts, (float(lidar_xy[0]), float(lidar_xy[1])), float(psi)))

    def paso(self, t, v, w, lidar_xy, psi):
        self.pasos.append((t, float(v), float(w), float(lidar_xy[0]), float(lidar_xy[1]), float(psi)))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _rm(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def _componer(p, inc):
    """pose_k = pose_j ∘ inc (inc en el marco del robot de j)."""
    c, s = math.cos(p[2]), math.sin(p[2])
    return np.array([p[0] + c * inc[0] - s * inc[1], p[1] + s * inc[0] + c * inc[1], _wrap(p[2] + inc[2])])


def _relativa(pj, pk):
    """inc tal que pk = pj ∘ inc."""
    c, s = math.cos(pj[2]), math.sin(pj[2])
    dx, dy = pk[0] - pj[0], pk[1] - pj[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, _wrap(pk[2] - pj[2])])


def a_rangos(pts, rng, sigma_extra):
    """Lista de {angle_deg, distance_m} -> vector (360,) con inf donde no hay retorno válido."""
    r = np.full(360, np.inf)
    for p in pts:
        i = int(round(p["angle_deg"])) % 360
        d = p["distance_m"]
        if R_MIN <= d <= R_MAX:
            r[i] = min(r[i], d + rng.normal(0.0, sigma_extra))
    return r


def a_xy(r):
    ok = np.isfinite(r) & (r >= R_MIN) & (r <= R_MAX)
    a = np.deg2rad(np.arange(360))[ok]
    return np.stack([r[ok] * np.cos(a), r[ok] * np.sin(a)], 1)


def icp(prev_xy, cur_xy, guess, iters=20):
    """Alinea `cur` (marco k) con `prev` (marco j). Devuelve (inc=(dx,dy,dθ), residual, ok). Punto a punto con rechazo."""
    if len(prev_xy) < 25 or len(cur_xy) < 25:
        return np.array(guess, float), 0.0, False
    th, t = guess[2], np.array(guess[:2], float)
    gate = 0.5
    res = 0.0
    for _ in range(iters):
        R = _rm(th)
        q = cur_xy @ R.T + t
        D2 = ((q[:, None, :] - prev_xy[None, :, :]) ** 2).sum(-1)          # 360x360: fuerza bruta, sin SciPy
        idx = D2.argmin(1)
        d = np.sqrt(D2[np.arange(len(q)), idx])
        m = d <= gate
        if m.sum() < 20:
            return np.array(guess, float), 0.0, False
        a, b = q[m], prev_xy[idx[m]]
        ca, cb = a.mean(0), b.mean(0)
        H = (a - ca).T @ (b - cb)
        U, _, Vt = np.linalg.svd(H)
        Rd = Vt.T @ U.T
        if np.linalg.det(Rd) < 0:
            Vt[-1] *= -1
            Rd = Vt.T @ U.T
        td = cb - Rd @ ca
        dth = math.atan2(Rd[1, 0], Rd[0, 0])
        th = _wrap(th + dth)
        t = Rd @ t + td
        res = float(d[m].mean())
        gate = max(0.15, gate * 0.8)
        if abs(dth) < 1e-5 and np.linalg.norm(td) < 1e-5:
            break
    return np.array([t[0], t[1], th]), res, True


def procesar(rec: Grabador, rng, sigma_total, sesgo):
    """Convierte la grabación de un episodio en muestras (una por scan)."""
    if len(rec.scans) < K + 3 or len(rec.pasos) < 50:
        return None
    sigma_extra = math.sqrt(max(sigma_total ** 2 - 0.01 ** 2, 0.0))
    pt = np.array([p[0] for p in rec.pasos])
    pv = np.array([p[1] for p in rec.pasos])
    pw = np.array([p[2] for p in rec.pasos])
    ppose = np.array([[p[3], p[4], p[5]] for p in rec.pasos])
    muestras = []
    est = None
    prev = None                       # (t, pose_real, xy, ok)
    inc_prev = np.zeros(3)
    for (t, pts, lxy, psi) in rec.scans:
        real = np.array([lxy[0], lxy[1], _wrap(psi)])
        r = a_rangos(pts, rng, sigma_extra) if pts is not None else None
        ok_scan = r is not None and np.isfinite(r).sum() >= 25
        if est is None:
            est = real.copy()
            prev = (t, real, a_xy(r) if ok_scan else None, ok_scan)
            continue
        tj, real_j, xy_j, ok_j = prev
        dt = t - tj
        if dt <= 1e-3:
            continue
        if dt > 1.5:                   # hueco largo (p. ej. corte del LIDAR): se reengancha con la guía
            pass
        i0, i1 = np.searchsorted(pt, tj), np.searchsorted(pt, t)
        if i1 - i0 < 1:
            continue
        v_c, w_c = float(pv[i0:i1].mean()), float(pw[i0:i1].mean())
        inc_true = _relativa(real_j, real)
        gz_true = inc_true[2] / dt
        n_imu = max(1, int(dt * 20))
        gyro = gz_true + sesgo + rng.normal(0.0, IMU_RUIDO_RPS / math.sqrt(n_imu))
        guess = np.array([v_c * dt, 0.0, gyro * dt])
        if ok_scan and ok_j and xy_j is not None:
            inc_est, res, ok_icp = icp(xy_j, a_xy(r), guess)
        else:
            inc_est, res, ok_icp = guess, 0.0, False
        est = _componer(est, inc_est)
        if r is not None:
            fr = r[np.r_[0:16, 345:360]]
            iz = r[60:121]
            de = r[240:301]
            dmin = [float(np.nanmin(np.where(np.isfinite(x), x, R_MAX))) for x in (fr, iz, de)]
        else:
            dmin = [R_MAX, R_MAX, R_MAX]
        base = [v_c, w_c, gyro, res, dmin[0], dmin[1], dmin[2], inc_prev[0], inc_prev[1], inc_prev[2],
                1.0 if (ok_icp) else 0.0, dt]
        err_pose = np.array([real[0] - est[0], real[1] - est[1], _wrap(real[2] - est[2])])
        muestras.append((t, base, err_pose, inc_true - inc_est, est.copy(), real.copy()))
        inc_prev = inc_est
        prev = (t, real, a_xy(r) if ok_scan else None, ok_scan)
    if len(muestras) < K + 3:
        return None
    # Ventana de K fotogramas (repite el primero al principio)
    base = np.array([m[1] for m in muestras])
    n = len(muestras)
    feats = np.zeros((n, K * N_BASE))
    for i in range(n):
        idx = [max(0, i - (K - 1) + j) for j in range(K)]
        feats[i] = base[idx].ravel()
    return {"features": feats, "targets": np.array([m[2] for m in muestras]), "targets_inc": np.array([m[3] for m in muestras]),
            "timestamp": np.array([m[0] for m in muestras]), "est_pose": np.array([m[4] for m in muestras]),
            "real_pose": np.array([m[5] for m in muestras])}


def _job(a):
    cfg_d, esc_n, hueco, seed, fallo = a
    cfg = S.Config(**cfg_d)
    esc = PG.todos_los_escenarios(hueco)[esc_n]
    if esc_n == "recto_tope":
        cfg = PG.a_toda_velocidad(cfg)
    ap = E.AP.Aprendizaje(Path("/nonexistent/m.json"), autoguardar=False)
    rec = Grabador()
    E.GRABADOR = rec
    try:
        r = E.episodio_fallas(esc, cfg, seed, fallo, ap, False)
    finally:
        E.GRABADOR = None
    rng = np.random.default_rng(900_000 + seed * 31 + len(esc_n) * 7 + len(fallo))
    sigma = float(rng.choice([0.01, 0.02, 0.03]))
    sesgo = float(rng.choice([-1, 1]) * math.radians(rng.uniform(0.2, 1.0)))
    d = procesar(rec, rng, sigma, sesgo)
    if d is None:
        return None
    d.update(escenario=esc_n, falla=fallo, semilla=seed, sigma=sigma, sesgo_dps=round(math.degrees(sesgo), 3),
             pasa=bool(r.get("pasa")), t_s=r.get("t_s", 0))
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", choices=("optimo", "actual", "interior"), default="optimo")
    ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--semillas", type=int, default=6)
    ap.add_argument("--rapido", action="store_true")
    ap.add_argument("--fallas", default=None, help="lista separada por comas (por defecto las 8 generales)")
    ap.add_argument("--salida", default=None)
    a = ap.parse_args()
    cfg = PG.config_base(a.config)
    escs = ["sala_vacia", "pasillo_macetas", "eslalon_macetas", "curva_pasillo", "recto_tope", "sala_cuadrada",
            "hileras_cuadricula", "zigzag_hileras"]
    fallos = tuple(a.fallas.split(",")) if a.fallas else FALLOS_GENERALES
    semillas = tuple(range(200, 200 + a.semillas))
    huecos = (0.60,)
    if a.rapido:
        escs, fallos, semillas = ["recto_tope", "pasillo_macetas"], ("ninguno", "lidar_cortes"), (200, 201, 202, 203)
    jobs = [(asdict(cfg), e, h, s, f) for e in escs for f in fallos for s in semillas for h in huecos]
    print(f"[dataset] config {cfg.nombre} · {len(jobs)} episodios ({len(escs)} esc x {len(fallos)} fallas x {len(semillas)} semillas) · K={K}", flush=True)
    t0 = time.time()
    with Pool(a.procs) as pool:
        res = [r for r in pool.imap_unordered(_job, jobs, chunksize=1) if r is not None]
    if not res:
        raise SystemExit("ningún episodio produjo muestras")
    ids = np.concatenate([np.full(len(r["features"]), i) for i, r in enumerate(res)])
    cat = lambda k: np.concatenate([r[k] for r in res])
    rep = lambda k: np.concatenate([[r[k]] * len(r["features"]) for r in res])
    out = dict(features=cat("features").astype(np.float32), targets=cat("targets").astype(np.float32),
               targets_inc=cat("targets_inc").astype(np.float32), timestamp=cat("timestamp").astype(np.float32),
               est_pose=cat("est_pose").astype(np.float32), real_pose=cat("real_pose").astype(np.float32),
               escenario=rep("escenario").astype(str), falla=rep("falla").astype(str), semilla=rep("semilla").astype(np.int32),
               episodio_id=ids.astype(np.int32), nombres_features=np.array(NOMBRES), K=np.array(K),
               meta=np.array(json.dumps({"fecha": time.strftime("%Y-%m-%d %H:%M"), "config": cfg.nombre, "geometria": S.GEOMETRIA,
                                         "episodios": len(res), "sigma_lidar_opciones_m": [0.01, 0.02, 0.03],
                                         "sesgo_imu_dps": [0.2, 1.0], "ruido_imu_dps": 0.3,
                                         "hipotesis": "física del simulador sin calibrar; sesgo IMU y σ LiDAR sorteados; "
                                                      "odometría estimada = ICP punto a punto con estimación inicial (cmd, gyro)"})))
    SALIDA.mkdir(parents=True, exist_ok=True)
    ruta = Path(a.salida) if a.salida else SALIDA / f"dataset_{time.strftime('%Y%m%d_%H%M')}.npz"
    np.savez_compressed(ruta, **out)
    N = len(out["features"])
    print(f"[dataset] {N} muestras · D={out['features'].shape[1]} · {len(res)} episodios · {int(time.time()-t0)} s -> {ruta}", flush=True)
    print(f"[dataset] |error de pose| medio: {np.abs(out['targets']).mean(0)} · |error de incremento| medio: {np.abs(out['targets_inc']).mean(0)}", flush=True)


if __name__ == "__main__":
    main()
