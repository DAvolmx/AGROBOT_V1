"""Reentrena el corrector de odometría por scans (MLP 60→128→64→32→3, el mismo del notebook de Kaggle) con el dataset SIMULADO + las
grabaciones REALES de hoy (pseudo-referencia = pose de slam_toolbox), y lo evalúa en una sesión real NO vista.

Variantes (misma arquitectura, mismos hiperparámetros del notebook: Huber, Adam 1e-3, batch 256 aquí por tiempo, paciencia 10):
  base        sin corrección (ICP crudo)
  existente   los pesos de models/slam_correccion (entrenados solo con simulación el 2026-10-21)
  sim+real    simulación + sesiones reales de entrenamiento (peso `--peso-real`)
Métrica: pose integrada (ICP + corrección) contra la pose del SLAM en la sesión retenida: ATE (m, alineación rígida) y RPE a 1 m (m, °).
Exporta el modelo final (entrenado con TODAS las sesiones reales) a ~/.agrobot/slam/modelo_hoy/{pesos_mlp.npz,scaler.pkl} (formato de
slam_correccion.py). NO despliega nada: `slam.correccion_ml_activa` sigue apagado.
Uso: python3 entrenar_slam_local.py [--peso-real 20] [--retener 2] [--epocas 60]"""
import argparse, json, math, os, pickle, time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

# Rutas por variable de entorno para correrlo igual en Kaggle: AGROBOT_SLAM_DIR (dataset_calibrado.npz + real_hoy.npz), AGROBOT_PESOS (pesos_mlp.npz
# existentes) y AGROBOT_SALIDA (dónde exportar).
HOME = Path(os.environ.get("AGROBOT_SLAM_DIR", Path.home() / ".agrobot" / "slam"))
BACKEND = Path(os.environ.get("AGROBOT_PESOS", Path(__file__).resolve().parent.parent / "jetson" / "backend" / "models" / "slam_correccion"))
SALIDA = Path(os.environ.get("AGROBOT_SALIDA", HOME / "modelo_hoy"))
DEV = "cuda" if torch.cuda.is_available() else "cpu"


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def componer(p, inc):
    c, s = math.cos(p[2]), math.sin(p[2])
    return np.array([p[0] + c * inc[0] - s * inc[1], p[1] + s * inc[0] + c * inc[1], wrap(p[2] + inc[2])])


def relativa(pj, pk):
    c, s = math.cos(pj[2]), math.sin(pj[2])
    dx, dy = pk[0] - pj[0], pk[1] - pj[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap(pk[2] - pj[2])])


def ate(est, real):
    a, b = est[:, :2], real[:, :2]
    ma, mb = a.mean(0), b.mean(0)
    U, _, Vt = np.linalg.svd((a - ma).T @ (b - mb))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    e = (a - ma) @ R.T + mb - b
    return float(np.sqrt((e ** 2).sum(1).mean()))


def rpe(est, real, paso=1.0):
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(real[:, :2], axis=0).T))]
    et, er = [], []
    for i in range(len(real)):
        j = np.searchsorted(s, s[i] + paso)
        if j >= len(real):
            break
        dr, de = relativa(real[i], real[j]), relativa(est[i], est[j])
        et.append(math.hypot(*(dr[:2] - de[:2])))
        er.append(abs(math.degrees(wrap(dr[2] - de[2]))))
    return (float(np.mean(et)) if et else float("nan")), (float(np.mean(er)) if er else float("nan"))


def integrar(real, est_inc, corr=None):
    """Pose integrada a partir de los incrementos estimados (+ corrección), arrancando en la pose real."""
    p = real[0].copy()
    out = [p.copy()]
    for k in range(1, len(real)):
        inc = est_inc[k] + (corr[k] if corr is not None else 0)
        p = componer(p, inc)
        out.append(p.copy())
    return np.array(out)


def mlp():
    return nn.Sequential(nn.Linear(60, 128), nn.ReLU(), nn.Linear(128, 64), nn.ReLU(), nn.Linear(64, 32), nn.ReLU(), nn.Linear(32, 3)).to(DEV)


def entrenar(X, Y, w, Xdv, Ydv, epocas, nombre):
    mu_x, sd_x = X.mean(0), X.std(0) + 1e-6
    mu_y, sd_y = Y.mean(0), Y.std(0) + 1e-9
    nx = lambda A: torch.tensor(((A - mu_x) / sd_x).astype(np.float32), device=DEV)
    ny = lambda A: torch.tensor(((A - mu_y) / sd_y).astype(np.float32), device=DEV)
    Xt, Yt, Wt = nx(X), ny(Y), torch.tensor(w.astype(np.float32), device=DEV)
    Xd, Yd = nx(Xdv), ny(Ydv)
    m = mlp()
    opt = torch.optim.Adam(m.parameters(), lr=1e-3)
    hub = nn.HuberLoss(delta=1.0, reduction="none")
    mejor, mala, best = math.inf, 0, None
    g = torch.Generator().manual_seed(0)
    t0 = time.time()
    for ep in range(epocas):
        m.train()
        perm = torch.randperm(len(Xt), generator=g).to(DEV)
        for i in range(0, len(perm), 256):
            b = perm[i:i + 256]
            opt.zero_grad()
            l = (hub(m(Xt[b]), Yt[b]).mean(1) * Wt[b]).sum() / Wt[b].sum()
            l.backward()
            opt.step()
        m.eval()
        with torch.no_grad():
            lv = float(hub(m(Xd), Yd).mean())
        if lv < mejor - 1e-5:
            mejor, mala, best = lv, 0, {k: v.clone() for k, v in m.state_dict().items()}
        else:
            mala += 1
        if mala >= 10:
            break
    m.load_state_dict(best)
    m.eval()
    print(f"  [{nombre}] {ep + 1} épocas · dev {mejor:.4f} · {time.time() - t0:.0f} s", flush=True)
    return m, (mu_x, sd_x, mu_y, sd_y)


def predecir(m, esc, X):
    mu_x, sd_x, mu_y, sd_y = esc
    with torch.no_grad():
        return m(torch.tensor(((X - mu_x) / sd_x).astype(np.float32), device=DEV)).cpu().numpy() * sd_y + mu_y


def exportar(m, esc, destino: Path):
    destino.mkdir(parents=True, exist_ok=True)
    mu_x, sd_x, mu_y, sd_y = esc
    W = [p.detach().cpu().numpy().astype(np.float32) for p in m.parameters()]
    nombres = np.array(["v_cmd", "w_cmd", "gyro_z", "icp_residual", "dmin_front", "dmin_izq", "dmin_der", "prev_dx", "prev_dy", "prev_dth", "scan_ok", "dt"])
    np.savez(destino / "pesos_mlp.npz", W1=W[0], b1=W[1], W2=W[2], b2=W[3], W3=W[4], b3=W[5], W4=W[6], b4=W[7], x_mean=mu_x, x_scale=sd_x,
             y_mean=mu_y, y_scale=sd_y, K=np.array(5), nombres=nombres)
    with open(destino / "scaler.pkl", "wb") as f:
        pickle.dump({"mean": mu_x, "scale": sd_x, "y_mean": mu_y, "y_scale": sd_y, "K": 5, "nombres": list(nombres)}, f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--peso-real", type=float, default=20.0)
    ap.add_argument("--retener", type=int, default=2, help="índice de la sesión real que NO se usa para entrenar")
    ap.add_argument("--epocas", type=int, default=60)
    ap.add_argument("--sim-max", type=int, default=120_000)
    a = ap.parse_args()
    sim = np.load(HOME / "dataset_calibrado.npz", allow_pickle=False)
    real = np.load(HOME / "real_hoy.npz", allow_pickle=False)
    rng = np.random.default_rng(0)
    isim = np.sort(rng.choice(len(sim["features"]), min(a.sim_max, len(sim["features"])), replace=False)) if a.sim_max > 0 else np.array([], dtype=int)
    Xs, Ys = sim["features"][isim].astype(np.float32), sim["targets_inc"][isim].astype(np.float32)
    Xr, Yr, EP = real["features"].astype(np.float32), real["targets_inc"].astype(np.float32), real["episodio_id"]
    ok = np.isfinite(Xr).all(1) & np.isfinite(Yr).all(1) & (np.abs(Yr[:, :2]).max(1) < 0.5)
    Xs = np.nan_to_num(Xs)
    print(f"sim {len(Xs)} · real {int(ok.sum())} de {len(Xr)} · sesiones {np.unique(EP).tolist()} · retenida {a.retener} · dispositivo {DEV}")
    res = {"peso_real": a.peso_real, "retenida": int(a.retener), "sesiones": {}}
    tr = ok & (EP != a.retener)
    te = (EP == a.retener) & ok
    # dev: 15 % de las filas reales de entrenamiento (por bloques contiguos, no mezcladas) + 5 % de sim
    idx = np.where(tr)[0]
    dv_real = idx[int(0.85 * len(idx)):]
    tr_real = idx[:int(0.85 * len(idx))]
    n_dv_s = int(0.05 * len(Xs))
    if len(Xs) == 0:
        Xs, Ys = np.zeros((0, 60), np.float32), np.zeros((0, 3), np.float32)
    X = np.concatenate([Xs[n_dv_s:], Xr[tr_real]])
    Y = np.concatenate([Ys[n_dv_s:], Yr[tr_real]])
    w = np.concatenate([np.ones(len(Xs) - n_dv_s), np.full(len(tr_real), a.peso_real)])
    Xdv = np.concatenate([Xs[:n_dv_s], Xr[dv_real]])
    Ydv = np.concatenate([Ys[:n_dv_s], Yr[dv_real]])
    m, esc = entrenar(X, Y, w, Xdv, Ydv, a.epocas, "sim+real")
    # modelo existente (solo simulación)
    pe = np.load(BACKEND / "pesos_mlp.npz")

    def fwd_exist(Xe):
        z = (Xe - pe["x_mean"]) / pe["x_scale"]
        for i, k in enumerate(("1", "2", "3", "4")):
            z = z @ pe["W" + k].T + pe["b" + k]
            if i < 3:
                z = np.maximum(z, 0)
        return z * pe["y_scale"] + pe["y_mean"]

    def evaluar(sel, nombre):
        real_p = real["real_pose"][sel]
        est_inc = np.zeros((len(real_p), 3))
        # incremento ICP estimado = referencia - error del incremento (targets_inc = real - est)
        ref_inc = np.array([np.zeros(3)] + [relativa(real_p[k - 1], real_p[k]) for k in range(1, len(real_p))])
        est_inc = ref_inc - real["targets_inc"][sel]
        Xe = real["features"][sel].astype(np.float32)
        filas = {}
        for n, corr in (("base", None), ("existente", np.clip(fwd_exist(Xe), -0.05, 0.05)), ("sim+real", np.clip(predecir(m, esc, Xe), -0.05, 0.05))):
            traj = integrar(real_p, est_inc, corr)
            a_, (rt, rr) = ate(traj, real_p), rpe(traj, real_p)
            filas[n] = {"ATE_m": round(a_, 3), "RPE1m_m": round(rt, 3), "RPE1m_deg": round(rr, 2),
                        "error_final_m": round(float(np.hypot(*(traj[-1, :2] - real_p[-1, :2]))), 3)}
        res["sesiones"][nombre] = {"muestras": int(len(real_p)), "recorrido_m": round(float(np.hypot(*np.diff(real_p[:, :2], axis=0).T).sum()), 1), **filas}
        print(nombre, json.dumps(filas), flush=True)

    evaluar(np.where(te)[0], f"retenida_{a.retener}")
    for k in np.unique(EP):
        if k != a.retener:
            evaluar(np.where((EP == k) & ok)[0], f"entrenamiento_{int(k)}")
    # modelo final con TODAS las sesiones reales
    full = ok
    X2 = np.concatenate([Xs[n_dv_s:], Xr[full]])
    Y2 = np.concatenate([Ys[n_dv_s:], Yr[full]])
    w2 = np.concatenate([np.ones(len(Xs) - n_dv_s), np.full(int(full.sum()), a.peso_real)])
    mf, esc_f = entrenar(X2, Y2, w2, Xdv, Ydv, a.epocas, "final (todas las sesiones)")
    exportar(mf, esc_f, SALIDA)
    (SALIDA / "resultados.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    print("modelo exportado a", SALIDA)


if __name__ == "__main__":
    main()
