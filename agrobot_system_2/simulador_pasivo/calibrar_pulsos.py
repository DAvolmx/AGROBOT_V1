#!/usr/bin/env python3
"""Ajusta la VARIABILIDAD POR PULSO del simulador a la del robot real (2026-10-22).

Real (telemetría de 8 sesiones, 586 movimientos sin failsafe): giro |Δrumbo| mediana 5,8° (cuartiles 1,4°–10,6°, p90 19°, 29 % < 2°);
avance recto |Δrumbo| mediana 2,4° (0,8°–4,9°, p90 9,1°). El simulador, con la física sorteada, sale mucho más regular (giro ~12° ±2°, recto ±1,2°).

Modelo añadido al mezclador del firmware simulado (sim_logica_actual._perturbar): al empezar cada pulso
    giro:  k = 0 con prob. `giro_p_muerto`, si no k ~ lognormal(0, `giro_sigma`)           (omega efectivo = omega·k)
    recto: deriva = N(0, sd) en % de omega, con sd = `deriva_sd_grande` (prob. `deriva_p_grande`) o `deriva_sd_chica`.
Se estiman con la distribución del simulador SIN variabilidad (base) y se buscan los parámetros cuyos cuantiles coincidan con los reales; luego se
VERIFICA simulando de nuevo. Salida: ~/.agrobot/calibracion/pulsos_real.json y diagrama real vs simulador.
  python3 calibrar_pulsos.py [--semillas 120] [--procs 5] [--salida CARPETA_PNG]
"""
import argparse
import csv
import glob
import json
import math
import time
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import calibracion_real as CR
import sim_logica_actual as S

SALIDA = Path.home() / ".agrobot" / "calibracion" / "pulsos_real.json"
DATOS = Path.home() / ".agrobot" / "datos"


def cuantiles_reales():
    G, F = [], []
    for d in sorted(glob.glob(str(DATOS / "2026102*"))) + sorted(glob.glob(str(DATOS / "2026101*")))[-8:]:
        try:
            txt = open(d + "/telemetria.csv", errors="ignore").read().replace("\x00", "")
        except OSError:
            continue
        rows = [r for r in csv.DictReader(txt.splitlines()) if r.get("t")]
        if len(rows) < 50:
            continue
        num = lambda r, k: float(r[k]) if r.get(k) not in ("", None) else float("nan")
        t = np.array([num(r, "t_unix") for r in rows]); gz = np.array([num(r, "giro_z") for r in rows])
        va = np.array([num(r, "v_aplicada") for r in rows]); wa = np.array([num(r, "w_aplicada") for r in rows]); fs = np.array([num(r, "c3_failsafe") for r in rows])
        mov = (np.nan_to_num(np.abs(va)) + np.nan_to_num(np.abs(wa))) > 0.01
        i = 0
        while i < len(rows):
            if mov[i]:
                j = u = i
                while j < len(rows) and (mov[j] or t[j] - t[u] < 0.3):
                    if mov[j]:
                        u = j
                    j += 1
                j = u + 1
                dur = t[j - 1] - t[i]
                if 0.2 < dur < 3.0 and np.nanmean(fs[i:j] > 0) < 0.01:
                    dt = np.clip(np.r_[np.diff(t[i:j]), 0.1], 0, 0.3)
                    dy = abs(float(np.nansum(gz[i:j] * dt)))
                    vm, wm = np.nanmean(va[i:j]), np.nanmean(wa[i:j])
                    if abs(wm) >= 0.05:
                        G.append(dy)
                    elif vm > 0.05:
                        F.append(dy)
                i = j
            else:
                i += 1
    q = lambda L: {"n": len(L), "p25": float(np.percentile(L, 25)), "p50": float(np.percentile(L, 50)), "p75": float(np.percentile(L, 75)),
                   "p90": float(np.percentile(L, 90)), "menor_2deg": float(np.mean(np.array(L) < 2.0))}
    return q(G), q(F), np.array(G), np.array(F)


def _pulso(a):
    """(yaw absoluto °, avance cm) de UN pulso con la física de la semilla. `cal` sobreescribe CAL (para medir/validar)."""
    seed, accion, modo, cal = a
    CR.CAL.update(cal)
    S.CAL.update(cal)
    hip = S.hip_por_semilla(seed)
    cfg = replace(S.config_actual(), modo=modo)
    params = replace(S.parametros_rusticos(hip.semilla_chasis), mu=hip.mu, front_drive_bias=hip.bias, rolling_drag=hip.drag, substep=S.SUBSTEP,
                     front_outer=S.PIVOTE_EXT_M, rear_outer=hip.eje_fijo or S.FIJO_EXT_M, lateral=hip.lateral)
    r = S.RobotModelo(params, (0, 0, 0))
    fw = S.crear_firmware(cfg)
    fw.omega_gain = hip.giro_gain
    fw.rng = np.random.default_rng(20_000 + seed)
    act = S.ActuadorJetson(cfg)
    con = S.Conductor(cfg, act)
    t, dt, prox = 0.0, S.TICK_FW, 0.0
    psi0, c0 = r.psi_real(), r.centro()
    con.ordenar(accion, 0.0)
    while t < 3.5:
        if t >= prox:
            con.paso(t)
            prox += S.TICK_CONDUCTOR
        act.tick(t)
        fw.recibir(act.out_v / S.V_SCALE * 100 if act.out_v else 0, act.out_w / S.W_SCALE * 100 if act.out_w else 0, t)
        r.step(dt, fw.tick(t), hip.dead)
        t += dt
    psi, c = r.psi_real(), r.centro()
    yaw = math.degrees(math.atan2(math.sin(psi - psi0), math.cos(psi - psi0)))
    return abs(yaw), 100 * (c[0] - c0[0]) * math.cos(psi0)


def correr(pool, semillas, accion, modo, cal):
    return np.array(pool.map(_pulso, [(s, accion, modo, cal) for s in semillas], chunksize=2))


def cuant(v):
    v = np.asarray(v)
    return {"p25": float(np.percentile(v, 25)), "p50": float(np.percentile(v, 50)), "p75": float(np.percentile(v, 75)), "p90": float(np.percentile(v, 90)),
            "menor_2deg": float(np.mean(v < 2.0))}


def error_cuantiles(sim, real):
    e = 0.0
    for k in ("p25", "p50", "p75", "p90"):
        e += ((sim[k] - real[k]) / max(real[k], 1.0)) ** 2
    return e + 4 * (sim["menor_2deg"] - real["menor_2deg"]) ** 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--semillas", type=int, default=120)
    ap.add_argument("--procs", type=int, default=5)
    ap.add_argument("--salida", default=str(Path.home() / ".agrobot" / "calibracion"))
    a = ap.parse_args()
    t0 = time.time()
    qG, qF, G, F = cuantiles_reales()
    print(f"[real] giros n={qG['n']}: {qG}\n[real] rectos n={qF['n']}: {qF}", flush=True)
    sem = list(range(500, 500 + a.semillas))
    sin = {"giro_p_muerto": 0.0, "giro_mu": 0.0, "giro_sigma": 0.0, "deriva_p_grande": 0.0, "deriva_sd_chica": 0.0, "deriva_sd_grande": 0.0}
    rng = np.random.default_rng(1)
    with Pool(a.procs) as pool:
        base_g = np.r_[correr(pool, sem, "izquierda", "preciso", sin)[:, 0], correr(pool, sem, "derecha", "preciso", sin)[:, 0]]
        base_f = correr(pool, sem, "adelante", "interior", sin)[:, 0]
        print("[sim base] giros", cuant(base_g), "| rectos", cuant(base_f), f"({int(time.time() - t0)} s)", flush=True)
        # sensibilidad de la deriva recta al desplazamiento de omega en % (lineal): yaw ≈ c · off
        pr = correr(pool, sem, "adelante", "interior", {**sin, "deriva_sd_chica": 8.0})[:, 0]
        c = float(np.sqrt(max(np.mean(pr ** 2) - np.mean(base_f ** 2), 1e-9)) / 8.0)
        print(f"[sim] sensibilidad deriva: {c:.3f}° por cada 1 % de omega", flush=True)
        # curva de respuesta REAL del simulador al factor k (el giro no es proporcional: la mezcla y la fricción saturan)
        ks = np.array([0.0, 0.15, 0.3, 0.5, 0.75, 1.0, 1.5, 2.2])
        resp = []
        for kk in ks:
            cal_k = {**sin, "giro_mu": float(np.log(kk)) if kk > 0 else 0.0, "giro_p_muerto": 1.0 if kk == 0 else 0.0}
            resp.append(float(np.mean(correr(pool, sem[:40], "izquierda", "preciso", cal_k)[:, 0])))
        resp = np.array(resp) / max(resp[5], 1e-6)
        g = lambda k: np.interp(k, ks, resp)
        print("[sim] respuesta del giro a k:", {float(k): round(float(r), 2) for k, r in zip(ks, resp)}, flush=True)
        N = 40000
        mejor_g = (1e9, None)
        for pm in np.linspace(0.0, 0.45, 19):
            for mu in np.linspace(-1.2, 0.8, 21):
                for sg in np.linspace(0.2, 1.8, 17):
                    k = np.where(rng.random(N) < pm, 0.0, np.minimum(2.2, np.exp(rng.normal(mu, sg, N))))
                    y = np.abs(rng.choice(base_g, N) * g(k))
                    e = error_cuantiles(cuant(y), qG)
                    if e < mejor_g[0]:
                        mejor_g = (e, {"giro_p_muerto": round(float(pm), 3), "giro_mu": round(float(mu), 3), "giro_sigma": round(float(sg), 3)})
        mejor_f = (1e9, None)
        for sc in np.linspace(0.2, 8.0, 40):
            for sb in np.linspace(3.0, 30.0, 28):
                for pb in np.linspace(0.0, 0.5, 11):
                    sd = np.where(rng.random(N) < pb, sb, sc)
                    y = np.abs(rng.choice(base_f, N) + c * rng.normal(0, 1, N) * sd * np.sign(rng.normal(0, 1, N)) * 0 + c * rng.normal(0, 1, N) * sd)
                    e = error_cuantiles(cuant(y), qF)
                    if e < mejor_f[0]:
                        mejor_f = (e, {"deriva_sd_chica": round(float(sc), 3), "deriva_sd_grande": round(float(sb), 3), "deriva_p_grande": round(float(pb), 3)})
        par = {**mejor_g[1], **mejor_f[1]}
        print("[ajuste]", par, f"errores {mejor_g[0]:.3f} / {mejor_f[0]:.3f}", flush=True)
        # ---- verificación con el simulador de verdad
        sem2 = list(range(900, 900 + a.semillas))
        val_g = np.r_[correr(pool, sem2, "izquierda", "preciso", par)[:, 0], correr(pool, sem2, "derecha", "preciso", par)[:, 0]]
        val_f = correr(pool, sem2, "adelante", "interior", par)[:, 0]
    sim_g, sim_f = cuant(val_g), cuant(val_f)
    print("[verificación] giros sim", sim_g, "\n[verificación] rectos sim", sim_f, flush=True)
    res = {"fecha": time.strftime("%Y-%m-%d %H:%M"), "parametros": par, "sensibilidad_deriva_deg_por_pct": c,
           "real": {"giro": qG, "recto": qF}, "sim_antes": {"giro": cuant(base_g), "recto": cuant(base_f)},
           "sim_despues": {"giro": sim_g, "recto": sim_f},
           "nota": "cuantiles de |Δrumbo| por pulso; real = 586 movimientos de 8 sesiones sin failsafe; mezcla de modos de pulso, por eso el ajuste es aproximado"}
    SALIDA.parent.mkdir(parents=True, exist_ok=True)
    SALIDA.write_text(json.dumps(res, indent=1, ensure_ascii=False))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out = Path(a.salida); out.mkdir(parents=True, exist_ok=True)
    fig, axs = plt.subplots(1, 2, figsize=(14, 4.2))
    for ax, titulo, real, antes, despues, lim in ((axs[0], "Giro: |Δrumbo| por pulso", G, base_g, val_g, 45), (axs[1], "Avance recto: |Δrumbo| por pulso", F, base_f, val_f, 25)):
        for datos, nom, col in ((real, "REAL", "k"), (antes, "simulador antes", "tab:blue"), (despues, "simulador calibrado", "tab:green")):
            x = np.sort(datos)
            ax.plot(x, np.arange(1, len(x) + 1) / len(x), label=f"{nom} (n={len(x)})", color=col, lw=2.2 if nom == "REAL" else 1.6)
        ax.set_xlim(0, lim); ax.set_xlabel("|Δrumbo| [°]"); ax.set_ylabel("fracción acumulada"); ax.set_title(titulo); ax.grid(alpha=.3); ax.legend()
    fig.tight_layout(); fig.savefig(out / "cal_03_pulsos_real_vs_simulador.png", dpi=110); plt.close(fig)
    print(f"listo en {int(time.time() - t0)} s -> {SALIDA}", flush=True)


if __name__ == "__main__":
    main()
