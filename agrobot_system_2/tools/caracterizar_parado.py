#!/usr/bin/env python3
"""Caracteriza el ROBOT PARADO (2026-10-22): ruido real del LIDAR y del IMU para calibrar el simulador.

Lee la sesión grabada más reciente con el robot quieto (`~/.agrobot/datos/*/`: scans_*.npz + telemetria.csv) y mide:
  LIDAR  desviación del rango por rayo en función de la distancia (σ(r) ≈ a + b·r), probabilidad de pérdida de un rayo estable (dropout) por
         tramo de distancia, retornos falsos cercanos, escalón de cuantización, periodo y jitter de los scans.
  IMU    media/desv. de acelerómetro y giroscopio por eje, sesgo del giroscopio, desviación de las medias de 1 s (deriva), jitter de la telemetría.
Salida: ~/.agrobot/calibracion/real_parado.json y figuras PNG (histogramas) en la carpeta indicada.
  python3 tools/caracterizar_parado.py [--sesion CARPETA] [--salida CARPETA_PNG] [--bloques 160]
"""
import argparse
import csv
import glob
import json
import time
from pathlib import Path

import numpy as np

RAIZ = Path.home() / ".agrobot" / "datos"
SALIDA_JSON = Path.home() / ".agrobot" / "calibracion" / "real_parado.json"
BINS = [(0.2, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 8.0)]


def elegir_sesion():
    for d in sorted(glob.glob(str(RAIZ / "2026*")), reverse=True):
        try:
            txt = open(d + "/telemetria.csv", errors="ignore").read().replace("\x00", "")
        except OSError:
            continue
        rows = [r for r in csv.DictReader(txt.splitlines()) if r.get("t")]
        if len(rows) < 3000:
            continue
        v = np.array([abs(float(r["v_aplicada"] or 0)) + abs(float(r["w_aplicada"] or 0)) for r in rows])
        if (v < 0.01).mean() > 0.97 and glob.glob(d + "/scans_*.npz"):
            return Path(d), rows
    raise SystemExit("no hay una sesión con el robot quieto")


def lidar(ses: Path, n_bloques: int):
    fs = sorted(glob.glob(str(ses / "scans_*.npz")))
    idx = np.linspace(0, len(fs) - 1, min(n_bloques, len(fs))).astype(int)
    sig = {b: [] for b in BINS}
    drop = {b: [] for b in BINS}
    falsos, dts, pasos, validos, total = 0, [], [], 0, 0
    n_falsos_den = 0
    for i in idx:
        b = np.load(fs[i])
        D = b["distancias_m"].astype(np.float64)
        T = b["t"]
        dts += list(np.diff(T))
        D = np.where((D > 0.12) & (D < 12.0), D, np.nan)
        validos += np.isfinite(D).sum()
        total += D.size
        med = np.nanmedian(D, 0)
        frac_val = np.isfinite(D).mean(0)
        est = frac_val > 0.5                                         # rayos con retorno casi siempre (estables)
        for lo, hi in BINS:
            m = est & (med >= lo) & (med < hi)
            if m.sum() < 5:
                continue
            sd = np.nanstd(D[:, m], 0)
            sig[(lo, hi)] += list(sd[np.isfinite(sd) & (sd < 0.5)])           # fuera los rayos en un borde (saltan de objeto)
            drop[(lo, hi)].append(1.0 - frac_val[m].mean())
        lejos = est & (med > 1.0)
        if lejos.any():
            falsos += int((D[:, lejos] < 0.3).sum())
            n_falsos_den += int(np.isfinite(D[:, lejos]).sum())
        v = np.sort(np.unique(np.round(D[np.isfinite(D)], 4)))
        if len(v) > 50:
            pasos.append(float(np.median(np.diff(v))))
    dts = np.array(dts)
    dts = dts[(dts > 0) & (dts < 2)]
    por_bin = {}
    x, y = [], []
    for (lo, hi), L in sig.items():
        if len(L) < 20:
            continue
        L = np.array(L)
        por_bin[f"{lo}-{hi}"] = {"sigma_mm": round(float(np.median(L)) * 1000, 2), "sigma_p90_mm": round(float(np.percentile(L, 90)) * 1000, 2),
                                 "dropout_pct": round(100 * float(np.mean(drop[(lo, hi)])), 2), "rayos": int(len(L))}
        x.append((lo + hi) / 2)
        y.append(float(np.median(L)))
    ab = np.polyfit(x, y, 1) if len(x) >= 2 else [0.0, 0.01]
    return {"scans_analizados": int(total / 720), "rayos_validos_pct": round(100 * validos / max(total, 1), 2), "por_distancia": por_bin,
            "sigma_m": {"a": round(float(ab[1]), 5), "b": round(float(ab[0]), 5), "nota": "sigma(r) = a + b*r"},
            "retornos_falsos_cercanos_pct": round(100 * falsos / max(n_falsos_den, 1), 4),
            "escalon_cuantizacion_mm": round(float(np.median(pasos)) * 1000, 2) if pasos else None,
            "periodo_scan_s": {"mediana": round(float(np.median(dts)), 4), "desv": round(float(dts.std()), 4)}}, sig


def imu(rows):
    num = lambda k: np.array([float(r[k]) if r.get(k) not in ("", None) else np.nan for r in rows])
    t = num("t_unix")
    out = {}
    for nombre, cols in (("acelerometro_g", ("acc_x", "acc_y", "acc_z")), ("giroscopio_dps", ("giro_x", "giro_y", "giro_z"))):
        d = {}
        for c in cols:
            x = num(c)
            x = x[np.isfinite(x)]
            if len(x) < 100:
                continue
            n10 = len(x) // 10
            medias_1s = x[: n10 * 10].reshape(n10, 10).mean(1)
            d[c] = {"media": round(float(x.mean()), 4), "desv": round(float(x.std()), 4), "desv_de_medias_1s": round(float(medias_1s.std()), 4)}
        out[nombre] = d
    dt = np.diff(t)
    dt = dt[(dt > 0) & (dt < 2)]
    out["telemetria_dt_s"] = {"mediana": round(float(np.median(dt)), 4), "desv": round(float(dt.std()), 4)}
    return out


def figuras(sig, rows, salida: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    salida.mkdir(parents=True, exist_ok=True)
    fig, axs = plt.subplots(1, 2, figsize=(13, 4))
    for (lo, hi), L in sig.items():
        if len(L) >= 20:
            axs[0].hist(np.array(L) * 1000, bins=60, range=(0, 60), histtype="step", density=True, label=f"{lo}-{hi} m")
    axs[0].set_xlabel("σ del rango entre scans [mm]"); axs[0].set_title("Ruido real del LIDAR por distancia (robot parado)"); axs[0].legend(); axs[0].grid(alpha=.3)
    num = lambda k: np.array([float(r[k]) if r.get(k) not in ("", None) else np.nan for r in rows])
    for c, col in (("giro_x", "C0"), ("giro_y", "C1"), ("giro_z", "C2")):
        x = num(c)
        axs[1].hist(x[np.isfinite(x)], bins=100, range=(-3, 3), histtype="step", density=True, label=c, color=col)
    axs[1].set_xlabel("giroscopio [°/s]"); axs[1].set_title("Ruido real del giroscopio (robot parado)"); axs[1].legend(); axs[1].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(salida / "cal_01_ruido_lidar_giroscopio.png", dpi=110); plt.close(fig)
    fig, axs = plt.subplots(1, 3, figsize=(15, 3.6))
    for ax, c in zip(axs, ("acc_x", "acc_y", "acc_z")):
        x = num(c); x = x[np.isfinite(x)]
        ax.hist(x, bins=100, color="#3b6ea5"); ax.set_title(f"{c} [g]  σ={x.std():.3f}"); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(salida / "cal_02_acelerometro.png", dpi=110); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sesion", default=None)
    ap.add_argument("--salida", default=str(Path.home() / ".agrobot" / "calibracion"))
    ap.add_argument("--bloques", type=int, default=160)
    a = ap.parse_args()
    if a.sesion:
        ses = Path(a.sesion)
        txt = open(ses / "telemetria.csv", errors="ignore").read().replace("\x00", "")
        rows = [r for r in csv.DictReader(txt.splitlines()) if r.get("t")]
    else:
        ses, rows = elegir_sesion()
    print("sesión:", ses.name, "·", len(rows), "filas de telemetría", flush=True)
    l, sig = lidar(ses, a.bloques)
    i = imu(rows)
    res = {"fecha": time.strftime("%Y-%m-%d %H:%M"), "sesion": ses.name, "lidar": l, "imu": i}
    SALIDA_JSON.parent.mkdir(parents=True, exist_ok=True)
    SALIDA_JSON.write_text(json.dumps(res, indent=1, ensure_ascii=False))
    figuras(sig, rows, Path(a.salida))
    print(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
