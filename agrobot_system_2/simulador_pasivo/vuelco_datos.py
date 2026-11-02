"""Datos para entrenar el DETECTOR DE VUELCO (jetson/backend/detector_vuelco.py), 2026-11-02.

Dos fuentes:
  · REALES (negativos): toda la telemetría grabada (~/.agrobot/datos/*/telemetria.csv, IMU a ~10 Hz). Ninguna de esas sesiones tiene un vuelco
    documentado (el del 2026-10-09 no se conservó), así que sirven para medir FALSAS ALARMAS mientras el robot se mueve. Cada sesión se NIVELA con la
    mediana de la gravedad en reposo (como imu_nivel): antes del 20-oct el montaje daba ~12° de balanceo falso.
  · SIMULADAS (positivos y negativos duros): el péndulo sobre la arista de fallos_3d.py (vuelco lateral al subir un bulto con una rueda, vuelco en giro
    rápido, caída de morro) y casi-vuelcos que se recuperan, taludes sostenidos y baches. La señal del IMU = gravedad girada + aceleración del cuerpo
    + RUIDO REAL (golpes de los pulsos sacados de la telemetría: residuo del acelerómetro y giroscopio en marcha), muestreada a ~10 Hz con jitter y
    paquetes repetidos como llega del nodo de sensores.

    python3 vuelco_datos.py --extraer      # lee los CSV y deja ~/.agrobot/vuelco/real_imu.npz (una vez, ~3 min)
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import os
from pathlib import Path

import numpy as np

from fallos_3d import Estabilidad, _inercia_balanceo, G

CACHE = Path.home() / ".agrobot" / "vuelco"
REAL_NPZ = CACHE / "real_imu.npz"
COLS = ("t", "acc_x", "acc_y", "acc_z", "giro_x", "giro_y", "giro_z", "v_aplicada", "w_aplicada")


# ----------------------------------------------------------------------------------------------- reales
def _rot_nivel(g):
    """Matriz que lleva g a (0,0,|g|) (Rodrigues), como imu_nivel.matriz_nivel."""
    g = np.asarray(g, float)
    n = np.linalg.norm(g)
    if n < 0.5:
        return np.eye(3)
    a = g / n
    b = np.array([0.0, 0.0, 1.0])
    v = np.cross(a, b)
    c = float(a @ b)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1 / (1 + c))


def extraer():
    """Todas las sesiones con IMU -> un npz con las series concatenadas y los cortes por sesión."""
    series, nombres, cortes, ofs = [], [], [0], []
    for f in sorted(glob.glob(os.path.expanduser("~/.agrobot/datos/*/telemetria.csv"))):
        filas = []
        try:
            with open(f) as fh:
                r = csv.reader(fh)
                h = next(r)
                if "acc_z" not in h:
                    continue
                idx = [h.index(k) for k in COLS]
                for row in r:
                    try:
                        filas.append([float(row[i]) if row[i] != "" else 0.0 for i in idx])
                    except (ValueError, IndexError):
                        pass
        except (OSError, StopIteration):
            continue
        if len(filas) < 100:
            continue
        a = np.array(filas, np.float32)
        a = a[np.argsort(a[:, 0], kind="stable")]
        acc = a[:, 1:4].astype(float)
        mag = np.linalg.norm(acc, axis=1)
        mov = (np.abs(a[:, 7]) > 0.01) | (np.abs(a[:, 8]) > 0.01)
        quieto = (~mov) & (np.abs(mag - 1.0) < 0.05) & (np.linalg.norm(a[:, 4:6], axis=1) < 3.0)
        if quieto.sum() > 50:
            g = np.median(acc[quieto], axis=0)
            R = _rot_nivel(g)
            ofs.append(math.degrees(math.acos(max(-1, min(1, g[2] / np.linalg.norm(g))))))
            a[:, 1:4] = (R @ acc.T).T
            a[:, 4:7] = (R @ a[:, 4:7].astype(float).T).T
        else:
            ofs.append(float("nan"))
        series.append(a)
        nombres.append(Path(f).parent.name)
        cortes.append(cortes[-1] + len(a))
    CACHE.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(REAL_NPZ, datos=np.concatenate(series), cortes=np.array(cortes), nombres=np.array(nombres), offset_deg=np.array(ofs))
    horas = sum((s[-1, 0] - s[0, 0]) for s in series) / 3600
    print(f"{len(series)} sesiones, {sum(len(s) for s in series)} muestras, {horas:.1f} h -> {REAL_NPZ}")


def cargar_reales():
    z = np.load(REAL_NPZ, allow_pickle=False)
    d, c, n = z["datos"], z["cortes"], z["nombres"]
    return [(str(n[i]), d[c[i]:c[i + 1]]) for i in range(len(n))]


def banco_ruido(sesiones, s_min="20261020"):
    """Trozos de ruido REAL en marcha: residuo del acelerómetro respecto a su mediana de 1 s, y el giroscopio crudo. Solo sesiones ya calibradas."""
    acc_r, gyr = [], []
    for nombre, a in sesiones:
        if nombre < s_min:
            continue
        mov = (np.abs(a[:, 7]) > 0.01) | (np.abs(a[:, 8]) > 0.01)
        if mov.sum() < 50:
            continue
        acc = a[:, 1:4].astype(float)
        k = 10
        if len(acc) < 3 * k:
            continue
        med = np.stack([np.convolve(np.pad(acc[:, i], (k // 2, k - k // 2 - 1), mode="edge"), np.ones(k) / k, mode="valid") for i in range(3)], 1)
        res = acc - med
        # trozos de 6 s seguidos en marcha (con lo que tengan de golpes)
        i = 0
        while i + 60 < len(a):
            if mov[i:i + 60].mean() > 0.5:
                acc_r.append(res[i:i + 60])
                gyr.append(a[i:i + 60, 4:7].astype(float))
            i += 60
    return np.array(acc_r), np.array(gyr)


# ----------------------------------------------------------------------------------------------- simuladas
def _trayectoria_pendulo(forcing, d, i_cg, m, h, phi0, phidot0, t_max, dt=2e-3, reposo_deg=None, rng=None):
    """phi(t) del péndulo sobre la arista (como fallos_3d._simular_pendulo, pero guardando la serie). Si vuelca sigue cayendo hasta `reposo_deg`
    (el robot tumbado de lado) con un rebote amortiguado. Devuelve (t, phi_rad, t_tip|None)."""
    r = math.hypot(d, h)
    a0 = math.atan2(h, d)
    phi_tip = math.atan2(d, h)
    i_pivot = i_cg + m * r * r
    phi, w = phi0, phidot0
    ts, ps = [], []
    t_tip = None
    reposo = math.radians(reposo_deg if reposo_deg is not None else 80.0)
    for k in range(int(t_max / dt)):
        t = k * dt
        if t_tip is None:
            f = forcing(t) if callable(forcing) else forcing
            tau = m * f * r * math.sin(a0 + phi) - m * G * r * math.cos(a0 + phi)
            w += (tau / i_pivot) * dt
            phi += w * dt
            if phi >= phi_tip:
                t_tip = t
            if phi <= 0.0 and w <= 0.0:
                phi, w = 0.0, 0.0
        else:
            if phi < reposo:
                tau = m * G * r * math.sin(phi - phi_tip) + 1e-3
                w += (tau / i_pivot) * dt
                phi += w * dt
            if phi >= reposo:
                phi, w = reposo, -0.25 * w if abs(w) > 0.3 else 0.0     # golpe contra el suelo: rebote corto
        ts.append(t)
        ps.append(phi)
    return np.array(ts), np.array(ps), t_tip


def _imu_desde_angulos(t, roll, pitch, a_lat, a_lon, ruido, rng, hz=10.0, stale=0.08, offset_deg=2.0):
    """Muestrea a ~hz con jitter: acc = gravedad girada + aceleración del cuerpo (en g) + ruido real; gyro = derivada de los ángulos (°/s) + ruido real.
    Ejes: x lateral (balanceo positivo = cae a la derecha -> +x), y adelante, z arriba."""
    tq = [0.0]
    while tq[-1] < t[-1]:
        tq.append(tq[-1] + rng.uniform(0.8, 1.25) / hz)
    tq = np.array(tq[:-1])
    ro = np.interp(tq, t, roll) + math.radians(rng.normal(0, offset_deg))
    pi = np.interp(tq, t, pitch) + math.radians(rng.normal(0, offset_deg))
    gx = np.sin(ro) * np.cos(pi)
    gy = np.sin(pi)
    gz = np.cos(ro) * np.cos(pi)
    acc = np.stack([gx + np.interp(tq, t, a_lat) / G, gy + np.interp(tq, t, a_lon) / G, gz], 1)
    dro = np.degrees(np.gradient(np.interp(tq, t, roll), tq))
    dpi = np.degrees(np.gradient(np.interp(tq, t, pitch), tq))
    gyro = np.stack([dpi, dro, np.zeros_like(tq)], 1)          # gyro X = cabeceo (eje lateral), gyro Y = balanceo (eje de avance)
    if ruido is not None:
        acc_r, gyr_r = ruido
        n = len(tq)
        idx = []
        while len(idx) < n:
            j = rng.integers(len(acc_r))
            idx.extend([(j, k) for k in range(acc_r.shape[1])])
        idx = idx[:n]
        acc += np.array([acc_r[j][k] for j, k in idx])
        gyro += np.array([gyr_r[j][k] for j, k in idx])
    for i in range(1, len(tq)):                                 # paquetes repetidos (el nodo no mandó uno nuevo)
        if rng.random() < stale:
            acc[i], gyro[i] = acc[i - 1], gyro[i - 1]
    return tq, acc, gyro


def episodio(tipo: str, rng, ruido, est: Estabilidad | None = None) -> dict:
    """Un episodio simulado. tipo: obstaculo | giro | morro | casi | talud | bache | llano. Devuelve {t, acc, gyro, t_tip, tipo, info}."""
    est = est or Estabilidad()
    p = est.p
    m, h, d = est.m, est.h, est.d_lateral()
    i_cg = _inercia_balanceo(m, p.rear_outer, 2 * h)
    pre = rng.uniform(3.0, 8.0)                 # conducción normal antes del suceso
    dur = pre + rng.uniform(4.0, 8.0)
    dt = 2e-3
    t = np.arange(0, dur, dt)
    roll = np.zeros_like(t)
    pitch = np.zeros_like(t)
    a_lat = np.zeros_like(t)
    a_lon = np.zeros_like(t)
    t_tip = None
    signo = 1.0 if rng.random() < 0.5 else -1.0
    info = {}

    def pegar(serie, ts, ys):
        k0 = int(pre / dt)
        n = min(len(ys), len(serie) - k0)
        serie[k0:k0 + n] = ys[:n]
        serie[k0 + n:] = ys[n - 1]

    if tipo in ("obstaculo", "casi"):
        h_obs = rng.uniform(0.03, 0.10)
        v = rng.uniform(0.15, 0.9)
        acople = rng.uniform(0.15, 0.5)
        phi0 = math.atan2(h_obs, 2 * est.hf)
        phidot0 = acople * v / (2 * est.hf) * (1.0 + getattr(p, "susp_recoil", 0.0))
        if tipo == "casi":                       # rebaja la energía para que NO vuelque pero llegue cerca
            phidot0 *= rng.uniform(0.3, 0.95)
        ts, ps, tt = _trayectoria_pendulo(0.0, d, i_cg, m, h, phi0, phidot0, dur - pre, reposo_deg=rng.uniform(55, 100))
        if tipo == "casi" and tt is not None:
            return episodio("casi", rng, ruido, est)
        pegar(roll, ts, signo * ps)
        t_tip = None if tt is None else pre + tt
        info = {"h_obs_cm": round(h_obs * 100, 1), "v": round(v, 2), "acople": round(acople, 2)}
    elif tipo == "giro":
        v = rng.uniform(0.4, 1.5)
        a_y_crit = d / h * G
        a_y = a_y_crit * rng.uniform(1.02, 1.6)
        ts, ps, tt = _trayectoria_pendulo(a_y, d, i_cg, m, h, 0.0, 0.0, dur - pre, reposo_deg=rng.uniform(55, 100))
        pegar(roll, ts, signo * ps)
        a_lat[int(pre / dt):] = -signo * a_y
        if tt is not None:
            a_lat[int((pre + tt) / dt):] = 0.0
        t_tip = None if tt is None else pre + tt
        info = {"a_y": round(a_y, 2), "v": round(v, 2)}
    elif tipo == "morro":                        # cae de morro (hoyo, escalón, cuesta abajo): cabeceo hasta tumbarse
        caida = rng.uniform(0.3, 1.2)
        fin = math.radians(rng.uniform(50, 90))
        ts = np.arange(0, dur - pre, dt)
        ps = np.minimum(1.0, (ts / caida) ** 2) * fin
        pegar(pitch, ts, signo * ps)
        t_tip = pre + caida * math.sqrt(math.radians(40) / fin) if fin > math.radians(40) else None
        info = {"caida_s": round(caida, 2), "fin_deg": round(math.degrees(fin))}
    elif tipo == "talud":                        # pendiente sostenida: NO es vuelco (como mucho precaución)
        ang_r = math.radians(rng.uniform(5, 24))
        ang_p = math.radians(rng.uniform(-20, 20))
        sub = rng.uniform(0.5, 2.0)
        k0 = int(pre / dt)
        rampa = np.minimum(1.0, (t[k0:] - pre) / sub)
        roll[k0:] = signo * ang_r * rampa
        pitch[k0:] = ang_p * rampa
        info = {"roll_deg": round(math.degrees(ang_r), 1), "pitch_deg": round(math.degrees(ang_p), 1)}
    elif tipo == "bache":                        # golpes sueltos que inclinan un instante (rueda que cae en un surco)
        k = int(pre / dt)
        for _ in range(rng.integers(1, 5)):
            c = k + rng.integers(0, int(3.0 / dt))
            w = int(rng.uniform(0.1, 0.5) / dt)
            amp = math.radians(rng.uniform(8, 28))
            seg = np.sin(np.linspace(0, math.pi, w)) * amp
            n = min(w, len(roll) - c)
            roll[c:c + n] += signo * seg[:n]
        info = {}
    # "llano": solo ruido
    tq, acc, gyro = _imu_desde_angulos(t, roll, pitch, a_lat, a_lon, ruido, rng)
    return {"t": tq, "acc": acc, "gyro": gyro, "t_tip": t_tip, "tipo": tipo, "info": info}


POSITIVOS = ("obstaculo", "giro", "morro")
NEGATIVOS = ("casi", "talud", "bache", "llano")


def lote(n: int, seed: int, ruido) -> list[dict]:
    rng = np.random.default_rng(seed)
    tipos = ["obstaculo"] * 4 + ["giro"] * 2 + ["morro"] * 2 + ["casi"] * 3 + ["talud"] * 3 + ["bache"] * 3 + ["llano"]
    out = []
    while len(out) < n:
        e = episodio(tipos[rng.integers(len(tipos))], rng, ruido)
        if e["tipo"] == "obstaculo" and e["t_tip"] is None:
            e["tipo"] = "casi"                   # el bulto no lo volcó: es un negativo duro
        out.append(e)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--extraer", action="store_true")
    a = ap.parse_args()
    if a.extraer:
        extraer()
