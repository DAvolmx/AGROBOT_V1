#!/usr/bin/env python3
"""Gráficas y métricas de las sesiones de navegación (~/.agrobot/navigation_logs).

    python3 tools/graficas_navegacion.py [carpeta_salida]

Por defecto escribe en agrobot_system_2/docs/graficas/: 7 PNG y metricas.json.
"Deriva" es la distancia entre la pose final y la inicial: en las pruebas de
calibración el robot vuelve al punto de partida, así que idealmente es 0.
"""
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

LOG_DIR = Path.home() / ".agrobot" / "navigation_logs"
SALIDA = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[3] / "docs" / "graficas"
MIN_POSES = 10
LIDAR_VALIDO_CM = 500


def _num(fila, clave):
    try:
        v = float(fila.get(clave) or 0)
    except ValueError:
        return None
    return v if 0 < v < LIDAR_VALIDO_CM else None


def cargar(ruta: Path) -> dict | None:
    filas = []
    with ruta.open() as f:
        for r in csv.DictReader(f):
            try:
                filas.append({"t": float(r["timestamp"]), "x": float(r["x"]), "y": float(r["y"]),
                              "theta": float(r["theta_deg"]),
                              "f": _num(r, "lidar_frontal_cm"), "i": _num(r, "lidar_lateral_izq_cm"),
                              "d": _num(r, "lidar_lateral_der_cm")})
            except (ValueError, KeyError):
                continue
    if len(filas) < MIN_POSES:
        return None
    t0 = filas[0]["t"]
    for r in filas:
        r["tr"] = r["t"] - t0
    dist = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(filas, filas[1:]))
    deriva = math.hypot(filas[-1]["x"] - filas[0]["x"], filas[-1]["y"] - filas[0]["y"])
    media = lambda k: (round(float(np.mean(v)), 1) if (v := [r[k] for r in filas if r[k]]) else None)
    return {"nombre": ruta.stem.replace("poses_", ""), "filas": filas, "poses": len(filas),
            "duracion_s": round(filas[-1]["tr"], 1), "distancia_m": round(dist, 3),
            "deriva_m": round(deriva, 3),
            "deriva_pct": round(100 * deriva / dist, 1) if dist > 0.05 else None,
            "lidar_frontal_cm": media("f"), "lidar_izq_cm": media("i"), "lidar_der_cm": media("d"),
            "lidar": {k: [r[k] for r in filas if r[k]] for k in "fid"}}


def main():
    SALIDA.mkdir(parents=True, exist_ok=True)
    sesiones = [s for p in sorted(LOG_DIR.glob("poses_202*.csv")) if (s := cargar(p))]
    viejo = LOG_DIR / "poses_19700101_001231.csv"
    s1970 = cargar(viejo) if viejo.exists() else None
    if not sesiones:
        sys.exit(f"sin sesiones en {LOG_DIR}")
    etiquetas = [f"#{i+1}" for i in range(len(sesiones))]

    # 1. Trayectorias
    cols = min(len(sesiones), 3)
    filas_g = math.ceil(len(sesiones) / cols)
    fig, ejes = plt.subplots(filas_g, cols, figsize=(5 * cols, 4.6 * filas_g), squeeze=False)
    for ax in ejes.flat[len(sesiones):]:
        ax.axis("off")
    for ax, s, et in zip(ejes.flat, sesiones, etiquetas):
        xs, ys = [r["x"] for r in s["filas"]], [r["y"] for r in s["filas"]]
        ax.plot(xs, ys, "b-", lw=0.8, alpha=0.7)
        ax.plot(xs[0], ys[0], "go", ms=8, label="Inicio")
        ax.plot(xs[-1], ys[-1], "rs", ms=8, label="Fin")
        ax.set_title(f"{et} ({s['poses']} poses, {s['duracion_s']:.0f} s)\n"
                     f"recorrido {s['distancia_m']} m · deriva {s['deriva_m']} m", fontsize=9)
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_aspect("equal")
        ax.grid(alpha=0.3); ax.legend(fontsize=7)
    fig.suptitle("Trayectorias XY por sesión", fontweight="bold"); fig.tight_layout()
    fig.savefig(SALIDA / "01_trayectorias.png", dpi=150); plt.close(fig)

    # 2. Distancia al origen vs tiempo
    fig, ax = plt.subplots(figsize=(10, 5))
    for s, et in zip(sesiones, etiquetas):
        x0, y0 = s["filas"][0]["x"], s["filas"][0]["y"]
        ax.plot([r["tr"] for r in s["filas"]], [math.hypot(r["x"] - x0, r["y"] - y0) for r in s["filas"]],
                lw=1.2, label=f"{et} ({s['poses']} poses)")
    ax.set_xlabel("Tiempo (s)"); ax.set_ylabel("Distancia al punto de inicio (m)")
    ax.set_title("Desplazamiento respecto del inicio"); ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(SALIDA / "02_desplazamiento.png", dpi=150); plt.close(fig)

    # 3. Distribución LIDAR
    fig, ejes = plt.subplots(1, 3, figsize=(14, 4))
    for ax, (k, titulo, color) in zip(ejes, (("f", "Frontal", "#e74c3c"), ("i", "Izquierda", "#3498db"),
                                              ("d", "Derecha", "#2ecc71"))):
        datos = [(s["lidar"][k], et) for s, et in zip(sesiones, etiquetas) if s["lidar"][k]]
        if datos:
            bp = ax.boxplot([d for d, _ in datos], tick_labels=[e for _, e in datos], patch_artist=True)
            for caja in bp["boxes"]:
                caja.set_facecolor(color); caja.set_alpha(0.6)
        ax.set_title(f"LIDAR {titulo} (cm)"); ax.grid(alpha=0.3)
    fig.suptitle("Distancias LIDAR por sesión", fontweight="bold"); fig.tight_layout()
    fig.savefig(SALIDA / "03_lidar_distribucion.png", dpi=150); plt.close(fig)

    # 4. Rumbo
    fig, ax = plt.subplots(figsize=(10, 5))
    for s, et in zip(sesiones, etiquetas):
        ax.plot([r["tr"] for r in s["filas"]], [r["theta"] for r in s["filas"]], lw=0.8, label=et)
    ax.set_xlabel("Tiempo (s)"); ax.set_ylabel("Rumbo (°)"); ax.set_title("Orientación vs tiempo")
    ax.legend(); ax.grid(alpha=0.3); fig.tight_layout()
    fig.savefig(SALIDA / "04_rumbo.png", dpi=150); plt.close(fig)

    # 5. LIDAR frontal vs tiempo (sesiones largas)
    largas = [(s, et) for s, et in zip(sesiones, etiquetas) if s["poses"] > 100]
    if largas:
        fig, ax = plt.subplots(figsize=(10, 5))
        for s, et in largas:
            ax.plot([r["tr"] for r in s["filas"] if r["f"]], [r["f"] for r in s["filas"] if r["f"]],
                    lw=0.8, label=et)
        ax.axhline(15, color="red", ls="--", lw=1, label="Zona crítica (15 cm)")
        ax.set_xlabel("Tiempo (s)"); ax.set_ylabel("Distancia frontal (cm)")
        ax.set_title("LIDAR frontal — sesiones largas"); ax.legend(); ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(SALIDA / "05_lidar_frontal.png", dpi=150); plt.close(fig)

    # 6. Resumen
    fig, ejes = plt.subplots(2, 2, figsize=(10, 8))
    for ax, clave, titulo, color in ((ejes[0, 0], "distancia_m", "Distancia recorrida (m)", "#3498db"),
                                     (ejes[0, 1], "deriva_m", "Deriva final (m)", "#e74c3c"),
                                     (ejes[1, 0], "duracion_s", "Duración (s)", "#2ecc71"),
                                     (ejes[1, 1], "lidar_frontal_cm", "LIDAR frontal medio (cm)", "#f39c12")):
        ax.bar(etiquetas, [s[clave] or 0 for s in sesiones], color=color); ax.set_title(titulo)
    fig.suptitle("Resumen por sesión", fontweight="bold"); fig.tight_layout()
    fig.savefig(SALIDA / "06_resumen.png", dpi=150); plt.close(fig)

    # 7. Antes / después del arreglo de la IMU
    if s1970:
        mejor = max(sesiones, key=lambda s: s["poses"])
        fig, (a, b) = plt.subplots(1, 2, figsize=(12, 5))
        for ax, s, titulo, col in ((a, s1970, "Aceleración IMU fusionada (bug)", "r-"),
                                   (b, mejor, "EKF corregido", "b-")):
            ax.plot([r["x"] for r in s["filas"]], [r["y"] for r in s["filas"]], col, lw=0.6)
            ax.plot(s["filas"][0]["x"], s["filas"][0]["y"], "go", ms=8)
            ax.set_title(f"{titulo}\nderiva {s['deriva_m']} m en {s['duracion_s']:.0f} s", fontsize=10)
            ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
        fig.suptitle("Antes vs después: corrección del EKF", fontweight="bold"); fig.tight_layout()
        fig.savefig(SALIDA / "07_antes_despues.png", dpi=150); plt.close(fig)

    quitar = ("filas", "lidar")
    metricas = {"sesiones": [{k: v for k, v in s.items() if k not in quitar} for s in sesiones],
                "bug_imu": {k: v for k, v in s1970.items() if k not in quitar} if s1970 else None}
    (SALIDA / "metricas.json").write_text(json.dumps(metricas, indent=2, ensure_ascii=False))
    for s, et in zip(sesiones, etiquetas):
        print(f"{et} {s['nombre']}: {s['poses']} poses, {s['duracion_s']} s, "
              f"{s['distancia_m']} m, deriva {s['deriva_m']} m ({s['deriva_pct']} %)")
    print(f"-> {SALIDA}")


if __name__ == "__main__":
    main()
