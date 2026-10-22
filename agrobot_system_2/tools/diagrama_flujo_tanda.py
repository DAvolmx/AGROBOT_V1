#!/usr/bin/env python3
"""Diagrama de FLUJO de la tanda de entrenamiento (2026-10-22): fases, estado, duraciones y métricas clave, leído del registro del orquestador.

  python3 tools/diagrama_flujo_tanda.py [--salida CARPETA]    -> flujo_tanda.png (y lo imprime como texto)
Lo regenera el loop de revisión cada 30 min.
"""
import argparse
import json
import re
import time
from datetime import datetime
from pathlib import Path

LOG = Path.home() / ".agrobot" / "aprendizaje" / "tanda_10h.log"
INICIO = Path.home() / ".agrobot" / "aprendizaje" / "tanda.inicio"
APR = Path.home() / ".agrobot" / "aprendizaje"
FASES = [("0", "Calibración con datos reales"), ("1", "Manejo robusto (continuo + pulsos)"), ("2", "Fallas con manejo asistido"),
         ("3", "Pruebas de geometría + informe"), ("4", "Dataset de SLAM calibrado + Kaggle"), ("5", "Informe final")]


def leer():
    est = {k: {"estado": "pendiente", "ini": None, "fin": None, "nota": ""} for k, _ in FASES}
    if not LOG.exists():
        return est, None
    t_ini = None
    for l in LOG.read_text(errors="ignore").splitlines():
        m = re.match(r"\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)\] (.*)", l)
        if not m:
            continue
        t = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S")
        txt = m.group(2)
        if "TANDA 10 H" in txt and t_ini is None:
            t_ini = t
        f = re.match(r"=== FASE (\d)", txt)
        if f:
            k = f.group(1)
            for kk in est:
                if est[kk]["estado"] == "en curso" and kk != k:
                    est[kk].update(estado="hecha", fin=t)
            est[k].update(estado="en curso", ini=t)
        if "TANDA 10 H TERMINADA" in txt:
            for kk in est:
                if est[kk]["estado"] == "en curso":
                    est[kk].update(estado="hecha", fin=t)
        if "sin tiempo" in txt or "se salta" in txt:
            for kk in est:
                if est[kk]["estado"] == "en curso":
                    est[kk]["nota"] = "parcial: " + txt[:60]
    return est, t_ini


def metricas():
    out = {}
    try:
        fs = sorted(APR.glob("modelo_manejo_optimo_*.json"), key=lambda p: p.stat().st_mtime)
        if fs and INICIO.exists() and fs[-1].stat().st_mtime > INICIO.stat().st_mtime:
            d = json.loads(fs[-1].read_text())
            p = d["prueba"]
            out["1"] = f"óptimo {d['control_optimo']}: éxito {p['exito_pct']} %, peor pasillo {p['peor_hueco_exito_pct']} %, {p['contactos_total']} contactos"
        fa = sorted(APR.glob("fallas_*.json"), key=lambda p: p.stat().st_mtime)
        if fa and INICIO.exists() and fa[-1].stat().st_mtime > INICIO.stat().st_mtime:
            h = json.loads(fa[-1].read_text())["historial"]
            out["2"] = f"rondas {len(h)}: entrenamiento {h[-1]['train_pct']} %, validación {h[-1]['val_pct']} %"
    except Exception:
        pass
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--salida", default=str(Path.home() / ".agrobot" / "informes"))
    a = ap.parse_args()
    est, t_ini = leer()
    met = metricas()
    ahora = datetime.utcnow()
    print(f"hora {ahora:%H:%M:%S} UTC · inicio {t_ini}")
    for k, nom in FASES:
        e = est[k]
        dur = ""
        if e["ini"]:
            dur = f"{((e['fin'] or ahora) - e['ini']).total_seconds() / 60:.0f} min"
        print(f"  [{e['estado']:9s}] {k} {nom:42s} {dur:8s} {met.get(k, '')} {e['nota']}")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyBboxPatch
    fig, ax = plt.subplots(figsize=(15, 3.8))
    ax.set_xlim(0, len(FASES) * 2.6); ax.set_ylim(0, 4); ax.axis("off")
    col = {"hecha": "#2a9d8f", "en curso": "#e9c46a", "pendiente": "#d9d9d9"}
    for i, (k, nom) in enumerate(FASES):
        e = est[k]
        x = i * 2.6
        ax.add_patch(FancyBboxPatch((x + .1, 1.0), 2.2, 2.2, boxstyle="round,pad=0.05", fc=col[e["estado"]], ec="#333"))
        ax.text(x + 1.2, 2.85, f"FASE {k}", ha="center", fontsize=10, weight="bold")
        ax.text(x + 1.2, 2.3, nom, ha="center", va="center", fontsize=8.5, wrap=True)
        dur = f"{((e['fin'] or ahora) - e['ini']).total_seconds() / 60:.0f} min" if e["ini"] else ""
        ax.text(x + 1.2, 1.55, f"{e['estado']} {dur}", ha="center", fontsize=8.5)
        if k in met:
            ax.text(x + 1.2, 0.55, met[k], ha="center", va="top", fontsize=7.2, wrap=True)
        if i < len(FASES) - 1:
            ax.annotate("", xy=(x + 2.65, 2.1), xytext=(x + 2.35, 2.1), arrowprops=dict(arrowstyle="->", lw=1.6))
    ax.set_title(f"Flujo de la tanda de entrenamiento · {ahora:%Y-%m-%d %H:%M} UTC" + (f" · lleva {(ahora - t_ini).total_seconds() / 3600:.1f} h de 10" if t_ini else ""))
    out = Path(a.salida) / "flujo_tanda.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(); fig.savefig(out, dpi=105); plt.close(fig)
    print("diagrama:", out)


if __name__ == "__main__":
    main()
