"""Capturas de episodios simulados: trayectoria del robot sobre cada escenario (paredes, plantas/macetas, meta) para varias configuraciones.
Uso: python3 graficar_episodios.py [--informe <intensivo_.../informe.json>] [--semilla 5]
Salida: ~/.agrobot/informes/episodios/<entorno>_<escenario>.png (una figura por escenario, una columna por configuración)."""
import argparse, json, sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

import entorno_calibrado as EC
import entrenar_intensivo as T
import sim_logica_actual as S

DEST = Path.home() / ".agrobot" / "informes" / "episodios"


def trayectoria(entorno, nombre, params, semilla):
    S.SUBSTEP = 0.005
    esc = EC.escenarios(entorno, semilla)[nombre]
    r = S.episodio(esc, S.Config(**T.campos(params, entorno)), semilla, EC.hip(semilla), traza=True)
    return esc, r


def dibujar(ax, esc, r, titulo):
    for (x0, y0, x1, y1) in esc.paredes:
        ax.plot([x0, x1], [y0, y1], "k-", lw=2)
    for (x, y, rad) in esc.macetas:
        ax.add_patch(Circle((x, y), rad, color="#4c9a5a", alpha=0.8))
    tr = r.get("traza") or []
    if tr:
        xs, ys = [p[1] for p in tr], [p[2] for p in tr]
        ax.plot(xs, ys, "-", color="#1d4ed8", lw=1.3)
        ch = [(p[1], p[2]) for p in tr if p[5]]
        if ch:
            ax.scatter([c[0] for c in ch], [c[1] for c in ch], color="red", s=14, zorder=5, label="contacto")
        ax.scatter([xs[0]], [ys[0]], color="green", s=40, zorder=6, label="inicio")
    meta = esc.meta_xy or (esc.goal_x, esc.centro_y)
    ax.scatter([meta[0]], [meta[1]], marker="*", color="gold", edgecolor="k", s=140, zorder=7, label="meta")
    ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_title(f"{titulo}\néxito={r['exito']} contactos={r['contactos']} progreso={r['progreso']:.2f} t={r['t_s']:.0f}s", fontsize=9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--informe", default=None)
    ap.add_argument("--semilla", type=int, default=5)
    a = ap.parse_args()
    configs = {"AYER": T.AYER, "HOY": T.HOY}
    if a.informe:
        inf = json.loads(Path(a.informe).read_text())
        configs[f"mejor ({inf['mejor']['nombre']})"] = inf["mejor"]["params"]
    DEST.mkdir(parents=True, exist_ok=True)
    for entorno in ("interior", "exterior"):
        for nombre in EC.escenarios(entorno, a.semilla):
            fig, axs = plt.subplots(1, len(configs), figsize=(4.6 * len(configs), 3.8), squeeze=False)
            for ax, (cn, p) in zip(axs[0], configs.items()):
                esc, r = trayectoria(entorno, nombre, p, a.semilla)
                dibujar(ax, esc, r, cn)
            axs[0][0].legend(loc="upper left", fontsize=7)
            fig.suptitle(f"{entorno} / {nombre} (semilla {a.semilla})")
            fig.tight_layout()
            fig.savefig(DEST / f"{entorno}_{nombre}.png", dpi=105)
            plt.close(fig)
            print("ok", entorno, nombre, flush=True)


if __name__ == "__main__":
    main()
