"""Capturas y gráficas del informe de interiores (matplotlib, sin pantalla).

Uso:  python3 capturas_interior.py [--json RUTA] [--salida DIR]
Lee el JSON de `entrenar_interior.py` y dibuja:
  01_rampa_firmware.png        duty alcanzado por pulso según rampa (DETERMINISTA, sin hipótesis de planta)
  02_<escenario>_antes_despues.png   trayectorias de la config actual y el candidato, misma semilla
  03_resumen_test.png          éxito / contactos / tiempo en TEST
  04_linea_de_tiempo.png       un episodio: pulsos, duty y órdenes descartadas
"""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                      # noqa: E402
import numpy as np                                   # noqa: E402
from matplotlib.patches import Circle, Polygon       # noqa: E402

import sim_logica_actual as S                        # noqa: E402

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parent
C_ACTUAL, C_PERFIL, C_CAND = "#c0392b", "#7f8c8d", "#1f7a4d"


def _rect(cx, cy, ang, largo, ancho):
    c, s = math.cos(ang), math.sin(ang)
    pts = [(-largo / 2, -ancho / 2), (largo / 2, -ancho / 2), (largo / 2, ancho / 2), (-largo / 2, ancho / 2)]
    return [(cx + x * c - y * s, cy + x * s + y * c) for x, y in pts]


def dibujar_robot(ax, cx, cy, psi, delta, color, choque=False, alpha=0.55):
    """Eje ancho FIJO delante (37 cm), módulo con pivote detrás (19 cm), cuerpo en medio."""
    col = "#e74c3c" if choque else color
    ax.add_patch(Polygon(_rect(cx, cy, psi, 0.35, 0.25), closed=True, fc=col, ec="k", lw=0.5, alpha=alpha * 0.6))
    fx, fy = cx + 0.185 * math.cos(psi), cy + 0.185 * math.sin(psi)
    rx, ry = cx - 0.185 * math.cos(psi), cy - 0.185 * math.sin(psi)
    ax.add_patch(Polygon(_rect(fx, fy, psi + math.pi / 2, 0.37, 0.05), closed=True, fc="k", alpha=alpha))
    ax.add_patch(Polygon(_rect(rx, ry, psi + delta + math.pi / 2, 0.19, 0.05), closed=True, fc="#2c3e50", alpha=alpha))
    ax.plot([cx, cx + 0.22 * math.cos(psi)], [cy, cy + 0.22 * math.sin(psi)], color=col, lw=1.2)


def dibujar_mundo(ax, esc):
    for (x0, y0, x1, y1) in esc.paredes:
        ax.plot([x0, x1], [y0, y1], color="#34495e", lw=2.2, solid_capstyle="round")
    for (x, y, r) in esc.macetas:
        ax.add_patch(Circle((x, y), r, fc="#b9835a", ec="#6e4b2a", lw=1.2))
        ax.add_patch(Circle((x, y), r * 0.55, fc="#4c8c4a", ec="none", alpha=0.9))
    if esc.linea_central:
        cx, cy = zip(*esc.linea_central)
        ax.plot(cx, cy, ls=":", color="#7f8c8d", lw=1)
    meta = esc.meta_xy or (esc.goal_x, esc.centro_y)
    ax.plot(*meta, marker="*", color="#f1c40f", mec="k", ms=15, zorder=5)
    ax.plot(esc.inicio[0], esc.inicio[1], "o", color="#2980b9", ms=7, zorder=5)


def panel(ax, esc, res, cfg, color, titulo):
    dibujar_mundo(ax, esc)
    tr = res["traza"]
    xs, ys = [p[1] for p in tr], [p[2] for p in tr]
    ax.plot(xs, ys, color=color, lw=1.6, zorder=4)
    paso = max(1, len(tr) // 14)
    for p in tr[::paso]:
        dibujar_robot(ax, p[1], p[2], p[3], p[4], color, bool(p[5]))
    xmin = min(min(xs) - 0.5, min(x for w in esc.paredes for x in (w[0], w[2])) - 0.1)
    xmax = max(max(xs) + 0.5, esc.goal_x + 0.4)
    ax.set_xlim(xmin, xmax)
    ys_all = [y for w in esc.paredes for y in (w[1], w[3])] + ys
    ax.set_ylim(min(ys_all) - 0.2, max(ys_all) + 0.2)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    estado = "LLEGÓ" if res["llego"] else ("ATASCADO" if res["atascado"] else "NO LLEGÓ")
    ax.set_title(f"{titulo}\n{estado} · {res['t_s']:.0f} s · contactos {res['contactos']} · "
                 f"pico {res['pico_cm']:.0f} cm", fontsize=10, color=color)
    ax.text(0.02, 0.02, f"mu {res['mu']:.2f} · zona muerta {res['dead']:.0f}% · pulsos {res['pulsos']}\n"
            f"correcciones {res['corr_ejecutadas']}/{res['correcciones']} ejecutadas",
            transform=ax.transAxes, fontsize=7.5, va="bottom",
            bbox=dict(fc="white", ec="none", alpha=0.8))


def fig_rampa(salida):
    """DETERMINISTA: lo que hace la rampa del firmware con cada duración de pulso."""
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    durs = np.linspace(0.1, 1.2, 100)
    for slew, col in ((60, "#c0392b"), (120, "#e67e22"), (200, "#2980b9"), (300, "#1f7a4d")):
        ax[0].plot(durs, np.minimum(60.0, slew * durs), color=col, lw=2, label=f"rampa {slew} %/s")
    for nombre, d in (("preciso\n0,40 s", 0.40), ("giro ext.\n0,35 s", 0.35), ("interior\n0,80 s", 0.80), ("exterior\n1,00 s", 1.0)):
        ax[0].axvline(d, color="k", lw=0.6, ls="--")
        ax[0].text(d - 0.012, 63.5, nombre.replace("\n", " "), fontsize=7.5, ha="right", va="top", rotation=90)
    ax[0].axhspan(25, 40, color="#f1c40f", alpha=0.25)
    ax[0].text(1.19, 32, "zona muerta posible\n(25-40 %, NO medida)", fontsize=8, ha="right", va="center")
    ax[0].set_xlabel("duración del pulso (s)")
    ax[0].set_ylabel("duty máximo alcanzado al terminar el pulso (%)")
    ax[0].set_title("La rampa del firmware decide cuánto empuja un pulso\n(cada pulso arranca de 0 %)", fontsize=11)
    ax[0].set_ylim(0, 65)
    ax[0].legend(fontsize=8, loc="lower right", framealpha=0.95)
    ax[0].grid(alpha=0.3)
    # tiempo hasta superar la zona muerta
    slews = np.array([30, 60, 90, 120, 150, 200, 250, 300])
    for dead, col in ((25, "#2980b9"), (40, "#c0392b")):
        ax[1].plot(slews, 1000 * dead / slews, "o-", color=col, label=f"zona muerta {dead} %")
    ax[1].set_xlabel("rampa del firmware (%/s)")
    ax[1].set_ylabel("tiempo de pulso SIN empuje útil (ms)")
    ax[1].set_title("Tiempo de pulso sin empuje útil\n(hasta superar la zona muerta)", fontsize=11)
    ax[1].axhline(350, color="k", ls="--", lw=0.8)
    ax[1].text(slews[-1], 360, "giro de 0,35 s", fontsize=8, ha="right")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(salida / "01_rampa_firmware.png", dpi=130)
    plt.close(fig)


def fig_resumen(datos, salida):
    test = datos["etapas"]["test"]
    nombres = [("actual", "Config actual", C_ACTUAL), ("perfil_interior", "Perfil interior", C_PERFIL),
               ("candidato", "Candidato", C_CAND)]
    escs = ["sala_vacia", "pasillo_macetas", "eslalon_macetas", "curva_pasillo"]
    fig, ax = plt.subplots(1, 3, figsize=(14, 4.2))
    w = 0.26
    for k, (clave, rotulo, col) in enumerate(nombres):
        if clave not in test:
            continue
        pe = test[clave]["por_escenario"]
        ax[0].bar(np.arange(len(escs)) + (k - 1) * w, [pe[e]["exito_pct"] for e in escs], w, color=col, label=rotulo)
        ax[1].bar(np.arange(len(escs)) + (k - 1) * w, [pe[e]["contactos"] for e in escs], w, color=col)
        ax[2].bar(np.arange(len(escs)) + (k - 1) * w, [pe[e]["t_s"] for e in escs], w, color=col)
    for a, t, yl in zip(ax, ("Éxito (llega sin tocar), %", "Contactos por recorrido", "Tiempo medio (s)"),
                        ("%", "contactos", "s")):
        a.set_xticks(range(len(escs)))
        a.set_xticklabels([e.replace("_", "\n") for e in escs], fontsize=8)
        a.set_title(t)
        a.grid(axis="y", alpha=0.3)
    ax[0].legend(fontsize=8)
    ax[0].set_ylim(0, 105)
    fig.suptitle(f"TEST ({len(datos['semillas']['test'])} semillas por escenario, hipótesis de motor y suelo aleatorizadas)", y=1.02)
    fig.tight_layout()
    fig.savefig(salida / "03_resumen_test.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def fig_linea_tiempo(esc, res, cfg, salida, nombre):
    tr = res["traza"]
    t = np.array([p[0] for p in tr])
    duty = np.array([p[6] for p in tr])
    act = np.array([p[7] for p in tr])
    x = np.array([p[1] for p in tr])
    y = np.array([esc.desvio(p[1], p[2]) for p in tr])
    v = np.gradient(x, t) if len(t) > 2 else np.zeros_like(t)
    fig, ax = plt.subplots(3, 1, figsize=(11, 6.5), sharex=True)
    ax[0].fill_between(t, 0, act, step="mid", color="#2980b9", alpha=0.35, label="pulso en curso")
    ax[0].plot(t, duty / 60.0, color="#c0392b", lw=1.2, label="duty de la rueda más cargada (÷60)")
    ax[0].set_yticks([])
    ax[0].legend(fontsize=8, loc="upper right")
    ax[0].set_title(f"{nombre}: cada pulso arranca de 0 y la rampa limita cuánto empuja")
    ax[1].plot(t, np.clip(v, 0, None) * 100, color="#1f7a4d")
    ax[1].set_ylabel("velocidad real (cm/s)")
    ax[1].grid(alpha=0.3)
    ax[2].plot(t, y * 100, color="#8e44ad")
    ax[2].set_ylabel("desvío del centro (cm)")
    ax[2].set_xlabel("tiempo (s)")
    ax[2].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(salida / "04_linea_de_tiempo.png", dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=None)
    ap.add_argument("--salida", default=str(RAIZ / "docs" / "capturas_interior_20261015"))
    ap.add_argument("--semilla", type=int, default=12)
    a = ap.parse_args()
    salida = Path(a.salida)
    salida.mkdir(parents=True, exist_ok=True)
    fj = Path(a.json) if a.json else sorted((Path.home() / ".agrobot" / "aprendizaje").glob("interior_tanda_*.json"))[-1]
    datos = json.loads(fj.read_text())

    fig_rampa(salida)
    fig_resumen(datos, salida)

    test = datos["etapas"]["test"]
    cfgs = {"actual": S.Config(**test["actual"]["cfg"]), "candidato": S.Config(**test["candidato"]["cfg"])}
    cfgs["actual"].nombre, cfgs["candidato"].nombre = "actual", "candidato"
    escs = S.escenarios_interior(0.60)
    for nombre in ("pasillo_macetas", "eslalon_macetas", "curva_pasillo"):
        fig, ax = plt.subplots(1, 2, figsize=(13, 5.2) if nombre != "curva_pasillo" else (13, 6.5))
        ra = S.episodio(escs[nombre], cfgs["actual"], a.semilla, traza=True)
        rc = S.episodio(escs[nombre], cfgs["candidato"], a.semilla, traza=True)
        panel(ax[0], escs[nombre], ra, cfgs["actual"], C_ACTUAL, "Config ACTUAL")
        panel(ax[1], escs[nombre], rc, cfgs["candidato"], C_CAND, "CANDIDATO")
        fig.suptitle(f"{nombre} — {escs[nombre].descripcion} (semilla {a.semilla}, misma física en ambos)", fontsize=11)
        fig.tight_layout()
        fig.savefig(salida / f"02_{nombre}_antes_despues.png", dpi=130)
        plt.close(fig)
        if nombre == "pasillo_macetas":
            fig_linea_tiempo(escs[nombre], ra, cfgs["actual"], salida, "Config actual, pasillo de macetas")
    print("capturas en", salida)
    for f in sorted(salida.iterdir()):
        print("  ", f.name, f.stat().st_size // 1024, "KB")


if __name__ == "__main__":
    main()
