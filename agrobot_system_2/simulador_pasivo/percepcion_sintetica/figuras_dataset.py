"""Figuras que no necesitan el modelo: muestras del dataset y zona ciega de la cámara."""
import glob
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import corredor as C
import render as R

AQUI = Path(__file__).resolve().parent
SAL = AQUI.parents[1] / "docs" / "capturas_percepcion_20261015"
SAL.mkdir(parents=True, exist_ok=True)


def muestras():
    fs = sorted(glob.glob(str(AQUI / "datos" / "train" / "*.npz")))
    rng = np.random.default_rng(4)
    elegidas = []
    for f in rng.choice(fs, 40, replace=False):
        d = np.load(f, allow_pickle=True)
        k = int(rng.integers(len(d["rgb"])))
        pres = np.bincount(d["clase"][k].ravel(), minlength=5)
        elegidas.append((pres[3] + pres[4] + pres[2], d["rgb"][k], d["clase"][k]))
    elegidas.sort(key=lambda t: -t[0])
    elegidas = elegidas[:8]
    fig, ax = plt.subplots(4, 4, figsize=(15, 11.5))
    for i, (_, rgb, cl) in enumerate(elegidas):
        a1, a2 = ax[(i // 4) * 2, i % 4], ax[(i // 4) * 2 + 1, i % 4]
        a1.imshow(rgb)
        a2.imshow(R.COLORES_CLASE[cl])
        a1.axis("off"); a2.axis("off")
    leyenda = [plt.Rectangle((0, 0), 1, 1, color=R.COLORES_CLASE[i] / 255) for i in range(5)]
    fig.legend(leyenda, R.CLASES, loc="lower center", ncol=5, fontsize=11)
    fig.suptitle("Dataset sintético de interiores: imagen (arriba) y verdad por píxel (abajo)", fontsize=13)
    fig.tight_layout(rect=(0, 0.04, 1, 0.97))
    fig.savefig(SAL / "01_dataset_muestras.png", dpi=110)
    plt.close(fig)


def zona_ciega():
    inc = np.linspace(0, 45, 91)
    ciega = [C.zona_ciega(i) for i in inc]
    fig, ax = plt.subplots(figsize=(8.5, 4.6))
    ax.plot(inc, np.array(ciega) * 100, lw=2.5, color="#c0392b")
    ax.fill_between(inc, 0, np.array(ciega) * 100, color="#c0392b", alpha=0.12)
    for i, rot in ((15.5, "HOME 98° (15,5°)"), (33.1, "servo 76° (33°)")):
        c = C.zona_ciega(i) * 100
        ax.plot([i], [c], "o", color="k")
        ax.annotate(f"{rot}\n{c:.0f} cm", (i, c), textcoords="offset points", xytext=(8, 10), fontsize=9)
    ax.axhline(52, ls="--", color="#2980b9")
    ax.text(44, 54, "margen de parada actual (52 cm)", ha="right", fontsize=8, color="#2980b9")
    ax.set_xlabel("inclinación de la cámara hacia abajo (°)")
    ax.set_ylabel("distancia a la que EMPIEZA a verse el suelo (cm, desde el centro)")
    ax.set_title("Zona ciega de la cámara: lo que está más cerca que esto NO se ve")
    ax.grid(alpha=0.3)
    ax.set_ylim(0, 130)
    fig.tight_layout()
    fig.savefig(SAL / "02_zona_ciega.png", dpi=125)
    plt.close(fig)


if __name__ == "__main__":
    muestras()
    zona_ciega()
    print("ok", sorted(p.name for p in SAL.iterdir()))
