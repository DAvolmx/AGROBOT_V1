"""Gráficas del entrenamiento de 7 h del explorador (entrenar_explorador_7h.py): costo por generación (HOY / mejor / media) y validación final
por escenario (recorrido y plantas pisadas). Uso: python3 graficar_explorador_7h.py [carpeta]  → <carpeta>/evolucion.png, validacion.png"""
import glob, json, sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

d = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(sorted(glob.glob(str(Path.home() / ".agrobot/aprendizaje/explorador_7h_*")))[-1])
h = [g for g in json.loads((d / "historial.json").read_text()) if not g.get("fallida")]
plt.rcParams.update({"font.size": 13})

fig, ax = plt.subplots(figsize=(11, 5.5))
g = [x["gen"] for x in h]
ax.plot(g, [x["hoy"] for x in h], "o-", color="#d9822b", label="Configuración actual (HOY)")
ax.plot(g, [x["mejor"] for x in h], "o-", color="#2e9e6b", label="Mejor candidato de la generación")
ax.plot(g, [x["media"] for x in h], "--", color="#7a8a99", label="Media de la población")
ax.set_xlabel("Generación (semillas nuevas en cada una)"); ax.set_ylabel("Costo (menor es mejor)")
ax.set_title("Entrenamiento CEM del explorador: 6 escenarios × 180 s simulados")
ax.grid(alpha=0.3); ax.legend()
fig.tight_layout(); fig.savefig(d / "evolucion.png", dpi=130)

inf = d / "informe.json"
if inf.exists():
    v = json.loads(inf.read_text())["validacion"]
    esc = list(v[0]["por_esc"])
    fig, axs = plt.subplots(1, 2, figsize=(14, 5.5))
    ancho = 0.8 / len(v)
    for i, c in enumerate(v):
        xs = [k + i * ancho for k in range(len(esc))]
        axs[0].bar(xs, [c["por_esc"][e]["recorrido_m"] for e in esc], ancho, label=c["nombre"])
        axs[1].bar(xs, [c["por_esc"][e]["plantas_pisadas"] for e in esc], ancho, label=c["nombre"])
    for a, t in zip(axs, ("Recorrido medio (m)", "Plantas pisadas por episodio")):
        a.set_xticks([k + 0.4 - ancho / 2 for k in range(len(esc))]); a.set_xticklabels([e.replace("_", "\n") for e in esc], fontsize=10)
        a.set_title(t); a.grid(axis="y", alpha=0.3)
    axs[0].legend()
    fig.suptitle("Validación final en semillas nunca vistas (episodios de 240 s)")
    fig.tight_layout(); fig.savefig(d / "validacion.png", dpi=130)
print(d)
