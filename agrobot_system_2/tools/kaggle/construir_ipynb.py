"""Convierte agrobot_slam_correccion.py (celdas `# %%` y `# %% [markdown]`) en agrobot_slam_correccion.ipynb (sin nbformat)."""
import json
from pathlib import Path

aqui = Path(__file__).parent
celdas, tipo, buf = [], None, []


def cerrar():
    if tipo is None or not "".join(buf).strip():
        return
    if tipo == "md":
        src = [l[2:] if l.startswith("# ") else l.lstrip("#") for l in buf]
        celdas.append({"cell_type": "markdown", "metadata": {}, "source": "".join(src).strip("\n").splitlines(True)})
    else:
        celdas.append({"cell_type": "code", "metadata": {}, "execution_count": None, "outputs": [],
                       "source": "".join(buf).strip("\n").splitlines(True)})


for linea in (aqui / "agrobot_slam_correccion.py").read_text().splitlines(True):
    if linea.startswith("# %%"):
        cerrar()
        tipo, buf = ("md" if "[markdown]" in linea else "code"), []
    else:
        buf.append(linea)
cerrar()
nb = {"cells": celdas, "metadata": {"kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
                                    "language_info": {"name": "python"}}, "nbformat": 4, "nbformat_minor": 5}
(aqui / "agrobot_slam_correccion.ipynb").write_text(json.dumps(nb, ensure_ascii=False, indent=1))
print("celdas:", len(celdas))
