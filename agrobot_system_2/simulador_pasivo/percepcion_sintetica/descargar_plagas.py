"""Descarga datasets REALES y abiertos de Hugging Face para plagas/enfermedades de col, acelga y mosca blanca.

Uso: python3 descargar_plagas.py [--cabbage_n 1200] [--shards 2]

Destino: agrobot_system_2/datos_plagas_reales/internet/<nombre>/   (+ FUENTES_Y_LICENCIAS.md)
Solo se baja un SUBCONJUNTO de los datasets grandes (disco y tiempo). Nada se entrena aquí.
"""
import argparse
import json
import sys
import urllib.request
from pathlib import Path

from huggingface_hub import hf_hub_download

DEST = Path(__file__).resolve().parents[2] / "datos_plagas_reales" / "internet"


def lista(repo):
    u = f"https://huggingface.co/api/datasets/{repo}"
    return [s["rfilename"] for s in json.load(urllib.request.urlopen(urllib.request.Request(u, headers={"User-Agent": "x"}), timeout=60))["siblings"]]


def bajar(repo, fich, carpeta):
    return hf_hub_download(repo, fich, repo_type="dataset", local_dir=str(carpeta))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cabbage_n", type=int, default=1200)
    ap.add_argument("--shards", type=int, default=2)
    a = ap.parse_args()
    DEST.mkdir(parents=True, exist_ok=True)
    # 1) minador (pequeño, Apache-2.0)
    r = "someoneskilled/leafminer"
    for f in lista(r):
        if f.lower().endswith((".jpg", ".jpeg", ".png")):
            bajar(r, f, DEST / "minador")
    print("minador ok", flush=True)
    # 2) enfermedades de col (zip, CC-BY-4.0)
    r = "Arko007/cabbage-disease-detection-v2"
    for f in lista(r):
        if f.endswith(".zip"):
            bajar(r, f, DEST / "col_enfermedades")
    print("col_enfermedades ok", flush=True)
    # 3) daño en col blanca: imágenes + cajas YOLO (CC-BY-4.0), subconjunto
    r = "Arko007/white-cabbage-leaf-damage"
    fs = lista(r)
    imgs = sorted(f for f in fs if f.startswith("data/Raw images") and f.lower().endswith(".jpg"))[:a.cabbage_n]
    labs = {Path(f).stem: f for f in fs if f.startswith("data/Annotated labels") and f.endswith(".txt")}
    bajar(r, "data/Annotated labels (YOLO format, TXT files)/classes.txt", DEST / "col_dano")
    for k, f in enumerate(imgs):
        bajar(r, f, DEST / "col_dano")
        if Path(f).stem in labs:
            bajar(r, labs[Path(f).stem], DEST / "col_dano")
        if (k + 1) % 200 == 0:
            print(f"  col_dano {k+1}/{len(imgs)}", flush=True)
    print("col_dano ok", flush=True)
    # 4) mosca blanca en yuca (parquet, 3000 imágenes con ~213 000 cajas), solo `shards` ficheros
    r = "Project-AgML/Cassava_whitefly_detection"
    pq = sorted(f for f in lista(r) if f.endswith(".parquet"))[:a.shards]
    for f in pq:
        bajar(r, f, DEST / "cassava_whitefly")
        print("  ", f, flush=True)
    print("cassava_whitefly ok", flush=True)
    (DEST / "FUENTES_Y_LICENCIAS.md").write_text(
        "# Fuentes y licencias (descargado 2026-10-15)\n\n"
        "| carpeta | origen | licencia | qué contiene |\n|---|---|---|---|\n"
        "| `cassava_whitefly/` | huggingface.co/datasets/Project-AgML/Cassava_whitefly_detection (Nakatumba-Nabende et al., 2022, Data in Brief) | **la ficha de HF no indica licencia**; el artículo original es de acceso abierto: verificar antes de redistribuir y CITAR | fotos de móvil de yuca en campo (Uganda), mosca blanca con cajas (x, y, ancho, alto) |\n"
        "| `col_dano/` | huggingface.co/datasets/Arko007/white-cabbage-leaf-damage | CC-BY-4.0 (citar) | col blanca con cajas YOLO: cabbage, capitata, damaged |\n"
        "| `col_enfermedades/` | huggingface.co/datasets/Arko007/cabbage-disease-detection-v2 | CC-BY-4.0 (citar) | zip: alternaria, mildiu, podredumbre negra, sana |\n"
        "| `minador/` | huggingface.co/datasets/someoneskilled/leafminer | Apache-2.0 | clasificación sana / minador |\n\n"
        "No se descargó `deepplants/cabbage` (CC-BY-NC-ND: no permite derivados ni uso comercial).\n"
        "**Son fotos de OTRAS plantas y cámaras**: sirven para preentrenar y para medir, no sustituyen fotos de tu planta.\n")
    print("listo", DEST)


if __name__ == "__main__":
    main()
