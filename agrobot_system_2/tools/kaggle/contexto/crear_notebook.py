"""Genera publicar/ para lanzar el modelo de contexto en Kaggle."""
import json
from pathlib import Path
AQUI = Path(__file__).resolve().parent; PUB = AQUI / "publicar"; PUB.mkdir(exist_ok=True)
fuente = (AQUI / "contexto_kaggle.py").read_text(encoding="utf-8")
nb = {"cells": [{"cell_type": "code", "execution_count": None, "metadata": {}, "outputs": [], "source": fuente.splitlines(keepends=True)}],
      "metadata": {"kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"}}, "nbformat": 4, "nbformat_minor": 4}
(PUB / "agrobot_contexto.ipynb").write_text(json.dumps(nb, ensure_ascii=False, indent=1), encoding="utf-8")
(PUB / "kernel-metadata.json").write_text(json.dumps({"id": "davidgomez32/agrobot-contexto", "title": "agrobot-contexto", "code_file": "agrobot_contexto.ipynb", "language": "python",
    "kernel_type": "notebook", "is_private": True, "enable_gpu": True, "enable_tpu": False, "enable_internet": True,
    "dataset_sources": ["awsaf49/coco-2017-dataset", "davidgomez32/agrobot-yolo-sintetico", "davidgomez32/agrobot-contexto-robot", "cubeai/crop-weed-detection-for-yolov8"], "kernel_sources": [], "competition_sources": [], "model_sources": []}, indent=1), encoding="utf-8")
print("listo en", PUB)
