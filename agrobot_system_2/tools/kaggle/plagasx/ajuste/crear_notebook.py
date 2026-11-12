"""Genera publicar/plagasx_ajuste.ipynb: reutiliza el armado de datos de PLAGASX v9 y reemplaza el entrenamiento por un reajuste de v9 con las fotos del operador."""
import json
from pathlib import Path

AQUI = Path(__file__).resolve().parent
PUB = AQUI / "publicar"
v9 = json.loads((AQUI.parent / "publicar" / "plagasx.ipynb").read_text(encoding="utf-8"))
src = "".join(v9["cells"][0]["source"])
meta = json.loads((AQUI.parent / "publicar" / "kernel-metadata.json").read_text(encoding="utf-8"))
corte = "# ---------------- 4) DETECTOR (YOLO) ----------------"
ancla = 'df = pd.DataFrame(registro, columns=["split", "origen", "clase", "archivo"])'
assert src.count(corte) == 1 and src.count(ancla) == 1
base = src.split(corte)[0]

OPERADOR = '''# ---------------- 5) FOTOS DEL ROBOT CON CAJAS A MANO: entrenan repetidas para que pesen frente a miles de imágenes externas ----------------
REPETIR = 6
raiz_op = next(p for p in sorted(ENTRADA.rglob("*")) if p.is_dir() and (p / "op_eval" / "images").is_dir() and (p / "pesos" / "v9_det.pt").exists())
for sp_op, sp, veces in (("op_train", "train", REPETIR), ("op_val", "val", 1)):
    for f in sorted((raiz_op / sp_op / "images").glob("*.jpg")):
        img = leer_redim(f)
        if img is None: continue
        cajas = [l.split() for l in (raiz_op / sp_op / "labels" / f"{f.stem}.txt").read_text().splitlines() if len(l.split()) == 5]
        for r in range(veces):
            guardar(img, sp, f"op{r}_{f.stem}", [(int(c[0]), c[1], c[2], c[3], c[4]) for c in cajas])
            registro.append((sp, "operador", -2, f"op{r}_{f.stem}"))
print("fotos del operador (con repetición):", sum(1 for r in registro if r[1] == "operador"))
assert not any(p.stem in {f.stem for f in (raiz_op / "op_train" / "images").glob("*.jpg")} for p in (raiz_op / "op_eval" / "images").glob("*.jpg"))

'''
FINAL = '''# ---------------- 6) REAJUSTE DE v9: congela la base de la red para conservar lo aprendido ----------------
from ultralytics import YOLO
CONF = 0.45
det = YOLO(str(raiz_op / "pesos" / "v9_det.pt"))
det.train(data=str(SALIDA / "dataset.yaml"), epochs=80, time=3.0, imgsz=960, batch=16, patience=25, close_mosaic=10, device=[0, 1], project=str(RES / "runs"), name="ajuste",
          freeze=10, optimizer="AdamW", lr0=0.0005, lrf=0.1, mosaic=0.5, mixup=0.0, hsv_h=.015, hsv_s=.5, hsv_v=.4, fliplr=.5, flipud=0.0, exist_ok=True, verbose=False)
shutil.copy(RES / "runs" / "ajuste" / "weights" / "best.pt", RES / "coles_det_ajuste.pt")

# ---------------- 7) COMPARACIÓN EN FOTOS DEL ROBOT QUE NINGÚN REAJUSTE VIO ----------------
fotos_eval = sorted((raiz_op / "op_eval" / "images").glob("*.jpg"))
def predecir(ruta):
    m = YOLO(str(ruta)); salida = []
    for f in fotos_eval:
        r = m.predict(str(f), imgsz=960, conf=CONF, verbose=False, device=0)[0]
        pred = [(r.names[int(c)], float(p), *map(float, b)) for c, p, b in zip(r.boxes.cls, r.boxes.conf, r.boxes.xywhn)]
        salida.append({"real": leer_etiqueta((raiz_op / "op_eval" / "labels" / f"{f.stem}.txt").read_text(), CLASES), "pred": pred})
    return salida
modelos = {"v8": raiz_op / "pesos" / "v8_det.pt", "v9": raiz_op / "pesos" / "v9_det.pt", "ajuste": RES / "coles_det_ajuste.pt"}
resultado = {n: evaluar(predecir(p)) for n, p in modelos.items()}
(RES / "comparacion_robot.json").write_text(json.dumps({"fotos_evaluacion": len(fotos_eval), "conf": CONF, "modelos": resultado}, ensure_ascii=False, indent=1))
display(pd.DataFrame({n: {k: f"{v['valor']} ({v['k']}/{v['n']}) IC95 {v['ic95']}" for k, v in r.items() if k != "por_clase"} for n, r in resultado.items()}))
print("HECHO. coles_det_ajuste.pt y comparacion_robot.json. Se conservan las fotos de evaluación fuera del entrenamiento; con pocas fotos, mirar los intervalos antes de elegir modelo.")
'''
base = base.replace(ancla, OPERADOR + ancla)
base = "# PLAGASX AJUSTE (2026-11-09): reajusta v9 con cajas a mano sobre fotos del robot y compara v8, v9 y el reajuste. Generado por crear_notebook.py.\n" + base
codigo = (AQUI / "metricas_ajuste.py").read_text(encoding="utf-8")
nb = {"cells": [{"cell_type": "code", "execution_count": None, "metadata": {}, "outputs": [], "source": (codigo + "\n\n" + base + FINAL).splitlines(keepends=True)}],
      "metadata": v9.get("metadata", {}), "nbformat": v9.get("nbformat", 4), "nbformat_minor": v9.get("nbformat_minor", 4)}
PUB.mkdir(exist_ok=True)
(PUB / "plagasx_ajuste.ipynb").write_text(json.dumps(nb, ensure_ascii=False, indent=1), encoding="utf-8")
meta.update(id="davidgomez32/plagasx-ajuste", title="PLAGASX ajuste", code_file="plagasx_ajuste.ipynb", is_private=True)
meta["dataset_sources"] = meta["dataset_sources"] + ["davidgomez32/agrobot-ajuste-operador"]
(PUB / "kernel-metadata.json").write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
print("notebook creado en", PUB)
