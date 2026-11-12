"""Prepara el dataset privado de Kaggle para el modelo de CONTEXTO del robot.

Las cajas a mano entran por NOMBRE: cualquier etiqueta llamada igual que una clase de contexto se usa, incluso de datasets subidos.
Del lote aleatorio, tramos de 5 fotos seguidas: 60 % entrena y 40 % evalúa. Las demás fotos solo entrenan.
No se generan cajas automáticas en fotos del robot: los detectores actuales encuentran pocas plantas y enseñarían que el resto es fondo.
Uso: python3 preparar_contexto.py --salida carpeta_nueva/agrobot-contexto-robot"""
import argparse, hashlib, json, shutil
from collections import Counter
from pathlib import Path

CLASES = ["planta", "col", "cogollo", "maceta", "obstaculo", "borde_bajo"]   # solo agronomía y lo que ayuda a manejar (2026-11-10): sin persona, animal ni vehículo
RAIZ = Path(__file__).resolve().parents[3]
HOME = Path.home() / ".agrobot"
MIN_EVAL = 15


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--salida", required=True); ap.add_argument("--min-eval", type=int, default=MIN_EVAL); ap.add_argument("--etiquetas", default=str(HOME / "etiquetas/plagas"))
    ap.add_argument("--macetas", default=str(HOME / "dataset_macetas/publicar"))
    ap.add_argument("--col", default=str(RAIZ / "simulador_pasivo/percepcion_sintetica/yolo_col"))
    ap.add_argument("--modelos", default=str(RAIZ / "jetson/backend/models"))
    ap.add_argument("--claude", default=str(HOME / "etiquetas/contexto_claude"))
    a = ap.parse_args()
    e = Path(a.etiquetas)
    nombres = json.loads((e / "clases.json").read_text(encoding="utf-8"))
    mapa = {i: CLASES.index(n) for i, n in enumerate(nombres) if n in CLASES}
    estado = json.loads((e / "estado.json").read_text(encoding="utf-8"))
    filas = {}
    for i, d in estado.items():
        lab = e / "labels" / f"{i}.txt"
        if not d.get("revisada") or not lab.exists() or not (e / "images" / f"{i}.jpg").exists():
            continue
        f = [f"{mapa[int(p[0])]} {' '.join(p[1:])}" for p in (l.split() for l in lab.read_text().splitlines()) if len(p) == 5 and int(p[0]) in mapa]
        if f:
            filas[i] = f
    lote = sorted((i for i in filas if "/lote_p6_" in estado[i]["origen"]), key=lambda i: Path(estado[i]["origen"]).name)
    destino = {i: "mano_train" for i in filas}
    for pos, i in enumerate(lote):
        bloque = f"{Path(estado[i]['origen']).name.split('__')[0]}:{pos // 5}"
        if int(hashlib.sha1(bloque.encode()).hexdigest(), 16) % 10 >= 6:
            destino[i] = "mano_eval"
    cuenta = Counter(destino.values())
    cajas = Counter(CLASES[int(l.split()[0])] for f in filas.values() for l in f)
    print("fotos:", dict(cuenta), "· cajas a mano:", dict(cajas))
    if cuenta["mano_eval"] < a.min_eval:
        raise SystemExit(f"Solo hay {cuenta['mano_eval']} fotos de evaluación y hacen falta {a.min_eval}.")
    sal = Path(a.salida)
    sal.mkdir(parents=True, exist_ok=False)
    for sp in ("mano_train", "mano_eval"):
        (sal / sp / "images").mkdir(parents=True); (sal / sp / "labels").mkdir()
    for i, sp in destino.items():
        shutil.copy2(e / "images" / f"{i}.jpg", sal / sp / "images" / f"{i}.jpg")
        (sal / sp / "labels" / f"{i}.txt").write_text("\n".join(filas[i]) + "\n")
    # Fotos del robot etiquetadas por Claude (revisión visual de propuestas): solo entrenamiento. Se eligieron a más de 60 s de cualquier foto de evaluación.
    cl = Path(a.claude)
    n_cl = 0
    if (cl / "labels").is_dir():
        (sal / "claude_train" / "images").mkdir(parents=True); (sal / "claude_train" / "labels").mkdir()
        for lab in sorted((cl / "labels").glob("*.txt")):
            shutil.copy2(cl / "images" / f"{lab.stem}.jpg", sal / "claude_train" / "images" / f"{lab.stem}.jpg")
            shutil.copy2(lab, sal / "claude_train" / "labels" / lab.name); n_cl += 1
        shutil.copy2(cl / "LEEME.json", sal / "claude_train" / "LEEME.json")
    print("fotos etiquetadas por Claude (solo entrenan):", n_cl)
    shutil.copytree(Path(a.macetas) / "images", sal / "fuente_macetas" / "images")
    shutil.copytree(Path(a.macetas) / "labels", sal / "fuente_macetas" / "labels")
    shutil.copytree(Path(a.col) / "images", sal / "fuente_col" / "images")
    shutil.copytree(Path(a.col) / "labels", sal / "fuente_col" / "labels")
    (sal / "pesos").mkdir()
    for n in ("agrobot_maceta_det.pt", "agrobot_col_yolo11n.pt"):
        shutil.copy2(Path(a.modelos) / n, sal / "pesos" / n)
    (sal / "pesos" / "CLASES.json").write_text(json.dumps(CLASES))
    print("dataset listo en", sal)


if __name__ == "__main__":
    main()
