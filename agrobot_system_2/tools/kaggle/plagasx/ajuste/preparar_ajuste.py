"""Prepara el dataset privado de Kaggle para reajustar PLAGASX con las cajas dibujadas a mano por el operador.

Solo usa fotos con cajas o marcadas «sin plaga»; una foto revisada sin nada de eso se omite porque no se sabe si estaba limpia.
El lote aleatorio se reparte por tramos de 5 fotos seguidas: 50 % entrenamiento, 10 % validación y 40 % evaluación; las fotos parecidas quedan juntas.
Las cajas «planta» no son una enfermedad y v9 no tiene esa clase: se quitan, y una foto con solo plantas cuenta como foto sin plaga.
Las demás fotos (elegidas a mano, no aleatorias) solo entrenan: no sirven para medir.
Uso: python3 preparar_ajuste.py --v9 ruta/coles_det_best.pt --salida carpeta [--etiquetas carpeta] [--v8 ruta]"""
import argparse, hashlib, json, shutil
from collections import Counter
from pathlib import Path

CLASES_MODELO = 12
IGNORAR = {"planta"}
MIN_EVAL = 40


def particion(bloque: str) -> str:
    h = int(hashlib.sha1(bloque.encode()).hexdigest(), 16) % 10
    return "op_train" if h < 5 else ("op_val" if h < 6 else "op_eval")


def planear(estado: dict, etiquetas: Path):
    util = {i: d for i, d in estado.items() if "/datos_usuario/subidas/" not in d["origen"] and d.get("revisada") and (d.get("n_cajas", 0) > 0 or d.get("sin_plaga")) and (etiquetas / "images" / f"{i}.jpg").exists()}
    lote = sorted(((i, d) for i, d in util.items() if "/lote_p6_" in d["origen"]), key=lambda x: Path(x[1]["origen"]).name)
    destino = {i: "op_train" for i, d in util.items() if "/lote_p6_" not in d["origen"]}
    for pos, (i, d) in enumerate(lote):
        nombre = Path(d["origen"]).name
        destino[i] = particion(f"{nombre.split('__')[0]}:{pos // 5}")
    return destino


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--v9"); ap.add_argument("--salida")
    ap.add_argument("--solo-resumen", action="store_true", help="muestra el reparto sin crear nada")
    ap.add_argument("--etiquetas", default=str(Path.home() / ".agrobot/etiquetas/plagas"))
    ap.add_argument("--v8", default=str(Path(__file__).resolve().parents[4] / "jetson/backend/models/agrobot_plagasx_det.pt"))
    a = ap.parse_args()
    e = Path(a.etiquetas)
    if not a.solo_resumen and not (a.v9 and a.salida):
        ap.error("hacen falta --v9 y --salida")
    nombres = json.loads((e / "clases.json").read_text(encoding="utf-8")) if (e / "clases.json").exists() else []
    ignoradas = {i for i, n in enumerate(nombres) if n in IGNORAR}
    destino = planear(json.loads((e / "estado.json").read_text()), e)
    filas = {}
    for i in destino:
        filas[i] = [l for l in (e / "labels" / f"{i}.txt").read_text().splitlines() if l.strip() and int(l.split()[0]) not in ignoradas]
        for linea in filas[i]:
            if int(linea.split()[0]) >= CLASES_MODELO:
                raise SystemExit(f"La foto {i} usa una etiqueta nueva (clase {linea.split()[0]}). El reajuste conserva las 12 clases de v9: una clase nueva necesita un entrenamiento completo.")
    cuenta = Counter(destino.values())
    print("reparto:", dict(cuenta), "· evaluación con plaga:", sum(1 for i, sp in destino.items() if sp == "op_eval" and any(int(l.split()[0]) != 0 for l in filas[i])))
    if a.solo_resumen:
        return
    if cuenta["op_eval"] < MIN_EVAL:
        raise SystemExit(f"Solo hay {cuenta['op_eval']} fotos de evaluación y hacen falta {MIN_EVAL}: {dict(cuenta)}. Sigue etiquetando el lote.")
    sal = Path(a.salida)
    sal.mkdir(parents=True, exist_ok=False)
    for sp in ("op_train", "op_val", "op_eval"):
        (sal / sp / "images").mkdir(parents=True); (sal / sp / "labels").mkdir(parents=True)
    for i, sp in destino.items():
        shutil.copy2(e / "images" / f"{i}.jpg", sal / sp / "images" / f"{i}.jpg")
        (sal / sp / "labels" / f"{i}.txt").write_text("".join(l + "\n" for l in filas[i]))
    (sal / "pesos").mkdir()
    shutil.copy2(a.v9, sal / "pesos" / "v9_det.pt"); shutil.copy2(a.v8, sal / "pesos" / "v8_det.pt")
    print("dataset listo en", sal, dict(cuenta))


if __name__ == "__main__":
    main()
