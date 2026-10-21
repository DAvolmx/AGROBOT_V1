#!/usr/bin/env python3
"""Ajusta el clasificador interior/exterior con lo que el robot REGISTRÓ (entorno_auto.py).

  python3 tools/entorno_entrenar.py [--carpeta ~/.agrobot/entorno] [--min 60]

Etiquetas = los modos que el operador fijó a mano (una pulsación de Interior/Exterior etiqueta los 10 minutos
siguientes). Regresión logística con L2 sobre los rasgos de LIDAR/cámara; la validación separa por DÍA/BLOQUE
(muestras seguidas son casi idénticas: mezclarlas inflaría la precisión) y se compara con las reglas actuales
sobre las MISMAS muestras de validación.

NO despliega nada: escribe modelo_candidato.json. Solo se usa si el operador lo copia a modelo.json
(`cp ~/.agrobot/entorno/modelo_candidato.json ~/.agrobot/entorno/modelo.json`) y reinicia el backend, y
solo debería hacerlo si gana a las reglas en validación.
"""
import argparse
import glob
import json
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))
import entorno_auto as E


def cargar(carpeta):
    filas = []
    for f in sorted(glob.glob(str(Path(carpeta) / "*.jsonl"))):
        dia = Path(f).stem
        with open(f) as fh:
            for linea in fh:
                try:
                    m = json.loads(linea)
                except ValueError:
                    continue
                if m.get("etiqueta") in ("interior", "exterior") and isinstance(m.get("rasgos"), dict):
                    filas.append((dia, m))
    return filas


def sigmoide(z):
    return 1.0 / (1.0 + np.exp(-np.clip(z, -30, 30)))


def ajustar(X, y, l2=0.05, it=800, lr=0.3):
    w, b = np.zeros(X.shape[1]), 0.0
    for _ in range(it):
        p = sigmoide(X @ w + b)
        w -= lr * (X.T @ (p - y) / len(y) + l2 * w)
        b -= lr * float(np.mean(p - y))
    return w, b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--carpeta", default=str(E.CARPETA))
    ap.add_argument("--min", type=int, default=60, help="mínimo de muestras etiquetadas por clase")
    a = ap.parse_args()
    filas = cargar(a.carpeta)
    y_all = np.array([1.0 if m["etiqueta"] == "interior" else 0.0 for _, m in filas])
    print(f"muestras etiquetadas: {len(filas)} (interior {int(y_all.sum())}, exterior {int(len(y_all) - y_all.sum())})")
    if len(filas) == 0 or min(y_all.sum(), len(y_all) - y_all.sum()) < a.min:
        sys.exit(f"Faltan datos: hacen falta ≥{a.min} muestras de CADA clase etiquetadas a mano "
                 f"(fija Interior o Exterior en el dashboard mientras el robot está en ese entorno).")

    cols = [k for k in E.RASGOS if np.mean([m["rasgos"].get(k) is not None for _, m in filas]) >= 0.8]
    X = np.array([[m["rasgos"].get(k) if m["rasgos"].get(k) is not None else 0.0 for k in cols] for _, m in filas], float)
    dias = np.array([d for d, _ in filas])
    # Validación por bloques: los días con ambas clases se reparten; si hay un solo día, bloques de tiempo contiguos.
    ud = sorted(set(dias))
    if len(ud) >= 3:
        val_dias = set(ud[-max(1, len(ud) // 4):])
        val = np.array([d in val_dias for d in dias])
    else:
        val = np.zeros(len(filas), bool)
        for c in (0.0, 1.0):
            idx = np.flatnonzero(y_all == c)
            val[idx[int(len(idx) * 0.8):]] = True
        print("AVISO: pocos días de datos: validación = el último 20 % de cada clase (hay fuga parcial)")
    tr = ~val
    if len(set(y_all[tr])) < 2 or len(set(y_all[val])) < 2:
        sys.exit("La validación o el entrenamiento se quedan con una sola clase: faltan datos de las dos.")

    media, desv = X[tr].mean(0), X[tr].std(0) + 1e-6
    w, b = ajustar((X[tr] - media) / desv, y_all[tr])
    pv = sigmoide(((X[val] - media) / desv) @ w + b)
    acc_modelo = float(np.mean((pv >= 0.5) == (y_all[val] == 1)))
    reglas = np.array([E.puntuacion(m["rasgos"])[0] or 0.0 for (_, m), v in zip(filas, val) if v])
    acc_reglas = float(np.mean((reglas >= 0) == (y_all[val] == 1)))
    ext_malos = float(np.mean(pv[y_all[val] == 0] >= 0.5))
    print(f"validación (n={int(val.sum())}): modelo {acc_modelo:.3f} · reglas actuales {acc_reglas:.3f}")
    print(f"  exterior tomado por interior (error SEGURO): {ext_malos:.3f}")
    print(f"  interior tomado por exterior (error PELIGROSO): {float(np.mean(pv[y_all[val] == 1] < 0.5)):.3f}")
    print("  pesos:", {k: round(float(v), 2) for k, v in zip(cols, w)})

    sal = Path(a.carpeta) / "modelo_candidato.json"
    sal.write_text(json.dumps({"rasgos": cols, "media": media.tolist(), "desv": desv.tolist(), "pesos": w.tolist(),
                               "sesgo": float(b), "n_train": int(tr.sum()), "n_val": int(val.sum()),
                               "acc_val_modelo": acc_modelo, "acc_val_reglas": acc_reglas}, indent=1))
    print(f"\nCandidato en {sal}  (NO desplegado)")
    print("GANA a las reglas: se puede probar con `cp modelo_candidato.json modelo.json`." if acc_modelo > acc_reglas
          else "NO gana a las reglas actuales: no cambiar nada.")


if __name__ == "__main__":
    main()
