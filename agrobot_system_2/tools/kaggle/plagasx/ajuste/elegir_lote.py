"""Elige las fotos del robot que el operador etiquetará para la prueba P6 y el reajuste de PLAGASX.

La selección no usa ningún modelo: quita fotos borrosas, sin vegetación y casi repetidas (el robot pasa mucho tiempo parado) y sortea con semilla fija.
Copia a ~/.agrobot/datos_usuario/lote_p6_<fecha>/, visible en etiquetar.html con la fuente «usuario».
Uso: python3 elegir_lote.py [--n 150] [--dry] [sesiones...]"""
import argparse, random, shutil, time
from pathlib import Path

import cv2
import numpy as np

CAMPO = Path.home() / ".agrobot" / "campo"
# Salidas a la huerta con coles sembradas (ensayos E-2 y E-3).
SESIONES = ["20261028_194540_ed7b3b", "20261028_195430_e352b3", "20261028_200037_32de40", "20261028_200833_d17537", "20261029_142951_a81509"]
MIN_NITIDEZ, MIN_VERDE, MIN_CAMBIO = 40.0, 0.08, 6.0


def medidas(ruta):
    img = cv2.imread(str(ruta))
    if img is None:
        return None
    b, g, r = [c.astype(np.int32) for c in cv2.split(cv2.resize(img, (160, 120), interpolation=cv2.INTER_AREA))]
    verde = float(np.mean((g > r + 8) & (g > b + 8)))
    gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gris, cv2.CV_64F).var()), verde, cv2.resize(gris, (32, 24), interpolation=cv2.INTER_AREA).astype(np.int16)


def candidatas(sesion):
    d = CAMPO / sesion / "fotos"
    fotos = sorted(d.glob("*.jpg"), key=lambda p: float(p.stem.split("_")[1])) if d.is_dir() else []
    out, ultima, filtro = [], None, {"borrosa": 0, "sin_verde": 0, "repetida": 0}
    for f in fotos:
        m = medidas(f)
        if m is None:
            continue
        nit, verde, mini = m
        if nit < MIN_NITIDEZ:
            filtro["borrosa"] += 1
        elif verde < MIN_VERDE:
            filtro["sin_verde"] += 1
        elif ultima is not None and float(np.mean(np.abs(mini - ultima))) < MIN_CAMBIO:
            filtro["repetida"] += 1
        else:
            out.append(f); ultima = mini
    print(f"{sesion}: {len(fotos)} fotos -> {len(out)} candidatas; descartadas {filtro}")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sesiones", nargs="*", default=SESIONES)
    ap.add_argument("--n", type=int, default=150)
    ap.add_argument("--dry", action="store_true")
    a = ap.parse_args()
    todas = [f for s in a.sesiones for f in candidatas(s)]
    if len(todas) < a.n:
        print(f"AVISO: solo hay {len(todas)} candidatas para {a.n} pedidas")
    elegidas = sorted(random.Random(42).sample(todas, min(a.n, len(todas))), key=str)
    print("elegidas por sesión:", {s: sum(1 for f in elegidas if f.parent.parent.name == s) for s in a.sesiones})
    if a.dry:
        return
    destino = Path.home() / ".agrobot" / "datos_usuario" / f"lote_p6_{time.strftime('%Y%m%d')}"
    destino.mkdir(parents=True, exist_ok=False)                 # no mezcla ni pisa un lote que ya se esté etiquetando
    for f in elegidas:
        shutil.copy2(f, destino / f"{f.parent.parent.name}__{f.name}")
    print("copiadas a", destino)


if __name__ == "__main__":
    main()
