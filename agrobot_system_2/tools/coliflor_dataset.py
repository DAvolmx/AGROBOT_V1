#!/usr/bin/env python3
"""Captura y cuenta fotos para dos clasificadores de la huerta:
  ESPECIE: col · coliflor · acelga · fondo
  ESTADO : sana · marchita · dano_hoja · minador · mosca_blanca · fondo
(dano_hoja = hoja con agujeros/mordidas; minador = galerías/serpenteos del minador de hoja.)

La etiqueta ES LA CARPETA: ~/.agrobot/dataset_planta/<tarea>/<clase>/<sesion>__NNN.jpg
(`fondo` = todo lo que no es la planta; se guarda en las dos tareas)
Toma los cuadros del MJPEG del backend (http://localhost:8080/video), descarta los borrosos, muy
oscuros o repetidos, y los guarda ya en la carpeta de la clase.

  python3 tools/coliflor_dataset.py capturar coliflor --n 40 --cada 1.5 --sesion planta1_manana
  python3 tools/coliflor_dataset.py capturar minador  --n 40 --cada 1.5 --sesion hojaB
  python3 tools/coliflor_dataset.py contar
"""
import argparse
import re
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

BASE = Path.home() / ".agrobot" / "dataset_planta"
TAREAS = {
    "especie": ("col", "coliflor", "acelga", "fondo"),
    "estado": ("sana", "marchita", "dano_hoja", "minador", "mosca_blanca", "fondo"),
}
CLASES = tuple(dict.fromkeys(c for cs in TAREAS.values() for c in cs))


def tareas_de(clase):
    return [t for t, cs in TAREAS.items() if clase in cs]
URL = "http://localhost:8080/video"
NITIDEZ_MIN = 40.0       # varianza del laplaciano: por debajo = borrosa
BRILLO_MIN = 35.0        # brillo medio (0-255): por debajo = muy oscura
DIF_MIN = 4.0            # diferencia media vs la última guardada: por debajo = casi igual
MIN_POR_CLASE, MIN_SESIONES = 60, 3


def cuadros(url=URL):
    """Generador de cuadros BGR del MJPEG del backend."""
    buf = b""
    with urllib.request.urlopen(url, timeout=10) as r:
        while True:
            buf += r.read(65536)
            a = buf.find(b"\xff\xd8")
            b = buf.find(b"\xff\xd9", a + 2) if a >= 0 else -1
            if a >= 0 and b > a:
                jpg, buf = buf[a:b + 2], buf[b + 2:]
                yield cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            elif len(buf) > 4_000_000:
                buf = b""


def capturar(clase, n, cada, sesion):
    carpetas = [BASE / t / clase for t in tareas_de(clase)]
    for c in carpetas:
        c.mkdir(parents=True, exist_ok=True)
    carpeta = carpetas[0]
    previas = len(list(carpeta.glob(f"{sesion}__*.jpg")))   # repetir la sesión CONTINÚA la numeración
    guardadas, previa, t_ult = 0, None, 0.0
    rechazos = {"borrosa": 0, "oscura": 0, "repetida": 0}
    print(f"Capturando {n} fotos de '{clase}' (sesión {sesion}) cada {cada} s. Mueve un poco la cámara/planta entre fotos.")
    for frame in cuadros():
        if time.time() - t_ult < cada:
            continue
        t_ult = time.time()
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if cv2.Laplacian(g, cv2.CV_64F).var() < NITIDEZ_MIN:
            rechazos["borrosa"] += 1; continue
        if g.mean() < BRILLO_MIN:
            rechazos["oscura"] += 1; continue
        chica = cv2.resize(g, (64, 48)).astype(np.float32)
        if previa is not None and float(np.abs(chica - previa).mean()) < DIF_MIN:
            rechazos["repetida"] += 1; continue
        previa = chica
        guardadas += 1
        for c in carpetas:
            cv2.imwrite(str(c / f"{sesion}__{previas + guardadas:03d}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        print(f"  [{guardadas}/{n}] guardada (nitidez ok, brillo {g.mean():.0f})", flush=True)
        if guardadas >= n:
            break
    print(f"Listo: {guardadas} fotos en {', '.join(str(c) for c in carpetas)}. Descartadas: {rechazos}")
    if rechazos["oscura"] > guardadas:
        print("  AVISO: muchas fotos muy oscuras — más luz (LED o lámpara cerca de la hoja).")
    if rechazos["borrosa"] > guardadas:
        print("  AVISO: muchas borrosas — robot/planta quietos y la cámara a 10-25 cm (enfoque).")


def contar():
    for tarea, clases in TAREAS.items():
        print(f"== {tarea.upper()}")
        ok = True
        for c in clases:
            fotos = sorted((BASE / tarea / c).glob("*.jpg")) if (BASE / tarea / c).exists() else []
            ses = {re.split(r"__", f.name)[0] for f in fotos}
            falta = []
            if len(fotos) < MIN_POR_CLASE:
                falta.append(f"faltan {MIN_POR_CLASE - len(fotos)} fotos")
            if len(ses) < MIN_SESIONES:
                falta.append(f"faltan {MIN_SESIONES - len(ses)} sesiones distintas")
            ok &= not falta or c == "fondo"
            print(f"  {c:13s} {len(fotos):4d} fotos · {len(ses)} sesiones" + (f"  → {', '.join(falta)}" if falta else "  ✓"))
        print("  Listo para entrenar.\n" if ok else f"  Aún no: mínimo {MIN_POR_CLASE} fotos y {MIN_SESIONES} sesiones por clase.\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capturar")
    c.add_argument("clase", choices=CLASES)
    c.add_argument("--n", type=int, default=40)
    c.add_argument("--cada", type=float, default=1.5)
    c.add_argument("--sesion", default=time.strftime("%Y%m%d_%H%M"))
    sub.add_parser("contar")
    a = ap.parse_args()
    if a.cmd == "capturar":
        capturar(a.clase, a.n, a.cada, re.sub(r"[^A-Za-z0-9_-]", "", a.sesion) or "s")
    else:
        contar()
