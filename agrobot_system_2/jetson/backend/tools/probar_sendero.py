#!/usr/bin/env python3
"""Prueba de regresión del detector de senderos (sendero.py) con escenas sintéticas.

    python3 tools/probar_sendero.py [carpeta_salida]

Genera cada píxel trazando su rayo hasta el suelo con la geometría real de la
cámara: sendero de tierra oscura bordeado por franjas de paja seca (como la
huerta de pruebas, fotos del 2026-09-19), pajas sueltas sobre el camino y
plantas escasas. Mueve y tuerce el robot, curva el sendero, y compara lo
medido con la verdad. Escribe resultados.json y una imagen de depuración.
Sale con código 1 si alguna decisión de giro es incorrecta.
"""
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sendero  # noqa: E402

W, H = 640, 480
GEO = {"camara_altura_m": 0.30, "camara_inclinacion_deg": 15.5, "camara_hfov_deg": 71,
       "camara_adelante_m": 0.0}
CFG = {"sendero_activo": True, "sendero_vision": True, "sendero_vision_peso": 1.0,
       "sendero_tolerancia_cm": 10, "sendero_pared_max_cm": 150, "sendero_paja": True,
       "sendero_verde_h_min": 30, "sendero_verde_h_max": 90, "sendero_verde_s_min": 60}
SALIDA = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[3] / "docs" / "sendero"

CASOS = [  # nombre, robot a la izquierda del centro (m), robot girado a la izq (°), curvatura
    ("centrado", 0, 0, 0), ("10 cm a la izquierda", 0.10, 0, 0), ("10 cm a la derecha", -0.10, 0, 0),
    ("20 cm a la izquierda", 0.20, 0, 0), ("20 cm a la derecha", -0.20, 0, 0),
    ("torcido 12° a la izquierda", 0, 12, 0), ("torcido 12° a la derecha", 0, -12, 0),
    ("15 cm izq + 10° izq", 0.15, 10, 0), ("curva a la izquierda", 0, 0, 0.12),
    ("curva a la derecha", 0, 0, -0.12), ("curva izq + 10 cm der", -0.10, 0, 0.12),
    ("sombra fuerte, 15 cm izq", 0.15, 0, 0),
]
# Cada caso se corre también con la imagen viñeteada. Está aquí porque este
# defecto pasó desapercibido hasta que el simulador lo midió: los bordes más
# oscuros que el centro hundían el seguimiento del sendero del 83 % al 52 %,
# y las escenas sintéticas de antes eran todas de luz perfectamente plana. La
# corrección vive en `sendero.igualar_luz`.
VINETEOS = (0.0, 0.35)
SEMILLAS = (1, 2, 3)
ANCHO_REAL = 0.50


def vinetear(img, fuerza):
    """Oscurece los bordes como hace cualquier objetivo real."""
    if fuerza <= 0:
        return img
    h, w = img.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w]
    r = np.sqrt(((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2)
    caida = 1 - fuerza * np.clip(r, 0, 1) ** 2
    return np.clip(img.astype(np.float32) * caida[..., None], 0, 255).astype(np.uint8)


def escena(lateral=0.0, yaw_deg=0.0, curva=0.0, ancho=ANCHO_REAL, paja_ancho=0.35, semilla=0, sombra=0.25):
    rng = np.random.default_rng(semilla)
    f = (W / 2) / math.tan(math.radians(GEO["camara_hfov_deg"]) / 2)
    phi, h = math.radians(GEO["camara_inclinacion_deg"]), GEO["camara_altura_m"]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    xn, yn = (u - W / 2) / f, (v - H / 2) / f
    baja = yn * math.cos(phi) + math.sin(phi)
    suelo = baja > 1e-3
    t = np.where(suelo, h / np.where(suelo, baja, 1), 0)
    xr, yr = t * (math.cos(phi) - yn * math.sin(phi)), -t * xn
    psi = math.radians(yaw_deg)
    xs = xr * math.cos(psi) - yr * math.sin(psi)
    ys = xr * math.sin(psi) + yr * math.cos(psi) + lateral
    d = np.abs(ys - curva * xs ** 2)
    en_paja = (d > ancho / 2) & (d < ancho / 2 + paja_ancho)
    fib = np.sin(xs * 173 + ys * 41) * np.sin(xs * 57 - ys * 211) + rng.normal(0, 0.35, xs.shape)
    es_paja = (en_paja & (fib > -0.40)) | (~en_paja & (d < ancho / 2) & (fib > 0.85))
    img = np.empty((H, W, 3), np.float32)
    img[:] = (58, 70, 84)
    img += rng.normal(0, 9, img.shape)
    img[es_paja] = np.array((105, 165, 195)) + rng.normal(0, 18, (int(es_paja.sum()), 3))
    for lado in (1, -1):
        for xp in np.arange(0.3, 4, rng.uniform(0.4, 0.8)):
            yp = lado * (ancho / 2 + paja_ancho / 2) + curva * xp ** 2
            img[((xs - xp) ** 2 + (ys - yp) ** 2) < 0.06 ** 2] = (50, 150, 70)
    img *= (1 - sombra * (u / W))[..., None]
    img[~suelo] = (210, 200, 190)
    img[int(H * 0.96):] = (30, 30, 170)          # chasis rojo en el borde inferior
    return np.clip(img, 0, 255).astype(np.uint8)


def main() -> int:
    SALIDA.mkdir(parents=True, exist_ok=True)
    filas, fallos, errores_lat = [], 0, []
    for (nombre, lat, yaw, cur), vineta in ((c, v) for c in CASOS for v in VINETEOS):
        sombra = 0.55 if nombre.startswith("sombra") else 0.25
        if vineta:
            nombre = f"{nombre} (viñeteado)"
        for sem in SEMILLAS:
            seg = sendero.SeguidorSendero(lambda: CFG, None, get_geometria=lambda: GEO)
            dec = None
            for k in range(3):          # 3 fotogramas: el seguidor aprende el ancho
                img = vinetear(escena(lat, yaw, cur, semilla=sem * 10 + k, sombra=sombra), vineta)
                seg._obtener_frame = lambda im=img: (im, 0, 0)
                dec = seg.correccion(cv2, {})
            err_real = ((-lat + cur * sendero.MIRADA_M ** 2) * math.cos(math.radians(yaw))
                        + sendero.MIRADA_M * math.sin(math.radians(-yaw)))
            tol = CFG["sendero_tolerancia_cm"] / 100
            esperado = "izquierda" if err_real > tol else ("derecha" if err_real < -tol else None)
            limite = abs(abs(err_real) - tol) < 0.03     # a 3 cm del umbral: cualquiera vale
            ok = dec == esperado or limite
            fallos += not ok
            v = seg.vision
            if v.lateral_m is not None and yaw == 0 and cur == 0:
                errores_lat.append(abs(v.lateral_m + lat))
            filas.append({"caso": nombre, "semilla": sem, "lateral_real_cm": round(-lat * 100),
                          "lateral_medido_cm": None if v.lateral_m is None else round(v.lateral_m * 100),
                          "ancho_medido_cm": None if v.ancho_m is None else round(v.ancho_m * 100),
                          "rumbo_real_deg": -yaw,
                          "rumbo_medido_deg": None if v.angulo_deg is None else round(v.angulo_deg),
                          "esperado": esperado or "seguir", "decision": dec or "seguir", "ok": ok})
            if not ok:
                print(f"FALLO {nombre} (semilla {sem}): esperado {esperado}, decidió {dec}")
    resumen = {"pruebas": len(filas), "fallos": fallos, "ancho_real_cm": ANCHO_REAL * 100,
               "error_lateral_medio_cm": round(100 * float(np.mean(errores_lat)), 1) if errores_lat else None}
    (SALIDA / "resultados.json").write_text(json.dumps({"resumen": resumen, "casos": filas},
                                                       indent=2, ensure_ascii=False))
    cen = sendero.Cenital(GEO)
    vistas = []
    for args in ((0.10, 0, 0), (0, 0, 0.12)):
        img = escena(*args, semilla=1)
        vistas.append(sendero.dibujar(cv2, img, sendero.detectar(cv2, img, CFG, GEO, cen)))
    cv2.imwrite(str(SALIDA / "deteccion_sintetica.jpg"), np.vstack(vistas))
    print(json.dumps(resumen, ensure_ascii=False))
    return 1 if fallos else 0


if __name__ == "__main__":
    sys.exit(main())
