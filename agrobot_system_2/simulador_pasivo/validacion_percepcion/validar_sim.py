#!/usr/bin/env python3
"""Validación CUALITATIVA de la UNet de Codex sobre imágenes del SIMULADOR.

Usa el modelo preentrenado (segmentation_models_pytorch, encoder timm
mobilenetv3, 6 canales, 5 clases) para inferir sobre fotogramas que renderiza el
simulador del huerto, y guarda capturas: RGB del sim + superposición de la
segmentación inferida. También resume qué clases predice y cuánto.

AVISOS DE HONESTIDAD (importantes, van también en el informe):
 · El PREPROCESADO exacto de Codex (cómo arma los 6 canales RGB+a*+ExGR+relieve y
   su normalización) NO está en el repo: aquí se RECONSTRUYE con la receta
   estándar. Si no coincide con el de Codex, la inferencia se degrada. Resultado
   = CUALITATIVO, no una métrica medida.
 · El modelo se entrenó con fotos (mIoU_real 0.73) y aquí ve imágenes SINTÉTICAS
   del sim: hay brecha de dominio (Codex ya midió mIoU_synth 0.45).
 · El sim NO produce máscara de VERDAD por clase, así que NO se calcula IoU aquí
   (eso exige el modo de render semántico, pendiente del bloque 3).

    python3 validar_sim.py
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import cv2
import torch
import segmentation_models_pytorch as smp

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parent
sys.path.insert(0, str(RAIZ))
sys.path.insert(0, str(RAIZ.parent / "jetson" / "backend"))

import mundo as mundo_mod   # noqa: E402

MODELO = AQUI / "modelo_agrobot_unet.pth"
MUNDO_JSON = RAIZ / "mundos" / "huerta_fotos_50cm.json"
CONFIG = RAIZ / "config_huerta.json"
SALIDAS = AQUI / "salidas"

CLASES = ["fondo", "sendero_visible", "zona_cultivo_no_pisar",
          "bancal_borde_elevado", "obstaculo_visible"]
# Colores BGR por clase (para la superposición).
COLORES = np.array([[80, 80, 80], [40, 120, 230], [60, 180, 60],
                    [40, 200, 230], [40, 40, 220]], dtype=np.uint8)
TAM = 320                     # lado de entrada al modelo (múltiplo de 32)
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], np.float32)


def cargar_modelo():
    m = smp.Unet(encoder_name="timm-mobilenetv3_large_100", encoder_weights=None,
                 in_channels=6, classes=5)
    m.load_state_dict(torch.load(MODELO, map_location="cpu"))
    m.eval()
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return m.to(dev), dev


def preprocesar(bgr):
    """RECONSTRUCCIÓN del input de 6 canales: RGB(ImageNet) + a*(LAB) + ExGR + relieve.
    Documentado como aproximado (no es el preprocesado exacto de Codex)."""
    img = cv2.resize(bgr, (TAM, TAM), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)
    a_star = (lab[:, :, 1] - 128.0) / 128.0
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    s = r + g + b + 1e-6
    rn, gn, bn = r / s, g / s, b / s
    exgr = (2 * gn - rn - bn) - (1.4 * rn - gn)                  # ExG - ExR
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    relieve = np.sqrt(gx * gx + gy * gy)
    rgbn = (rgb - IMAGENET_MEAN) / IMAGENET_STD
    ch = np.stack([rgbn[:, :, 0], rgbn[:, :, 1], rgbn[:, :, 2],
                   a_star, exgr, relieve], axis=0).astype(np.float32)
    return torch.from_numpy(ch).unsqueeze(0)


def inferir(modelo, dev, bgr):
    x = preprocesar(bgr).to(dev)
    with torch.no_grad():
        logits = modelo(x)
        pred = logits.argmax(1)[0].cpu().numpy().astype(np.uint8)   # TAM x TAM
    return cv2.resize(pred, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)


def superponer(bgr, pred, alpha=0.5):
    color = COLORES[pred]
    return cv2.addWeighted(bgr, 1 - alpha, color, alpha, 0)


def poses_sendero(sendero, n=6):
    pts = np.array(sendero, float)
    idx = np.linspace(0, len(pts) - 2, n).astype(int)
    for i in idx:
        x, y = pts[i]
        dx, dy = pts[i + 1] - pts[i]
        yield float(x), float(y), math.atan2(dy, dx)


def main():
    print("Cargando modelo...", flush=True)
    modelo, dev = cargar_modelo()
    print(f"  modelo en {dev}", flush=True)

    d = json.load(open(MUNDO_JSON))
    mundo = mundo_mod.Mundo.desde_dict(d)
    cfg = json.load(open(CONFIG))
    geo = dict(cfg["odometria_webcam"])
    # Mirada al sendero (cámara inclinada hacia abajo). Ver calibracion_camara.
    geo["camara_inclinacion_deg"] = 33.0

    cobertura = np.zeros(5, float)
    compuestos = []
    for k, (x, y, th) in enumerate(poses_sendero(d["sendero"])):
        bgr = mundo.camara(x, y, th, geo, 640, 480)
        pred = inferir(modelo, dev, bgr)
        over = superponer(bgr, pred)
        comp = np.hstack([bgr, over])
        cv2.imwrite(str(SALIDAS / f"frame_{k:02d}.png"), comp)
        cv2.imwrite(str(SALIDAS / f"rgb_{k:02d}.png"), bgr)
        compuestos.append(comp)
        vals, cnts = np.unique(pred, return_counts=True)
        for v, c in zip(vals, cnts):
            cobertura[v] += c
        print(f"  frame {k}: clases predichas {[CLASES[v] for v in vals]}", flush=True)

    # Montaje (rejilla 2 columnas de compuestos)
    filas = [np.hstack([compuestos[i], compuestos[i + 1]]) if i + 1 < len(compuestos)
             else compuestos[i] for i in range(0, len(compuestos), 2)]
    h = min(f.shape[0] for f in filas); w = min(f.shape[1] for f in filas)
    montaje = np.vstack([f[:h, :w] for f in filas])
    cv2.imwrite(str(SALIDAS / "montaje.png"), montaje)

    cobertura = cobertura / cobertura.sum() * 100
    resumen = {CLASES[i]: round(float(cobertura[i]), 1) for i in range(5)}
    print("\nCobertura de píxeles predichos por clase (%):", resumen)
    json.dump({"cobertura_pct": resumen,
               "nota": "CUALITATIVO: preprocesado reconstruido, sin GT, brecha de dominio"},
              open(SALIDAS / "resumen.json", "w"), indent=2, ensure_ascii=False)
    print(f"\nCapturas en {SALIDAS}")


if __name__ == "__main__":
    main()
