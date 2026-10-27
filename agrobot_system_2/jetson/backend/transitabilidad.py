"""Modelo de transitabilidad (libre / obstáculo) para suelo_libre (2026-10-27).

Envuelve la UNet de `simulador_pasivo/percepcion_sintetica` (entrenar_trav.py): imagen BGR de la cámara -> mapa de 5 clases (0 fondo, 1 camino libre, 2 planta/maceta,
3 borde bajo, 4 obstáculo). SueloLibre (metodo="modelo") toma «suelo = clase 1» y deja que el resto de su cadena (primera racha de no-suelo por columna, geometría,
persistencia en fotogramas) siga igual; el scan virtual resultante lo fusiona camara_lidar.py. Entrenado SOLO con imágenes sintéticas: sin evaluación en fotos reales.
Se activa con autonomia.suelo_modelo = "<ruta .pt>"; si falla al cargar, el sistema sigue con el método por apariencia."""
from __future__ import annotations

import numpy as np

ALTO, ANCHO, PAD = 240, 320, 8
MEDIA = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


class Transitabilidad:
    def __init__(self, ruta: str, dispositivo: str | None = None):
        import torch
        import segmentation_models_pytorch as smp
        self.torch = torch
        self.dev = torch.device(dispositivo or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.net = smp.Unet("timm-mobilenetv3_large_100", encoder_weights=None, in_channels=3, classes=5).to(self.dev)
        ck = torch.load(ruta, map_location=self.dev, weights_only=False)
        self.net.load_state_dict(ck["state_dict"] if "state_dict" in ck else ck)
        self.net.eval()
        if self.dev.type == "cuda":
            self.net.half()

    def __call__(self, frame_bgr) -> np.ndarray:
        """(H,W,3) BGR uint8 -> (240,320) uint8 con las 5 clases."""
        import cv2
        torch = self.torch
        rgb = cv2.cvtColor(cv2.resize(frame_bgr, (ANCHO, ALTO), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
        x = (rgb.astype(np.float32) / 255.0 - MEDIA) / STD
        x = torch.from_numpy(x).permute(2, 0, 1).unsqueeze(0).to(self.dev)
        x = torch.nn.functional.pad(x, (0, 0, PAD, PAD), mode="reflect")
        if self.dev.type == "cuda":
            x = x.half()
        with torch.no_grad():
            return self.net(x)[:, :, PAD:-PAD, :].argmax(1)[0].cpu().numpy().astype(np.uint8)
