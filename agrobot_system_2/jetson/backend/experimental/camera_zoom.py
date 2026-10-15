"""
Camera Zoom Control — Multi-Scale Detection de Plagas
======================================================

Controla zoom óptico/digital de la cámara para detectar plagas y plantas
en diferentes escalas:
  • Zoom 1x (60° FOV): Sendero completo, orientación
  • Zoom 2x (30° FOV): Plantas individuales
  • Zoom 4x (15° FOV): Plagas, hojas, detalles

Soporta v4l2 para USB cameras y libcamera para CSI/UVC.
"""

import subprocess
import json
import numpy as np
import cv2
from pathlib import Path
from typing import Optional, Dict, Tuple
import threading
import time


class CameraZoomControl:
    """Controla zoom de cámara y parámetros de exposición."""

    def __init__(self, camera_device: str = "/dev/video0",
                 zoom_strategy: str = "adaptive"):
        self.device = camera_device
        self.zoom_strategy = zoom_strategy  # "adaptive" o "manual"

        # Niveles de zoom soportados
        self.zoom_levels = {
            1: {"crop": 1.0, "name": "wide", "fov_deg": 60},
            2: {"crop": 0.5, "name": "medium", "fov_deg": 30},
            4: {"crop": 0.25, "name": "tight", "fov_deg": 15}
        }

        self.current_zoom = 1
        self.target_zoom = 1
        self.zoom_smooth = 0.2  # factor de suavizado

        # Auto-zoom basado en confianza de detección
        self.detection_confidence = 0.0
        self.auto_zoom_threshold = 0.6
        self.time_at_zoom = 0.0
        self.min_zoom_duration = 2.0  # segundos mínimos en cada zoom

        # Exposición dinámico
        self.exposure_mode = "auto"  # "auto", "manual", "night"
        self.exposure_value = 0
        self.iso_value = 100

        self._lock = threading.Lock()
        self._zoom_thread = None
        self._stop = threading.Event()

    def set_manual_zoom(self, level: float):
        """Establece zoom manual (1, 2, 4)."""
        level = float(level)
        if level not in self.zoom_levels:
            level = min(self.zoom_levels.keys(), key=lambda x: abs(x - level))

        with self._lock:
            self.target_zoom = level
            self.zoom_strategy = "manual"

    def update_detection_confidence(self, confidence: float):
        """Actualiza confianza de detección para auto-zoom."""
        self.detection_confidence = float(np.clip(confidence, 0.0, 1.0))

    def _compute_target_zoom(self) -> float:
        """Calcula zoom objetivo basado en confianza."""
        if self.zoom_strategy != "adaptive":
            return self.target_zoom

        # Zoom automático: más zoom si baja confianza
        if self.detection_confidence < 0.3:
            return 4.0  # Zoom máximo (busca plagas pequeñas)
        elif self.detection_confidence < 0.6:
            return 2.0  # Zoom medio (plantas)
        else:
            return 1.0  # Zoom normal (sendero)

    def update_zoom(self) -> float:
        """Actualiza zoom suavemente hacia objetivo."""
        target = self._compute_target_zoom()

        # Cambio lento para evitar mareos
        current_smooth = (self.current_zoom * (1 - self.zoom_smooth) +
                         target * self.zoom_smooth)

        # Snap a nivel si está cerca
        closest_level = min(self.zoom_levels.keys(),
                          key=lambda x: abs(x - current_smooth))
        if abs(current_smooth - closest_level) < 0.1:
            current_smooth = closest_level

        self.current_zoom = current_smooth
        return current_smooth

    def apply_zoom_to_frame(self, frame: np.ndarray) -> np.ndarray:
        """Aplica zoom a frame (crop + upscale)."""
        zoom = self.current_zoom

        if zoom <= 1.0:
            return frame

        h, w = frame.shape[:2]

        # Crop centered
        crop_h = int(h / zoom)
        crop_w = int(w / zoom)

        y_start = (h - crop_h) // 2
        x_start = (w - crop_w) // 2

        cropped = frame[y_start:y_start+crop_h, x_start:x_start+crop_w]

        # Upscale de vuelta a tamaño original
        zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)

        return zoomed

    def get_camera_matrix_adjusted(self, base_camera_matrix: np.ndarray
                                  ) -> np.ndarray:
        """Ajusta matriz de cámara por zoom actual."""
        zoom = self.current_zoom

        # Zoom afecta focal length (aumenta effective focal)
        adjusted = base_camera_matrix.copy()
        adjusted[0, 0] *= zoom  # fx
        adjusted[1, 1] *= zoom  # fy

        # Punto principal se mantiene igual (es pixel absoluto)
        return adjusted

    def set_exposure_mode(self, mode: str):
        """Cambia modo de exposición: "auto", "manual", "night"."""
        self.exposure_mode = mode
        self._apply_exposure_settings()

    def _apply_exposure_settings(self):
        """Aplica configuración de exposición a cámara (v4l2-ctl)."""
        try:
            if self.exposure_mode == "auto":
                subprocess.run([
                    "v4l2-ctl", "-d", self.device,
                    "-c", "exposure_auto=1",  # Auto
                    "-c", "auto_exposure=1"
                ], capture_output=True, timeout=2)

            elif self.exposure_mode == "manual":
                # Manual: exposición media para plagas pequeñas
                subprocess.run([
                    "v4l2-ctl", "-d", self.device,
                    "-c", "exposure_auto=3",  # Manual
                    "-c", "exposure_time=1000",
                    "-c", "gain=50"
                ], capture_output=True, timeout=2)

            elif self.exposure_mode == "night":
                # Noche: mucha ganancia, exposición larga
                subprocess.run([
                    "v4l2-ctl", "-d", self.device,
                    "-c", "exposure_auto=3",  # Manual
                    "-c", "exposure_time=3000",
                    "-c", "gain=200"
                ], capture_output=True, timeout=2)

        except Exception as e:
            print(f"[CameraZoom] Error aplicando exposición: {e}")

    def get_zoom_info(self) -> Dict:
        """Información actual de zoom."""
        return {
            "current_zoom": float(self.current_zoom),
            "target_zoom": float(self.target_zoom),
            "zoom_level_name": self.zoom_levels[int(self.current_zoom)]["name"],
            "fov_deg": self.zoom_levels[int(self.current_zoom)]["fov_deg"],
            "detection_confidence": float(self.detection_confidence),
            "exposure_mode": self.exposure_mode,
            "strategy": self.zoom_strategy
        }


class MultiScaleYOLO:
    """Ejecuta YOLO en múltiples escalas (1x, 2x, 4x) para máxima cobertura."""

    def __init__(self, model_path: str):
        try:
            from ultralytics import YOLO
            self.yolo = YOLO(model_path)
        except:
            self.yolo = None
            print("[MultiScaleYOLO] YOLO no disponible")

    def infer_multi_scale(self, frame: np.ndarray,
                         zoom_control: CameraZoomControl) -> Dict:
        """Corre inferencia en múltiples escalas."""
        if self.yolo is None:
            return {"detections": [], "confidence": 0.0}

        detections = []
        max_conf = 0.0

        current_zoom = zoom_control.current_zoom

        # Infer at current zoom
        frame_zoomed = zoom_control.apply_zoom_to_frame(frame)

        try:
            results = self.yolo(frame_zoomed, conf=0.3, verbose=False)

            for det in results[0].boxes.data:
                x1, y1, x2, y2, conf, cls_id = det.tolist()

                # Undo zoom en coordenadas
                h, w = frame.shape[:2]
                zh, zw = frame_zoomed.shape[:2]

                crop_h = int(h / current_zoom)
                crop_w = int(w / current_zoom)
                y_off = (h - crop_h) // 2
                x_off = (w - crop_w) // 2

                x1_orig = x1 * crop_w / zw + x_off
                y1_orig = y1 * crop_h / zh + y_off
                x2_orig = x2 * crop_w / zw + x_off
                y2_orig = y2 * crop_h / zh + y_off

                detections.append({
                    "bbox": [int(x1_orig), int(y1_orig), int(x2_orig), int(y2_orig)],
                    "confidence": float(conf),
                    "class": int(cls_id),
                    "class_name": self.yolo.names[int(cls_id)],
                    "zoom_level": float(current_zoom)
                })

                max_conf = max(max_conf, float(conf))

        except Exception as e:
            print(f"[MultiScaleYOLO] Error en inferencia: {e}")

        zoom_control.update_detection_confidence(max_conf)

        return {
            "detections": detections,
            "confidence": max_conf,
            "zoom": float(current_zoom),
            "n_detections": len(detections)
        }
