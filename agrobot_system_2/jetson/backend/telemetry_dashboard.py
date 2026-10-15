"""
Telemetry Dashboard — Visualización en Tiempo Real del Bot Real
==============================================================

Captura datos del bot real y los muestra con:
  • Video en vivo con overlays
  • Análisis multispectral (NDVI, Chlorophyll)
  • SLAM visual + trayectoria
  • LIDAR en tiempo real
  • Detección de plagas
  • Path detection feedback

Se integra con jetson_server.py
"""

import asyncio
import cv2
import numpy as np
from typing import Dict, Optional, Tuple, List
from dataclasses import dataclass
from datetime import datetime
import threading


@dataclass
class TelemetryData:
    """Datos telemétricos del robot."""
    timestamp: float
    robot_x: float
    robot_y: float
    robot_theta: float  # radianes
    velocity: float
    omega: float
    on_sendero: bool
    sendero_confidence: float
    ndvi: float
    chlorophyll: float
    pests_detected: List[str]
    lidar_distance: float
    lidar_scan: Optional[np.ndarray] = None
    path_center_x: float = -1
    path_angle_deg: float = 0
    zoom_level: int = 1
    slam_pose: Optional[Tuple[float, float, float]] = None


class MultispectralProcessor:
    """Procesa datos multiespectrales desde el bot."""

    @staticmethod
    def calculate_ndvi_from_frame(frame: np.ndarray) -> float:
        """
        Calcula NDVI simulado desde frame RGB.
        En un bot real, usarías cámara multispectral.
        """
        if frame is None or frame.size == 0:
            return 0.5

        # Convierte a HSV para análisis de vegetación
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Extrae canal de verde
        h = hsv[:, :, 0].astype(np.float32)
        v = hsv[:, :, 2].astype(np.float32)

        # Verde está en rango H: 35-85
        green_mask = ((h >= 35) & (h <= 85)).astype(np.float32)

        # NDVI aproximado desde brillo del verde
        ndvi = np.mean(v[green_mask > 0] / 255.0) if np.any(green_mask) else 0.5
        return np.clip(ndvi, 0, 1)

    @staticmethod
    def calculate_chlorophyll_from_frame(frame: np.ndarray) -> float:
        """Estima clorofila desde intensidad del verde."""
        if frame is None or frame.size == 0:
            return 50

        # Convierte a LAB
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

        # Canal A (verde-magenta)
        a_channel = lab[:, :, 1].astype(np.float32)

        # Valores bajos en A = más verde
        # Mapea: 128 (neutral) → 50, 100 (verde) → 70
        chlorophyll = 50 + (128 - a_channel.mean()) * 0.2
        return np.clip(chlorophyll, 10, 80)


class RealtimeTelemetry:
    """Captura y visualiza telemetría en tiempo real."""

    def __init__(self, width: int = 1280, height: int = 720):
        self.width = width
        self.height = height

        # Datos actuales
        self.telemetry: Optional[TelemetryData] = None
        self.latest_frame: Optional[np.ndarray] = None

        # Historial para gráficos
        self.history_ndvi = []
        self.history_chlorophyll = []
        self.history_pests = []
        self.history_time = []

        self.max_history = 300  # 10 segundos a 30 fps

        # Procesador
        self.processor = MultispectralProcessor()

        # Fonts
        self.font_small = cv2.FONT_HERSHEY_SIMPLEX
        self.font_size_small = 0.5
        self.font_size_medium = 0.7
        self.font_size_large = 1.0

        # Colores
        self.COLOR_GREEN = (0, 255, 0)
        self.COLOR_RED = (0, 0, 255)
        self.COLOR_YELLOW = (0, 255, 255)
        self.COLOR_WHITE = (255, 255, 255)
        self.COLOR_BLACK = (0, 0, 0)

    def update_telemetry(self, data: TelemetryData, frame: np.ndarray):
        """Actualiza datos telemétricos y frame."""
        self.telemetry = data
        self.latest_frame = frame.copy() if frame is not None else None

        # Actualiza historial
        self.history_time.append(data.timestamp)
        self.history_ndvi.append(data.ndvi)
        self.history_chlorophyll.append(data.chlorophyll)
        self.history_pests.append(len(data.pests_detected))

        # Limita historial
        if len(self.history_time) > self.max_history:
            self.history_time.pop(0)
            self.history_ndvi.pop(0)
            self.history_chlorophyll.pop(0)
            self.history_pests.pop(0)

    def render(self) -> np.ndarray:
        """Renderiza el dashboard completo."""
        if self.latest_frame is None or self.telemetry is None:
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Redimensiona frame
        frame = cv2.resize(self.latest_frame, (self.width - 300, self.height))

        # Crea canvas
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        canvas[:, :self.width - 300] = frame

        # Lado derecho (paneles)
        self._render_spectral_panel(canvas)
        self._render_status_panel(canvas)
        self._render_lidar_panel(canvas)
        self._render_graphs(canvas)

        return canvas

    def _render_spectral_panel(self, canvas: np.ndarray):
        """Panel de análisis multispectral."""
        x, y = self.width - 300, 10

        # Fondo semi-transparente
        overlay = canvas[y:y+180, x:x+290].copy()
        overlay = cv2.addWeighted(overlay, 0.3, np.zeros_like(overlay), 0, 0)
        canvas[y:y+180, x:x+290] = overlay

        # Título
        cv2.putText(canvas, "MULTISPECTRAL", (x + 10, y + 25),
                   self.font_small, self.font_size_medium, self.COLOR_GREEN, 1)
        cv2.putText(canvas, "ANALYSIS", (x + 10, y + 45),
                   self.font_small, self.font_size_small, self.COLOR_GREEN, 1)

        # NDVI
        cv2.putText(canvas, f"NDVI: {self.telemetry.ndvi:.3f}", (x + 10, y + 70),
                   self.font_small, self.font_size_small, self.COLOR_GREEN, 1)

        # Barra NDVI
        bar_width = 250
        ndvi_bar_len = int(self.telemetry.ndvi * bar_width)
        cv2.rectangle(canvas, (x + 10, y + 80), (x + 10 + bar_width, y + 88),
                     self.COLOR_GREEN, 1)
        cv2.rectangle(canvas, (x + 10, y + 80), (x + 10 + ndvi_bar_len, y + 88),
                     self.COLOR_GREEN, -1)

        # Chlorophyll
        cv2.putText(canvas, f"Chlorophyll: {self.telemetry.chlorophyll:.1f}", (x + 10, y + 110),
                   self.font_small, self.font_size_small, self.COLOR_GREEN, 1)

        # Barra Chlorophyll
        chloro_bar_len = int((self.telemetry.chlorophyll / 80) * bar_width)
        cv2.rectangle(canvas, (x + 10, y + 120), (x + 10 + bar_width, y + 128),
                     self.COLOR_GREEN, 1)
        cv2.rectangle(canvas, (x + 10, y + 120), (x + 10 + chloro_bar_len, y + 128),
                     (0, 200, 0), -1)

        # Plagas detectadas
        pest_color = self.COLOR_RED if self.telemetry.pests_detected else self.COLOR_GREEN
        pest_text = " + ".join(self.telemetry.pests_detected[:2]) if self.telemetry.pests_detected else "HEALTHY"
        cv2.putText(canvas, f"Status: {pest_text}", (x + 10, y + 150),
                   self.font_small, self.font_size_small, pest_color, 1)

    def _render_status_panel(self, canvas: np.ndarray):
        """Panel de estado del robot."""
        x, y = self.width - 300, 210

        # Overlay
        overlay = canvas[y:y+180, x:x+290].copy()
        overlay = cv2.addWeighted(overlay, 0.3, np.zeros_like(overlay), 0, 0)
        canvas[y:y+180, x:x+290] = overlay

        # Título
        cv2.putText(canvas, "ROBOT STATUS", (x + 10, y + 25),
                   self.font_small, self.font_size_medium, self.COLOR_WHITE, 1)

        # Posición
        cv2.putText(canvas, f"X: {self.telemetry.robot_x:.2f} m", (x + 10, y + 50),
                   self.font_small, self.font_size_small, self.COLOR_WHITE, 1)
        cv2.putText(canvas, f"Y: {self.telemetry.robot_y:.2f} m", (x + 150, y + 50),
                   self.font_small, self.font_size_small, self.COLOR_WHITE, 1)

        # Velocidad
        cv2.putText(canvas, f"V: {self.telemetry.velocity:.2f} m/s", (x + 10, y + 75),
                   self.font_small, self.font_size_small, self.COLOR_WHITE, 1)
        cv2.putText(canvas, f"Omega: {self.telemetry.omega:.2f}", (x + 150, y + 75),
                   self.font_small, self.font_size_small, self.COLOR_WHITE, 1)

        # Sendero
        sendero_color = self.COLOR_GREEN if self.telemetry.on_sendero else self.COLOR_RED
        sendero_text = f"DETECTADO ({self.telemetry.sendero_confidence:.0%})" if self.telemetry.on_sendero else "BUSCANDO"
        cv2.putText(canvas, f"Sendero: {sendero_text}", (x + 10, y + 100),
                   self.font_small, self.font_size_small, sendero_color, 1)

        # Path angle
        if self.telemetry.path_center_x >= 0:
            cv2.putText(canvas, f"Path Angle: {self.telemetry.path_angle_deg:.1f}°", (x + 10, y + 125),
                       self.font_small, self.font_size_small, self.COLOR_YELLOW, 1)

        # LIDAR
        cv2.putText(canvas, f"LIDAR: {self.telemetry.lidar_distance:.2f} m", (x + 10, y + 150),
                   self.font_small, self.font_size_small, self.COLOR_RED, 1)

        # Zoom
        cv2.putText(canvas, f"Zoom: {self.telemetry.zoom_level}x", (x + 150, y + 150),
                   self.font_small, self.font_size_small, self.COLOR_YELLOW, 1)

    def _render_lidar_panel(self, canvas: np.ndarray):
        """Panel con visualización LIDAR."""
        x, y = self.width - 300, 410
        size = 150

        # Overlay
        overlay = canvas[y:y+size+30, x:x+290].copy()
        overlay = cv2.addWeighted(overlay, 0.3, np.zeros_like(overlay), 0, 0)
        canvas[y:y+size+30, x:x+290] = overlay

        # Título
        cv2.putText(canvas, "LIDAR SCAN", (x + 10, y + 20),
                   self.font_small, self.font_size_medium, self.COLOR_RED, 1)

        # Dibuja círculos LIDAR
        center_x = x + 150
        center_y = y + 90

        # Rango (100m → 150px)
        scale = 150 / 4.0  # 4m es el máximo en sim

        if self.telemetry.lidar_scan is not None:
            # Dibuja puntos LIDAR
            for i, dist in enumerate(self.telemetry.lidar_scan):
                if dist < 4.0:
                    angle = (i / len(self.telemetry.lidar_scan)) * np.pi
                    px = int(center_x + dist * scale * np.cos(angle - np.pi/2))
                    py = int(center_y + dist * scale * np.sin(angle - np.pi/2))

                    if 0 <= px < self.width and 0 <= py < self.height:
                        cv2.circle(canvas, (px, py), 2, self.COLOR_RED, -1)

        # Dibuja robot
        cv2.circle(canvas, (center_x, center_y), 5, self.COLOR_YELLOW, -1)

    def _render_graphs(self, canvas: np.ndarray):
        """Renderiza gráficos históricos."""
        x, y = self.width - 300, 590
        width = 290
        height = 130

        if len(self.history_ndvi) < 2:
            return

        # Overlay
        overlay = canvas[y:y+height, x:x+width].copy()
        overlay = cv2.addWeighted(overlay, 0.3, np.zeros_like(overlay), 0, 0)
        canvas[y:y+height, x:x+width] = overlay

        # Título
        cv2.putText(canvas, "TRENDS", (x + 10, y + 20),
                   self.font_small, self.font_size_small, self.COLOR_WHITE, 1)

        # Gráfico NDVI
        points = []
        for i, ndvi in enumerate(self.history_ndvi):
            px = x + 10 + int((i / len(self.history_ndvi)) * (width - 20))
            py = y + 50 - int(ndvi * 50)
            points.append((px, py))

        if len(points) > 1:
            for i in range(len(points) - 1):
                cv2.line(canvas, points[i], points[i + 1], self.COLOR_GREEN, 1)

        # Label
        cv2.putText(canvas, "NDVI", (x + 10, y + 65),
                   self.font_small, 0.4, self.COLOR_GREEN, 1)
        cv2.putText(canvas, f"{self.history_ndvi[-1]:.2f}", (x + 50, y + 65),
                   self.font_small, 0.4, self.COLOR_GREEN, 1)

        # Gráfico Chlorophyll
        points = []
        for i, chloro in enumerate(self.history_chlorophyll):
            px = x + 10 + int((i / len(self.history_chlorophyll)) * (width - 20))
            py = y + 110 - int((chloro / 80) * 50)
            points.append((px, py))

        if len(points) > 1:
            for i in range(len(points) - 1):
                cv2.line(canvas, points[i], points[i + 1], (0, 200, 0), 1)

        # Label
        cv2.putText(canvas, "Chlorophyll", (x + 10, y + 125),
                   self.font_small, 0.4, (0, 200, 0), 1)
