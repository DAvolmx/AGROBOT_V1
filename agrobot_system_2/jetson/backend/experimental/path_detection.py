"""
Path Detection Engine — Detección de Senderos en Cultivos
===========================================================

Detecta senderos (espacios entre plantas) usando:
  • Color HSV (rápido, para senderos verdes/marrones)
  • Contornos y líneas de regresión
  • Confidence scoring para fallback a LIDAR

Mejoras sobre v1:
  • Temporal smoothing → evita oscilaciones
  • Múltiples métodos (color, morphology, contour)
  • Confidence scoring robusto
"""

import cv2
import numpy as np
from typing import Optional, Dict, Tuple, List
from collections import deque
import threading


class PathDetector:
    """Detecta sendero (path) en frame actual."""

    def __init__(self,
                 sendero_h_range: Tuple[int, int] = (30, 90),
                 sendero_s_range: Tuple[int, int] = (60, 255),
                 sendero_v_range: Tuple[int, int] = (50, 255)):
        """
        Rango HSV para detectar sendero verde/marrón.

        Default: Verde (H:30-90, S:60-255, V:50-255)
        Para cultivos marrones: cambiar H a (10-40)
        """
        self.sendero_h_range = sendero_h_range
        self.sendero_s_range = sendero_s_range
        self.sendero_v_range = sendero_v_range

        # Histórico para temporal smoothing
        self.history_center_x = deque(maxlen=10)
        self.history_angle = deque(maxlen=10)
        self.history_confidence = deque(maxlen=10)

        # Parámetros
        self.min_contour_area = 500  # pixels
        self.max_contour_area = 500000
        self.line_fitting_points = 100

        self._lock = threading.Lock()

    def _extract_sendero_mask(self, frame: np.ndarray) -> np.ndarray:
        """Extrae máscara del sendero por color HSV."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Rango HSV para sendero
        lower = np.array([self.sendero_h_range[0],
                         self.sendero_s_range[0],
                         self.sendero_v_range[0]])
        upper = np.array([self.sendero_h_range[1],
                         self.sendero_s_range[1],
                         self.sendero_v_range[1]])

        mask = cv2.inRange(hsv, lower, upper)

        # Morphology: cierre (rellena huecos), apertura (reduce ruido)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        return mask

    def _fit_line_ransac(self, points: np.ndarray) -> Optional[Tuple]:
        """Ajusta línea a puntos usando RANSAC."""
        if len(points) < self.line_fitting_points:
            return None

        # cv2.fitLine retorna [vx, vy, x0, y0]
        # (vx, vy) = dirección unitaria
        # (x0, y0) = punto en la línea
        try:
            line = cv2.fitLine(points.astype(np.float32),
                              cv2.DIST_L2, 0, 0.01, 0.01)
            vx, vy, x0, y0 = line

            # Ángulo de la línea
            angle = np.arctan2(vy, vx)

            return (x0, y0, vx, vy, angle)
        except:
            return None

    def detect(self, frame: np.ndarray) -> Dict:
        """
        Detecta sendero en frame.

        Retorna:
        {
            "success": bool,
            "center_x": pixel X del centro del sendero,
            "center_y": pixel Y,
            "angle_deg": ángulo de avance (grados),
            "confidence": 0.0-1.0,
            "line": (x0, y0, dx, dy),
            "mask": máscara binaria del sendero
        }
        """
        h, w = frame.shape[:2]

        # 1. Extract mask
        mask = self._extract_sendero_mask(frame)

        # 2. Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # Filter by area
        valid_contours = [c for c in contours
                         if self.min_contour_area < cv2.contourArea(c)
                                                   < self.max_contour_area]

        if not valid_contours:
            return {
                "success": False,
                "reason": "no_contours",
                "confidence": 0.0,
                "mask": mask
            }

        # 3. Combine contours → puntos del sendero
        all_points = np.vstack([cv2.approxPolyDP(c, 0.5, False)
                               for c in valid_contours])
        all_points = all_points.reshape(-1, 2)

        if len(all_points) < self.line_fitting_points:
            return {
                "success": False,
                "reason": "too_few_points",
                "confidence": 0.0,
                "mask": mask
            }

        # 4. Fit line
        line_fit = self._fit_line_ransac(all_points)
        if line_fit is None:
            return {
                "success": False,
                "reason": "line_fit_failed",
                "confidence": 0.0,
                "mask": mask
            }

        x0, y0, vx, vy, angle = line_fit

        # 5. Center of sendero (línea intersecta con mitad del frame)
        # y = y0 + t * vy, cuando y = h/2
        if abs(vy) > 0.01:
            t = (h / 2.0 - y0) / vy
            center_x = x0 + t * vx
        else:
            center_x = x0

        center_x = np.clip(center_x, 0, w - 1)
        center_y = h / 2.0

        # 6. Confidence scoring
        contour_area = sum(cv2.contourArea(c) for c in valid_contours)
        area_confidence = min(1.0, contour_area / (w * h * 0.3))

        # Cantidad de puntos inliers
        inlier_dist = 15  # pixels
        dists = np.abs((all_points[:, 0] - x0) * vy -
                       (all_points[:, 1] - y0) * vx) / (np.sqrt(vx**2 + vy**2) + 1e-6)
        n_inliers = np.sum(dists < inlier_dist)
        inlier_confidence = n_inliers / len(all_points)

        # Confidence total
        confidence = 0.6 * area_confidence + 0.4 * inlier_confidence

        # 7. Temporal smoothing
        with self._lock:
            self.history_center_x.append(center_x)
            self.history_angle.append(np.degrees(angle))
            self.history_confidence.append(confidence)

            # Promedia últimos valores
            if len(self.history_center_x) > 3:
                center_x_smooth = np.median(self.history_center_x)
                angle_smooth = np.median(self.history_angle)
                confidence_smooth = np.mean(self.history_confidence)
            else:
                center_x_smooth = center_x
                angle_smooth = np.degrees(angle)
                confidence_smooth = confidence

        return {
            "success": True,
            "center_x": float(center_x_smooth),
            "center_y": float(center_y),
            "angle_deg": float(angle_smooth),
            "confidence": float(confidence_smooth),
            "line": (float(x0), float(y0), float(vx), float(vy)),
            "mask": mask,
            "n_contours": len(valid_contours),
            "n_inliers": int(n_inliers)
        }

    def visualize(self, frame: np.ndarray, detection: Dict) -> np.ndarray:
        """Dibuja resultado de detección en frame."""
        vis = frame.copy()
        h, w = vis.shape[:2]

        if not detection["success"]:
            # Dibuja "no path detected"
            cv2.putText(vis, "No path detected", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return vis

        # Centro del sendero
        cx = int(detection["center_x"])
        cy = int(detection["center_y"])
        cv2.circle(vis, (cx, cy), 10, (0, 255, 0), -1)

        # Línea del sendero
        if "line" in detection:
            x0, y0, vx, vy = detection["line"][:4]
            x0, y0 = int(x0), int(y0)

            # Extiende línea a través del frame
            t_min = -y0 / (vy + 1e-6)
            t_max = (h - y0) / (vy + 1e-6)

            pt1 = (int(x0 + t_min * vx), int(y0 + t_min * vy))
            pt2 = (int(x0 + t_max * vx), int(y0 + t_max * vy))

            cv2.line(vis, pt1, pt2, (0, 255, 0), 2)

        # Confianza
        conf = detection["confidence"]
        color = (0, 255, 0) if conf > 0.7 else (0, 165, 255) if conf > 0.5 else (0, 0, 255)
        cv2.putText(vis, f"Conf: {conf:.2f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # Ángulo
        angle = detection["angle_deg"]
        cv2.putText(vis, f"Angle: {angle:.1f}°", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        return vis


class PathDecision:
    """Toma decisión de navegación basada en detección de sendero."""

    def __init__(self, frame_width: int = 640):
        self.frame_width = frame_width
        self.center_x_target = frame_width / 2.0

        # PID para control de ángulo
        self.kp_angle = 0.5
        self.ki_angle = 0.1
        self.kd_angle = 0.2

        self.error_integral = 0.0
        self.error_previous = 0.0

    def compute_control(self, path_detection: Dict,
                       fallback_omega: float = 0.0) -> Tuple[float, float, float]:
        """
        Computa velocidades (v, omega) basada en detección.

        Retorna: (v, omega, confidence)
        """
        confidence = path_detection.get("confidence", 0.0)

        # Si confianza baja, usa fallback (LIDAR)
        if confidence < 0.5:
            return (0.3, fallback_omega, confidence)  # v lenta, omega del LIDAR

        # Control de ángulo: traer center_x al centro
        center_x = path_detection.get("center_x", self.frame_width / 2.0)
        error = center_x - self.center_x_target

        # PID
        self.error_integral += error
        self.error_integral = np.clip(self.error_integral, -100, 100)

        d_error = error - self.error_previous
        self.error_previous = error

        # Omega = control del ángulo (normaliza error a -1..+1)
        error_norm = error / (self.frame_width / 2.0)
        omega = (self.kp_angle * error_norm +
                self.ki_angle * (self.error_integral / self.frame_width) +
                self.kd_angle * d_error / self.frame_width)

        omega = np.clip(omega, -1.5, 1.5)

        # Velocidad adelante (más confianza = más rápido)
        v = 0.3 + 0.5 * confidence  # 0.3 a 0.8 m/s

        return (v, omega, confidence)
