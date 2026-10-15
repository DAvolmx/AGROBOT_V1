"""
Vision System v3 — Orquestador integrado
========================================

Integra en un solo módulo:
  • SLAM v2 (visual + loop closure)
  • Camera Zoom (adaptativo 1x/2x/4x)
  • Path Detection (sendero robusto)
  • Semantic Costmap (fusión LIDAR + Vision)

Se llama desde jetson_server.py para reemplazar detección anterior.
"""

import asyncio
import time
import cv2
import numpy as np
from typing import Dict, Optional, Tuple
from pathlib import Path
import threading

try:
    from slam_v2 import SLAMv2
except ImportError:
    SLAMv2 = None

try:
    from camera_zoom import CameraZoomControl, MultiScaleYOLO
except ImportError:
    CameraZoomControl = None
    MultiScaleYOLO = None

try:
    from path_detection import PathDetector, PathDecision
except ImportError:
    PathDetector = None


class VisionSystemV3:
    """Sistema de visión integrado v3."""

    def __init__(self, camera_matrix: np.ndarray, config: Dict):
        self.config = config
        self.camera_matrix = camera_matrix

        # SLAM v2
        self.slam = None
        if SLAMv2:
            self.slam = SLAMv2(
                camera_matrix,
                keyframe_interval=config.get("slam_v2", {}).get("keyframe_interval", 10),
                loop_closure_threshold=config.get("slam_v2", {}).get("loop_closure_threshold", 0.65)
            )
            print("[VisionV3] SLAM v2 inicializado")
        else:
            print("[VisionV3] ⚠️  SLAM v2 no disponible")

        # Zoom de cámara
        self.zoom = None
        if CameraZoomControl:
            self.zoom = CameraZoomControl(
                camera_device="/dev/video0",
                zoom_strategy=config.get("camera_zoom", {}).get("zoom_strategy", "adaptive")
            )
            print("[VisionV3] Camera zoom inicializado")
        else:
            print("[VisionV3] ⚠️  Camera zoom no disponible")

        # YOLO Multi-scale
        self.yolo_multi = None
        if MultiScaleYOLO:
            model_path = config.get("ai", {}).get("active_model", "ip102_pest_yolo11s.pt")
            try:
                self.yolo_multi = MultiScaleYOLO(model_path)
                print("[VisionV3] YOLO multi-scale inicializado")
            except Exception as e:
                print(f"[VisionV3] ⚠️  YOLO error: {e}")
        else:
            print("[VisionV3] ⚠️  YOLO multi-scale no disponible")

        # Path detection
        self.path_detector = None
        if PathDetector:
            pd_config = config.get("path_detection", {})
            self.path_detector = PathDetector(
                sendero_h_range=(pd_config.get("sendero_h_min", 10),
                                pd_config.get("sendero_h_max", 45)),
                sendero_s_range=(pd_config.get("sendero_s_min", 20),
                                pd_config.get("sendero_s_max", 90)),
                sendero_v_range=(pd_config.get("sendero_v_min", 30),
                                pd_config.get("sendero_v_max", 150))
            )
            print("[VisionV3] Path detector inicializado")
        else:
            print("[VisionV3] ⚠️  Path detector no disponible")

        # Path decision
        self.path_decision = None
        if PathDecision:
            self.path_decision = PathDecision(frame_width=640)

        # Estado
        self.last_frame = None
        self.last_slam_result = None
        self.last_path_result = None
        self.last_yolo_result = None
        self.fps = 0.0
        self._last_fps_update = time.time()
        self._frame_count = 0

    async def process_frame(self, frame: np.ndarray,
                           gyro_z_deg_s: float = 0.0,
                           lidar_omega: float = 0.0) -> Dict:
        """Procesa frame con todo el sistema v3."""
        t0 = time.time()

        self.last_frame = frame.copy()
        self._frame_count += 1

        results = {
            "success": True,
            "timestamp": t0,
            "slam": None,
            "zoom": None,
            "path": None,
            "detections": None,
            "control": None,
            "fps": self.fps
        }

        # 1. SLAM visual
        if self.slam:
            try:
                slam_res = self.slam.process_frame(frame, gyro_z_deg_s)
                results["slam"] = slam_res
                self.last_slam_result = slam_res
            except Exception as e:
                print(f"[VisionV3] SLAM error: {e}")

        # 2. Zoom adaptativo
        if self.zoom:
            try:
                self.zoom.update_zoom()
                results["zoom"] = self.zoom.get_zoom_info()
            except Exception as e:
                print(f"[VisionV3] Zoom error: {e}")

        # 3. Path detection
        if self.path_detector:
            try:
                path_res = self.path_detector.detect(frame)
                results["path"] = path_res
                self.last_path_result = path_res
            except Exception as e:
                print(f"[VisionV3] Path detection error: {e}")

        # 4. YOLO Multi-scale
        if self.yolo_multi and self.zoom:
            try:
                yolo_res = self.yolo_multi.infer_multi_scale(frame, self.zoom)
                results["detections"] = yolo_res
                self.last_yolo_result = yolo_res
            except Exception as e:
                print(f"[VisionV3] YOLO error: {e}")

        # 5. Control decision
        if self.path_decision and self.last_path_result:
            try:
                v, omega, confidence = self.path_decision.compute_control(
                    self.last_path_result,
                    fallback_omega=lidar_omega
                )
                results["control"] = {
                    "v": v,
                    "omega": omega,
                    "confidence": confidence,
                    "source": "path_detection" if confidence > 0.5 else "lidar_fallback"
                }
            except Exception as e:
                print(f"[VisionV3] Control decision error: {e}")

        # FPS
        if self._frame_count % 30 == 0:
            now = time.time()
            dt = now - self._last_fps_update
            self.fps = 30.0 / dt if dt > 0 else 0
            self._last_fps_update = now

        elapsed = time.time() - t0
        if elapsed > 0.1:
            print(f"[VisionV3] Lento: {elapsed*1000:.1f}ms "
                  f"SLAM={results['slam'] is not None} "
                  f"Path={results['path'] is not None} "
                  f"YOLO={results['detections'] is not None}")

        return results

    def get_status(self) -> Dict:
        """Retorna estado del sistema."""
        return {
            "slam": {
                "enabled": self.slam is not None,
                "tracking": self.slam.is_tracking if self.slam else False,
                "n_keyframes": len(self.slam.keyframes) if self.slam else 0
            },
            "zoom": {
                "enabled": self.zoom is not None,
                "current": self.zoom.current_zoom if self.zoom else 1.0,
                "mode": self.zoom.zoom_strategy if self.zoom else None
            },
            "path": {
                "enabled": self.path_detector is not None,
                "last_confidence": self.last_path_result.get("confidence", 0) if self.last_path_result else 0
            },
            "yolo": {
                "enabled": self.yolo_multi is not None,
                "fps": self.fps
            }
        }

    def save_slam_map(self, path: str):
        """Guarda mapa SLAM."""
        if self.slam:
            self.slam.save_map(path)
            print(f"[VisionV3] Mapa SLAM guardado en {path}")

    def load_slam_map(self, path: str) -> bool:
        """Carga mapa SLAM previo."""
        if self.slam:
            return self.slam.load_map(path)
        return False


class SemanticCostmap:
    """Fusiona costmap LIDAR + información semántica de visión."""

    def __init__(self, width: int = 256, height: int = 256,
                 resolution: float = 0.05):
        self.width = width
        self.height = height
        self.resolution = resolution  # m/cell

        # Static costmap (SLAM)
        self.static = np.zeros((height, width), dtype=np.uint8)

        # Dynamic costmap (LIDAR)
        self.dynamic = np.zeros((height, width), dtype=np.uint8)

        # Semantic costmap (Visión)
        self.semantic = np.zeros((height, width), dtype=np.uint8)
        # 0: libre, 50: sendero, 100: planta, 200: daño/peligro

        self._lock = threading.Lock()

    def update_semantic_from_path(self, path_detection: Dict):
        """Actualiza capa semántica desde detección de sendero."""
        if not path_detection.get("success"):
            return

        # Marca sendero en el centro
        center_x = int(path_detection.get("center_x", self.width / 2))
        center_y = int(self.height / 2)

        with self._lock:
            # Borra columna anterior
            self.semantic[:, :] = 0

            # Marca sendero (valor 50)
            if 0 <= center_x < self.width:
                width_margin = 30  # pixels
                x_min = max(0, center_x - width_margin)
                x_max = min(self.width, center_x + width_margin)
                self.semantic[:, x_min:x_max] = 50

    def update_dynamic_from_lidar(self, costmap_local):
        """Actualiza capa dinámica desde LIDAR."""
        if hasattr(costmap_local, 'grid') and costmap_local.grid is not None:
            with self._lock:
                # Redimensiona si es necesario
                if costmap_local.grid.shape == self.dynamic.shape:
                    self.dynamic = costmap_local.grid.astype(np.uint8)

    def get_combined(self) -> np.ndarray:
        """Retorna costmap combinado (fusión ponderada)."""
        with self._lock:
            # Pesa: 40% dinámico (LIDAR), 40% estático (SLAM), 20% semántico (Vision)
            combined = (
                0.4 * self.dynamic.astype(np.float32) +
                0.4 * self.static.astype(np.float32) +
                0.2 * self.semantic.astype(np.float32)
            ).astype(np.uint8)

        return combined
