"""
Logger de Navegación — Recolecta datos para retroalimentar mejoras
====================================================================

Captura:
  · Poses (posición + orientación) cada 0.5s
  · Lecturas de LIDAR (distancias por sector)
  · Detecciones de obstáculos (cámara + LIDAR)
  · Comandos de velocidad (deseados vs reales)
  · Eventos de patrulla (waypoints, escaneos, errores)

Los datos se guardan en CSV + JSON para análisis e ingesta en modelos.
"""

import asyncio
import csv
import json
import time
from pathlib import Path
from typing import Optional, Dict, Any
from datetime import datetime


class NavigationLogger:
    """Logger asincrónico de eventos de navegación."""

    def __init__(self, output_dir: Path = None):
        self.output_dir = output_dir or (Path.home() / ".agrobot" / "navigation_logs")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Archivos
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.poses_file = self.output_dir / f"poses_{self.session_id}.csv"
        self.events_file = self.output_dir / f"events_{self.session_id}.jsonl"
        self.lidar_file = self.output_dir / f"lidar_{self.session_id}.jsonl"
        self.obstacles_file = self.output_dir / f"obstacles_{self.session_id}.jsonl"

        # Inicializar CSV de poses
        self._init_poses_csv()

        # Buffer asincrónico
        self._buffer = asyncio.Queue()
        self._activo = False
        self._tarea_flush = None

    def _init_poses_csv(self):
        """Crea el archivo CSV con headers."""
        with self.poses_file.open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=[
                'timestamp', 'x', 'y', 'theta_deg',
                'vx_desired', 'omega_desired', 'vx_real', 'omega_real',
                'lidar_frontal_cm', 'lidar_lateral_izq_cm', 'lidar_lateral_der_cm'
            ])
            w.writeheader()

    async def iniciar(self):
        """Inicia el logger asincrónico."""
        if self._activo:
            return
        self._activo = True
        self._tarea_flush = asyncio.create_task(self._flush_periodico())
        print(f"[nav_logger] sesión {self.session_id} activa")

    def detener(self):
        """Detiene el logger y guarda datos pendientes."""
        self._activo = False
        if self._tarea_flush:
            self._tarea_flush.cancel()
        print(f"[nav_logger] sesión finalizada → {self.output_dir}")

    # ─────────────────────────────────────────────────────────────
    # Métodos para registrar eventos
    # ─────────────────────────────────────────────────────────────

    def log_pose(self, x: float, y: float, theta_deg: float,
                 vx_desired: float = 0.0, omega_desired: float = 0.0,
                 vx_real: float = 0.0, omega_real: float = 0.0,
                 lidar_frontal: float = None, lidar_izq: float = None,
                 lidar_der: float = None):
        """Registra pose actual + velocidades + LIDAR."""
        if not self._activo:
            return

        asyncio.create_task(self._enqueue({
            'tipo': 'pose',
            'timestamp': time.time(),
            'x': x, 'y': y, 'theta_deg': theta_deg,
            'vx_desired': vx_desired, 'omega_desired': omega_desired,
            'vx_real': vx_real, 'omega_real': omega_real,
            'lidar_frontal_cm': lidar_frontal,
            'lidar_lateral_izq_cm': lidar_izq,
            'lidar_lateral_der_cm': lidar_der,
        }))

    def log_lidar(self, frontal, f_izq, lateral_izq, lateral_der, t_izq, t_der, trasero):
        """Registra scan completo del LIDAR."""
        if not self._activo:
            return

        asyncio.create_task(self._enqueue({
            'tipo': 'lidar',
            'timestamp': time.time(),
            'frontal': frontal,
            'frontal_izq': f_izq,
            'lateral_izq': lateral_izq,
            'lateral_der': lateral_der,
            'trasero_izq': t_izq,
            'trasero_der': t_der,
            'trasero': trasero,
        }))

    def log_obstaculo(self, clase: str, confianza: float, posicion_x: float,
                      posicion_y: float, area_pixels: int, accion: str):
        """Registra detección de obstáculo (cámara/LIDAR) + acción tomada."""
        if not self._activo:
            return

        asyncio.create_task(self._enqueue({
            'tipo': 'obstaculo',
            'timestamp': time.time(),
            'clase': clase,
            'confianza': confianza,
            'posicion_x': posicion_x,
            'posicion_y': posicion_y,
            'area_pixels': area_pixels,
            'accion': accion,  # "parada", "evasion_lateral", "retroceso", etc
        }))

    def log_patrulla(self, evento: str, detalles: Dict[str, Any]):
        """Registra evento de patrulla."""
        if not self._activo:
            return

        data = {'tipo': 'patrulla', 'timestamp': time.time(), 'evento': evento}
        data.update(detalles)
        asyncio.create_task(self._enqueue(data))

    def log_evasion(self, estado_anterior: str, estado_nuevo: str, motivo: str):
        """Registra cambios de estado en evasión de obstáculos."""
        if not self._activo:
            return

        asyncio.create_task(self._enqueue({
            'tipo': 'evasion',
            'timestamp': time.time(),
            'estado_anterior': estado_anterior,
            'estado_nuevo': estado_nuevo,
            'motivo': motivo,
        }))

    # ─────────────────────────────────────────────────────────────
    # Internals
    # ─────────────────────────────────────────────────────────────

    async def _enqueue(self, data: Dict):
        """Encola dato en el buffer asincrónico."""
        try:
            await self._buffer.put(data)
        except:
            pass

    async def _flush_periodico(self):
        """Descarga el buffer cada 1s."""
        try:
            while self._activo:
                await asyncio.sleep(1.0)
                await self._flush()
        except asyncio.CancelledError:
            await self._flush()
            raise

    async def _flush(self):
        """Vacía el buffer en los archivos correspondientes."""
        items = []
        while not self._buffer.empty():
            try:
                items.append(self._buffer.get_nowait())
            except asyncio.QueueEmpty:
                break

        for item in items:
            tipo = item.pop('tipo')
            timestamp = item.pop('timestamp')

            if tipo == 'pose':
                self._write_pose(timestamp, item)
            elif tipo == 'lidar':
                self._write_jsonl(self.lidar_file, timestamp, item)
            elif tipo == 'obstaculo':
                self._write_jsonl(self.obstacles_file, timestamp, item)
            elif tipo in ('patrulla', 'evasion'):
                self._write_jsonl(self.events_file, timestamp, item)

    def _write_pose(self, timestamp: float, data: Dict):
        """Escribe en CSV de poses."""
        try:
            with self.poses_file.open('a', newline='') as f:
                w = csv.DictWriter(f, fieldnames=[
                    'timestamp', 'x', 'y', 'theta_deg',
                    'vx_desired', 'omega_desired', 'vx_real', 'omega_real',
                    'lidar_frontal_cm', 'lidar_lateral_izq_cm', 'lidar_lateral_der_cm'
                ])
                row = {'timestamp': timestamp}
                row.update(data)
                w.writerow(row)
        except Exception as e:
            print(f"[nav_logger] error escribiendo pose: {e}")

    def _write_jsonl(self, filepath: Path, timestamp: float, data: Dict):
        """Escribe en archivo JSONL."""
        try:
            line = json.dumps({'timestamp': timestamp, **data})
            with filepath.open('a') as f:
                f.write(line + '\n')
        except Exception as e:
            print(f"[nav_logger] error escribiendo {filepath.name}: {e}")

    def get_session_info(self) -> Dict[str, Any]:
        """Retorna info de la sesión actual."""
        return {
            'session_id': self.session_id,
            'output_dir': str(self.output_dir),
            'poses_file': str(self.poses_file),
            'events_file': str(self.events_file),
            'lidar_file': str(self.lidar_file),
            'obstacles_file': str(self.obstacles_file),
        }


# ════════════════════════════════════════════════════════════════════════════════
# ANÁLISIS DE DATOS
# ════════════════════════════════════════════════════════════════════════════════

def analizar_sesion(session_id: str, output_dir: Path = None) -> Dict[str, Any]:
    """Analiza una sesión guardada y retorna estadísticas."""
    output_dir = output_dir or (Path.home() / ".agrobot" / "navigation_logs")

    poses_file = output_dir / f"poses_{session_id}.csv"
    events_file = output_dir / f"events_{session_id}.jsonl"
    lidar_file = output_dir / f"lidar_{session_id}.jsonl"
    obstacles_file = output_dir / f"obstacles_{session_id}.jsonl"

    stats = {
        'session_id': session_id,
        'poses_count': 0,
        'eventos_count': 0,
        'obstaculos_count': 0,
        'distancia_recorrida_m': 0.0,
        'tiempo_total_s': 0.0,
        'velocidad_promedio_ms': 0.0,
        'evasiones_totales': 0,
        'eventos_patrulla': [],
    }

    # Leer poses
    poses = []
    if poses_file.exists():
        with poses_file.open('r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    poses.append({
                        'timestamp': float(row['timestamp']),
                        'x': float(row['x']),
                        'y': float(row['y']),
                    })
                except:
                    pass

    stats['poses_count'] = len(poses)

    # Calcular distancia recorrida
    if len(poses) > 1:
        distancia = 0.0
        for i in range(1, len(poses)):
            dx = poses[i]['x'] - poses[i-1]['x']
            dy = poses[i]['y'] - poses[i-1]['y']
            distancia += (dx**2 + dy**2) ** 0.5
        stats['distancia_recorrida_m'] = distancia
        stats['tiempo_total_s'] = poses[-1]['timestamp'] - poses[0]['timestamp']
        if stats['tiempo_total_s'] > 0:
            stats['velocidad_promedio_ms'] = distancia / stats['tiempo_total_s']

    # Leer eventos
    if events_file.exists():
        evasiones = 0
        patrullas = []
        with events_file.open('r') as f:
            for line in f:
                try:
                    data = json.loads(line)
                    if data.get('tipo') == 'evasion':
                        evasiones += 1
                    elif data.get('tipo') == 'patrulla':
                        patrullas.append(data.get('evento'))
                except:
                    pass
        stats['evasiones_totales'] = evasiones
        stats['eventos_patrulla'] = patrullas

    # Leer obstáculos
    if obstacles_file.exists():
        with obstacles_file.open('r') as f:
            stats['obstaculos_count'] = sum(1 for _ in f)

    return stats
