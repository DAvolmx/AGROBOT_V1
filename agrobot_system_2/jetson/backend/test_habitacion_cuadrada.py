"""
Pruebas de Navegación en Habitación Cuadrada
==============================================

Procotolo:
  1. Patrulla rectangular simple (4 waypoints en esquinas)
  2. Recolecta datos de navegación
  3. Ejecuta pruebas de evasión con obstáculos simulados
  4. Genera reporte de mejoras necesarias

Ejecución:
  $ python3 test_habitacion_cuadrada.py --lado 3.0 --velocidad 0.2
"""

import asyncio
import argparse
import json
from pathlib import Path
from dataclasses import dataclass
from typing import Callable, Optional

from navigation_logger import NavigationLogger, analizar_sesion


@dataclass
class ConfiguracionPrueba:
    """Configuración del test."""
    lado_m: float = 3.0           # lado del cuadrado
    velocidad_ms: float = 0.2     # velocidad lineal
    velocidad_giro_rad_s: float = 0.5  # velocidad angular
    waypoint_tol_m: float = 0.15  # tolerancia de llegada
    lidar_threshold_cm: float = 50.0  # distancia para detectar obstáculo


class RobotSimulado:
    """Simulador simplificado del robot para pruebas de escritorio."""

    def __init__(self, cfg: ConfiguracionPrueba):
        self.cfg = cfg
        self.pose = {'x': 0.0, 'y': 0.0, 'theta_deg': 0.0}
        self.velocidad = {'vx': 0.0, 'omega': 0.0}
        self.lidar = {
            'frontal': 300.0,
            'frontal_izq': 300.0,
            'lateral_izq': 300.0,
            'lateral_der': 300.0,
            'trasero_izq': 300.0,
            'trasero_der': 300.0,
            'trasero': 300.0,
        }

    def actualizar_pose(self, dt: float):
        """Actualiza pose según velocidades."""
        import math

        # Avance lineal
        vx = self.velocidad['vx']
        omega = self.velocidad['omega']

        if abs(omega) > 0.001:
            # Movimiento circular
            radio = vx / omega if abs(omega) > 0.001 else 0.0
            theta_rad = math.radians(self.pose['theta_deg'])
            vx_local = vx * math.cos(theta_rad)
            vy_local = vx * math.sin(theta_rad)
            self.pose['x'] += vx_local * dt
            self.pose['y'] += vy_local * dt
        else:
            # Movimiento recto
            theta_rad = math.radians(self.pose['theta_deg'])
            self.pose['x'] += vx * math.cos(theta_rad) * dt
            self.pose['y'] += vx * math.sin(theta_rad) * dt

        # Giro
        self.pose['theta_deg'] += omega * dt * (180.0 / 3.14159)
        while self.pose['theta_deg'] > 180:
            self.pose['theta_deg'] -= 360
        while self.pose['theta_deg'] < -180:
            self.pose['theta_deg'] += 360

    def simular_lidar(self, obstaculos: list):
        """Simula mediciones de LIDAR dado obstáculos en el mapa."""
        import math

        # Reset a máxima distancia
        for k in self.lidar:
            self.lidar[k] = 300.0

        # Calcular distancias a cada obstáculo
        for obs in obstaculos:
            dx = obs['x'] - self.pose['x']
            dy = obs['y'] - self.pose['y']

            # Ángulo relativo al robot
            ang_obs = math.atan2(dy, dx) * 180.0 / 3.14159
            ang_robot_deg = self.pose['theta_deg']
            ang_relativo = ang_obs - ang_robot_deg

            # Normalizar a [-180, 180]
            while ang_relativo > 180:
                ang_relativo -= 360
            while ang_relativo < -180:
                ang_relativo += 360

            # Distancia
            dist_cm = (dx**2 + dy**2) ** 0.5 * 100

            # Asignar al sector
            if -30 <= ang_relativo <= 30:
                self.lidar['frontal'] = min(self.lidar['frontal'], dist_cm)
            elif -90 < ang_relativo < -30:
                self.lidar['frontal_izq'] = min(self.lidar['frontal_izq'], dist_cm)
            elif -120 < ang_relativo <= -60:
                self.lidar['lateral_izq'] = min(self.lidar['lateral_izq'], dist_cm)
            elif 60 <= ang_relativo < 120:
                self.lidar['lateral_der'] = min(self.lidar['lateral_der'], dist_cm)


class PruebaHabitacionCuadrada:
    """Controlador de la prueba."""

    def __init__(self, cfg: ConfiguracionPrueba):
        self.cfg = cfg
        self.robot = RobotSimulado(cfg)
        self.logger = NavigationLogger()

        # Generar waypoints en esquinas
        lado = cfg.lado_m
        self.waypoints = [
            {'x': 0.0, 'y': 0.0, 'theta_deg': 0.0},
            {'x': lado, 'y': 0.0, 'theta_deg': 90.0},
            {'x': lado, 'y': lado, 'theta_deg': 180.0},
            {'x': 0.0, 'y': lado, 'theta_deg': 270.0},
        ]

        # Obstáculos en el ambiente (para simular)
        self.obstaculos = [
            {'x': cfg.lado_m / 2, 'y': cfg.lado_m / 2, 'tipo': 'central'},  # obstáculo central
        ]

    async def ejecutar(self):
        """Ejecuta la prueba completa."""
        await self.logger.iniciar()
        print(f"\n[PRUEBA] Iniciando en habitación {self.cfg.lado_m}×{self.cfg.lado_m}m")
        print(f"Waypoints: {self.waypoints}")

        try:
            # Recorre cada waypoint
            for i, wp in enumerate(self.waypoints):
                print(f"\n[WP{i}] Yendo a {wp['x']:.2f}, {wp['y']:.2f}")
                await self._navegar_a(wp)

                # Pausa para "barrido"
                print(f"[WP{i}] Realizando escaneo...")
                await asyncio.sleep(2.0)

        finally:
            self.logger.detener()
            await self._mostrar_resumen()

    async def _navegar_a(self, wp: dict):
        """Navega a un waypoint."""
        import math

        # Control simple: girar hacia el waypoint, luego avanzar
        timeout = 60.0  # máximo tiempo
        inicio = asyncio.get_event_loop().time()

        while asyncio.get_event_loop().time() - inicio < timeout:
            dx = wp['x'] - self.robot.pose['x']
            dy = wp['y'] - self.robot.pose['y']
            dist = (dx**2 + dy**2) ** 0.5

            if dist < self.cfg.waypoint_tol_m:
                print(f"  ✓ Llegado")
                self.robot.velocidad = {'vx': 0.0, 'omega': 0.0}
                return

            # Ángulo hacia el waypoint
            ang_deseado = math.atan2(dy, dx) * 180.0 / 3.14159
            ang_error = ang_deseado - self.robot.pose['theta_deg']

            # Normalizar
            while ang_error > 180:
                ang_error -= 360
            while ang_error < -180:
                ang_error += 360

            # Control proporcional
            if abs(ang_error) > 5:
                # Girar
                self.robot.velocidad['omega'] = 0.5 if ang_error > 0 else -0.5
                self.robot.velocidad['vx'] = 0.05
            else:
                # Avanzar
                self.robot.velocidad['vx'] = self.cfg.velocidad_ms
                self.robot.velocidad['omega'] = ang_error * 0.1

            # Simular LIDAR y verificar obstáculos
            self.robot.simular_lidar(self.obstaculos)

            # Lógica de evasión simple
            dist_frontal = self.robot.lidar['frontal']
            if dist_frontal < self.cfg.lidar_threshold_cm:
                print(f"  ! Obstáculo a {dist_frontal:.0f}cm, evadiendo...")
                self.robot.velocidad['vx'] = 0.1
                self.robot.velocidad['omega'] = 0.3
                self.logger.log_evasion('nav_normal', 'evasion', f'obstáculo frontal {dist_frontal:.0f}cm')

            # Log de pose
            self.logger.log_pose(
                self.robot.pose['x'], self.robot.pose['y'], self.robot.pose['theta_deg'],
                vx_desired=self.robot.velocidad['vx'],
                omega_desired=self.robot.velocidad['omega'],
                lidar_frontal=dist_frontal,
                lidar_izq=self.robot.lidar['lateral_izq'],
                lidar_der=self.robot.lidar['lateral_der'],
            )

            # Actualizar pose
            dt = 0.1
            self.robot.actualizar_pose(dt)

            await asyncio.sleep(dt)

    async def _mostrar_resumen(self):
        """Analiza y muestra resultados."""
        print("\n" + "="*60)
        print("RESUMEN DE PRUEBA")
        print("="*60)

        stats = analizar_sesion(self.logger.session_id)

        print(f"Sesión: {stats['session_id']}")
        print(f"Distancia recorrida: {stats['distancia_recorrida_m']:.2f} m")
        print(f"Tiempo total: {stats['tiempo_total_s']:.1f} s")
        print(f"Velocidad promedio: {stats['velocidad_promedio_ms']:.3f} m/s")
        print(f"Poses registradas: {stats['poses_count']}")
        print(f"Obstáculos detectados: {stats['obstaculos_count']}")
        print(f"Evasiones totales: {stats['evasiones_totales']}")

        print(f"\nDatos guardados en: {self.logger.output_dir}")
        print(f"  - poses_{self.logger.session_id}.csv")
        print(f"  - events_{self.logger.session_id}.jsonl")
        print(f"  - lidar_{self.logger.session_id}.jsonl")
        print(f"  - obstacles_{self.logger.session_id}.jsonl")


async def main():
    parser = argparse.ArgumentParser(description='Prueba en habitación cuadrada')
    parser.add_argument('--lado', type=float, default=3.0, help='Lado del cuadrado en metros')
    parser.add_argument('--velocidad', type=float, default=0.2, help='Velocidad lineal en m/s')
    parser.add_argument('--lidar-threshold', type=float, default=50.0, help='Distancia de evasión en cm')

    args = parser.parse_args()

    cfg = ConfiguracionPrueba(
        lado_m=args.lado,
        velocidad_ms=args.velocidad,
        lidar_threshold_cm=args.lidar_threshold,
    )

    prueba = PruebaHabitacionCuadrada(cfg)
    await prueba.ejecutar()


if __name__ == '__main__':
    asyncio.run(main())
