"""
Simulador 3D Mejorado v3 — Con Análisis Multispectral
======================================================

Simula:
  • Cultivos 3D (lechuga, acelga, tomate)
  • Robot agrícola navegando
  • LIDAR en vivo (grid rojo)
  • Análisis multispectral (NDVI, Chlorophyll)
  • Detección de plagas
  • Path planning visual
  • Mapa SLAM en tiempo real

Usa: pygame + numpy + simulación física básica
"""

import pygame
import numpy as np
import math
from typing import List, Tuple, Dict
from dataclasses import dataclass
from enum import Enum


class PlantType(Enum):
    """Tipos de plantas disponibles."""
    LECHUGA = 1
    ACELGA = 2
    TOMATE = 3
    MALEZA = 4


@dataclass
class Plant:
    """Una planta en el simulador."""
    x: float
    y: float
    plant_type: PlantType
    health: float = 1.0  # 0-1
    ndvi: float = 0.7   # Normalized Difference Vegetation Index
    chlorophyll: float = 50  # mg/cm²
    pest_level: float = 0.0  # 0-1 plagas

    def get_color(self) -> Tuple[int, int, int]:
        """Color basado en salud y tipo."""
        if self.plant_type == PlantType.LECHUGA:
            base = (34, 139, 34)  # Verde oscuro
        elif self.plant_type == PlantType.ACELGA:
            base = (50, 150, 50)  # Verde medio
        elif self.plant_type == PlantType.TOMATE:
            base = (139, 69, 19)  # Marrón (no maduro)
        else:
            base = (100, 100, 50)  # Maleza

        # Reduce color si tiene plagas
        if self.pest_level > 0.3:
            base = (
                max(0, int(base[0] * (1 - self.pest_level))),
                max(100, int(base[1] * (1 - self.pest_level))),
                int(base[2] * (1 - self.pest_level))
            )

        # Palidez si no es saludable
        if self.health < 0.7:
            base = (
                int(base[0] * 0.8 + 100),
                int(base[1] * 0.8 + 80),
                int(base[2] * 0.8 + 60)
            )

        return base


@dataclass
class Robot:
    """El robot en la simulación."""
    x: float
    y: float
    theta: float  # Ángulo en radianes
    v: float = 0.0  # Velocidad lineal
    omega: float = 0.0  # Velocidad angular

    # Estado
    lidar_distance: float = 3.0
    on_sendero: bool = False
    detected_pests: List = None

    def __post_init__(self):
        if self.detected_pests is None:
            self.detected_pests = []

    def get_color(self) -> Tuple[int, int, int]:
        """Color del robot según estado."""
        if self.on_sendero:
            return (0, 255, 0)  # Verde = sendero detectado
        else:
            return (255, 0, 0)  # Rojo = buscando


@dataclass
class SLAMMap:
    """Mapa SLAM del robot."""
    keyframes: List[Tuple[float, float, float]] = None
    points_3d: List[Tuple[float, float, float]] = None
    loop_closures: List[Tuple[int, int]] = None

    def __post_init__(self):
        if self.keyframes is None:
            self.keyframes = []
        if self.points_3d is None:
            self.points_3d = []
        if self.loop_closures is None:
            self.loop_closures = []


class MultispectralAnalyzer:
    """Analiza datos multiespectrales de plantas."""

    @staticmethod
    def calculate_ndvi(plant: Plant) -> float:
        """
        NDVI = (NIR - Red) / (NIR + Red)
        Rango: -1 a 1
        >0.5 = vegetación sana
        """
        # Simulación basada en salud y plagas
        base_ndvi = 0.6 + (plant.health * 0.3)
        pest_reduction = plant.pest_level * 0.2
        return max(0, base_ndvi - pest_reduction)

    @staticmethod
    def calculate_chlorophyll(plant: Plant) -> float:
        """Contenido de clorofila (mg/cm²)."""
        base = 50 + (plant.health * 30)
        pest_reduction = plant.pest_level * 20
        return max(10, base - pest_reduction)

    @staticmethod
    def detect_pests(plant: Plant) -> List[str]:
        """Detecta plagas por NDVI y color."""
        pests = []

        if plant.ndvi < 0.5:
            pests.append("Mildiu")
        if plant.chlorophyll < 30:
            pests.append("Deficiencia Nutricional")
        if plant.pest_level > 0.5:
            pests.append("Mosca Blanca")

        return pests


class SimuladorV3:
    """Simulador 3D profesional con análisis."""

    def __init__(self, width: int = 1400, height: int = 900):
        pygame.init()

        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("AgroBot v3 — Simulador 3D Multispectral")

        self.clock = pygame.time.Clock()
        self.fps = 60
        self.running = True

        # Colores
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.SOIL = (139, 90, 43)
        self.SENDERO = (100, 70, 40)
        self.GRID_COLOR = (255, 0, 0)  # Rojo para LIDAR

        # Mundo
        self.field_width = 50  # metros
        self.field_height = 40  # metros
        self.pixels_per_meter = 15

        # Plantas
        self.plants: List[Plant] = []
        self._init_plants()

        # Robot
        self.robot = Robot(x=25, y=20, theta=0)
        self.path_waypoints: List[Tuple[float, float]] = []
        self.slam_map = SLAMMap()

        # UI
        self.font_small = pygame.font.Font(None, 24)
        self.font_medium = pygame.font.Font(None, 32)
        self.font_large = pygame.font.Font(None, 48)

        # Estadísticas
        self.stats = {
            "fps": 0,
            "avg_ndvi": 0.7,
            "avg_chlorophyll": 50,
            "pests_detected": 0,
            "robot_pos": (0, 0),
            "on_sendero": False
        }

        self.analyzer = MultispectralAnalyzer()

    def _init_plants(self):
        """Inicializa cultivo en el campo."""
        np.random.seed(42)

        # Plantación en filas
        row_spacing = 0.6
        col_spacing = 0.5

        for row in range(15):
            for col in range(20):
                x = 5 + col * col_spacing
                y = 5 + row * row_spacing

                # Tipo aleatorio
                plant_type = np.random.choice(
                    [PlantType.LECHUGA, PlantType.ACELGA],
                    p=[0.7, 0.3]
                )

                # Añade algunos con plagas
                pest_level = np.random.choice(
                    [0, 0, 0, 0.3, 0.6],
                    p=[0.6, 0.2, 0.1, 0.05, 0.05]
                )

                health = 1.0 - (pest_level * 0.5)

                plant = Plant(
                    x=x,
                    y=y,
                    plant_type=plant_type,
                    health=health,
                    pest_level=pest_level
                )

                # Calcula índices
                plant.ndvi = self.analyzer.calculate_ndvi(plant)
                plant.chlorophyll = self.analyzer.calculate_chlorophyll(plant)

                self.plants.append(plant)

    def _world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convierte coordenadas mundo a pantalla."""
        screen_x = int(x * self.pixels_per_meter)
        screen_y = int(y * self.pixels_per_meter)
        return screen_x, screen_y

    def _screen_to_world(self, sx: int, sy: int) -> Tuple[float, float]:
        """Convierte pantalla a mundo."""
        x = sx / self.pixels_per_meter
        y = sy / self.pixels_per_meter
        return x, y

    def update(self, dt: float):
        """Actualiza simulación."""
        # Entrada
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:
            self.robot.v = min(self.robot.v + 0.1, 1.5)
        elif keys[pygame.K_DOWN]:
            self.robot.v = max(self.robot.v - 0.1, -0.5)
        else:
            self.robot.v *= 0.9  # Inercia

        if keys[pygame.K_LEFT]:
            self.robot.omega = min(self.robot.omega + 0.5, 4.0)
        elif keys[pygame.K_RIGHT]:
            self.robot.omega = max(self.robot.omega - 0.5, -4.0)
        else:
            self.robot.omega *= 0.8  # Inercia

        # Física del robot
        self.robot.x += self.robot.v * math.cos(self.robot.theta) * dt
        self.robot.y += self.robot.v * math.sin(self.robot.theta) * dt
        self.robot.theta += self.robot.omega * dt

        # Límites del campo
        self.robot.x = max(1, min(self.robot.x, self.field_width - 1))
        self.robot.y = max(1, min(self.robot.y, self.field_height - 1))

        # Detecta sendero
        self.robot.on_sendero = False
        for plant in self.plants:
            dist = math.sqrt((self.robot.x - plant.x)**2 + (self.robot.y - plant.y)**2)
            if 0.3 < dist < 0.6:
                # En el espacio entre plantas = sendero
                self.robot.on_sendero = True
                break

        # SLAM - Añade keyframe cada 5 frames
        if len(self.slam_map.keyframes) == 0 or \
           math.sqrt((self.robot.x - self.slam_map.keyframes[-1][0])**2 + \
                     (self.robot.y - self.slam_map.keyframes[-1][1])**2) > 2.0:
            self.slam_map.keyframes.append((self.robot.x, self.robot.y, self.robot.theta))

        # Detección de LIDAR simulado
        self.robot.lidar_distance = self._simulate_lidar()

        # Actualiza estadísticas
        self._update_stats()

    def _simulate_lidar(self) -> float:
        """Simula LIDAR del robot."""
        # Raycast en dirección del robot
        for dist in np.linspace(0.1, 4.0, 40):
            x = self.robot.x + dist * math.cos(self.robot.theta)
            y = self.robot.y + dist * math.sin(self.robot.theta)

            # Chequea colisión con plantas
            for plant in self.plants:
                pdist = math.sqrt((x - plant.x)**2 + (y - plant.y)**2)
                if pdist < 0.15:  # Radio de planta
                    return dist

        return 4.0  # Sin obstáculos

    def _update_stats(self):
        """Actualiza estadísticas de la simulación."""
        ndvi_values = [p.ndvi for p in self.plants]
        chlorophyll_values = [p.chlorophyll for p in self.plants]
        pests = sum(1 for p in self.plants if p.pest_level > 0.3)

        self.stats["avg_ndvi"] = np.mean(ndvi_values)
        self.stats["avg_chlorophyll"] = np.mean(chlorophyll_values)
        self.stats["pests_detected"] = pests
        self.stats["robot_pos"] = (self.robot.x, self.robot.y)
        self.stats["on_sendero"] = self.robot.on_sendero
        self.stats["fps"] = int(self.clock.get_fps())

    def render(self):
        """Renderiza la escena."""
        # Limpia
        self.screen.fill(self.SOIL)

        # 1. Renderiza campo
        self._render_field()

        # 2. Renderiza plantas
        self._render_plants()

        # 3. Renderiza SLAM
        self._render_slam()

        # 4. Renderiza robot + LIDAR
        self._render_robot()

        # 5. Renderiza UI (overlays)
        self._render_ui()

        pygame.display.flip()

    def _render_field(self):
        """Renderiza el campo base."""
        field_rect = pygame.Rect(
            0, 0,
            self.field_width * self.pixels_per_meter,
            self.field_height * self.pixels_per_meter
        )
        pygame.draw.rect(self.screen, self.SOIL, field_rect, 2)

    def _render_plants(self):
        """Renderiza todas las plantas."""
        for plant in self.plants:
            sx, sy = self._world_to_screen(plant.x, plant.y)

            # Radio basado en tipo
            radius = 6 if plant.plant_type in [PlantType.LECHUGA, PlantType.ACELGA] else 8

            # Color
            color = plant.get_color()
            pygame.draw.circle(self.screen, color, (sx, sy), radius)

            # Borde si tiene plagas
            if plant.pest_level > 0.3:
                pygame.draw.circle(self.screen, (255, 200, 0), (sx, sy), radius, 2)

    def _render_slam(self):
        """Renderiza mapa SLAM."""
        # Keyframes
        for i, (x, y, theta) in enumerate(self.slam_map.keyframes):
            sx, sy = self._world_to_screen(x, y)
            pygame.draw.circle(self.screen, (100, 255, 100), (sx, sy), 3)

            # Línea de rumbo
            ex = sx + 10 * math.cos(theta)
            ey = sy + 10 * math.sin(theta)
            pygame.draw.line(self.screen, (100, 255, 100), (sx, sy), (ex, ey), 1)

        # Trayectoria
        for i in range(len(self.slam_map.keyframes) - 1):
            x1, y1, _ = self.slam_map.keyframes[i]
            x2, y2, _ = self.slam_map.keyframes[i + 1]
            sx1, sy1 = self._world_to_screen(x1, y1)
            sx2, sy2 = self._world_to_screen(x2, y2)
            pygame.draw.line(self.screen, (100, 200, 100), (sx1, sy1), (sx2, sy2), 1)

    def _render_robot(self):
        """Renderiza robot y LIDAR."""
        sx, sy = self._world_to_screen(self.robot.x, self.robot.y)

        # Robot
        color = self.robot.get_color()
        pygame.draw.circle(self.screen, color, (sx, sy), 12)

        # Indicador de frente
        front_x = sx + 15 * math.cos(self.robot.theta)
        front_y = sy + 15 * math.sin(self.robot.theta)
        pygame.draw.line(self.screen, (255, 255, 0), (sx, sy), (front_x, front_y), 3)

        # LIDAR grid (rojo)
        for angle in np.linspace(-math.pi/2, math.pi/2, 30):
            abs_angle = self.robot.theta + angle
            dist = self.robot.lidar_distance

            lx = sx + dist * self.pixels_per_meter * math.cos(abs_angle)
            ly = sy + dist * self.pixels_per_meter * math.sin(abs_angle)

            pygame.draw.line(self.screen, self.GRID_COLOR, (sx, sy), (lx, ly), 1)

    def _render_ui(self):
        """Renderiza overlays de UI."""
        # Panel superior derecho - NDVI + Chlorophyll
        self._render_spectral_panel()

        # Panel inferior - Info del robot
        self._render_robot_panel()

        # Panel izquierdo - Mapa pequeño
        self._render_minimap()

    def _render_spectral_panel(self):
        """Renderiza gráficos espectrales."""
        x, y = self.width - 250, 20

        # Fondo semi-transparente
        panel = pygame.Surface((230, 200))
        panel.set_alpha(200)
        panel.fill((0, 0, 0))
        self.screen.blit(panel, (x, y))

        # Título
        title = self.font_medium.render("MULTISPECTRAL SCAN", True, (0, 255, 0))
        self.screen.blit(title, (x + 10, y + 5))

        # NDVI
        ndvi_text = self.font_small.render(f"NDVI: {self.stats['avg_ndvi']:.2f}", True, (0, 255, 0))
        self.screen.blit(ndvi_text, (x + 10, y + 40))

        # Barra NDVI
        bar_width = 200
        bar_height = 15
        ndvi_bar = int(self.stats['avg_ndvi'] * bar_width)
        pygame.draw.rect(self.screen, (100, 100, 100), (x + 10, y + 60, bar_width, bar_height))
        pygame.draw.rect(self.screen, (0, 255, 0), (x + 10, y + 60, ndvi_bar, bar_height))

        # Chlorophyll Index
        chloro_text = self.font_small.render(f"Chlorophyll: {self.stats['avg_chlorophyll']:.1f}", True, (0, 255, 0))
        self.screen.blit(chloro_text, (x + 10, y + 85))

        # Barra Chlorophyll
        chloro_bar = int((self.stats['avg_chlorophyll'] / 80) * bar_width)
        pygame.draw.rect(self.screen, (100, 100, 100), (x + 10, y + 105, bar_width, bar_height))
        pygame.draw.rect(self.screen, (0, 200, 0), (x + 10, y + 105, chloro_bar, bar_height))

        # Plagas detectadas
        pests_text = self.font_small.render(f"Pests: {self.stats['pests_detected']}", True, (255, 100, 0))
        self.screen.blit(pests_text, (x + 10, y + 130))

        # FPS
        fps_text = self.font_small.render(f"FPS: {self.stats['fps']}", True, (200, 200, 200))
        self.screen.blit(fps_text, (x + 10, y + 155))

    def _render_robot_panel(self):
        """Panel inferior con info del robot."""
        x, y = 20, self.height - 120

        panel = pygame.Surface((400, 100))
        panel.set_alpha(200)
        panel.fill((0, 0, 0))
        self.screen.blit(panel, (x, y))

        # Posición
        pos_text = self.font_small.render(
            f"Position: ({self.stats['robot_pos'][0]:.1f}, {self.stats['robot_pos'][1]:.1f})",
            True, (255, 255, 255)
        )
        self.screen.blit(pos_text, (x + 10, y + 5))

        # Velocidad
        vel_text = self.font_small.render(
            f"Velocity: {self.robot.v:.2f} m/s | Omega: {self.robot.omega:.2f} rad/s",
            True, (255, 255, 255)
        )
        self.screen.blit(vel_text, (x + 10, y + 30))

        # Sendero
        sendero_color = (0, 255, 0) if self.stats["on_sendero"] else (255, 0, 0)
        sendero_text = self.font_small.render(
            f"Sendero: {'DETECTADO' if self.stats['on_sendero'] else 'BUSCANDO'}",
            True, sendero_color
        )
        self.screen.blit(sendero_text, (x + 10, y + 55))

        # LIDAR
        lidar_text = self.font_small.render(
            f"LIDAR Distance: {self.robot.lidar_distance:.2f} m",
            True, (255, 0, 0)
        )
        self.screen.blit(lidar_text, (x + 200, y + 55))

    def _render_minimap(self):
        """Mapa pequeño en esquina."""
        minimap_size = 150
        x, y = 20, 20

        panel = pygame.Surface((minimap_size, minimap_size))
        panel.fill(self.SOIL)

        # Escala
        scale = minimap_size / max(self.field_width, self.field_height)

        # Plantas en minimap
        for plant in self.plants[:30]:  # Solo algunas
            mx = int(plant.x * scale)
            my = int(plant.y * scale)
            color = (0, 100, 0) if plant.plant_type in [PlantType.LECHUGA, PlantType.ACELGA] else (100, 0, 0)
            pygame.draw.circle(panel, color, (mx, my), 2)

        # Robot en minimap
        rx = int(self.robot.x * scale)
        ry = int(self.robot.y * scale)
        pygame.draw.circle(panel, (255, 255, 0), (rx, ry), 3)

        self.screen.blit(panel, (x, y))
        pygame.draw.rect(self.screen, (200, 200, 200), (x, y, minimap_size, minimap_size), 2)

    def run(self):
        """Loop principal."""
        while self.running:
            dt = self.clock.tick(self.fps) / 1000.0

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
                    elif event.key == pygame.K_r:
                        self.plants = []
                        self._init_plants()
                        self.slam_map = SLAMMap()

            self.update(dt)
            self.render()

        pygame.quit()


if __name__ == "__main__":
    sim = SimuladorV3(width=1400, height=900)
    sim.run()
