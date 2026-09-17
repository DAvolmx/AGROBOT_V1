"""
Sistema de Evasión de Obstáculos con LIDAR y Cámara — AgroBot
===============================================================

Módulo que implementa lógica de evasión reactiva basada en:
  · Datos del LIDAR 2D (scans de 360°, análisis por sectores)
  · Detección de obstáculos por cámara (YOLO)
  · Navegación integrada con Nav2 cuando disponible

Flujo de decisión:
  1. Procesar scan del LIDAR → extraer distancias por sector (frontal, laterales, trasero)
  2. Analizar detecciones YOLO para obstáculos en cámara
  3. Si hay riesgo inminente (< 20cm) → detener al instante
  4. Si hay obstáculo parcial (20-50cm) → evasión reactiva (girar, retroceder)
  5. Retomar ruta de navegación cuando camino se despeje

Parámetros configurables:
  DIST_CRITICA_PARADA: distancia donde se detiene de inmediato (cm, ej: 15-20)
  DIST_PRECAUCIÓN: distancia donde se prepara evasión (cm, ej: 40-60)
  VELOCIDAD_EVASION: velocidad durante maniobras de evasión (m/s, ej: 0.1-0.15)
"""
import asyncio
import time
import math
from dataclasses import dataclass
from typing import Optional, Callable, Dict, List
from enum import Enum


class EstadoEvasion(Enum):
    """Estados de la máquina de evasión de obstáculos."""
    NAVEGACION_NORMAL = "nav_normal"
    ALERTA_PROXIMIDAD = "alerta"
    PARADA_CRITICA = "stop"
    EVASION_LATERAL = "evasion_lateral"
    EVASION_RETROCESO = "evasion_retro"
    RECUPERACION = "recuperacion"


class SectorLIDAR(Enum):
    """Sector del LIDAR (barrido de 360°)."""
    FRONTAL = "frontal"          # 0° ± 30° (frente)
    FRONTAL_IZQUIERDA = "f_izq"  # -30° a -90° (frente-izq)
    LATERAL_IZQUIERDO = "lat_izq" # -90° ± 30° (izquierda)
    LATERAL_DERECHO = "lat_der"   # +90° ± 30° (derecha)
    TRASERO_IZQUIERDA = "t_izq"  # +120° a +150° (atrás-izq)
    TRASERO_DERECHA = "t_der"    # -120° a -150° (atrás-der)
    TRASERO = "trasero"          # ±180° (atrás)


@dataclass
class LecturasSensoresLIDAR:
    """
    Lectura instantánea de distancias LIDAR procesadas por sector.

    Se obtienen del scan 360° del LIDAR, agrupando mediciones por sector
    angular e ignorando lecturas inválidas/ruidosas.
    """
    frontal: float              # distancia frontal en cm (0° ± 30°)
    frontal_izquierda: float    # frente-izquierda en cm (-30° a -90°)
    lateral_izquierdo: float    # izquierda pura en cm (-90° ± 30°)
    lateral_derecho: float      # derecha pura en cm (+90° ± 30°)
    trasero_izquierda: float    # atrás-izquierda en cm (+120° a +150°)
    trasero_derecha: float      # atrás-derecha en cm (-120° a -150°)
    trasero: float              # atrás puro en cm (±180°)
    timestamp: float = 0.0      # momento de la lectura del LIDAR

    def distancia_minima(self) -> float:
        """Retorna la distancia mínima de todos los sectores."""
        return min(self.frontal, self.frontal_izquierda, self.lateral_izquierdo,
                   self.lateral_derecho, self.trasero_izquierda, self.trasero_derecha,
                   self.trasero)

    def sector_critico(self) -> Optional[SectorLIDAR]:
        """Identifica qué sector LIDAR disparó la alarma crítica."""
        critico = self.distancia_minima()
        if self.frontal == critico:
            return SectorLIDAR.FRONTAL
        elif self.frontal_izquierda == critico:
            return SectorLIDAR.FRONTAL_IZQUIERDA
        elif self.lateral_izquierdo == critico:
            return SectorLIDAR.LATERAL_IZQUIERDO
        elif self.lateral_derecho == critico:
            return SectorLIDAR.LATERAL_DERECHO
        elif self.trasero_izquierda == critico:
            return SectorLIDAR.TRASERO_IZQUIERDA
        elif self.trasero_derecha == critico:
            return SectorLIDAR.TRASERO_DERECHA
        elif self.trasero == critico:
            return SectorLIDAR.TRASERO
        return None

    @staticmethod
    def desde_scan_lidar(ranges: list, angles: list, timestamp: float = 0.0) -> "LecturasSensoresLIDAR":
        """
        Procesa un scan del LIDAR (360°) y extrae distancias por sector.

        Args:
            ranges: lista de distancias en metros
            angles: lista de ángulos en radianes (0° = frente, +90° = izq, -90° = der)
            timestamp: momento del scan

        Returns:
            LecturasSensoresLIDAR con distancias procesadas (en cm)
        """
        import math

        # Inicializar sectores con valores altos (nada detectado)
        sectores = {s: 300.0 for s in SectorLIDAR}

        # Procesar cada medición del LIDAR
        for rango, angulo in zip(ranges, angles):
            if rango <= 0 or rango > 10:  # ignorar inválidos y muy lejanos
                continue

            distancia_cm = rango * 100
            angulo_deg = math.degrees(angulo)

            # Normalizar ángulo a [-180, 180]
            while angulo_deg > 180:
                angulo_deg -= 360
            while angulo_deg < -180:
                angulo_deg += 360

            # Asignar al sector más cercano
            if -30 <= angulo_deg <= 30:
                sectores[SectorLIDAR.FRONTAL] = min(sectores[SectorLIDAR.FRONTAL], distancia_cm)
            elif -90 < angulo_deg < -30:
                sectores[SectorLIDAR.FRONTAL_IZQUIERDA] = min(sectores[SectorLIDAR.FRONTAL_IZQUIERDA], distancia_cm)
            elif -120 < angulo_deg <= -60:
                sectores[SectorLIDAR.LATERAL_IZQUIERDO] = min(sectores[SectorLIDAR.LATERAL_IZQUIERDO], distancia_cm)
            elif 60 <= angulo_deg < 120:
                sectores[SectorLIDAR.LATERAL_DERECHO] = min(sectores[SectorLIDAR.LATERAL_DERECHO], distancia_cm)
            elif 120 <= angulo_deg <= 150:
                sectores[SectorLIDAR.TRASERO_DERECHA] = min(sectores[SectorLIDAR.TRASERO_DERECHA], distancia_cm)
            elif -150 <= angulo_deg <= -120:
                sectores[SectorLIDAR.TRASERO_IZQUIERDA] = min(sectores[SectorLIDAR.TRASERO_IZQUIERDA], distancia_cm)
            elif angulo_deg < -150 or angulo_deg > 150:
                sectores[SectorLIDAR.TRASERO] = min(sectores[SectorLIDAR.TRASERO], distancia_cm)

        return LecturasSensoresLIDAR(
            frontal=sectores[SectorLIDAR.FRONTAL],
            frontal_izquierda=sectores[SectorLIDAR.FRONTAL_IZQUIERDA],
            lateral_izquierdo=sectores[SectorLIDAR.LATERAL_IZQUIERDO],
            lateral_derecho=sectores[SectorLIDAR.LATERAL_DERECHO],
            trasero_izquierda=sectores[SectorLIDAR.TRASERO_IZQUIERDA],
            trasero_derecha=sectores[SectorLIDAR.TRASERO_DERECHA],
            trasero=sectores[SectorLIDAR.TRASERO],
            timestamp=timestamp
        )


@dataclass
class DeteccionObstaculoCamara:
    """Detección de obstáculo por visión (cámara/YOLO)."""
    hay_obstaculo: bool         # True si se detectó algo
    clase: str                  # clase del objeto detectado
    confianza: float            # 0.0-1.0, confianza de la detección
    posicion_x: float           # 0.0-1.0, posición normalizada en X
    posicion_y: float           # 0.0-1.0, posición normalizada en Y
    area_pixels: int            # área en píxeles del bounding box
    timestamp: float = 0.0      # momento de la detección

    @property
    def esta_centrado(self) -> bool:
        """True si el obstáculo está más o menos centrado (±20% del ancho)."""
        return 0.4 < self.posicion_x < 0.6

    @property
    def esta_a_la_izquierda(self) -> bool:
        """True si el obstáculo está a la izquierda (< 40%)."""
        return self.posicion_x < 0.4

    @property
    def esta_a_la_derecha(self) -> bool:
        """True si el obstáculo está a la derecha (> 60%)."""
        return self.posicion_x > 0.6


class ControladorEvasion:
    """
    Orquestador de la lógica de evasión de obstáculos.

    Recibe lecturas de sensores y detecciones de cámara, decide acciones,
    y coordina con el controlador de movimiento del robot.
    """

    # ====== PARÁMETROS CRÍTICOS ======
    DIST_CRITICA_PARADA_CM = 15.0      # parada inmediata si algo < 15cm
    DIST_PRECAUCIÓN_CM = 40.0           # zona de alerta (40-15cm)
    DIST_SEGURA_CM = 60.0               # distancia confortable

    VELOCIDAD_EVASION_MS = 0.15         # velocidad lenta durante evasión
    VELOCIDAD_RETROCESO_MS = 0.1        # velocidad en retroceso
    VELOCIDAD_GIRO_EVASION = 0.3        # velocidad angular durante giro evasión

    TIMEOUT_EVASION_S = 10.0            # máximo tiempo en evasión antes de parada
    HYSTERESIS_RECUPERACION_CM = 20.0   # buffer para salir de alerta

    # Confianza mínima para reaccionar a detecciones YOLO
    CONFIANZA_MINIMA_CAMARA = 0.5

    # Tamaño mínimo de bounding box para considerar amenaza real
    AREA_MINIMA_PIXELS = 500

    def __init__(self,
                 callback_comando: Callable[[float, float], None],
                 callback_parada: Callable[[], None],
                 callback_debug: Optional[Callable[[str], None]] = None):
        """
        Inicializa el controlador de evasión.

        Args:
            callback_comando: func(vx: float, omega: float) — envía velocidad
                             y velocidad angular al robot
            callback_parada: func() — detiene el robot al instante
            callback_debug: func(msg: str) — opcional, para logging
        """
        self._callback_comando = callback_comando
        self._callback_parada = callback_parada
        self._callback_debug = callback_debug or (lambda x: None)

        self.estado = EstadoEvasion.NAVEGACION_NORMAL
        self._ultima_lectura_lidar: Optional[LecturasSensoresLIDAR] = None
        self._ultima_deteccion_camara: Optional[DeteccionObstaculoCamara] = None
        self._tiempo_inicio_evasion = 0.0
        self._vx_deseada_previo = 0.0
        self._omega_deseada_previo = 0.0

    def procesar_lidar(self, lecturas: LecturasSensoresLIDAR) -> EstadoEvasion:
        """
        Procesa scan del LIDAR y actualiza el estado de evasión.

        Args:
            lecturas: LecturasSensoresLIDAR con distancias por sector

        Returns:
            EstadoEvasion nuevo
        """
        self._ultima_lectura_lidar = lecturas
        dist_min = lecturas.distancia_minima()
        sector = lecturas.sector_critico()

        if dist_min < self.DIST_CRITICA_PARADA_CM:
            self._entrar_parada_critica(sector)
        elif dist_min < self.DIST_PRECAUCIÓN_CM:
            self._entrar_alerta_proximidad(sector)
        elif self.estado in (EstadoEvasion.ALERTA_PROXIMIDAD,
                            EstadoEvasion.EVASION_LATERAL,
                            EstadoEvasion.EVASION_RETROCESO):
            if dist_min > self.DIST_PRECAUCIÓN_CM + self.HYSTERESIS_RECUPERACION_CM:
                self._recuperar_navegacion_normal()

        return self.estado

    def procesar_deteccion_camara(self, deteccion: DeteccionObstaculoCamara) -> EstadoEvasion:
        """
        Procesa detección de cámara/YOLO.

        Returns:
            EstadoEvasion nuevo
        """
        self._ultima_deteccion_camara = deteccion

        # Si no hay obstáculo o tiene baja confianza/tamaño, ignorar
        if not deteccion.hay_obstaculo:
            return self.estado
        if deteccion.confianza < self.CONFIANZA_MINIMA_CAMARA:
            return self.estado
        if deteccion.area_pixels < self.AREA_MINIMA_PIXELS:
            return self.estado

        # Obstáculo detectado de verdad: reaccionar
        if self.estado == EstadoEvasion.NAVEGACION_NORMAL:
            self._entrar_alerta_camara(deteccion)
        elif self.estado == EstadoEvasion.ALERTA_PROXIMIDAD:
            self._entrar_evasion_lateral(deteccion)

        return self.estado

    def aplicar_velocidad_deseada(self, vx: float, omega: float) -> None:
        """
        Aplica velocidad deseada, modificándola según el estado de evasión.

        Este método INTERCEPTA la velocidad deseada por el navegador y puede:
          · Permitirla tal cual si es seguro
          · Modificarla para evasión reactiva
          · Detenerla si hay riesgo crítico
        """
        self._vx_deseada_previo = vx
        self._omega_deseada_previo = omega

        # Parada crítica siempre gana
        if self.estado == EstadoEvasion.PARADA_CRITICA:
            self._callback_parada()
            self._callback_debug(f"[EVASION] PARADA CRÍTICA — dist_min: {self._ultima_lectura_sensores.distancia_minima():.1f}cm")
            return

        # En evasión, dejar solo la maniobra de evasión
        if self.estado in (EstadoEvasion.EVASION_LATERAL, EstadoEvasion.EVASION_RETROCESO):
            self._aplicar_evasion()
            return

        # En alerta pero sin evasión activa, reducir velocidad
        if self.estado == EstadoEvasion.ALERTA_PROXIMIDAD:
            vx_limitado = min(vx, 0.15)  # máximo 0.15 m/s en alerta
            omega_limitado = omega
            self._callback_comando(vx_limitado, omega_limitado)
            return

        # Navegación normal: pasar tal cual
        self._callback_comando(vx, omega)

    def _entrar_parada_critica(self, sector: Optional[SectorLIDAR]) -> None:
        """Detiene al instante sin preguntar."""
        if self.estado != EstadoEvasion.PARADA_CRITICA:
            self._callback_debug(f"[EVASION] → PARADA CRÍTICA (LIDAR sector {sector})")
            self.estado = EstadoEvasion.PARADA_CRITICA
            self._callback_parada()

    def _entrar_alerta_proximidad(self, sector: Optional[SectorLIDAR]) -> None:
        """Entra en modo de alerta por proximidad (LIDAR)."""
        if self.estado != EstadoEvasion.ALERTA_PROXIMIDAD:
            self._callback_debug(f"[EVASION] → ALERTA PROXIMIDAD (LIDAR sector {sector})")
            self.estado = EstadoEvasion.ALERTA_PROXIMIDAD

    def _entrar_alerta_camara(self, deteccion: DeteccionObstaculoCamara) -> None:
        """Entra en modo de alerta por detección de cámara."""
        if self.estado != EstadoEvasion.ALERTA_PROXIMIDAD:
            self._callback_debug(f"[EVASION] → ALERTA CÁMARA ({deteccion.clase}, conf={deteccion.confianza:.2f})")
            self.estado = EstadoEvasion.ALERTA_PROXIMIDAD

    def _entrar_evasion_lateral(self, deteccion: DeteccionObstaculoCamara) -> None:
        """Inicia maniobra de evasión lateral."""
        self._tiempo_inicio_evasion = time.time()

        if deteccion.esta_a_la_izquierda:
            self._callback_debug(f"[EVASION] → EVASIÓN LATERAL DERECHA (obstáculo a la izq)")
            self.estado = EstadoEvasion.EVASION_LATERAL
        elif deteccion.esta_a_la_derecha:
            self._callback_debug(f"[EVASION] → EVASIÓN LATERAL IZQUIERDA (obstáculo a la der)")
            self.estado = EstadoEvasion.EVASION_LATERAL
        else:
            self._callback_debug(f"[EVASION] → EVASIÓN RETROCESO (obstáculo centrado)")
            self.estado = EstadoEvasion.EVASION_RETROCESO

    def _recuperar_navegacion_normal(self) -> None:
        """Vuelve a navegación normal cuando el camino se despeja."""
        self._callback_debug("[EVASION] → RECUPERACIÓN NAVEGACIÓN NORMAL")
        self.estado = EstadoEvasion.NAVEGACION_NORMAL
        self._tiempo_inicio_evasion = 0.0

    def _aplicar_evasion(self) -> None:
        """Ejecuta maniobra de evasión según el estado."""
        tiempo_transcurrido = time.time() - self._tiempo_inicio_evasion

        if tiempo_transcurrido > self.TIMEOUT_EVASION_S:
            self._callback_debug("[EVASION] timeout de evasión — paradon")
            self._callback_parada()
            self.estado = EstadoEvasion.PARADA_CRITICA
            return

        if self.estado == EstadoEvasion.EVASION_LATERAL:
            # Giro en función del obstáculo
            if self._ultima_deteccion_camara:
                if self._ultima_deteccion_camara.esta_a_la_izquierda:
                    # Girar a la derecha y avanzar lentamente
                    self._callback_comando(self.VELOCIDAD_EVASION_MS,
                                         -self.VELOCIDAD_GIRO_EVASION)
                else:
                    # Girar a la izquierda y avanzar lentamente
                    self._callback_comando(self.VELOCIDAD_EVASION_MS,
                                         self.VELOCIDAD_GIRO_EVASION)

        elif self.estado == EstadoEvasion.EVASION_RETROCESO:
            # Retroceder en línea recta
            self._callback_comando(-self.VELOCIDAD_RETROCESO_MS, 0.0)

    def get_estado_json(self) -> Dict:
        """Retorna el estado actual como dict serializable a JSON."""
        lidar = self._ultima_lectura_lidar
        deteccion = self._ultima_deteccion_camara

        return {
            "estado": self.estado.value,
            "lidar_sectores_cm": {
                "frontal": lidar.frontal if lidar else None,
                "frontal_izquierda": lidar.frontal_izquierda if lidar else None,
                "lateral_izquierdo": lidar.lateral_izquierdo if lidar else None,
                "lateral_derecho": lidar.lateral_derecho if lidar else None,
                "trasero_izquierda": lidar.trasero_izquierda if lidar else None,
                "trasero_derecha": lidar.trasero_derecha if lidar else None,
                "trasero": lidar.trasero if lidar else None,
                "distancia_minima_cm": lidar.distancia_minima() if lidar else None,
            },
            "camara": {
                "hay_obstaculo": deteccion.hay_obstaculo if deteccion else False,
                "clase": deteccion.clase if deteccion else None,
                "confianza": deteccion.confianza if deteccion else 0.0,
                "posicion_x": deteccion.posicion_x if deteccion else None,
            },
            "velocidad_actual": {
                "vx": self._vx_deseada_previo,
                "omega": self._omega_deseada_previo,
            }
        }
