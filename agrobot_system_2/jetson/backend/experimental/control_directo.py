"""
Control Directo SIN BLOQUEOS
============================

Envía velocidades DIRECTAMENTE al C3 SIN:
  ❌ Pulsos
  ❌ Asyncio.sleep
  ❌ Timeouts
  ❌ Rampa
  ❌ Procesamiento de cámara
  ❌ SLAM
  ❌ LIDAR

SOLO: v (velocidad) + omega (giro) → C3 INMEDIATO

Reemplaza pulse_drive.py completamente.
"""

import struct
from typing import Optional


class ControlDirecto:
    """Control directo sin bloqueos."""

    def __init__(self, send_command_func):
        """
        Args:
            send_command_func: función(v, omega) que envía al C3
        """
        self.send_command = send_command_func
        self.last_v = 0.0
        self.last_omega = 0.0

    def set_velocity(self, v: float, omega: float) -> bool:
        """
        Envía velocidad INMEDIATAMENTE al C3.

        SIN BLOQUEOS. SIN ESPERAS. SIN NADA.

        Args:
            v: velocidad lineal (-1.5 a +1.5 m/s)
            omega: velocidad angular (-4.0 a +4.0 rad/s)

        Returns:
            True si se envió, False si falló
        """
        # Saturación
        v = max(-1.5, min(1.5, v))
        omega = max(-4.0, min(4.0, omega))

        # Solo envía si cambió (ahorra ancho de banda)
        if abs(v - self.last_v) < 0.01 and abs(omega - self.last_omega) < 0.01:
            return True

        self.last_v = v
        self.last_omega = omega

        try:
            # Envía INMEDIATAMENTE
            self.send_command(v, omega, 0)
            return True
        except Exception as e:
            print(f"[ControlDirecto] Error enviando comando: {e}")
            return False

    def stop(self):
        """Para el robot."""
        self.set_velocity(0.0, 0.0)

    def adelante(self, velocidad: float = 0.5):
        """Adelante."""
        self.set_velocity(velocidad, 0.0)

    def atras(self, velocidad: float = 0.3):
        """Atrás."""
        self.set_velocity(-velocidad, 0.0)

    def giro_izquierda(self, omega: float = 2.0):
        """Giro izquierda puro."""
        self.set_velocity(0.0, omega)

    def giro_derecha(self, omega: float = 2.0):
        """Giro derecha puro."""
        self.set_velocity(0.0, -omega)

    def arco_adelante_izquierda(self, v: float = 0.5, omega: float = 1.5):
        """Arco adelante izquierda."""
        self.set_velocity(v, omega)

    def arco_adelante_derecha(self, v: float = 0.5, omega: float = 1.5):
        """Arco adelante derecha."""
        self.set_velocity(v, -omega)


# ============================================================================
# USO EN jetson_server.py
# ============================================================================
"""
REEMPLAZA pulse_drive con:

from control_directo import ControlDirecto

conductor = ControlDirecto(send_command=udp_actuator_link.send_command)

# En el handler de comandos:
async def handle_manual_command(cmd):
    v = cmd.get('velocity', 0.0)      # -1.5 a +1.5
    omega = cmd.get('omega', 0.0)     # -4.0 a +4.0
    conductor.set_velocity(v, omega)  # INMEDIATO, SIN BLOQUEOS


# En el handler ADELANTE del dashboard:
async def handle_adelante(request):
    conductor.adelante(velocidad=0.5)
    return web.json_response({"ok": True})


# En el handler GIRO:
async def handle_giro_izquierda(request):
    conductor.giro_izquierda(omega=2.0)
    return web.json_response({"ok": True})
"""
