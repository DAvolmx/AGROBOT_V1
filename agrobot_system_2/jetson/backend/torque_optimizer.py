"""
Optimizador de torque: maximiza potencia sin comprometer seguridad.

Estrategias:
  · Reducir suavidad en terreno plano (duty 0.6 → 0.8)
  · Aumentar torque máximo dinámicamente según terreno/inclinación
  · Control adaptativo: si hay peso (carga), permitir más duty cycle
  · Monitoreo de sobrecalentamiento de motores
"""
from __future__ import annotations

import time
import asyncio
from typing import Callable


class TorqueOptimizer:
    def __init__(self, config_store=None):
        """
        config_store: instancia de config_store.py para leer/escribir config.
        """
        self.config_store = config_store

        # Parámetros adaptables (base → max)
        self.suavidad_duty_actual = 0.6
        self.suavidad_duty_max = 0.95

        self.max_v_mps_actual = 1.5
        self.max_v_mps_max = 2.5

        self.max_omega_rps_actual = 4.0
        self.max_omega_rps_max = 6.0

        # Estado del terreno (se actualiza por sensores)
        self.inclinacion_grados = 0.0
        self.en_pendiente = False
        self.es_terreno_duro = False  # ej: cemento vs barro

        # Monitoreo
        self.temperatura_motor_c = [20, 20]  # [izq, der]
        self.tiempo_ultimo_ajuste = time.monotonic()
        self.ajustes_realizados = 0

        self._on_cambio = None

    def set_on_cambio(self, callback: Callable[[dict], None]):
        """Callback cuando se ajusta el torque."""
        self._on_cambio = callback

    async def start(self):
        """Inicia monitoreo y ajustes automáticos."""
        asyncio.create_task(self._bucle_ajustes())

    def actualizar_sensores(self, inclinacion: float, temp_izq: float, temp_der: float):
        """Actualiza datos de sensores para decisiones de torque."""
        self.inclinacion_grados = inclinacion
        self.en_pendiente = abs(inclinacion) > 5.0
        self.temperatura_motor_c = [temp_izq, temp_der]

    async def _bucle_ajustes(self):
        """Ajusta torque cada 5 segundos basado en condiciones."""
        while True:
            try:
                await asyncio.sleep(5)
                await self._calcular_torque_optimo()
            except Exception as e:
                print(f"[torque] error en bucle: {e}", flush=True)

    async def _calcular_torque_optimo(self):
        """Calcula y aplica el torque óptimo."""
        # Base: usar valores configurados
        if self.config_store:
            try:
                conf = self.config_store.get("control")
                suavidad_base = conf.get("suavidad_duty", 0.6)
                v_max_base = conf.get("max_v_mps", 1.5)
                omega_max_base = conf.get("max_omega_rps", 4.0)
            except:
                suavidad_base = 0.6
                v_max_base = 1.5
                omega_max_base = 4.0
        else:
            suavidad_base = 0.6
            v_max_base = 1.5
            omega_max_base = 4.0

        # Calcular multiplicadores según condiciones
        factor_suavidad = 1.0
        factor_velocidad = 1.0
        factor_rotacion = 1.0

        # En pendiente: reducir (menos torque disponible para no volcarse)
        if self.en_pendiente:
            factor_velocidad *= 0.8
            factor_rotacion *= 0.75
        else:
            # En terreno plano: aumentar suavidad para mejor respuesta
            factor_suavidad = 1.3

        # Si terreno es duro (cemento): más tracción disponible
        if self.es_terreno_duro:
            factor_velocidad *= 1.15

        # Monitoreo de temperatura: throttle si se calienta
        temp_max = max(self.temperatura_motor_c)
        if temp_max > 60:
            factor_velocidad *= 0.7
            factor_rotacion *= 0.7
            print(f"[torque] ⚠ motor caliente ({temp_max}°C), reduciendo potencia", flush=True)
        elif temp_max > 50:
            factor_velocidad *= 0.85
            factor_rotacion *= 0.85

        # Aplicar factores
        nuevo_duty = min(suavidad_base * factor_suavidad, self.suavidad_duty_max)
        nuevo_v_max = min(v_max_base * factor_velocidad, self.max_v_mps_max)
        nuevo_omega_max = min(omega_max_base * factor_rotacion, self.max_omega_rps_max)

        # Si cambió algo, actualizar configuración
        if (abs(nuevo_duty - self.suavidad_duty_actual) > 0.05 or
            abs(nuevo_v_max - self.max_v_mps_actual) > 0.1 or
            abs(nuevo_omega_max - self.max_omega_rps_actual) > 0.2):

            self.suavidad_duty_actual = nuevo_duty
            self.max_v_mps_actual = nuevo_v_max
            self.max_omega_rps_actual = nuevo_omega_max
            self.tiempo_ultimo_ajuste = time.monotonic()
            self.ajustes_realizados += 1

            # Intentar actualizar config store
            if self.config_store:
                try:
                    conf = self.config_store.get("control")
                    conf["suavidad_duty"] = round(nuevo_duty, 3)
                    conf["max_v_mps"] = round(nuevo_v_max, 2)
                    conf["max_omega_rps"] = round(nuevo_omega_max, 2)
                    self.config_store.set("control", conf)

                    print(f"[torque] optimizado: duty={nuevo_duty:.2f}, "
                          f"v_max={nuevo_v_max:.1f}, omega={nuevo_omega_max:.1f}", flush=True)
                except Exception as e:
                    print(f"[torque] error guardando config: {e}", flush=True)

            if self._on_cambio:
                try:
                    self._on_cambio({
                        "suavidad_duty": round(nuevo_duty, 3),
                        "max_v_mps": round(nuevo_v_max, 2),
                        "max_omega_rps": round(nuevo_omega_max, 2),
                        "razon": self._razon_ajuste(),
                    })
                except Exception as e:
                    print(f"[torque] error en callback: {e}", flush=True)

    def _razon_ajuste(self) -> str:
        """Describe por qué se hizo el ajuste."""
        if self.en_pendiente:
            return f"pendiente {self.inclinacion_grados:.1f}°"
        elif self.es_terreno_duro:
            return "terreno duro (tracción alta)"
        elif max(self.temperatura_motor_c) > 60:
            return "motor muy caliente"
        else:
            return "terreno plano (torque máximo)"

    def resumen(self) -> dict:
        """Retorna estado actual."""
        return {
            "suavidad_duty": round(self.suavidad_duty_actual, 3),
            "max_v_mps": round(self.max_v_mps_actual, 2),
            "max_omega_rps": round(self.max_omega_rps_actual, 2),
            "inclinacion_grados": round(self.inclinacion_grados, 1),
            "temperatura_motores": [round(t, 1) for t in self.temperatura_motor_c],
            "ajustes_realizados": self.ajustes_realizados,
            "pendiente_detectada": self.en_pendiente,
        }


async def crear_optimizer(config_store=None) -> TorqueOptimizer:
    """Factory."""
    opt = TorqueOptimizer(config_store)
    await opt.start()
    return opt
