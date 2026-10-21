"""Saludo del robot: izquierda, derecha, mira arriba, parpadea el foco 3 veces y hace un barrido de cámara.

Orquestador puro: recibe las acciones como funciones async (`girar`, `servo`, `foco`) y el `dormir`, así se
prueba sin hardware. Si lo cancelan (parada de emergencia) o falla un paso, deja el servo en reposo y el
foco apagado.
"""
from __future__ import annotations

import asyncio

from protocol import SERVO_HOME_DEG

ANG_ARRIBA = 110                 # tope del servo: la cámara mira lo más arriba que puede
BARRIDO_MIN, BARRIDO_PASO = 60, 5


class Saludo:
    def __init__(self, girar, servo, foco, dormir=asyncio.sleep, parpadeos=3, t_parpadeo=0.3,
                 t_mirar=1.2, t_paso_barrido=0.25):
        self.girar, self.servo, self.foco, self.dormir = girar, servo, foco, dormir
        self.parpadeos, self.t_parpadeo, self.t_mirar, self.t_paso = parpadeos, t_parpadeo, t_mirar, t_paso_barrido
        self.pasos: list[str] = []

    async def ejecutar(self) -> list[str]:
        try:
            await self.girar("izquierda"); self.pasos.append("izquierda")
            await self.girar("derecha"); self.pasos.append("derecha")
            await self.servo(ANG_ARRIBA); self.pasos.append("mirar_arriba")
            await self.dormir(self.t_mirar)
            for _ in range(self.parpadeos):
                await self.foco(True); await self.dormir(self.t_parpadeo)
                await self.foco(False); await self.dormir(self.t_parpadeo)
            self.pasos.append("foco")
            for ang in list(range(ANG_ARRIBA, BARRIDO_MIN - 1, -BARRIDO_PASO)) + \
                    list(range(BARRIDO_MIN + BARRIDO_PASO, ANG_ARRIBA + 1, BARRIDO_PASO)):
                await self.servo(ang); await self.dormir(self.t_paso)
            self.pasos.append("barrido")
        finally:                               # pase lo que pase: foco apagado y cámara en reposo
            try:
                await self.foco(False)
                await self.servo(SERVO_HOME_DEG)
            except Exception:
                pass
        return self.pasos
