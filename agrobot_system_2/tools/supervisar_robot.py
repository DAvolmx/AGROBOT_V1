"""
Supervisor del robot: una línea por cada cosa que CAMBIA o es peligrosa.
=======================================================================

Pensado para vigilar al robot mientras alguien lo maneja o mientras corre
la autonomía, sin inundar: no imprime el estado, imprime los cambios.

  · salud (/api/health): cada fila que cambia de estado
  · seguridad: cambios de nivel (normal / precaución / seguro) y motivos
  · C3: vías activas, reinicios, failsafe
  · movimiento: cuándo arranca y para, con cuánto avanzó/giró (LIDAR/rumbo)
  · freno: cada frenada por obstáculo y si fue el LIDAR o la cámara
  · peligro: hueco < 10 cm en cualquier lado
  · log del backend: emergencia, failsafe, errores, evasión, reinicios

    python3 tools/supervisar_robot.py [http://localhost:8080]
"""
import asyncio
import json
import sys
import time
from pathlib import Path

import aiohttp

BASE = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:8080"
LOG = Path(__file__).resolve().parents[1] / "logs" / "backend.log"
PELIGRO_CM = 10
CLAVES_LOG = ("[emergencia]", "FAILSAFE", "Traceback", "[seguridad]", "acaba de arrancar",
              "vías activas", "[evasion]", "[suelo]", "se reinició", "Killed", "Error")


def decir(*partes):
    print(time.strftime("%H:%M:%S"), *partes, flush=True)


async def salud(s):
    previo = {}
    seg_previo = None
    while True:
        try:
            async with s.get(f"{BASE}/api/health", timeout=4) as r:
                filas = await r.json()
            for f in filas:
                clave = f["key"]
                actual = (f["status"], f["detail"] if f["status"] != "ok" else "")
                if clave in previo and previo[clave] != actual:
                    decir(f"[salud] {clave}: {previo[clave][0]} -> {f['status']} {f['detail']}")
                previo[clave] = actual
            async with s.get(f"{BASE}/api/seguridad", timeout=4) as r:
                seg = await r.json()
            clave_seg = (seg["nivel"], tuple(seg["motivos"]))
            if clave_seg != seg_previo:
                decir(f"[seguridad] {seg['nivel']}" + (f": {'; '.join(seg['motivos'])}" if seg["motivos"] else ""))
                seg_previo = clave_seg
        except Exception as exc:
            decir(f"[supervisor] backend sin respuesta ({type(exc).__name__})")
            await asyncio.sleep(5)
        await asyncio.sleep(2)


async def en_vivo(s):
    while True:
        try:
            async with s.ws_connect(f"{BASE}/ws", heartbeat=10) as ws:
                lidar = {}
                rumbo = None
                movimiento = None          # (t0, lidar0, rumbo0, máx v, máx w)
                ultimo_mov = 0.0
                peligro_avisado = set()
                ultimo_freno = 0.0
                failsafe_previo = None
                async for m in ws:
                    if m.type != aiohttp.WSMsgType.TEXT:
                        continue
                    d = json.loads(m.data)
                    t, data = d.get("type"), d.get("data") or {}
                    ahora = time.monotonic()
                    if t == "lidar_distancias":
                        lidar = data
                        for lado in ("adelante", "atras", "izquierda", "derecha"):
                            v = data.get(lado)
                            if v is not None and v < PELIGRO_CM and lado not in peligro_avisado:
                                decir(f"[PELIGRO] {lado}: {v} cm")
                                peligro_avisado.add(lado)
                            elif v is not None and v >= PELIGRO_CM + 5:
                                peligro_avisado.discard(lado)
                    elif t == "pose":
                        rumbo = data.get("theta_deg")
                    elif t == "obstacle_warning" and ahora - ultimo_freno > 2:
                        ultimo_freno = ahora
                        decir(f"[freno] {data.get('lado')} a {data.get('distancia_m')} m "
                              f"(umbral {data.get('umbral_m')}) por {data.get('fuente', 'lidar')}")
                    elif t == "actuator_status":
                        fs = data.get("failsafe")
                        if failsafe_previo is not None and fs != failsafe_previo and fs:
                            decir("[C3] failsafe (sin órdenes)")
                        failsafe_previo = fs
                        v, w = data.get("v_cmd", 0) or 0, data.get("omega_cmd", 0) or 0
                        if abs(v) > 0.01 or abs(w) > 0.01:
                            ultimo_mov = ahora
                            if movimiento is None:
                                movimiento = [ahora, dict(lidar), rumbo, v, w]
                            else:
                                movimiento[3] = v if abs(v) > abs(movimiento[3]) else movimiento[3]
                                movimiento[4] = w if abs(w) > abs(movimiento[4]) else movimiento[4]
                        elif movimiento is not None and ahora - ultimo_mov > 1.5:
                            t0, l0, r0, vmax, wmax = movimiento
                            movimiento = None
                            av = None
                            if l0.get("adelante") is not None and lidar.get("adelante") is not None:
                                av = l0["adelante"] - lidar["adelante"]
                            giro = None if (r0 is None or rumbo is None) else ((rumbo - r0 + 180) % 360 - 180)
                            decir(f"[movimiento] {ultimo_mov - t0:.1f} s, v máx {vmax:+.2f}, w máx {wmax:+.2f} | "
                                  f"frente {l0.get('adelante')}→{lidar.get('adelante')} cm"
                                  + (f" ({av:+} cm)" if av is not None else "")
                                  + (f" | rumbo {giro:+.1f}°" if giro is not None else "")
                                  + f" | huecos izq {lidar.get('izquierda')} der {lidar.get('derecha')} "
                                    f"atrás {lidar.get('atras')}")
        except Exception as exc:
            decir(f"[supervisor] WebSocket caído ({type(exc).__name__}), reintento")
            await asyncio.sleep(3)


async def log_backend():
    pos = LOG.stat().st_size if LOG.exists() else 0
    while True:
        await asyncio.sleep(1)
        try:
            tam = LOG.stat().st_size
            if tam < pos:
                pos = 0                      # el log se rotó
            if tam == pos:
                continue
            with open(LOG, "rb") as f:
                f.seek(pos)
                nuevo = f.read().decode("utf-8", "replace")
            pos = tam
            for linea in nuevo.splitlines():
                if any(k in linea for k in CLAVES_LOG):
                    decir("[log]", linea.strip()[:200])
        except OSError:
            pass


async def main():
    decir(f"[supervisor] vigilando {BASE}")
    async with aiohttp.ClientSession() as s:
        await asyncio.gather(salud(s), en_vivo(s), log_backend())


if __name__ == "__main__":
    asyncio.run(main())
