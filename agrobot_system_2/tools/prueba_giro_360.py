"""Giro de 360° con el robot REAL (maniobra_giro.py, POST /api/maniobra_giro) con vigilancia propia: cada 2 s anota progreso, pose y huecos; si algún hueco útil (frente/izquierda/derecha, sin
contar lo pegado al LIDAR <6 cm) baja de PARO_M durante la maniobra, manda la parada de emergencia. Resume: ¿completó?, tiempo, desplazamiento neto y rumbo final.
Uso: python3 tools/prueba_giro_360.py <nombre> [grados=360] [tramo=20]"""
import asyncio, json, math, sys, time
from pathlib import Path

import aiohttp

BASE = "http://localhost:8080"
NOMBRE = sys.argv[1] if len(sys.argv) > 1 else "prueba"
GRADOS = float(sys.argv[2]) if len(sys.argv) > 2 else 360.0
TRAMO = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
PARO_M = 0.07
CARPETA = Path.home() / ".agrobot" / "pruebas" / NOMBRE; CARPETA.mkdir(parents=True, exist_ok=True)


async def main():
    ult, log = {}, []
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(BASE + "/ws") as ws:
            async def leer():
                async for m in ws:
                    if m.type == aiohttp.WSMsgType.TEXT:
                        j = json.loads(m.data); ult[j.get("type")] = j.get("data")
            lector = asyncio.create_task(leer()); await asyncio.sleep(2)
            p0 = dict(ult.get("pose") or {})
            print("inicio: pose", {k: round(v, 2) for k, v in p0.items() if k in ("x", "y", "theta_deg")}, "huecos(cm)", ult.get("lidar_distancias"), flush=True)
            async with s.post(BASE + "/api/maniobra_giro", json={"grados": GRADOS, "tramo": TRAMO}) as r:
                print("POST", r.status, await r.text(), flush=True)
                if r.status != 200:
                    return
            t0, parada = time.time(), None
            while time.time() - t0 < 150:
                await asyncio.sleep(2)
                async with s.get(BASE + "/api/maniobra_giro") as r:
                    est = await r.json()
                d = {k: (v / 100.0 if isinstance(v, (int, float)) else v) for k, v in (ult.get("lidar_distancias") or {}).items()}
                pose = ult.get("pose") or {}
                fila = {"t": round(time.time() - t0), "estado": est, "pose": [round(pose.get("x", 0), 2), round(pose.get("y", 0), 2), round(pose.get("theta_deg", 0))], "huecos": {k: d.get(k) for k in ("adelante", "izquierda", "derecha")}}
                log.append(fila)
                print(f"t={fila['t']:3d}s girado={est.get('girado_deg')}° tramos={est.get('tramos')} atrás={est.get('atras')} adelante={est.get('adelante')} [{est.get('estado')}/{est.get('motivo')}] pose={fila['pose']} huecos={fila['huecos']}", flush=True)
                utiles = [v for k, v in fila["huecos"].items() if v is not None and v >= 0.06]
                if utiles and min(utiles) < PARO_M and est.get("activa"):
                    parada = f"hueco {min(utiles) * 100:.0f} cm"
                    async with s.post(BASE + "/api/emergencia") as r2:
                        print("¡PARADA DE EMERGENCIA!", parada, r2.status, flush=True)
                    break
                if not est.get("activa") and est.get("estado") == "fin":
                    break
            lector.cancel()
    pf = ult.get("pose") or {}
    res = {"pedido": GRADOS, "tramo": TRAMO, "duracion_s": round(time.time() - t0), "parada": parada, "final": est, "desplazamiento_neto_m": round(math.hypot(pf.get("x", 0) - p0.get("x", 0), pf.get("y", 0) - p0.get("y", 0)), 2),
           "rumbo_inicial": round(p0.get("theta_deg", 0)), "rumbo_final": round(pf.get("theta_deg", 0)), "log": log}
    (CARPETA / f"giro_{int(GRADOS)}.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    print("\nRESUMEN:", {k: res[k] for k in ("pedido", "duracion_s", "parada", "desplazamiento_neto_m", "rumbo_inicial", "rumbo_final")}, "| girado", est.get("girado_deg"), "tramos", est.get("tramos"), "atrás", est.get("atras"), "adelante", est.get("adelante"), "motivo", est.get("motivo"), flush=True)

asyncio.run(main())
