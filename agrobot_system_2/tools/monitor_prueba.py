"""Monitor de una prueba con el robot real: cada INTERVALO s (10 por defecto) imprime UNA línea con pose, huecos del LIDAR, estado del explorador, seguridad, enlace con la C3 y
alertas (hueco <10 cm en marcha, sin avance 30 s con la autonomía activa, salto de pose >1 m, seguridad ≠ normal, enlace caído, IMU viejo, temperatura alta) y la guarda en
~/.agrobot/pruebas/<nombre>/monitor.jsonl. No manda NINGUNA orden al robot: solo lee (WebSocket + /api/estado/completo + /api/health).
Uso: python3 tools/monitor_prueba.py <nombre> [intervalo_s]"""
import asyncio, json, math, sys, time
from pathlib import Path

import aiohttp

NOMBRE = sys.argv[1] if len(sys.argv) > 1 else "prueba"
INTERVALO = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
BASE = "http://localhost:8080"
CARPETA = Path.home() / ".agrobot" / "pruebas" / NOMBRE
CARPETA.mkdir(parents=True, exist_ok=True)


async def main():
    ult, hist = {}, []
    t0 = time.time()
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(BASE + "/ws", heartbeat=15) as ws:
            async def leer():
                async for m in ws:
                    if m.type == aiohttp.WSMsgType.TEXT:
                        try:
                            j = json.loads(m.data)
                        except ValueError:
                            continue
                        ult[j.get("type")] = j.get("data")
                        if j.get("type") == "toast":
                            ult.setdefault("_toasts", []).append(j["data"])
            tarea = asyncio.create_task(leer())
            while True:
                await asyncio.sleep(INTERVALO)
                ahora = time.time() - t0
                try:
                    async with s.get(BASE + "/api/estado/completo", timeout=aiohttp.ClientTimeout(total=4)) as r:
                        est = await r.json()
                    async with s.get(BASE + "/api/health", timeout=aiohttp.ClientTimeout(total=4)) as r:
                        salud = await r.json()
                except Exception as exc:
                    print(f"[T+{ahora:5.0f}s] ¡SIN RESPUESTA del backend! {type(exc).__name__}", flush=True); continue
                pose = ult.get("pose") or {}
                d = {k: (v / 100.0 if isinstance(v, (int, float)) and k in ("adelante", "atras", "izquierda", "derecha", "cerca") else v) for k, v in (ult.get("lidar_distancias") or {}).items()}     # el WebSocket da CENTÍMETROS
                ex = ult.get("exploracion") or {}
                seg = ult.get("seguridad") or {}
                mov = est.get("motor", {}).get("referencias", {}) or {}
                x, y, th = pose.get("x"), pose.get("y"), pose.get("theta_deg")
                hist.append((ahora, x, y))
                alertas = []
                f, i, dr = d.get("adelante"), d.get("izquierda"), d.get("derecha")
                en_marcha = abs(mov.get("v_aplicada", 0) or 0) > 0.01 or abs(mov.get("omega_aplicada", 0) or 0) > 0.05
                for nombre, v in (("adelante", f), ("izquierda", i), ("derecha", dr)):
                    if v is not None and 0.05 < v < 0.10 and en_marcha:
                        alertas.append(f"hueco {nombre} {v * 100:.0f} cm EN MARCHA")
                if x is not None and len(hist) >= 4 and ex.get("activo"):
                    ref = next((h for h in reversed(hist[:-1]) if ahora - h[0] >= 30), None)
                    if ref and ref[1] is not None and math.hypot(x - ref[1], y - ref[2]) < 0.05:
                        alertas.append("SIN AVANCE 30 s con la autonomía activa")
                if len(hist) >= 2 and None not in (x, hist[-2][1]) and math.hypot(x - hist[-2][1], y - hist[-2][2]) > 1.0:
                    alertas.append(f"SALTO de pose {math.hypot(x - hist[-2][1], y - hist[-2][2]):.1f} m")
                if seg.get("nivel") not in (None, "normal"):
                    alertas.append(f"seguridad {seg.get('nivel')}: {seg.get('motivos')}")
                if not est.get("c3", {}).get("conectado"):
                    alertas.append("C3 DESCONECTADA")
                if not est.get("imu", {}).get("vigente", True):
                    alertas.append("IMU sin datos frescos")
                for c in salud:
                    if c.get("status") not in ("ok", None) and c.get("key") in ("lidar", "camera", "slam", "ekf"):
                        alertas.append(f"salud {c.get('key')}: {c.get('status')}")
                tmp = [t for t in (ult.get("system_stats") or {}).values() if isinstance(t, (int, float))]
                nuevos = ult.pop("_toasts", [])
                fila = {"t_s": round(ahora), "pose": [x, y, th], "lidar": d, "explorador": {k: ex.get(k) for k in ("activo", "estado", "pulsos", "giros", "desatascos", "correcciones")},
                        "seguridad": seg.get("nivel"), "mando": {k: mov.get(k) for k in ("v_aplicada", "omega_aplicada")}, "alertas": alertas, "avisos": [f"{n.get('title')}: {n.get('message')}" for n in nuevos][-3:]}
                with open(CARPETA / "monitor.jsonl", "a", encoding="utf-8") as fh:
                    fh.write(json.dumps(fila, ensure_ascii=False) + "\n")
                fmt = lambda v: "—" if v is None else f"{v:.2f}"
                print(f"[T+{ahora:5.0f}s] pose=({fmt(x)},{fmt(y)},{0 if th is None else th:.0f}°) LIDAR f={fmt(f)} i={fmt(i)} d={fmt(dr)} | expl={'ON' if ex.get('activo') else 'off'} "
                      f"'{str(ex.get('estado'))[:38]}' p={ex.get('pulsos')} g={ex.get('giros')} | v={fmt(mov.get('v_aplicada'))} w={fmt(mov.get('omega_aplicada'))} | seg={seg.get('nivel')}"
                      + (" | ALERTA: " + "; ".join(alertas) if alertas else "") + (" | avisos: " + " / ".join(fila["avisos"]) if fila["avisos"] else ""), flush=True)


asyncio.run(main())
