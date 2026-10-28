"""Prueba de giros con el robot REAL: manda giros de lazo cerrado (POST /api/giro_cerrado) de poco ángulo, alternando izquierda y derecha para que el robot quede donde empezó, y mide
cuánto gira de verdad (giroscopio, con la inercia, y rumbo de la pose). Se escala de a poco (ángulo y potencia) y se PARA si algún hueco del LIDAR baja de MIN_HUECO_M, si el giro no
responde o si la seguridad deja de estar en «normal». Salida: tabla en pantalla + ~/.agrobot/pruebas/<nombre>/giros.json. Uso: python3 tools/prueba_giros_real.py <nombre>"""
import asyncio, json, math, sys, time
from pathlib import Path

import aiohttp

BASE = "http://localhost:8080"
NOMBRE = sys.argv[1] if len(sys.argv) > 1 else "prueba"
MIN_HUECO_M = 0.12            # sin contar lo que está pegado al LIDAR (el propio chasis: <0,06 m)
PLAN = [(6, 0.6), (6, 0.9), (10, 0.6), (10, 0.9), (10, 1.2), (15, 0.9), (15, 1.2), (20, 1.2)]       # (grados, rad/s)
CARPETA = Path.home() / ".agrobot" / "pruebas" / NOMBRE; CARPETA.mkdir(parents=True, exist_ok=True)


def dang(a, b):
    return (a - b + 180) % 360 - 180


async def main():
    res, ult = [], {}
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect(BASE + "/ws") as ws:
            async def leer():
                async for m in ws:
                    if m.type == aiohttp.WSMsgType.TEXT:
                        j = json.loads(m.data); ult[j.get("type")] = j.get("data")
            t = asyncio.create_task(leer())
            await asyncio.sleep(2)

            def huecos():
                d = ult.get("lidar_distancias") or {}
                return {k: d.get(k) for k in ("adelante", "izquierda", "derecha")}

            def libre():
                return all((v is None or v >= MIN_HUECO_M or v < 0.06) for v in huecos().values())
            for n, (g, w) in enumerate(PLAN):
                for signo in (+1, -1):
                    if not libre():
                        print(f"PARO: hueco < {MIN_HUECO_M} m: {huecos()}"); break
                    if (ult.get("seguridad") or {}).get("nivel") not in (None, "normal"):
                        print("PARO: seguridad no normal", ult.get("seguridad")); break
                    th0 = (ult.get("pose") or {}).get("theta_deg")
                    async with s.post(BASE + "/api/giro_cerrado", json={"grados": signo * g, "w": w}) as r:
                        ok = r.status == 200
                        if not ok:
                            print("rechazado:", r.status, await r.text()); break
                    t0 = time.time()
                    while time.time() - t0 < 8:
                        await asyncio.sleep(0.25)
                        async with s.get(BASE + "/api/giro_cerrado") as r:
                            j = await r.json()
                        if not j.get("activo") and j.get("ultimo"):
                            break
                    await asyncio.sleep(1.2)
                    u = j.get("ultimo") or {}
                    th1 = (ult.get("pose") or {}).get("theta_deg")
                    fila = {"pedido": signo * g, "w": w, "girado_giro": u.get("girado_deg"), "final_giro": u.get("final_deg"), "duracion_s": u.get("duracion_s"), "motivo": u.get("motivo"),
                            "w_final": u.get("w_final"), "rumbo_pose": None if None in (th0, th1) else round(dang(th1, th0), 1), "huecos_despues": huecos()}
                    res.append(fila)
                    err = None if u.get("final_deg") is None else round(u["final_deg"] - signo * g, 1)
                    print(f"pedido {signo * g:+3d}° w={w:.1f}: giroscopio {u.get('girado_deg')}° (final {u.get('final_deg')}°) en {u.get('duracion_s')} s [{u.get('motivo')}] · rumbo pose {fila['rumbo_pose']}° · error {err}° · huecos {huecos()}", flush=True)
                    await asyncio.sleep(1.0)
                else:
                    continue
                break
            t.cancel()
    (CARPETA / "giros.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    ok = [r for r in res if r["final_giro"] is not None]
    if ok:
        import statistics as st
        print("\nRESUMEN:", len(ok), "giros; error mediano |final−pedido| =", round(st.median(abs(r["final_giro"] - r["pedido"]) for r in ok), 1), "°;",
              "fracción que alcanzó ≥70 % del pedido:", round(sum(abs(r["final_giro"]) >= 0.7 * abs(r["pedido"]) for r in ok) / len(ok), 2))

asyncio.run(main())
