"""Anota un movimiento HECHO POR EL OPERADOR (con la distancia real que midió) junto con la pose y los huecos del robot en ese instante, para compararlos después con lo que midieron el SLAM y
el EKF (verdad de terreno). Archivo: ~/.agrobot/pruebas/<prueba>/movimientos_operador.jsonl. Uso: python3 tools/anotar_movimiento.py <prueba> "texto" [--dist_m 0.5] [--grados 90]"""
import argparse, asyncio, json, time
from pathlib import Path

import aiohttp

ap = argparse.ArgumentParser(); ap.add_argument("prueba"); ap.add_argument("texto"); ap.add_argument("--dist_m", type=float); ap.add_argument("--grados", type=float)
a = ap.parse_args()


async def main():
    ult = {}
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect("http://localhost:8080/ws") as ws:
            t0 = time.time()
            while time.time() - t0 < 6 and not ("pose" in ult and "lidar_distancias" in ult):
                try:
                    m = await asyncio.wait_for(ws.receive_json(), 2)
                except Exception:
                    break
                ult[m.get("type")] = m.get("data")
    d = {k: (v / 100.0 if isinstance(v, (int, float)) and k in ("adelante", "atras", "izquierda", "derecha") else v) for k, v in (ult.get("lidar_distancias") or {}).items()}
    p = ult.get("pose") or {}
    fila = {"hora": time.strftime("%H:%M:%S"), "unix": round(time.time(), 2), "texto": a.texto, "distancia_real_m": a.dist_m, "giro_real_deg": a.grados,
            "pose": [round(p.get("x", 0), 3), round(p.get("y", 0), 3), round(p.get("theta_deg", 0), 1)] if p else None, "huecos_m": {k: d.get(k) for k in ("adelante", "izquierda", "derecha")}}
    out = Path.home() / ".agrobot" / "pruebas" / a.prueba; out.mkdir(parents=True, exist_ok=True)
    with open(out / "movimientos_operador.jsonl", "a", encoding="utf-8") as fh:
        fh.write(json.dumps(fila, ensure_ascii=False) + "\n")
    print("anotado:", json.dumps(fila, ensure_ascii=False))

asyncio.run(main())
