"""Capturas ANOTADAS del dashboard para el informe de la tesis: abre cada pestaña del dashboard en vivo (http://localhost:8080), mide la posición de cada control de
tools/dashboard_spec.py con Chromium headless (CDP), dibuja cuadros y números ROJOS sobre la captura y guarda docs/INFORME_DASHBOARD/anotadas/<archivo>.png + posiciones.json.
Uso: python3 tools/capturas_anotadas_dashboard.py"""
import asyncio, base64, glob, json, subprocess, sys, time, urllib.request
from pathlib import Path

import aiohttp
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dashboard_spec import TABS

OUT = Path(__file__).resolve().parents[1] / "docs" / "INFORME_DASHBOARD"
SAL = OUT / "anotadas"; SAL.mkdir(parents=True, exist_ok=True)
CH = glob.glob(str(Path.home() / ".cache/ms-playwright/chromium_headless_shell-*/chrome-linux/headless_shell"))[0]
ROJO = (40, 40, 235)          # BGR
JS_RECT = """(()=>{const sels=%s;const closest=%s;let el=null;for(const s of sels.split(',,')){try{el=document.querySelector(s.trim());}catch(e){el=null} if(el&&el.getBoundingClientRect().width>2)break;}
if(!el)return null;if(closest){const c=el.closest(closest);if(c)el=c;}const r=el.getBoundingClientRect();return JSON.stringify({x:r.x,y:r.y,w:r.width,h:r.height});})()"""


def marcar(img, rects):
    """Dibuja cuadro rojo + círculo numerado, evitando que los círculos se tapen entre sí."""
    ocupados = []
    h, w = img.shape[:2]
    for n, r in rects:
        x, y, rw, rh = [int(round(v)) for v in (r["x"], r["y"], r["w"], r["h"])]
        x0, y0, x1, y1 = max(0, x), max(0, y), min(w - 1, x + rw), min(h - 1, y + rh)
        if x1 - x0 < 3 or y1 - y0 < 3:
            continue
        cv2.rectangle(img, (x0, y0), (x1, y1), ROJO, 3, cv2.LINE_AA)
        cx, cy = max(x0 + 2, 18), max(y0 + 2, 18)
        while any((cx - ox) ** 2 + (cy - oy) ** 2 < 30 ** 2 for ox, oy in ocupados) and cx < w - 30:
            cx += 34
        ocupados.append((cx, cy))
        cv2.circle(img, (cx, cy), 15, ROJO, -1, cv2.LINE_AA); cv2.circle(img, (cx, cy), 15, (255, 255, 255), 2, cv2.LINE_AA)
        txt = str(n); (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_DUPLEX, 0.6, 2)
        cv2.putText(img, txt, (cx - tw // 2, cy + th // 2), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return img


async def main():
    p = subprocess.Popen([CH, "--no-sandbox", "--disable-gpu", "--hide-scrollbars", "--window-size=1500,920", "--remote-debugging-port=9336", "about:blank"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    pos = {}
    try:
        for _ in range(40):
            try:
                tabs = json.load(urllib.request.urlopen("http://127.0.0.1:9336/json")); break
            except Exception:
                time.sleep(0.5)
        ws_url = next(t for t in tabs if t["type"] == "page")["webSocketDebuggerUrl"]
        async with aiohttp.ClientSession() as s, s.ws_connect(ws_url, max_msg_size=0) as ws:
            n = 0

            async def cdp(m, **par):
                nonlocal n
                n += 1; mid = n
                await ws.send_json({"id": mid, "method": m, "params": par})
                while True:
                    r = await ws.receive_json()
                    if r.get("id") == mid:
                        return r.get("result", {})

            ev = lambda e: cdp("Runtime.evaluate", expression=e, returnByValue=True)
            await cdp("Page.enable")
            await cdp("Emulation.setDeviceMetricsOverride", width=1500, height=920, deviceScaleFactor=1, mobile=False)
            await cdp("Page.navigate", url="http://localhost:8080/"); await asyncio.sleep(1.5)
            await ev("try{localStorage.setItem('agrobot-tema','hud')}catch(e){}")
            await cdp("Page.navigate", url="http://localhost:8080/"); await asyncio.sleep(6)
            for nombre, ruta, ancho, alto in (("aux_etiquetar", "/etiquetar.html", 1500, 900), ("aux_manejo_movil", "/manejo.html", 420, 860), ("aux_motores", "/motores.html", 1100, 900)):
                try:
                    await cdp("Emulation.setDeviceMetricsOverride", width=ancho, height=alto, deviceScaleFactor=1, mobile=(ancho < 600))
                    await cdp("Page.navigate", url="http://localhost:8080" + ruta); await asyncio.sleep(4)
                    shot = await cdp("Page.captureScreenshot", format="png")
                    (SAL / f"{nombre}.png").write_bytes(base64.b64decode(shot["data"])); print("pagina auxiliar", nombre, flush=True)
                except Exception as exc:
                    print("falló", nombre, exc, flush=True)
            await cdp("Page.navigate", url="http://localhost:8080/"); await asyncio.sleep(6)
            for t in TABS:
                await cdp("Emulation.setDeviceMetricsOverride", width=1500, height=920, deviceScaleFactor=1, mobile=False)
                await ev(f"document.querySelector('[data-tab=\"{t['id']}\"]').click()"); await asyncio.sleep(2.0)
                if t.get("preparar"):
                    await ev(t["preparar"]); await asyncio.sleep(2.0)
                alto = (await ev(f"document.getElementById('{t['id']}').scrollHeight"))["result"]["value"]
                alto = int(min(max(alto + 40, 920), t.get("alto_max", 2500)))
                await cdp("Emulation.setDeviceMetricsOverride", width=1500, height=alto, deviceScaleFactor=1, mobile=False)
                await asyncio.sleep(2.5)
                rects, faltan = [], []
                for i, it in enumerate(t["items"], 1):
                    sel, clos = it[0], it[1]
                    r = await ev(JS_RECT % (json.dumps(sel.replace(", ", ",,")), json.dumps(clos)))
                    v = r["result"].get("value")
                    if v:
                        rects.append((i, json.loads(v)))
                    else:
                        faltan.append((i, sel))
                shot = await cdp("Page.captureScreenshot", format="png")
                img = cv2.imdecode(np.frombuffer(base64.b64decode(shot["data"]), np.uint8), cv2.IMREAD_COLOR)
                limpia = img.copy()
                cv2.imwrite(str(SAL / f"{t['archivo']}_limpia.png"), limpia)
                cv2.imwrite(str(SAL / f"{t['archivo']}.png"), marcar(img, rects))
                pos[t["archivo"]] = {"alto": alto, "marcados": [n for n, _ in rects], "faltan": faltan}
                print(f"{t['archivo']}: {len(rects)}/{len(t['items'])} marcados; faltan {[f[1][:30] for f in faltan]}", flush=True)
    finally:
        p.terminate()
        (OUT / "posiciones.json").write_text(json.dumps(pos, indent=1, ensure_ascii=False))

asyncio.run(main())
