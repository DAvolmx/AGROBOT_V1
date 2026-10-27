"""Captura todas las pestañas del dashboard en vivo (http://localhost:8080) con Chromium headless por el protocolo CDP. Uso: python3 tools/capturas_dashboard.py <carpeta_salida> [tema=hud|sol]"""
import asyncio, base64, glob, json, subprocess, sys, time, urllib.request
from pathlib import Path
import aiohttp

SAL = Path(sys.argv[1]); SAL.mkdir(parents=True, exist_ok=True)
TEMA = sys.argv[2] if len(sys.argv) > 2 else "hud"
CH = glob.glob(str(Path.home() / ".cache/ms-playwright/chromium_headless_shell-*/chrome-linux/headless_shell"))[0]
TABS = ["tab-dashboard", "tab-asistido", "tab-galeria", "tab-nav", "tab-slam", "tab-ai", "tab-sim", "tab-motores", "tab-hw", "tab-params", "tab-sistema"]
NOMBRES = {"tab-dashboard": "01_dashboard", "tab-asistido": "02_manejo_asistido", "tab-galeria": "03_galeria", "tab-nav": "04_navegacion", "tab-slam": "05_slam_telemetria",
           "tab-ai": "06_modelos_ia", "tab-sim": "07_simulacion", "tab-motores": "08_motores_servo", "tab-hw": "09_config_hardware", "tab-params": "10_parametros", "tab-sistema": "11_sistema"}


async def main():
    p = subprocess.Popen([CH, "--no-sandbox", "--disable-gpu", "--hide-scrollbars", "--window-size=1500,920", "--remote-debugging-port=9333", "about:blank"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(40):
            try:
                tabs = json.load(urllib.request.urlopen("http://127.0.0.1:9333/json")); break
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
            await cdp("Page.enable")
            await cdp("Emulation.setDeviceMetricsOverride", width=1500, height=920, deviceScaleFactor=1, mobile=False)
            await cdp("Page.navigate", url="http://localhost:8080/")
            await asyncio.sleep(1.5)
            await cdp("Runtime.evaluate", expression=f"try{{localStorage.setItem('agrobot-tema','{TEMA}')}}catch(e){{}}")
            await cdp("Page.navigate", url="http://localhost:8080/")
            await asyncio.sleep(6)
            for t in TABS:
                await cdp("Runtime.evaluate", expression=f"(()=>{{const b=document.querySelector('[data-tab=\"{t}\"]'); if(b) b.click(); return !!b}})()")
                await asyncio.sleep(3.5)
                r = await cdp("Page.captureScreenshot", format="png")
                (SAL / f"{NOMBRES[t]}_{TEMA}.png").write_bytes(base64.b64decode(r["data"]))
                print("ok", NOMBRES[t])
            # pestaña larga: dashboard completo (scroll por dentro del .tab)
            await cdp("Runtime.evaluate", expression="document.querySelector('[data-tab=\"tab-dashboard\"]').click()")
            await asyncio.sleep(2)
            h = (await cdp("Runtime.evaluate", expression="(()=>{const t=document.getElementById('tab-dashboard'); return t.scrollHeight})()"))["result"]["value"]
            await cdp("Emulation.setDeviceMetricsOverride", width=1500, height=min(int(h) + 80, 4000), deviceScaleFactor=1, mobile=False)
            await asyncio.sleep(2.5)
            r = await cdp("Page.captureScreenshot", format="png")
            (SAL / f"00_dashboard_completo_{TEMA}.png").write_bytes(base64.b64decode(r["data"])); print("ok completo", h)
    finally:
        p.terminate()

asyncio.run(main())
