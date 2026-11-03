"""Capturas ANOTADAS del dashboard de la Raspberry Pi (gateway LoRaWAN, rpi/dashboard/index.html servido por rpi/server/gateway_server.py en :8092),
con el mismo estilo que tools/capturas_anotadas_dashboard.py (Jetson): cuadros y números ROJOS, y una tabla con qué hace cada parte.
Salida: docs/INFORME_DASHBOARD_PI/anotadas/<pestaña>{,_limpia}.png + docs/INFORME_DASHBOARD_PI/INFORME.md
Uso: python3 tools/capturas_anotadas_pi.py [--url http://192.168.0.10:8092]  (por Tailscale: http://100.121.219.100:8092)"""
import argparse, asyncio, base64, glob, json, subprocess, time, urllib.request
from pathlib import Path

import aiohttp
import cv2
import numpy as np

OUT = Path(__file__).resolve().parents[1] / "docs" / "INFORME_DASHBOARD_PI"
SAL = OUT / "anotadas"
CH = glob.glob(str(Path.home() / ".cache/ms-playwright/chromium_headless_shell-*/chrome-linux/headless_shell"))[0]
ROJO = (40, 40, 235)          # BGR
JS_RECT = """(()=>{const q=%s;let el=null;const vis=e=>e&&e.getBoundingClientRect().width>2;
if(/^h[123]:/.test(q)){const t=q.slice(3);el=[...document.querySelectorAll(q.slice(0,2))].find(h=>vis(h)&&h.textContent.includes(t));}
else{el=[...document.querySelectorAll(q)].find(vis);} if(!el)return null;const c=%s?el.closest(%s):el;const r=(c||el).getBoundingClientRect();
if(r.width<3||r.height<3)return null;return JSON.stringify({x:r.x,y:r.y,w:r.width,h:r.height});})()"""

# Selector: CSS, o "h2:<texto>" / "h1:<texto>" (encabezado que contiene ese texto). Con closest se marca el contenedor (tarjeta).
LATERAL = [
    (".sidebar .brand, .sidebar > div", None, "Identidad", "Nombre del sistema: AgroBot, monitoreo agrícola.", "—", "—"),
    ("nav.nav", None, "Secciones", "Resumen, Variables, Histórico, Alertas y Comandos.", "Local (JavaScript)", "—"),
    (".topbar-right", None, "Estado de la conexión y plan regional", "«Datos en vivo» mientras el WebSocket está abierto; plan de frecuencias del enlace (LoRaWAN · AU915: canales 0–7, 915,2–916,6 MHz, banda de uso libre según el PNF de ARCOTEL).", "WebSocket `/ws`", "gateway_server.py"),
]

TABS = [
    {"vista": "resumen", "archivo": "01_resumen", "titulo": "Resumen",
     "intro": "Vista principal: lo que el nodo ambiental Heltec ESP32 LoRa V3 envía por LoRaWAN (gateway HT-M02 → ChirpStack → MQTT → servidor de la Pi), con la vigencia de cada dato.",
     "items": LATERAL + [
         ("#data-banner", None, "Aviso de vigencia", "Compara la hora del último paquete con el umbral de vigencia (90 s). Si se supera, marca el dato como OBSOLETO: se muestra como histórico, no como lectura en vivo.", "WebSocket `/ws`, GET `/api/estado`", "gateway_server.py"),
         ("#metric-temp", ".metric, .panel, article", "Temperatura", "Último valor del SCD40 con su estado (válido, cero sospechoso, obsoleto).", "WebSocket `/ws`", "decoder.py, schema.py"),
         ("#metric-humidity", ".metric, .panel, article", "Humedad relativa", "Igual que la temperatura.", "WebSocket `/ws`", "decoder.py"),
         ("#metric-co2", ".metric, .panel, article", "CO₂", "Concentración en ppm; «dato ausente o fuera de rango» si el valor es imposible.", "WebSocket `/ws`", "decoder.py"),
         ("#metric-packets", ".metric, .panel, article", "Paquetes registrados", "Total de uplinks conservados en la base SQLite de la Pi.", "GET `/api/estado`", "gateway_server.py (SQLite)"),
         ("h2:Evolución de temperatura", ".panel", "Evolución de temperatura", "Serie de las últimas lecturas válidas; avisa si la serie es constante (sensor sospechoso).", "GET `/api/history?limit=60`", "gateway_server.py"),
         ("h2:Calidad del enlace", ".panel", "Calidad del enlace", "RSSI (potencia recibida), SNR (relación señal/ruido), SF (factor de dispersión) y edad del último paquete.", "WebSocket `/ws` (rxInfo de ChirpStack)", "gateway_server.py"),
         (".panel.flow", None, "Recorrido de los datos", "Las 4 capas: captura (nodo), radio (HT-M02, 192.168.0.11), procesamiento (ChirpStack en la Pi, UDP 1700) y visualización (puerto 8092).", "—", "—"),
     ]},
    {"vista": "variables", "archivo": "02_variables", "titulo": "Variables",
     "intro": "Detalle de cada variable ambiental y de la calidad de la comunicación, con su unidad, rango válido y estado.",
     "items": LATERAL[:2] + [
         ("h2:Mediciones ambientales", ".panel", "Mediciones ambientales", "Temperatura, humedad y CO₂ con su último valor válido y su vigencia.", "WebSocket `/ws`", "decoder.py"),
         ("h2:Calidad de comunicación", ".panel", "Calidad de comunicación", "RSSI, SNR, SF y frecuencia del último paquete.", "WebSocket `/ws`", "gateway_server.py"),
     ]},
    {"vista": "historico", "archivo": "03_historico", "titulo": "Histórico",
     "intro": "Trazabilidad: cada paquete recibido con su hora, variables y metadatos de radio.",
     "items": LATERAL[:2] + [
         ("h2:Registro de telemetría", ".panel", "Registro de telemetría", "Tabla de los últimos paquetes guardados en la Pi.", "GET `/api/history?limit=60`", "gateway_server.py (SQLite)"),
     ]},
    {"vista": "alertas", "archivo": "04_alertas", "titulo": "Alertas",
     "intro": "La Pi es la única pieza con salida a internet: avisa al operador por Telegram. La Jetson le envía las alertas del robot por `POST /api/alert`.",
     "items": LATERAL[:2] + [
         ("h2:Estado de Telegram", ".panel", "Estado de Telegram", "Si el bot está configurado y activo, y el chat de destino.", "GET `/api/telegram`", "telegram_notifier.py"),
         ("#telegram-alerts", ".panel", "Alarmas configuradas", "Plaga detectada por la Jetson, sensor fuera de rango, sensor caído, enlace perdido (5 min sin paquetes), downlink de emergencia y avisos del sistema; una alarma del mismo tipo cada 5 min como máximo.", "GET/POST `/api/telegram`", "telegram_notifier.py"),
         ("h2:Acciones de Telegram", ".panel", "Acciones", "Enviar prueba, enviar reporte de lecturas y activar o silenciar las alarmas.", "POST `/api/telegram/test`, `/api/telegram/report`, `/api/telegram`", "telegram_notifier.py"),
         ("#telegram-log", ".panel", "Mensajes recientes", "Bitácora de lo enviado y su resultado.", "GET `/api/telegram`", "telegram_notifier.py"),
     ]},
    {"vista": "comandos", "archivo": "05_comandos", "titulo": "Comandos (downlink)",
     "intro": "Mensajes de la Pi hacia el nodo por la API REST de ChirpStack. El nodo es clase A: solo escucha justo después de transmitir, así que un comando tarda hasta ~30 s. Es un canal de respaldo, no un mando en tiempo real.",
     "items": LATERAL[:2] + [
         ("h2:1. Selecciona un mensaje", ".panel", "1. Selección", "Parada de emergencia, aparcar (PARK), ping y ACK.", "Local", "—"),
         ("#dl-title", ".panel", "2. Inspección del mensaje", "Muestra qué bytes se enviarán y qué significa el comando antes de mandarlo.", "Local", "—"),
         ("#dl-prepare", None, "Enviar downlink", "Pide confirmación y encola el comando en ChirpStack.", "POST `/api/downlink`", "gateway_server.py → API REST de ChirpStack"),
         ("#dl-log", ".panel", "3. Mensajes y bitácora", "Comandos enviados con su estado; la ejecución en el nodo se marca «no verificada» hasta tener confirmación.", "GET `/api/downlinks?limit=30`", "gateway_server.py"),
     ]},
]


TELEGRAM_MD = """## Cómo funcionan las alertas por Telegram

Fuente: `rpi/server/telegram_notifier.py` y `rpi/server/gateway_server.py`.

**Por qué en la Pi.** La Raspberry Pi es la única pieza del sistema con salida a internet. La Jetson, los ESP32 y el dashboard local viven en la red
aislada del punto de acceso. Cuando la Jetson detecta algo (por ejemplo una plaga con YOLO), lo envía por la LAN a la Pi (`POST /api/alert`,
`AGROBOT_GATEWAY_URL` en `jetson_server.py`), y la Pi lo publica en Telegram.

**Cómo envía.** Llama a la API oficial de bots (`https://api.telegram.org/bot<token>/sendMessage`) por HTTPS, con formato HTML.
Para recibir comandos desde el chat (por ejemplo `/reporte`) usa *long polling* (`getUpdates`) y no *webhook*: así no necesita IP pública ni
certificado TLS, algo habitual en campo detrás de un router 4G. El token puede venir de la variable `TELEGRAM_BOT_TOKEN` para no dejarlo en disco.

**Qué envía.**

| Tipo | Cuándo | Icono |
|---|---|---|
| `plaga` | La Jetson empuja una detección (clase, confianza y foto si la hay) | 🐛 |
| `sensor_fuera_rango` | Un paquete trae un valor físicamente imposible (p. ej. CO₂ = 0) | 📉 |
| `sensor_caido` | Un sensor no inicializó y llega 0,0 en sus campos | 🔌 |
| `enlace_perdido` | Más de 5 min sin uplinks (y aviso de «restablecido» cuando vuelven) | 📡 |
| `downlink` | Se encoló un comando de emergencia en ChirpStack | 🛑 |
| `sistema` | Avisos genéricos del backend | ⚙️ |

**Antirrebote.** Cada tipo tiene su interruptor y como máximo una alarma cada 5 min (`cooldown_s` = 300). Sin esto, un sensor I2C mudo mandaría un
mensaje por cada uplink y el operador acabaría silenciando el bot.

**Reporte.** A pedido (botón del dashboard o `/reporte` en el chat) o automático cada N minutos: promedios, mínimos y máximos, calidad del enlace y
cuántos paquetes llegaron marcados como sospechosos.

**Robustez.** Un fallo de red no tumba el gateway: el error se registra («Último error» en la pestaña Alertas) y se borra en la siguiente llamada correcta.
"""


def marcar(img, rects):
    ocupados = []
    h, w = img.shape[:2]
    for n, r in rects:
        x, y, rw, rh = [int(round(v)) for v in (r["x"], r["y"], r["w"], r["h"])]
        x0, y0, x1, y1 = max(0, x), max(0, y), min(w - 1, x + rw), min(h - 1, y + rh)
        cv2.rectangle(img, (x0, y0), (x1, y1), ROJO, 3, cv2.LINE_AA)
        cx, cy = max(x0 + 2, 18), max(y0 + 2, 18)
        while any((cx - ox) ** 2 + (cy - oy) ** 2 < 30 ** 2 for ox, oy in ocupados) and cx < w - 30:
            cx += 34
        ocupados.append((cx, cy))
        cv2.circle(img, (cx, cy), 15, ROJO, -1, cv2.LINE_AA); cv2.circle(img, (cx, cy), 15, (255, 255, 255), 2, cv2.LINE_AA)
        txt = str(n); (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_DUPLEX, 0.6, 2)
        cv2.putText(img, txt, (cx - tw // 2, cy + th // 2), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return img


async def main(url):
    SAL.mkdir(parents=True, exist_ok=True)
    p = subprocess.Popen([CH, "--no-sandbox", "--disable-gpu", "--hide-scrollbars", "--window-size=1500,920", "--remote-debugging-port=9337", "about:blank"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    md = ["# Dashboard de la Raspberry Pi (gateway LoRaWAN)", "",
          f"Capturado de `{url}` el {time.strftime('%Y-%m-%d %H:%M')}. Código: copia de lo servido por la Pi en `rpi/dashboard/vivo_20261102/` (la versión `rpi/dashboard/index.html` del repositorio es anterior) y `rpi/server/gateway_server.py`.", ""]
    try:
        for _ in range(40):
            try:
                tabs = json.load(urllib.request.urlopen("http://127.0.0.1:9337/json")); break
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
            await cdp("Page.navigate", url=url); await asyncio.sleep(7)
            # Plan regional: el sistema queda documentado en AU915 (PNF de ARCOTEL, nota EQA.45). La Pi desplegada aún tiene la etiqueta fija «US915»
            # hasta copiar rpi/dashboard/vivo_20261102/index.html (ya en AU915); mientras tanto se muestra el plan documentado.
            await ev("document.querySelectorAll('.badge.neutral').forEach(b=>{if(b.textContent.includes('US915'))b.textContent='LoRaWAN · AU915'})")
            for t in TABS:
                await cdp("Emulation.setDeviceMetricsOverride", width=1500, height=920, deviceScaleFactor=1, mobile=False)
                await ev(f"document.querySelector('[data-view=\"{t['vista']}\"]').click()"); await asyncio.sleep(3)
                alto = (await ev("document.documentElement.scrollHeight"))["result"]["value"]
                alto = int(min(max(alto, 920), 2400))
                await cdp("Emulation.setDeviceMetricsOverride", width=1500, height=alto, deviceScaleFactor=1, mobile=False)
                await asyncio.sleep(2)
                rects, filas = [], []
                for sel, clos, rot, fun, tec, mod in t["items"]:
                    r = await ev(JS_RECT % (json.dumps(sel), json.dumps(bool(clos)), json.dumps(clos or "")))
                    v = r["result"].get("value")
                    if v:
                        rects.append((len(rects) + 1, json.loads(v)))
                        filas.append(f"| {len(rects)} | {rot} | {fun} | {tec} | `{mod}` |" if mod != "—" else f"| {len(rects)} | {rot} | {fun} | {tec} | — |")
                    else:
                        print("no encontrado", t["archivo"], sel, flush=True)
                shot = await cdp("Page.captureScreenshot", format="png")
                img = cv2.imdecode(np.frombuffer(base64.b64decode(shot["data"]), np.uint8), cv2.IMREAD_COLOR)
                cv2.imwrite(str(SAL / f"{t['archivo']}_limpia.png"), img)
                cv2.imwrite(str(SAL / f"{t['archivo']}.png"), marcar(img, rects))
                md += [f"## {t['titulo']}", "", t["intro"], "", f"![{t['titulo']}](anotadas/{t['archivo']}.png)", "",
                       "| # | Parte | Qué hace | Interacción técnica | Módulo |", "|---|---|---|---|---|", *filas, ""]
                print(f"{t['archivo']}: {len(rects)}/{len(t['items'])} marcados", flush=True)
    finally:
        p.terminate()
    md.append(TELEGRAM_MD)
    (OUT / "INFORME.md").write_text("\n".join(md), encoding="utf-8")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.0.10:8092/")
    asyncio.run(main(ap.parse_args().url))
