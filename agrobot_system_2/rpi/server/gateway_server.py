"""
AgroBot — servidor de la Raspberry Pi (`gateway_server.py`).
=============================================================
Gateway LoRaWAN / ChirpStack.

El nombre del archivo importa: antes este módulo y el backend de la Jetson
se llamaban los DOS `server.py`. Como Python importa por NOMBRE de módulo
(no por ruta), un `import server` cargaba el que estuviera primero en
sys.path — el archivo equivocado, sin ningún aviso. Ahora cada uno tiene un
nombre único: `gateway_server.py` acá y `jetson_server.py` en la Jetson.

Rol: puente entre ChirpStack (que ya recibe los uplinks del gateway
HT-M02) y un dashboard remoto propio, más el envío de downlinks de
emergencia. NO reemplaza a ChirpStack — se conecta a él por MQTT para
enterarse de cada paquete y por su API REST para mandar downlinks.

  - Cliente MQTT  -> suscrito al broker de ChirpStack (mosquitto local),
                      tópico application/<app_id>/device/<dev_eui>/event/up
  - SQLite        -> histórico de paquetes (independiente de la base de
                      ChirpStack, para las gráficas/tabla del dashboard)
  - WebSocket /ws -> push inmediato al dashboard remoto
  - REST /api/*   -> histórico, export CSV, envío de downlinks, alarmas
  - Telegram      -> alarmas y reportes al celular del operador
                     (telegram_notifier.py). Esta es la ÚNICA pieza del
                     sistema con salida a internet: la Jetson y el robot
                     viven en la red aislada del AP, así que le empujan sus
                     eventos acá por LAN (POST /api/alert) y el gateway los
                     publica.

Downlinks van por la API REST de ChirpStack (`/api/devices/{dev_eui}/queue`)
— nunca directo al dispositivo, LoRaWAN no funciona así.
"""
import asyncio
import base64
import csv
import io
import json
import os
import sqlite3
import time
from pathlib import Path

import aiohttp
from aiohttp import web, WSMsgType
import paho.mqtt.client as mqtt

from decoder import decode
import schema
from telegram_notifier import TelegramNotifier

# =============================================
# ===== CONFIG (ajustar a tu instancia de ChirpStack) =====
# =============================================
MQTT_HOST = os.environ.get("MQTT_HOST", "localhost")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
# El application_id de ChirpStack v4 es un UUID, no un entero: el "1" que
# usaba v3 acá produce el tópico `application/1/device/+/event/up`, que no
# coincide con nada. El cliente MQTT se suscribe sin error y no llega jamás
# un paquete — indistinguible de que el robot no esté transmitiendo.
CHIRPSTACK_APP_ID = os.environ.get(
    "CHIRPSTACK_APP_ID", "aa4c5278-0d39-4833-be36-286895ffa4c4")
MQTT_TOPIC = f"application/{CHIRPSTACK_APP_ID}/device/+/event/up"

# 8090, no 8080: en el despliegue con Docker la REST API es un contenedor
# aparte (chirpstack-rest-api). El 8080 sirve SOLO la interfaz web, y pegarle
# ahí devuelve 404 en todos los /api/... — es también el motivo por el que
# este gateway escucha en 8092 y no en 8090.
CHIRPSTACK_API_URL = os.environ.get("CHIRPSTACK_API_URL", "http://localhost:8090")
CHIRPSTACK_API_TOKEN = os.environ.get("CHIRPSTACK_API_TOKEN", "")  # API key de ChirpStack v4
DEV_EUI = os.environ.get("AGROBOT_DEV_EUI", "f59d08345985b27d")  # de CARRITO2_LORAWAN_STATUS.md

HTTP_PORT = 8092
DB_PATH = Path(__file__).parent / "agrobot_history.db"

# Comandos de downlink (1 byte, puerto de aplicación 10 — distinto del
# puerto 2 que usa el uplink de sensores, para no mezclarlos)
DOWNLINK_PORT = 10
DOWNLINK_CODES = {
    "EMERGENCY_STOP": 0x01,
    "PARK": 0x02,
    "PING": 0x03,
    "ACK": 0x04,
}


# =============================================
# ===== BASE DE DATOS =====
# =============================================
def init_db():
    con = sqlite3.connect(DB_PATH)
    con.execute("""
        CREATE TABLE IF NOT EXISTS packets (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            received_at REAL,
            dev_eui TEXT,
            rssi INTEGER,
            snr REAL,
            sf INTEGER,
            co2 REAL, temp_scd REAL, humidity REAL,
            accel_x REAL, accel_y REAL, accel_z REAL,
            gyro_x REAL, gyro_y REAL, gyro_z REAL,
            out_of_range TEXT,
            sensors_missing TEXT,
            schema_version INTEGER DEFAULT 1
        )
    """)
    # Migración para bases creadas antes de que existiera `sensors_missing`:
    # SQLite no tiene "ADD COLUMN IF NOT EXISTS", así que se consulta primero
    # qué columnas hay. Sin esto, actualizar el código contra un histórico ya
    # existente rompía todos los INSERT.
    existing = {row[1] for row in con.execute("PRAGMA table_info(packets)")}
    if "sensors_missing" not in existing:
        con.execute("ALTER TABLE packets ADD COLUMN sensors_missing TEXT")
    con.execute("""
        CREATE TABLE IF NOT EXISTS downlinks (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            sent_at REAL,
            command TEXT,
            delivered INTEGER DEFAULT 0
        )
    """)
    # Último estado conocido. Es UNA fila (id=1) que se sobrescribe con cada
    # uplink, no un histórico —para eso ya está `packets`.
    #
    # Existe porque `state.last_packet` vivía solo en memoria: al reiniciar el
    # gateway se perdía y el dashboard quedaba en blanco hasta el siguiente
    # paquete, que con un ciclo de 30 s son 30 segundos de pantalla vacía, y
    # si el robot está apagado son para siempre. Con un mapa y un histórico
    # llenos de datos detrás, esa pantalla vacía se lee como "el sistema no
    # funciona".
    #
    # Se guarda el paquete entero como JSON a propósito, en vez de una
    # columna por campo: así añadir un sensor al payload no obliga a migrar
    # esta tabla. `packets` sí tiene columnas porque sobre ella se consulta
    # y se grafica; sobre esta solo se lee la última fila.
    con.execute("""
        CREATE TABLE IF NOT EXISTS estado_actual (
            id INTEGER PRIMARY KEY CHECK (id = 1),
            updated_at REAL,
            packet_json TEXT,
            packet_count INTEGER DEFAULT 0
        )
    """)
    con.commit()
    con.close()


def guardar_estado(row: dict, packet_count: int):
    """Sobrescribe el último estado conocido. Se llama en cada uplink."""
    con = sqlite3.connect(DB_PATH)
    con.execute("""
        INSERT INTO estado_actual (id, updated_at, packet_json, packet_count)
        VALUES (1, ?, ?, ?)
        ON CONFLICT(id) DO UPDATE SET
            updated_at = excluded.updated_at,
            packet_json = excluded.packet_json,
            packet_count = excluded.packet_count
    """, (row.get("received_at", time.time()), json.dumps(row), packet_count))
    con.commit()
    con.close()


def cargar_estado() -> tuple[dict, int]:
    """Recupera el último estado al arrancar. Devuelve ({}, 0) si no hay nada.

    Nunca lanza: un histórico corrupto o de una versión anterior no puede
    impedir que el gateway arranque. Como mucho se pierde el último dato,
    que es exactamente la situación que había antes de existir esta tabla.
    """
    try:
        con = sqlite3.connect(DB_PATH)
        fila = con.execute(
            "SELECT packet_json, packet_count FROM estado_actual WHERE id = 1"
        ).fetchone()
        con.close()
        if not fila or not fila[0]:
            return {}, 0
        return json.loads(fila[0]), int(fila[1] or 0)
    except Exception as e:
        print(f"[estado] no se pudo recuperar el último estado ({e}), se empieza vacío")
        return {}, 0


def insert_packet(row: dict):
    con = sqlite3.connect(DB_PATH)
    con.execute("""
        INSERT INTO packets (received_at, dev_eui, rssi, snr, sf,
            co2, temp_scd, humidity,
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
            out_of_range, sensors_missing, schema_version)
        VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
    """, (
        row["received_at"], row["dev_eui"], row.get("rssi"), row.get("snr"), row.get("sf"),
        row["co2"], row["tempSCD"], row["humidity"],
        row["accelX"], row["accelY"], row["accelZ"], row["gyroX"], row["gyroY"], row["gyroZ"],
        json.dumps(row.get("_out_of_range", [])),
        json.dumps(row.get("_sensors_missing", [])), schema.SCHEMA_VERSION,
    ))
    con.commit()
    con.close()


def _row_to_packet(row: dict) -> dict:
    """Traduce una fila de SQLite (columnas snake_case) al mismo formato
    camelCase que ya usa el paquete en vivo (`decode()`/`schema.validate()`),
    para que el frontend no tenga que manejar dos convenciones de nombres
    distintas según si el dato viene del histórico o de MQTT en vivo."""
    return {
        "id": row["id"],
        "received_at": row["received_at"],
        "dev_eui": row["dev_eui"],
        "rssi": row["rssi"],
        "snr": row["snr"],
        "sf": row["sf"],
        "co2": row["co2"],
        "tempSCD": row["temp_scd"],
        "humidity": row["humidity"],
        "accelX": row["accel_x"],
        "accelY": row["accel_y"],
        "accelZ": row["accel_z"],
        "gyroX": row["gyro_x"],
        "gyroY": row["gyro_y"],
        "gyroZ": row["gyro_z"],
        "_out_of_range": json.loads(row["out_of_range"]) if row["out_of_range"] else [],
        "_sensors_missing": json.loads(row["sensors_missing"]) if row.get("sensors_missing") else [],
        "schema_version": row["schema_version"],
    }


def fetch_history(limit=200) -> list[dict]:
    con = sqlite3.connect(DB_PATH)
    con.row_factory = sqlite3.Row
    rows = con.execute("SELECT * FROM packets ORDER BY id DESC LIMIT ?", (limit,)).fetchall()
    con.close()
    return [_row_to_packet(dict(r)) for r in rows]


def insert_downlink(command: str):
    con = sqlite3.connect(DB_PATH)
    con.execute("INSERT INTO downlinks (sent_at, command) VALUES (?, ?)", (time.time(), command))
    con.commit()
    con.close()


def count_packets() -> int:
    con = sqlite3.connect(DB_PATH)
    total = con.execute("SELECT COUNT(*) FROM packets").fetchone()[0]
    con.close()
    return total


def fetch_downlinks(limit=50) -> list[dict]:
    con = sqlite3.connect(DB_PATH)
    con.row_factory = sqlite3.Row
    rows = con.execute("SELECT * FROM downlinks ORDER BY id DESC LIMIT ?", (limit,)).fetchall()
    con.close()
    return [dict(r) for r in rows]


# =============================================
# ===== ESTADO COMPARTIDO =====
# =============================================
class SharedState:
    def __init__(self):
        self.last_packet = {}
        self.packet_count = 0
        self.ws_clients: set[web.WebSocketResponse] = set()
        self.loop: asyncio.AbstractEventLoop | None = None

    async def broadcast(self, message: dict):
        if not self.ws_clients:
            return
        payload = json.dumps(message)
        dead = []
        for ws in self.ws_clients:
            try:
                await ws.send_str(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.ws_clients.discard(ws)

    def broadcast_threadsafe(self, message: dict):
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.broadcast(message), self.loop)


state = SharedState()

# Puente con Telegram: única pieza que sale a internet. Se le pasan las
# funciones de lectura del histórico en vez de que importe este módulo,
# para no crear un import circular.
telegram = TelegramNotifier(fetch_history=fetch_history,
                            fetch_downlinks=fetch_downlinks)


# =============================================
# ===== CLIENTE MQTT (ChirpStack uplinks) =====
# =============================================
def on_mqtt_connect(client, userdata, flags, rc, properties=None):
    print(f"[MQTT] conectado a {MQTT_HOST}:{MQTT_PORT}, rc={rc}")
    client.subscribe(MQTT_TOPIC)
    print(f"[MQTT] suscrito a {MQTT_TOPIC}")


def on_mqtt_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
    except json.JSONDecodeError:
        return

    b64_data = payload.get("data")
    if not b64_data:
        return
    raw = base64.b64decode(b64_data)
    decoded = decode(raw)
    if decoded is None:
        print(f"[MQTT] payload de tamaño inesperado ({len(raw)} bytes), se ignora")
        return
    decoded = schema.validate(decoded)
    if decoded["_out_of_range"]:
        print(f"[MQTT] ⚠️ campos fuera de rango físico esperado: {decoded['_out_of_range']}")

    rx_info = (payload.get("rxInfo") or [{}])[0]
    tx_info = payload.get("txInfo") or {}

    row = {
        **decoded,
        "received_at": time.time(),
        "dev_eui": payload.get("deviceInfo", {}).get("devEui", DEV_EUI),
        "rssi": rx_info.get("rssi"),
        "snr": rx_info.get("snr"),
        "sf": (tx_info.get("modulation", {}).get("lora", {}) or {}).get("spreadingFactor"),
    }
    insert_packet(row)
    state.last_packet = row
    state.packet_count += 1
    guardar_estado(row, state.packet_count)

    state.broadcast_threadsafe({"type": "packet", "data": row, "packet_count": state.packet_count})

    # on_packet crea tareas asyncio, así que TIENE que ejecutarse dentro del
    # event loop — este callback corre en el hilo de paho-mqtt, donde
    # `asyncio.create_task` lanzaría "no running event loop".
    if state.loop:
        state.loop.call_soon_threadsafe(telegram.on_packet, row)


def on_mqtt_disconnect(client, userdata, *args):
    # la firma cambió entre paho v1 y v2, por eso *args
    print(f"[MQTT] desconectado de {MQTT_HOST}:{MQTT_PORT} — reintentando en segundo plano")


def start_mqtt_client():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) if hasattr(mqtt, "CallbackAPIVersion") else mqtt.Client()
    client.on_connect = on_mqtt_connect
    client.on_message = on_mqtt_message
    client.on_disconnect = on_mqtt_disconnect

    # connect_async + loop_start en vez de connect(): `connect()` es
    # bloqueante y LANZA si el broker todavía no está escuchando, lo que
    # mataba el proceso entero con un traceback. En la Raspberry Pi real eso
    # es una carrera de arranque de verdad: si systemd levanta este servicio
    # antes que Mosquitto, el gateway moría para siempre en vez de esperar.
    # Con connect_async el hilo de red reintenta solo, y mientras tanto el
    # dashboard y el histórico siguen sirviéndose (los datos que ya están en
    # SQLite se pueden consultar aunque no entren paquetes nuevos).
    client.reconnect_delay_set(min_delay=1, max_delay=30)
    client.connect_async(MQTT_HOST, MQTT_PORT, keepalive=30)
    client.loop_start()
    print(f"[MQTT] conectando a {MQTT_HOST}:{MQTT_PORT} en segundo plano…")
    return client


# =============================================
# ===== WEBSOCKET =====
# =============================================
async def ws_handler(request: web.Request):
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    state.ws_clients.add(ws)

    await ws.send_str(json.dumps({
        "type": "snapshot",
        "data": {
            "last_packet": state.last_packet,
            "packet_count": state.packet_count,
            "history": fetch_history(60),
            "downlinks": fetch_downlinks(20),
            # El snapshot puede traer un dato recuperado del disco de hace
            # horas. Sin esto el dashboard lo pintaría idéntico a uno en vivo.
            "estado": _antiguedad(state.last_packet),
        },
    }))

    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                pass  # este dashboard es de solo lectura + downlinks (ver /api/downlink)
    finally:
        state.ws_clients.discard(ws)
    return ws


# =============================================
# ===== DOWNLINKS (ChirpStack REST API) =====
# =============================================
async def send_downlink_via_chirpstack(command: str) -> bool:
    if command not in DOWNLINK_CODES:
        return False
    if not CHIRPSTACK_API_TOKEN:
        print("[downlink] CHIRPSTACK_API_TOKEN no configurado, no se puede enviar")
        return False

    payload_b64 = base64.b64encode(bytes([DOWNLINK_CODES[command]])).decode()
    url = f"{CHIRPSTACK_API_URL}/api/devices/{DEV_EUI}/queue"
    # La clave es `queueItem`, no `deviceQueueItem`: ese era el nombre en la
    # API de ChirpStack v3 y en v4 cambió. El fallo es especialmente
    # engañoso porque no dice nada del nombre: al no reconocer la clave,
    # ChirpStack recibe un item vacío, ve fPort = 0 y responde
    # "FPort must be between 1 - 255" — un mensaje que manda a revisar el
    # puerto, que era justamente lo único que estaba bien.
    body = {
        "queueItem": {
            "confirmed": True,
            "data": payload_b64,
            "devEui": DEV_EUI,
            "fPort": DOWNLINK_PORT,
        }
    }
    headers = {"Authorization": f"Bearer {CHIRPSTACK_API_TOKEN}", "Content-Type": "application/json"}

    async with aiohttp.ClientSession() as session:
        async with session.post(url, json=body, headers=headers) as resp:
            ok = resp.status < 300
            if not ok:
                print(f"[downlink] error {resp.status}: {await resp.text()}")
            return ok


async def downlink_handler(request: web.Request):
    body = await request.json()
    command = body.get("command")
    ok = await send_downlink_via_chirpstack(command)
    insert_downlink(command)
    await state.broadcast({"type": "downlink_sent", "data": {"command": command, "ok": ok, "sent_at": time.time()}})
    telegram.on_downlink(command, ok)
    return web.json_response({"ok": ok, "command": command})


# =============================================
# ===== HISTÓRICO / EXPORT =====
# =============================================
async def history_handler(request: web.Request):
    limit = int(request.query.get("limit", "200"))
    return web.json_response(fetch_history(limit))


async def export_csv_handler(request: web.Request):
    rows = fetch_history(5000)
    buf = io.StringIO()
    if rows:
        writer = csv.DictWriter(buf, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    return web.Response(
        text=buf.getvalue(),
        content_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=agrobot_history.csv"},
    )


# =============================================
# ===== TELEGRAM =====
# =============================================
async def telegram_estado_handler(request: web.Request):
    return web.json_response(telegram.estado())


async def telegram_config_handler(request: web.Request):
    return web.json_response(telegram.actualizar_config(await request.json()))


async def telegram_test_handler(request: web.Request):
    ok = await telegram.enviar(
        "✅ <b>Prueba de AgroBot</b>\nSi ves este mensaje, el gateway de la "
        "Raspberry Pi puede avisarte de lo que pase en el cultivo.", "prueba")
    return web.json_response({"ok": ok, "error": telegram.last_error})


async def telegram_report_handler(request: web.Request):
    """Genera el reporte. `?enviar=0` solo lo devuelve para previsualizarlo
    en el dashboard, sin gastar un mensaje en el chat."""
    enviar = request.query.get("enviar", "1") != "0"
    texto = telegram.construir_reporte()
    ok = await telegram.enviar(texto, "reporte") if enviar else True
    return web.json_response({"ok": ok, "texto": texto, "enviado": enviar,
                              "error": telegram.last_error})


async def alert_handler(request: web.Request):
    """Ingesta de alarmas desde la Jetson (plagas detectadas por YOLO).

    La Jetson no tiene salida a internet: está dentro de la red aislada del
    AP. Empuja el evento acá por la LAN y el gateway lo publica en Telegram.
    """
    body = await request.json()
    enviado = await telegram.on_alerta_externa(body)
    await state.broadcast({"type": "alerta_externa", "data": body})
    return web.json_response({"ok": True, "telegram_enviado": enviado})


# =============================================
# ===== ESTADO ACTUAL =====
# =============================================
# Un dato viejo mostrado como si fuera de ahora es peor que no mostrar nada:
# el operador toma decisiones sobre una lectura que ya no describe el campo.
# Por eso todo lo que sale de aquí lleva su antigüedad al lado, y un umbral
# ya calculado en el servidor para que el frontend no tenga que replicar la
# regla (y no se desincronicen cuando cambie el ciclo de transmisión).
#
# El ciclo del robot es de 30 s (appTxDutyCycle en el firmware). Se da por
# fresco hasta 3 ciclos: uno perdido es normal en LoRa, tres seguidos ya no.
CICLO_TX_SEG = 30
UMBRAL_OBSOLETO_SEG = CICLO_TX_SEG * 3


def _antiguedad(row: dict) -> dict:
    if not row:
        return {"hay_dato": False, "obsoleto": True, "edad_seg": None}
    edad = time.time() - row.get("received_at", 0)
    return {
        "hay_dato": True,
        "edad_seg": round(edad, 1),
        "obsoleto": edad > UMBRAL_OBSOLETO_SEG,
        "umbral_seg": UMBRAL_OBSOLETO_SEG,
    }


async def estado_handler(request: web.Request):
    """Último dato conocido, con su antigüedad.

    Existe además del WebSocket porque no todo lo que quiere leer el estado
    es un navegador con sesión abierta: un script, el bot de Telegram o una
    comprobación de arranque necesitan una respuesta puntual sin mantener
    una conexión viva.
    """
    return web.json_response({
        "last_packet": state.last_packet,
        "packet_count": state.packet_count,
        **_antiguedad(state.last_packet),
    })


# =============================================
# ===== HISTOGRAMAS =====
# =============================================
# Un histograma responde una pregunta que ni el valor actual ni la serie
# temporal contestan: qué valores toma normalmente este sensor. Con eso a la
# vista, un dato raro se reconoce al instante como raro, sin tener que
# acordarse de cuál era el rango típico.
#
# Se calcula en el servidor y no en el navegador por dos motivos. Uno: el
# dashboard solo recibe las últimas 60 filas, y 60 muestras no describen una
# distribución. Dos: la Raspberry ya tiene el dato indexado y el cálculo es
# una pasada; mandar miles de filas por WebSocket para que el navegador las
# recorra sería mover el trabajo al aparato más lento de los dos.

# Columna de SQLite por cada campo que el frontend puede pedir. Es una lista
# blanca explícita, no una interpolación del nombre recibido: el parámetro
# entra directo en el SQL y aceptar texto libre ahí sería inyección.
CAMPOS_HISTOGRAMA = {
    "co2": ("co2", "CO₂", "ppm"),
    "tempSCD": ("temp_scd", "Temperatura", "°C"),
    "humidity": ("humidity", "Humedad", "%"),
    "rssi": ("rssi", "RSSI", "dBm"),
    "snr": ("snr", "SNR", "dB"),
    "accelZ": ("accel_z", "Aceleración Z", "g"),
    "gyroX": ("gyro_x", "Giro X", "°/s"),
}


def calcular_histograma(campo: str, horas: float, bins: int) -> dict:
    columna, etiqueta, unidad = CAMPOS_HISTOGRAMA[campo]
    desde = time.time() - horas * 3600

    # Se excluyen las muestras cuyo sensor no respondió. Un histograma que
    # mezcla lecturas reales con los ceros de un sensor caído no describe el
    # campo, describe la avería: era exactamente el caso del CO₂, con una
    # montaña en 0 y otra en 610 ppm. La avería ya se reporta aparte, en
    # `_sensors_missing`; aquí estorba.
    con = sqlite3.connect(DB_PATH)
    filas = con.execute(
        f"SELECT {columna}, sensors_missing FROM packets "
        f"WHERE received_at >= ? AND {columna} IS NOT NULL",
        (desde,)
    ).fetchall()
    con.close()

    valores, descartadas = [], 0
    for valor, ausentes_json in filas:
        try:
            ausentes = json.loads(ausentes_json) if ausentes_json else []
        except (ValueError, TypeError):
            ausentes = []
        if campo in ausentes:
            descartadas += 1
            continue
        valores.append(valor)

    base = {"campo": campo, "etiqueta": etiqueta, "unidad": unidad,
            "horas": horas, "n": len(valores),
            "descartadas": descartadas, "bins": []}
    if not valores:
        return base

    lo, hi = min(valores), max(valores)
    orden = sorted(valores)
    base.update({
        "min": lo, "max": hi,
        "media": sum(valores) / len(valores),
        "mediana": orden[len(orden) // 2],
    })

    # Sensor clavado en un solo valor: el caso de los sensores que no
    # responden y mandan 0.0 siempre. Con rango cero el ancho de bin sería 0
    # y la división reventaría; además un histograma de una sola barra no
    # dice nada que no diga el número. Se devuelve un bin único y marcado
    # para que el frontend lo explique en vez de dibujar una barra sola.
    if hi == lo:
        base["bins"] = [{"desde": lo, "hasta": lo, "n": len(valores)}]
        base["constante"] = True
        return base

    ancho = (hi - lo) / bins
    cuentas = [0] * bins
    for v in valores:
        # El máximo cae justo en el borde superior y daría índice `bins`,
        # fuera de la lista: se mete en el último bin.
        i = min(int((v - lo) / ancho), bins - 1)
        cuentas[i] += 1

    base["bins"] = [
        {"desde": lo + i * ancho, "hasta": lo + (i + 1) * ancho, "n": c}
        for i, c in enumerate(cuentas)
    ]
    base["constante"] = False
    return base


async def histograma_handler(request: web.Request):
    campo = request.query.get("campo", "co2")
    if campo not in CAMPOS_HISTOGRAMA:
        return web.json_response(
            {"error": f"campo desconocido: {campo}",
             "disponibles": sorted(CAMPOS_HISTOGRAMA)}, status=400)
    try:
        horas = max(0.1, min(float(request.query.get("horas", "24")), 24 * 30))
        bins = max(4, min(int(request.query.get("bins", "20")), 60))
    except ValueError:
        return web.json_response({"error": "horas y bins deben ser números"}, status=400)
    return web.json_response(calcular_histograma(campo, horas, bins))


# =============================================
# ===== APP =====
# =============================================
def create_app() -> web.Application:
    init_db()
    app = web.Application()
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/api/estado", estado_handler)
    app.router.add_get("/api/histograma", histograma_handler)
    app.router.add_get("/api/history", history_handler)
    app.router.add_get("/api/history/export.csv", export_csv_handler)
    app.router.add_post("/api/downlink", downlink_handler)
    app.router.add_get("/api/telegram", telegram_estado_handler)
    app.router.add_post("/api/telegram", telegram_config_handler)
    app.router.add_post("/api/telegram/test", telegram_test_handler)
    app.router.add_post("/api/telegram/report", telegram_report_handler)
    app.router.add_post("/api/alert", alert_handler)
    # La raíz sirve el dashboard directamente. Antes `add_static` con
    # `show_index=True` devolvía un LISTADO DE ARCHIVOS en "/": había que
    # entrar a /index.html a mano, y quien llegaba a la raíz veía una lista
    # con un solo enlace y creía que el panel estaba roto.
    #
    # El navegador además cachea agresivamente el HTML, así que se manda
    # `no-cache`: sin eso, una versión vieja del panel sigue apareciendo
    # después de actualizar el archivo en la Pi, y el síntoma —"me sale el
    # dashboard antiguo"— no tiene nada que ver con el servidor.
    DASHBOARD = Path(__file__).parent.parent / "dashboard"

    async def index_handler(request: web.Request):
        return web.FileResponse(
            DASHBOARD / "index.html",
            headers={"Cache-Control": "no-cache, must-revalidate"})

    app.router.add_get("/", index_handler)
    app.router.add_get("/index.html", index_handler)
    app.router.add_static("/", DASHBOARD, show_index=False)

    async def on_startup(app):
        state.loop = asyncio.get_running_loop()
        # El contador es HISTÓRICO, no "desde que arrancó el proceso": si no
        # se siembra desde la base, el panel muestra "Total 0" con meses de
        # datos guardados y parece que el gateway no recibió nunca nada.
        state.packet_count = count_packets()

        # Último estado conocido, para que el dashboard pinte algo desde el
        # primer segundo en vez de esperar al siguiente uplink. Se marca
        # explícitamente de dónde vino: un dato recuperado del disco NO es
        # lo mismo que un dato en vivo, y presentarlo igual sería peor que
        # no mostrarlo — el operador vería una lectura de ayer creyendo que
        # es de ahora. La antigüedad viaja con el dato (ver `_antiguedad`).
        guardado, _ = cargar_estado()
        if guardado:
            state.last_packet = guardado
            edad = time.time() - guardado.get("received_at", 0)
            print(f"[estado] último dato recuperado del disco ({edad/60:.1f} min de antigüedad)")
        else:
            print("[estado] sin dato previo guardado: se espera al primer uplink")

        await telegram.start()
        app["mqtt_client"] = start_mqtt_client()

    async def on_cleanup(app):
        client = app.get("mqtt_client")
        if client:
            client.loop_stop()
        await telegram.stop()

    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    return app


if __name__ == "__main__":
    web.run_app(create_app(), host="0.0.0.0", port=HTTP_PORT)
