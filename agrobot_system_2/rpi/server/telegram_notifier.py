"""
AgroBot — puente con Telegram (`telegram_notifier.py`).
========================================================
Vive en la Raspberry Pi, junto al gateway. Es la ÚNICA pieza del sistema
que habla con internet: todo lo demás (Jetson, ESP32, dashboard local)
corre dentro de la red aislada del AP y no tiene salida. Por eso el aviso
al operador sale desde acá y no desde la Jetson — la Jetson le empuja el
evento al gateway por la LAN (POST /api/alert) y el gateway lo publica.

Hace dos cosas:

  1. ALARMAS (push): plaga detectada por la Jetson, sensor fuera de rango
     físico, sensor sin inicializar, enlace LoRa caído, downlink de
     emergencia enviado. Cada tipo tiene su propio interruptor y su propio
     antirrebote — sin eso, un sensor I2C mudo genera un mensaje por cada
     uplink y en media hora el chat es inusable.

  2. REPORTE (pull o programado): resumen de los últimos paquetes que
     recibió el gateway — promedios, mínimos/máximos, calidad del enlace,
     cuántos venían marcados como sospechosos. Se puede pedir desde el
     dashboard, desde el propio chat (`/reporte`) o dejarlo automático
     cada N minutos.

El bot también escucha comandos por long-polling (`getUpdates`), así que
el operador consulta el estado del cultivo desde el celular sin estar en
la red del robot. No se usa webhook a propósito: exigiría IP pública y
certificado TLS en la Raspberry Pi, y la mitad de los despliegues de campo
están detrás de un router 4G sin puerto abierto.

Configuración: `telegram_config.json` (se crea solo). El token puede venir
también de la variable de entorno TELEGRAM_BOT_TOKEN, que tiene prioridad
— útil para no dejarlo escrito en disco en un equipo compartido.
"""
import asyncio
import json
import os
import time
from pathlib import Path

import aiohttp

CONFIG_PATH = Path(__file__).parent / "telegram_config.json"
API_BASE = "https://api.telegram.org"

# Antirrebote por TIPO de alarma, en segundos. Un cultivo con un sensor
# roto emite el mismo aviso en cada uplink; sin esto el chat se vuelve
# ruido y el operador termina silenciando el bot entero, que es justo lo
# contrario de lo que queremos.
DEFAULTS = {
    "enabled": False,
    "bot_token": "",
    "chat_id": "",
    "alerts": {
        "plaga": True,              # detección YOLO empujada por la Jetson
        "sensor_fuera_rango": True, # valor físicamente imposible
        "sensor_caido": True,       # sensor que no inicializó (llega 0.0)
        "enlace_perdido": True,     # sin uplinks por más de link_timeout_s
        "downlink": True,           # se envió un comando de emergencia
        "sistema": True,            # avisos genéricos del backend
    },
    "cooldown_s": 300,              # mismo tipo de alarma: 1 cada 5 min
    "link_timeout_s": 300,          # sin paquetes 5 min = enlace caído
    "report_every_min": 0,          # 0 = reporte automático desactivado
    "report_packets": 50,           # cuántos paquetes resume el reporte
}

MAX_LOG = 60

# Nombres legibles: en el chat no sirve leer "tempSCD" ni "co2".
ETIQUETAS = {
    "co2": "CO₂", "tempSCD": "Temperatura", "humidity": "Humedad",
    "rssi": "RSSI", "snr": "SNR",
}


def load_config() -> dict:
    cfg = json.loads(json.dumps(DEFAULTS))
    if CONFIG_PATH.exists():
        try:
            guardado = json.loads(CONFIG_PATH.read_text())
        except json.JSONDecodeError:
            guardado = {}
        for k, v in guardado.items():
            if k not in cfg:
                continue
            if isinstance(cfg[k], dict) and isinstance(v, dict):
                cfg[k].update({sk: sv for sk, sv in v.items() if sk in cfg[k]})
            else:
                cfg[k] = v
    # El entorno gana sobre el archivo: permite operar sin escribir el
    # token en disco.
    if os.environ.get("TELEGRAM_BOT_TOKEN"):
        cfg["bot_token"] = os.environ["TELEGRAM_BOT_TOKEN"]
    if os.environ.get("TELEGRAM_CHAT_ID"):
        cfg["chat_id"] = os.environ["TELEGRAM_CHAT_ID"]
    return cfg


def save_config(cfg: dict) -> None:
    a_guardar = {k: cfg.get(k, v) for k, v in DEFAULTS.items()}
    # El token es una credencial: si vino del entorno no se persiste, y el
    # archivo se crea sin permisos para el resto del sistema.
    if os.environ.get("TELEGRAM_BOT_TOKEN"):
        a_guardar["bot_token"] = ""
    CONFIG_PATH.write_text(json.dumps(a_guardar, indent=2))
    try:
        CONFIG_PATH.chmod(0o600)
    except OSError:
        pass


def _esc(texto) -> str:
    """Escapa lo mínimo para parse_mode=HTML de Telegram."""
    return (str(texto).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


class TelegramNotifier:
    def __init__(self, fetch_history, fetch_downlinks):
        # Se le inyectan las funciones de lectura del gateway en vez de
        # importar el módulo: así este archivo no depende de
        # gateway_server.py y se puede probar solo.
        self._fetch_history = fetch_history
        self._fetch_downlinks = fetch_downlinks
        self.config = load_config()
        self.log: list[dict] = []          # últimos envíos, para el dashboard
        self.last_sent_at: dict[str, float] = {}
        self.last_packet_at: float | None = None
        self._link_avisado = False
        self._session: aiohttp.ClientSession | None = None
        self._update_offset = 0
        self._tasks: list[asyncio.Task] = []
        self.last_error = ""

    # ------------------------------------------------------------------
    # Ciclo de vida
    # ------------------------------------------------------------------
    async def start(self):
        self._session = aiohttp.ClientSession(
            timeout=aiohttp.ClientTimeout(total=35))
        self._tasks = [
            asyncio.create_task(self._poll_loop()),
            asyncio.create_task(self._watchdog_loop()),
            asyncio.create_task(self._report_loop()),
        ]

    async def stop(self):
        for t in self._tasks:
            t.cancel()
        if self._session:
            await self._session.close()

    @property
    def configurado(self) -> bool:
        return bool(self.config["bot_token"] and self.config["chat_id"])

    def estado(self) -> dict:
        """Lo que el dashboard necesita para pintar la pestaña."""
        return {
            "enabled": self.config["enabled"],
            "configurado": self.configurado,
            # NUNCA se devuelve el token: el dashboard solo necesita saber
            # si hay uno cargado, no cuál es.
            "tiene_token": bool(self.config["bot_token"]),
            "token_desde_entorno": bool(os.environ.get("TELEGRAM_BOT_TOKEN")),
            "chat_id": self.config["chat_id"],
            "alerts": self.config["alerts"],
            "cooldown_s": self.config["cooldown_s"],
            "link_timeout_s": self.config["link_timeout_s"],
            "report_every_min": self.config["report_every_min"],
            "report_packets": self.config["report_packets"],
            "log": list(reversed(self.log)),
            "last_error": self.last_error,
        }

    def actualizar_config(self, cambios: dict) -> dict:
        for k, v in cambios.items():
            if k == "alerts" and isinstance(v, dict):
                self.config["alerts"].update(
                    {sk: bool(sv) for sk, sv in v.items() if sk in self.config["alerts"]})
            elif k in DEFAULTS and k != "alerts":
                self.config[k] = v
        save_config(self.config)
        return self.estado()

    # ------------------------------------------------------------------
    # Envío
    # ------------------------------------------------------------------
    def _anotar(self, tipo: str, texto: str, ok: bool, detalle: str = ""):
        self.log.append({"ts": time.time(), "tipo": tipo, "ok": ok,
                         "texto": texto[:220], "detalle": detalle})
        if len(self.log) > MAX_LOG:
            self.log.pop(0)

    async def _api(self, metodo: str, payload: dict) -> dict | None:
        if not self._session or not self.config["bot_token"]:
            return None
        url = f"{API_BASE}/bot{self.config['bot_token']}/{metodo}"
        try:
            async with self._session.post(url, json=payload) as resp:
                data = await resp.json()
                if data.get("ok"):
                    # Se LIMPIA en cada llamada correcta. Antes solo se
                    # escribía al fallar y no se borraba nunca, así que un
                    # error puntual del arranque —típicamente el "Conflict"
                    # de getUpdates cuando quedaba una instancia anterior
                    # viva— se quedaba pegado para siempre y el dashboard
                    # mostraba Telegram como averiado mientras los mensajes
                    # salían perfectamente.
                    self.last_error = ""
                else:
                    self.last_error = str(data.get("description", ""))[:200]
                return data
        except Exception as exc:
            # Una Raspberry Pi de campo pierde internet constantemente. Un
            # fallo de red NO puede tumbar el gateway: se registra y sigue.
            self.last_error = f"{type(exc).__name__}: {exc}"[:200]
            return None

    async def enviar(self, texto: str, tipo: str = "manual") -> bool:
        if not self.config["enabled"] or not self.configurado:
            self._anotar(tipo, texto, False, "bot deshabilitado o sin configurar")
            return False
        data = await self._api("sendMessage", {
            "chat_id": self.config["chat_id"],
            "text": texto,
            "parse_mode": "HTML",
            "disable_web_page_preview": True,
        })
        ok = bool(data and data.get("ok"))
        self._anotar(tipo, texto, ok, "" if ok else self.last_error)
        return ok

    async def alarma(self, tipo: str, titulo: str, cuerpo: str,
                     forzar: bool = False) -> bool:
        """Alarma con antirrebote e interruptor por tipo."""
        if not forzar:
            if not self.config["alerts"].get(tipo, True):
                return False
            ultimo = self.last_sent_at.get(tipo, 0)
            if time.time() - ultimo < self.config["cooldown_s"]:
                return False
        self.last_sent_at[tipo] = time.time()
        icono = {"plaga": "🐛", "sensor_fuera_rango": "📉", "sensor_caido": "🔌",
                 "enlace_perdido": "📡", "downlink": "🛑",
                 "sistema": "⚙️"}.get(tipo, "⚠️")
        hora = time.strftime("%H:%M:%S")
        return await self.enviar(
            f"{icono} <b>{_esc(titulo)}</b>\n{cuerpo}\n\n<i>AgroBot · {hora}</i>",
            tipo)

    # ------------------------------------------------------------------
    # Enganches con el gateway
    # ------------------------------------------------------------------
    def on_packet(self, row: dict):
        """Llamar con CADA uplink decodificado. No es async: se dispara y
        sigue, para no meter latencia en el hilo de MQTT."""
        self.last_packet_at = time.time()
        if self._link_avisado:
            self._link_avisado = False
            asyncio.create_task(self.alarma(
                "enlace_perdido", "Enlace LoRa restablecido",
                f"Volvieron a llegar uplinks de <code>{_esc(row.get('dev_eui','?'))}</code>.",
                forzar=True))

        fuera = row.get("_out_of_range") or []
        if fuera:
            detalle = "\n".join(
                f"• <code>{_esc(c)}</code> = {_esc(round(row.get(c, 0), 2))}" for c in fuera)
            asyncio.create_task(self.alarma(
                "sensor_fuera_rango", "Lectura fuera de rango físico",
                f"El gateway recibió valores imposibles:\n{detalle}\n"
                f"Suele ser ruido en el bus I2C o un sensor a punto de fallar."))

        faltantes = row.get("_sensors_missing") or []
        if faltantes:
            asyncio.create_task(self.alarma(
                "sensor_caido", "Sensor sin inicializar",
                "No hay dato real de: " +
                ", ".join(f"<code>{_esc(c)}</code>" for c in faltantes) +
                "\nEl payload trae 0.0 en esos campos — no son mediciones."))

    def on_downlink(self, comando: str, ok: bool):
        asyncio.create_task(self.alarma(
            "downlink", f"Downlink {_esc(comando)}",
            "Encolado en ChirpStack. Se entrega en el próximo uplink del "
            "dispositivo (LoRaWAN clase A, hasta ~30 s)." if ok else
            "❌ <b>No se pudo encolar</b> — revisá CHIRPSTACK_API_TOKEN."))

    async def on_alerta_externa(self, payload: dict) -> bool:
        """POST /api/alert — la Jetson empuja acá lo que detecta su YOLO."""
        tipo = payload.get("tipo", "plaga")
        titulo = payload.get("titulo") or "Alerta del robot"
        cuerpo = payload.get("mensaje") or ""
        foto = payload.get("image_url")

        if tipo == "plaga":
            clase = payload.get("clase", "desconocida")
            conf = payload.get("confianza")
            cuerpo = (f"Clase: <b>{_esc(clase)}</b>\n" +
                      (f"Confianza: <b>{conf * 100:.0f}%</b>\n" if isinstance(conf, (int, float)) else "") +
                      (cuerpo or "Detectada por el modelo YOLO de la Jetson."))
            titulo = "Plaga detectada"

        enviado = await self.alarma(tipo, titulo, cuerpo)
        # La foto va aparte y solo si el texto salió: sin evidencia visual
        # el operador tiene que ir hasta el robot para saber si es un falso
        # positivo, y ese viaje es justo lo que el sistema quiere evitar.
        if enviado and foto:
            await self._api("sendPhoto", {
                "chat_id": self.config["chat_id"],
                "photo": foto,
                "caption": f"AgroBot · {_esc(titulo)}",
            })
        return enviado

    # ------------------------------------------------------------------
    # Reporte
    # ------------------------------------------------------------------
    def construir_reporte(self, limite: int | None = None) -> str:
        n = limite or self.config["report_packets"]
        filas = self._fetch_history(n)
        if not filas:
            return ("📋 <b>Reporte AgroBot</b>\n\nEl gateway todavía no recibió "
                    "ningún paquete. Revisá que el nodo LoRa esté encendido y "
                    "dado de alta en ChirpStack.")

        def col(nombre):
            """Valores utilizables de un campo.

            Se DESCARTAN los que el propio esquema ya marcó como basura en
            ese paquete: un sensor caído manda 0.0 y uno con ruido I2C manda
            99999, y promediarlos daba un reporte peor que no tener reporte
            (una temperatura media de 0 °C porque el SCD40 no arrancó, un
            CO2 medio de 3000 ppm por una sola lectura imposible). Los
            paquetes afectados se siguen contando aparte, en la sección
            "calidad del dato" — se excluyen del promedio, no del informe.
            """
            return [f[nombre] for f in filas
                    if isinstance(f.get(nombre), (int, float))
                    and nombre not in (f.get("_out_of_range") or [])
                    and nombre not in (f.get("_sensors_missing") or [])]

        def resumen(nombre, unidad, dec=1):
            vals = col(nombre)
            etiqueta = ETIQUETAS.get(nombre, nombre)
            if not vals:
                return f"• {etiqueta}: <i>sin lecturas válidas</i>"
            prom = sum(vals) / len(vals)
            descartados = len(filas) - len(vals)
            nota = f" <i>({descartados} descartadas)</i>" if descartados else ""
            return (f"• {etiqueta}: <b>{prom:.{dec}f}</b> {unidad} "
                    f"(mín {min(vals):.{dec}f} · máx {max(vals):.{dec}f}){nota}")

        ultimo = filas[0]
        primero = filas[-1]
        ventana_min = max((ultimo["received_at"] - primero["received_at"]) / 60, 0)
        sospechosos = sum(1 for f in filas if f.get("_out_of_range"))
        caidos = sum(1 for f in filas if f.get("_sensors_missing"))
        edad = time.time() - ultimo["received_at"]
        edad_txt = f"hace {int(edad)} s" if edad < 120 else f"hace {int(edad / 60)} min"

        rssi = col("rssi")
        calidad = ("sin dato" if not rssi else
                   "excelente" if sum(rssi) / len(rssi) > -90 else
                   "aceptable" if sum(rssi) / len(rssi) > -110 else "al límite")

        lineas = [
            "📋 <b>Reporte AgroBot — gateway LoRaWAN</b>",
            f"<i>{time.strftime('%d/%m/%Y %H:%M')}</i>",
            "",
            f"<b>{len(filas)}</b> paquetes analizados sobre "
            f"<b>{ventana_min:.0f} min</b> · último {edad_txt}",
            f"Dispositivo: <code>{_esc(ultimo.get('dev_eui', '?'))}</code>",
            "",
            "<b>Ambiente</b>",
            resumen("co2", "ppm", 0),
            resumen("tempSCD", "°C"),
            resumen("humidity", "%"),
            "",
            "<b>Enlace LoRa</b>",
            resumen("rssi", "dBm", 0),
            resumen("snr", "dB"),
            f"• Calidad estimada: <b>{calidad}</b>",
            "",
            "<b>Calidad del dato</b>",
            f"• Paquetes con valores fuera de rango: <b>{sospechosos}</b>",
            f"• Paquetes con algún sensor caído: <b>{caidos}</b>",
        ]

        downlinks = self._fetch_downlinks(5)
        if downlinks:
            lineas += ["", "<b>Últimos downlinks</b>"]
            lineas += [f"• {_esc(d['command'])} — "
                       f"{time.strftime('%H:%M', time.localtime(d['sent_at']))}"
                       for d in downlinks]
        return "\n".join(lineas)

    async def enviar_reporte(self, limite: int | None = None) -> bool:
        return await self.enviar(self.construir_reporte(limite), "reporte")

    # ------------------------------------------------------------------
    # Bucles de fondo
    # ------------------------------------------------------------------
    async def _watchdog_loop(self):
        """Avisa una sola vez cuando el enlace LoRa se cae."""
        while True:
            await asyncio.sleep(30)
            if not (self.config["enabled"] and self.last_packet_at):
                continue
            silencio = time.time() - self.last_packet_at
            if silencio > self.config["link_timeout_s"] and not self._link_avisado:
                self._link_avisado = True
                await self.alarma(
                    "enlace_perdido", "Enlace LoRa caído",
                    f"No llegan uplinks desde hace <b>{int(silencio / 60)} min</b>. "
                    f"Puede ser el nodo sin batería, fuera de cobertura, o el "
                    f"gateway HT-M02 desconectado.", forzar=True)

    async def _report_loop(self):
        while True:
            cada = self.config["report_every_min"]
            await asyncio.sleep(max(cada, 1) * 60 if cada > 0 else 60)
            if self.config["enabled"] and self.config["report_every_min"] > 0:
                await self.enviar_reporte()

    async def _poll_loop(self):
        """Escucha comandos del chat (long-polling).

        Basta con tener token: NO se exige `enabled`. Antes sí se exigía, y
        eso creaba un bloqueo del que no se podía salir — `enabled` arranca
        en False, el emparejamiento se hace mandando /start al bot, y ese
        /start llega justo por este bucle. Con el bucle apagado, el mensaje
        no se leía nunca y el bot parecía muerto: el operador escribía y no
        pasaba nada, sin ningún error en ninguna parte.

        Escuchar siempre es además lo correcto para /silencio: si el bucle
        se apagara al silenciar las alarmas, no habría forma de volver a
        activarlas desde el chat.
        """
        while True:
            if not self.config["bot_token"]:
                await asyncio.sleep(5)
                continue
            data = await self._api("getUpdates", {
                "offset": self._update_offset, "timeout": 25})
            if not data or not data.get("ok"):
                await asyncio.sleep(5)
                continue
            for update in data.get("result", []):
                self._update_offset = update["update_id"] + 1
                await self._manejar_update(update)

    async def _manejar_update(self, update: dict):
        mensaje = update.get("message") or {}
        texto = (mensaje.get("text") or "").strip().lower()
        chat = str((mensaje.get("chat") or {}).get("id", ""))
        if not texto or not chat:
            return

        # Si todavía no hay chat_id configurado, el primer /start lo fija.
        # Es la forma menos frustrante de emparejar: el operador no tiene
        # que ir a buscar su ID numérico a otro bot.
        if not self.config["chat_id"] and texto.startswith("/start"):
            self.config["chat_id"] = chat
            # Se activa junto con el emparejamiento. Vincular el chat y dejar
            # las alarmas apagadas es un estado que no quiere nadie: quien
            # manda /start lo hace para recibir avisos, y el paso extra solo
            # se descubriría el día que no llegue la primera alarma.
            self.config["enabled"] = True
            save_config(self.config)
            await self.enviar(
                "✅ <b>AgroBot vinculado</b>\nEste chat va a recibir las alarmas "
                "del cultivo.\nEscribí /ayuda para ver los comandos.", "sistema")
            return

        # Cualquier otro chat se ignora: el bot es de un solo operador y su
        # token puede filtrarse (aparece en la URL de cada request).
        if chat != str(self.config["chat_id"]):
            return

        comando = texto.split()[0].lstrip("/").split("@")[0]
        if comando in ("reporte", "report"):
            await self.enviar_reporte()
        elif comando in ("estado", "status"):
            await self.enviar(self._texto_estado(), "estado")
        elif comando in ("ultimos", "último", "ultimo"):
            await self.enviar(self._texto_ultimo(), "estado")
        elif comando in ("activar", "unmute"):
            # Contrapartida de /silencio. Sin esto, silenciar era un camino
            # de ida: había que entrar al dashboard para volver a encender.
            self.config["enabled"] = True
            save_config(self.config)
            await self.enviar("🔔 <b>Alarmas reactivadas</b>", "sistema")
        elif comando in ("silencio", "mute"):
            self.config["enabled"] = False
            save_config(self.config)
            self._anotar("sistema", "alarmas silenciadas desde el chat", True)
        elif comando in ("ayuda", "help", "start"):
            await self.enviar(
                "<b>AgroBot · comandos</b>\n"
                "/estado — resumen corto en vivo\n"
                "/ultimos — último paquete completo\n"
                "/reporte — reporte de los últimos paquetes\n"
                "/silencio — cortar las alarmas\n"
                "/activar — volver a encenderlas\n"
                "/ayuda — esta lista", "sistema")

    def _texto_estado(self) -> str:
        filas = self._fetch_history(1)
        if not filas:
            return "Sin paquetes recibidos todavía."
        d = filas[0]
        edad = int(time.time() - d["received_at"])
        return (f"🌱 <b>AgroBot en vivo</b>\n"
                f"CO₂ <b>{d['co2']:.0f}</b> ppm · "
                f"{d['tempSCD']:.1f} °C · {d['humidity']:.0f} % HR\n"
                f"RSSI {d['rssi']} dBm · último paquete hace {edad} s")

    def _texto_ultimo(self) -> str:
        filas = self._fetch_history(1)
        if not filas:
            return "Sin paquetes recibidos todavía."
        d = filas[0]
        campos = ["co2", "tempSCD", "humidity",
                  "accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ",
                  "rssi", "snr", "sf"]
        cuerpo = "\n".join(f"{c}: <code>{_esc(d.get(c))}</code>" for c in campos)
        return (f"📦 <b>Paquete #{d.get('id')}</b>\n"
                f"<i>{time.strftime('%H:%M:%S', time.localtime(d['received_at']))}</i>\n\n"
                f"{cuerpo}")
