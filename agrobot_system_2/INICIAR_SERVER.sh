#!/usr/bin/env bash
# =============================================================================
# INICIAR_SERVER.sh — arranca el gateway de la Raspberry Pi con un comando.
#
#   ./INICIAR_SERVER.sh
#
# Este script NO levanta ChirpStack ni Mosquitto: los dos son servicios de
# systemd que ya vienen instalados y arrancan solos con la Pi. Lo que hace es
# comprobar que estén realmente arriba ANTES de lanzar el gateway, porque el
# fallo típico no es un error sino un silencio: gateway_server.py arranca
# igual sin broker, el dashboard abre, y simplemente no llega nunca un
# paquete. Visto desde afuera es idéntico a "el robot no está transmitiendo".
#
# Config: si existe un archivo `.env` junto a este script, se carga. Ahí van
# CHIRPSTACK_API_TOKEN y el resto, para no tener que exportarlas a mano cada
# vez ni dejar el token escrito dentro del código.
# =============================================================================
set -u
cd "$(dirname "$0")"

RAIZ="$(pwd)"
SRV="$RAIZ/rpi/server"
LOGS="$RAIZ/logs"
mkdir -p "$LOGS"

[ -f "$SRV/gateway_server.py" ] || {
  echo "[server] ✗ no encuentro $SRV/gateway_server.py"; exit 1; }

# ---------------------------------------------------------------------------
# Configuración
# ---------------------------------------------------------------------------
if [ -f "$RAIZ/.env" ]; then
  set -a; # shellcheck disable=SC1091
  source "$RAIZ/.env"; set +a
  echo "[server] .env cargado"
fi

# UUID, no un entero: ChirpStack v4 cambió el formato del application_id.
export CHIRPSTACK_APP_ID="${CHIRPSTACK_APP_ID:-aa4c5278-0d39-4833-be36-286895ffa4c4}"
# 8090 = contenedor chirpstack-rest-api. El 8080 es la interfaz web.
export CHIRPSTACK_API_URL="${CHIRPSTACK_API_URL:-http://localhost:8090}"
export CHIRPSTACK_API_TOKEN="${CHIRPSTACK_API_TOKEN:-}"
export AGROBOT_DEV_EUI="${AGROBOT_DEV_EUI:-f59d08345985b27d}"
export MQTT_HOST="${MQTT_HOST:-localhost}"
export MQTT_PORT="${MQTT_PORT:-1883}"
PUERTO=8092

# ---------------------------------------------------------------------------
# Entorno de Python. El venv es el que ya existe en rpi/server/venv; si no
# está, se usa el python del sistema y se avisa, en vez de fallar.
# ---------------------------------------------------------------------------
if [ -x "$SRV/venv/bin/python" ]; then
  PY="$SRV/venv/bin/python"
  echo "[server] venv: $SRV/venv"
else
  PY=python3
  echo "[server] ⚠️  sin venv en rpi/server/venv — usando python3 del sistema."
  echo "         python3 -m venv rpi/server/venv && rpi/server/venv/bin/pip install -r rpi/server/requirements.txt"
fi

FALTAN=$("$PY" - <<'EOF'
import importlib.util as u
faltan = [pip for mod, pip in (("aiohttp", "aiohttp"), ("paho.mqtt", "paho-mqtt"))
          if u.find_spec(mod) is None]
print(" ".join(faltan))
EOF
)
[ -n "$FALTAN" ] && {
  echo "[server] ✗ faltan dependencias: $FALTAN"
  echo "         $PY -m pip install -r rpi/server/requirements.txt"; exit 1; }

# ---------------------------------------------------------------------------
# Comprobaciones previas — cada una corresponde a un modo de fallo silencioso
# ---------------------------------------------------------------------------
aviso() { echo -e "[server] \033[33m⚠\033[0m  $1"; }
ok()    { echo -e "[server] \033[32m✓\033[0m  $1"; }

# 1. Mosquitto. Sin broker no hay uplinks: el dashboard queda vacío para siempre.
if (exec 3<>"/dev/tcp/$MQTT_HOST/$MQTT_PORT") 2>/dev/null; then
  ok "broker MQTT en $MQTT_HOST:$MQTT_PORT"
else
  aviso "no responde el broker MQTT en $MQTT_HOST:$MQTT_PORT."
  echo "         sudo systemctl status mosquitto    — sin esto NO llega ni un paquete."
fi

# 2. API REST de ChirpStack. Solo hace falta para downlinks; los uplinks
#    entran por MQTT. Se comprueba un endpoint /api real, no solo que el
#    puerto acepte conexiones: en el despliegue con Docker la REST API vive
#    en un contenedor aparte (:8090) y el :8080 sirve SOLO la web, que
#    responde 200 en la raíz y 404 en todo /api. Mirar el puerto a secas da
#    verde apuntando al sitio equivocado.
API_HOST=${CHIRPSTACK_API_URL#*://}
CODIGO=$(curl -s -o /dev/null -w "%{http_code}" -m 8 \
         -H "Authorization: Bearer $CHIRPSTACK_API_TOKEN" \
         "$CHIRPSTACK_API_URL/api/devices/$AGROBOT_DEV_EUI" 2>/dev/null)
case "$CODIGO" in
  200) ok "API de ChirpStack en $API_HOST — dispositivo $AGROBOT_DEV_EUI encontrado" ;;
  401|403) aviso "la API responde pero rechaza el token (HTTP $CODIGO): revisá CHIRPSTACK_API_TOKEN." ;;
  404) aviso "la API responde en $API_HOST pero no conoce el DEV_EUI $AGROBOT_DEV_EUI."
       echo "         Si el 404 es en TODOS los /api, estás apuntando a la web (:8080)"
       echo "         en vez de a la REST API (:8090). Corregí CHIRPSTACK_API_URL." ;;
  *)   aviso "la API de ChirpStack no responde en $API_HOST (HTTP ${CODIGO:-sin respuesta})."
       echo "         docker ps | grep chirpstack-rest-api" ;;
esac

# 3. Token de API. Su ausencia no rompe nada al arrancar: rompe MÁS TARDE, y
#    solo el botón de parada de emergencia, que es justo cuando peor viene.
if [ -n "$CHIRPSTACK_API_TOKEN" ]; then
  ok "CHIRPSTACK_API_TOKEN presente (downlinks habilitados)"
else
  aviso "CHIRPSTACK_API_TOKEN vacío: los uplinks y el dashboard funcionan,"
  echo "         pero PARADA DE EMERGENCIA, PARK, PING y ACK no se van a enviar."
  echo "         Generá una API key en ChirpStack y ponela en $RAIZ/.env"
fi

# 4. Telegram. Opcional, pero es la única salida a internet del sistema.
[ -n "${TELEGRAM_BOT_TOKEN:-}" ] && ok "Telegram configurado" \
  || aviso "sin TELEGRAM_BOT_TOKEN — no habrá alarmas al celular."

# 5. Puerto libre. Arrancar con el puerto tomado da un traceback largo cuya
#    causa real está en la última línea.
if ss -ltn 2>/dev/null | grep -q ":$PUERTO "; then
  echo "[server] ✗ el puerto $PUERTO ya está ocupado — ¿hay otro gateway corriendo?"
  echo "         pgrep -af gateway_server.py"; exit 1
fi

# ---------------------------------------------------------------------------
# Arranque
# ---------------------------------------------------------------------------
IP=$(hostname -I | awk '{print $1}')
echo
echo "  ============================================"
echo "   AgroBot gateway en marcha"
echo "   dashboard: http://${IP:-127.0.0.1}:$PUERTO"
echo "   log:       $LOGS/gateway.log"
echo "   Ctrl+C para bajarlo"
echo "  ============================================"
echo

# `tee` para ver la salida en la terminal y guardarla a la vez. `-u` desactiva
# el buffer de Python: sin eso los `print` del gateway se quedan atascados en
# el buffer y el log aparece vacío justo cuando se lo va a consultar porque
# algo falló.
cd "$SRV"
"$PY" -u gateway_server.py 2>&1 | tee "$LOGS/gateway.log"
