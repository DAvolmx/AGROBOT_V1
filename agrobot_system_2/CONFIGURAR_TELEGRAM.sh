#!/usr/bin/env bash
# =============================================================================
# CONFIGURAR_TELEGRAM.sh — deja el bot listo en la Raspberry.
#
#   ./CONFIGURAR_TELEGRAM.sh <token-de-BotFather>
#
# Antes de ejecutarlo hay que crear el bot, que se hace desde el propio
# Telegram y no desde aquí:
#
#   1. Abre Telegram y busca el contacto  @BotFather
#   2. /newbot
#   3. Nombre visible:  AgroBot Alertas
#   4. Usuario, tiene que acabar en "bot":  agrobot_parcela_bot
#   5. BotFather responde con un token de la forma  1234567890:AA...
#
# El chat_id NO hay que buscarlo: el gateway lo aprende solo la primera vez
# que le escribes /start al bot. Eso evita el paso que más se atasca, que es
# pelearse con getUpdates para averiguar el número del chat.
# =============================================================================
set -u
cd "$(dirname "$0")"

TOKEN="${1:-}"
if [ -z "$TOKEN" ]; then
  echo "Uso: ./CONFIGURAR_TELEGRAM.sh <token-de-BotFather>"
  echo
  echo "El token se saca hablando con @BotFather en Telegram (/newbot)."
  exit 1
fi

# Se valida contra la API ANTES de escribir nada. Un token mal copiado
# guardado en el .env produce un bot que arranca sin quejarse y no manda
# ninguna alarma: el fallo aparecería el día de la primera emergencia.
echo "[tg] comprobando el token…"
RESP=$(curl -s -m 15 "https://api.telegram.org/bot${TOKEN}/getMe")
OK=$(printf '%s' "$RESP" | python3 -c "import sys,json;print(json.load(sys.stdin).get('ok'))" 2>/dev/null)

if [ "$OK" != "True" ]; then
  echo "[tg] ✗ el token no es válido. Respuesta de Telegram:"
  printf '%s\n' "$RESP" | head -c 300; echo
  echo "     Cópialo entero, incluidos los dos puntos del medio."
  exit 1
fi

NOMBRE=$(printf '%s' "$RESP" | python3 -c "import sys,json;print(json.load(sys.stdin)['result']['username'])")
echo "[tg] ✓ token válido — el bot es @${NOMBRE}"

# Se guarda en el .env y no en telegram_config.json a propósito: el módulo
# de Telegram da prioridad al entorno y, cuando el token viene de ahí, NO lo
# copia al archivo de configuración. Así la credencial vive en un único
# sitio, con permisos 600, en vez de duplicada en dos ficheros.
touch .env && chmod 600 .env
if grep -q '^TELEGRAM_BOT_TOKEN=' .env 2>/dev/null; then
  sed -i "s|^TELEGRAM_BOT_TOKEN=.*|TELEGRAM_BOT_TOKEN=${TOKEN}|" .env
  echo "[tg] token actualizado en .env"
else
  echo "TELEGRAM_BOT_TOKEN=${TOKEN}" >> .env
  echo "[tg] token añadido a .env"
fi

cat <<FIN

  ============================================
   Falta un paso, y tiene que hacerlo un humano
  ============================================

  1. Reinicia el gateway:

       pkill -f gateway_server.py
       ./INICIAR_SERVER.sh

  2. Abre Telegram, busca  @${NOMBRE}  y mándale:

       /start

     Con ese mensaje el gateway aprende tu chat_id y se activa solo.
     No hace falta buscarlo a mano.

  3. Comprueba desde el dashboard, pestaña Telegram, o con:

       curl -s http://localhost:8092/api/telegram | python3 -m json.tool

  Por qué el /start es imprescindible: la API de Telegram no deja que un bot
  escriba a alguien que no le ha hablado antes. Es una protección contra el
  spam, no un fallo de configuración, y no se puede saltar desde el servidor.

FIN
