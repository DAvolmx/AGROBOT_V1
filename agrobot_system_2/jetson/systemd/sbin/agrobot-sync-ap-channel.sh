#!/bin/bash
# Mantiene el canal del AP igual al del cliente Wi-Fi.
#
# La radio (Intel 8265) es una sola. Con el AP y el cliente en canales
# distintos el driver reparte el tiempo entre ambos: el enlace se vuelve
# intermitente, sube la latencia y el dashboard "se cae". El router reelige
# canal por su cuenta, así que esto corre también por temporizador cada
# minuto, no solo al arrancar hostapd (el 2026-09-17 quedaron en 11 vs 1).
set -u

CONF=/etc/hostapd/agrobot.conf
STA_IF=wlP1p1s0
REINICIAR=${1:-no}          # "reiniciar" = aplica el cambio ya

ch=$(iw dev "$STA_IF" info 2>/dev/null | sed -n 's/.*channel \([0-9]\{1,3\}\).*/\1/p' | head -1)
if [ -z "$ch" ]; then
    echo "[sync-ap] $STA_IF sin canal (desconectado) — se respeta el canal del archivo"
    exit 0
fi

actual=$(sed -n 's/^channel=\([0-9]\{1,3\}\).*/\1/p' "$CONF" | head -1)
en_uso=$(iw dev ap0 info 2>/dev/null | sed -n 's/.*channel \([0-9]\{1,3\}\).*/\1/p' | head -1)

if [ "$ch" = "$actual" ] && [ "$ch" = "${en_uso:-$ch}" ]; then
    exit 0
fi

sed -i "s/^channel=.*/channel=$ch/" "$CONF"
echo "[sync-ap] canal del AP: archivo ${actual:-?} / en uso ${en_uso:-?} -> $ch (sigue a $STA_IF)"

if [ "$REINICIAR" = "reiniciar" ]; then
    # Corta el AP unos segundos, pero es preferible a dejarlo en otro canal:
    # así el enlace va mal todo el rato.
    systemctl restart agrobot-hostapd.service
fi
