#!/bin/bash
# Pone el canal del AP igual al del cliente Wi-Fi antes de arrancar hostapd.
#
# La radio (Intel 8265) es una sola. Con el AP y el cliente en canales
# distintos el driver reparte el tiempo entre ambos: el enlace se vuelve
# intermitente y el dashboard parece colgarse. El 2026-09-16 el AP estaba
# en el canal 1 y LoRa_Wan en el 11 — sincronizarlo a mano no basta porque
# el router reelige canal solo.
set -u

CONF=/etc/hostapd/agrobot.conf
STA_IF=wlP1p1s0

ch=$(iw dev "$STA_IF" info 2>/dev/null | sed -n 's/.*channel \([0-9]\{1,3\}\).*/\1/p' | head -1)

if [ -z "$ch" ]; then
    echo "[sync-ap] $STA_IF sin canal (desconectado) — se respeta el canal del archivo"
    exit 0
fi

actual=$(sed -n 's/^channel=\([0-9]\{1,3\}\).*/\1/p' "$CONF" | head -1)

if [ "$ch" = "$actual" ]; then
    echo "[sync-ap] canal ya sincronizado en $ch"
    exit 0
fi

sed -i "s/^channel=.*/channel=$ch/" "$CONF"
echo "[sync-ap] canal del AP: ${actual:-?} -> $ch (sigue a $STA_IF)"
