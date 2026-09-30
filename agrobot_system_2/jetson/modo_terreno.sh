#!/bin/bash
# Modo terreno / modo casa para la radio WiFi de la Jetson (2026-09-30).
#
# La Jetson tiene UNA radio (Intel 8265) que hace de punto de acceso
# (JETSON_AP: dashboard, C3 por WiFi, LoRa V3) y de cliente del router de
# casa (LoRa_Wan). En el terreno no hay router: NetworkManager sigue
# buscándolo y, en cada barrido de canales, la radio abandona el canal del
# AP unas décimas de segundo. Eso son microcortes para la C3 por WiFi, la
# LoRa y el móvil con el dashboard.
#
#   ./modo_terreno.sh terreno   -> el cliente deja de buscar el router
#   ./modo_terreno.sh casa      -> vuelve a conectarse a LoRa_Wan
#   ./modo_terreno.sh           -> muestra el estado
#
# No toca el AP (ap0 lo maneja hostapd, no NetworkManager).
set -u
CLIENTE=wlP1p1s0
PERFIL=LoRa_Wan

estado() {
  echo "cliente $CLIENTE: $(nmcli -t -f DEVICE,STATE dev | grep "^$CLIENTE:" | cut -d: -f2)"
  echo "autoconexión de $PERFIL: $(nmcli -t -f NAME,AUTOCONNECT con show | grep "^$PERFIL:" | cut -d: -f2)"
  echo "canal del AP: $(iw dev ap0 info 2>/dev/null | sed -n 's/.*channel \([0-9]*\).*/\1/p')"
}

case "${1:-}" in
  terreno)
    sudo nmcli con modify "$PERFIL" connection.autoconnect no
    sudo nmcli dev disconnect "$CLIENTE" 2>/dev/null
    echo "modo TERRENO: el cliente WiFi ya no busca el router; el AP sigue igual"
    estado ;;
  casa)
    sudo nmcli con modify "$PERFIL" connection.autoconnect yes
    sudo nmcli con up "$PERFIL"
    echo "modo CASA: conectado otra vez a $PERFIL"
    estado ;;
  *)
    estado ;;
esac
