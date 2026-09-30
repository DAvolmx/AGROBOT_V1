#!/bin/bash
# Arranca sllidar_node con el puerto del RPLIDAR C1 elegido EN CADA ARRANQUE.
#
# El launch lo usa como `prefix`, así que también decide en cada respawn:
#   · Adaptador USB CP210x enchufado -> ese, por /dev/serial/by-id (no
#     /dev/ttyUSBn: tras un corte USB vuelve con otro número, 2026-09-19).
#     Primero el adaptador conocido; si se cambió por otro CP210x, ese.
#   · Si no, la UART del conector de 40 pines (pines 8/10 -> /dev/ttyTHS1).
# Con AGROBOT_LIDAR_PUERTO=<ruta> se fuerza uno ("auto" o vacío = elegir).
#
# El 2026-09-29 se probó la UART y el C1 no respondió en ningún sentido de
# TX/RX; se volvió al USB. Así no hay que tocar el launch al cambiar.
#
# Sin USB se espera hasta ESPERA_USB_S a que aparezca antes de probar la
# UART. Sin esa espera el driver moría y renacía cada 6 s contra una UART
# sin LIDAR: 5400 arranques fallidos en un día llenaron logs/ros.log.
USB_CONOCIDO=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_5e723775cd73ef11b311cf8c8fcc3fa0-if00-port0
UART=/dev/ttyTHS1
ESPERA_USB_S=${AGROBOT_LIDAR_ESPERA_USB_S:-30}

buscar_usb() {
  if [ -e "$USB_CONOCIDO" ]; then echo "$USB_CONOCIDO"; return; fi
  for p in /dev/serial/by-id/usb-Silicon_Labs_CP210*; do
    [ -e "$p" ] && { echo "$p"; return; }
  done
}

PUERTO=${AGROBOT_LIDAR_PUERTO:-auto}
if [ "$PUERTO" = "auto" ]; then
  PUERTO=$(buscar_usb)
  if [ -z "$PUERTO" ]; then
    echo "[lidar] sin adaptador USB; espero hasta ${ESPERA_USB_S} s antes de probar $UART" >&2
    for _ in $(seq "$ESPERA_USB_S"); do
      sleep 1
      PUERTO=$(buscar_usb)
      [ -n "$PUERTO" ] && break
    done
    [ -z "$PUERTO" ] && PUERTO=$UART
  fi
fi
echo "[lidar] puerto: $PUERTO" >&2
exec "$@" -p "serial_port:=$PUERTO"
