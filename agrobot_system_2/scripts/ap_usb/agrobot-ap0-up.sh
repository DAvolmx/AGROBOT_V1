#!/bin/bash
# Crea ap0 (192.168.12.1/24): sobre el adaptador USB RTL8188ETV (driver 8188eu) si esta presente;
# si no, sobre la radio interna (Intel) como siempre. NUNCA usa `iw dev <usb> del`: ese driver
# se cuelga y bloquea toda la red del kernel (2026-10-16). Para dejar el adaptador se renombra.
IP=192.168.12.1/24
usb_if() {
  for n in /sys/class/net/*; do
    [ -e "$n/phy80211" ] || continue
    [ "$(basename "$(readlink -f "$n/device/driver" 2>/dev/null)")" = 8188eu ] && { basename "$n"; return; }
  done
}
# Si el adaptador USB esta conectado, espera (hasta 25 s) a que el driver cree su interfaz.
hay_usb=0
for d in /sys/bus/usb/devices/*; do
  [ "$(cat $d/idVendor 2>/dev/null)$(cat $d/idProduct 2>/dev/null)" = "0bda0179" ] && hay_usb=1
done
[ $hay_usb = 1 ] && { modprobe 8188eu 2>/dev/null; for i in $(seq 25); do [ -n "$(usb_if)" ] && break; sleep 1; done; }
U=$(usb_if)
if [ -n "$U" ]; then
  if [ "$U" != ap0 ]; then
    [ -e /sys/class/net/ap0 ] && iw dev ap0 del 2>/dev/null   # ap0 virtual de la radio interna (Intel)
    ip link set "$U" down; ip link set "$U" name ap0
  fi
  ip addr flush dev ap0; ip addr add $IP dev ap0; ip link set ap0 up
  iw dev ap0 set txpower fixed 2000 2>/dev/null
  echo "[ap0] AP sobre el adaptador USB"
else
  [ -e /sys/class/net/ap0 ] && iw dev ap0 del 2>/dev/null
  iw phy phy0 interface add ap0 type __ap
  ip link set ap0 address 76:70:fd:e3:ec:70
  ip addr add $IP dev ap0; ip link set ap0 up
  echo "[ap0] AP sobre la radio interna (sin adaptador USB)"
fi
