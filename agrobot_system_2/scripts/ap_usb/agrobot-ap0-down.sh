#!/bin/bash
# Parar ap0 sin `iw del` en el adaptador USB (ver agrobot-ap0-up.sh).
[ -e /sys/class/net/ap0 ] || exit 0
if [ "$(basename "$(readlink -f /sys/class/net/ap0/device/driver 2>/dev/null)")" = 8188eu ]; then
  ip addr flush dev ap0; ip link set ap0 down
  ip link set ap0 name wlx54e4bd7cfda1 2>/dev/null
else
  iw dev ap0 del
fi
