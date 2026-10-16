#!/bin/bash
# Comprueba que el AP en el adaptador USB está bien (úsalo tras un reinicio). Solo lectura.
ok=0; mal=0
chk() { if eval "$2" >/dev/null 2>&1; then echo "  OK   $1"; ok=$((ok+1)); else echo "  FALLA $1"; mal=$((mal+1)); fi; }
echo "Verificación AP en adaptador USB — $(date '+%F %H:%M') (arranque hace $(uptime -p))"
chk "módulo 8188eu cargado"              "lsmod | grep -q '^8188eu'"
chk "ap0 existe y es el adaptador USB"   "[ \"\$(basename \$(readlink -f /sys/class/net/ap0/device/driver))\" = 8188eu ]"
chk "ap0 en modo AP con SSID JETSON_AP"  "iw dev ap0 info | grep -q 'ssid JETSON_AP'"
chk "ap0 tiene 192.168.12.1/24"          "ip -4 -br addr show ap0 | grep -q '192.168.12.1/24'"
chk "potencia 20 dBm"                    "iw dev ap0 info | grep -q 'txpower 20'"
chk "hostapd activo"                     "systemctl is-active --quiet agrobot-hostapd"
chk "dnsmasq activo"                     "systemctl is-active --quiet agrobot-dnsmasq"
chk "NetworkManager no gestiona ap0"     "nmcli -t -f DEVICE,STATE dev | grep -q '^ap0:unmanaged'"
chk "radio interna libre (sin AP)"       "! iw dev wlP1p1s0 info | grep -q 'type AP'"
chk "sin procesos colgados (estado D)"   "[ \$(ps -eo stat | grep -c '^D') -le 1 ]"
chk "dashboard responde"                 "curl -s -m5 -o /dev/null -w '%{http_code}' http://localhost:8080/ | grep -q 200"
n=$(iw dev ap0 station dump 2>/dev/null | grep -c '^Station'); echo "  INFO estaciones conectadas: $n"
echo "Resultado: $ok OK, $mal fallos"; [ $mal -eq 0 ]
