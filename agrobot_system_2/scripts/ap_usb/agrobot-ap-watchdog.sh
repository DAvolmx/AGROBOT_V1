#!/bin/bash
# Vigilante del AP JETSON_AP en el adaptador USB RTL8188ETV (antena externa). Corre como servicio (root).
# Cada 10 s comprueba: ap0 en el USB, en modo AP, con la IP, hostapd habilitado y dnsmasq vivo. Si algo falla lo
# recrea con `systemctl restart` (NUNCA `iw dev <usb> del`: cuelga la red del kernel). Máx. 3 recreaciones en
# 10 min; luego espera 5 min (evita un bucle que tumbe a la C3 sin parar). Estado en /run/agrobot_ap_status.json.
# 2026-11-03: acepta 0bda:0179 (RTL8188ETV viejo) y 0bda:8179 (RTL8188EUS nuevo, MAC 00:e0:4c:87:2c:33).
# 2026-11-11: acepta también 0bda:f179 (RTL8188FTV, MAC 34:7d:e4:46:88:37, driver rtl8xxxu compilado del kernel 6.8).
IP=192.168.12.1
LOG=/var/log/agrobot_ap_watchdog.log
ESTADO=/run/agrobot_ap_status.json
declare -a RECREOS=()
fallo_txt=""
n_recreos_total=0
# 2026-11-03: un adaptador con mal contacto entraba y salía del bus (error -71) y cada vez el AP se movía a él y caía con
# la C3 dentro. Ahora solo se mueve al USB si lleva USB_ESTABLE_S presente sin cortes.
USB_ESTABLE_S=60
usb_desde=0
usb_num_antes=""
usb_num() { for d in /sys/bus/usb/devices/*; do [[ "$(cat $d/idVendor 2>/dev/null)$(cat $d/idProduct 2>/dev/null)" =~ ^0bda[08f]179$ ]] && { cat $d/devnum; return; }; done; }

log() { echo "[$(date '+%F %T')] $*" >> "$LOG"; }
usb_presente() { for d in /sys/bus/usb/devices/*; do [[ "$(cat $d/idVendor 2>/dev/null)$(cat $d/idProduct 2>/dev/null)" =~ ^0bda[08f]179$ ]] && return 0; done; return 1; }
ap_en_usb() { [[ "$(basename "$(readlink -f /sys/class/net/ap0/device/driver 2>/dev/null)")" =~ ^(8188eu|rtl8xxxu)$ ]]; }
ap_modo() { iw dev ap0 info 2>/dev/null | grep -q "type AP"; }
ip_ok() { ip -4 addr show dev ap0 2>/dev/null | grep -q "$IP/"; }
hostapd_ok() {
  systemctl is-active --quiet agrobot-hostapd.service || return 1
  local st; st=$(hostapd_cli -i ap0 status 2>/dev/null)
  [ -z "$st" ] && return 0          # sin socket de control: nos fiamos de systemd + modo AP
  echo "$st" | grep -q "state=ENABLED"
}
dnsmasq_ok() { systemctl is-active --quiet agrobot-dnsmasq.service; }
estaciones() { iw dev ap0 station dump 2>/dev/null | grep -c "^Station"; }

escribir_estado() {   # escribir_estado <ok 0|1> <modo> <detalle>
  local canal; canal=$(iw dev ap0 info 2>/dev/null | sed -n 's/.*channel \([0-9]*\).*/\1/p' | head -1)
  printf '{"t": %s, "ok": %s, "modo": "%s", "detalle": "%s", "en_usb": %s, "usb_presente": %s, "estaciones": %s, "canal": "%s", "recreaciones": %s}\n' \
    "$(date +%s)" "$([ "$1" = 1 ] && echo true || echo false)" "$2" "$3" \
    "$(ap_en_usb && echo true || echo false)" "$(usb_presente && echo true || echo false)" "$(estaciones)" "${canal:-}" "$n_recreos_total" > "$ESTADO.tmp" && mv "$ESTADO.tmp" "$ESTADO"
}

recrear() {
  local ahora=$(date +%s) recientes=() t
  for t in "${RECREOS[@]}"; do [ $((ahora - t)) -lt 600 ] && recientes+=("$t"); done
  RECREOS=("${recientes[@]}")
  if [ ${#RECREOS[@]} -ge 3 ]; then
    log "demasiadas recreaciones (${#RECREOS[@]} en 10 min): espero 300 s"; escribir_estado 0 "espera" "demasiadas recreaciones"; sleep 300; RECREOS=(); return
  fi
  RECREOS+=("$ahora"); n_recreos_total=$((n_recreos_total + 1))
  log "RECREANDO AP ($1)"
  systemctl stop agrobot-hostapd.service
  systemctl restart agrobot-ap0.service
  sleep 3
  systemctl start agrobot-hostapd.service
  systemctl restart agrobot-dnsmasq.service
  sleep 8
  if ap_modo && ip_ok && hostapd_ok; then log "AP recreado: ok ($(estaciones) estaciones)"; else log "AP recreado pero NO queda sano"; fi
}

log "vigilante iniciado"
while true; do
  motivo=""
  if usb_presente; then
    n=$(usb_num)                      # número de dispositivo nuevo = se desconectó y volvió entre dos comprobaciones
    if [ $usb_desde = 0 ] || [ "$n" != "$usb_num_antes" ]; then usb_desde=$(date +%s); usb_num_antes=$n; fi
    if [ $(( $(date +%s) - usb_desde )) -ge $USB_ESTABLE_S ]; then
      ap_en_usb || motivo="el AP no está en el adaptador USB"
    fi
  else
    usb_desde=0
  fi
  [ -z "$motivo" ] && { [ -e /sys/class/net/ap0 ] || motivo="ap0 no existe"; }
  [ -z "$motivo" ] && { ap_modo || motivo="ap0 no está en modo AP"; }
  [ -z "$motivo" ] && { ip_ok || motivo="ap0 sin la IP $IP"; }
  [ -z "$motivo" ] && { hostapd_ok || motivo="hostapd caído o sin habilitar"; }
  if [ -n "$motivo" ]; then
    fallo_txt="$motivo"; escribir_estado 0 "recuperando" "$motivo"; recrear "$motivo"
  elif ! dnsmasq_ok; then
    log "dnsmasq caído: reiniciando"; systemctl restart agrobot-dnsmasq.service
  else
    if usb_presente; then escribir_estado 1 "usb" "AP en la antena USB"; else escribir_estado 1 "respaldo_interna" "sin adaptador USB: AP en la radio interna"; fi
  fi
  sleep 10
done
