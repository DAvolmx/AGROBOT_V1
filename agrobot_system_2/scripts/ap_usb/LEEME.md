# AP en el adaptador USB RTL8188ETV (2026-10-16)

`ap0` (192.168.12.1/24, SSID JETSON_AP) se crea sobre el adaptador USB si está; si no, sobre la
radio interna Intel como antes (respaldo automático). hostapd, dnsmasq y el backend no cambian.

Piezas (viven FUERA del repo; aquí hay copia):
- `/usr/local/sbin/agrobot-ap0-up.sh` y `-down.sh`, unidad `agrobot-ap0.service`.
- Driver `drivers/rtl8188eus` (aircrack-ng, NO versionado): `8188eu.ko` instalado en
  `/lib/modules/<kernel>/extra/` + `/etc/modules-load.d/8188eu.conf`. Tras actualizar el kernel hay
  que recompilar (`make ARCH=arm64`) e instalar; mientras tanto el AP cae a la radio interna.
- `/etc/NetworkManager/conf.d/99-agrobot-usb-ap.conf`: NM no gestiona el adaptador.
- `agrobot-sync-ap-channel.sh`: sale sin hacer nada si el AP está en el USB (ya no comparten radio).

**NUNCA** `iw dev <adaptador> del`: este driver se cuelga con el bloqueo de red del kernel y deja
NetworkManager, wpa_supplicant y ROS en estado D hasta reiniciar. Para parar se renombra (ver down.sh).

Volver a la radio interna: `sudo cp agrobot-ap0.service.ORIGINAL_radio_interna /etc/systemd/system/agrobot-ap0.service && sudo systemctl daemon-reload && sudo systemctl restart agrobot-ap0`
(con el adaptador desenchufado o sin el módulo cargado).
