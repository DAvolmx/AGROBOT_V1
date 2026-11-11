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

## 2026-11-03 — adaptador nuevo 0bda:8179
El adaptador que se usa ahora es un RTL8188EUS (`0bda:8179`, MAC `00:e0:4c:87:2c:33`), no el RTL8188ETV (`0bda:0179`)
del principio. `agrobot-ap0-up.sh` y `agrobot-ap-watchdog.sh` (copia aquí) solo buscaban `0179`, así que el vigilante
nunca movía el AP a la antena externa y se quedaba en la radio interna. Ahora aceptan los dos, y
`/etc/NetworkManager/conf.d/99-agrobot-usb-ap.conf` excluye cualquier dispositivo con driver `8188eu`.
Al enchufarlo con el AP ya en la radio interna, el vigilante recrea el AP en el USB en ≤10 s (C3 y nodo se reasocian ~20 s).
Conectarlo directo a la Jetson: en el hub salió a full-speed ("not running at top speed").

## 2026-11-11 — adaptador nuevo 0bda:f179 (RTL8188FTV)
- Chip RTL8188FU, MAC `34:7d:e4:46:88:37`, interfaz `wlx347de4468837`. El `8188eu` no lo soporta y el kernel tegra
  no trae `rtl8xxxu` (`CONFIG_RTL8XXXU` no está activado). Se compiló `rtl8xxxu` desde `/usr/src/linux-source-6.8.0`, solo
  esa carpeta, en `~/.agrobot/drivers/rtl8xxxu_8188fu/` (no versionado):
  `make -C /lib/modules/$(uname -r)/build M=$PWD CONFIG_RTL8XXXU=m CONFIG_RTL8XXXU_UNTESTED=y modules`.
  Instalado en `/lib/modules/<kernel>/extra/rtl8xxxu.ko` + `/etc/modules-load.d/rtl8xxxu.conf`. **Tras actualizar el kernel
  hay que recompilarlo.**
- Firmware: el kernel no lee `.zst`. Se descomprimió `/lib/firmware/rtlwifi/rtl8188fufw.bin.zst` a `.bin`; sin eso, el
  probe falla con -11 (lo mismo que pasó con el Bluetooth Intel).
- Los scripts aceptan `0bda:[08f]179` y los drivers `8188eu|rtl8xxxu`. `agrobot-ap0-down.sh` renombra a `wlx<MAC>`
  en vez de usar un nombre fijo. NM excluye `driver:rtl8xxxu` y la MAC nueva. Respaldos `*.pre_f179_20261111` en `/usr/local/sbin`
  y `/etc/NetworkManager/conf.d/.99-agrobot-usb-ap.conf.pre_f179_20261111`.
- Resultado: el vigilante movió el AP al USB en 12 s, `verificar_ap_usb.sh` dio 11 OK, C3 a -34 dBm y la otra estación a
  -50 dBm (MCS 7), ping a la C3 de 2,9 ms y 0 % de pérdida, potencia 20 dBm en el canal 1.
- Ojo: `rtl8xxxu` también reconoce el `0bda:8179` viejo. Si se vuelve a enchufar, puede tomarlo cualquiera de los dos drivers.
