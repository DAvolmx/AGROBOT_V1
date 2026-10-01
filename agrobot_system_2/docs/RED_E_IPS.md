# Red del AgroBot — direcciones IP y cómo mantenerlas

Estado vigente desde el **2026-10-01**. Este documento manda sobre cualquier
plan de direcciones anterior (`192.168.12.2` para la Pi, `192.168.0.170` en
el gateway, etc.).

La idea: el router DIR-615 viaja con el sistema y reparte siempre las mismas
IP. Para llevar el robot a otro cultivo se conecta todo, se enciende, y solo
cambia **de dónde saca internet el router** (cable o el celular).

> Las claves no se guardan en git (igual que `CAMBIAR_AL_INSTALAR` en los
> archivos de sistema). Están anotadas fuera del repo, con el usuario.

---

## 1. Esquema

```
 Internet ── cable (Huawei / lo que haya)  ─┐
          └─ hotspot del celular "LORA" ─────┤  (respaldo, Cliente Wi-Fi)
                                             ▼
                    ┌───────────────────────────────────────┐
                    │ D-Link DIR-615X — modo ROUTER          │
                    │ LAN 192.168.0.1/24, DHCP .100–.200     │
                    │ Wi-Fi canal 1 FIJO, dos SSID:          │
                    │   "LoRa-Wan"  y  "LoRa_Wan" (misma red)│
                    └───────────────────────────────────────┘
        cable LAN │            Wi-Fi │                 Wi-Fi │
                  ▼                  ▼                       ▼
     Raspberry Pi (ChirpStack)   Jetson (cerebro)     Gateway Heltec HT-M02
        192.168.0.10             192.168.0.142            192.168.0.11
        (Wi-Fi: .20)               │                    → UDP 1700 a la Pi
                                   │ crea su propio AP
                                   ▼
                        JETSON_AP 192.168.12.0/24
                        (ESP32-C3 .137, ESP32 LoRa V3, celulares)
```

## 2. Tabla de direcciones

| Equipo | Conexión | IP | MAC | Cómo queda fija |
|---|---|---|---|---|
| Router DIR-615X | — | `192.168.0.1` | `bc:0f:9a:ee:64:a4` (LAN) | IP del router |
| Raspberry Pi `LoRa-Server-pi` | **cable** (principal) | `192.168.0.10` | `2c:cf:67:21:34:09` | reserva DHCP |
| Raspberry Pi | Wi-Fi `LoRa-Wan` (respaldo) | `192.168.0.20` | `2c:cf:67:21:34:0b` | reserva DHCP |
| Jetson | Wi-Fi `LoRa-Wan` | `192.168.0.142` | `74:70:fd:e3:ec:70` | reserva DHCP |
| Jetson | cable (sin usar) | — | `48:b0:2d:e9:f5:6b` | sin reserva (ver §4) |
| Gateway Heltec HT-M02 | Wi-Fi `LoRa_Wan` | `192.168.0.11` | `20:50:e7:16:03:0f` | **IP estática en el propio gateway** + reserva DHCP |
| Gateway Heltec (puerto de cable) | — | `192.168.4.1` (red propia) | `06:5f:31:49:b5:a6` | no conectar al router (ver §5) |
| Pool DHCP (celulares, PC) | Wi-Fi | `.100`–`.200` | — | dinámica |

Dentro del AP de la Jetson (red `192.168.12.0/24`) no cambió nada: Jetson
`192.168.12.1`, ESP32-C3 `192.168.12.137`, ESP32 LoRa V3 por reserva.

### Tailscale (acceso desde cualquier lugar)

| Equipo | Nombre | IP Tailscale |
|---|---|---|
| Jetson | `localhost-0` (conviene renombrarla) | `100.82.220.28` |
| PC del usuario | `david` | `100.65.91.7` |
| Raspberry Pi | `lora-server-pi` | la asigna Tailscale al autorizarla |

Dashboard de la Jetson desde fuera: `http://100.82.220.28:8080`.

## 3. Quién habla con quién

| Origen → destino | Puerto | Para qué | Dónde se configura |
|---|---|---|---|
| Heltec → Pi | UDP 1700 | paquetes LoRaWAN (Semtech UDP) | web del Heltec, *Gateway Setting → Server Addr* |
| Jetson → Pi | TCP 8092 `/api/alert` | alertas de plagas → Telegram | `GATEWAY_ALERT_URL` en `jetson/backend/jetson_server.py` (o `AGROBOT_GATEWAY_URL`) |
| cualquiera → Pi | 8080 / 8090 / 1883 | web ChirpStack / REST API / MQTT | Docker en la Pi |
| cualquiera → Jetson | 8080 | dashboard del robot | — |

Gateway ID del Heltec en ChirpStack: `065f31fffe49b5a6`, región `us915_0`.
Verificado el 2026-10-01: la Pi recibe sus estadísticas cada 30 s.

## 4. Reglas del router (aprendidas a la mala)

- **Nunca dos reservas DHCP con la misma IP.** La página/API lo acepta, pero
  el dnsmasq del router muere y la red se queda **sin DNS ni DHCP** para
  todos. Por eso el cable de la Jetson no tiene reserva: un equipo = una IP
  por interfaz.
- **Canal Wi-Fi fijo en 1.** La Jetson usa una sola radio para su AP y para
  conectarse al router; los dos deben estar en el mismo canal. El timer
  `agrobot-sync-ap-channel.timer` revisa cada minuto y mueve el AP de la
  Jetson al canal del router si cambia (p. ej. cuando el router sale a
  internet por el celular).
- **Dos SSID, misma red.** `LoRa-Wan` (Jetson, Pi, PC) y `LoRa_Wan` (el
  nombre antiguo, que es el que tiene guardado el Heltec y el sketch de
  banco `firmware/test_motores`). Misma clave, misma LAN, sin aislamiento.
- El DNS del router resuelve los nombres DHCP (`LoRa-Server-pi`, `HT-M02`,
  `jetson`); la Jetson aún no los usa porque su systemd-resolved no manda
  nombres de una sola palabra al DNS.
- Respaldo de la configuración del router: `respaldos/` (raíz del repo, fuera de git porque contiene claves).
- Administración: `http://192.168.0.1`, usuario `admin`.

## 5. Gateway Heltec HT-M02

- Web: `http://192.168.0.11` (usuario `root`). El SSH tiene otra clave.
- Tiene dos modos de red **excluyentes**:
  - *ETH WAN + WIFI AP*: sale por cable y crea el Wi-Fi `HT-M02-AP` en `192.168.4.1`.
  - *WIFI Client + ETH LAN* (**el que usamos**): sale por Wi-Fi y su puerto
    de cable pasa a ser una red propia `192.168.4.1`. **No conectar ese
    cable al router**: pide IP al arrancar y después queda mudo.
- Configuración vigente: Wi-Fi `LoRa_Wan`, IP estática `192.168.0.11/24`,
  gateway `192.168.0.1`, UDP Packet Forwarder, US915 canales 0-7,64,
  servidor `192.168.0.10`, puertos 1700/1700.

## 6. Raspberry Pi

- SSH: `admin@192.168.0.10`. ChirpStack v4 en Docker (`~/chirpstack`).
- Wi-Fi: perfil NetworkManager `LoRa-Wan` (antes decía `LoRa_Wan`, por eso
  no se conectaba sin cable), métrica 600 → el cable manda si está.

## 7. Mudanza a otro cultivo (checklist)

1. Router a la corriente; la Pi por cable a un puerto LAN.
2. Encender Jetson y Heltec (el Heltec **sin** cable de red).
3. Internet: cable al puerto *Internet* del router **o** hotspot "LORA" del
   celular encendido (Cliente Wi-Fi del router, respaldo de WAN).
4. Comprobar: `http://192.168.0.142:8080` (robot), `http://192.168.0.10:8080`
   (ChirpStack), `http://192.168.0.11` (Heltec). Desde fuera, por Tailscale.

## 8. Pendiente (2026-10-01)

- Autorizar la Pi en Tailscale (enlace generado con `tailscale up`).
- Reiniciar `agrobot-backend` (requiere sudo) para que las alertas vayan a
  `192.168.0.10:8092`.
- Configurar en el router el Cliente Wi-Fi "LORA" + reserva de WAN.
