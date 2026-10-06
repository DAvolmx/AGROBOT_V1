# 🤖 AGROBOT — Sistema Autónomo 2026-10-06

## 📋 Resumen de Cambios

El sistema se ha **COMPLETAMENTE REDISEÑADO** para ser **100% autónomo**:

### ✅ **Problemas Resueltos**

| Problema | Causa | Solución |
|----------|-------|----------|
| AP no levantaba | Contraseña: `CAMBIAR_AL_INSTALAR` | ✅ Corregida a `agrobot123` |
| Sin DHCP en AP | No había dnsmasq | ✅ Instalado dnsmasq con DHCP 192.168.12.50-200 |
| Sin conexión externa | Sin reconexión automática | ✅ Script auto-connect a CNT-RUIZ + LoRa_Wan |
| Sin redundancia BT | Solo HTTP | ✅ Dashboard por Bluetooth (RFCOMM) |
| Sin monitoreo | Sin servicios systemd completos | ✅ 5 servicios systemd nuevos |

---

## 🚀 **INSTALACIÓN RÁPIDA**

### Opción A: Instalación Automática (RECOMENDADO)

```bash
cd /home/jetson/Desktop/AGROBOT/agrobot_system_2/jetson/systemd
sudo bash install-services.sh
```

Esto:
- ✅ Copia configuración a `/etc/hostapd` y `/etc/dnsmasq.d`
- ✅ Instala 5 servicios systemd
- ✅ Habilita autostart en boot
- ✅ Verifica dependencias

### Opción B: Instalación Manual

```bash
# 1. Copiar hostapd config
sudo cp /home/jetson/Desktop/AGROBOT/agrobot_system_2/jetson/systemd/etc/hostapd/agrobot.conf /etc/hostapd/

# 2. Copiar dnsmasq config
sudo mkdir -p /etc/dnsmasq.d
sudo cp /home/jetson/Desktop/AGROBOT/agrobot_system_2/jetson/systemd/etc/dnsmasq.d/agrobot-ap.conf /etc/dnsmasq.d/

# 3. Copiar servicios
sudo cp /home/jetson/Desktop/AGROBOT/agrobot_system_2/jetson/systemd/agrobot-*.service /etc/systemd/system/

# 4. Recargar systemd
sudo systemctl daemon-reload

# 5. Habilitar servicios
sudo systemctl enable agrobot-{ap0,hostapd,dnsmasq,wifi-auto,bluetooth-gw}.service

# 6. Iniciar
sudo systemctl restart agrobot-ap0.service
```

---

## 🔌 **ARQUITECTURA DEL SISTEMA**

```
                    ┌─────────────────────────┐
                    │  CNT-RUIZ (Internet)    │
                    │  1850370113             │
                    └────────────┬────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │  DIR-615 (Router)       │
                    │  192.168.0.1            │
                    │  Cliente WiFi CNT-RUIZ  │
                    │  Distribuye a:          │
                    │  • 192.168.0.x (DHCP)   │
                    │  • LoRa_Wan (optional)  │
                    └──┬──────────┬───────────┘
                       │          │
                       │          └─── LoRa_Wan (opcional)
                       │               admin/admin1
                       │
        ┌──────────────▼──────────────────────────┐
        │         JETSON NANO (AP Local)          │
        ├──────────────────────────────────────────┤
        │                                          │
        │  ┌──────────────┐                       │
        │  │  ap0 (vif)   │                       │
        │  │ 192.168.12.1 │                       │
        │  └──────────────┘                       │
        │       │                                  │
        │   hostapd                                │
        │  (JETSON_AP)                             │
        │   agrobot123                             │
        │                                          │
        │  ┌───────────────────────────────────┐  │
        │  │ Backend HTTP (puerto 8000)        │  │
        │  │ • REST API (/api/motor/*, etc)    │  │
        │  │ • Dashboard (index.html)          │  │
        │  └───────────────────────────────────┘  │
        │     │                     │              │
        │     ├─ WiFi: 192.168.12.1:8000         │
        │     └─ BT: RFCOMM puerto 1              │
        │                                          │
        │  ┌───────────────────────────────────┐  │
        │  │ dnsmasq (DHCP 192.168.12.50-200)  │  │
        │  │ • Sirve IPs a ESP32, móviles       │  │
        │  └───────────────────────────────────┘  │
        │                                          │
        │  ┌───────────────────────────────────┐  │
        │  │ BT Gateway (RFCOMM) ← Redundancia │  │
        │  └───────────────────────────────────┘  │
        │                                          │
        └──────────────────────────────────────────┘

REDUNDANCIA DE REDES:
  • ESP32-C3 → WiFi JETSON_AP + BLE backup
  • LoRa V3 → WiFi JETSON_AP + WiFi DIR-615 backup
  • Móvil → WiFi JETSON_AP O Bluetooth
  • Internet → DIR-615 (CNT-RUIZ) O LoRa_Wan
```

---

## 📡 **REDES DISPONIBLES**

### 1️⃣ **JETSON_AP** (AP Local)
- **SSID**: `JETSON_AP`
- **Contraseña**: `agrobot123`
- **IP**: `192.168.12.1`
- **DHCP**: `192.168.12.50` — `192.168.12.200`
- **Uso**: Robots locales, móviles, patrullaje

### 2️⃣ **CNT-RUIZ** (Red Residencial)
- **SSID**: `CNT-RUIZ`
- **Contraseña**: `1850370113`
- **Automático**: Jetson intenta conectar si disponible
- **Uso**: Internet, monitoreo remoto

### 3️⃣ **LoRa_Wan** (Router LoRa-Wan)
- **SSID**: `LoRa_Wan`
- **Usuario**: `admin`
- **Contraseña**: `admin1`
- **IP**: `192.168.0.1`
- **Automático**: Fallback si CNT-RUIZ no está
- **Uso**: Redundancia, redes LoRa

---

## 🌐 **ACCESO AL DASHBOARD**

### Opción A: WiFi (RECOMENDADO)
```
http://192.168.12.1:8000
```
- Conectar a `JETSON_AP` con contraseña `agrobot123`
- Abrir navegador en teléfono/PC
- Acceder a URL anterior

### Opción B: Bluetooth (Móvil Android)
```
Bluetooth → Emparejar "AGROBOT" → RFCOMM Puerto 1
URL: http://127.0.0.1:8000  (emulado localmente)
```
- Requiere app que soporte RFCOMM (Socket)
- Mayor redundancia si WiFi cae

### Opción C: Internet (Si conectado a CNT-RUIZ)
- Jetson obtiene IP en 192.168.x.x (DHCP del router)
- Desde otra red: acceso remoto (requiere port forwarding)

---

## ⚙️ **COMANDOS DE GESTIÓN**

### Ver Estado de Servicios
```bash
sudo systemctl status agrobot-*.service
```

### Ver Logs
```bash
# Todo
sudo journalctl -u agrobot-ap0.service -f

# Solo un servicio
sudo journalctl -u agrobot-hostapd.service -f
sudo journalctl -u agrobot-dnsmasq.service -f
sudo journalctl -u agrobot-wifi-auto.service -f
sudo journalctl -u agrobot-bluetooth-gw.service -f
```

### Reiniciar Sistema
```bash
# Reiniciar todo
sudo systemctl restart agrobot-ap0.service agrobot-hostapd.service agrobot-dnsmasq.service

# Reiniciar WiFi auto-connect
sudo systemctl restart agrobot-wifi-auto.service

# Reiniciar Bluetooth
sudo systemctl restart agrobot-bluetooth-gw.service
```

### Detener Servicios
```bash
sudo systemctl stop agrobot-*.service
```

### Habilitar/Deshabilitar en Boot
```bash
# Habilitar
sudo systemctl enable agrobot-ap0.service

# Deshabilitar
sudo systemctl disable agrobot-ap0.service
```

---

## 🔍 **VERIFICACIÓN PUNTO A PUNTO**

### 1. AP Levantado ✅
```bash
sudo systemctl status agrobot-hostapd.service
# Debe estar "active (running)"
```

### 2. DHCP Funcionando ✅
```bash
sudo systemctl status agrobot-dnsmasq.service
# Debe estar "active (running)"
```

### 3. Interfaz ap0 Activa ✅
```bash
ip addr show ap0
# Debe mostrar: 192.168.12.1/24
```

### 4. WiFi Auto-Connect Intentando ✅
```bash
sudo journalctl -u agrobot-wifi-auto.service -f
# Debe mostrar intentos de conexión a CNT-RUIZ / LoRa_Wan
```

### 5. Bluetooth Gateway Funcionando ✅
```bash
sudo journalctl -u agrobot-bluetooth-gw.service -f
# Debe mostrar RFCOMM abierto
```

### 6. Backend HTTP Responde ✅
```bash
curl http://127.0.0.1:8000/api/estado
# Debe devolver JSON con estado del robot
```

### 7. Clientes Conectados ✅
```bash
sudo nmcli device wifi
# Debe listar JETSON_AP disponible

sudo nmcli connection show
# Debe mostrar conexión activa a JETSON_AP (o CNT-RUIZ/LoRa_Wan)
```

---

## 🛠️ **TROUBLESHOOTING**

### AP No Aparece en WiFi Scanning
```bash
# Verificar hostapd
sudo systemctl restart agrobot-hostapd.service
sudo journalctl -u agrobot-hostapd.service -n 20

# Verificar ap0 existe
ip link show ap0

# Recriar ap0
sudo systemctl restart agrobot-ap0.service
```

### Sin DHCP (No obtiene IP)
```bash
# Verificar dnsmasq
sudo systemctl restart agrobot-dnsmasq.service
sudo journalctl -u agrobot-dnsmasq.service -n 20

# Probar manualmente
sudo dhclient ap0  # Si no está configurado como AP
```

### No Conecta a CNT-RUIZ
```bash
# Ver redes disponibles
sudo nmcli dev wifi rescan
sudo nmcli dev wifi list

# Conectar manual
sudo nmcli dev wifi connect "CNT-RUIZ" password "1850370113"

# Ver estado
nmcli connection show
```

### Dashboard No Responde
```bash
# Verificar puerto
sudo lsof -i :8000

# Reiniciar backend
sudo systemctl restart agrobot-backend.service

# Probar localmente
curl http://127.0.0.1:8000/
```

---

## 📦 **ARCHIVOS NUEVOS/MODIFICADOS**

### Creados
- ✅ `agrobot-dnsmasq.service` — Servicio DHCP
- ✅ `agrobot-wifi-auto.service` — Auto-conexión WiFi
- ✅ `agrobot-bluetooth-gw.service` — Gateway BT
- ✅ `wifi_auto_connect.py` — Script auto-conexión
- ✅ `bluetooth_gateway.py` — Script gateway BT
- ✅ `etc/dnsmasq.d/agrobot-ap.conf` — Config DHCP
- ✅ `install-services.sh` — Instalador

### Modificados
- ✅ `etc/hostapd/agrobot.conf` — Contraseña: `agrobot123` (era `CAMBIAR_AL_INSTALAR`)

---

## 🚨 **NOTA IMPORTANTE: PRIMERA INSTALACIÓN**

**La Jetson DEBE tener**:
- ✅ Python 3.8+
- ✅ hostapd instalado (`sudo apt-get install hostapd`)
- ✅ dnsmasq instalado (`sudo apt-get install dnsmasq`)
- ✅ NetworkManager instalado (`sudo apt-get install network-manager`)
- ✅ socat instalado (`sudo apt-get install socat`)  ← Para BT
- ✅ rfcomm tools (`sudo apt-get install bluez`)

**El instalador intenta instalar todo automáticamente**, pero si falla:
```bash
sudo apt-get update
sudo apt-get install -y hostapd dnsmasq network-manager socat bluez
```

---

## 📊 **ESTADO DEL SISTEMA (Post-Instalación)**

Después de ejecutar `install-services.sh`, el sistema debe estar **100% funcional**:

```
✅ AP "JETSON_AP" disponible en todo dispositivo WiFi cercano
✅ DHCP activo: nuevos clientes obtienen IP 192.168.12.x automáticamente
✅ Dashboard accesible en http://192.168.12.1:8000
✅ Bluetooth disponible: RFCOMM puerto 1 para acceso redundante
✅ WiFi auto-connect intentando CNT-RUIZ y LoRa_Wan en paralelo
✅ Sistema totalmente autónomo sin intervención manual
✅ Logs en journalctl para debugging
✅ Servicios con restart automático en caso de fallo
```

---

## 🎯 **PRÓXIMOS PASOS**

1. **Instalar**: `sudo bash install-services.sh`
2. **Verificar**: `sudo systemctl status agrobot-*.service`
3. **Conectar**: Buscar WiFi "JETSON_AP", contraseña "agrobot123"
4. **Acceder**: `http://192.168.12.1:8000`
5. **Usar**: Dashboard con control total del robot

---

**Generado**: 2026-10-06  
**Estado**: ✅ PRODUCCIÓN  
**Versión**: 2.0 (Autonomía Completa)
