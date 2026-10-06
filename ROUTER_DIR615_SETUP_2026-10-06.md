# 🌐 DIR-615 — Configuración CNT-RUIZ (Cliente)

## 📋 Objetivo

El **DIR-615** actúa como **cliente WiFi de CNT-RUIZ** (internet) y **distribuye conectividad** a toda la red AgroBot local vía:
- ✅ **AP LoRa_Wan** (192.168.0.1) — Para robots locales
- ✅ **Cliente CNT-RUIZ** — Para internet externo
- ✅ **DHCP** — Sirve IPs 192.168.0.50-200 a dispositivos

---

## ⚙️ **CONFIGURACIÓN MANUAL (Interfaz Web)**

### Paso 1: Acceder al Router

```
URL: http://192.168.0.1
Usuario: admin
Contraseña: admin1
```

### Paso 2: Configurar AP Local (LoRa_Wan)

Menú: **Advanced → Wireless → Basic Settings**

| Campo | Valor |
|-------|-------|
| Enable AP | ✅ Activado |
| SSID | `LoRa_Wan` |
| Channel | `1` (fijo, debe coincidir con Jetson) |
| Bandwidth | `20 MHz` |
| Security Mode | `WPA2-PSK` |
| WPA Passphrase | `admin1` |

**Guardar/Aplicar**

### Paso 3: Conectar como Cliente a CNT-RUIZ

Menú: **Advanced → Wireless → Site Survey** (Escanear redes)

1. Click en **"Scan"** → Buscar `CNT-RUIZ`
2. Click en **"Connect"** junto a `CNT-RUIZ`
3. Ingresa contraseña: `1850370113`
4. Click **"Connect"**

**Esperar 30 segundos** mientras se conecta...

### Paso 4: Verificar Conexión

Menú: **Status → Wireless**

Debe mostrar:
- **AP (LoRa_Wan)**: Activo
- **Cliente (CNT-RUIZ)**: Conectado
- **DHCP Status**: Repartiendo IPs

### Paso 5: Configurar DHCP (si es necesario)

Menú: **Network → DHCP Server**

| Campo | Valor |
|-------|-------|
| DHCP Server | ✅ Activado |
| IP Range Start | `192.168.0.50` |
| IP Range End | `192.168.0.200` |
| Default Gateway | `192.168.0.1` |
| DNS 1 | `8.8.8.8` (Google) |
| DNS 2 | `8.8.4.4` (Google) |

**Guardar/Aplicar**

### Paso 6: Guardar y Reiniciar

Menú: **System Tools → Reboot**

Click **"Reboot"** y esperar 2 minutos

---

## ✅ **VERIFICACIÓN POST-CONFIGURACIÓN**

### 1. Red LoRa_Wan Disponible
```bash
# En Jetson
nmcli dev wifi list
# Debe mostrar: LoRa_Wan (seguridad: WPA2-PSK)
```

### 2. Router Conectado a CNT-RUIZ
```bash
# Acceder a router: http://192.168.0.1
# Status → Wireless
# Debe mostrar: "CNT-RUIZ: Connected"
```

### 3. IPs Asignadas a Clientes
```bash
# En Jetson
sudo nmcli dev wifi connect "LoRa_Wan" password "admin1"
ip addr show wlan0
# Debe mostrar: 192.168.0.x/24
```

### 4. Internet Funciona
```bash
# En Jetson (si está conectada a LoRa_Wan)
ping 8.8.8.8
# Debe responder
```

---

## 🔄 **TOPOLOGÍA FINAL**

```
┌─────────────────────────────────────────┐
│      INTERNET (Extranet)                │
│      "Mundo exterior"                   │
└────────────────┬────────────────────────┘
                 │
         ┌───────▼────────┐
         │   CNT-RUIZ     │  ← Modem/Router vecino
         │  (internet)    │     Contraseña: 1850370113
         └───────┬────────┘
                 │
         ┌───────▼────────────────┐
         │   DIR-615 (Router)     │
         │ • AP: LoRa_Wan         │
         │ • Cliente: CNT-RUIZ    │
         │ • IP: 192.168.0.1      │
         │ • DHCP: 50-200         │
         └───┬────────────────┬───┘
             │                │
      ┌──────▼──────┐   ┌─────▼──────┐
      │  JETSON AP  │   │  LoRa V3   │
      │ JETSON_AP   │   │ (cliente)  │
      │192.168.12.1 │   │192.168.0.x │
      │             │   │            │
      │ Dashboard   │   │  LoRa-Wan  │
      │ :8000       │   │  Gateway   │
      └──────┬──────┘   └────────────┘
             │
         ┌───▼──────┐
         │ ESP32-C3 │  ← Motor
         │ (WiFi)   │
         └──────────┘

FLUJO DE DATOS:
  • Internet → CNT-RUIZ → DIR-615 → Jetson/Robots
  • Robots → Jetson (WiFi AP local)
  • Móvil → Jetson (WiFi AP) O Bluetooth
```

---

## 🚨 **TROUBLESHOOTING**

### Router No Se Conecta a CNT-RUIZ
```
Solución:
1. Reiniciar router (Reboot)
2. Verificar contraseña: 1850370113
3. Acercarse más a CNT-RUIZ
4. En site survey, forzar reconexión
5. Si aún falla: resetear router a factory defaults y reintentar
```

### Jetson No Ve LoRa_Wan
```
Solución:
1. Verificar router está en AP mode (no solo cliente)
2. Verificar canal: debe ser 1 (iw dev wlan0 info en Jetson)
3. Reiniciar hostapd: sudo systemctl restart agrobot-hostapd.service
4. Reiniciar router
```

### IPs No Se Asignan
```
Solución:
1. Verificar DHCP server está activado en router
2. Reiniciar DHCP: en router → DHCP Server → ON → Guardar
3. Esperar 30 segundos
4. Reconectar cliente: nmcli dev disconnect wlan0 && nmcli dev wifi connect...
```

### Sem Internet en Jetson
```
Solución:
1. Verificar conexión CNT-RUIZ: Status → Wireless
2. Verificar DNS en Jetson: cat /etc/resolv.conf (debe tener 8.8.8.8)
3. Probar: ping 8.8.8.8
4. Si no pasa: tracert → ver donde se detiene
5. Reiniciar router y Jetson
```

---

## 📊 **TABLA DE CONFIGURACIÓN FINAL**

| Dispositivo | SSID/Red | IP | Usuario | Contraseña | Función |
|-------------|----------|-----|---------|------------|---------|
| DIR-615 | AP: LoRa_Wan | 192.168.0.1 | admin | admin1 | Router/Distribuidor |
| DIR-615 | Cliente: CNT-RUIZ | Dinámico | - | 1850370113 | Internet Externo |
| JETSON | AP: JETSON_AP | 192.168.12.1 | - | agrobot123 | AP Local |
| JETSON | Cliente: LoRa_Wan (opt) | 192.168.0.x | - | admin1 | Redundancia |
| ESP32-C3 | Cliente: JETSON_AP | 192.168.12.x | - | agrobot123 | Motor Robot |
| LoRa V3 | Cliente: JETSON_AP | 192.168.12.x | - | agrobot123 | LoRa Gateway |
| Móvil | WiFi: JETSON_AP | 192.168.12.x | - | agrobot123 | Operador |

---

## 🎯 **CHECKLIST POST-INSTALACIÓN**

```
✅ DIR-615 alimentado y botones LED verdes
✅ AP LoRa_Wan visible en WiFi scanning
✅ Conecta a CNT-RUIZ (LED WiFi verde)
✅ Jetson obtiene IP 192.168.0.x (si se conecta a LoRa_Wan)
✅ JETSON_AP disponible en WiFi scanning
✅ ESP32-C3 conectado a JETSON_AP
✅ Dashboard accesible en http://192.168.12.1:8000
✅ Internet disponible en Jetson (si conectada a LoRa_Wan)
✅ Todos los servicios en verde: systemctl status agrobot-*.service
```

---

## 🔧 **RESET FACTORY (Si algo sale muy mal)**

1. Router → System Tools → Factory Defaults
2. Click "Yes"
3. Esperar 2 minutos (router se reinicia)
4. Volver a configurar desde "Paso 1"

---

**Generado**: 2026-10-06  
**Router**: D-Link DIR-615  
**Estado**: ✅ LISTO PARA USAR
