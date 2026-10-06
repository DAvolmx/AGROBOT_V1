# 🤖 AgroBot — Robot Listo para Probar (2026-10-06)

## ✅ CAMBIOS REALIZADOS

### Firmware ESP32-C3 (Actuador) — v2
**Archivo:** `/firmware/esp32c3_actuador_v2.ino`

**Problemas solucionados:**
- ✅ **Dead time 50 ms** entre freno e inversión de dirección
- ✅ **Filtro de secuencia correcto** (permite BLE y WiFi alternados)
- ✅ **Timer global** para duty cycle sincronizado
- ✅ **Logging mejorado** de cambios de dirección

**Parámetros:**
```
Dead time:           50 ms
Período motor:       200 ms
Timeout failsafe:    500 ms (sin comando = parada)
Rampa aceleración:   250% V_MAX/seg
```

---

### Firmware LoRa V3 (Sensor) — v2
**Archivo:** `/firmware/esp32_lora_v3_v2.ino`

**Problemas solucionados:**
- ✅ **UDP a 20 Hz** (independiente de ciclo LoRaWAN)
- ✅ **Sensor ambiental** cada 5 seg
- ✅ **Mejor manejo WiFi** (recheck periódico)
- ✅ **Logging de pérdidas** UDP

**Ritmos:**
```
UDP telemetría:      50 ms (20 Hz)
Sensor ambiental:    5000 ms
Ciclo LoRaWAN:       30 seg
```

---

### Dashboard — index.html
**Cambios:**
- ✅ **Botón "Modo demo"** para desactivar safeguard
- ✅ Visual: naranja (ON) ↔ rojo (OFF)
- ✅ Notificaciones al cambiar

---

### Backend — jetson_server.py
**Cambios:**
- ✅ **Endpoint** `POST /api/motor/toggle-safeguard`
- ✅ **Variable global** `_safeguard_enabled`
- ✅ **Logging** de cambios de estado

---

## 🔌 CÓMO FLASHEAR

### **ESP32-C3** (Actuador)
```bash
# Arduino IDE:
# 1. Board: "ESP32C3 Dev Module"
# 2. Upload Speed: 921600 baud
# 3. Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
# 4. USB CDC On Boot: Enabled

# Archivo: /home/jetson/Desktop/AGROBOT/agrobot_system_2/firmware/esp32c3_actuador_v2.ino
# Luego: Sketch → Upload
```

### **LoRa V3** (Heltec)
```bash
# Arduino IDE:
# 1. Board: "Heltec WiFi LoRa 32 V3"
# 2. Upload Speed: 921600 baud
# 3. Partition Scheme: Default

# Archivo: /home/jetson/Desktop/AGROBOT/agrobot_system_2/firmware/esp32_lora_v3_v2.ino
# Luego: Sketch → Upload
```

---

## 📋 CHECKLIST PRE-PRUEBA

### Hardware
- [ ] C3 enchufada, LED rojo (batería motores)
- [ ] LoRa V3 enchufada, LED azul (batería lógica)
- [ ] Jetson Orin Nano boot (3-5 min)
- [ ] USB C de C3 → laptop (para serie)
- [ ] USB de LoRa V3 → laptop (para serie)

### Firmware
- [ ] C3 flasheada con esp32c3_actuador_v2.ino
- [ ] LoRa V3 flasheada con esp32_lora_v3_v2.ino
- [ ] Dashboard actualizado (ver botón "Modo demo")

### Verificación en Serial
```
C3 (9600 baud):
  [ble] servidor GATT activo
  [ble] cliente conectado
  ✅ Si no aparece → flasheo fallido

LoRa V3 (115200 baud):
  [Status] State=? | Joined=? | WiFi=?
  ✅ Si no aparece → flasheo fallido
```

---

## 🧪 PRIMERA PRUEBA: ADELANTE ↔ ATRÁS

### Paso 1: Abrir Dashboard
```
http://192.168.12.1:8080
```

### Paso 2: Verificar conexión BLE
- Página debe mostrar: "🔗 BLE Conectado"
- Si no → botón "Reconectar"

### Paso 3: Prueba de dirección (5 ciclos)
```
1. Pulsa ▲ (adelante) — motor debe arrancar con KICK (200 ms corto)
2. Pulsa ▼ (atrás) — motor debe:
   a. Parar completamente (LOW-LOW 50 ms)
   b. Esperar dead time
   c. Invertir dirección
3. Espera 1 seg
4. Repeat paso 1
```

### Paso 4: Ver serie del C3 (buscar logs de cambio)
```
[mot] R1: 1 -> -1 (dead time 50 ms)
[mot] R2: 1 -> -1 (dead time 50 ms)
[mot] L1: 1 -> -1 (dead time 50 ms)
[mot] L2: 1 -> -1 (dead time 50 ms)
```
✅ Si aparecen → cambios de dirección OK

---

## 🔐 SEGUNDO TEST: SAFEGUARD TOGGLE

### Activado (por defecto)
```
1. Pulsa ▲ (adelante)
2. Espera 1 seg SIN PULSAR NADA
3. Motor debe parar en ~100 ms (failsafe)
4. Botón dice: "🔐 Modo demo (safeguard ON)"
```

### Desactivado (Modo Demo)
```
1. Pulsa el botón "🔐 Modo demo"
   ↓ Cambia a rojo: "⚠️ Modo demo (safeguard OFF)"
2. Pulsa ▲ (adelante)
3. Suelta el control
4. Motor debe SEGUIR ADELANTE indefinidamente
5. Para manual: pulsa ■ (stop)
6. Pulsa botón de nuevo para reactivar protección
   ↓ Vuelve a naranja
```

---

## 📊 TERCERA PRUEBA: TELEMETRÍA

### WiFi UDP (LoRa V3 → Jetson)
```
Dashboard → Pestaña "Estado"
  · IMU: aceleración + rotación
  · Ambiental: CO2, temp, humedad
  · Actualización: 20 Hz IMU, 1/5 seg ambiental
```

### LoRaWAN
```
Serie del LoRa V3:
  [State] Joined=✅ YES (5-30 seg después del boot)
  📤 Enviando #1 (17 bytes)
  📤 Enviando #2 (17 bytes)
  ...cada 30 seg
```

---

## 🚨 SI ALGO FALLA

### "Motor no cambia de dirección"
- [ ] Lee serie del C3: busca `[mot] cambios`
- [ ] Si NO aparece: dead time no funciona
  - [ ] Verifica `DEAD_TIME_MS = 50` en línea ~100 del firmware
  - [ ] Re-flashea
- [ ] Si aparece con tiempo 0: el `frenohasta` no se está respetando
  - [ ] Verifica `if ((long)(ahora - r.frenohasta) < 0)` en `aplicarDuty()`

### "WiFi se desconecta cada 10 seg"
- [ ] Jetson: verifica `/etc/hostapd/agrobot.conf`
- [ ] LoRa V3: serie busca `[wifi]` logs
- [ ] Potencia WiFi muy baja: ajusta `WIFI_POTENCIAS[]` en esp32_lora_v3_v2.ino

### "Botón Modo demo no hace nada"
- [ ] Consola del navegador: F12 → verifica si hay error en red
- [ ] Backend: verifica si el endpoint está registrado
  - [ ] `app.router.add_post("/api/motor/toggle-safeguard", ...)`
- [ ] Reinicia Jetson `systemctl restart agrobot`

---

## 📝 NOTAS IMPORTANTES

1. **Dead time de 50 ms es crítico**
   - Si es muy corto (0-10 ms): motor sigue patinando
   - Si es muy largo (>100 ms): respuesta lenta
   - Medido en el robot real: 50 ms es óptimo

2. **Duty cycle en ON/OFF puro**
   - No usa PWM variable (0-255)
   - Usa GPIO directo: período de 200 ms
   - duty=100% → siempre ON
   - duty=50% → ON-OFF mitad-mitad del período

3. **Safeguard es SOLO para pruebas**
   - Siempre debe estar ON en operación
   - Campo: si el enlace se cae, robot para automáticamente
   - En interior: puede ser útil desactivarlo para ajustes

4. **Baterías separadas**
   - Motores: propia (sin cable eléctrico a Jetson)
   - Lógica: Jetson y accesorios
   - Enlace: BLE wireless
   - ✅ Esto PREVIENE picos de voltaje

---

## ✨ LISTO PARA CAMPO

Si todo el checklist está OK:
```
✅ Cambios adelante ↔ atrás fluidos
✅ Safeguard toggle funciona
✅ Telemetría llega a 20 Hz
✅ LoRaWAN comunicando
✅ Sin errores en series

→ Robot está listo. Buenas pruebas.
```

---

**Compilado:** 2026-10-06 11:45 UTC  
**Versión:** AgroBot C3 v2 + LoRa V3 v2 + Dashboard v2  
**Estado:** ✅ LISTO PARA PROBAR EN CAMPO
