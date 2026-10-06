# Firmware Corregido — AgroBot 2026-10-06

## ⚠️ PROBLEMAS IDENTIFICADOS Y SOLUCIONADOS

### 1. **Cambios de Dirección Fallidos** (CRÍTICO)
**Síntoma:** Al pasar de adelante → atrás el motor no cambiaba de dirección limpiamente.

**Causa Raíz:**
- No había *dead time* (tiempo muerto) entre freno e inversión
- El motor pasaba de FI-BI ambos LOW a FI-HIGH SIN ESPERA
- Con duty cycle poco sincronizado, el cambio de fase podía ser abrupto

**Solución (esp32c3_actuador_v2.ino):**
```cpp
// ⭐ NUEVO: dead time explícito de 50 ms entre freno e inversión
const unsigned long DEAD_TIME_MS = 50;

// En setRueda():
if (sentido != r.sentido) {
    motorFreno(r.pinFI, r.pinBI);
    r.frenohasta = ahora + DEAD_TIME_MS;  // ← espera controlada
    ...
}

// En aplicarDuty():
// Verifica dead time antes de aplicar duty
if ((long)(ahora - r.frenohasta) < 0) {
    motorFreno(r.pinFI, r.pinBI);
    return;
}
```

### 2. **Filtro de Secuencia Defectuoso**
**Síntoma:** A veces se descartaban comandos válidos entre BLE y WiFi.

**Causa:** El check `d > -1000` permitía hasta 1000 saltos atrás en secuencia; con alternancia de vías esto descartaba tramas válidas.

**Solución:**
```cpp
// ⭐ NUEVO: solo rechaza tramas realmente viejas/duplicadas
#define SECUENCIA_VALIDA(seq_nuevo, seq_anterior) \
  (((uint16_t)(seq_nuevo - seq_anterior)) != 0)

// En aplicarComando():
if (!inFailsafe && !SECUENCIA_VALIDA(c.seq, lastSeq)) {
    tramasViejas++;
    return false;
}
```

### 3. **Duty Cycle No Sincronizado**
**Síntoma:** Si el loop se retrasaba, la fase PWM saltaba sin control suave.

**Causa:** Cada rueda usaba `ahora % MOTOR_PERIODO_MS` (timer local); retrasos hacían saltar la fase.

**Solución:**
```cpp
// ⭐ NUEVO: timer global de ciclo motor
static unsigned long cicloMotoraAt = 0;

// En aplicarDuty():
unsigned long faseGlobal = (ahora - cicloMotoraAt) % MOTOR_PERIODO_MS;
```

### 4. **WiFi UDP Frágil** (esp32_lora_v3_v2.ino)
**Síntoma:** UDP perdía datagramas, telemetría inconsistente a la Jetson.

**Soluciones:**
- Separar ritmo de UDP (20 Hz) del ciclo LoRaWAN (30 s)
- Envío periódico de estado (no solo en cambios)
- Mejor logging de pérdidas
- Timeout adaptativo de WiFi

```cpp
// ⭐ NUEVO: UDP independiente del ciclo LoRa
const unsigned long UDP_PERIODO_MS = 50;  // 20 Hz

if (millis() - lastUdpMillis >= UDP_PERIODO_MS) {
    lastUdpMillis = millis();
    readIMU();
    sendUDP();  // ← SIEMPRE, sin atarse a loop()
}
```

---

## 📦 ARCHIVOS NUEVOS

### **ESP32-C3 (Actuador)**
**Archivo:** `/firmware/esp32c3_actuador_v2.ino`

**Cambios principales:**
- ✅ Dead time de 50 ms entre freno e inversión
- ✅ Filtro de secuencia correcto (permite alternancia BLE/WiFi)
- ✅ Timer global para duty cycle sincronizado
- ✅ Mejor logging de cambios de dirección
- ✅ Envío periódico de estado (200 ms)

**Parámetros críticos:**
```cpp
const unsigned long DEAD_TIME_MS = 50;       // tiempo muerto
const unsigned long MOTOR_PERIODO_MS = 200;  // período duty
const unsigned long CMD_TIMEOUT_MS = 500;    // failsafe sin comando
```

### **LoRa V3 (Sensor + WiFi)**
**Archivo:** `/firmware/esp32_lora_v3_v2.ino`

**Cambios principales:**
- ✅ UDP a 20 Hz (independiente de LoRaWAN)
- ✅ Ambiental (SCD40) cada 5 s
- ✅ Mejor manejo de WiFi (check periódico)
- ✅ Logging de pérdidas UDP

**Timing:**
```cpp
const unsigned long UDP_PERIODO_MS = 50;        // 20 Hz
const unsigned long AMBIENTE_PERIODO_MS = 5000; // 5 seg
```

### **Dashboard (index.html)**
**Cambios:**
- ✅ Botón "Modo demo (safeguard ON/OFF)" en Control de Motores
- ✅ Toggle para desactivar timeout de seguridad en pruebas
- ✅ Visual: naranja (safeguard ON), rojo oscuro (OFF)

---

## 🔌 CÓMO FLASHEAR

### **ESP32-C3 (Actuador)**
```bash
# En Arduino IDE:
# 1. Board: ESP32-C3-DevKitC-02 (o "ESP32C3 Dev Module")
# 2. Upload Speed: 921600 baud
# 3. Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
#    ↓ Importante: BLE+WiFi necesita >1.2 MB
# 4. USB CDC On Boot: Enabled
#    ↓ Para que los logs de Serial salgan por USB

# Luego: Sketch → Upload
# O por CLI:
cd ~/Arduino/sketches
arduino-cli upload -p /dev/ttyACM0 -b esp32:esp32:esp32c3dev \
  agrobot_system_2/firmware/esp32c3_actuador_v2.ino
```

### **LoRa V3 (Heltec)**
```bash
# En Arduino IDE:
# 1. Board: Heltec WiFi LoRa 32 V3
# 2. Upload Speed: 921600 baud
# 3. Partition Scheme: Default

# Luego: Sketch → Upload
```

---

## ✅ CHECKLIST ANTES DE PROBAR

- [ ] C3 flasheada con v2, BLE anunciándose
- [ ] LoRa V3 flasheada con v2, WiFi conectada al AP
- [ ] Dashboard cargado (ver botón "Modo demo")
- [ ] Serial del C3 limpio (sin ERROR)
- [ ] Log "ble cliente conectado" en C3
- [ ] Log "WiFi asociada" en LoRa V3

---

## 🧪 PRIMERA PRUEBA

1. **Desde el dashboard:**
   - Pulsa **▲** (adelante) — motor debe arrancar con kick
   - Pulsa **▼** (atrás) — motor debe frenar, esperar 50 ms, invertir
   - Repeat 5 veces — debe ser suave y predecible
   
2. **Si hay problema:**
   - Lee la serie del C3: `[mot] cambios de dirección`
   - Verifica `deadtime <N> ms` en el log
   - Si no aparece, flasheo fallido

3. **Modo demo (pruebas):**
   - Pulsa **🔐 Modo demo** para desactivar timeout
   - Ahora el robot NO se frena en 100 ms sin comando
   - El botón cambia a rojo y dice "safeguard OFF"
   - Para reactivar: pulsa el botón de nuevo

---

## 📋 RESUMEN DE CAMBIOS EN DASHBOARD

**Archivo:** `agrobot_system_2/jetson/dashboard/index.html`

**Línea ~760:** Nuevo botón
```html
<button id="safeguard-btn" onclick="toggleSafeguard()" 
  style="...">🔐 Modo demo (safeguard ON)</button>
```

**Línea ~3334:** Nueva función
```javascript
async function toggleSafeguard() {
  // POST a /api/motor/toggle-safeguard
  // Alterna protección de seguridad
}
```

**Backend esperado:**
```python
@app.post("/api/motor/toggle-safeguard")
async def toggle_safeguard():
    global safeguard_enabled
    safeguard_enabled = not safeguard_enabled
    # Si False, no hay timeout; si True, hay parada a los 500 ms
    return {"safeguard_enabled": safeguard_enabled}
```

---

## 🎯 PRÓXIMOS PASOS

1. Flashear ambos firmwares v2
2. Probar cambios adelante ↔ atrás (5 veces seguidas)
3. Activar/desactivar safeguard desde dashboard
4. Medir latencia UDP (LoRa V3 → Jetson)
5. ✅ Listo para campo

---

**Compilado:** 2026-10-06 | **Estado:** LISTO PARA PROBAR
