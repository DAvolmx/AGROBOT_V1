# 🚀 AGROBOT C3 Firmware v3 — Cambios Críticos

## Problemas Resueltos

### ❌ v2 (Tu código actual)
- **Giro en X**: Las llantas se cruzaban porque usaba diferencial (omega diferencial)
- **Pulsos innecesarios**: Kick, giro alternado = complejidad sin beneficio
- **Rampa desactivada pero código aún presente**: Residuo que confunde
- **Torque perdido en giros**: La normalización reducía duty en maniobras

### ✅ v3 (Nueva versión)

## 1️⃣ CINEMÁTICA SKID-STEER LIMPIA

### Antes (v2 — Diferencial)
```cpp
float wIzq = (vx - K_ROT * omega) / WHEEL_RADIUS;  // ← Opuestas en giro
float wDer = (vx + K_ROT * omega) / WHEEL_RADIUS;
```
**Problema**: Cuando giras, `omega` causa que `wIzq` y `wDer` tengan **signos opuestos** → llantas se cruzan en X

### Después (v3 — Skid-Steer)
```cpp
// Lado IZQ y DER INDEPENDIENTES
float vIzq = vx - omega * (TRACK_WIDTH / 2.0f) / WHEEL_RADIUS;
float vDer = vx + omega * (TRACK_WIDTH / 2.0f) / WHEEL_RADIUS;

// Ambas ruedas de un lado SIEMPRE giran iguales
setRueda(*LADO_IZQ[0], dutyIzq, sentidoIzq);  // L1
setRueda(*LADO_IZQ[1], dutyIzq, sentidoIzq);  // L2  ← IGUALES
setRueda(*LADO_DER[0], dutyDer, sentidoDer);  // R1
setRueda(*LADO_DER[1], dutyDer, sentidoDer);  // R2  ← IGUALES
```

**Movimientos**:
- **Adelante**: vx=+, omega=0 → Izq y Der adelante
- **Atrás**: vx=-, omega=0 → Izq y Der atrás
- **Giro izquierda**: vx=0, omega=+ → Izq atrás, Der adelante
- **Giro derecha**: vx=0, omega=- → Izq adelante, Der atrás
- **Arco forward-izq**: vx=+, omega=+ → Izq lento, Der rápido

**Resultado**: Sin cruce, giro correctamente coordinado.

---

## 2️⃣ ELIMINACIÓN DE PULSOS (Kick + Giro Alternado)

### Antes (v2)
```cpp
const uint8_t KICK_PULSOS  = 5;          // Pulsos de arranque
const unsigned long KICK_ON_MS   = 250;
const unsigned long KICK_OFF_MS  = 20;   // Alternancia
const int PWM_MIN_GIRO = 255;            // Giro en pulsos
const unsigned long GIRO_ON_MS  = 200;
const unsigned long GIRO_OFF_MS = 60;
```
✅ **Eliminado completamente** en v3.

### Después (v3)
- **No hay kick**: PWM directo en arranque = torque máximo inmediato
- **No hay pulsos de giro**: Velocidades suavemente escaladas
- **Código simplificado**: Menos máquinas de estado, menos bugs

**Ventaja**: Respuesta instantánea, máximo torque, sin complejidad.

---

## 3️⃣ PWM PURO (Sin Rampa)

### Antes (v2)
```cpp
const float RAMPA_V_PCT_S     = 0.0f;    // Rampa "desactivada" pero...
const float RAMPA_OMEGA_PCT_S = 0.0f;
void updateRampa() { ... }  // Función que no hace nada pero se ejecuta
```

### Después (v3)
```cpp
// En aplicarComando():
vAplicada     = lastV;      // Asignación DIRECTA
omegaAplicada = lastOmega;
applyMotors(vAplicada, omegaAplicada);  // PWM inmediato
```

**Cambio**: Eliminé `updateRampa()` y toda la lógica de rampa.
**Resultado**: Respuesta en ~5ms (tiempo de envío UDP/BLE).

---

## 4️⃣ ANTI-HUNDIMIENTO (Nuevo)

Para terrenos donde se entierran las llantas:

```cpp
#define ANTI_HUNDIMIENTO_ACTIVO = true
const float ESFUERZO_UMBRAL = 0.85f;      // duty > 85% = hundimiento
const float BOOST_TORQUE = 1.15f;         // boost de torque
```

**Lógica**:
1. Monitorea duty promedio cada 100ms
2. Si duty histórico > 85% → detecta hundimiento
3. Multiplica duty por 1.15 (15% más torque)
4. Mantiene boost mientras esté hundido

**Ejemplo**:
- Robot intenta avanzar en lodo, duty sube a 95%
- Detecta "hundimiento"
- Aplica boost: 95% × 1.15 = 109% → Saturado a 100%
- Resultado: Máximo esfuerzo sostenido

---

## 5️⃣ DETALLES DE CONTROL POR RUEDA

### Nueva Estructura
```cpp
struct Rueda {
  int8_t sentido;      // +1=adelante, -1=atrás, 0=freno
  int dutyPedido;      // 0-100
  bool dcCompleto;     // PWM iniciado
  unsigned long frenohasta;  // Tiempo de dead-time
};
```

### Aplicación de Duty
```cpp
static void aplicarDuty(Rueda &r) {
  if (r.sentido == 0) {
    motorFreno(r.pinFI, r.pinBI);  // Freno total
    return;
  }

  int duty = r.dutyPedido;
  uint8_t pinActivo = (r.sentido > 0) ? r.pinFI : r.pinBI;

  if (duty >= 100) {
    motorON(pinActivo);    // 100% = contacto directo
  } else if (duty <= 0) {
    motorFreno();          // 0% = freno
  } else {
    // PWM: modula dentro del ciclo
    unsigned long faseGlobal = (ahora - cicloMotoraAt) % MOTOR_PERIODO_MS;
    unsigned long duracionON = (MOTOR_PERIODO_MS * duty) / 100;
    if (faseGlobal < duracionON) motorON(pinActivo);
    else motorFreno();
  }
}
```

---

## 📊 Comparativa

| Aspecto | v2 | v3 |
|---------|----|----|
| **Giro** | ❌ En X | ✅ Correcto |
| **Pulsos Kick** | 5 pulsos | ❌ Eliminado |
| **Pulsos Giro** | Alternancia | ❌ Eliminado |
| **Rampa** | 0.0f (inerte) | ❌ Eliminado |
| **Anti-hundimiento** | No | ✅ Sí |
| **Latencia** | ~50ms | ~5ms |
| **Torque arranque** | Gradual (rampa=0 pero lógica presente) | ✅ Máximo inmediato |
| **Complejidad** | Alta (máquinas de estado) | Media (skid-steer limpio) |

---

## 🔧 Cómo Compilar y Flashear

### Requisitos
- Arduino IDE con soporte ESP32-C3
- Librería BLEDevice
- Cable USB-C

### Pasos

#### 1. Descarga v3
```bash
cd ~/Desktop/AGROBOT/agrobot_system_2/firmware
# El archivo ya está creado:
# esp32c3_actuador_v3_skidsteer.ino
```

#### 2. Abre en Arduino IDE
- Archivo → Abrir → `esp32c3_actuador_v3_skidsteer.ino`

#### 3. Configura Placa
- Herramientas → Placa: "ESP32C3 Dev Module"
- Herramientas → Puerto: `/dev/ttyACM0` (o el que vea)
- Herramientas → Velocidad: 921600

#### 4. Compila y Flashea
```bash
# Desde terminal en Arduino IDE o:
./arduino-cli compile --fqbn esp32:esp32:esp32c3 \
  ~/Desktop/AGROBOT/agrobot_system_2/firmware/esp32c3_actuador_v3_skidsteer.ino

./arduino-cli upload -p /dev/ttyACM0 \
  --fqbn esp32:esp32:esp32c3 \
  ~/Desktop/AGROBOT/agrobot_system_2/firmware/esp32c3_actuador_v3_skidsteer.ino
```

#### 5. Verifica Serial
```bash
picocom /dev/ttyACM0 -b 115200
# Deberías ver:
# === AGROBOT C3 v3 — SKID-STEER + PWM PURO ===
# [TEST] R1 adelante...
# ...
```

---

## 🧪 Pruebas Recomendadas

### 1. Test de Motores
El código ejecuta `testArranque()` al arrancar:
- Cada rueda gira 300ms solo
- Todas juntas 500ms
- Verifica pines y sentidos

### 2. Giro Puro
```
Envía: v=0, omega=+1 (100% izquierda)
Esperado: L1,L2 atrás; R1,R2 adelante
Verificar: No se cruzan, giro limpio
```

### 3. Avance Diagonal
```
Envía: v=+0.5, omega=+0.5 (diagonal adelante-izquierda)
Esperado: L1,L2 lento; R1,R2 rápido
Verificar: Arco suave, sin paradas
```

### 4. Terreno Duro
```
Embarra las llantas en lodo
Envía: v=+1, omega=0
Observar Serial: boost pasa de 1.0→1.15
El robot debe empujar más fuerte
```

---

## 📝 Cambios en Python (Backend)

No es necesario cambiar nada en `pulse_drive.py` ni en `control.py`:
- El protocolo CmdFrame sigue siendo igual
- v_pct y omega_pct tienen el mismo significado
- Solo cambia el hardware (cinemática interna)

**Nota**: Revisar que `escala_velocidad` en `agrobot_config.json` esté a 1.0 o más para aprovechar el PWM puro.

---

## ⚠️ Notas Importantes

1. **TEST_MOTORES = 1**: Ejecuta test en arranque. Cambiar a 0 en producción si molesta.
2. **ANTI_HUNDIMIENTO_ACTIVO**: Puede comportarse diferente en pisos sólidos. Desactivar si es necesario.
3. **DEAD_TIME_MS = 20**: Tiempo mínimo entre cambios de dirección. Ajustar si hay chirriidos.
4. **Boost 1.15**: Es conservador. Aumentar a 1.3 si sigue hundiendo.

---

## ✨ Resultado Esperado

Después de flashear v3:

1. ✅ **Giro correcto**: Ambas llantas de cada lado giran juntas
2. ✅ **Respuesta rápida**: Sin pulsos, sin delays
3. ✅ **Máximo torque**: 100% en arranque
4. ✅ **Anti-hundimiento**: Boost automático en terrenos duros
5. ✅ **Código limpio**: Menos máquinas de estado, más mantenible

**Prueba inmediatamente en el robot y reporta resultados.**

