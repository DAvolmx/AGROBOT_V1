# Integración: WiFi Máxima Potencia + Reconexión + Diagnóstico + Torque

## Resumen de cambios

Se han creado 4 nuevos módulos para mejorar:
1. **WiFi a máxima potencia TX (30 dBm)**
2. **Reconexión automática a "Lora_Wan" en modo Estación**
3. **AP redundante que reinicia cada 60s si se cae**
4. **Diagnóstico en tiempo real de caídas**
5. **Optimización dinámica de torque según terreno**

## Archivos nuevos

```
agrobot_system_2/jetson/backend/
  ├── wifi_manager.py          # Gestor de WiFi + modos
  ├── diagnostico_wifi.py      # Rastreador de caídas y patrones
  └── torque_optimizer.py      # Optimización adaptativa de torque

tools/
  └── diagnostico_wifi_rt.py   # Monitor en tiempo real (ejecutable)
```

## 1. WiFi Manager (wifi_manager.py)

### Características

- **Máxima potencia TX**: Automáticamente establece 30 dBm
- **Dos modos de operación**:
  - `"estacion"`: Conecta a red "Lora_Wan" (estación base)
  - `"patrullaje"`: Levanta AP JETSON_AP (autónomo en campo)
- **Reconexión automática**: Si cae el AP, intenta recrearlo cada 60s
- **Monitoreo de salud**: Detecta caídas en tiempo real

### Uso en jetson_server.py

```python
# En __init__ o setup:
from wifi_manager import abrir as abrir_wifi

# Crear gestor (inicia en modo patrullaje por defecto)
self.wifi_mgr = await abrir_wifi(modo_inicial="patrullaje")

# Cambiar modo cuando se detecte dock/estación
if en_estacion_base:
    await self.wifi_mgr.cambiar_modo("estacion")
else:
    await self.wifi_mgr.cambiar_modo("patrullaje")

# En el resumen de estado:
status["wifi"] = self.wifi_mgr.resumen()
```

### Configuración de Lora_Wan

Edita en `wifi_manager.py`:
```python
self.lora_wan_ssid = "Lora_Wan"           # Nombre de la red
self.lora_wan_pass = ""                   # Contraseña (vacía si sin contraseña)
```

## 2. Diagnóstico de WiFi (diagnostico_wifi.py)

### Características

- **Rastrea caídas simultáneas** (BLE + WiFi juntas)
- **Detecta patrones** (caídas aisladas vs sistémicas)
- **Mide recuperación**: tiempo promedio para volver online
- **Evaluación de salud**: buena/degradada/pobre/crítica

### Uso en jetson_server.py

```python
from diagnostico_wifi import crear_diagnostico

# Crear diagnóstico
self.diag_wifi = await crear_diagnostico()

# Actualizar cada vez que cambia estado de vías
self.diag_wifi.actualizar_estado(
    ble=self.enlace_c3.ble.connected,
    wifi=self.enlace_c3.wifi.conectado if self.enlace_c3.wifi else False
)

# Análisis cuando sea necesario
analisis = await self.diag_wifi.analizar_caidas()
print(f"Salud WiFi: {analisis['salud']}")

# En resumen:
status["diagnostico_wifi"] = self.diag_wifi.resumen()
```

## 3. Optimizador de Torque (torque_optimizer.py)

### Características

- **Adapta duty cycle** según terreno (0.6 → 0.95 max)
- **Ajusta velocidad máxima** según inclinación
- **Monitoreo de temperatura** de motores
- **Throttle automático** si se calienta (>60°C)

### Uso en jetson_server.py

```python
from torque_optimizer import crear_optimizer

# Crear optimizador (necesita config_store)
self.torque_opt = await crear_optimizer(self.config_store)

# Actualizar sensores cada ciclo (si tienes cabeceo/IMU)
self.torque_opt.actualizar_sensores(
    inclinacion=self.imu_cabeceo,
    temp_izq=self.motor_temp_izq,
    temp_der=self.motor_temp_der
)

# En resumen:
status["torque"] = self.torque_opt.resumen()
```

## 4. Monitoreo en tiempo real (diagnostico_wifi_rt.py)

Ejecutable para verificar caídas en vivo:

```bash
cd /home/jetson/Desktop/AGROBOT
python3 tools/diagnostico_wifi_rt.py 5    # Actualizar cada 5s
```

Muestra:
- Estado BLE/WiFi actual
- Estadísticas de caídas
- Patrones detectados
- Salud general del enlace

## Diagrama de integración

```
jetson_server.py
    ├── wifi_manager
    │   ├── Levanta AP o conecta a Lora_Wan
    │   ├── Maximiza potencia TX
    │   └── Reintentos cada 60s
    │
    ├── enlace_c3 (ya existe)
    │   ├── ble_link
    │   └── udp_actuator_link (WiFi)
    │
    ├── diagnostico_wifi
    │   └── Rastrea caídas y patrones
    │
    └── torque_optimizer
        └── Ajusta dinamámicamente duty/velocidad
```

## Cambios en config/agrobot_config.json

Opcionalmente, puedes preajustar:

```json
{
  "control": {
    "suavidad_duty": 0.8,      // Aumentado de 0.6 para más torque
    "suavidad_periodo_s": 0.2,
    "max_v_mps": 1.8,          // Aumentado de 1.5 (opcional)
    "max_omega_rps": 5.0       // Aumentado de 4.0 (opcional)
  }
}
```

**Nota**: El torque_optimizer puede sobrescribir estos valores según condiciones.

## Verificación post-instalación

1. **Máxima potencia TX**:
   ```bash
   iw wlan0 get txpower
   # Debe mostrar: TX power: 3000 (30 dBm)
   ```

2. **AP levantado**:
   ```bash
   nmcli device show wlan0
   # Debe mostrar JETSON_AP activo
   ```

3. **Conexión a Lora_Wan**:
   ```bash
   nmcli device wifi list | grep Lora_Wan
   # Debe encontrarse
   ```

4. **Diagnóstico ejecutándose**:
   ```bash
   python3 tools/diagnostico_wifi_rt.py
   # Debe mostrar dashboard con estadísticas
   ```

## Solución de problemas

### AP no se levanta
- Verificar `nmcli device` muestre wlan0 como wifi
- Probar: `nmcli device wifi hotspot ifname wlan0 ssid TEST password test123`

### No encuentra Lora_Wan
- Verificar nombre exacto: `iw dev wlan0 scan | grep "SSID:"`
- Ajustar `lora_wan_ssid` en wifi_manager.py

### WiFi se cae constantemente
- Ejecutar `diagnostico_wifi_rt.py` para analizar patrones
- Si simultánea con BLE: problema de radio compartida (Intel 8265)
- Si solo WiFi: revisión de AP, canal, o interferencia

### Torque no aumenta
- Verificar `config_store` está siendo usado correctamente
- Revisar valores en `agrobot_config.json` sección `control`
- Monitor: `diagnostico_wifi_rt.py` muestra ajustes realizados

## Próximos pasos

1. Integrar en `jetson_server.py`: llamar `abrir_wifi()` y `crear_optimizer()`
2. Actualizar estado del diagnóstico cada ciclo de control
3. Definir sensor de inclinación/temperatura de motores (si no existen)
4. Calibrar parámetros de torque según comportamiento real del robot

## Licencia y autoría

Módulos creados por Claude Haiku 4.5, integración con AGROBOT 2026-10-03.
