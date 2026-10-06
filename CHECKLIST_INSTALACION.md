# Checklist de Instalación: WiFi + Torque + Diagnóstico

## ✓ Archivos creados

- [x] `wifi_manager.py` - Gestor de WiFi con máxima potencia
- [x] `diagnostico_wifi.py` - Diagnóstico de caídas
- [x] `torque_optimizer.py` - Optimización de torque
- [x] `diagnostico_wifi_rt.py` - Monitor en tiempo real
- [x] `ejemplo_integracion.py` - Template de integración
- [x] Documentación de integración

## 📋 Paso 1: Verificar dependencias

```bash
# Todas están en requierments.txt ya:
pip list | grep -E "(bleak|asyncio)"
```

Expected:
- `bleak` (BLE)
- `asyncio` (incluido en Python 3.7+)
- `nmcli` (instalado en sistema, o instalar: sudo apt install network-manager)

## 📋 Paso 2: Verificar WiFi físicamente

```bash
# Ver interfaz wlan0
ip link show wlan0

# Debe salir: 
# 2: wlan0: <BROADCAST,MULTICAST,UP,LOWER_UP> ...
```

Si no está UP:
```bash
sudo ip link set wlan0 up
```

## 📋 Paso 3: Maximizar potencia TX

```bash
# Ver región reguladora
iw reg get

# Ver potencia actual
iw wlan0 get txpower

# Establecer máxima (30 dBm)
sudo iw wlan0 set txpower fixed 3000

# Verificar
iw wlan0 get txpower
# Debe mostrar: TX power: 3000 (30 dBm)
```

## 📋 Paso 4: Buscar "Lora_Wan"

```bash
# Escanear redes WiFi disponibles
sudo nmcli device wifi rescan
sleep 3
nmcli device wifi list

# Buscar "Lora_Wan" en la lista
nmcli device wifi list | grep "Lora_Wan"
```

Si no aparece:
- [ ] Verificar que el router DIR-615 esté prendido
- [ ] Verificar que transmita WiFi (check LED del router)
- [ ] Intentar en otra ubicación más cerca del router

## 📋 Paso 5: Intentar conectar a Lora_Wan (manual)

```bash
# Conectar
sudo nmcli device wifi connect "Lora_Wan" password "contraseña_si_tiene"

# Verificar conexión
nmcli device show wlan0
# Debe mostrar: IP4.ADDRESS: 192.168.0.x/24
```

Si funciona:
- [ ] Anotar la IP que recibió (ej: 192.168.0.50)
- [ ] Probar ping: `ping 192.168.0.1` (puerta de enlace del router)

Si no funciona:
- [ ] Revisar contraseña en DIR-615 (por defecto: admin/admin)
- [ ] Verificar canal del router (debe ser fijo, anotar cuál es)
- [ ] Intentar conectar desde otra máquina para verificar que el router funciona

## 📋 Paso 6: Levantar AP "JETSON_AP" (manual)

```bash
# Apagar conexión anterior si está
sudo nmcli device disconnect wlan0

# Levantar AP
sudo nmcli device wifi hotspot ifname wlan0 ssid "JETSON_AP" password "agrobot123"

# Verificar
nmcli device show wlan0
# Debe mostrar: WIFI-HW: yes   WIFI: yes    IP4: 192.168.12.x/24
```

Desde otro dispositivo (teléfono, laptop):
- [ ] Buscar WiFi "JETSON_AP"
- [ ] Conectar con contraseña "agrobot123"
- [ ] Verificar que accedes a internet (si redirecciona)

## 📋 Paso 7: Integrar en jetson_server.py

Abrir `agrobot_system_2/jetson/backend/jetson_server.py`:

1. **Importar módulos nuevos** (después de los imports existentes):
```python
from wifi_manager import abrir as abrir_wifi
from diagnostico_wifi import crear_diagnostico
from torque_optimizer import crear_optimizer
```

2. **En `__init__`** (agregar estos atributos):
```python
self.wifi_mgr = None
self.diag_wifi = None
self.torque_opt = None
```

3. **En `async def start()`** (después de inicializar enlace_c3):
```python
# WiFi + Diagnóstico + Torque
try:
    self.wifi_mgr = await abrir_wifi(modo_inicial="patrullaje")
    self.diag_wifi = await crear_diagnostico()
    self.torque_opt = await crear_optimizer(self.config_store)
    print("[servidor] WiFi + Diagnóstico + Torque listos", flush=True)
except Exception as e:
    print(f"[servidor] error inicializando sistemas: {e}", flush=True)
```

4. **En el bucle de control** (cada ciclo):
```python
# Actualizar diagnóstico (después de procesar comandos)
if self.diag_wifi and self.enlace_c3:
    self.diag_wifi.actualizar_estado(
        ble=self.enlace_c3.ble.connected,
        wifi=(self.enlace_c3.wifi.conectado 
              if self.enlace_c3.wifi else False)
    )
```

5. **En `async def stop()`** (antes de retornar):
```python
if self.wifi_mgr:
    await self.wifi_mgr.stop()
```

## 📋 Paso 8: Ejecutar diagnóstico en tiempo real

```bash
cd /home/jetson/Desktop/AGROBOT
chmod +x tools/diagnostico_wifi_rt.py
python3 tools/diagnostico_wifi_rt.py 5
```

Verás un dashboard que se actualiza cada 5s mostrando:
- Estado actual BLE/WiFi
- Estadísticas de caídas
- Patrones detectados
- Salud general

Presiona **Ctrl+C** para detener.

## 📋 Paso 9: Probar cambio de modos

Con el servidor ejecutándose:

```bash
# Ver estado actual
curl http://localhost:8000/api/estado | jq '.wifi'

# Cambiar a modo estación (si Lora_Wan está disponible)
curl -X POST http://localhost:8000/api/wifi/cambiar-modo -H "Content-Type: application/json" -d '{"modo":"estacion"}'

# Ver cambio
curl http://localhost:8000/api/estado | jq '.wifi'

# Volver a patrullaje
curl -X POST http://localhost:8000/api/wifi/cambiar-modo -d '{"modo":"patrullaje"}'
```

## 📋 Paso 10: Calibrar torque

1. **Revisar valores base** en `agrobot_config.json`:
```json
"control": {
  "suavidad_duty": 0.6,
  "max_v_mps": 1.5,
  "max_omega_rps": 4.0
}
```

2. **Probar en campo**:
   - [ ] Robot en terreno plano: ¿sube inclinaciones? ¿tiene torque?
   - [ ] Robot en pendiente: ¿mantiene control?
   - [ ] Robot con carga: ¿se mueve?

3. **Ajustar si es necesario**:
   - Si falta torque: aumentar `suavidad_duty` (0.6 → 0.7 → 0.8)
   - Si se acelera demasiado: aumentar `suavidad_periodo_s` (0.3 → 0.5)
   - Si gira lento: aumentar `max_omega_rps` (4.0 → 5.0)

4. **Ver ajustes en tiempo real**:
```bash
curl http://localhost:8000/api/torque/ajustes | jq
```

## 📋 Paso 11: Monitoreo de caídas WiFi

**En caso de que se caigan las conexiones**:

1. Ejecutar diagnóstico en tiempo real:
```bash
python3 tools/diagnostico_wifi_rt.py
```

2. Revisar patrones:
   - ¿Caída simultánea BLE+WiFi? → Problema de radio compartida (Intel 8265)
   - ¿Solo WiFi? → Problema de AP o interferencia
   - ¿Solo BLE? → Revisión de enlace BLE

3. Ver logs del servidor:
```bash
# Si está en systemd
journalctl -u agrobot-jetson -f | grep -E "\[wifi\]|\[c3\]|\[diag"
```

## 📋 Verificación final

- [ ] WiFi a máxima potencia TX (30 dBm)
- [ ] AP "JETSON_AP" levanta correctamente
- [ ] Se conecta a "Lora_Wan" cuando modo = estación
- [ ] Diagnóstico rastrea caídas correctamente
- [ ] Torque aumenta en terreno plano
- [ ] Servidor no crashea con nuevos módulos
- [ ] Dashboard muestra estado WiFi/Diagnóstico/Torque
- [ ] Reconexión automática del AP cada 60s (si se cae)

## 🆘 Troubleshooting

### "WiFi se cae cada 30 segundos"
- Ejecutar diagnóstico RT para ver patrón
- Si simultánea BLE+WiFi: revisar compartición de radio (Intel 8265)
- Si solo WiFi: revisar interferencia, cambiar canal del AP

### "Lora_Wan no aparece"
- Verificar DIR-615 está prendido
- Verificar tiene WiFi habilitado
- Verificar SSID es exactamente "Lora_Wan" (mayúsculas/minúsculas)
- Probar escaneo: `sudo nmcli device wifi rescan && sleep 3 && nmcli device wifi list`

### "AP no se levanta"
- Verificar wlan0 está disponible: `iw dev`
- Probar manualmente: `sudo nmcli device wifi hotspot ifname wlan0 ssid TEST password test123`
- Si falla, revisar si Network Manager está corriendo: `systemctl status NetworkManager`

### "Torque no aumenta"
- Verificar config_store está siendo actualizado
- Ver logs: `cat agrobot_system_2/jetson/backend/captures/...` (si los hay)
- Revisar que inclinación se detecta correctamente

## Notas importantes

- **Potencia TX máxima**: Se establece en boot del wifi_manager. Si se reinicia wifi, puede volver al default.
- **Reconexión AP**: Ocurre cada 60s solo en modo patrullaje. En estación no hay reintentos.
- **Diagnóstico**: No afecta rendimiento (overhead <1%). Guardar logs si hay incidentes.
- **Torque**: Los ajustes pueden cambiar durante operación (no es estático).

---

**Fecha creación**: 2026-10-03  
**Versión**: 1.0  
**Autor**: Claude Haiku 4.5 + AGROBOT Team
