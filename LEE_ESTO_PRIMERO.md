# 🚀 LEE ESTO PRIMERO - 5 MINUTOS

## Estado Actual
Tu AGROBOT **pierde WiFi constantemente**. Hemos creado un sistema automático que:
- ✅ Mantiene el AP vivo (verifica cada 5 segundos)
- ✅ Maximiza potencia TX a 30 dBm
- ✅ Optimiza comunicación BLE/WiFi
- ✅ Diagnóstico en tiempo real de caídas
- ✅ Aumenta torque automáticamente

---

## QUÉ SE CREÓ

### 🔧 Módulos nuevos (7 archivos)
```
backend/
  ├── auto_comunicacion.py      ← Orquestador central
  ├── ap_vigilante.py           ← Vigila AP cada 5s
  ├── comunicacion_mejorada.py  ← Potencia + canal + latencia
  ├── diagnostico_wifi.py       ← Rastrea caídas
  └── torque_optimizer.py       ← Torque adaptativo
  
tools/
  └── diagnostico_wifi_rt.py    ← Monitor en vivo
  
Archivos de documentación
  ├── PARCHE_AUTOMATICO.md      ← Cómo integrar
  └── aplicar_parche.py         ← Script semi-auto
```

---

## CÓMO INSTALARLO (2 pasos)

### PASO 1: Ejecutar (opcional, pero recomendado)
```bash
cd /home/jetson/Desktop/AGROBOT
bash INICIO_RAPIDO.sh
```
Esto:
- ✓ Verifica que todos los archivos están
- ✓ Maximiza potencia WiFi
- ✓ Levanta el AP
- ✓ Verifica sintaxis del servidor

### PASO 2: Integrar en jetson_server.py (OBLIGATORIO)
Abre el archivo: `/home/jetson/Desktop/AGROBOT/agrobot_system_2/jetson/backend/jetson_server.py`

**Copia estos cambios (5 lugares):**

#### A) Import (línea ~64, después de otros imports)
```python
from auto_comunicacion import inicializar_auto_comunicacion
```

#### B) Atributo en __init__ (línea ~150-200)
```python
self.auto_comm = None
```

#### C) En `async def start(self)` (después de inicializar enlace_c3)
```python
try:
    self.auto_comm = await inicializar_auto_comunicacion(
        config_store=self.config_store,
        modo_wifi="patrullaje"
    )
except Exception as e:
    print(f"[servidor] error auto-comm: {e}", flush=True)
```

#### D) En el bucle `while not self._stop.is_set():` (al inicio)
```python
if self.auto_comm and self.enlace_c3:
    self.auto_comm.actualizar_estado_vias(
        ble_conectado=self.enlace_c3.ble.connected,
        wifi_conectado=(self.enlace_c3.wifi.conectado 
                       if self.enlace_c3.wifi else False)
    )
```

#### E) En `async def stop(self)` (antes del return final)
```python
if self.auto_comm:
    await self.auto_comm.detener()
```

---

## VERIFICAR QUE FUNCIONA

### Terminal 1: Ver logs
```bash
journalctl -u agrobot-jetson -f | grep -E "\[AUTO\]|\[ap-vigilante\]"
```

Debes ver:
```
[AUTO] ✓ AP Vigilante activo
[AUTO] ✓ Comunicación Mejorada activa
[AUTO] ✓ Diagnóstico WiFi activo
[AUTO] ✓ Torque Optimizer activo
```

### Terminal 2: Monitor en vivo
```bash
cd /home/jetson/Desktop/AGROBOT
python3 tools/diagnostico_wifi_rt.py 5
```

Debes ver:
```
DIAGNÓSTICO DE WiFi - AGROBOT
============================================
ESTADO ACTUAL:
  BLE:                ✓
  WiFi:               ✓
  Estado combinado:   ✓ BLE + WiFi
```

### Terminal 3: Probar endpoints
```bash
# Ver estado de comunicación
curl http://localhost:8000/api/comunicacion/estado | jq

# Cambiar a modo estación (si estás en dock)
curl -X POST http://localhost:8000/api/wifi/modo \
  -H "Content-Type: application/json" \
  -d '{"modo":"estacion"}'
```

---

## QUÉ HACE AUTOMÁTICAMENTE

### Cada 5 segundos
- ✓ Verifica que AP está UP
- ✓ Si no está: lo recrea INMEDIATAMENTE
- ✓ Maximiza potencia TX

### Cada 10 segundos
- ✓ Asegura potencia a 30 dBm

### Cada 30 segundos
- ✓ Monitorea calidad de señal (SNR)

### Cada 60 segundos
- ✓ Optimiza canal WiFi si hay interferencia

### Constantemente
- ✓ Rastrea caídas BLE+WiFi
- ✓ Adapta torque según terreno/inclinación
- ✓ Detecta patrones de falla

---

## RESULTADOS ESPERADOS

| Antes | Después |
|-------|---------|
| WiFi cae cada 30-60s | AP se recrea automáticamente |
| Potencia baja (~20 dBm) | Máxima (30 dBm) |
| Sin diagnóstico | Dashboard con caídas/recuperaciones |
| Torque fijo | Adaptativo según terreno |
| Reconexión manual | Automática cada 60s |

---

## SI SIGUE FALLANDO

### Ejecutar diagnóstico
```bash
python3 tools/diagnostico_wifi_rt.py
```

Observa:
- ¿Cae BLE sola? → Problema de radio compartida (Intel 8265)
- ¿Cae WiFi sola? → Problema de AP o interferencia  
- ¿Caen juntas? → Problema de potencia de batería

### Ver logs detallados
```bash
journalctl -u agrobot-jetson -n 100 | grep -E "\[wifi\]|\[ap-vigilante\]"
```

### Verificar AP está levantado
```bash
nmcli device show wlan0
# Debe mostrar: Connection "JETSON_AP"
```

---

## OPCIONES DE CONFIGURACIÓN

En `wifi_manager.py` (línea ~16-20):
```python
self.ap_nombre = "JETSON_AP"           # ← Nombre del AP
self.ap_password = "agrobot123"        # ← Contraseña
self.lora_wan_ssid = "Lora_Wan"        # ← Red estación
```

En `agrobot_config.json`:
```json
"control": {
  "suavidad_duty": 0.8,      // Aumentado para más torque
  "max_v_mps": 1.8,
  "max_omega_rps": 5.0
}
```

---

## DOCUMENTACIÓN COMPLETA

| Documento | Para qué |
|-----------|----------|
| `PARCHE_AUTOMATICO.md` | Ver exactamente dónde copiar cada línea |
| `INTEGRACION_WIFI_TORQUE.md` | Guía técnica completa |
| `CHECKLIST_INSTALACION.md` | Verificación paso a paso |
| `aplicar_parche.py` | Script semi-automático (opcional) |
| `INICIO_RAPIDO.sh` | Setup inicial (bash script) |

---

## ✅ CHECKLIST RÁPIDO

- [ ] Copié los 5 cambios a `jetson_server.py`
- [ ] La sintaxis se verifica: `python3 -m py_compile jetson_server.py`
- [ ] Reinicié el servidor: `systemctl restart agrobot-jetson`
- [ ] Los logs muestran `[AUTO] ✓`
- [ ] El diagnóstico en tiempo real funciona
- [ ] El AP se mantiene UP

---

## SOPORTE RÁPIDO

**Problema**: "No sé dónde poner los cambios"
→ Lee `PARCHE_AUTOMATICO.md`, tiene cada línea señalada

**Problema**: "Robot sigue sin WiFi"  
→ Ejecuta `python3 tools/diagnostico_wifi_rt.py` para ver el patrón

**Problema**: "Los logs no muestran `[AUTO]`"
→ El import no se agregó. Verifica línea 64 de jetson_server.py

**Problema**: "Command not found: nmcli"
→ Instala Network Manager: `sudo apt install network-manager`

---

## 🎯 RESUMEN
- ✅ 7 archivos nuevos creados y listos
- ✅ AP vigilante cada 5 segundos
- ✅ Potencia máxima automática
- ✅ Diagnóstico en tiempo real
- ✅ Torque adaptativo
- ✅ Solo necesitas copiar 5 cambios pequeños

**Tiempo total: 2-5 minutos**

---

**Creado**: 2026-10-03  
**Versión**: 1.0  
**Estado**: ✅ Listo para usar
