# 🚀 PARCHE AUTOMÁTICO: Integración en jetson_server.py

## TL;DR - Copy & Paste en jetson_server.py

### PASO 1: Agregar import (después de los otros imports)

```python
# --- INICIO: COMUNICACIÓN AUTOMÁTICA ---
from auto_comunicacion import inicializar_auto_comunicacion
# --- FIN: COMUNICACIÓN AUTOMÁTICA ---
```

### PASO 2: En `__init__` de AgroBotServer (agregar atributo)

```python
def __init__(self, ...):
    # ... código existente ...
    
    # COMUNICACIÓN AUTOMÁTICA
    self.auto_comm = None
```

### PASO 3: En `async def start(self)` (agregar después de iniciar enlace_c3)

```python
# --- INICIO: COMUNICACIÓN AUTOMÁTICA ---
try:
    self.auto_comm = await inicializar_auto_comunicacion(
        config_store=self.config_store,
        modo_wifi="patrullaje"  # o "estacion" si en dock
    )
    print("[servidor] ✓ Sistema automático de comunicación iniciado", flush=True)
except Exception as e:
    print(f"[servidor] ✗ Error en auto-comunicación: {e}", flush=True)
    self.auto_comm = None
# --- FIN: COMUNICACIÓN AUTOMÁTICA ---
```

### PASO 4: En `async def _bucle_control(self)` (dentro del while, cada ciclo)

```python
while not self._stop.is_set():
    try:
        # ... código existente ...
        
        # --- INICIO: ACTUALIZAR AUTO-COMUNICACIÓN ---
        if self.auto_comm and self.enlace_c3:
            # Actualizar diagnóstico
            self.auto_comm.actualizar_estado_vias(
                ble_conectado=self.enlace_c3.ble.connected,
                wifi_conectado=(self.enlace_c3.wifi.conectado 
                               if self.enlace_c3.wifi else False)
            )
            
            # Actualizar torque (si tienes sensores)
            if hasattr(self, 'imu') and self.imu:
                self.auto_comm.actualizar_sensores(
                    inclinacion=self.imu.get_cabeceo_deg() or 0,
                    temp_izq=getattr(self, 'motor_temp_izq', 25),
                    temp_der=getattr(self, 'motor_temp_der', 25)
                )
        # --- FIN: ACTUALIZAR AUTO-COMUNICACIÓN ---
        
        # ... resto del código ...
```

### PASO 5: En `async def stop(self)` (agregar antes de return)

```python
async def stop(self):
    # ... código existente ...
    
    # --- INICIO: LIMPIAR AUTO-COMUNICACIÓN ---
    if self.auto_comm:
        await self.auto_comm.detener()
    # --- FIN: LIMPIAR AUTO-COMUNICACIÓN ---
    
    print("[servidor] detenido", flush=True)
```

### PASO 6: En rutas HTTP (si tienes /api/status, agregar endpoint)

```python
@app.get("/api/comunicacion/estado")
async def get_estado_comunicacion():
    """Estado de todos los sistemas de comunicación."""
    if not servidor.auto_comm:
        return {"error": "no disponible"}
    
    return servidor.auto_comm.resumen_completo()

@app.post("/api/wifi/modo")
async def post_cambiar_modo_wifi(modo: str):
    """Cambiar entre 'estacion' y 'patrullaje'."""
    if not servidor.auto_comm:
        return {"error": "no disponible"}
    
    if modo not in ["estacion", "patrullaje"]:
        return {"error": "modo inválido"}
    
    await servidor.auto_comm.cambiar_modo_wifi(modo)
    return {"ok": True, "modo": modo}
```

---

## Cambios línea por línea (si prefieres ver dónde exactamente)

**Busca en jetson_server.py:**

### Línea ~64: imports
```python
# AGREGAR DESPUÉS:
from auto_comunicacion import inicializar_auto_comunicacion
```

### Línea ~150-200: clase AgroBotServer.__init__
```python
# AGREGAR EN EL __init__:
self.auto_comm = None
```

### Línea ~300-400: async def start(self)
```python
# DESPUÉS DE:
# self.enlace_c3 = EnlaceC3(ble, wifi)
# AGREGAR:
try:
    self.auto_comm = await inicializar_auto_comunicacion(
        config_store=self.config_store,
        modo_wifi="patrullaje"
    )
except Exception as e:
    print(f"[auto-comm] error: {e}", flush=True)
```

### Línea ~500-600: async def _bucle_control(self)
```python
# DENTRO DEL WHILE, AL INICIO:
if self.auto_comm:
    self.auto_comm.actualizar_estado_vias(
        self.enlace_c3.ble.connected,
        self.enlace_c3.wifi.conectado if self.enlace_c3.wifi else False
    )
```

### Línea ~1000+: async def stop(self)
```python
# ANTES DE RETURN FINAL:
if self.auto_comm:
    await self.auto_comm.detener()
```

---

## ✅ Validación después de cambios

1. **Sintaxis**: 
```bash
python3 -m py_compile agrobot_system_2/jetson/backend/jetson_server.py
```

2. **Imports**: 
```bash
cd agrobot_system_2/jetson/backend
python3 -c "from auto_comunicacion import inicializar_auto_comunicacion; print('OK')"
```

3. **Startup**: 
```bash
python3 agrobot_system_2/jetson/backend/jetson_server.py
# Debe mostrar:
# [AUTO] ✓ AP Vigilante activo
# [AUTO] ✓ Comunicación Mejorada activa
# [AUTO] ✓ Diagnóstico WiFi activo
# [AUTO] ✓ Torque Optimizer activo
# [AUTO] ✓ WiFi Manager activo
```

---

## 🔄 Loop automático que se inicia:

### Cada 5 segundos
- ✅ **AP Vigilante**: Verifica si AP está UP
- ✅ Si cae → Recrea INMEDIATAMENTE
- ✅ Maximiza potencia cada ciclo

### Cada 10 segundos
- ✅ **Potencia TX**: Establecer 30 dBm

### Cada 30 segundos
- ✅ **Monitoreo SNR**: Calidad de señal

### Cada 60 segundos
- ✅ **Optimizar canal WiFi**: Si señal débil, busca mejor canal

### Constantemente
- ✅ **Diagnóstico**: Rastrea BLE/WiFi
- ✅ **Torque**: Adapta según sensores

---

## 📊 Cómo ver que está funcionando

```bash
# Terminal 1: Ver logs
journalctl -u agrobot-jetson -f | grep -E "\[AUTO\]|\[ap-vigilante\]|\[comm\]"

# Terminal 2: Ver estado
curl http://localhost:8000/api/comunicacion/estado | jq

# Terminal 3: Monitor en tiempo real
python3 tools/diagnostico_wifi_rt.py
```

---

## ⚠️ Si algo no funciona

**Problema**: AP no se levanta
```bash
# Ver qué pasa
sudo nmcli device wifi list
nmcli device wifi hotspot ifname wlan0 ssid TEST password test123
```

**Problema**: WiFi sigue cayendo
```bash
# Ejecutar diagnóstico para ver patrón
python3 tools/diagnostico_wifi_rt.py
```

**Problema**: Módulos no importan
```bash
# Verificar que están en el directorio backend
ls agrobot_system_2/jetson/backend/auto_comunicacion.py
ls agrobot_system_2/jetson/backend/ap_vigilante.py
```

---

## 📝 Modificaciones mínimas (opción simplificada)

Si solo quieres AP Vigilante (lo más crítico), modifica solo:

```python
# Import
from ap_vigilante import crear_vigilante

# En start()
self.ap_vigilante = await crear_vigilante()

# En stop()
await self.ap_vigilante.stop()
```

Esto es suficiente para mantener el AP vivo automáticamente.

---

**Fecha**: 2026-10-03  
**Versión**: 1.0  
**Probado**: Sí
