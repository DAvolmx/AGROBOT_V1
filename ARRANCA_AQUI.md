# 🚀 AGROBOT v3 — ARRANCA AQUI

## ✅ Sistema MONTADO Y LISTO

Todo está integrado en el backend. Tienes:

- ✅ **Firmware v3** (ESP32-C3): Skid-steer + PWM puro
- ✅ **SLAM v2** (Python): Visual con cierre de loops
- ✅ **Zoom automático** (Python): 1x/2x/4x para plagas
- ✅ **Path Detection** (Python): Detección de senderos
- ✅ **Configuración calibrada**: Para tus cultivos

---

## 🎯 Tres Pasos Para Arrancar

### Paso 1: Flashea Firmware en ESP32-C3 (⏱️ 5 min)

```bash
# Abre Arduino IDE
# File → Open → Desktop/AGROBOT/agrobot_system_2/firmware/esp32c3_actuador_v3_skidsteer.ino

# Configura
# Tools → Board: ESP32C3 Dev Module
# Tools → Port: /dev/ttyACM0 (o /dev/ttyUSB0)
# Tools → Speed: 921600

# Sube
# Sketch → Upload (Ctrl+U)

# Verifica en Serial Monitor (Ctrl+Shift+M)
# Deberías ver:
# === AGROBOT C3 v3 — SKID-STEER + PWM PURO ===
# [TEST] R1 adelante...
# [TEST] OK
```

**Status**: ✅ Motor test completo en arranque
- Cada rueda gira 300ms
- Todas juntas 500ms
- Verifica pines y sentidos sin cliente

---

### Paso 2: Arranca Backend (⏱️ 2 min)

```bash
# En terminal Jetson
cd ~/Desktop/AGROBOT/agrobot_system_2/jetson/backend

# Arranca
python3 jetson_server.py

# Deberías ver:
# [jetson_server] iniciando...
# [VisionV3] SLAM v2 inicializado
# [VisionV3] Camera zoom inicializado
# [VisionV3] Path detector inicializado
# [HTTP] 192.168.12.1:8080 escuchando
# [WS] /ws lista para dashboard
```

**Status**: ✅ Backend escuchando en puerto 8080

---

### Paso 3: Abre Dashboard (⏱️ 1 min)

En tu navegador (desde PC/teléfono en la red del AP):

```
http://192.168.12.1:8080
```

Deberías ver:
- 📹 Video en vivo (30 fps)
- 🗺️ Mapa SLAM actualizado
- 📊 Detectiones YOLO (plagas, plantas)
- 🎯 Sendero detectado (línea verde)
- 🔧 Controles: adelante/atrás/giro
- 📈 Métricas: FPS, Latencia, Zoom

---

## 🧪 Pruebas Rápidas

### Test 1: Movimiento (Sin robot atrapado)

```bash
# En dashboard → Manual Control
# Botón "Adelante" → Robot avanza
# Botón "Giro" → Gira SIN cruzarse (delantera/trasera juntas)
# Botón "Atrás" → Retrocede

✅ Resultado esperado:
   • Giro CORRECTO (sin X)
   • Respuesta INMEDIATA (~100ms)
   • Máximo torque en arranque
```

### Test 2: SLAM Visual

```bash
# Con cámara apuntando a cultivo
# Dashboard → Mapa (Live SLAM)

✅ Resultado esperado:
   • Puntos verdes = keyframes
   • Trayectoria del robot visible
   • Mapa persistente tras reinicio
```

### Test 3: Path Detection

```bash
# Robot en el sendero
# Dashboard → Visión → Path Detection

✅ Resultado esperado:
   • Línea verde = sendero detectado
   • Confianza > 70%
   • Ángulo correcto (hacia adelante)
```

### Test 4: Zoom Automático

```bash
# Dashboard → Zoom (Live)

✅ Resultado esperado:
   • Zoom 1x = Sendero completo (60° FOV)
   • Zoom 2x = Plantas (30° FOV) cuando no detecta bien
   • Zoom 4x = Plagas (15° FOV) si baja confianza
```

---

## 🔧 Si Algo Falla

### ❌ "Firmware no ve comandos"
- ✅ Verifica Serial Monitor: ¿aparece el boot del C3?
- ✅ Flashea de nuevo (a veces necesita reset)
- ✅ Chequea cables USB

### ❌ "Backend no arranca"
```bash
# Verifica dependencias
python3 -c "import cv2; print('OK')"
python3 -c "import asyncio; print('OK')"

# Si falta algo:
# python3 -m pip install --break-system-packages opencv-contrib-python
```

### ❌ "Path detection no detecta sendero"
```bash
# Edita config
nano agrobot_config.json

# Busca sección "path_detection"
# Ajusta rangos HSV:
# sendero_h_min/max ±5
# sendero_s_min/max ±10
# sendero_v_min/max ±20

# Ejemplo si es más oscuro:
"sendero_v_min": 20,  # Reduce este
```

### ❌ "YOLO lento en Jetson"
```bash
# Reduce resolución en config
"imgsz": 416,  # Era 480, ahora 416

# O desactiva en sombra
"infer_hz_max": 5,  # Menos inferencias por segundo
```

---

## 📊 Estado del Sistema

```bash
# Ver logs en vivo
cd ~/Desktop/AGROBOT/agrobot_system_2/jetson/backend
tail -f jetson_server.log

# Ver directorios
ls -lah slam_maps/          # Mapas SLAM
ls -lah captures/           # Screenshots de plagas
ls -lah models/             # Modelos YOLO

# Restart
killall python3
python3 jetson_server.py
```

---

## 📋 Checklist de Confirmación

- [ ] Firmware v3 flasheado en C3
- [ ] Serial Monitor: Boot test OK
- [ ] Backend inicia sin errores
- [ ] Dashboard accesible en http://192.168.12.1:8080
- [ ] Video en vivo visible (30 fps)
- [ ] Path detection activo (línea verde)
- [ ] Zoom automático respondiendo
- [ ] Manual control funcionando

---

## 🆘 Soporte Rápido

| Problema | Solución |
|----------|----------|
| Giro en X | Reflashea firmware v3 |
| Lentitud | Reduce imgsz en config |
| Sin sendero | Calibra HSV con tus imágenes |
| SLAM no rastrea | Mejor iluminación o cambio de escena |
| Zoom no cambia | Aumenta auto_zoom_confidence en config |

---

## 🎬 ¿Qué Sigue?

### Hoy
1. Flashea firmware
2. Arranca backend
3. Abre dashboard

### Mañana
1. Pruebas en campo (sendero real)
2. Ajusta HSV si es necesario
3. Verifica giro + path detection

### Próxima Semana
1. Navegación autónoma (Nav2)
2. Mapa persistente (SLAM)
3. Detección de plagas avanzada

---

## 📞 Contacto

Si necesitas ayuda:

- 📧 Revisa logs en terminal
- 🐛 Grep por `[ERROR]` en jetson_server.log
- 🔍 Comprueba config: `cat agrobot_config.json | grep -A5 path_detection`

**¡Listo! Ahora sí: arranca el bot y diviértete.**

🚀 **¡A sembrar automáticamente!**
