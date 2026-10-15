# 🎮 Simulador v3 — Con Análisis Multispectral

## ✨ Lo Que Tienes

Un **simulador 3D profesional** que muestra:

- ✅ **Cultivo 3D**: Lechuga, Acelga plantadas en filas
- ✅ **Robot agrícola**: Navegando con cinemática skid-steer
- ✅ **LIDAR en vivo**: Grid rojo mostrando obstáculos
- ✅ **SLAM visual**: Trayectoria del robot + keyframes verdes
- ✅ **Análisis multispectral**:
  - NDVI (Índice de Vegetación)
  - Chlorophyll Index
  - Detección de plagas
- ✅ **Detección de senderos**: Indica cuando está en el camino
- ✅ **Mapa miniatura**: En esquina para orientación

---

## 🚀 Cómo Ejecutar

### Instalación (Una sola vez)

```bash
# Instala pygame (si no lo tienes)
pip3 install --break-system-packages pygame numpy

# O si no tienes permisos de sistema
python3 -m pip install --user pygame numpy
```

### Ejecutar el simulador

```bash
cd ~/Desktop/AGROBOT

python3 simulador_mejorado_v3.py
```

**Resultado esperado:**
- Ventana 1400x900 píxeles
- Cultivo de lechuga/acelga
- Robot amarillo/rojo navegando
- Overlays de datos en tiempo real

---

## 🎮 Controles

| Tecla | Acción |
|-------|--------|
| **↑** | Acelera adelante |
| **↓** | Frena / Atrás |
| **←** | Gira izquierda |
| **→** | Gira derecha |
| **R** | Reinicia cultivo |
| **ESC** | Cierra simulador |

---

## 📊 Panels de Información

### Panel Superior Derecho (Verde)
**MULTISPECTRAL SCAN**
- `NDVI`: Índice de Vegetación (0-1)
  - >0.7 = Vegetación sana
  - <0.5 = Estrés o plagas
- `Chlorophyll`: Contenido de clorofila (mg/cm²)
  - >50 = Sano
  - <30 = Deficiencia
- `Pests`: Cantidad de plantas con plagas
- `FPS`: Frames por segundo

### Panel Inferior (Blanco)
**ROBOT STATUS**
- `Position`: Coordenadas X, Y (metros)
- `Velocity`: Velocidad lineal y angular
- `Sendero`: Detectado (verde) o buscando (rojo)
- `LIDAR Distance`: Distancia al obstáculo más cercano (metros)

### Panel Superior Izquierdo (Minimap)
- Pequeño mapa del campo
- Plantas (puntos verdes)
- Robot (punto amarillo)

---

## 🌾 Elementos de la Simulación

### Plantas
- **Lechuga** (verde oscuro): Tipo principal
- **Acelga** (verde claro): Tipo secundario
- **Con plagas** (borde naranja): Mosca blanca, mildiu

**Indicadores de salud:**
- Verde brillante = Sano
- Pálido/amarillo = Enfermo
- Borde naranja = Con plagas detectadas

### Robot
- **Rojo**: Buscando sendero (no detectado)
- **Verde**: En el sendero (detectado correctamente)
- **Línea amarilla**: Dirección de movimiento
- **Grid rojo**: LIDAR barriendo la zona

### SLAM
- **Puntos verdes**: Keyframes del mapa
- **Línea verde**: Trayectoria recorrida
- **Línea amarilla en robot**: Rumbo actual

---

## 🔬 Cálculos Multiespectrales

### NDVI (Normalized Difference Vegetation Index)
```
NDVI = (NIR - Red) / (NIR + Red)
Rango: -1 a 1

En la simulación:
- Base: 0.6 + (salud × 0.3)
- Reducción por plagas: -0.2
- Resultado: 0.3 a 0.9
```

### Índice de Clorofila
```
Rango: 10 a 80 mg/cm²

En la simulación:
- Base: 50 + (salud × 30)
- Reducción por plagas: -20
- Resultado: 10 a 80
```

### Detección de Plagas
- NDVI < 0.5 → Mildiu
- Chlorophyll < 30 → Deficiencia Nutricional
- Pest Level > 0.5 → Mosca Blanca

---

## 🎯 Casos de Prueba

### Test 1: Navegación Limpia
```
1. Arranca simulador
2. Mueve robot ↑ + ↔ suavemente
3. Resultado esperado:
   - Sendero = DETECTADO (verde)
   - LIDAR detecta plantas
   - SLAM crea trayectoria
```

### Test 2: Detección de Plagas
```
1. Observa plantas con borde naranja
2. Lee NDVI y Chlorophyll
3. Resultado esperado:
   - NDVI < 0.5 (problema)
   - Chlorophyll bajo
   - Plagas > 0
```

### Test 3: Mapa SLAM
```
1. Navega en círculo
2. Observa trayectoria verde
3. Resultado esperado:
   - Puntos verdes (keyframes)
   - Línea de trayectoria cerrada
   - Mapa consistente
```

---

## ⚙️ Parámetros Configurables

Si quieres ajustar el simulador, edita `simulador_mejorado_v3.py`:

```python
# Línea ~120 en __init__
self.field_width = 50      # Ancho del campo (metros)
self.field_height = 40     # Alto del campo (metros)
self.pixels_per_meter = 15 # Zoom

# Línea ~180 en _init_plants
# Densidad de plantas
row_spacing = 0.6  # Distancia entre filas
col_spacing = 0.5  # Distancia entre columnas

# Línea ~195
# Probabilidad de plagas
p=[0.6, 0.2, 0.1, 0.05, 0.05]  # 60% sano, 5% con plagas
```

---

## 🐛 Troubleshooting

### ❌ "ModuleNotFoundError: No module named 'pygame'"
```bash
pip3 install --break-system-packages pygame
```

### ❌ Simulador lento (< 30 FPS)
- Reduce plantas en `_init_plants()`: change `range(15)` to `range(10)`
- O reduce resolución: `self.pixels_per_meter = 10`

### ❌ Robot no se mueve
- Verifica teclas: ↑ para adelante
- Chequea que pygame tenga foco

### ❌ Plantas no tienen plagas
- Es aleatorio (seed=42). Ejecuta varias veces
- O aumenta probabilidad en línea ~195: `p=[0.2, 0.3, 0.2, 0.15, 0.15]`

---

## 📈 Interpretación de Datos

### NDVI Alta (>0.6)
✅ Vegetación sana y vigorosa
- Fotosíntesis activa
- Sin estrés hídrico
- Sin plagas

### NDVI Baja (<0.5)
⚠️ Problemas detectados
- Enfermedad (mildiu)
- Deficiencia nutricional
- Plagas activas
- Estrés hídrico

### Chlorophyll Bajo (<30)
⚠️ Necesita intervención
- Riego aumentado
- Fertilización
- Control de plagas

---

## 🎬 Próximas Mejoras

### Posibles extensiones:
1. **Export a Unity**: Usar datos de simulación en motor gráfico
2. **Integrar con backend real**: Datos del bot real en simulador
3. **Multispectral 3D**: Visualizar bandas NIR/Red/Green separadas
4. **Predictor de cosecha**: Basado en NDVI histórico

---

## 📊 Salida Esperada

```
Simulador AGROBOT v3 — Ventana 1400x900

┌─────────────────────────────────────────┐
│  Campo de 50m × 40m con plantas          │
│  Robot amarillo/rojo navegando           │
│  Grid rojo (LIDAR) en tiempo real        │
│  Trayectoria verde (SLAM)                │
│                                         │
│  ├─ MULTISPECTRAL SCAN                   │
│  │  ├─ NDVI: 0.68                        │
│  │  ├─ Chlorophyll: 52                   │
│  │  ├─ Pests: 3                          │
│  │  └─ FPS: 60                           │
│  │                                       │
│  └─ ROBOT STATUS                         │
│     ├─ Position: (25.3, 20.1)            │
│     ├─ Velocity: 0.45 m/s                │
│     ├─ Sendero: DETECTADO ✅             │
│     └─ LIDAR: 1.2 m                      │
└─────────────────────────────────────────┘
```

---

## 🚀 Integración con Bot Real

Para conectar el simulador con el bot real:

```python
# En jetson_server.py, añade:
from simulador_mejorado_v3 import SimuladorV3

# Crea simulador paralelo
sim = SimuladorV3()

# En cada frame del bot real:
sim.robot.x = real_robot.x
sim.robot.y = real_robot.y
sim.robot.theta = real_robot.theta
sim.update(dt)
sim.render()

# Así ves en tiempo real qué hace el bot
```

---

## 📞 Soporte

Si el simulador falla:
1. Verifica que Python sea 3.8+: `python3 --version`
2. Instala pygame: `pip3 install pygame`
3. Ejecuta desde el directorio correcto

¡Que disfrutes el simulador! 🎮🌾

