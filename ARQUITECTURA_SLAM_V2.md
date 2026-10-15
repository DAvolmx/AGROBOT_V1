# 🗺️ SLAM V2 — Arquitectura Mejorada (MathWorks + Visión)

## Problema Actual (v1)
- ❌ Dead-reckoning sin cierre de loops → deriva infinita
- ❌ Sin mapa persistente → no puede retomar caminos
- ❌ Sin corrección visual → acumula error de gyro
- ❌ Cámara sin zoom → no detecta plagas pequeñas
- ❌ Senderos detectados solo por color → falla en sombra

## Solución (v2)

### 1️⃣ SLAM Visual (MathWorks Approach)

```
┌─────────────────────────────────────────────────┐
│         CAMERA FRAMES (30 fps)                  │
└────────────┬────────────────────────────────────┘
             │
             ▼
    ┌────────────────────┐
    │ Feature Detection  │  ORB/SIFT features
    │ (500 pts/frame)    │
    └────────┬───────────┘
             │
             ▼
    ┌────────────────────┐
    │ Pose Estimation    │  PnP + RANSAC
    │ (Per frame)        │  (Camera → World)
    └────────┬───────────┘
             │
             ▼
    ┌────────────────────┐
    │ Loop Closure       │  BoW (Bag of Words)
    │ Detection          │  + Re-localization
    └────────┬───────────┘
             │
             ▼
    ┌────────────────────┐
    │ Pose Graph        │  g2o optimization
    │ Optimization       │  (Bundle adjustment)
    └────────┬───────────┘
             │
             ▼
    ┌────────────────────┐
    │ Mapa Denso         │  Depth from stereo
    │ Costmap            │  or monocular SfM
    └────────────────────┘
```

### 2️⃣ Camera Zoom System

```
┌─────────────────────────────────────────┐
│   CAMERA CONTROL (v4l2-ctl / libcamera) │
├─────────────────────────────────────────┤
│ • Zoom óptico: 1x → 4x (si soporta)     │
│ • Zoom digital: crop + upscale          │
│ • Auto-focus: Libcamera AF              │
│ • Expo/ISO dinámicos: Para plagas/luz   │
└─────────────────────────────────────────┘
     ↓
┌─────────────────────────────────────────┐
│  MULTI-SCALE DETECTION                  │
├─────────────────────────────────────────┤
│ Zoom 1x: Sendero/camino (60° FOV)       │
│ Zoom 2x: Plantas (30° FOV)              │
│ Zoom 4x: Plagas/hojas (15° FOV)         │
└─────────────────────────────────────────┘
     ↓
┌─────────────────────────────────────────┐
│  YOLO Multi-Resolution                  │
├─────────────────────────────────────────┤
│ • 640x480 @ 1x (fast, sendero)          │
│ • 1280x960 @ 2x (medium, plantas)       │
│ • 1920x1440 @ 4x (slow, plagas)         │
└─────────────────────────────────────────┘
```

### 3️⃣ Path Detection Engine

```
CAMERA → HSV Filter (sendero verde/marrón)
           ↓
         Morphology (cierre + apertura)
           ↓
         Contour finding (busca líneas)
           ↓
         Línea de regresión (center + angle)
           ↓
         Confidence scoring
           ↓
       ┌─ High (>90%) → Navegación por visión
       │
       ├─ Medium (70-90%) → Fusión LIDAR+Vision
       │
       └─ Low (<70%) → Fallback a LIDAR puro
```

### 4️⃣ Navegación Mejorada

```
LOCAL PLANNER (Nav2 DWB):
  • Entrada: Costmap (LIDAR + Vision)
  • Salida: Velocidades (v, omega)

┌─────────────────────────────────────────┐
│  TRAYECTORIA GLOBAL (from SLAM map)     │
├─────────────────────────────────────────┤
│ • Path planning con A* / RRT*           │
│ • Evita zonas de alto costo             │
│ • Replanifica si SLAM cierra loop       │
└─────────────────────────────────────────┘
          ↓
┌─────────────────────────────────────────┐
│  COSTMAP FUSIONADO                      │
├─────────────────────────────────────────┤
│ • Static: del mapa SLAM (1 hz)          │
│ • Dynamic: LIDAR actual (20 hz)         │
│ • Semantic: sendero (visión, 10 hz)     │
└─────────────────────────────────────────┘
```

---

## 📁 Archivos Nuevos/Modificados

### ✨ Nuevos
| Archivo | Función |
|---------|---------|
| `slam_v2.py` | SLAM visual con pose graph |
| `camera_zoom.py` | Control de zoom + multi-scale |
| `path_detection.py` | Detección de senderos |
| `semantic_costmap.py` | Fusión LIDAR + Visión |
| `pose_graph.py` | g2o wrapper para optimización |

### 🔄 Modificados
| Archivo | Cambios |
|---------|---------|
| `slam.py` | ✅ Mantiene PoseEstimator, añade SLAM visual como fusión |
| `camera.py` | ✅ Añade soporte multi-zoom, llamadas a path_detection |
| `navigation.py` | ✅ Usa costmap semántico |

---

## 🎯 Workflow Completo

```
STARTUP
  ↓
LOAD PREVIOUS SLAM MAP (si existe)
  ↓
START CAMERAS + LIDAR
  ↓
INITIALIZE POSE GRAPH
  ↓
┌─ MAIN LOOP (20 Hz) ─────────────────────┐
│                                         │
│ 1. LIDAR SCAN (20 hz)                  │
│    → LidarReactive + CostmapLocal      │
│                                         │
│ 2. CAMERA FRAME (30 fps)               │
│    → Feature extraction                │
│    → Pose estimation                   │
│    → Add to pose graph                 │
│                                         │
│ 3. EVERY 100 frames:                   │
│    → Loop closure detection            │
│    → Pose graph optimization (g2o)     │
│    → Update map                        │
│                                         │
│ 4. EVERY 500 frames:                   │
│    → Path detection                    │
│    → Update semantic costmap           │
│    → Replan global path                │
│                                         │
│ 5. CONTROL                             │
│    → Local planner (DWB)               │
│    → Send to C3                        │
│                                         │
└─────────────────────────────────────────┘
      ↓
SHUTDOWN
  ↓
SAVE SLAM MAP + Poses
```

---

## 💾 Persistencia

### Mapa SLAM (durability)
```
agrobot_map.slam/
  ├── keyframes.npz      # Frames clave (poses + descriptores)
  ├── poses.json         # Historial de poses
  ├── loop_closures.json # Restricciones de cierre de loop
  ├── depth_map.npy      # Mapa de profundidad denso
  └── metadata.json      # Calibración, timestamp, etc.
```

### Cámara Zoom (por resolución)
```
zoom_1x.yaml    # Parámetros a 1x
zoom_2x.yaml    # Parámetros a 2x
zoom_4x.yaml    # Parámetros a 4x
```

---

## 🔌 Dependencias Nuevas

```bash
pip install opencv-contrib-python  # ORB/SIFT/AKAZE
pip install g2o-py                 # Pose graph optimization
pip install dbow2                  # Loop closure (Bag of Words)
pip install numpy scipy            # Math
pip install v4l2-python            # Cámara control (libv4l2)
pip install piexif                 # EXIF para metadatos
```

---

## 📊 Parámetros Configurables

```json
{
  "slam_v2": {
    "feature_detector": "ORB",           // ORB / SIFT / AKAZE
    "max_features": 500,
    "min_tracked_points": 50,
    "loop_closure_threshold": 0.7,
    "pose_graph_iterations": 20,
    "keyframe_interval": 10,             // frames entre keyframes
    "depth_estimation": "monocular_sfm"  // o "stereo" si hay
  },
  "camera_zoom": {
    "zoom_levels": [1, 2, 4],
    "zoom_strategy": "adaptive",         // o "manual"
    "auto_zoom_confidence": 0.6,
    "focus_mode": "continuous",
    "exposure_compensation": 0
  },
  "path_detection": {
    "sendero_h_min": 30,
    "sendero_h_max": 90,
    "sendero_s_min": 60,
    "confidence_threshold": 0.7,
    "temporal_smoothing": 0.8
  }
}
```

---

## 🚀 Fases de Implementación

### FASE 1: SLAM Visual (v2.0)
- ✅ Feature extraction (ORB)
- ✅ Pose estimation (PnP + RANSAC)
- ✅ Pose graph basic
- Timeline: 2-3 días

### FASE 2: Loop Closure + Map Optimization (v2.1)
- ✅ Loop closure detection (BoW)
- ✅ g2o pose graph optimization
- ✅ Persistent map saving/loading
- Timeline: 3-4 días

### FASE 3: Camera Zoom + Multi-Scale (v2.2)
- ✅ v4l2 zoom control
- ✅ Multi-resolution YOLO
- ✅ Auto-zoom strategy
- Timeline: 2-3 días

### FASE 4: Advanced Path Detection (v2.3)
- ✅ Semantic segmentation (sendero)
- ✅ Temporal smoothing
- ✅ Confidence scoring
- Timeline: 2 días

### FASE 5: Integration + Testing (v2.4)
- ✅ Fusión SLAM + Visión + LIDAR
- ✅ Dashboard con mapa actualizado
- ✅ Field testing
- Timeline: 3-4 días

**Total: ~2-3 semanas** para sistema completo

---

## ✅ Resultados Esperados

| Métrica | v1 | v2 |
|---------|----|----|
| **Deriva (100m)** | 5-8m | <1m |
| **Loop closure** | ❌ No | ✅ Sí |
| **Reuso de mapa** | ❌ No | ✅ Sí |
| **Detección plagas** | Solo grandes | Pequeñas (zoom 4x) |
| **Path detection** | 60-70% (color) | 90%+ (semantic) |
| **Latencia total** | 300-500ms | 100-200ms |
| **Mapa persistente** | ❌ No | ✅ Sí |

