# Exposición 2026-10-16 — lista de comprobación

## Estado dejado (2026-10-16)
- **Autónomo desbloqueado**: el «Perfil de demostración» (`demo_guard`) ya no bloquea exploración, misión,
  patrulla ni Nav2. Se controla con `control.solo_manual` (False ahora; True lo vuelve a bloquear).
- **Freno por LIDAR/cámara SOLO en autónomo** (`safety.freno_lidar_autonomo`, True): no avanza si hay algo a
  menos de margen+deslizamiento (12 + 30 = 42 cm libres desde la carrocería) ni sin LIDAR fresco (> 0,6 s).
  El manejo manual no cambia. Verificado con tests (frena ante obstáculo, avanza con camino libre, sin datos
  no avanza, manual intacto).
- **Giro en el sitio activado** (`control.pivote_en_sitio` True): sin él Nav2 no puede girar parado.
  NO verificado en el robot con el eje pasivo: pruébalo con las ruedas en el aire antes.
- **Plantas vs plagas**: las clases de `detection.clases_planta` (col_planta, col_cogollo, maceta_planta)
  disparan el escaneo pero NO el aviso «Plaga detectada».
- SLAM (slam_toolbox) activo: /scan a 10 Hz, /map a 1 Hz, EKF publicando.

## Antes de arrancar (2 min)
1. Dashboard → LIDAR «midiendo» y mapa creciendo. 2. Pestaña Motores: «C3 conectada» y las 4 ruedas.
3. Robot con ≥ 60 cm libres delante. 4. Botón rojo de emergencia a mano. 5. Batería de motores cargada.
6. Prueba manual corta (adelante/atrás/giro) para ver sentido de cada rueda.

## Arrancar
- «🤖 Modo autónomo» = patrulla por los waypoints guardados (Nav2 + SLAM), escanea al detectar planta.
- «Seguir sendero» = exploración reactiva entre hileras (por pulsos; en pasillos de < 15 cm de margen por
  lado el simulador la mostró poco fiable).
- PARADA DE EMERGENCIA cancela todo (exploración, patrulla, Nav2, saludo, pulsos, motores).

## Ajustes en caliente (Parámetros / Motores)
- Más/menos cautela del freno: `slam.deslizamiento_cm` (30 ahora; sube si se acerca demasiado).
- Velocidad del autónomo: `autonomia.escala_velocidad` (0,5 ahora; bájala a 0,3 para ir más seguro).
- Fuerza y suavidad: pestaña Motores → «Fuerza y suavidad».

## Lo que NO está demostrado (decirlo si preguntan)
- **Detección de mosca blanca/pulgón en col: NO funciona** con los modelos disponibles (probado en tus 4 fotos).
  El detector de col entrenado (`entrenar_col.py`) detecta planta, cogollo y daño de hoja (masticadores),
  validado solo con fotos de otro campo y cámara.
- El freno no evita roces laterales entre macetas muy juntas (solo mira hacia donde avanza).
- Navegación en simulación: 42 % de éxito en interiores con el firmware nuevo; no es una garantía de campo.
