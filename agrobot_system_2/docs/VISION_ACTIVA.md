# Visión activa (2026-10-22)

El robot mueve el **servo de tilt** y gira en **pulsos cortos** para mirar el entorno, centrar un objetivo (planta/maceta/col), acercarse
con el LIDAR y analizarlo (foto + registro + segunda opinión con el clasificador PLAGASX). La cámara solo tiene tilt: el «paneo» lo hace el robot.

## Usarla (APAGADA por defecto)
```bash
# ENSAYO: mueve el servo y decide, pero el robot NO se mueve (los pulsos solo se registran)
curl -X POST localhost:8080/api/params/vision_activa -H 'Content-Type: application/json' -d '{"sin_mover": true}'
curl -X POST localhost:8080/api/vision_activa -H 'Content-Type: application/json' -d '{"activa": true}'
curl localhost:8080/api/vision_activa            # estado: estado, servo, objetivo (yaw/pitch), pulsos, aviso, resultado
# REAL (el robot se mueve en pulsos): primero ensayo, luego
curl -X POST localhost:8080/api/params/vision_activa -H 'Content-Type: application/json' -d '{"sin_mover": false}'
curl -X POST localhost:8080/api/vision_activa -H 'Content-Type: application/json' -d '{"activa": false}'    # parar
```
- **Modelo activo:** para centrar hace falta uno que LOCALICE: «Interior (macetas)» o «Col (planta y cogollo)». PLAGASX tiene cajas de imagen
  completa (no localiza): se ignoran con un aviso. La clasificación de enfermedad la hace PLAGASX sobre la foto al final.
- **Resultados:** `~/.agrobot/vision_activa/AAAAMMDD.jsonl` y fotos en `~/.agrobot/vision_activa/fotos/`.
- **Seguridad:** se pausa sin scan fresco del LIDAR, sin C3 o con la exploración/patrulla activa; nunca avanza con < 20 cm de hueco; la
  parada de emergencia, el STOP de manejo o cualquier orden manual la cancelan; la anticolisión con retroceso sigue activa; límites de pulsos,
  giros de búsqueda y tiempo (`tiempo_max_s`).
- **Parámetros** (sección `vision_activa`, editables con `/api/params/vision_activa`): clases_objetivo, conf_min, tolerancias de rumbo y tilt,
  distancia_inspeccion_m, camara_yaw_offset_deg (si la cámara no mira recto), ganancia_tilt, rango del servo, etc.

## Lo medido (ensayo en vivo, robot en el piso)
El servo centró verticalmente una maceta en ~7 s (98° → 90°); detectó que estaba ~9° a la izquierda y pidió giros a la izquierda. El
robot no se movió (modo ensayo), así que **el giro y el avance reales siguen sin probarse**.
