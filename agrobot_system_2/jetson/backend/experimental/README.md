# Código experimental / NO integrado (movido 2026-10-07)

Estos módulos NO los importa `jetson_server.py` ni ningún proceso real
(verificado con grep en todo el repo: solo se importaban entre ellos). Se
movieron aquí para que no se conecten por error. Problemas detallados en
`../../../../BITACORA_AUDITORIA.md` y en la memoria del proyecto:

- `slam_v2.py` — promete g2o/triangulación/loop-closure pero no lo implementa
  (PnP sin puntos 3D, reproyección 3x3·vec4, loop closure por media de
  descriptores). El SLAM real es `slam_toolbox` (ROS), que sí corre.
- `control_directo.py` / `path_detection.py` (PathDecision) — emiten velocidad
  CONTINUA que el chasis no ejecuta (firmware: PWM mínimo físico ~200/255).
- `vision_system_v3.py` — orquestador que depende de los anteriores.
- `camera_zoom.py` — idea rescatable (zoom adaptativo), pero acá sin uso.

## Restaurar (volver a dejarlos en backend/)
    mv experimental/*.py .
    # y borrar este README si ya no hace falta
