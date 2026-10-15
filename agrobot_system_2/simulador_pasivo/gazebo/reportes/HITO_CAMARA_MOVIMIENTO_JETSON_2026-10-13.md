# Hito: cámara válida durante movimiento y memoria estable

**Fecha de validación:** 13 de octubre de 2026 (hora de la Jetson)  
**Equipo:** Jetson Orin, Gazebo Harmonic, ROS 2 Jazzy  
**Mundo:** `huerta_ruda` / `generados/huerta_mixta.sdf`  
**Dominio ROS:** 42 (aislado del robot real)

## Cambio aplicado

Se configuró la cámara RGB del mundo en **3 Hz de simulación**, a 240 × 180, y se conservó el renderizado EGL de NVIDIA con sombras desactivadas para reducir la carga de texturas. Las cuatro juntas de rueda mantienen su eje de giro alineado con el cilindro (`0 -1 0`). Se reiniciaron Gazebo y el bridge para aplicar la configuración; ambos quedaron activos.

## Prueba reproducible

Se publicó `linear.x = 0.04 m/s` durante 8 s, con imagen RGB, odometría y reloj simulados en ROS Domain 42. El topic de cámara se midió después con `ros2 topic hz`.

| Métrica | Resultado |
|---|---:|
| Frames capturados en la prueba de movimiento | 25 en 13 s |
| Frecuencia efectiva sostenida | 1.943 imágenes/s |
| Intervalo entre imágenes | 0.511–0.520 s |
| Dispersión del intervalo | 0.0033 s |
| Odometría longitudinal al terminar | +0.2036 m |
| Avance esperado por RTF 0.65 | 0.208 m |
| Diferencia avance medido/esperado | ~2.1 % |
| RAM usada / total | 1.9 / 7.4 GiB |
| RAM disponible | 5.5 GiB |
| Swap usado / total | 320 MiB / 2 GiB |

La captura `gazebo_camera_movimiento_3hz.png` se tomó durante el avance: el sendero de tierra aparece entre los surcos y la vegetación. Las imágenes durante el movimiento tienen cientos de colores únicos; los frames repetidos al final de la serie corresponden a la orden de velocidad cero y a la escena ya quieta, no a una cámara congelada.

## Estado de servicios

- `gz sim --headless-rendering -s -r`: PID 175743, activo al cerrar la validación.
- `ros_gz_bridge`: PID 175766, activo.
- Cámara `/agrobot/camera/image_raw`: frecuencia sostenida cercana a 1.94 Hz en tiempo real. El factor de tiempo del mundo (0.65) limita la tasa real aunque el sensor esté a 3 Hz simulados.
- Entrenamiento: **detenido** para no volver a agotar memoria. Los datos y la política candidata previos se conservaron bajo `~/.agrobot/aprendizaje_pivote_seguimiento_v4/`; el modelo no se sustituyó.

## Alcance y siguiente validación

Este hito verifica transporte/render RGB, movimiento hacia delante en la odometría, estabilidad básica de memoria y persistencia de los servicios. No demuestra todavía que el segmentador detecte el sendero con precisión, que el robot esquive todos los obstáculos, ni que las hipótesis de contacto, masa o fricción coincidan con el robot físico. La velocidad usada fue deliberadamente baja y la validación fue en simulación.

El siguiente ensayo debe medir seguimiento visual y tasa de detección durante una ruta completa, además de registrar pose dinámica e inclinación del chasis al cruzar los surcos. No se debe conectar el controlador de esta prueba al robot real sin validar el signo/ganancia del pivote y comprobar el paro de emergencia.
