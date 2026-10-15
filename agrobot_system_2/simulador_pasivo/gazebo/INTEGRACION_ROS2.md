# Perfil ROS 2 aislado para Gazebo

Gazebo Sim 8 (Harmonic), `ros_gz_bridge` y el controlador RGB/LiDAR ya existen
en esta Jetson. Este perfil proporciona arranque y apagado repetibles sin
conectar comandos de la simulación al robot real.

## Contrato común

`robot_contract.json` documenta las medidas de geometría compartidas y los
tópicos ROS de la simulación. La huella declarada es 50 × 33 cm, ancho exterior
delantero 19 cm, ancho trasero 37 cm, batalla 37 cm y pivote delantero pasivo
limitado a ±20°. Los datos de masa, centro de gravedad, radio de rueda, fricción
y respuesta de motor siguen siendo hipótesis pendientes de medir.

Los mensajes usan tipos ROS estándar: `Twist`, `Odometry`, `LaserScan`, `Image`
y `Contacts`. El simulador usa `/agrobot/...`; el robot real mantiene sus
tópicos habituales en ROS Domain 0. El Domain 42 de la simulación es obligatorio
para que ni Nav2 ni el puente del robot real reciban `/agrobot/cmd_vel`.

## Surcos físicos

El escenario generado añade dos caballones laterales de tierra con geometría elipsoidal y colisión en Gazebo. Cinco tramos largos solapados por lado forman dos bordes continuos, con ligeras variaciones de altura para representar el sendero irregular de las fotos. La separación libre mínima entre sus caras interiores es 0.50 m; los cultivos quedan por fuera del borde y los obstáculos de los perfiles mixto/bajo pueden ocupar el corredor. Las hojas del modelo se dibujan como óvalos 3D redondeados y conservan colisiones aproximadas para no elevar demasiado el coste del render. La forma, altura y fricción de estos surcos son aproximaciones visuales y físicas, pendientes de medir en el huerto.

Para reducir la carga del render manteniendo detalle visual, la configuración final usa cámara 240×180/5 Hz y LiDAR GPU de 120 muestras/3 Hz. El piloto y `diagnosticar_camera.py` comparten `percepcion_sendero.py`: intentan primero los bordes verdes de los cultivos y, si no hay señal suficiente, buscan contraste a ambos lados del suelo para seguir los caballones. Es una heurística de simulación, no una validación de seguridad ni una red entrenada; las imágenes nuevas del huerto servirán para verificarla con cambios de luz, barro y vegetación.

## Incidencia de cámara durante movimiento (12-oct-2026)

La cámara entrega imagen útil mientras el robot está quieto: bajo carga, una prueba de 5 s a 240×180 obtuvo 9 frames, 100% RGB utilizable y 100% de ventanas con bordes de surco (media 19 filas). Al avanzar 8 s, la cámara pasó a frames planos (0% utilizable, 100% negros) y el piloto se detuvo por seguridad; avanzó 0.165 m, con error transversal medido de 0.18 cm y sin contactos. Reducir resolución y frecuencia no eliminó la incidencia. Gazebo Sim 8/Harmonic en esta Jetson presenta un defecto de renderizado OGRE2 de cámara al mover modelos; el reporte coincide con el problema documentado por Gazebo: https://github.com/gazebosim/gz-sim/issues/2708. Cambiar a OGRE 1 no es una alternativa en esta instalación: el plugin abortó al iniciar. Se restauró OGRE2 y se verificó el RGB estático y el tópico LiDAR después.

Por seguridad, el piloto se detiene si no recibe detección reciente durante 0.55 s y reduce la velocidad cuando la imagen tiene más de 0.30 s. No usar las métricas de avance de esta corrida como prueba de navegación autónoma ni entrenar desde esos frames vacíos. Para aprobar conducción en Gazebo se requiere repetir prueba con movimiento y obtener al menos 90% RGB utilizable y 50% de frames con bordes; la prueba estática sola no satisface ese criterio.

## Arrancar y probar

```bash
cd ~/Desktop/AGROBOT/agrobot_system_2/simulador_pasivo/gazebo
bash iniciar_ros_gazebo.sh
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic list
python3 diagnosticar_camera.py --seconds 10 --out ~/.agrobot/gazebo_sim_ros/camera_quality.json
python3 validar_escenario.py --duration 9 --salida ~/.agrobot/gazebo_sim_ros/smoke.json
python3 seguir_sendero.py --duration 20 --salida ~/.agrobot/gazebo_sim_ros/lane_metrics.json
```

La ventana de captura debe lograr ≥90% de imágenes utilizables (variación y al
menos 16 colores distintos; así se rechazan frames planos) y ≥50% con al menos
dos filas de borde detectadas por plantas o surcos. Esa prueba sola no aprueba
entrenamiento: `camera_approved_for_training` requiere además que una prueba
separada con movimiento haya pasado y se invoque `diagnosticar_camera.py` con
`--movement-verified`. `validar_escenario.py` puede publicar movimiento, pero solo dentro de
Domain 42. Para apagar únicamente Gazebo y su puente:

```bash
bash detener_ros_gazebo.sh
```

Los datos de prueba y logs quedan bajo `~/.agrobot/gazebo_sim_ros/`.
El arranque predeterminado genera un corredor mixto repetible y usa una
inclinación descendente inicial estimada para que la cámara encuadre los cultivos
bajos; hay que reemplazar esa hipótesis por la calibración medida del servo.

## Interpretación actual

Las pruebas de reposo pasan, pero el ensayo móvil de 8 s detectó sendero en
7/22 frames y registró 15 frames planos; no cumplió el criterio de aprobación.
Por eso la cámara y el controlador aún no están validados para entrenar o
transferir al robot. La simulación de Python y Gazebo tampoco deben compararse
como si ya compartieran parámetros dinámicos identificados; primero hay que
medir ruedas, masas, pivote y fricción reales.


