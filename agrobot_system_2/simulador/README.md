# Simulador del AgroBot

Corre **el código real del robot** contra un mundo virtual. No hay una copia
de la lógica: se importan y ejecutan los módulos de la Jetson —
`slam.LidarReactive` (freno), `sendero.SeguidorSendero` (cámara),
`explorador.ExploradorReactivo` (autonomía) y `pulse_drive.ConductorPulsos`
(pulsos de manejo)— así que lo que se arregla aquí es lo que corre en el campo.

Sirve para entrenar y probar sin sacar el robot: seguir el sendero, esquivar,
escanear plantas, salir de atascos y detectar cuándo se queda dando vueltas.

## Qué simula

| Pieza | Cómo |
|---|---|
| LIDAR | 720 rayos a 10 Hz contra las paredes, con ruido y alcance de 8 m |
| Cámara | Imagen sintética por trazado de rayos: tierra, paja fibrosa, plantas, paredes |
| Chasis | Skid-steer con **piso de motores** (PWM 200..255), patinaje al girar y deslizamiento al soltar |
| Servo de cámara | Inclina (no panea): 25°–110°, 1° de servo ≈ 0,8° de inclinación, con tiempo de asentado |
| SLAM | Pose estimada con deriva y saltos de cierre de bucle + mapa de ocupación |
| Objetos | Paredes y macetas (sólidos), ramas y arbustos (los ve el LIDAR pero se atraviesan), plantas chicas (invisibles al LIDAR, el robot **las pisa**) |
| Rampa | La misma `safety.max_*_accel` de `agrobot_config.json` |

La distinción clave, que es el problema real del robot: **las paredes son
altas y el LIDAR las ve; el sendero, la paja y las plantas chicas son bajas y
solo existen para la cámara** (el LIDAR va a ~30 cm y pasa por encima).

## Escenarios

| Nombre | Qué entrena |
|---|---|
| `bucle` | La huerta del usuario: sendero en bucle alrededor de un cantero. Seguir el camino y dar la vuelta |
| `hileras` | Cultivo en hileras paralelas: centrarse entre dos hileras |
| `pasillo` | Pasillo de 0,9 m con un giro: no rozar y girar en sitio justo |
| `callejon` | Bolsillo cerrado: reconocer que no hay salida y volver |
| `trampa_bucle` | Obstáculo central simétrico: detectar que está dando vueltas |
| `aleatorio` | Sala, sendero, obstáculos y plantas distintos cada vez |

Además, en el **Editor de mapas** se dibujan mapas propios (sendero, paredes,
plantas, puntos de escaneo) y se pueden importar los **mapas reales del SLAM**
del robot (`maps/<nombre>/mapa.pgm`) para dibujarles encima el recorrido.

## Uso

```bash
python3 simulador/correr.py --escenario bucle --segundos 120
python3 simulador/correr.py --todos --segundos 120 --salida simulador/resultados
python3 simulador/correr.py --escenario callejon --video callejon.mp4
```

Cada episodio deja en `--salida`: un CSV con la misma forma que los registros
reales del robot (`~/.agrobot/navigation_logs`) más el desvío del sendero, los
eventos en JSON, el resumen de métricas y un PNG con la trayectoria.

El reloj es **virtual**: un episodio de 2 minutos tarda unos 20 s reales
(entre 3× y 17× según el escenario).

## En WSL (Windows)

```bash
sudo apt update && sudo apt install -y python3-pip python3-opencv python3-numpy
git clone <tu-repo> && cd AGROBOT/agrobot_system_2
python3 simulador/correr.py --todos --segundos 120
```

No hace falta ROS, ni GPU, ni interfaz gráfica: todo se escribe a archivos.
Para ver los vídeos y PNG desde Windows, la carpeta de WSL está en
`\\wsl$\Ubuntu\home\<usuario>\...`.

## Entrenamiento (de la simulación al robot real)

```bash
python3 simulador/entrenar.py --todo --episodios 20     # recolectar + entrenar + evaluar
```

1. **Recolectar**: genera escenarios al azar y los recorre con el piloto de la
   ruta ideal (por el centro del sendero). Cada pulso es un ejemplo
   "percibiendo esto → había que hacer esto". Los datos se acumulan en
   `~/.agrobot/aprendizaje/dataset.csv`: cada tanda suma a la anterior.
2. **Entrenar**: una red pequeña (14 entradas → 4 acciones, ~5000 parámetros)
   imita esos ejemplos. Sale `~/.agrobot/aprendizaje/politica.pt`.
3. **Evaluar**: corre los mismos escenarios con y sin la política y compara.

Medido el 2026-09-20 con 3018 ejemplos (82,6 % de acierto en validación):

| | Sin política | Con política |
|---|---|---|
| Desvío de la ruta | 35,7 cm | **3,5 cm** |
| Plantas pisadas | 1,67 | **0** |
| Choques | 1,0 | **0** |

En el robot real se activa con `autonomia.politica_activa` en la config. El
freno por LIDAR y las reglas de atasco siguen por encima: la política
propone, no dispone.

## Corregir en vivo y enseñar maniobras

En el dashboard, pestaña **Aprendizaje**: se puede corregir al robot **sin
salir de automático**. En cuanto se toca un control, la autonomía cede el
mando unos segundos. Los pulsos seguidos se guardan como UNA maniobra atada a
la situación (hueco libre alrededor, dónde cae el sendero, qué intentaba
hacer). Cuando el robot vuelva a verse así, la repite.
Se guardan en `~/.agrobot/aprendizaje/maniobras.json`, que lee el robot real.

## SLAM simulado, ruta ideal y comparación

Pestaña **SLAM y ruta**:

- El robot navega con la pose que CREE tener, no con la verdadera: deriva
  (~3,5 % por metro, más en los giros) y da saltos al cerrar bucles, como en
  el robot real. Se ve el mapa que va construyendo, con las paredes
  emborronadas cuando la pose se fue.
- **Iniciar recorrido ideal**: traza una línea por el centro del sendero y la
  recorre con persecución pura. Es la referencia de "así había que hacerlo".
- **Volver al inicio y comparar**: repite el mismo tramo en automático y
  enfrenta las dos: desvío, plantas pisadas, choques y error del SLAM.
- **Filtro de conducción ideal**: deja conducir a la autonomía pero corrige
  el pulso cuando se aleja de la ruta más de media anchura de sendero.

## Métricas

- `recorrido_m`, `colisiones` (episodios de choque) y `contacto_s` (tiempo rozando)
- `en_sendero_pct` y `desvio_medio_cm`: cuánto se mantiene en el camino
- `pulsos`, `giros`, `correcciones_sendero`, `desatascos`
- `escaneos` y `plagas_detectadas`
- `bucles_detectados`: volver 3 veces a la misma celda de 50 cm tras 15 s fuera

## Límites (lo que NO simula)

- Física de contacto real: un choque para el robot, no lo vuelca ni lo desvía.
- Terreno plano: la huerta real tiene pendiente, y eso descuadra la vista cenital.
- SLAM: la pose que ve el software es la verdadera, sin deriva ni saltos. Para
  estudiar la deriva hay que usar los registros del robot real.
- Iluminación: la cámara sintética no tiene sol duro, sombras móviles ni motion blur.
