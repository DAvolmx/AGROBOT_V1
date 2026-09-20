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
