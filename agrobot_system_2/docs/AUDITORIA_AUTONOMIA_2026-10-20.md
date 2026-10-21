# Auditoría de la autonomía — 2026-10-20

Objetivo: comprobar que NADA impide que «Explorar» / «Iniciar patrulla» / Nav2 arranquen y se muevan, y dejar
una lista de verificación repetible: `python3 tools/preflight_autonomia.py [--ros] [--suelo-ok]` (solo lee).

## Resultado
**0 bloqueos.** Estado medido en vivo: C3 conectada (WiFi, sin failsafe), LIDAR a ~10 Hz (scan de 70 ms), pose SLAM
presente, cámara ok, IMU fresco y nivel calibrado, y slam_toolbox / planner / controller / bt_navigator / behavior_server
**active** (lifecycle). rf2o publica una sola odometría (`/odom_rf2o`, 1 publicador; el nombre de nodo repetido en
`ros2 node list` es de un único proceso).

## Todo lo que puede frenar o rechazar la autonomía (y en qué estado está)
| # | Dónde | Qué hace | Estado |
|---|---|---|---|
| 1 | middleware `demo_guard` (`control.solo_manual`) | devuelve 409 a misión, exploración, evasión, patrulla-auto, Nav2, pasos | **false** ✓ |
| 2 | `exploracion/start` | exige scan de menos de 1 s y patrulla parada | ✓ |
| 3 | `patrol.iniciar` | sin waypoints rechazaba | ahora **cae a explorar** y graba el recorrido ✓ |
| 4 | `/api/drive`, WebSocket `control` | exigen C3 con enlace | C3 conectada ✓ |
| 5 | modo seguro (`safety.modo_seguro_activo`) | detenía la autonomía y se enclavaba (VUELCO por IMU sin nivelar) | **apagado**; además el IMU viejo (>3 s) ya no cuenta |
| 6 | `safety.freno_lidar_autonomo` | corta el avance a margen+deslizamiento de un obstáculo | **apagado (tu configuración)** ⚠ ver abajo |
| 7 | `safety.supervisor_activo` | enclava emergencia y exige rearmar | apagado ✓ |
| 8 | monitor de patinaje/atasco | comentado en el código (no frena) | inactivo ✓ |
| 9 | rampa del actuador / `DEADMAN_S` | limita aceleración; suelta a 0 si deja de llegar el mando | normal, no bloquea |
| 10 | firmware C3: failsafe de 500 ms | para los motores si se corta el enlace | **debe quedar** (seguridad real) |
| 11 | umbrales del explorador (`avance_libre_cm`, `lado_libre_cm`) | si no hay ese hueco, no avanza: gira o retrocede | **dependen del modo**: exterior 100/60 cm, interior 40/30 cm. El entorno automático (activo) eligió interior |
| 12 | sendero: `parada_incierta_ciclos`, `si_no_cabe` | podían parar el robot sin referencia | 0 (desactivado) y «aviso» ✓ |
| 13 | botón STOP de Manejo | parada total | ya no se anuncia como emergencia y no enclava nada |

## Lo que queda de tu lado (no es software)
- ⚠ **Mesa:** el LIDAR no ve el vacío; con el robot en una mesa el explorador avanzará hasta caerse. Pruebas en el suelo.
- ⚠ **Freno LIDAR autónomo apagado:** no impide arrancar, pero en las primeras corridas autónomas conviene activarlo
  (`safety.freno_lidar_autonomo=true`, Parámetros → Seguridad): corta el avance a ~42 cm de un obstáculo (12 + 30 de deslizamiento).
- ⚠ **Encajonado:** con 3–5 cm de hueco a los lados no puede girar (necesita `lado_libre`): sácalo a un sitio abierto.

## Cambios hechos hoy que afectan a la autonomía
- Entorno automático interior/exterior ACTIVADO (`control.entorno_auto`): pasó solo a **interior** (escala 0,13, avance_libre 40,
  lado_libre 30, deslizamiento 20, cono 30°). Si en la huerta se equivoca, se fija a mano y calla 10 minutos.
- «Iniciar patrulla» sin recorrido grabado → explora y mapea; el zoom vuelve a 1× al iniciar.
- Calibración de nivel del IMU y descarte de telemetría vieja.
