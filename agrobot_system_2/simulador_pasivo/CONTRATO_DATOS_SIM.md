# Contrato de datos del simulador AgroBot → percepción/etiquetas (Codex)

Versión 0.2 · 2026-10-11 · clases ALINEADAS con el dataset real de Codex (aún no
implementado el volcado; este documento fija QUÉ producirá el simulador).

**Reparto de trabajo:** este lado (física/sim/evaluación) genera RGB + verdad del
simulador. Codex se encarga del dataset de imágenes, las etiquetas de percepción
y el detector de sendero. No duplicar.

## Regla de oro
La **verdad del simulador** (pose real, máscaras, contactos, altura de obstáculos)
sirve para **etiquetas y evaluación**, y **NO debe entrar como entrada al
controlador**. El controlador solo consume lo que vería el robot real: imagen
RGB, LIDAR simulado con su ruido, pose ESTIMADA, ángulo de cámara comandado.

## Estructura por episodio
```
datasets_sim/<escenario>/<seed>/
  meta.json                 # config + seed del episodio
  camara.json               # parámetros intrínsecos + montaje de la cámara
  frames/000123.json        # un registro por fotograma (ver abajo)
  rgb/000123.png            # imagen RGB (entrada del robot)
  mask_sem/000123.png       # máscara semántica (etiqueta)   [verdad del sim]
  mask_inst/000123.png      # ids de instancia por objeto     [verdad del sim]
```

## `meta.json` (por episodio)
`seed`, `escenario`, `version_sim`, `dt`, `duracion_s`, `split` ∈ {train,val,test},
`parametros_fisica` (hash + los valores usados), `notas_hipotesis`.

## `camara.json`
`ancho`, `alto`, `fov_deg` (o fx,fy,cx,cy), `altura_m`, `montaje_inclinacion_home_deg`,
`calibracion_verificada` (=false hoy). El tilt por fotograma va en cada frame.

## `frames/NNNNNN.json` (por fotograma)
Separa claramente **entrada**, **verdad** y **estimado**:
```json
{
  "t": 12.34,
  "entrada": {
    "rgb": "rgb/000123.png",
    "lidar": [[ang_deg, dist_m], ...],
    "servo_angle_deg": 74.0,
    "servo_asentado": true,
    "pose_estimada": {"x": .., "y": .., "psi": ..}
  },
  "verdad_sim": {
    "pose_real": {"x": .., "y": .., "psi": ..},
    "pivote_delta_deg": 7.2,
    "camara_inclinacion_abajo_deg": 34.7,
    "contactos": [{"rueda":"FL","N":.., "slip_long":.., "slip_lat":..}, ...],
    "obstaculos": [{"id":3, "tipo":"monticulo", "altura_m":0.05,
                    "dist_m":0.8, "cruza_plano_lidar": false}],
    "mask_sem": "mask_sem/000123.png",
    "mask_inst": "mask_inst/000123.png"
  }
}
```

## Clases semánticas — ALINEADAS con el dataset de Codex (v0.2, 2026-10-11)
Se adoptan EXACTAMENTE las clases de `dataset_info.json` de Codex (mismo id, para
que las máscaras del sim entren directas en su pipeline):

| id | nombre | qué es en el sim |
|----|--------|------------------|
| 0 | `fondo` | todo lo no clasificado |
| 1 | `sendero_visible` | el suelo transitable del sendero |
| 2 | `zona_cultivo_no_pisar` | plantas / bancales de cultivo |
| 3 | `bancal_borde_elevado` | el borde elevado del bancal |
| 4 | `obstaculo_visible` | **montículos / obstáculos bajos** (lo que vuelca/atranca) |

Las máscaras se guardan como PNG de 1 canal con el id de clase por píxel
(`ignore_index: null`, como su dataset). La tabla se versiona en `meta.json`.

### Por qué esto importa (gap detectado en las métricas de Codex)
En su entrenamiento la clase **`4 obstaculo_visible` salió IoU = null/0.00**: el
dataset NO tiene ejemplos de obstáculos. El simulador puede **rellenar justo esa
clase** generando escenas con montículos bajos etiquetados (altura, posición),
que es la información que el LIDAR alto no ve y que causó el vuelco del 2026-10-09.
Esta es la mayor aportación del sim a la percepción.

### Nomenclatura y splits (para encajar con su dataset)
Imágenes sintéticas del sim: `synth-NNN.jpg` (mismo prefijo que su fuente
sintética). Se añaden a `train.txt` / `val.txt` / `test.txt` (un nombre por línea).
AVISO de dominio: en sus métricas `mIoU_real=0.73` pero `mIoU_synth=0.45` → hay
brecha real↔sintético. Por eso la CALIDAD VISUAL de las escenas del sim importa
(separada de la física); conviene validar que lo sintético del sim AYUDA y no
degrada, con un holdout real, antes de mezclarlo masivamente.

## Sincronización y marca de tiempo
Cada fotograma lleva `t` (reloj del sim), `servo_angle_deg` y `servo_asentado`
(el servo oscila tras moverse; con `servo_asentado=false` la geometría no vale —
útil para que Codex descarte o pondere esas capturas).

## Lo que el simulador NO afirmará (honestidad)
- No presentará mapas hechos con pose real como SLAM real.
- No dará probabilidades de detección supuestas como precisión medida.
- No inventará NDVI ni profundidad métrica desde la webcam RGB.

## Estado (v0.2, 2026-10-11)
Clases ALINEADAS con el dataset real de Codex (5 clases). Siguiente paso de este
lado: implementar el volcado sincronizado (RGB + máscara con esos ids) respetando
la regla de oro, PRIORIZANDO escenas con `obstaculo_visible` etiquetado (la clase
que a Codex le falta). Validar aporte con holdout real antes de mezclar.
Comentarios de Codex → v0.3.
