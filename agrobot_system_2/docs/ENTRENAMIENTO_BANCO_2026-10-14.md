# Entrenamiento: qué fallaba en el banco y qué se parcheó — 2026-10-14

Continuación de `AUDITORIA_CORREDOR_SEGURO_2026-10-14.md`. Todo esto es
**simulador**. `contrato_robot.json` sigue con `calibrated: false`: masa,
inercias, fricción y respuesta PWM son estimaciones. Nada de aquí se desplegó
al robot ni se midió en campo.

---

## 1. El fallo de fondo: el banco pedía rutas imposibles

`evaluacion.py` puntuaba seis escenarios. Dos de ellos piden al eje pasivo más
de lo que puede dar. Con la batalla medida (0,37 m) y el contrato:

```
R_min = batalla / tan(pivote)      límite útil 17° -> 1,210 m
                                   tope mecánico 20° -> 1,017 m
```

| escenario | radio | pivote que exige | veredicto |
|---|---|---|---|
| `curva` | 1,2 m | **17,1°** | pasa el límite útil |
| `ese` | 1,0 m | **20,3°** | **pasa el TOPE MECÁNICO** |

`ese` es geométricamente imposible: su `progreso = 0,47` medía cinemática, no
control. Y su coste (`6·(1−progreso)`) es un **suelo irreducible**.

**Esto explica las 4 h y 142 episodios sin aceptar ningún candidato.** Con un
tercio del objetivo clavado en un valor que ninguna ganancia mueve, el ajuste
no puede distinguir candidatos.

Parche: nuevo [`cinematica_pasiva.py`](../simulador_pasivo/cinematica_pasiva.py)
(`radio_minimo`, `pivote_para_radio`, `radio_local`, `factibilidad`,
`radio_seguro`). El banco ahora marca el veredicto de cada episodio, y **el
objetivo que se optimiza excluye las rutas imposibles** (se siguen midiendo y
reportando). Añadidos `curva_amplia` y `ese_amplia` con `radio_seguro()`
= R_min_útil × 1,25 ≈ 1,51 m, que deja autoridad para corregir en vez de ir
con el pivote al tope. **No se borró ningún escenario existente.**

## 2. Segundo fallo: el presupuesto de tiempo era fijo

`t_max = 10 s` para todas las rutas y `progreso = s_max/largo`. Una ruta más
larga puntuaba peor con el mismo control: **el banco penalizaba rediseñar la
ruta**. Se descubrió porque `ese_amplia` (factible) salía PEOR que `ese`
(imposible), solo por ser un 51 % más larga.

Peor aún: `ese` mide **5,34 m** y a la velocidad nominal del contrato
(0,5 m/s) necesita 10,7 s. **No había tiempo material para completarla ni a
tope de velocidad.** Parche: `presupuesto_s()` = 5 s/m, mínimo 10 s.

## 3. Tercer fallo: el episodio no terminaba al acabar la ruta

El bucle corría todos los pasos. Con la ruta hecha, `_proyecta` se satura en el
último tramo y el error crece mientras el robot se aleja: **el RMSE medía
vagabundeo posterior, no seguimiento**, y quedaba a merced del presupuesto.
Visible al hacer el presupuesto proporcional: `recta` pasó de 0,18 a 0,66 sin
que el control cambiase en nada.

Parche: corte cuando `s_max >= largo − FIN_RUTA_M` (5 cm). Efecto inmediato:

| escenario | presupuesto fijo | proporcional (sin corte) | proporcional + corte |
|---|---|---|---|
| `recta` | 0,18 | 0,66 | **0,20** |
| `suelo_flojo` | 0,34 | 1,00 | **0,14** |
| `paso_estrecho` | 5,52 | 7,32 | **5,53** |
| `obstaculo_bajo` | 3,15 | 2,02 | **1,82** |

## 4. Con el banco sano, el ajuste SÍ acepta un candidato

Misma rejilla de búsqueda que ya había (kp 2,5/4,0 · ki 1,0/2,5 · v 0,4/0,55),
así que la mejora viene de arreglar el banco, no de buscar más:

```
kp2,5 ki1,0 v0,40 -> 2,494      kp4,0 ki1,0 v0,40 -> 2,656
kp2,5 ki1,0 v0,55 -> 3,038      kp4,0 ki1,0 v0,55 -> 2,755
kp2,5 ki2,5 v0,40 -> 1,839      kp4,0 ki2,5 v0,40 -> 1,802  <-- mejor
kp2,5 ki2,5 v0,55 -> 2,554      kp4,0 ki2,5 v0,55 -> 2,501
```

**Aceptado: kp 4,0 · ki 2,5 · v 0,4** — train 1,802 → val 2,056 → test 1,754,
con split por semilla (train 0-11, val 12-17, test 18-23). Antes 2,345 con las
ganancias anteriores; naive 3,672. Nota: la rejilla prefiere `v=0,4` sobre
`0,55` en los cuatro pares, consistente con que el coste lo domina el desvío.

### TEST (6 semillas, mismas para ref y candidato)

| escenario | naive | candidato | pivote | veredicto |
|---|---|---|---|---|
| `recta` | 0,33 | **0,16** | 0,0° | ok |
| `curva` | 2,57 | 2,30 | 17,1° | al límite |
| `ese` | 3,15 | 2,97 | 20,3° | **imposible, fuera del objetivo** |
| `paso_estrecho` | 7,00 | **2,82** | 0,0° | ok |
| `suelo_flojo` | 0,26 | **0,13** | 0,0° | ok |
| `obstaculo_bajo` | 10,31 | **0,54** | 0,0° | ok |
| `curva_amplia` | 2,69 | 2,33 | 13,8° | ok |
| `ese_amplia` | 3,05 | 2,78 | 13,8° | ok |
| **global** | **3,672** | **1,754** | | |
| **solo factibles** | **3,746** | **1,580** | | |

**No comparable con el 2,564 de la corrida anterior:** el presupuesto y la
terminación cambiaron, así que la escala no es la misma. Lo comparable es
candidato vs naive dentro de esta corrida.

## 5. La holgura lateral era un número inventado, y ahora está medida

`geometria_robot.HOLGURA_LATERAL_M` valía 0,06 m puesto a ojo. El banco lo
desmiente: en el pasillo de 50 cm (6,5 cm de holgura, o sea **más** que el
umbral) el robot choca en el 88 % de los episodios.

Nuevo [`calibrar_holgura.py`](../simulador_pasivo/calibrar_holgura.py): barre el
ancho del pasillo, 8 semillas, y busca dónde desaparecen los choques.

| pasillo | holgura | % con choque (viejo) | % con choque (ajustado) |
|---|---|---|---|
| 45 cm | 4,0 cm | 88 % | 88 % |
| 49 cm | 6,0 cm | 88 % | 62 % |
| 53 cm | 8,0 cm | — | 38 % |
| 57 cm | 10,0 cm | 50 % | 12 % |
| 65 cm | 14,0 cm | 25 % | **0 %** |
| 75 cm | 19,0 cm | **0 %** | 0 % |

**Por qué el RMSE no basta para fijarla:** era 6,0 cm y *constante* a todos los
anchos. El que manda es el **PICO**: 15,8 cm de desvío lateral y 7,5° de rumbo
(que sobre un cuerpo de 35 cm añade ~2,3 cm). 15,8 + 2,3 ≈ 18 cm explica que
con el control viejo los choques solo desaparecieran a 19 cm.

Descartado como causa el barrido de la huella en giro: el semiancho es 18,5 cm
con el pivote a 0° **y** a 20° (manda la vía trasera; el módulo delantero es
más estrecho).

**La holgura es una propiedad del CONTROLADOR, no del chasis.** Al aceptar las
ganancias nuevas el RMSE cae 6,0 → 3,6 cm y la holgura necesaria 19 → **14 cm**
(pasillo de 65 cm). Como los senderos del operador miden 60-80 cm, **arreglar
el control es lo que los hace transitables**; relajar el umbral no lo habría
conseguido.

## 6. Configuración resultante

| clave | antes | ahora | por qué |
|---|---|---|---|
| `sendero_holgura_lateral_cm` | 6 (a ojo) | **14** | medido, con el control ajustado |
| `sendero_si_no_cabe` | `true`/`false` | **`"aviso"`** | decisión del operador: contar sin frenar |
| `sendero_parada_incierta_ciclos` | — | 5 | |
| ganancias del banco | kp3,5 ki2,0 v0,5 | **kp4,0 ki2,5 v0,4** | aceptado en train/val/test |

`sendero_si_no_cabe` pasó de booleano a tres modos: `"aviso"` cuenta cuántas
veces el sendero no admitiría la envolvente y **sigue** (contador
`avisos_estrechos` en el snapshot), `"frenar"` para, cualquier otro valor lo
apaga. Arranca en `"aviso"` porque los 14 cm salen del simulador, no del robot.

## 7. Verificación

| suite | resultado |
|---|---|
| backend (`jetson/backend/tests/`) | **86 pasan**, 1 falla previa en `test_enlace_c3` (otro módulo) |
| simulador (cinemática, presupuesto, evaluación, pasivo, rústico) | **39 pasan** |

Nuevos: `test_cinematica_pasiva.py` (11), `test_presupuesto_banco.py` (9),
`test_corredor_seguro.py` (12).

Respaldo: `evaluacion.py.bak-20261014_cinematica`.

## 8. Lo que sigue sin demostrarse

- **Nada de esto se midió en el robot.** Son resultados de simulador con el
  contrato sin calibrar. La tabla de éxito de recorrido, contactos reales,
  invasión de plantas y latencia sigue vacía.
- `curva` (17,1°) se queda **al límite** del pivote útil: se puede seguir, pero
  con la dirección al tope y sin margen para corregir. Las rutas de campo
  deberían planificarse sobre `radio_seguro()` ≈ 1,51 m, no sobre R_min.
- El pico de desvío (15,8 cm con el control viejo) **no se ha medido en el
  robot**; es lo que decide la holgura, así que medirlo es prioritario antes de
  pasar `sendero_si_no_cabe` a `"frenar"`.
- Sigue pendiente todo lo del informe anterior: calibración física del servo,
  biases del giróscopo, barrido de la huella en giro, preprocesado de la UNet,
  limpieza de los 9.684 registros y el reinicio de Nav2 con el footprint nuevo.
