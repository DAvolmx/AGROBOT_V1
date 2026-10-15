# AgroBot · procedencia de los parámetros (medido / estimado / hipótesis)

Actualizado 2026-10-10. Objetivo de la directiva: **no dar por medidas** la masa,
el CG, la inercia, los neumáticos, la adherencia, el pivote ni la distribución de
motores. Aquí se distingue qué es qué. La física del simulador vive en
`dinamica_pasiva.py` / `fallos_3d.py`; la calibración de cámara en
`../jetson/backend/calibracion_camara.py`.

Leyenda: **MEDIDO** = con cinta/instrumento. **ESTIMADO** = a ojo por el operador
(fecha). **HIPÓTESIS** = valor por defecto configurable, sin evidencia.

## Geometría — MEDIDO (operador, confirmado)
| Parámetro | Valor | Nota |
|---|---|---|
| Ancho exterior delantero | 0.19 m | las 2 ruedas delanteras giran como un módulo |
| Ancho exterior trasero | 0.37 m | |
| Distancia entre ejes (`wheelbase`) | 0.37 m | |
| Pivote delantero | libre, pasivo, ±20° | sin servo de dirección; sin autoalineamiento |
| Servo existente | inclina la cámara (tilt), NO dirección | |
| Ancho del sendero | ~0.50 m | aproximado |

⚠️ Los anchos exteriores **no** son distancia entre centros de rueda (hay que
restar el ancho de neumático). **Largo total: NO confirmado** — no reutilizar 33×50.

## Masa y reparto — ESTIMADO (operador, 2026-10-09)
| Parámetro | Valor usado | Estado |
|---|---|---|
| Masa total | ~12 kg | ESTIMADO (a ojo, "12 kg o más") |
| Altura del CG (`cg_height`) | 0.20 m | ESTIMADO ("alto, 18-22 cm") — **parámetro nº1 del vuelco** |
| Motores: delanteros más fuertes | `front_drive_bias=2.0` | ESTIMADO (dirección), magnitud HIPÓTESIS |
| Obstáculo del vuelco del 2026-10-09 | ~0.05 m | ESTIMADO |

## Resto de la física — HIPÓTESIS (defaults sin medir)
`tire_width=0.047`, `radius=0.086`, `trail=0.0` (desplazamiento del pivote),
`chassis_inertia/front_inertia`, `mu=0.6-0.65` (adherencia), rigideces de
deslizamiento `longitudinal/lateral`, `pivot_damping/pivot_friction`,
`motor_gain/max_torque`, reparto chasis/módulo de la masa, `susp_recoil`
(recoil del amortiguador), `wheel_inertia`, `substep`.
Las imperfecciones del chasis rústico (`toe_ruedas`, `gain_ruedas`) están
AISLADAS en `robot_rustico.py` (default IDEAL en los archivos oficiales).

## Calibración de cámara — HIPÓTESIS, pendiente de verificación física
`calibracion_camara.py` (`VERIFICADO=False`): `inclinacion_abajo = 15.5 + 0.8·(98−servo)`.
Resuelto el 2026-10-10 el conflicto de signo (sim y backend usaban sentidos
opuestos); ahora comparten esta fórmula. **PENDIENTE medir** el signo real, la
ganancia 0,8°/° y los 15,5° de HOME.

## Pendiente de medir para transferir al robot real
1. **Masa** (báscula) y **reparto** delantero/trasero.
2. **Altura del CG** (método de inclinar/balancear hasta despegue de rueda).
3. **Radio de rueda** y **ancho de neumático** (calibre) → vía entre centros.
4. **Desplazamiento del pivote** (`trail`) respecto al eje delantero.
5. **Par de cada motor** y relación delantero/trasero real.
6. **Adherencia** (`mu`) en el terreno real (prueba de arrastre).
7. **Cámara**: cabeceo real a servo 25/98/110 (nivel o IMU) → signo + ganancia.
8. **Largo total** del robot (extremo a extremo de ruedas).

Mientras no se midan, las métricas del simulador son **comparativas entre
versiones bajo las mismas hipótesis**, no predicciones absolutas del robot real.
