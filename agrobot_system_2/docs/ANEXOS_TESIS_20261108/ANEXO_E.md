# Anexo E. Fichas de ensayos interiores y exteriores

Este anexo relaciona cada resultado físico de la tesis con el ensayo que lo produjo. Se compiló el 8 de noviembre de 2026 a partir de los registros del robot y de los informes de cada sesión.

**Cómo leer las fichas.** Los criterios de aceptación de las pruebas P1 a P12 (sección 3.19) se definieron después de estos ensayos. Por eso se aplican aquí de forma retrospectiva y solo como referencia: ninguno de estos ensayos fue diseñado para demostrar esos criterios. «No evaluable» significa que el ensayo no midió lo que el criterio exige.

**Limitaciones comunes.**

- El clima, la temperatura y la humedad del suelo no se registraron en ninguna sesión.
- No hay sensor de contacto: «sin colisiones registradas» no demuestra que no las hubiera.
- La Jetson no tiene RTC, y varias sesiones tienen saltos del reloj de pared. Las horas son orientativas y los intervalos fiables se calculan con tiempo monótono.
- Desde el 23 de octubre de 2026, CO₂, temperatura y humedad son **datos simulados** porque el SCD40 está dañado. Ningún resultado de estas fichas usa esas variables.
- La columna de versión indica el último commit anterior al ensayo. El árbol de trabajo podía tener cambios sin confirmar, de modo que esa versión es aproximada.

## E.1 Trazabilidad de resultados

| Resultado en la tesis | Tabla o sección | Ensayo |
|---|---|---|
| Cierre de bucle SLAM interior | Tabla 56; 3.12.1 | I-1 |
| Detección de macetas 20/20 y 0/30 | Tabla 57 | I-2 |
| Giro de lazo cerrado y corte a 158° | Tabla 56; 3.12.1 | I-3 |
| Especie por CLIP 17/17; 4/4 plantas por pasada | Tabla 57 | I-3 |
| Avisos de plaga sobre puertas y armarios | Tabla 57 | I-4 |
| Salto de reloj y saltos de pose | Tabla 59; 3.13.1 | E-1 |
| Avisos sobre suculentas | Tabla 60 | E-1 |
| 16,2 min de autonomía; 26 % hacia atrás | Tabla 59; 3.13.2 | E-2 |
| 261 avisos y 360 falsas macetas | Tabla 60 | E-2 |
| Giro en tierra suelta; 95 empujes del operador | Tabla 59; 3.13.1 | E-3 |
| 0 falsas plagas de PLAGASX v8 en campo | Tabla 60; 3.13.1 | E-3 |
| Enlace LoRaWAN en AU915 | Parte A, 1.3.15.6; P8 | L-1 |

## E.2 Ficha I-1. SLAM con regreso a la marca

| Campo | Valor |
|---|---|
| Fecha | 22-oct-2026 |
| Entorno | Interior, sala |
| Condiciones | 23 m con muchos giros, 3 min, conducido con el mando |
| Configuración | slam_toolbox con filtro de caja del chasis; versión de referencia `706eb6a` |
| Registros originales | `~/.agrobot/campo/20261022_213805_5dde93/` (868 muestras a 5 Hz, sin pérdidas); reproducción en `simular_slam.py` |
| Resultado | Cierre de bucle de 36 cm y −67° en vivo (≈1,5 % del recorrido); 23–24 cm reproducido en simulación |
| Criterio de referencia | P4: error de cierre ≤ 2 % del recorrido, medido con cinta |
| Dictamen | **No evaluable como P4:** el valor es menor que 2 %, pero el error sale del propio SLAM, sin referencia externa, en un solo recorrido |

## E.3 Ficha I-2. Demostración de plantas en maceta

| Campo | Valor |
|---|---|
| Fecha | 23-oct-2026 |
| Entorno | Interior, laboratorio con mesas, sillas y una coliflor en maceta |
| Condiciones | Presentación con público; iluminación no registrada |
| Configuración | Misión de plantas en maceta; detector de macetas; versión de referencia `5c98f64` |
| Registros originales | **No identificados con certeza.** Hay sesiones de ese día en `~/.agrobot/datos/20261023_*`, sin etiqueta que indique cuál es la demostración |
| Resultado | 20/20 cuadros con maceta detectada; 0/30 cuadros sin maceta con falsa detección |
| Criterio de referencia | Ninguno definido |
| Dictamen | **Sin criterio.** Antes de citar 20/20 y 0/30 como resultado, conviene identificar la sesión original |

## E.4 Ficha I-3. Escenario en L

| Campo | Valor |
|---|---|
| Fecha | 28-oct-2026 (etiquetado a las 01:51 UTC) |
| Entorno | Interior según la tesis; la etiqueta original dice «por confirmar». La configuración guardada estaba en modo `interior` |
| Condiciones | L de 2,70 × 2,40 m; pasillo de 0,70 m el primer metro y 0,90 m el resto; 4 plantas reales (1 coliflor); 1 093 s monitorizados |
| Configuración | Misión ×4, giro de lazo cerrado en tramos de 20°; `config_snapshot.json`; versión de referencia `4c591f0` |
| Registros originales | `~/.agrobot/pruebas/L_20261028/` (`RESUMEN.md`, `monitor.jsonl`, `giro_360.json`, `plantas_consolidadas.json`, fotos) |
| Resultado de percepción | Especie 17/17 (coliflor p_col 0,933–0,996; otras ≤ 0,374); 6, 3, 4 y 4 escaneos por pasada |
| Resultado de navegación | Giros de 15–22° por tramo; el giro de 360° se detuvo a 158° por falta de espacio; 12 actuaciones de anticolisión; distancia mínima 5 cm |
| Criterio de referencia | P3: error mediano ≤ 6° y ≥ 90 % de giros alcanzados |
| Dictamen | **No evaluable como P3:** sin medida externa de ángulo ni 20 repeticiones. La misión escaneó exactamente las 4 plantas en 2 de 4 pasadas |

## E.5 Ficha I-4. Escenario en L, segunda sesión

| Campo | Valor |
|---|---|
| Fecha | 28-oct-2026 |
| Entorno | Interior; mismo escenario que I-3, con armarios y puertas |
| Configuración | Revisión de avisos de plaga; corrección en el commit `4c591f0` |
| Registros originales | `~/.agrobot/pruebas/L2_20261028/` (`monitor.jsonl`, `movimientos_operador.jsonl`, `fotos_aviso/`) |
| Resultado | 16–17 de 18 avisos eran puertas, cajas o pared sin maceta en el cuadro |
| Criterio de referencia | P6: ≤ 5 % de falsas |
| Dictamen | **No cumple** con el modelo de entonces. Motivó descartar el aviso cuando no hay maceta en el cuadro |

## E.6 Ficha E-1. Patio con macetas

| Campo | Valor |
|---|---|
| Fecha | 27-oct-2026, alrededor de las 21:55 UTC |
| Entorno | Exterior: patio o jardín junto a un muro, con macetas de suculentas y ornamentales, y tramo cubierto |
| Condiciones | Diurna; clima no registrado |
| Configuración | Misión de plantas, giro de lazo cerrado, pulsos de 6°; versión de referencia `325a244` |
| Registros originales | `~/.agrobot/datos/20261027_215547/`, grabaciones `campo/20261027_215828_25f568` y `221926_28a5d4`; análisis en `docs/PRUEBA_EXTERIOR_20261027/metricas.json` |
| Resultado | Misión de 664 s; 5 plantas inspeccionadas y 7 inspecciones fallidas (6 por orden manual); 41,9 m por SLAM sin los 3 saltos; giro mediano de 1,5° frente a 6° pedidos; salto de reloj de +711,5 s con saltos de pose de 30,1 y 24,4 m |
| Criterio de referencia | P3 y P1 |
| Dictamen | **No cumple** el objetivo de giro (1,5° frente a 6° pedidos, por giroscopio) y **no evaluable** como P1. Motivó el vigía de pose |

## E.7 Ficha E-2. Huerta con coles, sendero corto

| Campo | Valor |
|---|---|
| Fecha | 28-oct-2026, 19:45–20:26 UTC según el registro (el reloj saltó +5,4 h a los 100 s) |
| Entorno | Exterior: huerta con coles sembradas, sin macetas |
| Condiciones | Diurna; sendero corto; clima no registrado |
| Configuración | Exploración, perfil exterior, cámara a 98°, asistencia activa casi todo el tiempo; versión de referencia `4c591f0` |
| Registros originales | `~/.agrobot/datos/20261028_141932/`; grabaciones en `pruebas/EXT_20261028/campo/`; etiqueta en `pruebas/EXT2_20261028/`; informe en `docs/PRUEBA_EXTERIOR2_20261028/INFORME.md` |
| Resultado | Sesión de 54 min; 972 s (30 %) en autonomía, en 45 tramos; 24,8 m adelante y 8,8 m atrás; 261 avisos de plaga falsos y 360 falsas macetas; coles vistas en 1 de 30 cuadros del primer tramo |
| Criterio de referencia | P1: ≥ 3 de 5 recorridos completos y ≤ 1 intervención por 10 m |
| Dictamen | **No cumple en lo observado:** hubo mando manual la mayor parte del tiempo. No se contaron intervenciones por 10 m |

## E.8 Ficha E-3. Huerta, sendero de tierra removida

| Campo | Valor |
|---|---|
| Fecha | 29-oct-2026, desde las 14:29 UTC (saltos de reloj de +573 s y +159 s) |
| Entorno | Exterior: sendero de tierra removida con coles, maíz y arbustos |
| Condiciones | Diurna; clima no registrado |
| Configuración | Perfil exterior, PLAGASX v8, filtros del sendero, `GIRO_IZQ_FACTOR` = 2,3; versión de referencia `0608377` |
| Registros originales | `~/.agrobot/campo/20261029_142951_a81509/` (video de 781 s, 1 359 fotos, 4 054 muestras a 5 Hz); `aprendizaje/intervenciones/2026-10-29.jsonl`; `aprendizaje/correcciones/guardian_20261029.jsonl`; informe en `docs/SESION_CAMPO_20261029/INFORME.md` |
| Resultado de navegación | ~3 m en x y 4,5 m en y; 77 % de las muestras parado; 1 525° de giro acumulado; 154 intervenciones del operador, 95 empujando hacia adelante; pulsos de 0,4–1 s giran 13–23° y los de 2–4,6 s, 0–5° |
| Resultado de percepción | 0 clases de plaga falsas de PLAGASX en 14 min, con 73 detecciones únicas |
| Criterio de referencia | P1 y P6 |
| Dictamen | **P1 no cumple** (recorrido corto y muchas intervenciones). **P6 no evaluable:** hay pocas falsas, pero la exhaustividad no se midió |

## E.9 Ficha L-1. Enlace LoRaWAN en AU915

| Campo | Valor |
|---|---|
| Fecha | 3-nov-2026 |
| Entorno | Banco de pruebas, nodo cerca del gateway |
| Configuración | Heltec V3 (`4f08332`), HT-M02 y ChirpStack `au915_0`; canales 0–7 y 64 |
| Registros originales | Base de datos de ChirpStack y `agrobot_history.db` en la Raspberry Pi; resumen en `MANIFEST_1.md`, entrada del 2026-11-03. No se archivó una exportación del ensayo |
| Resultado | Unión OTAA correcta; uplinks fCnt 5 y 6 en 916,0 y 915,2 MHz, SF7, RSSI −29 y −31 dBm. Los sensores no responden: la carga útil es cero |
| Criterio de referencia | P8: 100 % de joins; ≥ 95 % de 100 uplinks; todas las subidas entre 915,2 y 916,6 MHz |
| Dictamen | **Parcial:** verifica unión y banda; solo se comprobaron 2 uplinks, no 100 |

## E.10 Integridad de los registros originales

Los registros pesan varios gigabytes y contienen fotos del entorno del operador, por lo que no se publican en el repositorio. Permanecen en la Jetson. La huella SHA-256 permite comprobar que un archivo presentado es el mismo que se analizó.

| Ensayo | Archivo (bajo `~/.agrobot/`) | Tamaño | SHA-256 (primeros 16) |
|---|---|---|---|
| I-1 | `campo/20261022_213805_5dde93/manifest.json` | 70.4 kB | `24d96c60306ef374` |
| I-1 | `campo/20261022_213805_5dde93/summary.json` | 0.5 kB | `692d993dcb1c1b1e` |
| I-3 | `pruebas/L_20261028/RESUMEN.md` | 2.8 kB | `f3b75dee86bc6689` |
| I-3 | `pruebas/L_20261028/monitor.jsonl` | 59.3 kB | `f3b0f6562a9c4516` |
| I-3 | `pruebas/L_20261028/config_snapshot.json` | 3.7 kB | `297cd102dc839630` |
| I-3 | `pruebas/L_20261028/giro_360.json` | 7.8 kB | `5f251709fce28a7e` |
| I-3 | `pruebas/L_20261028/ETIQUETA.json` | 0.3 kB | `674045d017edf2a7` |
| I-4 | `pruebas/L2_20261028/monitor.jsonl` | 44.2 kB | `fcee6d1d495b0751` |
| I-4 | `pruebas/L2_20261028/movimientos_operador.jsonl` | 0.6 kB | `12cb0c713fd85df0` |
| E-1 | `datos/20261027_215547/ETIQUETA.json` | 0.6 kB | `04a4979c86d98663` |
| E-1 | `datos/20261027_215547/eventos.jsonl` | 28.3 MB | `e08fefd24ae62a8f` |
| E-1 | `campo/20261027_215828_25f568/manifest.json` | 65.3 kB | `9ea26a8fd0fcb763` |
| E-1 | `campo/20261027_221926_28a5d4/manifest.json` | 65.5 kB | `dec423a288d3d9e1` |
| E-2 | `pruebas/EXT2_20261028/ETIQUETA.json` | 0.4 kB | `becf2e5dde046880` |
| E-2 | `datos/20261028_141932/eventos.jsonl` | 62.0 MB | `0537c02746c8c739` |
| E-2 | `pruebas/EXT_20261028/campo/20261028_194540_ed7b3b.zip` | 455.2 MB | `710bbd18f9cdaf5b` |
| E-2 | `pruebas/EXT_20261028/campo/20261028_195430_e352b3.zip` | 357.9 MB | `2be76401fe401be7` |
| E-2 | `pruebas/EXT_20261028/campo/20261028_200037_32de40.zip` | 407.2 MB | `c4ead9d17d426e4e` |
| E-2 | `pruebas/EXT_20261028/campo/20261028_200833_d17537.zip` | 956.2 MB | `5c9d32880a0f9a1c` |
| E-3 | `campo/20261029_142951_a81509/manifest.json` | 57.2 kB | `65b8d9dd32e0deb9` |
| E-3 | `campo/20261029_142951_a81509/summary.json` | 0.5 kB | `345fe765356834a8` |
| E-3 | `campo/20261029_142951_a81509/frames.jsonl` | 1.4 MB | `57e1dd911c4a2379` |
| E-3 | `campo/20261029_142951_a81509/variables.jsonl` | 121.2 MB | `5bb0610a352cde84` |
| E-3 | `campo/20261029_142951_a81509/video_1793284192317064097.avi` | 600.1 MB | `0b22a079117d5e32` |
| E-3 | `aprendizaje/intervenciones/2026-10-29.jsonl` | 61.8 kB | `4c1df131eb636933` |
| E-3 | `aprendizaje/correcciones/guardian_20261029.jsonl` | 25.1 kB | `929c55fd74999331` |

## E.11 Ensayos pendientes

Las pruebas P1 a P12 de la sección 3.19 siguen sin ejecutarse según su protocolo. Cada una deberá registrarse con esta misma ficha, definiendo el criterio antes del ensayo.
