# Conclusión: ¿cuál es la mejor estrategia?

3 semillas nuevas × 10 misiones × 10 estrategias (1024 s).

| estrategia | éxito % | llega % | vuelve % | VUELVE SIN CHOQUE % | sin choque % | contactos/ep | cobertura escaneo | recall | tiempo (s) |
|---|---|---|---|---|---|---|---|---|---|
| D00 hueco + predictor (referencia) | 0.0 | 56.7 | 36.7 | 30.0 | 83.3 | 0.233 | 0.568 | 0.74 | 1107.503 |
| D01 sonar+lados, máx 3 pulsos | 0.0 | 23.3 | 16.7 | 16.7 | 20.0 | 7.033 | 0.249 | 0.327 | 392.387 |
| D04 máx 3 + atrás antes de giros > 60° | 0.0 | 20.0 | 16.7 | 13.3 | 26.7 | 7.133 | 0.257 | 0.328 | 487.143 |
| D09 D04 giro máx 90° | 0.0 | 20.0 | 16.7 | 13.3 | 26.7 | 7.133 | 0.257 | 0.328 | 487.143 |
| D08 D04 sin centrado ni eje | 0.0 | 23.3 | 13.3 | 13.3 | 30.0 | 5.833 | 0.269 | 0.34 | 518.457 |
| D05 máx 3 + atrás antes de giros > 40° | 0.0 | 16.7 | 13.3 | 13.3 | 30.0 | 6.3 | 0.265 | 0.295 | 489.227 |
| D07 D04 solo scan (sin mapa) | 0.0 | 23.3 | 13.3 | 13.3 | 23.3 | 6.767 | 0.24 | 0.341 | 439.47 |
| D06 D04 con profundidad 1,4 m | 0.0 | 16.7 | 13.3 | 13.3 | 20.0 | 7.633 | 0.247 | 0.302 | 438.827 |
| D02 sonar+lados, máx 2 pulsos | 0.0 | 16.7 | 13.3 | 13.3 | 13.3 | 7.967 | 0.235 | 0.337 | 395.36 |
| D03 sonar+lados, máx 5 pulsos | 0.0 | 20.0 | 10.0 | 10.0 | 26.7 | 6.7 | 0.285 | 0.294 | 374.827 |

**Mejor en esta medición: D00 hueco + predictor (referencia).**
