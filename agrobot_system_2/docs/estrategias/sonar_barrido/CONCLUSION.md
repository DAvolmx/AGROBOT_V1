# Conclusión: ¿cuál es la mejor estrategia?

3 semillas nuevas × 10 misiones × 15 estrategias (1089 s).

| estrategia | éxito % | llega % | vuelve % | VUELVE SIN CHOQUE % | sin choque % | contactos/ep | cobertura escaneo | recall | tiempo (s) |
|---|---|---|---|---|---|---|---|---|---|
| C00 hueco + predictor (referencia) | 0.0 | 56.7 | 36.7 | 30.0 | 83.3 | 0.233 | 0.568 | 0.74 | 1107.503 |
| C14 hueco + frente barrido + centrado | 0.0 | 23.3 | 10.0 | 10.0 | 16.7 | 5.867 | 0.217 | 0.314 | 337.233 |
| C04 + centrado por lados | 0.0 | 23.3 | 10.0 | 10.0 | 16.7 | 5.9 | 0.217 | 0.314 | 326.71 |
| C08 C05 profundidad 0,8 m | 0.0 | 13.3 | 10.0 | 10.0 | 16.7 | 6.033 | 0.244 | 0.309 | 249.437 |
| C01 sonar base (cono) | 0.0 | 16.7 | 10.0 | 10.0 | 13.3 | 6.033 | 0.259 | 0.305 | 269.76 |
| C02 sonar + frente barrido | 0.0 | 16.7 | 10.0 | 10.0 | 13.3 | 6.033 | 0.259 | 0.305 | 269.76 |
| C03 + castigo a girar hacia pared | 0.0 | 16.7 | 10.0 | 10.0 | 10.0 | 6.067 | 0.259 | 0.305 | 226.037 |
| C09 C05 36 rayos | 0.0 | 16.7 | 10.0 | 10.0 | 16.7 | 6.133 | 0.244 | 0.29 | 228.507 |
| C10 C05 144 rayos | 0.0 | 20.0 | 10.0 | 10.0 | 16.7 | 6.167 | 0.202 | 0.315 | 293.283 |
| C11 C05 solo scan (sin mapa) | 0.0 | 20.0 | 10.0 | 10.0 | 13.3 | 6.2 | 0.227 | 0.305 | 321.997 |
| C07 C05 profundidad 1,4 m | 0.0 | 20.0 | 10.0 | 10.0 | 10.0 | 6.4 | 0.236 | 0.347 | 399.9 |
| C06 C05 giro máx 90° | 0.0 | 16.7 | 10.0 | 10.0 | 16.7 | 6.6 | 0.288 | 0.286 | 295.383 |
| C05 + eje del pasillo | 0.0 | 20.0 | 10.0 | 10.0 | 13.3 | 6.7 | 0.281 | 0.293 | 294.847 |
| C13 C05 meta pesa 2 | 0.0 | 20.0 | 10.0 | 10.0 | 13.3 | 6.7 | 0.281 | 0.293 | 294.847 |
| C12 C05 sin memoria de visitas | 0.0 | 20.0 | 10.0 | 10.0 | 13.3 | 6.867 | 0.27 | 0.298 | 316.933 |

**Mejor en esta medición: C00 hueco + predictor (referencia).**
