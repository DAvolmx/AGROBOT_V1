# Conclusión: ¿cuál es la mejor estrategia?

5 semillas nuevas × 10 misiones × 4 estrategias (1436 s).

| estrategia | éxito % | llega % | vuelve % | VUELVE SIN CHOQUE % | sin choque % | contactos/ep | cobertura escaneo | recall | tiempo (s) |
|---|---|---|---|---|---|---|---|---|---|
| E11 E10 + repetir escaneo fallido | 6.0 | 68.0 | 62.0 | 38.0 | 44.0 | 5.32 | 0.718 | 0.588 | 1137.7 |
| E10 A* + visitar y parar a ~0,8 m | 4.0 | 66.0 | 60.0 | 34.0 | 44.0 | 5.42 | 0.519 | 0.585 | 1147.096 |
| E6 A* ida y vuelta (referencia) | 0.0 | 70.0 | 64.0 | 58.0 | 62.0 | 5.32 | 0.315 | 0.471 | 1018.628 |
| E9 A* + escanear pasando (<= 0,8 m) | 0.0 | 68.0 | 64.0 | 54.0 | 58.0 | 5.28 | 0.378 | 0.486 | 1054.23 |

**Mejor en esta medición: E11 E10 + repetir escaneo fallido.**
