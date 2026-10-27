# Conclusión: ¿cuál es la mejor estrategia?

5 semillas nuevas × 10 misiones × 8 estrategias (2503 s).

| estrategia | éxito % | llega % | vuelve % | VUELVE SIN CHOQUE % | sin choque % | contactos/ep | cobertura escaneo | recall | tiempo (s) |
|---|---|---|---|---|---|---|---|---|---|
| G0 hueco + predictor (referencia) | 0.0 | 68.0 | 44.0 | 36.0 | 68.0 | 1.3 | 0.561 | 0.816 | 1267.062 |
| G7 CONTROL: disparo rápido, escape viejo (15 s, 4) | 0.0 | 54.0 | 36.0 | 32.0 | 62.0 | 1.7 | 0.531 | 0.779 | 1197.542 |
| G1 + escape con sonar (25 s, 8 bloqueos) | 0.0 | 64.0 | 34.0 | 30.0 | 60.0 | 2.14 | 0.559 | 0.777 | 1388.68 |
| G2 + escape con sonar (15 s, 4 bloqueos) | 0.0 | 60.0 | 30.0 | 28.0 | 62.0 | 1.5 | 0.553 | 0.762 | 1322.766 |
| G6 G2 + memoria de visitas | 0.0 | 60.0 | 30.0 | 28.0 | 62.0 | 1.5 | 0.553 | 0.762 | 1322.766 |
| G4 G2 con máx 6 pulsos de giro | 0.0 | 66.0 | 30.0 | 28.0 | 64.0 | 2.22 | 0.54 | 0.782 | 1298.05 |
| G3 + escape con sonar (10 s, 3 bloqueos) | 0.0 | 60.0 | 28.0 | 28.0 | 62.0 | 1.72 | 0.558 | 0.779 | 1271.608 |
| G5 G2 sonar solo scan (sin mapa) | 0.0 | 60.0 | 26.0 | 24.0 | 68.0 | 1.6 | 0.566 | 0.763 | 1381.658 |

**Mejor en esta medición: G0 hueco + predictor (referencia).**
