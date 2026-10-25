# Conclusión: ¿cuál es la mejor estrategia?

5 semillas nuevas × 10 misiones × 8 estrategias (1775 s).

| estrategia | éxito % | llega % | vuelve % | VUELVE SIN CHOQUE % | sin choque % | contactos/ep | cobertura escaneo | recall | tiempo (s) |
|---|---|---|---|---|---|---|---|---|---|
| E6 A* solo ida y vuelta (sin escanear) | 0.0 | 70.0 | 64.0 | 58.0 | 62.0 | 5.32 | 0.315 | 0.471 | 1018.628 |
| E3 hueco + predictor | 0.0 | 52.0 | 44.0 | 42.0 | 96.0 | 0.04 | 0.406 | 0.676 | 701.804 |
| E8 híbrido: hueco+predictor suave, A* al volver | 0.0 | 58.0 | 52.0 | 40.0 | 76.0 | 0.94 | 0.465 | 0.735 | 789.34 |
| E7 hueco + predictor suave | 0.0 | 66.0 | 42.0 | 40.0 | 86.0 | 1.2 | 0.469 | 0.754 | 908.476 |
| E2 seguir el hueco (VFH) | 0.0 | 90.0 | 72.0 | 32.0 | 34.0 | 7.1 | 0.5 | 0.855 | 977.34 |
| E5 hueco + predictor + migas | 0.0 | 46.0 | 28.0 | 24.0 | 90.0 | 0.1 | 0.404 | 0.65 | 777.616 |
| E4 planificador A* + predictor | 0.0 | 66.0 | 58.0 | 20.0 | 24.0 | 4.68 | 0.457 | 0.585 | 1080.102 |
| E1 reactiva clásica | 0.0 | 44.0 | 22.0 | 18.0 | 54.0 | 5.3 | 0.474 | 0.607 | 1426.628 |

**Mejor en esta medición: E6 A* solo ida y vuelta (sin escanear).**

## Lectura y conclusión (2026-10-25)

Qué se midió: 8 estrategias × 10 misiones × 5 semillas nuevas, mismos parámetros de bajo nivel, físico calibrado con paredes sólidas. «Vuelve sin choque» = recorre todo el camino, regresa al inicio y no toca nada (la prioridad que pediste). El **éxito completo** (además ≥90 % de las plantas vistas escaneadas y recall ≥0,8) fue **0 % en las 8**.

**La mejor para recorrer el camino y volver sin chocar: E6, planificador A\* (ida y vuelta, sin visitar plantas): 58 % vuelve sin choque.**
- Resuelve al 100 % la huerta larga, el campo extenso, el camino curvo y el laboratorio con macetas; 80 % salas con puerta y pasillo en L.
- El resto de las estrategias queda en 18-42 %. La reactiva de hoy, 18 %.

**Por qué ninguna cumple la misión completa**
1. **Visitar plantas rompe a la mejor estrategia:** E4 es el mismo A\* pero acercándose a cada planta y baja de 58 % a 20 % sin choque (4,7 contactos/ep). Acercarse a plantas es ir hacia un obstáculo con la física de pulsos. Con A\* sin visitar, la cobertura de escaneo es solo 0,32.
2. **Seguir el hueco (E2) es la que más llega (90 %) y más ve (recall 0,86), pero choca 7,1 veces por episodio** (solo 32 % de las veces vuelve sin chocar).
3. **El predictor de colisión elimina los choques** (E3: 0,04 contactos/ep, 96 % de episodios sin choque) pero el robot se queda a medio camino (52 % llega). Un predictor más suave (E7, E8) deja pasar choques (0,9-1,2/ep) sin ganar mucho.
4. **Las migas (enseñar y repetir) no ayudan** (E5: 28 % vuelve): seguir el rastro en pulsos arrastra los errores de la ida.
5. **Siguen sin resolverse con ninguna:** el laberinto de macetas, el laberinto grande y el campo denso (0 % en todas las estrategias); el laboratorio con sillas llega a 40 % como mucho.

**Recomendación**
- Para llevar al robot real: **planificador A\* para el tránsito (ida y regreso) + predictor de colisión como seguro**, que es la combinación que más veces completa el recorrido sin chocar.
- El escaneo de cada planta hay que resolverlo **sin acercarse con el planificador**: parar a la distancia de seguridad (≈0,9 m) cuando la planta entra en la cámara y escanear desde ahí, o hacerlo en una segunda pasada controlada. Esto no está probado y es el siguiente experimento.
- Los laberintos con macetas contra las paredes necesitan otra idea (los pasillos de 1,1-1,2 m con macetas dejan ~0,5 m: el robot de 38 cm apenas cabe y su giro por pulsos no lo permite en las esquinas).

**Límites de esta conclusión:** es simulación; la cámara, el detector y el conteo son hipótesis del simulador; la búsqueda en línea de literatura no estuvo disponible (la elección de estrategias sale de conocimiento general de navegación con LIDAR 2D, sin verificar); 5 semillas por misión dan resultados con escalones de 20 % por misión.
