# Corrección de dinámica y evaluación del seguimiento de sendero

La dinámica del simulador ya mantiene el pivote delantero en el punto fijado al chasis, a 0.37 m del eje trasero. Antes, la posición del pivote dependía del ángulo del eje delantero; al girar ±20°, el modelo trasladaba lateralmente el punto de giro como si estuviera montado sobre el eje trasero. Eso alteraba las fuerzas de contacto y la trayectoria.

Las dimensiones usadas siguen siendo las confirmadas: batalla de 0.37 m, ancho exterior delantero de 0.19 m, ancho exterior trasero de 0.37 m y pivote compartido limitado a ±20°. Las masas, adherencia, inercias, ganancia de motor y fricción del pivote siguen marcadas como hipótesis hasta medirlas en el robot.

Se añadió un seguidor pure pursuit con progreso de arco persistente y proyección local. La ventana de búsqueda impide que el estimador salte a una fila vecina cuando dos partes del sendero pasan cerca; el control adapta velocidad a la curvatura y limita la aceleración lateral estimada. La evaluación usa pose SLAM ruidosa para ordenar y verdad simulada solo para puntuar.

## Validación

- 15 pruebas de dinámica y seguimiento pasan en Jetson y en el entorno local.
- La prueba de diferencias finitas verifica que el Jacobiano de los contactos delanteros coincide con la geometría del pivote.
- Evaluación corta reproducible: 12 s por episodio, huerta con camino de 50 cm, cinco candidatos, dos semillas de ajuste, dos de validación y tres de prueba.
- La selección conservó el control base (0.20 m/s, mirada 0.35 m, ganancia 1.2): ningún candidato mejoró el costo de validación. Esto evita instalar un ajuste más agresivo sin evidencia.
- En las tres semillas de prueba, el RMSE de seguimiento fue 1.7, 1.4 y 7.1 cm; el percentil 95 fue 4.6, 2.0 y 12.6 cm; hubo cero colisiones y cero contactos con cultivo. Son episodios breves del escenario simulado, no una garantía de desempeño real.

La política de navegación de larga duración permanece en entrenamiento aparte. En la tanda 6 obtuvo 76.0% de acierto global y 74.5% equilibrado, pero empeoró el desvío de 77.8 a 82.1 cm en su evaluación reciente y tuvo 29.17 choques por 10 m. No se acepta esa política para conducir hasta que reduzca desvío y choques en validación independiente.

## Límites de transferencia

Esta prueba entrega una ruta desde el mapa del mundo; no evalúa la segmentación de cámara ni el SLAM completo de Gazebo. La fricción de neumático, transferencia de carga, suspensión, torque real, posición del centro de masa y respuesta mecánica del pivote requieren identificación experimental. El resultado sirve para depurar geometría y control de simulación, no certifica seguridad del chasis.

Archivos: `dinamica_pasiva.py`, `seguimiento_sendero.py`, `mini_entrenamiento.py` y sus pruebas. Resultados en Jetson: `~/.agrobot/aprendizaje/eval_seguidor_v2/`.
