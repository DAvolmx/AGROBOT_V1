# Navegación y filtro de sendero (2026-10-11)

## Cambios activos en el simulador

- El modo automático arranca al crear el episodio y el escenario guardado `huerta_fotos_50cm` ya puede reiniciarse desde el panel.
- El planificador fusiona la distancia de obstáculo bajo medida por cámara con el frente del LIDAR. Convierte la distancia desde el centro del chasis a espacio libre delante; así deja de ordenar correcciones mientras el freno de suelo cancela el movimiento. El retroceso sigue condicionado por el despeje trasero del LIDAR.
- El detector temporal usa mediana de cinco lecturas, EMA adaptativa, memoria de hasta tres cuadros perdidos y prior de ancho de sendero de 50 cm. Se rechazan candidatos muy laterales (18 cm) y el LIDAR lateral solo se usa como guía de pasillo cuando ambos lados están dentro de 40 cm, evitando que una sola hilera incline la ruta. La confianza mínima de cámara es 0.30 y su peso 0.8 en este escenario.
- El umbral de avance se ajustó a 58 cm libres; el freno de seguridad conserva 52 cm (margen de parada + deslizamiento configurados).
- Para el eje pasivo limitado a ±20°, el simulador ya no ordena giros estacionarios como si el chasis pudiera pivotar. Si falta espacio para la maniobra completa, solo permite un arco si la lectura frontal mantiene al menos 56 cm libres; si no, queda en estado detenido por falta de espacio seguro.
- `/sendero.jpg` tolera el instante en que el episodio se reinicia y aún no montó el seguidor de cámara.

## Verificación y límites

`py_compile` y validación JSON pasaron. Se ejecutaron 37 pruebas de física y navegación del simulador. La suite completa del backend no se ejecutó porque incluye pruebas de actuadores, puerto serie y enlace C3.

En el muestreo más reciente (t=209.4 s) el robot avanzó 76.32 m, estuvo 54.8% del tiempo dentro del sendero, registró 63 choques y 0 pisadas; SLAM mostró error de 11 cm, rumbo -0.3° y 7 saltos. La variación frente al muestreo anterior (t=132.8 s: 75.8% dentro, 28 choques) confirma que esta corrida no es estable ni prueba navegación fiable. El vehículo queda a veces frenado frente a plantas que solo ve la cámara, con el servo moviéndose y sin lectura visual utilizable; faltan mejoras en la detección de suelo y una evaluación de varias semillas.

El mini-entrenamiento anterior comparó un seguidor de ruta con mapa conocido y SLAM ruidoso; no entrenó ni evaluó una red neuronal visual. Allí el candidato mejoró RMSE de 15.51 a 11.46 cm y ATE SLAM de 22.57 a 15.10 cm, pero la cobertura bajó de 48.77% a 45.46%. Es solo una referencia de simulación.

## Acceso y seguridad

Panel: `http://192.168.0.181:8093/`. Sin conexión a motores ni modificación del servicio físico en puerto 8080. La geometría nominal del robot es 33 × 50 cm; el pasillo es 50 cm; los parámetros dinámicos siguen siendo hipótesis hasta medirlos en el chasis.
