# Huerta de pruebas · 7 de octubre de 2026

Versión aislada del simulador existente. Abrir http://192.168.0.181:8092.

## Uso

Desde `agrobot_system_2/simulador_huerta`:

```bash
python3 servidor.py --puerto 8092
```

Arranca en manual. Los botones de flechas controlan únicamente el robot virtual.
La pestaña SLAM permite iniciar el recorrido ideal y compararlo con la autonomía.
Auto ejecuta el explorador existente: no implica que ya navegue correctamente por
este camino estrecho. El editor permite ajustar el trazado aproximado.

## Cambios

- Huella rectangular de 50 cm de largo por 33 cm de ancho total, incluyendo ruedas.
- Comprobación del barrido de traslación y giro: un choque impide atravesar paredes
  o girar a través de ellas. Pasos de comprobación de como máximo 5 mm en la esquina.
- Escenario `huerta_fotos_50cm`: sendero de 50 cm, tierra seca, paja, 42 plantas,
  tanque y banco como referencias visuales. El trazado general es aproximado.
- Robot con cuatro ruedas de tacos, suspensión roja, caja negra, cámara frontal,
  luces, conducto y sensor superior. Las alturas y detalles son aproximaciones.
- Hojas curvadas y texturizadas de aspecto col, lechuga y papa. Agujeros y manchas
  son ejemplos sintéticos; no identifican la especie causante del daño.

## Entrenamiento separado

Los procesos de esta copia redirigen HOME a `laboratorio/` antes de importar los
módulos de aprendizaje. El dataset y los candidatos no se guardan en el HOME real.
El endpoint de despliegue está deshabilitado. El backend real no fue modificado.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 nice -n 10 python3 entrenar.py --recolectar 4 --mundo huerta_fotos_50cm --mezcla 1 --segundos 60
python3 entrenar.py --entrenar --epocas 60
python3 entrenar.py --evaluar 4 --segundos 60
```

Esto entrena la política de conducción existente, NO un detector de cultivos o
plagas. El evaluador heredado usa sus escenarios de prueba; no demuestra por sí
solo rendimiento en esta huerta. Para visión hacen falta anotaciones verificadas
y fotografías independientes de validación; estas imágenes no son un modelo entrenado.

## Límites de fidelidad

La vista orbital usa follaje detallado; la cámara de navegación conserva el
generador sintético original. Los pequeños montículos son visuales y no añaden
una física 3D de suspensión. No hay fotogrametría ni escala global medida.
No se calculan NDVI o clorofila desde RGB. El LIDAR y el SLAM son simulados.
Los parámetros de deslizamiento y motores heredados requieren calibración real.

En un pasillo de 50 cm quedan 8,5 cm por lado al ir centrado. Durante un giro la
proyección transversal puede alcanzar 59,9 cm: no cabe cualquier giro en ese pasillo.

## Comprobaciones

`python3 -m unittest test_huella -v` comprueba paso recto, roce lateral, contacto
con círculo en esquina y barrido de giro/traslación. Verificación adicional en
navegador: conexión WebSocket, WebGL, cámara, controles y ausencia de errores JS.
