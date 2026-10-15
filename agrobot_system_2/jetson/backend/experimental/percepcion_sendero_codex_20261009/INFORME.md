# Percepción del sendero y navegación de AgroBot

9 de octubre de 2026

Se revisó el dataset local y una instantánea de 94 archivos actuales de la Jetson, incluyendo los cambios recientes de Claude. El dataset sirve para aprender apariencia del terreno y preparar pruebas de percepción. No contiene las anotaciones, la calibración ni las medidas necesarias para afirmar que un sistema ya navega con fiabilidad o mide alturas de obstáculos.

## Resultado de la auditoría

| Contenido | Cantidad | Observación |
|---|---:|---|
| CAMINO | 768 fotos | Dos ráfagas de 348 y 420 imágenes |
| CAMINO ROTADO | 467 fotos | Dos ráfagas de 266 y 201 imágenes |
| PLANTAS | 518 fotos | Incluye seis duplicados exactos de CAMINO |
| Videos | 2 | 76,32 y 33,23 segundos, aproximadamente 30 FPS |
| Total | 1753 fotos y 2 videos | Aproximadamente 6,73 GB decimales |

Las 1753 imágenes pudieron decodificarse. Todas tienen resolución almacenada 4608×3456 y cámara EXIF Moto G Power. Hay 1747 fotos distintas por contenido binario, pero muchas más similitudes temporales dentro de las ráfagas. Orientación EXIF: 793 con valor 1, 507 con valor 3, 401 con valor 6 y 52 con valor 8. Se debe aplicar esa orientación antes de anotar o entrenar. No se debe interpretar CAMINO ROTADO como una calibración conocida del servo.

Se inspeccionaron visualmente 12 muestras distribuidas por carpeta y cuatro fotogramas por video, además de verificar la lectura de todas las fotos. Se observan senderos, cultivos, suelo de aspecto parecido dentro y fuera del camino, sombras, iluminación concentrada y desenfoque de movimiento. No se midió aún el porcentaje total de imágenes desenfocadas. No se encontraron archivos de máscaras ni etiquetas en este dataset.

## Qué se gana con estas imágenes

La aplicación más útil es segmentar la superficie transitable y sus bordes: distinguir sendero de bancales y plantas, incluso cuando ambos contienen tierra marrón. Un clasificador por color de suelo no resuelve esa diferencia. PLANTAS aporta ejemplos de zonas que no se deben pisar; sus fotos cercanas no deben usarse como si fueran todas vistas de conducción.

Las imágenes borrosas sirven para entrenar o evaluar un rechazo por mala calidad. No se deben convertir en etiquetas seguras de suelo libre. Los videos permiten evaluar consistencia temporal, pérdida del sendero y recuperación. Al proceder de un teléfono, requieren adaptación con capturas de la webcam instalada: lente, altura, exposición, resolución, vibraciones y perspectiva son diferentes.

La colección también puede aportar texturas y ejemplos visuales al simulador. No demuestra alturas de montículos, adherencia, capacidad de frenado ni una reconstrucción métrica del jardín. Tampoco etiqueta especies de plagas.

## Preparación para entrenar sin fuga de datos

Se crearon inventory.json, headers.json y videos.json con evidencias del inventario. navigation_split.json separa las cuatro ráfagas completas: 768 fotos de entrenamiento, 266 de validación y 201 de prueba. Es una separación inicial exigente, con cambio de apariencia entre sesiones; no es evidencia de generalización a otros jardines o días. Las seis copias de CAMINO presentes en PLANTAS deben permanecer en el mismo grupo si se incorpora esa carpeta al entrenamiento.

annotation_queue.json selecciona 60 fotos distribuidas temporalmente, 15 por ráfaga. Son candidatas pendientes de revisión, no máscaras producidas automáticamente. Una primera anotación debería distinguir sendero transitable, cultivo o bancal prohibido, obstáculo o borde peligroso y desconocido. Las sombras conservan la clase de su superficie cuando esta sea visible; de otro modo son desconocidas. No se puede identificar la altura real de todos los obstáculos solo por apariencia.

Después de revisar las máscaras se puede ajustar un segmentador ligero preentrenado, medirlo a la resolución de la webcam y exportar una versión compatible con la Jetson. No se debe entrenar desde cero con solo cuatro recorridos ni separar aleatoriamente fotogramas vecinos. La selección del modelo utiliza validación; la prueba reservada se ejecuta una vez al terminar la selección.

## Navegación con webcam y LIDAR

1. **LIDAR para localización y obstáculos en su plano.** Aporta geometría de paredes y objetos altos. Un rayo sin retorno no certifica el suelo situado debajo del plano del sensor.
2. **Webcam para terreno y cultivos.** Produce máscaras de camino, zonas prohibidas y desconocidas. La geometría se calcula únicamente para píxeles válidos y cámara calibrada.
3. **Servo para inspección activa.** Una vista intermedia sigue el camino, una cercana inspecciona antes de avanzar y una lejana anticipa curvas. Se descartan fotogramas capturados durante el movimiento o antes de que el servo se estabilice. Cada vista lleva hora y ángulo asociados.
4. **Mapa local de tres estados.** Libre observado, ocupado o prohibido, y desconocido. La visión puede impedir circular por una planta aunque el LIDAR no la vea. El LIDAR no borra esos obstáculos bajos al medir espacio libre más arriba. Una observación vieja tampoco autoriza avanzar.
5. **Planificación con huella real.** Evaluar la envolvente del chasis y las ruedas a lo largo de cada trayectoria y el límite pasivo de ±20°, no solo el centro de una línea. En un camino de 50 cm y ancho trasero de 37 cm solo quedan 6,5 cm nominales por lado antes de descontar incertidumbre y oscilación.
6. **Velocidad ligada a lo observado.** Frenar dentro de la distancia libre conservadora: v·latencia + v²/(2a) + margen ≤ distancia libre, medida desde el borde delantero barrido. La desaceleración a se mide frenando en tierra; no se sustituye por el ajuste de aceleración del firmware. Si la franja inmediata está oculta o es dudosa, detenerse e inspeccionar.

El giro del servo cambia la vista, pero no convierte una webcam en cámara de profundidad. Una rotación casi pura no proporciona la base de triangulación de un estéreo. La intersección de rayos con un plano de suelo permite estimar distancias sobre ese plano cuando se conocen intrínsecos, altura y orientación. En bancales, baches o suelo inclinado esa hipótesis puede fallar.

Con movimiento real medido, correspondencias del mismo punto y poses fiables sí se puede explorar paralaje temporal. No basta comparar la distancia al obstáculo más cercano entre fotogramas: puede ser otro punto u otro objeto. Si no hay encoders, la traslación debe estimarse del LIDAR y su incertidumbre; los comandos de motor no son una medida de desplazamiento, especialmente con patinamiento.

## Cambios de Claude revisados

Se conservaron los archivos actuales y se registraron hashes en base_actual_claude/sha256.json. Se encontraron módulos nuevos de fallos 3D, robot rústico, control zigzag, Monte Carlo y barrido de cámara. Sus nombres no prueban validación real.

Hallazgos que deben resolverse antes de usar medidas métricas de cámara en control:

- simulador_pasivo/robot.py calcula inclinación como 15,5 + 0,8·(98−servo); jetson/backend/barrido_tilt.py añade 0,8·(servo−98). Son sentidos opuestos. Hay que medir el sentido físico y compartir una única calibración, sin elegir uno por intuición.
- percepcion_tilt.py emplea probabilidades de detección supuestas. Sus tasas de vuelco comparativas no son precisión medida sobre este dataset.
- EstimadorAltura necesita seguir un punto coherente y una base de traslación fiable. La interfaz actual que recibe distancia al obstáculo más cercano no garantiza esas condiciones.
- suelo_libre.hueco_delante devuelve None tanto ante ausencia de detección como ante datos viejos o referencia dudosa. La navegación debe distinguir explícitamente libre y desconocido.
- La persistencia del detector de suelo compara filas de imágenes anteriores. Si cambia el tilt, esas filas representan otros rayos: se debe reiniciar la memoria geométrica o reproyectarla con la pose de cada captura.

No se sobrescribieron los módulos de Claude ni se reinició el control físico del robot para este análisis.

## Implementación entregada

percepcion_navegacion/fusion_sendero.py contiene una base independiente de hardware para proyectar máscaras al suelo con calibración explícita, rechazar imágenes viejas o tomadas con el servo en movimiento y fusionar contactos de LIDAR sin despejar obstáculos bajos. Incluye límite de velocidad por distancia de frenado y seis pruebas automáticas que pasaron localmente.

Es una interfaz de percepción, no un segmentador entrenado. Requiere máscaras y una calibración validada; no genera distancias de objetos elevados ni sustituye el planificador de huella. El umbral de confianza incluido es provisional y se debe ajustar con validación. La rejilla actual es por observación, no una memoria temporal completa con transformación de poses. Su instalación experimental no habilita movimiento real.

## Métricas que deben decidir si mejora

Medir IoU de sendero y zonas prohibidas, error de borde en centímetros sobre puntos medidos, falsos libres en obstáculos bajos, rechazo por desenfoque y latencia extremo a extremo. En recorridos separados: contactos con plantas, salidas del camino, intervenciones humanas, porcentaje de ruta completada, distancia mínima real a obstáculos y error de localización cuando exista referencia independiente. No publicar porcentajes de precisión mientras falten etiquetas.

La mejora automática debe recopilar fallos y seleccionar ejemplos para revisión. Puede entrenar candidatos fuera de línea y compararlos en escenarios reservados, pero no debe autoconfirmar sus propias etiquetas ni reemplazar al modelo activo basándose únicamente en su confianza. Los casos ambiguos y las correcciones humanas alimentan la siguiente versión.

## Referencias técnicas

- OpenCV, calibración de cámara: https://docs.opencv.org/5.0/main_modules/calib.html
- Nav2 Jazzy, mapa de costes y capas: https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/
- Nav2, zonas prohibidas: https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_keepout_filter/navigation2_with_keepout_filter/

Estas referencias explican componentes generales. Los resultados del inventario proceden de los archivos locales; no hay una certificación externa del robot.
