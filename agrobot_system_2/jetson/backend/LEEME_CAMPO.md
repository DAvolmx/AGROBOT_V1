# Captura de campo

Abrir http://192.168.0.181:8080/ y pulsar **Datos de campo / Fotos / Video**.

1. Pulsar **Recolectar datos** al iniciar el recorrido.
2. **Tomar foto** guarda una imagen adicional. Las fotos automáticas se guardan hasta 2 Hz.
3. **Grabar / detener video** alterna clips AVI MJPEG, hasta 10 imágenes por segundo.
4. Pulsar **Terminar sesión** y esperar que la cola quede en cero.
5. **Descargar sesión ZIP** descarga la última sesión. Todas permanecen en `~/.agrobot/campo/`.

`variables.jsonl` incluye telemetría ambiental disponible, estado C3, órdenes y velocidades, servo, pose estimada y su edad, scan LiDAR completo disponible y edad, seguridad, cámara, detecciones y estadísticas. `manifest.json` registra configuración inicial; `frames.jsonl` vincula fotos y video con secuencia e instante de captura. Los datos faltantes no se inventan. El muestreo de variables es 5 Hz, inferior a los aproximadamente 10 Hz del LiDAR: no equivale a un rosbag de todos los mensajes.

Los sensores se vinculan por la muestra más reciente y marcas de tiempo; no hay sincronización física. La pose del EKF es estimada, no verdad terreno. Un video usa reproducción nominal de 10 FPS: para intervalos reales deben usarse las marcas de `frames.jsonl`.

El ZIP sirve para inspección, reproducción de sensores y ajuste posterior de la simulación. No se autoetiqueta ni incorpora al entrenamiento sin revisar contexto y anotaciones. La prueba de escritorio de instalación debe excluirse del entrenamiento de huerto.

La cola está limitada a 20 trabajos y se cuentan descartes. JPEG, AVI, JSONL y empaquetado se ejecutan fuera del event loop de control. Con menos de 5 GB libres, se detiene la recolección. Terminar una sesión antes de apagar la Jetson finaliza el video y crea el ZIP; un apagado abrupto puede dejar el último clip sin finalizar.

Se deshabilitó el módulo auxiliar `auto_comunicacion` por interfaz WiFi inexistente y contrato de configuración incompatible. El enlace principal BLE/UDP y ROS permanecen disponibles. No se cambió firmware ni calibración física. Los controles de captura no mandan movimiento; los controles originales de manejo siguen presentes.

Antes de navegar, comprobar cámara, LiDAR fresco, enlace C3, parada y calibración física. Estas pruebas de captura no certifican navegación autónoma ni detección de obstáculos bajos.
