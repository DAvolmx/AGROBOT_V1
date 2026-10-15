# AGROBOT v7 candidata — base original BLE + WiFi

NO es una version validada en el robot. Conservar el original funcional como rollback.

Una sola red JETSON_AP/12345678; WIFI_STA, DHCP, sin AP ni web. BLE AGROBOT-C3 con los UUID originales. Jetson debe mantener c3_wifi_only=false y la polaridad legacy del foco. No cambies simultaneamente red y configuracion de motores.

Cambios: autotest al encender deshabilitado; PWM hardware 5kHz/8bits en lugar de pulsos software 200ms; rampa de subida 100 puntos porcentuales/s y bajada inmediata; tiempo muerto 20ms al invertir; rechazo de secuencias duplicadas/atrasadas con vuelta 65535→0; limites de trama y puerto UDP; rearme tras arranque/timeout exige orden cero; no boost por supuesto atasco; diagnostico serie 115200 de tiempo activo, reset, memoria, enlaces y descartes.

Giro: pivote nominal libre de 37cm entre ejes; limite conservador 17° frente al tope mecanico ±20°. Sin giro sobre sitio. Anchos exteriores 19/37cm; ancho de goma 4.7cm ESTIMADO, vias 14.3/32.3cm. Mantiene asignacion fisica de lados invertida del original. R1/L1 se suponen delanteras; debe verificarse. Las cuatro ruedas pueden llegar a 100% en recta; PWM no equivale a velocidad medida. Retroceso requiere prueba especifica del pivote libre.

No incluye identificacion automatica de motores, calibracion persistente, sensor de corriente, sensor del pivote ni encoders. No mide atasco, resbalamiento ni estado de condensadores. No garantiza proteccion electrica.

Importante: no habilitar TEST_MOTORES. Identificar modelo del puente H para validar entradas IN/IN, frecuencia PWM y estados de frenado. Probar con ruedas elevadas, confirmar sentidos y parada al cortar ambos enlaces antes de suelo.

Compilacion Arduino core ESP32 3.3.12, ESP32C3 Dev Module. Resultado final se informa aparte. La tarea de seguridad permanece en el loop original, no tiene tarea de motor independiente; cualquier bloqueo largo provoca parada por WDT/reinicio, no garantia de parada exacta en 500ms bajo bloqueo.
