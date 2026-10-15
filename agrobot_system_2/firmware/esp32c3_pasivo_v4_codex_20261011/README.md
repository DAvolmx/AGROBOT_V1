# AGROBOT C3 v4 - pivote pasivo

Versión candidata basada en el firmware adjunto por el usuario. No flasheada. `PROFILE_VERIFIED=false` impide energizar motores y aceptar comandos hasta verificar el perfil físico. BLE/WiFi se anuncian, estado indica failsafe. No cambiar este indicador sin completar el mapa y ensayo de banco.

## Cambios

- Se elimina el test automático que movía las ruedas al encender.
- Mezcla dimensional coherente: izquierda = v - omega*b/2, derecha = v + omega*b/2. No dividir un término en m/s por el radio antes de sumarlo a v.
- Batalla medida 0,37 m; anchos exteriores delantero 0,19 y trasero 0,37 m. `TRACK_CENTERS_M=0.30` es un marcador NO medido. Sustituir por distancia transversal entre centros de contacto traseros.
- Orden de curvatura limitada a tan(17 grados)/0,37; margen respecto a topes mecánicos de 20 grados. A v=0 no hay giro. No es realimentación del pivote ni garantía de no tocar topes.
- Velocidad nominal de orden limitada a +0,12/-0,06 m/s y rampas. Son consignas abiertas, NO velocidades medidas; la conversión PWM sigue necesitando calibración bajo carga.
- PWM hardware 20 kHz/10 bits con canales 2..5 y servo canal 0 a 50 Hz. Cambio de sentido con ambas entradas bajas y 20 ms de espera. Comprobar compatibilidad de frecuencia y tabla de verdad del puente H. LOW/LOW no se denomina frenado activo porque depende del driver.
- Sin boost por supuesto hundimiento: PWM elevado no demuestra esfuerzo, corriente, movimiento ni patinamiento.
- Failsafe a 500 ms, fallo de inicialización PWM y arranque bloqueado. El temporizador de watchdog del loop se conserva. Una parada desenergiza; no garantiza distancia de frenado mecánico.
- Secuencias uint16 con orden modular y rechazo de duplicados/antiguas. BLE y UDP usan el contador compartido de Jetson, y se aceptan tramas nuevas por cualquiera de las dos vías. Orden neutra desenergiza inmediatamente, incluso para emergency_stop del backend existente.

## Pendientes que afectan el movimiento

Confirmar qué motor es delantero/trasero y qué lado físico representan R1/R2/L1/L2. Se conserva como hipótesis el cableado original: IZQ=R2/R1, DER=L2/L1. El código no transforma esos nombres en hechos medidos. Agrupar por lado es una aproximación provisional: las ruedas delanteras giran juntas en orientación, pero no necesariamente a igual RPM. Si son motrices, se necesita modelo por rueda y ángulo del pivote observado/estimado antes de pretender cinemática exacta.

Medir ancho de neumáticos, vía entre centros, radio cargado, ángulo/trail del pivote, relación PWM-velocidad y polaridad de cada motor. Comprobar que GPIO2/8 y el cableado de arranque de la C3 funcionan con los periféricos conectados. Los pines originales se conservan; no se inventa un pin extra para dirección.

## Contrato para Jetson / Claude

Paquetes de 8 bytes, magic, CRC8 0x07 y UUID originales conservados. v_pct se interpreta sobre 0,55 m/s y omega_pct sobre 1,5 rad/s para conservar escala del emisor, luego se limita. No son velocidades observadas. Estado informa consignas limitadas/rampadas, no encoders. Servo reportado también es posición comandada interpolada, no medición ni bandera de asentamiento.

Al conectar: enviar paquete nuevo con v=0, omega=0, servo dentro de [25,110] y flags válidos; esperar estado fuera de failsafe antes de pedir avance. Mantener secuencia creciente común BLE/UDP. El reinicio del contador del emisor no rearma automáticamente: preservar contador o reiniciar controladamente el nodo con el robot inmovilizado y volver a neutro. CRC es integridad, no autenticación.

El servo es de cámara. La convención de Claude es pitch_abajo=15.5+0.8*(98-servo), pendiente de comprobación física. Escaneo entre comandos 74 y 106 durante 10 s, disparado por pulso de FLAG_SCAN como hace el backend; las siguientes tramas sin ese bit no lo cancelan. No ajustar la calibración por software como si ya estuviera medida. Un supervisor Jetson debe invalidar la percepción durante movimiento/asentamiento del servo y conservar UNKNOWN donde no hay evidencia.

## Validación

Prueba C++ del núcleo: 40401 combinaciones de mando, límite de curvatura, ausencia de contrarrotación, unidades, NaN, rampa y secuencias con wrap. No prueba el puente H ni la latencia física de parada. Compilación del sketch por separado con Arduino ESP32 3.3.11 / esp32c3. Ningún ensayo con motores reales ni flasheo forma parte de esta entrega.

Documentación LEDC consultada: https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html
