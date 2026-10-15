# AGROBOT C3 v6 — actuador WiFi ligero

Abrir `agrobot_c3_v6.ino` junto a `control_model.h` y `web_panel.h` en su misma carpeta. Placa **ESP32C3 Dev Module**, core Arduino ESP32 **3.x**. No necesita bibliotecas externas: WiFi, UDP, WebServer y Preferences vienen con el core. No se flasheó la placa.

## Redes y acceso

| Perfil | C3 fija | Jetson destino UDP | Red |
|---|---|---|---|
| Principal | 192.168.12.240 | 192.168.12.1:5002 | JETSON_AP / 12345678 |
| Alternativo | 192.168.0.50 | 192.168.0.181:5002 | LoRa-Wan / 12345678 |
| Respaldo local | 192.168.4.1 | Control web directo | AGROBOT-C3-RESPALDO / 12345678 |

La IP AP .240 queda fuera del rango DHCP .50–.200 verificado en la Jetson. Reservar la IP LAN .50 en el router; no se comprobó su exclusión de DHCP. La selección alterna cada 15 segundos si no logra asociación. Si la STA se asocia al router pero la Jetson no manda comandos, también aparece el AP de respaldo. Se abre después de 15 segundos desde arranque cuando no hubo comandos recientes, o al comprobar 3 segundos sin comandos una vez pasado ese intervalo. El AP permanece hasta reiniciar, para no desconectar al operador durante una maniobra.

Web: `http://192.168.12.240/`, `http://192.168.0.50/` o `http://192.168.4.1/`. Usuario **agrobot**, clave **12345678**; cambiar `WEB_PASS` y las claves si se comparte la red. Autenticación HTTP básica en red WPA; no hay TLS. El CRC verifica integridad, no autentica criptográficamente UDP.

## Primer uso

1. El chasis debe estar elevado para identificar ruedas. Al encender no hay auto-test ni impulso de arranque. Ningún comando Jetson mueve hasta confirmar mapeo.
2. Mantener `+` en cada rueda: un motor por vez, PWM inicial 25%, máximo de identificación 50%. Suelta y se para; pérdida de renovación corta motores a 500 ms más hasta 10 ms de periodo de tarea.
3. Si `+` va hacia atrás, **Invertir sentido**. Queda guardado en NVS. Una inversión invalida la confirmación anterior. Los cuatro motores tienen canales PWM independientes y el mapeo R/L no está intercambiado artificialmente.
4. Verificar que R1/L1 sean delanteros y R2/L2 traseros. Si el pin rotulado no pertenece a esa rueda, corregir el cableado o la tabla `motors`; una inversión no cambia la identidad de una rueda.
5. Pulsar **Confirmar que los 4 motores avanzan correctamente**. No es una certificación automática: registra la verificación del operador.
6. Conducir manteniendo el botón. La orden inicial es 25%; en recta el ajuste 100% permite PWM completo en las cuatro ruedas. El giro normal no hace contrarrotación ni pivota en el sitio.
7. **Entregar control a Jetson** detiene primero y espera una orden fresca cero para armar. Si el backend aún manda una orden no cero, seguirá parado: detener desde la Jetson. No se retoma una orden vieja tras reconectar.

En control web, las órdenes UDP de la Jetson se ignoran para motores, servo y foco. El AP no toma control por sí solo. `PARADA` bloquea la autoridad Jetson cambiando a modo web y anula el token de manejo; hace falta una nueva pulsación o entregar control explícitamente. Token y secuencia impiden que peticiones web demoradas revivan una maniobra detenida. La C3 solo acepta UDP desde la IP Jetson del perfil y puerto 5002.

## Integración Jetson

Se conserva `<BbbBBHB>` (8 bytes little-endian), magic A5/5A, CRC8 polinomio 0x07, secuencia uint16. El enlace activo es `ble_link.py`/`udp_actuator_link.py`, aunque esta versión de firmware **no tiene BLE**. El viejo `protocol.py` describe otros paquetes de 14/15 bytes; esos no son compatibles.

En la red AP, `udp_actuator_link.py` ya acepta 192.168.12.240 y aprende su dirección desde el estado. Para la red LoRa-Wan, el backend anterior rechaza 192.168.0.x: ejecutar en la Jetson el script adjunto `configurar_udp_jetson.py` pasando su carpeta backend. Añade solo 192.168.0.50 a la lista permitida; no abre a toda la LAN. Reiniciar el backend detenido. **Ya aplicado en la Jetson al encenderla de nuevo el 13 de octubre.**

La Jetson quedó con control.c3_wifi_only=true: se desactivó la búsqueda BLE y se usa UDP. La inversión antigua del bit del foco se conserva solo en modo legado; para v6 se usa bit positivo tanto en comando como en estado. El estado de firmware representa referencias limitadas y echo, **no velocidades medidas**. Se alineó la configuración del backend con límites de protocolo de 0.55 m/s y 1.5 rad/s. No cambiar escala mientras está en movimiento.

`FLAG_SCAN` no inicia un barrido autónomo en esta versión: la Jetson debe mandar cada posición del servo. Su rango se conserva en 25–110°, HOME 98°. El foco permanece mientras llega la renovación; al perder control se apaga desde el bucle. El servo mantiene la última posición. La dirección NO se controla con ese servo.

## Modelo utilizado y límites

Entre ejes medido L=0.37 m; ancho exterior frontal=0.19 m y trasero=0.37 m. Las separaciones **entre centros** BF=0.143 m y BR=0.323 m usan el ancho de neumático estimado de 0.047 m del simulador local; necesitan medirse. No usar 0.19/0.37 directamente como vía entre centros.

Se limita la curvatura nominal a tan(17°)/L = 0.8263 m⁻¹. Con v,w se obtiene δ_ref=atan(Lw/v). Órdenes nominales traseras: v±w BR/2; delanteras: v/cos(δ_ref)±w BF/2. Se saturan todas proporcionalmente, conservando relación de velocidades. Es una aproximación de rodadura cuasi estacionaria que supone pivote alineado y deriva angular del pivote nula; en tierra suelta, los topes y el deslizamiento la invalidan. La marcha atrás con pivote pasivo requiere validación física adicional.

La escala heredada de rueda a PWM es 0.55 m/s→100%; no es curva de motor medida. Sin encoders, corriente y ángulo del pivote, el firmware no puede cerrar un lazo de velocidad, detectar hundimiento ni mantener el pivote centrado. No incorpora pesos U-Net ni afirma transferencia validada del entrenamiento. La percepción y navegación continúan en la Jetson. El mejor ajuste de simulación a baja velocidad no demuestra que esta escala física sea correcta.

PWM hardware 5 kHz, 8 bits; 4 canales motor y 1 canal servo. Tarea motor de 3072 bytes de stack a 100 Hz, prioridad superior al HTTP; corte independiente por timeout. Rampa máxima estimada 100 puntos porcentuales/s, parada inmediata y 25 ms sin PWM antes de invertir. Sin boost, auto-test, pulsos de 200 ms, servidor asíncrono pesado, JSON dinámico grande o BLE. La web reside en flash; NVS se escribe solo al cambiar inversión/confirmación. `/status` permite medir heap real; el tamaño de RAM estática de compilación no representa consumo total WiFi/RTOS.

## Fallos eléctricos previos

Este firmware no puede reparar la causa de las dos placas quemadas. GPIO son señales de lógica de 3.3 V a un puente H compatible, nunca alimentación de motores. Verificar tabla de entradas del driver: aquí LOW/LOW es deshabilitado o rueda libre, no se afirma freno activo. No se usa HIGH/HIGH. Proteger alimentación de lógica frente a transitorios de motor, comprobar masa lógica del driver y regulador, protección y desacoplo; no unir dominios con un cable improvisado de la Jetson. La causa anterior no está establecida.

GPIO2 (motor) y GPIO8 (foco) son strapping pins; el firmware no puede asegurar sus niveles durante reset. Revisar circuito y arranque con la alimentación de potencia deshabilitada. La retención de driver mediante EN/STBY y polarizaciones compatibles a nivel hardware es necesaria para una garantía de apagado antes de `setup()`; el pinout actual no proporciona un enable dedicado.

Referencias: [LEDC Arduino ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html), [diseño hardware ESP32-C3](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c3/schematic-checklist.html).

## Validación de esta entrega

Compilación y enlace completos con SDK/core 3.3.12 para ESP32-C3. Imagen de aplicación: 1,053,472 bytes. RAM de datos estática: 38,696 bytes; IRAM de código: 66,048 bytes. No confundir estos números con el heap disponible en ejecución. Secuencia/CRC verificados con asserts de compilación y seis escenarios del JS real del panel pasaron en un navegador simulado. No se flasheó ni se probó eléctricamente esta versión.

La Jetson quedó con AP JETSON_AP activo en 192.168.12.1, UDP 5002, modo c3_wifi_only, escalas 0.55/1.5 y bit del foco positivo. El JSON de captura ya admite ausencia de sensores mediante null y no vuelve a publicar Infinity/NaN. La cámara se reintenta sin abrir drivers cuando falta /dev/video0. Estas comprobaciones no equivalen a una prueba real de navegación.
