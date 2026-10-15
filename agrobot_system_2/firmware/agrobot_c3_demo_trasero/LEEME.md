# AgroBot demo trasero 1.1 — ESP32-C3 / dos módulos TA6586

Abrir `agrobot_c3_demo_trasero.ino` con `control_model.h` y `web_panel.h` en la misma carpeta. Arduino IDE: placa **ESP32C3 Dev Module**, paquete **esp32 de Espressif 3.3.12**. No requiere librerías externas. Compilar no carga la placa.

## Pines heredados del proyecto: comprobar contra el cableado

| Canal físico (asignación inicial) | Entrada FI | Entrada BI |
|---|---:|---:|
| R1 delantero derecho | GPIO0 | GPIO1 |
| L1 delantero izquierdo | GPIO3 | GPIO2 |
| L2 trasero izquierdo | GPIO4 | GPIO5 |
| R2 trasero derecho | GPIO6 | GPIO7 |

Servo de cámara GPIO10; foco GPIO8 activo bajo. El servo no dirige el robot. GND de C3 y lógica de los drivers debe compartir referencia. Motores alimentados desde la fuente de potencia del módulo, nunca desde GPIO. El pin rotulado 3.3V output del módulo no debe conectarse como si fuera entrada ni unirse a otra fuente de 3.3V sin validar el esquema.

TA6586: FI/BI=10 y 01 dan sentidos opuestos; 00 deja salidas abiertas (rueda libre); 11 frena. Esta versión usa 00 al parar y durante inversión; no aplica frenado activo. PWM heredado 5 kHz, pendiente de validación térmica y de respuesta con los motores reales. La corriente admisible del módulo montado no queda certificada por el máximo absoluto del chip. GPIO2 y GPIO8 son pines de arranque del C3: el firmware no controla sus niveles antes de setup; primera carga con potencia de motores desconectada.

## Primera puesta en marcha

1. Desconectar la alimentación de potencia de motores y cargar por USB el sketch. No hay autotest ni orden de movimiento al arrancar. Para verificar ruedas luego, elevar el chasis de forma estable.
2. La C3 busca **JETSON_AP**, clave `12345678`, IP fija **192.168.12.240**. Esperar su conexión. En el dashboard Jetson **http://192.168.0.181:8080/** abrir la pestaña **Motores y servo**. El firmware anterior puede aparecer conectado por telemetría pero no permite usar esta configuración.
3. Seleccionar un canal (par de GPIO). Con ruedas elevadas y potencia habilitada para la prueba, mantener `+` o `−`. Empezar con 25% y ajustar si no arranca. Al soltar se corta la orden. Hacer clic en la rueda de la imagen que realmente giró; el frente del robot está arriba. Repetir los cuatro canales y pulsar **Guardar asignación**. Los GPIO físicos permanecen fijos: se asigna qué rueda está conectada a cada par, sin recablear.
4. `+` debe impulsar cada rueda hacia delante. Si gira al revés, pulsar **Invertir sentido** para ese canal. Probar los cuatro y marcar la casilla de comprobación; pulsar **Confirmar configuración**. Asignación e inversiones se guardan en NVS, pero la confirmación se exige después de cada reinicio.
5. Pulsar **Entregar control a Jetson** y abrir el manejo manual. La C3 espera una nueva orden cero antes de aceptar avance; no reanuda órdenes anteriores. El backend bloquea conducción con firmware anterior o configuración sin confirmar. El control es manual, no una demostración de navegación autónoma.
6. Servo de cámara: seleccionar **25 a 130 grados**, centro inicial 98, y pulsar **Aplicar ángulo**. Se detienen los motores y el servo cambia gradualmente. Después, entregar de nuevo el control a Jetson para conducir. El ángulo es la orden del servo, no una medición de inclinación física.
7. Alternativamente, panel directo C3 **http://192.168.12.240/**, usuario `agrobot`, contraseña `12345678`. Para asignar ruedas visualmente usar la pestaña Jetson. Los botones de prueba directos corresponden a los canales físicos originales.

## Lógica

- Cuatro motores independientes; mezcla por lados. En curva se reduce el lado interior hasta un 65% respecto al exterior. Esto pide una curva, no garantiza radio ni ángulo del pivote pasivo.
- Adelante limitado a 60% PWM; atrás a 35% y sin curva. No hay contrarrotación ni giro sobre sitio. Orden de giro sin avance deja todos los motores en cero.
- Rampa inicial 60 puntos porcentuales/s y pausa de 25 ms antes de invertir. Estos valores requieren prueba física; no son una calibración de motor.
- Pérdida de órdenes: tarea de motor corta potencia tras 500 ms más periodo de tarea de 10 ms, sujeto a planificación del procesador. La parada es rueda libre; distancia real de parada por medir.
- Web y Jetson no controlan a la vez; secuencias rechazan duplicados y paquetes atrasados. Tras caducidad de sesión web es necesario reclamar una nueva.
- No hay encoders, sensor de pivote ni corriente: los porcentajes y referencias no son mediciones de velocidad, ángulo, atasco ni esfuerzo.

## Entrega

Los cambios del backend y panel están en `reportes/control_demo_20261014/`. La copia anterior permanece en `reportes/manejo_snapshot_20261014/`. Consultar `BUILD.json` para el resultado real de compilación y si hubo carga; este documento no implica que se haya flasheado o probado físicamente.

Referencia de fabricante reproducida: https://components101.com/sites/default/files/component_datasheet/ta6586-datasheet.pdf

## Compilación y comprobaciones de esta entrega

Compilado para ESP32-C3 con core Espressif 3.3.12. Se compiló el sketch nuevo y enlazó con core/librerías almacenados de esa versión. BUILD.json registra las huellas del código y binario. No se ha cargado automáticamente ni se han probado motores reales. El ZIP incluye el proyecto fuente completo; mantener los dos .h junto al .ino. En Arduino IDE abrir el .ino, seleccionar ESP32C3 Dev Module y el puerto USB correcto, compilar y subir con potencia de motores desconectada.
