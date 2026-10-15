# AgroBot C3 demo-trasero 1.2 — ESP32-C3 + dos módulos TA6586

Base: la 1.1 de Codex (mismos pines, mismo protocolo, misma API web) **más**:

1. **Límites configurables** desde el tab *Motores y servo* del dashboard, guardados en la C3.
2. **Corrección de los pines al soltar el PWM** (ver abajo).
3. El backend acepta 1.1 y 1.2; la configuración de límites exige 1.2.

Con los valores por defecto el comportamiento es **idéntico a la 1.1**.

## Cargarlo (Arduino IDE)

1. Abre `agrobot_c3_demo_trasero_1_2.ino` (los dos `.h` en la misma carpeta).
2. Placa **ESP32C3 Dev Module**, paquete **esp32 de Espressif** (compilado con 3.3.11; la 3.3.12 de tu PC debe valer). Opciones por defecto.
3. **Desconecta la potencia de los motores** y sube por USB.
4. En el monitor serie (115200) debe salir `AGROBOT demo-trasero-1.2`.

Alternativa sin IDE: `esptool.py --chip esp32c3 --port COMx write_flash 0x0 agrobot_c3_demo_trasero_1_2.merged.bin`

Si ya tenía la 1.1, **se conservan** la asignación de ruedas y las inversiones (mismo espacio NVS). La confirmación se pide de nuevo tras cada arranque.

## Pines (heredados, comprueba contra el cableado)

| Canal | Rueda inicial | FI | BI |
|---|---|---:|---:|
| 1 | R1 delantera derecha | GPIO0 | GPIO1 |
| 2 | L1 delantera izquierda | GPIO3 | GPIO2 |
| 3 | L2 trasera izquierda | GPIO4 | GPIO5 |
| 4 | R2 trasera derecha | GPIO6 | GPIO7 |

Servo GPIO10 (25–130°, inicio 98°), foco GPIO8 activo bajo. TA6586: `10`/`01` = sentidos, `00` = rueda libre, `11` = freno (no se usa). GPIO2 y GPIO8 son pines de arranque: primera carga **sin potencia de motores**.

Red: `JETSON_AP`, IP fija `192.168.12.240`. Panel directo `http://192.168.12.240/`, usuario `agrobot`. **Cambia las contraseñas antes de compartir la red.**

## Puesta en marcha (ruedas en el aire)

1. Dashboard → **Motores y servo** (debe poner `demo-trasero-1.2`).
2. Elige un canal, mantén **+** al 25 %, y haz clic en la rueda **de la imagen** que giró. Repite los 4 y **Guardar asignación**. Si una gira al revés: **Invertir sentido**.
3. Marca la casilla → **Confirmar configuración** → **Entregar control a Jetson**.
4. Abre **manejo manual**: la C3 espera una orden cero antes de aceptar avance.

### Límites (tarjeta «Límites de potencia»)

| Campo | Defecto | Rango | Qué hace |
|---|---:|---|---|
| Máx. adelante | 60 % | 10–100 | tope de PWM hacia delante |
| Máx. atrás | 35 % | 10–100 | tope hacia atrás (siempre recto) |
| **Arranque mínimo** | **0** | 0–60 | si pides poco pero >0, sube a este valor. **Úsalo si el motor solo zumba**. 0 = desactivado |
| Reducción interior | 65 % | 0–90 | cuánto se frena la rueda interior en curva |
| Rampa | 60 %/s | 20–300 | aceleración máxima |
| Pausa al invertir | 25 ms | 10–200 | rueda libre antes de cambiar de sentido |

Aplicar **detiene el robot**. Un valor fuera de rango se rechaza **entero**, en el backend y de nuevo en el firmware, y no cambia nada. `minp` no puede superar `fwd`. «Valores por defecto» restaura los de la 1.1.

## Qué cambió respecto a la 1.1 y por qué

**Pines definidos al soltar el PWM.** Tras `ledcDetach` el pin deja de ser GPIO para el gestor de periféricos, y el `digitalWrite(LOW)` posterior **no hace nada** (el core lo rechaza). La 1.1 daba por hecho que FI/BI quedaban en LOW (rueda libre) pero el nivel dependía de un registro que el código no controlaba. La 1.2 hace `pinMode(OUTPUT)` + `digitalWrite(LOW)` explícitos. *No lo he podido verificar en la placa*: tras parar, comprueba que cada rueda queda **libre**, no frenada ni empujando.

## Qué está verificado y qué NO

**Verificado (sin hardware):**
- Compila para ESP32-C3 (80 % de flash, 11 % de RAM).
- `control_model.h`: 7001 comprobaciones en PC, 0 fallos (nunca contrarrotación, giro solo no mueve, entradas NaN/inf → cero, nunca fuera de ±100 %, arranque mínimo no rompe lo anterior).
- CRC8: 3000 tramas aleatorias, **0 discrepancias** entre la Jetson (Python) y el firmware (C++).
- Las 180 combinaciones de órdenes que arma la Jetson pasan las reglas de `udpTick()`; el servo se recorta a 25–130 en la Jetson.
- Las escalas coinciden (`v_max=0,55`, `omega_max=1,5` = `V_SCALE`/`W_SCALE`) y el rango del servo.
- Puente backend↔C3 contra una C3 simulada: 26 pruebas.
- Backend reiniciado y comprobado en vivo.

**NO verificado (solo con la placa y el robot):**
- Que ningún motor gire en el sentido esperado, ni cuál es cuál.
- Duty mínimo de arranque, calentamiento del TA6586, corriente.
- Distancia real de parada en rueda libre.
- Que a 130° el soporte del servo no tope.
- Margen del enlace WiFi (potencia fijada a 8,5 dBm; mira `rssi` en `/status`).
- Comportamiento ante una caída de alimentación.

## Si la C3 se reinicia (brownout)

`verified` vuelve a falso y la Jetson **no podrá conducir** hasta repetir: casilla → **Confirmar** → **Entregar control a Jetson**. Es a propósito (falla seguro), pero **ensáyalo** antes de la revisión.

Ver `BUILD.json` para hashes y resultado de compilación.
