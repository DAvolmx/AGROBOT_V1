# AGROBOT C3 «suave» v4

Base: tu firmware skid-steer que funciona (BLE, WiFi UDP, protocolo, servo y failsafe **sin cambios**).

## Qué cambia
| | Antes (v3) | Ahora (v4) |
|---|---|---|
| Delanteras R1, L1 | giraban en sentido contrario en los giros | **solo adelante/atrás, las dos iguales**, también al girar |
| Traseras R2, L2 | igual que las delanteras | **hacen todo el giro** por diferencial, con potencia de pivote |
| PWM | on/off por software de 200 ms (tirones) | **PWM de hardware 10 kHz con rampas** |
| Parada | corte | rampa; el failsafe (500 ms sin órdenes) suelta las ruedas **al instante** |
| Arranque | test de motores al encender | **sin movimientos** |
| Anti-hundimiento (+15 %) | sí | quitado (con rampas no tiene sentido) |

Girando en el sitio (v=0) las delanteras quedan libres y solo pivotan las traseras.

## Cargarlo
Arduino IDE: abre `agrobot_c3_suave.ino` (con `suave_model.h` en la MISMA carpeta). Placa *ESP32C3 Dev Module*.
O sin IDE: `esptool.py --chip esp32c3 --port COMx write_flash 0x0 agrobot_c3_suave.merged.bin`.
Ocupa el 96 % de la flash (cabe). **Sube con la potencia de motores desconectada.**

## Configurable desde el dashboard (pestaña Motores, se guarda en la C3)
Pines FI/BI por rueda, invertir, intercambiar, probar cada rueda, y **Fuerza y suavidad**:

| Límite | Defecto | Rango | Qué hace |
|---|---:|---|---|
| Máx. adelante | 80 % | 10–100 | tope de duty hacia delante |
| Máx. atrás | 60 % | 10–100 | tope hacia atrás |
| Potencia de pivote | 90 % | 30–100 | diferencial de las traseras al girar |
| Arranque mínimo | 50 % | 0–90 | sube duties pequeños; si el motor solo zumba, súbelo (≤ topes) |
| Aceleración | 120 %/s | 20–600 | rampa de subida |
| Frenado | 300 %/s | 50–1000 | rampa de bajada |

Cambiar un límite detiene el robot. Un valor fuera de rango se rechaza entero.

## Primera prueba (ruedas en el aire)
1. Pestaña Motores → «C3 conectada» y las 4 ruedas listadas. Si no aparecen, el firmware no es este.
2. Prueba cada rueda con +: debe girar hacia delante. Si no, marca Invertir.
3. Manejo manual: adelante → las 4 giran igual. Girar → solo las **traseras** en sentido contrario.
4. Sube «Arranque mínimo» hasta que arranque con el pulso más corto.

## Qué está verificado y qué NO
Verificado: compila; la mezcla y las rampas (C++ real) coinciden con el port de Python en 6000 casos;
los valores de fábrica cumplen sus propias reglas. **NO verificado en hardware**: el PWM dinámico por
canal LEDC (como en el 1.2), el sentido físico de cada rueda, el arranque mínimo real de tus motores ni
el pivote con el eje pasivo. Con una C3 sana y los motores aislados, pruébalo con las ruedas en el aire.
