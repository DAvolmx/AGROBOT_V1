# Anexo E. Resumen de ensayos

Este anexo resume cada ensayo físico del robot y su resultado.

**Cómo leerlo.** Los criterios P1 a P12 (sección 3.19) se definieron después de estos ensayos, así que se aplican solo como referencia. «No evaluable» significa que el ensayo no midió lo que pide el criterio. Las pruebas P1 a P12 siguen sin ejecutarse según su protocolo.

**Limitaciones de todos los ensayos.**

- No se registró clima, temperatura ni humedad del suelo.
- El robot no tiene sensor de contacto, por lo que pudo haber choques sin registrar.
- La Jetson no tiene reloj propio, por lo que las horas son aproximadas.
- Desde el 23 de octubre, CO₂, temperatura y humedad son **datos simulados** porque el SCD40 está dañado. Ningún resultado de este anexo los usa.

## E.1 Ensayos interiores

| Ensayo | Fecha y lugar | Resultado | Dictamen |
|---|---|---|---|
| **I-1.** SLAM con regreso al punto inicial | 22-oct. Sala, 23 m | Error de cierre de 36 cm (≈1,5 %) | **No evaluable** en P4: sin medida externa y con un solo recorrido |
| **I-2.** Plantas en maceta | 23-oct. Laboratorio | Maceta detectada en 20/20 cuadros; 0/30 falsas | **Sin criterio.** Registro original sin identificar |
| **I-3.** Recorrido en L con 4 plantas | 28-oct. L de 2,70 × 2,40 m | Especie 17/17; giros de 15–22° por tramo; el giro de 360° se detuvo a 158° por falta de espacio | **No evaluable** en P3: sin medida externa ni 20 repeticiones |
| **I-4.** Recorrido en L, segunda sesión | 28-oct. Con armarios y puertas | 16–17 de 18 avisos de plaga eran falsos | **No cumple** P6 con el modelo de entonces; se corrigió el aviso |

## E.2 Ensayos exteriores

| Ensayo | Fecha y lugar | Resultado | Dictamen |
|---|---|---|---|
| **E-1.** Patio con macetas | 27-oct. Patio junto a un muro | 5 plantas inspeccionadas y 7 fallidas (6 por orden manual); giro de 1,5° frente a 6° pedidos; salto de reloj de 711 s | **No cumple** el giro; **no evaluable** en P1 |
| **E-2.** Huerta, sendero corto | 28-oct. Huerta con coles | 16,2 min en autonomía, en 45 tramos; 24,8 m adelante y 8,8 m atrás; 261 avisos de plaga y 360 macetas falsas | **No cumple** P1: predominó el mando manual |
| **E-3.** Huerta, tierra removida | 29-oct. Sendero con coles y maíz | ≈3 m en x y 4,5 m en y; 77 % del tiempo parado; 154 intervenciones del operador; 0 plagas falsas de PLAGASX v8 en 14 min | **No cumple** P1. P6 **no evaluable**: no se midió cuántas plagas reales detecta |

## E.3 Enlace LoRaWAN

| Ensayo | Fecha y lugar | Resultado | Dictamen |
|---|---|---|---|
| **L-1.** AU915 | 3-nov. Banco de pruebas | Unión OTAA correcta; 2 mensajes en 916,0 y 915,2 MHz, RSSI −29 y −31 dBm; carga útil en cero porque los sensores no responden | **Parcial** en P8: se comprobaron 2 mensajes, no 100 |

## E.4 Dónde están los datos

Los registros se guardan en la Jetson, bajo `~/.agrobot/`, porque ocupan varios gigabytes y contienen fotos del entorno.

| Ensayo | Carpeta | Se cita en la tesis |
|---|---|---|
| I-1 | `campo/20261022_213805_5dde93/` | Tabla 56 |
| I-2 | Sin identificar en `datos/20261023_*` | Tabla 57 |
| I-3 | `pruebas/L_20261028/` | Tablas 56 y 57 |
| I-4 | `pruebas/L2_20261028/` | Tabla 57 |
| E-1 | `datos/20261027_215547/` | Tablas 59 y 60 |
| E-2 | `datos/20261028_141932/` y `pruebas/EXT_20261028/` | Tablas 59 y 60 |
| E-3 | `campo/20261029_142951_a81509/` | Tablas 59 y 60 |
| L-1 | Base de datos de ChirpStack, en la Raspberry Pi | Sección 1.3.15.6 |
