> **AVISO 2026-10-15 — leer antes de usar este guion.** Los números de abajo
> (deslizamiento 14 cm a 0,2 m/s, ganancias kp2/ki6, 49 cm entre macetas) se
> midieron en el simulador con el pivote pasivo **delante**. El chasis actual
> lleva el pivote **detrás** (37 cm fijo delante, 19 cm con pivote atrás), así
> que son **hipótesis a verificar en el suelo**, no datos. Lo único que no
> depende de eso es la lógica del perfil (acoplar velocidad y margen de parada).
> Además `pulse_drive.py` se reescribió el 15-oct y se le devolvió el troceado
> por duty: sin el troceado el perfil de interior no frena como se calculó.
> **Mide el recorrido tras soltar con cinta antes de confiar en `deslizamiento_cm`.**

# Guion del demo en interiores — tesis 2026-10-16

Escrito el 2026-10-14. **Nada de esto se ha probado en el robot todavía**: los
números salen del simulador. Este guion es para que la primera prueba real sea
ordenada y reversible, no para dar por bueno el comportamiento.

---

## 0. Antes de tocar nada: respaldo

```bash
cd ~/Desktop/AGROBOT/agrobot_system_2/jetson/backend && cp agrobot_config.json agrobot_config.json.bak_demo_$(date +%Y%m%d_%H%M)
```

## 1. Por qué el robot no se movía en una sala

El modo de manejo (`interior`/`preciso`/`exterior`) solo cambiaba **la duración
de los pulsos**. Los umbrales de ESPACIO seguían siendo de huerta:

| | huerta | lo que significa en una sala de 4 x 3 m |
|---|---|---|
| `avance_libre_cm` | 100 | exige **1 m libre delante** para lanzar un pulso |
| `margen_parada_cm` + `deslizamiento_cm` | 12 + 40 = **52 cm** | corta el avance a medio metro de todo |
| `distancia_puntos_cm` | 290 | los waypoints se separan 2,9 m |
| `sendero_pared_max_cm` | 150 | una pared a 1,2 m "no cuenta" |

Con eso el robot se queda quieto, y no es culpa del control.

## 2. Aplicar el perfil de interior

Ahora `POST /api/drive/mode` acepta `"perfil": true`, que aplica además los
umbrales espaciales:

```bash
curl -s -X POST localhost:8080/api/drive/mode -H 'Content-Type: application/json' -d '{"modo":"interior","perfil":true}' | python3 -m json.tool
```

Para ver qué cambiaría **sin aplicarlo**:

```bash
curl -s localhost:8080/api/drive/mode | python3 -m json.tool
```

Lo que cambia (`perfiles_entorno.PERFILES["interior"]`):

| clave | huerta | interior |
|---|---|---|
| `escala_velocidad` | 0,5 | **0,13** (= 0,195 m/s) |
| `deslizamiento_cm` | 40 | **20** |
| `margen_parada_cm` | 12 | 12 → corte **32 cm** (antes 52) |
| `avance_libre_cm` | 100 | **40** |
| `lado_libre_cm` | 60 | **30** |
| `retroceso_libre_cm` | 30 | 30 |
| `distancia_puntos_cm` | 290 | **120** |
| `sendero_pared_max_cm` | 150 | **120** |
| `cono_frontal_deg` | 10 | **30** |

### El acoplamiento que NO se puede romper

El margen de parada existe porque **los frenos están desactivados**: al soltar
el mando las ruedas quedan libres y el robot sigue rodando. Ese recorrido
depende muchísimo de la velocidad. Medido en el simulador (mu 0,8, suelo de
interior):

| v pedida | sigue rodando |
|---|---|
| 0,20 m/s | **14,0 cm** |
| 0,30 m/s | 27,5 cm |
| 0,40 m/s | 86,6 cm |
| 0,50 m/s | 142,0 cm |

Cuadra con lo que midió el operador en el robot real (~80 cm, que en esta curva
cae en ~0,4 m/s), y los 40 cm de la config corresponden a ~0,3 m/s.

Por eso **bajar `deslizamiento_cm` sin bajar la velocidad es quitarle la
distancia de frenado**. `perfiles_entorno.validar()` se niega a aplicar una sin
la otra, y la API devuelve 400 en vez de aplicar el perfil a medias.

### Primera verificación en el suelo real (10 minutos, hazla)

Con el perfil de interior puesto, en el suelo donde vas a presentar:

1. Manda avanzar en línea recta unos 2 m.
2. Suelta el mando y **mide con cinta** cuánto sigue rodando.
3. Si pasa de 20 cm, sube `deslizamiento_cm` a lo medido + margen y vuelve a
   aplicar. Si no, el perfil vale.

Un suelo pulido puede deslizar **más** que el simulador, no menos.

## 3. Volver a huerta

```bash
curl -s -X POST localhost:8080/api/drive/mode -H 'Content-Type: application/json' -d '{"modo":"exterior","perfil":true}'
```

## 4. Ruta de detección de plagas (ya existe, no hay que montarla)

`patrol.py` ya es un sistema completo de waypoints con escaneo. Secuencia:

```bash
# 1. Borrar la ruta anterior
curl -s -X POST localhost:8080/api/patrol/clear
# 2. Conducir el robot a mano hasta cada maceta y grabar el punto
curl -s -X POST localhost:8080/api/patrol/record
# 3. Arrancar la misión: recorre los puntos y escanea
curl -s -X POST localhost:8080/api/mission/start
# 4. Estado en vivo
curl -s localhost:8080/api/patrol | python3 -m json.tool
# 5. Escaneo manual (para la demo en vivo)
curl -s -X POST localhost:8080/api/patrol/scan
curl -s -X POST localhost:8080/api/mission/stop
```

También hay grabación automática de waypoints cada `distancia_puntos_cm`
(120 cm en interior): `POST /api/patrol/auto {"activa": true}`.

**El escaneo por detección ya está enganchado**: `_on_detection` →
`patrol.solicitar_escaneo_por_deteccion(...)`, con `detection.escanear_al_detectar`
= `"siempre"` (funciona con la patrulla apagada), filtros de persistencia
(varias detecciones en ventana), *cooldown*, y exigencia de haberse movido
antes de repetir el barrido. Sin esos filtros el robot se quedaría barriendo
delante de la primera maceta que viera.

Lo que corre es **YOLO de plagas** (`ip102_pest_yolo11s.pt`, imgsz 480,
conf 0,6), no la U-Net de segmentación —esa no está en el camino de inferencia
del backend (ver `AUDITORIA_CORREDOR_SEGURO_2026-10-14.md`).

## 5. Lo que el simulador SÍ valida, y lo que no

Con las ganancias que el ajuste de interior eligió (mu 0,8, macetas de 12 cm de
radio, 4 semillas):

| escenario | éxito | RMSE | PICO | contactos |
|---|---|---|---|---|
| `pasillo_macetas` | 75 % | **0,9 cm** | **5,4 cm** | **0 (0 %)** |
| `eslalon_macetas` | 75 % | **0,9 cm** | **5,4 cm** | **0 (0 %)** |
| `esquina_maceta` | 100 % | 30,0 cm | 97,7 cm | 0,75 (75 %) |

**Tramos rectos entre macetas: muy buenos** (cero contactos, 5,4 cm de pico).

### Cuánto hueco dejar entre macetas (barrido, 6 semillas)

| paso libre | holgura/lado | pasillo | eslalon |
|---|---|---|---|
| 42 cm | 2,5 cm | 33 % con contacto | 17 % |
| 46 cm | 4,5 cm | **0 %** | 17 % |
| **49 cm** | **6,0 cm** | **0 %** | **0 %** |
| 55 cm | 9,0 cm | 0 % | 0 % |
| 65 cm | 14,0 cm | 0 % | 0 % |

→ **Deja 50 cm de paso libre entre macetas.** Por debajo de 49 el eslalon
empieza a rozar.

Interior necesita **menos** holgura que huerta (6 cm/lado frente a 14) porque
va más despacio (0,195 frente a 0,4 m/s) y las ganancias están ajustadas al
suelo agarrado: el pico de desvío baja de 15,8 a **5,4 cm**.

**La esquina está mal medida, no rota.** El banco usa un controlador CONTINUO
(v, omega) siguiendo un arco, y un PI sobre el rumbo no tiene acción
anticipativa de curvatura: en un arco de curvatura constante arrastra un
retraso permanente. Falla igual a todas las velocidades probadas (0,195 a
0,7 m/s: RMSE 30 a 44 cm), así que **no es falta de autoridad de giro**.

Pero **el robot real no gira así**: gira a pulsos sobre su propio eje
(`sendero.py`: «la corrección es TODO-O-NADA, se gira un pulso corto y se
vuelve a medir»). Ese modo de giro **no está modelado en este banco**, así que
el resultado de la esquina no dice nada sobre cómo girará en la sala. Es un
hueco del banco, no un veredicto sobre el robot.

→ Para el demo: **gira en parado con pulsos, no intentes trazar curvas suaves.**

## 6. Lo que NO toqué, a propósito

- **SLAM.** Los parámetros de `slam_toolbox_mapping.yaml` (resolución 5 cm,
  `max_laser_range` 8 m, `minimum_travel_distance` 0,12) son razonables para una
  sala de 4 m, y cambiarlos a dos días de la defensa sin poder probarlos en el
  sitio es más riesgo que beneficio. Si quieres probar algo, lo de más efecto
  sería bajar `max_laser_range` a ~5 m (en una sala de 4 m la mitad del barrido
  es vacío) — pero pruébalo antes, con el stack reiniciado y mirando el mapa.
  Nota a favor: **las macetas ayudan al SLAM**, porque dan rasgos donde una
  pared lisa no da ninguno.
- **El stack de ROS en marcha.** Nav2 cargó el `footprint` viejo (±0,175) al
  arrancar; el fichero ya está corregido a ±0,185 pero hace falta reiniciar.
- **Mezclas de PWM y sentido de motores.** Nada.

## 7. Riesgos para el día de la presentación

1. **El deslizamiento en suelo pulido** puede ser mayor que en el simulador.
   Haz la medida con cinta del punto 2.
2. **Giros.** Usa pulsos sobre el eje. No hay validación de curvas suaves.
3. **El pico de desvío del robot real no está medido.** Es lo que decide si
   roza una maceta. Por eso `sendero_si_no_cabe` está en `"aviso"`: cuenta y
   deja pasar, no frena.
4. **Pasillos de menos de 49 cm entre macetas** no están validados ni en
   simulador. Deja al menos 50 cm.
