# Sesión de campo 2026-10-29 (huerta, perfil exterior)

Datos crudos: `~/.agrobot/campo/20261029_142951_a81509/` (video MJPEG 640×480 10 fps de 781 s, 1359 fotos, 4054 muestras a 5 Hz con LIDAR/pose/IMU/detecciones),
`~/.agrobot/aprendizaje/intervenciones/2026-10-29.jsonl` (154 empujes del operador) y `.../correcciones/guardian_20261029.jsonl` (39 correcciones del guardián).
Mosaico del video cada 50 s: `mosaico_video.jpg`.

## Qué muestra el video
Sendero de tierra removida con coles, maíz y arbustos a los lados, y la casa/cobertizo al fondo. El robot recorre ~3 m en x y 4,5 m en y, con 1525° de giro acumulado
(giró mucho más de lo que avanzó). El 77 % de las muestras está parado (hay 163 s de pausas largas). Seguridad siempre en «normal», pose siempre fiable, sin colisiones.

## Hallazgos
1. **La autonomía quiere girar SIEMPRE a la derecha.** En 153 de 154 intervenciones la orden de la autonomía era `w = -1,2` (derecha). El operador giró al lado contrario 49 veces
   (a la izquierda) y al mismo lado solo 6. Las correcciones del guardián hoy: 32 a la derecha, 7 a la izquierda (el 28-oct estaban equilibradas: 92/101).
2. **El operador empuja hacia adelante 95 veces cuando la autonomía no avanza**, con 89 cm libres delante (mediana) y 38–39 cm a cada lado. Es el patrón de un sendero estrecho:
   el perfil exterior exige `lado_libre_cm = 60` y `avance_libre_cm = 100`, y los laterales entre plantas miden menos. No lo cambié: sin el simulador de lados no puedo medir si bajarlo hace pisar plantas.
3. **Los giros largos no rinden en tierra suelta.** Pulsos de 0,4–1 s dieron 13–23° (izq +21°, der −19°). Los pulsos de 2–4,6 s dieron ~0–5°
   (izq: 13 pulsos, 7,5° de media; der: 13 pulsos, −4,6°). Con `GIRO_IZQ_FACTOR = 2,3` el tope de tiempo a la izquierda es de 4,6 s y se vieron 4 pulsos de 4,4–4,6 s con −0,1° a +8,6°:
   el factor alarga el pulso, pero el robot patina. **No hay evidencia en estos datos de que la izquierda sea más débil**: por segundo ordenado gira 2,7°/s a la izquierda y −2,8°/s a la derecha. Se recomienda
   volver a 1,0 (o probar 1,3 con pulsos cortos) antes de la próxima prueba.
4. **Detector v8 en campo: sin falsos positivos de plaga.** En 14 min con 73 detecciones únicas no apareció ninguna clase de plaga del PLAGASX (antes 50–67 % de los fotogramas daban caja).
   Las 73 son del detector de coles (col_planta 33, col_cogollo 17, col_dano_insecto 5, col_mildiu_velloso 1: reales, conf 0,3–0,5) y de macetas (17, conf media 0,35, máx. 0,43).
5. **Falsos positivos de maceta:** las 17 macetas de la huerta tienen conf ≤ 0,43. En sesiones con macetas reales la mediana es 0,5–0,69, pero el p10 baja a 0,33–0,48,
   así que subir `maceta_conf` a 0,45 perdería ~25 % de macetas reales. Mejor exigir persistencia o vegetación en exterior (pendiente).
6. **Reloj:** saltos de pared de +573 s a las 14:28 y +159 s a las 14:48 (sin RTC). Los timestamps `t_unix` de la sesión pueden estar corridos; usar `t_monotonic`.
7. **El clasificador de entorno pasó a «interior» a las 14:45** (ema 0,445, cerca del umbral) estando todavía en la huerta; con perfil interior el robot usa 40 cm de avance y 0,13 de velocidad.
8. Los datos ambientales (CO₂/temperatura/humedad) siguen siendo simulados (`_sensors_simulated`).

## Sin aplicar (a decidir)
- Poner `GIRO_IZQ_FACTOR = 1.0` (hallazgo 3).
- Revisar por qué la autonomía sesga a la derecha (hallazgo 1) y si `lado_libre_cm` exterior debe bajar (hallazgo 2) usando el simulador de sendero con laterales de 38 cm.
- Subir el umbral del clasificador de entorno o fijar el modo exterior en la huerta (hallazgo 7).
