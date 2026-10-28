# Simulación y entrenamiento del escenario en L (2026-10-28)

**Escenario modelado** (`simulador_pasivo/entrenar_L.py`): L con el tramo largo de 0,70 m el primer metro y 0,90 m el resto (se probó también 1,1 y 1,3 m), tramo corto hacia +y, 4 plantas pegadas a las paredes (posiciones aproximadas; en la primera versión dos macetas escalonadas cerraban el pasillo y esos resultados se descartaron). Estrategia E8 (hueco + predictor suave + A* al volver). El simulador usa el `pulse_drive` real, así que cada pulso de giro de 0,4 s da ~2,3° igual que en el robot.

| Barrido | Qué varía | Episodios | Resultado |
|---|---|---|---|
| 1 | duración del pulso de giro (0,4 / 2,0 / 2,8 s) × grados supuestos (6 / 15) | 36 | éxito 0 %; ida 0–33 %; progreso 0,18–0,34; contactos 5–24. Las diferencias están dentro del ruido (n = 6). |
| 2 | cono lateral (40/60/75°), margen del predictor (0,03/0,05 m), hueco mínimo (40/60 cm) | 48 | éxito 0 %; sin mejora; el cono lateral no cambia nada (60° = 75°). |
| 3 | ancho del pasillo (0,9 / 1,1 / 1,3 m) × giro (0,4 s/6° frente a 2,0 s/15°) | 30 | ida 0–40 %, vuelta 0 % en todos: el ancho no es el cuello de botella. |

**Conclusión honesta:** la simulación reproduce que el L es muy difícil para la estrategia actual (no completa la vuelta en ningún caso), pero **no encontró un parámetro que lo arregle**: ni el giro, ni los frenos, ni el ancho. El cuello de botella parece estar en el fondo del tramo corto (media vuelta en un callejón sin salida) y en visitar plantas en un espacio tan estrecho. El simulador tampoco modela el giro de lazo cerrado (solo pulsos de duración fija), así que no puede validar el cambio de giro.

**Cambios aplicados al robot** (basados en las mediciones reales del 28-oct, no en la simulación):
1. `autonomia.mx_grados_pulso` 6 → **12** (con `giro_lazo_cerrado`): en el robot real los tramos de lazo cerrado de 20° giraron 15–22°, mientras que los de 6° daban ≈1°.
2. El explorador espera a que el conductor termine cada pulso de giro antes de mandar el siguiente (`_pausa_pulso`, `ocupado_cb`): con giros de 1–3 s el pulso siguiente se descartaba en silencio y el giro quedaba corto.
3. `t_max_s` del giro de lazo cerrado 1,2 → 2,0 s.
4. `guardar_mapa_mision` guarda una copia del estado (plantas.json salía con 0 plantas al relanzar la misión).
5. Monitor de 10 s: ya no da falsas alarmas por giros en el sitio ni por movimiento a 0,2 m/s.

**Pendiente de confirmar con el operador:** cono lateral 75° → 85° (propuesta del aprendizaje de correcciones; la simulación no ve diferencia).
Para volver al giro anterior: `autonomia.mx_grados_pulso = 6`.
