# Manejo asistido (Claude maneja el robot)

Claude conduce por la API del backend (puerto 8080) y mira el estado antes de cada movimiento.
Nada de esto salta el freno por LIDAR: cada pulso lo vigila el backend.

## Ver (solo lectura)
`GET /api/asistente/estado` → hueco libre del LIDAR (adelante/atrás/izq/der, m), pose en el mapa,
enlace con la C3, autonomía en marcha, modo seguro e IMU. Se llama antes de CADA orden.

## Mover
- Avanzar/girar medido, lazo cerrado (pulso → quieto → medir → decidir):
  `POST /api/paso {"accion":"avanzar","cm":50}` · `{"accion":"girar","grados":-90}` · `{"accion":"parar"}`
  Estado en `GET /api/paso`. Se niega (409) si hay exploración/patrulla en marcha.
- Pulso suelto: `POST /api/drive {"accion":"adelante|atras|izquierda|derecha|parar","escala":0.1..1}`
- Ir a un punto del mapa con Nav2 (esquiva con el costmap): `POST /api/nav/set_goal {"x":..,"y":..,"theta_deg":..}`,
  cancelar con `POST /api/nav/cancel`.

## Autónomo
- `POST /api/exploracion/start` / `stop`: recorre, esquiva y mapea.
- `POST /api/mission/start`: patrulla por los waypoints grabados; **sin recorrido grabado explora y mapea solo**
  (y deja el recorrido grabado para la siguiente patrulla). `POST /api/mission/stop` para todo.
- Parada total: `POST /api/mission/stop` o el botón rojo del dashboard.

## Reglas de Claude al conducir
1. Mirar `/api/asistente/estado`: scan con edad < 1 s, C3 conectada, hueco delante mayor que lo que se va a avanzar.
2. Pasos cortos (≤ 50 cm, giros ≤ 90°), uno por vez, esperando a que `GET /api/paso` termine.
3. Si `hueco_m` delante < 45 cm: no avanzar, girar al lado más libre o retroceder si atrás > 30 cm.
4. Ante cualquier cosa rara (C3 sin enlace, LIDAR sin datos, 2 pasos seguidos sin progreso): `parar` y avisar.
