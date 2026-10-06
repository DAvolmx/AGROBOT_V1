# Widget de Control de Redes - AGROBOT

## ¿Qué es?

Panel web completo para cambiar entre dos modos de operación del robot:
- **Patrullaje (C3)**: Conectado al AP de la ESP32-C3 (192.168.12.x)
- **Estación Base**: Conectado a LoRa_Wan para docking (192.168.0.x)

## Acceso Rápido

**URL del widget:**
```
http://192.168.12.8:8000/dashboard/widget_redes.html
```

o desde la Jetson:
```
http://localhost:8000/dashboard/widget_redes.html
```

## Características

✓ **Interfaz limpia** - Diseño adaptable (móvil/tablet/desktop)
✓ **Botones intuitivos** - Un clic para cambiar de red
✓ **Estado en tiempo real** - Muestra conexión actual
✓ **Sin dependencias** - HTML/CSS/JS puro, funciona sin internet
✓ **Integrado con el backend** - Usa API `/api/wifi/modo`

## Cómo Usar

1. **Abrir el widget en el navegador:**
   - Desde tu laptop: `http://192.168.12.8:8000/dashboard/widget_redes.html`
   - O directamente desde la Jetson

2. **Ver estado actual:**
   - El panel muestra la red activa (verde)
   - Muestra el icono de conexión (✓ online)

3. **Cambiar de red:**
   - Click en el botón "Conectar" de la red deseada
   - Esperar 2-3 segundos
   - El panel se actualiza automáticamente

## Flujo de Operación

### Para Patrullaje (fuera de la base):
```
Usuario hace click → "Conectar" a Patrullaje (C3)
     ↓
API cambia a modo "patrullaje"
     ↓
WiFi Manager intenta conectar a JETSON_AP
     ↓
Robot conectado a C3 AP (192.168.12.x)
     ↓
¡Listo para controlar el robot en el campo!
```

### Para Estación Base (en el dock):
```
Usuario hace click → "Conectar" a Estación Base
     ↓
API cambia a modo "estacion"
     ↓
WiFi Manager desconecta de C3
     ↓
WiFi Manager intenta conectar a Lora_Wan
     ↓
Robot conectado a red base (192.168.0.x)
     ↓
¡Listo para cargar batería y sincronizar!
```

## Integración en Dashboard Principal

Si quieres incluir el widget en el dashboard principal (`index.html`):

### Opción 1: Iframe (recomendado)

Añade esto a `index.html` donde quieras que aparezca:

```html
<div class="panel" style="grid-column: span 1;">
    <iframe 
        src="widget_redes.html" 
        style="
            width: 100%;
            height: 600px;
            border: none;
            border-radius: 14px;
            box-shadow: 0 1px 2px rgba(22, 40, 28, 0.04), 0 8px 24px -18px rgba(22, 40, 28, 0.35);
        "
    ></iframe>
</div>
```

### Opción 2: Incluir directamente

Copia el contenido del `<style>` y `<script>` del widget directamente en `index.html`.

## API Backend Requerida

El widget necesita que el servidor backend tenga estos endpoints:

### GET `/api/comunicacion/estado`
**Retorna:** Estado actual de comunicación
```json
{
  "modo": "patrullaje",
  "conectado": true,
  "ble_conectado": true,
  "wifi_conectado": true
}
```

### POST `/api/wifi/modo`
**Body:**
```json
{
  "modo": "patrullaje"  // o "estacion"
}
```

**Retorna:** 200 OK si cambio exitoso

> ✓ Estos endpoints ya están en `jetson_server.py`

## Troubleshooting

### Widget no carga
- Verificar que el servidor backend está corriendo: `systemctl status agrobot-backend`
- Verificar URL correcta (cambiar IP según tu red)

### Cambio de red lento
- Normal: toma 3-5 segundos
- El sistema automático de reconexión puede interferir
- Ver `/var/log/agrobot_reconnect.log`

### "Error de conexión"
- Verificar que la C3 está encendida
- Verificar que JETSON_AP existe (desde laptop escanear redes)
- Ver logs: `journalctl -u agrobot-backend -f`

## Archivos

```
/home/jetson/Desktop/AGROBOT/
├── agrobot_system_2/jetson/dashboard/
│   ├── index.html                    (dashboard principal)
│   ├── widget_redes.html             ← NUEVO
│   └── training.html
└── INTEGRACION_WIDGET_REDES.md       ← Este archivo
```

## Personalización

### Cambiar colores
Edita las variables CSS en el `<style>`:
```css
--accent: #00bd4f;      /* verde patrullaje */
--lora: #9333ff;        /* púrpura estación */
--warn: #ff9500;        /* naranja advertencias */
```

### Cambiar textos
Busca la sección `REDES = { ... }` en el JavaScript:
```javascript
'patrullaje': {
    nombre: 'Patrullaje (C3)',        ← cambiar
    icono: '🤖',                      ← cambiar emoji
    ssid: 'JETSON_AP',
    ...
}
```

### Cambiar intervalo de actualización
En el JavaScript, busca:
```javascript
setInterval(actualizarEstado, 5000);  // 5000 ms = 5 segundos
```

## Verificación Final

✓ Widget abre en navegador  
✓ Muestra estado actual de conexión  
✓ Botones responden al click  
✓ Log muestra cambios de modo en `/var/log/agrobot_reconnect.log`  
✓ Robot cambia de red correctamente  

---

**Creado:** 2026-10-03  
**Versión:** 1.0  
**Status:** ✅ Listo para usar
