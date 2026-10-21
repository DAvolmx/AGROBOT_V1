# Reconocer coliflor sana / mosca blanca / marchita — cómo capturar y etiquetar

**Etiquetar = guardar la foto en la carpeta de su clase.** No hace falta ninguna herramienta de dibujo.
Un clasificador mira la foto entera y dice «sana», «mosca_blanca», «marchita» o «fondo».

## Dos clasificadores (se entrenan por separado)
**ESPECIE** — ¿qué planta es? `col` · `coliflor` · `acelga` · `fondo`
**ESTADO** — ¿cómo está la hoja? `sana` · `marchita` · `dano_hoja` · `minador` · `mosca_blanca` · `fondo`

| Clase | Qué debe verse en la foto |
|---|---|
| `col` / `coliflor` / `acelga` | La planta (hoja o planta entera) de esa especie. Acelga: tallo y nervadura claros, hoja brillante/ondulada. |
| `sana` | Hoja verde, firme, sin insectos ni manchas. |
| `marchita` | Hojas caídas/flácidas, amarilleo extendido o bordes secos. |
| `dano_hoja` | Hoja con agujeros o mordidas (orugas, caracoles), sin galerías. |
| `minador` | Galerías o serpenteos claros bajo la epidermis (minador de hoja), a veces con puntos oscuros. |
| `mosca_blanca` | Moscas blancas de 1-2 mm (sobre todo en el envés), o ninfas/mielada. |
| `fondo` | Todo lo que NO es la planta (cargador amarillo, mesa, pared, manos). Se guarda en las dos tareas. |

## Reglas para que funcione (importan más que la cantidad)
1. **Mínimo 60 fotos por clase y 3 sesiones distintas por clase.** Una sesión = una hoja/planta, una luz, un momento.
   Fotos seguidas de la misma hoja son casi iguales; el entrenador separa por sesión para medir de verdad.
2. **Cámara a 10-25 cm de la hoja** y quieta. La mosca blanca mide 1-2 mm: a 50 cm no existe en la imagen.
3. **Luz buena y directa**: LED o lámpara cerca. El script rechaza fotos oscuras (brillo < 35), borrosas y repetidas.
4. **Varía**: hojas distintas, ángulos, haz y envés, mañana y tarde, fondo distinto. No uses una sola hoja.
5. **No mezcles**: una foto de `sana` no debe tener ni una mosca; una de `mosca_blanca` debe mostrarlas claramente.
   Ante la duda, no la guardes (bórrala de la carpeta).
6. Si la planta está sana y enferma a la vez, hazlas en sesiones separadas: cada foto con UNA clase.

## Paso a paso
1. Backend y cámara en marcha (`http://<jetson>:8080`); mira el video del dashboard para encuadrar.
   El LED se enciende desde el dashboard (interruptor de LED). Con el robot QUIETO.
2. Una sesión por vez (cambia `--sesion` cada vez):
   ```bash
   cd ~/Desktop/AGROBOT/agrobot_system_2
   python3 tools/coliflor_dataset.py capturar coliflor     --n 25 --cada 2 --sesion plantaA_manana
   python3 tools/coliflor_dataset.py capturar acelga       --n 25 --cada 2 --sesion plantaB_manana
   python3 tools/coliflor_dataset.py capturar minador      --n 25 --cada 2 --sesion hojaC_manana
   python3 tools/coliflor_dataset.py capturar mosca_blanca --n 25 --cada 2 --sesion hojaD_manana
   python3 tools/coliflor_dataset.py capturar fondo        --n 25 --cada 2 --sesion mesa1
   ```
   Entre foto y foto (2 s) mueve un poco la hoja o la cámara; el script descarta cuadros iguales.
3. Mira el avance: `python3 tools/coliflor_dataset.py contar` (dice qué falta).
4. Revisa las fotos en `~/.agrobot/dataset_planta/<tarea>/<clase>/` y **borra las dudosas** (esta es la etiquetación).
5. Con todo ✓, entrena (tarda minutos; un entrenamiento a la vez):
   ```bash
   python3 tools/coliflor_entrenar.py especie
   python3 tools/coliflor_entrenar.py estado
   ```
   Imprime la precisión por clase sobre sesiones que NO vio y con qué clase confunde cada una.
   Deja el modelo en `~/.agrobot/aprendizaje/coliflor/candidato_<tarea>_cls.pt` sin desplegarlo.

## Después
- Si la precisión por clase ≥ 85-90 % y casi no confunde `sana` con `mosca_blanca`, se integra al backend
  (hoy `camera.py` solo dibuja cajas de detección; un clasificador necesita una pequeña adaptación) y se prueba en vivo.
- Si falla en `mosca_blanca`: más fotos de cerca del envés, con más luz y más sesiones.
- Para **localizar** cada mosca (cajas) haría falta etiquetar con rectángulos (Label Studio/Roboflow): es la fase 2.
