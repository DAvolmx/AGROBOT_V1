# Anexo A. Planos, cableado y montaje

Este anexo reúne la documentación mecánica y eléctrica disponible del prototipo. Las imágenes A-1 a A-5 proceden del borrador de la tesis. Las piezas que faltan se indican al final, sin sustituirlas por material de otra etapa.

## A.1 Chasis

![Figura A-1. Diseño CAD del robot en SolidWorks [10], vista frontal.](figuras/chasis_cad_1.png)

![Figura A-2. Diseño CAD del robot en SolidWorks [10], vista lateral. En el borrador aparece como «prototipo construido», pero es un render CAD.](figuras/chasis_cad_2.png)

![Figura A-3. Chasis metálico soldado del prototipo [10], vista inferior.](figuras/chasis_soldado.png)

**Tabla A-1.** Geometría del prototipo registrada en el proyecto (`geometria_robot.py`, configuración del 20-oct-2026).

| Magnitud | Valor |
|---|---|
| Eje delantero, fijo y con los motores más fuertes | 28 cm de ancho |
| Módulo trasero con pivote pasivo | 38 cm de ancho; giro mecánico ≈ ±20° |
| Batalla | 37 cm |
| Longitud total usada por la navegación | 49 cm |
| LIDAR | A 37,5 cm del suelo; 24 cm por delante del eje trasero |
| Masa y centro de gravedad | 12 kg; 0,20 m (estimados, sin ficha de medición) |

El borrador de la tesis da 19 cm para el ancho delantero. El operador confirmó 28 cm el 20-oct-2026. Debe unificarse en el capítulo III.

## A.2 PCB y esquemático

![Figura A-4. Esquemático de control y distribución de señales en Proteus [10].](figuras/esquematico_proteus.png)

El esquemático y la PCB se diseñaron para una ESP32 LoRa V3 como controlador. En el robot actual, la **ESP32-C3** controla motores y servo y la Heltec V3 adquiere telemetría. Deben cotejarse con el montaje real antes de usarlos para fabricar una placa.

## A.3 Cableado

![Figura A-5. Diagrama histórico de cableado del prototipo [10].](figuras/cableado_historico.png)

**Esta figura representa una etapa anterior.** Las diferencias frente al robot actual son:

| Elemento | En la figura | Robot actual |
|---|---|---|
| Computador | Jetson Nano | Jetson Orin Nano Developer Kit |
| LIDAR | RPLIDAR A1 | RPLIDAR C1, USB (CP2102N), 460 800 baudios |
| Enlace Jetson–C3 | UART | USB CDC y Wi-Fi UDP (puerto 5002) |
| Puentes de motor | Módulos genéricos | TA6586 (entradas FI/BI por rueda) |

La asignación de pines vigente está en el Anexo B. Debe prepararse un diagrama actualizado a partir del equipo instalado.

## A.4 Fotografías y elementos pendientes

| Elemento | Estado | Qué falta |
|---|---|---|
| Planos acotados del chasis | **Pendiente** | No hay en el repositorio ni archivos SolidWorks ni planos con cotas. Exportarlos del modelo CAD |
| Plano de la PCB (figura 38) | **Pendiente** | La figura sale en blanco en el borrador y no hay archivo Proteus ARES en el repositorio |
| Diagrama de cableado actual | **Pendiente** | Dibujarlo según la tabla anterior y comprobar continuidad de cada conexión |
| Fotos del montaje | **Pendiente** | Solo existe la foto del chasis soldado. Tomar: robot completo, electrónica, puentes TA6586, LIDAR y cámara con servo |
| Masa y centro de gravedad | **Pendiente** | Medir con balanza, según el protocolo de identificación mecánica del borrador |
