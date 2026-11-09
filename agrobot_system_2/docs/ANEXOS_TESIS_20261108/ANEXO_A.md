# Anexo A. Planos, cableado y montaje

Este anexo reúne los planos e imágenes disponibles del robot. Al final se indica lo que todavía falta.

## A.1 Chasis

![Figura A-1. Diseño CAD del robot en SolidWorks [10].](figuras/chasis_cad_1.png)

![Figura A-2. Chasis metálico soldado, vista inferior [10].](figuras/chasis_soldado.png)

**Tabla A-1.** Medidas del robot.

| Medida | Valor |
|---|---|
| Eje delantero (fijo, motores más fuertes) | 28 cm de ancho |
| Módulo trasero (pivote libre, ±20°) | 38 cm de ancho |
| Distancia entre ejes | 37 cm |
| Longitud total | 49 cm |
| Altura del LIDAR | 37,5 cm |
| Masa y centro de gravedad | 12 kg; 0,20 m (estimados, sin medir) |

El borrador de la tesis indica 19 cm de ancho delantero; el valor confirmado es 28 cm.

## A.2 Esquemático y cableado

![Figura A-3. Esquemático de control en Proteus [10].](figuras/esquematico_proteus.png)

![Figura A-4. Cableado de una etapa anterior del prototipo [10].](figuras/cableado_historico.png)

Las figuras A-3 y A-4 corresponden a una etapa anterior. El robot actual cambió en lo siguiente:

| Elemento | En las figuras | Robot actual |
|---|---|---|
| Computador | Jetson Nano | Jetson Orin Nano |
| LIDAR | RPLIDAR A1 | RPLIDAR C1 por USB |
| Control de motores | ESP32 LoRa V3 | ESP32-C3 |
| Enlace con la Jetson | UART | USB y Wi-Fi |
| Puentes de motor | Módulos genéricos | TA6586 |

## A.3 Pendiente

- Planos del chasis con medidas y plano de la PCB.
- Diagrama del cableado actual.
- Fotos del robot completo, la electrónica, el LIDAR y la cámara.
- Medición real de la masa y del centro de gravedad.
