# Informe · validación de la UNet de Codex sobre el simulador

Fecha: 2026-10-11 · Autor: lado de física/simulación/evaluación (el modelo y su
dataset son de Codex). Hecho en SIMULACIÓN, sin tocar el robot real.

## Objetivo
Usar el modelo preentrenado (`modelo_agrobot_unet.pth`) **dentro del simulador**:
inferir sobre fotogramas que renderiza el sim del huerto y ver cómo se comporta,
como paso previo a subir un modelo mejorado e iterar.

## Qué se hizo
- **Modelo cargado OK**: `segmentation_models_pytorch` UNet, encoder
  `timm-mobilenetv3_large_100`, **6 canales de entrada, 5 clases**. El state_dict
  casó al 100 % (0 claves faltantes/sobrantes). Corre en la GPU de la Orin.
- **Sim**: escenario `huerta_fotos_50cm`, cámara del robot a 640×480, inclinada
  ~33° al sendero. 6 fotogramas a lo largo del sendero.
- **Inferencia**: por cada frame, superposición de la segmentación. Capturas en
  `salidas/` (`frame_NN.png` = RGB | inferencia, `montaje.png`, `rgb_NN.png`).

## Resultado (cobertura de píxeles predichos)
| clase | % píxeles |
|---|---|
| fondo | 59.7 |
| sendero_visible | 7.9 |
| zona_cultivo_no_pisar | 26.7 |
| bancal_borde_elevado | 5.8 |
| **obstaculo_visible** | **0.0** |

**Cualitativo** (ver capturas): el modelo capta a grandes rasgos el **cultivo**
(verde, hileras laterales) y a veces el **sendero** (naranja, centro), pero:
- **confunde la paja/tierra del sim con cultivo** (sobre-predice verde),
- **no separa limpio el sendero** (naranja parcheado y a veces desplazado),
- **nunca predice `obstaculo_visible`** (0.0 %): la clase que su dataset no tenía.

Esto es coherente con las métricas de Codex: `mIoU_real 0.73` vs `mIoU_synth 0.45`
(brecha real↔sintético) y la clase obstáculo con IoU null (sin ejemplos).

## Limitaciones honestas (no son métricas medidas)
1. **Preprocesado RECONSTRUIDO**: cómo arma Codex los 6 canales (RGB + a* + ExGR
   + relieve) y su normalización NO está en el repo. Aquí se usó la receta
   estándar (RGB ImageNet, a* de LAB, ExGR, relieve=gradiente). Si no coincide con
   el suyo, la inferencia se degrada → el resultado es **cualitativo**.
2. **Sin verdad de terreno en el sim**: el render es foto-realista por texturas,
   no lleva clase por píxel, así que **NO se calcula IoU** aquí.
3. **Brecha de dominio**: modelo entrenado con fotos, evaluado en imágenes
   sintéticas del sim.

## Qué mejorar (lo que revela esta validación)
- **Simulador**: (a) añadir un **modo de render SEMÁNTICO** (máscara de clase por
  píxel desde la geometría conocida del mundo) → permitiría medir IoU de verdad
  y, sobre todo, **generar datos etiquetados**; (b) subir el **realismo de
  texturas/iluminación** para cerrar la brecha (hoy la paja se confunde con
  cultivo).
- **Datos para Codex**: el sim puede generar la clase **`obstaculo_visible`**
  (montículos bajos etiquetados) que su dataset no tiene — es la mayor aportación
  posible, y conecta directamente con la física de vuelco/altura ya construida.
- **Preprocesado**: pedir a Codex su función exacta de 6 canales para una
  validación fiel (y para que el robot infiera igual que en entrenamiento).

## Reproducibilidad
```
cd simulador_pasivo/validacion_percepcion && python3 validar_sim.py
```
Entorno: torch 2.13.0+cu130, segmentation-models-pytorch 0.5.0, timm, opencv 5.0,
instalados con `--break-system-packages --no-deps` (no se tocó el torch con CUDA).
Modelo y métricas de Codex copiados en este directorio para reproducibilidad.
