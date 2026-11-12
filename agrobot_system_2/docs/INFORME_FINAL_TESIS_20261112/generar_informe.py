#!/usr/bin/env python3
"""Informe final de la tesis AgroBot (DOCX + PDF), con todos los ensayos hasta el 12-nov-2026.

Todas las cifras salen de los datos del robot, de los informes de cada prueba en docs/ y de los anexos.
Las métricas de PLAGASX v10 y ARANDANOS v2 se completan con --metricas cuando terminen en Kaggle.
Uso: python3 generar_informe.py
"""
import json, os, subprocess
from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT

H = os.path.expanduser('~')
D = H + '/Desktop/AGROBOT/agrobot_system_2/docs/'
OUT = D + 'INFORME_FINAL_TESIS_20261112/'
AS = H + '/video_ldz/p4/assets/'
PL = H + '/.agrobot/pruebas/fotos_plagas_20261112/'
LOGO = H + '/video_ldz/p4/logo/NlogUTA_Mesa-de-trabajo-1.png'
AZUL = RGBColor(0x1d, 0x4e, 0x7e)
MET = OUT + 'metricas_reentrenamiento.json'   # se crea cuando terminan los entrenamientos

doc = Document()
st = doc.styles['Normal']; st.font.name = 'Calibri'; st.font.size = Pt(11)
for s in doc.sections: s.left_margin = s.right_margin = Cm(2.5); s.top_margin = s.bottom_margin = Cm(2.2)

def h(t, n=1):
    p = doc.add_heading(t, n)
    for r in p.runs: r.font.color.rgb = AZUL
def par(t, b=False, i=False):
    p = doc.add_paragraph(); r = p.add_run(t); r.bold = b; r.italic = i; return p
def vin(items):
    for t in items: doc.add_paragraph(t, style='List Bullet')
def tabla(cab, filas, anchos=None, titulo=None):
    if titulo: par(titulo, b=True)
    t = doc.add_table(rows=1, cols=len(cab)); t.style = 'Light Grid Accent 1'; t.alignment = WD_TABLE_ALIGNMENT.CENTER
    for c, v in zip(t.rows[0].cells, cab): c.text = v
    for f in filas:
        cs = t.add_row().cells
        for c, v in zip(cs, f): c.text = str(v)
    if anchos:
        for row in t.rows:
            for c, w in zip(row.cells, anchos): c.width = Cm(w)
    doc.add_paragraph()
def fig(path, ancho, pie):
    if not os.path.exists(path): par(f'[Falta la figura: {path}]', i=True); return
    doc.add_picture(path, width=Cm(ancho)); doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
    p = par(pie, i=True); p.alignment = WD_ALIGN_PARAGRAPH.CENTER

# ---------------------------------------------------------------- portada
doc.add_picture(LOGO, width=Cm(9)); doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
for t, sz, b in (('Facultad de Ingeniería en Sistemas, Electrónica e Industrial', 13, False), ('Carrera de Telecomunicaciones', 13, False), ('', 10, False),
                 ('Informe final de resultados', 15, False),
                 ('AgroBot: robot agrícola autónomo con visión artificial y comunicación LoRaWAN para el monitoreo de plagas en cultivos de col y arándano', 18, True),
                 ('', 10, False), ('Autor: Luis David Gómez Ruiz', 12, False), ('Tutor: Ing. Edgar Patricio Córdova', 12, False), ('Ambato, noviembre de 2026', 12, False)):
    p = doc.add_paragraph(); p.alignment = WD_ALIGN_PARAGRAPH.CENTER; r = p.add_run(t); r.font.size = Pt(sz); r.bold = b
    if b: r.font.color.rgb = AZUL
doc.add_page_break()

# ---------------------------------------------------------------- resumen
h('Resumen')
par('Este informe reúne los resultados del robot AgroBot hasta el 12 de noviembre de 2026: su construcción, la fusión de sensores, la navegación autónoma, '
    'los modelos de visión, las comunicaciones y todos los ensayos físicos, en interior y en campo. Cada ensayo se reporta con su resultado medido y un '
    'dictamen frente a los criterios de la tesis. Las cifras provienen de los registros del robot; donde algo no se midió, se indica.')
vin(['El sistema completo funcionó en campo: SLAM con un mapa de 13,8 × 9,1 m y una vuelta cerrada en la huerta de coles, detección en la Jetson, recolección de datos sin pérdidas y LoRaWAN con 34 de 34 paquetes recibidos.',
     'La localización se recupera sola tras un reinicio desde el 11-nov (28 s en la prueba de coles).',
     'El límite actual es la percepción: los modelos entrenados con fotos públicas generan falsos positivos en campo. Se corrigieron en parte con filtros y se reentrenan con fotos propias y aumentación.',
     'Quedan pendientes las pruebas P1 a P12 con su protocolo completo, la medición de alcance LoRa a 2–3 km y la optimización con TensorRT.'])

# ---------------------------------------------------------------- sistema
h('1. Descripción del sistema')
h('1.1 Hardware', 2)
tabla(['Componente', 'Modelo', 'Función'], [
    ('Computadora', 'NVIDIA Jetson Orin Nano (modo 15 W)', 'ROS 2 Jazzy, SLAM, navegación, visión, dashboard'),
    ('Control de motores', 'ESP32-C3 (firmware FUSIÓN)', 'Motores tipo tanque y servo de la cámara, por Wi-Fi UDP y Bluetooth'),
    ('LIDAR 2D', 'Slamtec RPLIDAR C1', 'Mapa, odometría láser y anticolisión (360°, hasta ~12 m)'),
    ('Cámara', 'NexiGo N60 con servo de inclinación', 'Detección, sendero y odometría visual (640×480)'),
    ('Nodo de sensores', 'Heltec WiFi LoRa 32 V3 + MPU6050', 'Giroscopio a 20 Hz y telemetría LoRaWAN'),
    ('Gateway', 'Heltec HT-M02 V2', 'Recibe LoRa AU915 y reenvía a la Raspberry Pi'),
    ('Servidor', 'Raspberry Pi + ChirpStack', 'Red LoRaWAN, base de datos y dashboard'),
    ('Chasis', 'Acero soldado, 12 kg, eje pasivo ±20°', 'Cuatro ruedas con giro diferencial')], [3.5, 5, 7.5], 'Tabla 1. Componentes del robot.')
h('1.2 Sensores y fusión', 2)
par('Cada sensor cumple un papel distinto. El filtro de Kalman extendido de robot_localization, a 20 Hz y en modo 2D, combina la velocidad de avance de la '
    'odometría láser (rf2o), de la cámara y del detector de reposo con la velocidad de giro del giroscopio, de la cámara y de las órdenes enviadas a los motores. '
    'slam_toolbox corrige la deriva con el mapa, y un vigía de pose descarta saltos imposibles del SLAM o del EKF.')
fig(AS + 'fusion.png', 16, 'Figura 1. Sensores, fusión y uso de cada estimación.')
tabla(['Fuente (tópico ROS)', 'Aporta al EKF'], [('/odom_rf2o (LIDAR)', 'vx'), ('/odom_webcam, /odom_visual (cámara)', 'vx y ω'), ('/imu/data_raw (MPU6050)', 'ω'),
      ('/wheel_odom (órdenes a la C3)', 'ω pedida'), ('/zupt (reposo)', 'vx = vy = ω = 0 cuando está quieto')], [8, 8], 'Tabla 2. Entradas del filtro de Kalman (ekf.yaml).')
h('1.3 Navegación y autonomía', 2)
vin(['Explorador por pulsos guiado por los huecos del LIDAR y por el sendero detectado con la cámara.',
     'Anticolisión: retrocede o se detiene con obstáculos a menos de ~20 cm.',
     'Nav2 con planificador y controlador DWB para ir a metas del mapa. Desde el 11-nov el resultado de cada meta queda registrado.',
     'Vigía de pose: tras un corte de odometría (reinicio de ROS o de la Jetson) adopta el SLAM nuevo cuando se mantiene estable 10 s.',
     'Manejo asistido: el mando del operador pausa la autonomía y el robot aprende de las correcciones.'])

# ---------------------------------------------------------------- visión
h('2. Visión artificial')
tabla(['', 'PLAGASX v8', 'ARANDANOS v1', 'Contexto'], [
    ('Uso', 'Coles: 11 clases', 'Arándano: maduro, inmaduro y 8 plagas', 'Siempre: plantas, macetas, personas, obstáculos'),
    ('Arquitectura', 'YOLOv8s', 'YOLO11s', 'YOLO11s'), ('Parámetros', '11,14 M', '9,43 M', '9,43 M'),
    ('Resolución en el robot', '480 px', '960 px', '640 px'), ('GFLOPs en el robot', '16,1', '49,1', '21,7'),
    ('Tiempo medido en la Jetson', '41 ms', '135 ms', '63 ms'), ('Validación (mAP50)', '0,83', '0,79', '0,68')],
    [4, 4, 4.2, 4.2], 'Tabla 3. Modelos de visión instalados.')
fig(AS + 'red_yolo11s.png', 15, 'Figura 2. Arquitectura YOLO11s (ARANDANOS y contexto), dibujada desde las capas reales del modelo.')
par('Cada modelo trae su propio perfil de filtros. Con el modelo de arándano, los filtros pensados para coles descartaban todos los frutos (el filtro de verde '
    'eliminaba los frutos azules y el filtro de especie exigía una col). Con el perfil propio, de 1144 frutos detectados en 20 fotos de prueba pasaron los 1144.')
fig(AS + 'precision_tam.png', 12, 'Figura 3. ARANDANOS: precisión y tiempo según la resolución de entrada.')
par('Reducir ARANDANOS de 960 a 640 px duplica la velocidad pero baja el mAP50 de 0,81 a 0,70. La optimización recomendada es TensorRT FP16 (no instalado en '
    'JetPack R39 al cierre de este informe), no reducir la imagen. En PyTorch, FP16 casi no acelera en la Jetson porque la versión instalada no trae núcleos para la arquitectura del Orin.')

# ---------------------------------------------------------------- comunicaciones
h('3. Comunicaciones')
vin(['Wi-Fi propio del robot (JETSON_AP) por un adaptador USB RTL8188FTV, con driver rtl8xxxu compilado para el kernel de la Jetson.',
     'LoRaWAN AU915, canales 0–7 + 64, en la banda de uso libre 915–928 MHz del PNF de ARCOTEL.',
     'Cadena: nodo Heltec V3 → gateway HT-M02 → ChirpStack en Raspberry Pi → dashboard y alertas. Acceso remoto por VPN (Tailscale).'])
fig(AS + 'lora_rssi.png', 12, 'Figura 4. Potencia recibida de cada paquete LoRa durante la prueba de coles (11-nov).')

# ---------------------------------------------------------------- ensayos
h('4. Ensayos')
par('Los criterios P1 a P12 se definieron después de los primeros ensayos; se aplican como referencia. «No evaluable» indica que el ensayo no midió lo que pide el criterio.', i=True)
tabla(['Ensayo', 'Fecha y lugar', 'Resultado', 'Dictamen'], [
    ('I-1. SLAM con regreso', '22-oct · sala, 23 m', 'Error de cierre de 36 cm (≈1,5 %)', 'No evaluable en P4: sin medida externa'),
    ('I-2. Plantas en maceta', '23-oct · laboratorio', 'Maceta en 20/20 cuadros; 0/30 falsas', 'Sin criterio'),
    ('I-3. Recorrido en L', '28-oct · L de 2,70 × 2,40 m', 'Especie 17/17; giro de 360° detenido a 158°', 'No evaluable en P3'),
    ('I-4. L con armarios', '28-oct', '16–17 de 18 avisos de plaga falsos', 'No cumple P6; aviso corregido')], [3.2, 3.2, 5.5, 4.2], 'Tabla 4. Ensayos interiores.')
tabla(['Ensayo', 'Fecha y lugar', 'Resultado', 'Dictamen'], [
    ('E-1. Patio con macetas', '27-oct', '5 plantas inspeccionadas, 7 fallidas; giro 1,5° de 6° pedidos', 'No cumple el giro'),
    ('E-2. Huerta, sendero corto', '28-oct', '16,2 min en autonomía; 24,8 m adelante', 'No cumple P1: predominó el mando manual'),
    ('E-3. Huerta, tierra removida', '29-oct', '0 plagas falsas de PLAGASX v8 en 14 min; 77 % del tiempo parado', 'No cumple P1; P6 no evaluable'),
    ('E-4. Vivero Vitro Plantas (arándano)', '11-nov', 'SLAM 9,5 × 12,1 m; ~7,5 m; 64 fotos + 56 s de video; todas las detecciones en vivo fueron falsas (vivero sin fruto)', 'P6 no evaluable; filtro corregido (11 → 0)'),
    ('E-5. Huerta de coles', '11-nov', '15,8 min; ~41 m; mapa 13,8 × 9,1 m con vuelta cerrada; 1237 fotos y 11 min de video sin pérdidas; localización recuperada en 28 s', 'P1 no evaluable: misiones cortas detenidas por el operador')],
    [3.2, 3.2, 5.8, 3.9], 'Tabla 5. Ensayos exteriores.')
fig(AS + 'mapa_coles.png', 12, 'Figura 5. Mapa SLAM de la huerta de coles (E-5). En rojo, la trayectoria; verde, inicio; morado, fin.')
tabla(['Ensayo', 'Fecha', 'Resultado', 'Dictamen'], [
    ('L-1. AU915 en banco', '3-nov', 'Unión OTAA; 2 mensajes, RSSI −29 y −31 dBm', 'Parcial en P8'),
    ('L-2. AU915 en campo (E-5)', '11-nov', '34/34 paquetes; RSSI −53 a −82 dBm; SNR 9,2 a 14,5 dB; SF7', 'Parcial en P8: 34 mensajes, no 100'),
    ('L-3. Falla y recuperación', '11-nov', 'Corte de luz: gateway caído ~6 h y receptor de ChirpStack detenido; tras reiniciar la Pi, el nodo volvió a unirse', 'Lección: respaldo de energía')],
    [3.4, 2.2, 6.8, 3.7], 'Tabla 6. Ensayos de LoRaWAN.')

# ---------------------------------------------------------------- campo 12-nov
h('5. Fotos de campo del 12-nov y plagas identificadas')
tabla(['Foto', 'Contenido real', 'Modelo actual (PLAGASX v8)'], [
    ('1–3', 'Col con mosca blanca (adultos y colonias)', 'Mildiu velloso (detector 0,22–0,76; clasificador 0,96–0,99); mosca blanca solo 0,21 en una'),
    ('4', 'Acelga con manchas café (probable cercospora)', 'Amarillamiento 0,69 (otra especie: fuera del alcance del modelo de col)'),
    ('5', 'Col con pudrición', 'Pulgón 0,42 (la clase pudrición no tenía ejemplos)')], [1.5, 6, 8.5], 'Tabla 7. Inferencias del modelo instalado sobre las fotos nuevas.')
for f, pie in (('inferencias/1.jpg', 'Figura 6. Col con mosca blanca: el modelo actual la clasifica como mildiu.'),
               ('filtro/1_colonias.jpg', 'Figura 7. Filtro top-hat: puntos (amarillo) y colonias (rojo) de mosca blanca.')):
    fig(PL + f, 12, pie)
par('El filtro clásico top-hat resalta las colonias, pero no separa las fotos con mosca blanca (24–72 colonias) de las que no la tienen (0–73), porque también '
    'marca brillos en hojas mojadas. Se usa como apoyo visual y para pre-etiquetar cajas reales, no como detector.')

# ---------------------------------------------------------------- reentrenamiento
h('6. Aumentación y reentrenamiento')
vin(['Fotos nuevas (mosca blanca y pudrición) con 12 variantes aumentadas cada una: recorte que conserva la lesión, espejo, rotación ±12°, brillo, contraste, tono, desenfoque y ruido.',
     'Aumentación en el entrenamiento: mosaico, mixup 0,15, rotación ±10°, traslación 12 %, escala 50 %, inclinación, perspectiva y borrado aleatorio 30 %.',
     'PLAGASX v10: YOLO11m a 960 px, 12 clases, con las fotos de campo sumadas al conjunto anterior (Kaggle, 2 GPU T4).',
     'ARANDANOS v2: YOLO11s a 960 px con 81 negativos del vivero de Vitro Plantas (×3 con aumentación) y medición de falsos positivos en el vivero real.'])
if os.path.exists(MET):
    m = json.load(open(MET)); tabla(['Modelo', 'Métrica', 'Antes', 'Después'], [tuple(f) for f in m['filas']], [4, 5, 3, 3], 'Tabla 8. Resultados del reentrenamiento.')
else:
    par('Los entrenamientos seguían en curso al generar esta versión. Sus métricas (mAP50 por clase, falsos positivos en el vivero) se agregarán en la tabla 8.', i=True)

# ---------------------------------------------------------------- conclusiones
h('7. Conclusiones')
vin(['AgroBot funciona como sistema integrado en campo: mapea, se localiza, detecta, registra datos y los envía por LoRaWAN.',
     'La fusión de LIDAR, cámara, giroscopio y detector de reposo, junto con el vigía de pose, dio una localización estable; la recuperación tras reinicios quedó resuelta.',
     'La percepción es el principal límite: los modelos entrenados con fotos públicas fallan en escenas reales (follaje lejano, hojas nuevas, brillos).',
     'La infraestructura de campo (energía de la Pi y del gateway, reloj sin batería en la Jetson) causó la mayor parte de las fallas, no el software.'])
h('8. Trabajo futuro')
vin(['Etiquetar a mano cajas reales sobre las lesiones y plagas, empezando por las fotos recolectadas.',
     'Instalar TensorRT y medir FP16 e INT8; unir contexto y cultivo en un solo modelo.',
     'Añadir un reloj con batería (RTC) y un respaldo de energía (UPS) para la Pi, el gateway y el robot.',
     'Medir el alcance LoRa a 2–3 km y ejecutar las pruebas P1 a P12 con su protocolo.'])
h('Fuentes de datos')
par('Sesiones en ~/.agrobot/datos y ~/.agrobot/campo; mapas en maps/; informes en docs/ (PRUEBA_COLES_EXTERIOR_20261111, MISION_FOTOS_ARANDANO_20261111, '
    'MODELOS_VISION_20261111, ANEXOS_TESIS_20261108); registros de ChirpStack en la Raspberry Pi.', i=True)

os.makedirs(OUT, exist_ok=True)
doc.save(OUT + 'Informe_final_AgroBot.docx')
subprocess.run(['soffice', '--headless', '--convert-to', 'pdf', '--outdir', OUT, OUT + 'Informe_final_AgroBot.docx'], check=True, capture_output=True)
print('listo:', OUT + 'Informe_final_AgroBot.docx / .pdf')
