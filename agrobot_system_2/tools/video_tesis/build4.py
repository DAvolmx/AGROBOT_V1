#!/usr/bin/env python3
"""Presentación final de la tesis AgroBot (PPTX) + guion + video explicativo de ~7 min.

Una sola lista de ESCENAS genera:
  1) salida4/AgroBot_presentacion_final.pptx  (16:9, logo UTA, guion en las notas del orador)
  2) salida4/guion.md                          (texto que se narra, por diapositiva)
  3) salida4/AgroBot_video_tesis.mp4           (diapositivas renderizadas por LibreOffice + voz Piper + subtítulos + clips reales)
Uso: venv/bin/python build4.py [pptx|video|todo]
"""
import glob, json, os, subprocess, sys, wave
from PIL import Image, ImageDraw, ImageFont
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

HOME = os.path.expanduser('~')
OUT = HOME + '/video_ldz/salida4'
for d in ('wav', 'slides', 'seg', 'poster'): os.makedirs(f'{OUT}/{d}', exist_ok=True)
P4 = HOME + '/video_ldz/p4/'
AS, CL, LOGO = P4 + 'assets/', P4 + 'clips/', P4 + 'logo/NlogUTA_Mesa-de-trabajo-1.png'
WA = sorted(glob.glob(HOME + '/.agrobot/pruebas/invernadero_arandano_20261111/fotos_whatsapp/*.jpeg'))
WV = sorted(glob.glob(HOME + '/.agrobot/pruebas/invernadero_arandano_20261111/whatsapp_20261112/*.mp4'))
PL = HOME + '/.agrobot/pruebas/fotos_plagas_20261112/'
DOCS = HOME + '/Desktop/AGROBOT/agrobot_system_2/docs/'
SIM = HOME + '/video_ldz/salida3/win_9.mp4'

AZUL, AZUL2, TINTA, GRIS, FONDO = RGBColor(0x1d, 0x4e, 0x7e), RGBColor(0x2f, 0x6f, 0xa8), RGBColor(0x1b, 0x26, 0x33), RGBColor(0x5b, 0x6b, 0x7a), RGBColor(0xf6, 0xf8, 0xfb)
FUENTE = 'DejaVu Sans'
SW, SH = 13.333, 7.5            # pulgadas; a 144 ppp = 1920 × 1080
PPI = 144
LIVIANA = os.environ.get('LIVIANA') == '1'   # 1 = presentación sin videos incrustados (para el repositorio)
NOMBRE = 'AgroBot_presentacion_final' + ('_sin_video' if LIVIANA else '')

ESCENAS = [
 dict(layout='portada', titulo='AgroBot', sub='Robot agrícola autónomo con visión artificial y comunicación LoRaWAN para el monitoreo de plagas en cultivos de col y arándano',
      img=WA[23],
      frases=['AgroBot es un robot agrícola autónomo, desarrollado como trabajo de titulación en la Universidad Técnica de Ambato.',
              'Recorre cultivos de col y de arándano, mapea el terreno, busca plagas y enfermedades con visión artificial, y envía sus datos por LoRaWAN.',
              'En este video resumo cómo está construido, cómo fusiona sus sensores, cómo son sus redes neuronales y qué mostraron las pruebas de campo.']),
 dict(layout='texto', titulo='Problema y objetivo',
      bullets=['Las plagas se detectan tarde: la revisión es manual y planta por planta', 'Los pequeños productores no tienen herramientas automáticas', 'Objetivo: un robot que recorra el cultivo, detecte plagas y avise a distancia', 'Restricción: hardware de bajo costo y energía limitada'],
      frases=['El problema es que las plagas y enfermedades se detectan tarde, porque la revisión se hace a mano, planta por planta.',
              'El objetivo fue construir un robot que recorra el cultivo por sí solo, detecte plagas con una cámara y avise a distancia, usando hardware de bajo costo.']),
 dict(layout='clip_der', titulo='El robot', clip=WV[1], clip_aspect=480 / 854,
      bullets=['Chasis soldado de 12 kg, 4 ruedas con giro tipo tanque y eje pasivo de ±20°', 'NVIDIA Jetson Orin Nano: ROS 2, mapas y visión', 'ESP32-C3: motores y servo, por Wi-Fi y Bluetooth', 'LIDAR RPLIDAR C1 y cámara NexiGo con servo de inclinación', 'Nodo Heltec LoRa V3 con IMU'],
      frases=['Este es AgroBot en el invernadero de Vitro Plantas. Tiene un chasis soldado de doce kilos y cuatro ruedas que giran como un tanque, con un eje pasivo que copia el terreno.',
              'El cerebro es una Jetson Orin Nano, que corre ROS dos, el mapeo y las redes neuronales. Una placa ESP32 controla los motores y el servo de la cámara.',
              'Arriba lleva un LIDAR de trescientos sesenta grados, una cámara con servo de inclinación, y un nodo LoRa con una unidad inercial.']),
 dict(layout='tabla', titulo='Qué sensor se usa para cada cosa',
      filas=[('Sensor', 'Se usa para'), ('LIDAR RPLIDAR C1', 'Mapa (SLAM), odometría láser, anticolisión y huecos para avanzar'),
             ('Cámara NexiGo N60', 'Detección de plantas y plagas, sendero y odometría visual'), ('IMU MPU6050 (nodo V3)', 'Velocidad de giro para el filtro de Kalman'),
             ('Órdenes de la ESP32-C3', 'Giro pedido: ayuda cuando los demás sensores dudan'), ('Detector de reposo', 'Fija la velocidad en cero cuando está quieto'),
             ('Nodo LoRa Heltec V3', 'Telemetría y alertas hacia la Raspberry Pi')],
      frases=['Cada sensor tiene un papel. El LIDAR construye el mapa, mide cuánto avanza el robot y evita choques.',
              'La cámara detecta plantas y plagas, sigue el sendero y también estima movimiento. La unidad inercial mide el giro.',
              'Y el nodo LoRa lleva la telemetría y las alertas hasta una Raspberry Pi con ChirpStack.']),
 dict(layout='imagen', titulo='Fusión de sensores', img=AS + 'fusion.png',
      frases=['Así se fusionan. Un filtro de Kalman extendido, a veinte hercios, combina la velocidad de avance del LIDAR y de la cámara con la velocidad de giro de la unidad inercial.',
              'Cuando el robot está quieto, un detector de reposo fija la velocidad en cero, para que la posición no se desplace sola.',
              'Encima, el SLAM corrige la deriva con el mapa, y un vigía de pose descarta los saltos imposibles. Desde ayer, también se reubica solo después de un reinicio.']),
 dict(layout='clip_der', titulo='Autonomía', clip=DOCS + 'SESION_CAMPO_20261029/inferencia_sendero_v2.mp4', clip_aspect=960 / 240,
      bullets=['Explorador por pulsos guiado por los huecos del LIDAR', 'Seguidor de sendero con la cámara', 'Anticolisión: retrocede o se detiene', 'Nav2 para ir a metas del mapa', 'Manejo asistido: aprende de las correcciones'],
      frases=['Para moverse, un explorador avanza por pulsos hacia los huecos que ve el LIDAR, y la cámara le indica dónde está el sendero.',
              'Si algo se acerca a menos de veinte centímetros, la anticolisión retrocede o detiene el robot. Para ir a un punto del mapa usa Nav2.',
              'Y cuando el operador corrige el rumbo con el mando, el robot guarda esa corrección y ajusta su comportamiento.']),
 dict(layout='clip_full', titulo='Simulador', clip=SIM, clip_aspect=1920 / 880,
      frases=['Antes de salir al campo, todo se prueba en un simulador que ejecuta el mismo código del robot, con un LIDAR y una cámara simulados.',
              'Aquí se ven sus cuatro vistas: el mundo en tres dimensiones, la cámara, el sendero y el mapa del SLAM, mientras una búsqueda evolutiva ajusta los parámetros.']),
 dict(layout='texto', titulo='Visión: qué modelos corren',
      bullets=['Siempre: modelo de contexto (plantas, macetas, personas, obstáculos)', 'Coles: PLAGASX v8, 11 clases de enfermedades y plagas', 'Arándano: ARANDANOS v1, fruto maduro e inmaduro y 8 plagas', 'Cada modelo trae sus propios filtros', 'Entrenados en Kaggle con 2 GPU T4'],
      frases=['En visión corren dos detectores a la vez: uno de contexto, que siempre está activo, y uno del cultivo elegido.',
              'Para coles se usa PLAGASX, con once clases de enfermedades y plagas. Para arándano, ARANDANOS, que distingue fruto maduro e inmaduro y ocho plagas.',
              'Cada modelo carga sus propios filtros, porque los que servían para coles descartaban todos los arándanos.']),
 dict(layout='imagen', titulo='Arquitectura de la red: YOLO11s', img=AS + 'red_yolo11s.png',
      frases=['Esta es la red YOLO once ese, la de ARANDANOS y la de contexto, dibujada desde sus capas reales. Tiene nueve coma cuatro millones de parámetros.',
              'El backbone reduce la imagen cinco veces y extrae rasgos. El neck los mezcla de arriba abajo y de abajo arriba, y la cabeza detecta en tres escalas: objetos chicos, medianos y grandes.',
              'Incluye un bloque de atención, C2PSA, que ayuda a encontrar objetos pequeños entre el follaje. PLAGASX usa la versión anterior, YOLO ocho ese.']),
 dict(layout='img_texto', titulo='Rendimiento en la Jetson', img=AS + 'precision_tam.png',
      bullets=['Contexto: 63 ms · PLAGASX: 41 ms · ARANDANOS: 135 ms', 'A 640 px ARANDANOS va al doble, pero pierde 0,11 de mAP50', 'FP16 en PyTorch casi no acelera aquí', 'Siguiente paso: TensorRT'],
      frases=['Medido en la Jetson, el modelo de contexto tarda sesenta y tres milisegundos, PLAGASX cuarenta y uno, y ARANDANOS ciento treinta y cinco, porque trabaja a novecientos sesenta píxeles.',
              'Si se baja a seiscientos cuarenta va al doble de rápido, pero pierde once puntos de precisión. Por eso la optimización correcta es TensorRT, no achicar la imagen.']),
 dict(layout='img_texto', titulo='Comunicaciones', img=AS + 'lora_rssi.png',
      bullets=['Wi-Fi propio del robot con antena USB', 'LoRaWAN AU915 (banda libre 915–928 MHz en Ecuador)', 'Nodo V3 → gateway HT-M02 → ChirpStack en Raspberry Pi', 'Acceso remoto por VPN (Tailscale)'],
      frases=['El robot crea su propia red Wi-Fi con una antena USB, y se ve también por VPN desde cualquier lugar.',
              'La telemetría viaja por LoRaWAN en la banda AU915, la de uso libre en Ecuador, hasta un gateway y una Raspberry Pi con ChirpStack.',
              'En la prueba de coles llegaron los treinta y cuatro paquetes, con una señal entre menos cincuenta y tres y menos ochenta y dos dBm.']),
 dict(layout='dos_img', titulo='Ensayos interiores', img=AS + 'mapa_interior.png', img2=sorted(glob.glob(HOME + '/.agrobot/pruebas/L_20261028/fotos/*maceta_frente.jpg'))[0],
      bullets=['SLAM: cierre de 36 cm en 23 m (≈1,5 %)', 'Macetas: 20/20 detectadas, 0/30 falsas', 'Recorrido en L: especie correcta 17/17', 'Avisos falsos en armarios: corregidos'],
      frases=['En interior, el SLAM recorrió veintitrés metros y volvió al punto de partida con un error de treinta y seis centímetros, cerca del uno coma cinco por ciento.',
              'El detector de macetas acertó veinte de veinte cuadros sin falsos positivos, y en un recorrido en L identificó la especie diecisiete de diecisiete veces.']),
 dict(layout='clip_izq', titulo='Prueba en la huerta de coles', clip=CL + 'coles_inferencia.mp4', clip_aspect=960 / 720, img=AS + 'mapa_coles.png',
      bullets=['15,8 min de operación y ~41 m recorridos', 'Mapa de 13,8 × 9,1 m con la vuelta completa', 'Localización recuperada sola en 28 s', 'LoRa 34/34 paquetes, sin pérdidas'],
      frases=['En la huerta de coles el robot operó casi dieciséis minutos y recorrió unos cuarenta y un metros.',
              'El SLAM generó un mapa de trece por nueve metros, con una vuelta completa alrededor de los canteros, y la localización se recuperó sola en veintiocho segundos tras un salto al arrancar.',
              'El modelo de contexto reconoció las coles, y PLAGASX solo dio avisos débiles, por debajo del umbral de alerta.']),
 dict(layout='clip_izq', titulo='Prueba en Vitro Plantas (arándano)', clip=CL + 'vitro_inferencia.mp4', clip_aspect=960 / 720, img=WA[16] if len(WA) > 16 else WA[0],
      bullets=['Vivero sin fruto: aparecieron falsos positivos', 'Filtro de cajas gigantes: 11 → 0 plagas falsas', 'Los 1273 frutos reales de prueba se conservan', 'En arbustos adultos detecta el fruto verde'],
      frases=['En el invernadero de Vitro Plantas el vivero no tenía fruto, y el modelo marcó plagas falsas sobre el suelo y las bandejas.',
              'Un filtro que descarta plagas que ocupan más de un cuarto de la imagen las bajó de once a cero, sin perder ninguno de los mil doscientos frutos reales de prueba.',
              'En la plantación adulta, en cambio, detectó bien los racimos de fruto verde.']),
 dict(layout='plagas', titulo='Plagas en fotos de campo', imgs=[PL + 'orig/1.webp', PL + 'filtro/1_colonias.jpg', PL + 'orig/5.webp', PL + 'orig/4.webp'],
      caps=['Mosca blanca en col', 'Filtro top-hat: colonias', 'Pudrición en col', 'Manchas café en acelga'],
      frases=['Con fotos de campo encontramos mosca blanca en la col, pudrición y manchas café en acelga. El modelo actual las confundía con mildiu.',
              'Probamos un filtro clásico de imagen para resaltar las colonias de mosca blanca. Sirve para verlas y para pre etiquetar, pero no como detector, porque también marca brillos en hojas mojadas.']),
 dict(layout='texto', titulo='Aumentación y reentrenamiento',
      bullets=['Fotos nuevas con 12 aumentaciones cada una: recorte, giro, luz, desenfoque', 'Entrenamiento con mosaico, mixup, rotación ±10°, escala y borrado aleatorio', 'PLAGASX v10 (YOLO11m, 960 px) en Kaggle', 'ARANDANOS v2 con 81 negativos de Vitro Plantas', 'Métricas: se agregan al informe al terminar'],
      frases=['Con esas fotos armamos un nuevo conjunto de datos, con doce variantes aumentadas de cada imagen, y reforzamos la aumentación del entrenamiento: rotación, escala, mezcla de imágenes y borrado aleatorio.',
              'PLAGASX diez y ARANDANOS dos se están entrenando en Kaggle. Sus métricas se agregarán al informe final cuando terminen.']),
 dict(layout='texto', titulo='Conclusiones y trabajo futuro',
      bullets=['El sistema completo funciona en campo: SLAM, visión, datos y LoRaWAN', 'La percepción es el límite: hacen falta datos propios etiquetados', 'Optimizar con TensorRT y unir contexto y cultivo en un modelo', 'Reloj con batería, respaldo de energía y prueba de alcance LoRa a 2–3 km'],
      frases=['En conclusión, AgroBot funciona como sistema completo en campo: mapea, se localiza, detecta, guarda datos y los envía por LoRaWAN.',
              'El límite actual es la percepción, que necesita más fotos propias etiquetadas. Como trabajo futuro quedan TensorRT, unir los modelos, y medir el alcance LoRa a dos o tres kilómetros.',
              'Gracias por su atención.']),
]

# ---------------------------------------------------------------- PPTX
def caja_texto(sl, x, y, w, h, txt, size, color=TINTA, bold=False, align=PP_ALIGN.LEFT):
    tb = sl.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h)); tf = tb.text_frame; tf.word_wrap = True
    p = tf.paragraphs[0]; p.alignment = align; r = p.add_run(); r.text = txt
    r.font.size = Pt(size); r.font.bold = bold; r.font.color.rgb = color; r.font.name = FUENTE
    return tb

def viñetas(sl, x, y, w, h, items, size=22):
    tb = sl.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h)); tf = tb.text_frame; tf.word_wrap = True
    for i, t in enumerate(items):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        r = p.add_run(); r.text = '•  ' + t; r.font.size = Pt(size); r.font.color.rgb = TINTA; r.font.name = FUENTE
        p.space_after = Pt(12)

def imagen(sl, path, x, y, w, h):
    im = Image.open(path); iw, ih = im.size; k = min(w / iw, h / ih)
    pw, ph = iw * k, ih * k
    return sl.shapes.add_picture(path, Inches(x + (w - pw) / 2), Inches(y + (h - ph) / 2), Inches(pw), Inches(ph))

def rect_clip(x, y, w, h, aspect):
    if w / h > aspect: cw, ch = h * aspect, h
    else: cw, ch = w, w / aspect
    return x + (w - cw) / 2, y + (h - ch) / 2, cw, ch

def poster(clip, n):
    p = f'{OUT}/poster/{n}.jpg'
    if not os.path.exists(p):
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-ss', '3', '-i', clip, '-frames:v', '1', p], check=True)
    return p

def encabezado(sl, titulo, n):
    bg = sl.background.fill; bg.solid(); bg.fore_color.rgb = FONDO
    bar = sl.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, Inches(SW), Inches(1.05)); bar.fill.solid(); bar.fill.fore_color.rgb = RGBColor(0xff, 0xff, 0xff); bar.line.fill.background()
    lin = sl.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, Inches(1.05), Inches(SW), Inches(0.06)); lin.fill.solid(); lin.fill.fore_color.rgb = AZUL; lin.line.fill.background()
    caja_texto(sl, 0.5, 0.2, 9.5, 0.75, titulo, 32, AZUL, True)
    sl.shapes.add_picture(LOGO, Inches(SW - 2.75), Inches(0.12), height=Inches(0.82))
    caja_texto(sl, 0.5, 7.0, 8, 0.35, 'AgroBot · Universidad Técnica de Ambato · FISEI', 11, GRIS)
    caja_texto(sl, SW - 1.3, 7.0, 0.9, 0.35, str(n), 11, GRIS, align=PP_ALIGN.RIGHT)

ZONA = (0.5, 1.4, SW - 1.0, 4.9)      # x, y, w, h del contenido (abajo quedan 1,2 in para subtítulos en el video)

def crear_pptx():
    prs = Presentation(); prs.slide_width, prs.slide_height = Inches(SW), Inches(SH)
    blank = prs.slide_layouts[6]
    clips = {}
    for n, e in enumerate(ESCENAS, 1):
        sl = prs.slides.add_slide(blank); L = e['layout']; x, y, w, h = ZONA
        if L == 'portada':
            bg = sl.background.fill; bg.solid(); bg.fore_color.rgb = RGBColor(0xff, 0xff, 0xff)
            sl.shapes.add_picture(LOGO, Inches(0.6), Inches(0.4), height=Inches(1.35))
            caja_texto(sl, 0.6, 2.05, 6.6, 1.0, e['titulo'], 54, AZUL, True)
            caja_texto(sl, 0.6, 3.1, 6.5, 1.8, e['sub'], 20, TINTA)
            caja_texto(sl, 0.6, 5.05, 6.5, 1.2, 'Autor: Luis David Gómez Ruiz\nTutor: Ing. Edgar Patricio Córdova\nCarrera de Telecomunicaciones · FISEI · noviembre de 2026', 15, GRIS)
            imagen(sl, e['img'], 7.4, 0.4, 5.5, 6.7)
        else:
            encabezado(sl, e['titulo'], n)
            if L == 'texto':
                viñetas(sl, x, y + 0.1, w, h, e['bullets'], 24)
            elif L == 'imagen':
                imagen(sl, e['img'], x, y, w, h)
            elif L == 'img_texto':
                imagen(sl, e['img'], x, y, 6.6, h); viñetas(sl, x + 6.9, y + 0.2, w - 6.9, h, e['bullets'], 19)
            elif L == 'dos_img':
                viñetas(sl, x, y + 0.1, 5.2, h, e['bullets'], 19); imagen(sl, e['img'], x + 5.4, y, 3.6, h); imagen(sl, e['img2'], x + 9.1, y, 3.2, h)
            elif L == 'tabla':
                filas = e['filas']; t = sl.shapes.add_table(len(filas), 2, Inches(x), Inches(y), Inches(w), Inches(0.62 * len(filas))).table
                t.columns[0].width = Inches(3.9); t.columns[1].width = Inches(w - 3.9)
                for i, fila in enumerate(filas):
                    for j, v in enumerate(fila):
                        c = t.cell(i, j); c.text = v; pr = c.text_frame.paragraphs[0]; pr.runs[0].font.size = Pt(17); pr.runs[0].font.name = FUENTE
                        pr.runs[0].font.bold = (i == 0); pr.runs[0].font.color.rgb = RGBColor(0xff, 0xff, 0xff) if i == 0 else TINTA
                        c.fill.solid(); c.fill.fore_color.rgb = AZUL if i == 0 else (RGBColor(0xff, 0xff, 0xff) if i % 2 else RGBColor(0xe9, 0xf0, 0xf7))
            elif L == 'plagas':
                cw = (w - 0.6) / 4
                for k, (im, cap) in enumerate(zip(e['imgs'], e['caps'])):
                    png = f'{OUT}/poster/plaga_{k}.jpg'; Image.open(im).convert('RGB').save(png, quality=90)
                    imagen(sl, png, x + k * (cw + 0.2), y, cw, h - 0.6)
                    caja_texto(sl, x + k * (cw + 0.2), y + h - 0.5, cw, 0.4, cap, 15, GRIS, align=PP_ALIGN.CENTER)
            elif L in ('clip_der', 'clip_full', 'clip_izq'):
                if L == 'clip_der':
                    viñetas(sl, x, y + 0.1, 6.2, h, e['bullets'], 19); r = rect_clip(x + 6.5, y, w - 6.5, h, e['clip_aspect'])
                elif L == 'clip_full':
                    r = rect_clip(x, y, w, h, e['clip_aspect'])
                else:
                    r = rect_clip(x, y, 6.4, h, e['clip_aspect']); viñetas(sl, x + 6.7, y + 0.1, w - 6.7, 2.9, e['bullets'], 17)
                    imagen(sl, e['img'], x + 6.7, y + 3.0, w - 6.7, h - 3.0)
                if LIVIANA: sl.shapes.add_picture(poster(e['clip'], n), Inches(r[0]), Inches(r[1]), Inches(r[2]), Inches(r[3]))   # sin video: cabe en GitHub
                else: sl.shapes.add_movie(e['clip'], Inches(r[0]), Inches(r[1]), Inches(r[2]), Inches(r[3]), poster_frame_image=poster(e['clip'], n), mime_type='video/mp4')
                clips[n - 1] = [int(round(v * PPI)) for v in r]
        sl.notes_slide.notes_text_frame.text = ' '.join(e['frases'])
    prs.save(f'{OUT}/{NOMBRE}.pptx')
    json.dump(clips, open(f'{OUT}/clips_rect.json', 'w'))
    with open(f'{OUT}/guion.md', 'w', encoding='utf-8') as g:
        g.write('# Guion del video final de AgroBot\n\n')
        for n, e in enumerate(ESCENAS, 1):
            g.write(f'## Diapositiva {n}. {e["titulo"]}\n\n' + '\n\n'.join(e['frases']) + '\n\n')
    print('pptx y guion listos')

def renderizar():
    subprocess.run(['soffice', '--headless', '--convert-to', 'pdf', '--outdir', OUT, f'{OUT}/AgroBot_presentacion_final.pptx'], check=True, capture_output=True)
    for f in glob.glob(f'{OUT}/slides/base-*.png'): os.remove(f)
    subprocess.run(['pdftoppm', '-png', '-r', str(PPI), f'{OUT}/AgroBot_presentacion_final.pdf', f'{OUT}/slides/base'], check=True)
    fs = sorted(glob.glob(f'{OUT}/slides/base-*.png')); print('diapositivas', len(fs), Image.open(fs[0]).size); return fs

# ---------------------------------------------------------------- voz y video
def hablado(t):
    for a, b in (('±', 'más o menos '), ('°', ' grados'), ('ms', ' milisegundos'), ('px', ' píxeles'), ('×', ' por ')): t = t.replace(a, b)
    return t

def font(sz, b=False): return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf' % ('-Bold' if b else ''), sz)

def con_subtitulo(base, txt):
    im = base.copy(); d = ImageDraw.Draw(im); W, H = im.size
    f = font(34); palabras, lineas, cur = txt.split(), [], ''
    for p in palabras:
        t = (cur + ' ' + p).strip()
        if d.textlength(t, font=f) > W - 260: lineas.append(cur); cur = p
        else: cur = t
    lineas.append(cur)
    h = 26 + 46 * len(lineas); y0 = H - h - 14
    d.rounded_rectangle([110, y0, W - 110, H - 14], 14, fill=(16, 32, 52))
    for i, l in enumerate(lineas): d.text((W // 2, y0 + 16 + 46 * i), l, font=f, fill=(255, 255, 255), anchor='ma')
    return im

def hablar():
    from piper import PiperVoice
    voz = PiperVoice.load(HOME + '/video_ldz/voz/es_MX-claude-high.onnx'); items = []
    for ei, e in enumerate(ESCENAS):
        for fi, fr in enumerate(e['frases']):
            p = f'{OUT}/wav/{ei:02d}_{fi:02d}.wav'
            if not os.path.exists(p):
                w = wave.open(p, 'wb'); voz.synthesize_wav(hablado(fr), w); w.close()
            n = wave.open(p); dur = n.getnframes() / n.getframerate(); n.close(); items.append((ei, fi, fr, p, dur))
    return items

def video():
    fs = renderizar(); clips = {int(k): v for k, v in json.load(open(f'{OUT}/clips_rect.json')).items()}
    items = hablar(); ultimo = {}
    for ei, fi, *_ in items: ultimo[ei] = fi
    segs, srt, t, cum = [], [], 0.0, {}
    ts = lambda x: f'{int(x // 3600):02d}:{int(x % 3600 // 60):02d}:{int(x % 60):02d},{int(round((x % 1) * 1000)):03d}'
    for k, (ei, fi, fr, p, dur) in enumerate(items):
        base = Image.open(fs[ei]).convert('RGB').resize((1920, 1080))
        D = dur + 0.35 + (0.9 if fi == ultimo[ei] and ei != len(ESCENAS) - 1 else 0)
        png = f'{OUT}/slides/{ei:02d}_{fi:02d}.png'; con_subtitulo(base, fr).save(png)
        seg = f'{OUT}/seg/{ei:02d}_{fi:02d}.mp4'; off = cum.get(ei, 0.0)
        cmd = ['ffmpeg', '-y', '-loglevel', 'error', '-loop', '1', '-framerate', '25', '-t', f'{D:.3f}', '-i', png]
        if ei in clips:
            x, y, w, h = clips[ei]; c = ESCENAS[ei]['clip']
            ln = float(subprocess.check_output(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', c]))
            cmd += ['-stream_loop', '-1', '-ss', f'{off % max(ln, 1):.3f}', '-i', c, '-i', p, '-filter_complex',
                    f'[1:v]fps=25,scale={w}:{h}[v];[0:v][v]overlay={x}:{y}:shortest=1,format=yuv420p[o];[2:a]aresample=44100,apad[a]', '-map', '[o]', '-map', '[a]']
        else:
            cmd += ['-i', p, '-vf', 'format=yuv420p', '-af', 'aresample=44100,apad', '-map', '0:v', '-map', '1:a']
        cmd += ['-t', f'{D:.3f}', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '21', '-r', '25', '-c:a', 'aac', '-b:a', '192k', '-ac', '1', seg]
        subprocess.run(cmd, check=True); cum[ei] = off + D; segs.append(seg)
        srt.append(f'{k + 1}\n{ts(t)} --> {ts(t + dur)}\n{fr}\n'); t += D
    open(f'{OUT}/subtitulos.srt', 'w', encoding='utf-8').write('\n'.join(srt))
    open(f'{OUT}/lista.txt', 'w').write(''.join(f"file '{s}'\n" for s in segs))
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'concat', '-safe', '0', '-i', f'{OUT}/lista.txt', '-c', 'copy', '-movflags', '+faststart', f'{OUT}/AgroBot_video_tesis.mp4'], check=True)
    print('duración', round(t, 1), 's =', round(t / 60, 2), 'min')

if __name__ == '__main__':
    modo = sys.argv[1] if len(sys.argv) > 1 else 'todo'
    if modo in ('pptx', 'todo'): crear_pptx()
    if modo in ('video', 'todo'): video()
