#!/usr/bin/env python3
"""Anexos A–G de la tesis AgroBot (Word + PDF), máximo 2 páginas cada uno, en tablas y figuras.

Datos al 12-nov-2026, tomados del repositorio, de los registros del robot (~/.agrobot) y de los informes en docs/.
Sin contraseñas ni claves. Uso: python3 generar_anexos.py
"""
import glob, json, os, subprocess
from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

H = os.path.expanduser('~')
R = H + '/Desktop/AGROBOT/agrobot_system_2/'
OUT = R + 'docs/ANEXOS_TESIS_20261112/'
FIG = R + 'docs/ANEXOS_TESIS_20261108/figuras/'
AS = R + 'tools/video_tesis/assets/'
SIM = R + 'docs/REPORTE_SIMULACION_20261023/figs/'
PL = R + 'docs/FOTOS_PLAGAS_20261112/'
WA = sorted(glob.glob(H + '/.agrobot/pruebas/invernadero_arandano_20261111/fotos_whatsapp/*.jpeg'))
AZUL = RGBColor(0x1d, 0x4e, 0x7e)

class Anexo:
    def __init__(self, letra, titulo, intro):
        self.d = Document(); self.letra = letra; self.nt = 0; self.nf = 0
        st = self.d.styles['Normal']; st.font.name = 'Calibri'; st.font.size = Pt(10)
        st.paragraph_format.space_after = Pt(3); st.paragraph_format.line_spacing = 1.05
        for s in self.d.sections:
            s.left_margin = s.right_margin = Cm(2.0); s.top_margin = Cm(1.6); s.bottom_margin = Cm(1.5)
            p = s.footer.paragraphs[0]; p.text = f'AgroBot · Anexo {letra} · Universidad Técnica de Ambato, FISEI'; p.runs[0].font.size = Pt(8)
        p = self.d.add_paragraph(); r = p.add_run(f'Anexo {letra}. {titulo}'); r.bold = True; r.font.size = Pt(15); r.font.color.rgb = AZUL
        self.p(intro, i=True, size=9)
    def h(self, t):
        p = self.d.add_paragraph(); p.paragraph_format.space_before = Pt(6); r = p.add_run(t); r.bold = True; r.font.size = Pt(11.5); r.font.color.rgb = AZUL
    def p(self, t, b=False, i=False, size=None):
        p = self.d.add_paragraph(); r = p.add_run(t); r.bold = b; r.italic = i
        if size: r.font.size = Pt(size)
        return p
    def vin(self, items):
        for t in items:
            p = self.d.add_paragraph(t, style='List Bullet'); p.paragraph_format.space_after = Pt(1)
            for r in p.runs: r.font.size = Pt(9.5)
    def tabla(self, titulo, cab, filas, anchos):
        self.nt += 1; self.p(f'Tabla {self.letra}-{self.nt}. {titulo}', b=True, size=9)
        t = self.d.add_table(rows=1, cols=len(cab)); t.style = 'Light Grid Accent 1'; t.alignment = WD_TABLE_ALIGNMENT.CENTER
        for c, v in zip(t.rows[0].cells, cab): c.text = v
        for f in filas:
            for c, v in zip(t.add_row().cells, f): c.text = str(v)
        # anchos fijos: sin esto LibreOffice reparte las columnas por igual y las tablas crecen en alto
        t.autofit = False
        tblPr = t._tbl.tblPr; lay = OxmlElement('w:tblLayout'); lay.set(qn('w:type'), 'fixed'); tblPr.append(lay)
        for gc, w in zip(t._tbl.tblGrid.findall(qn('w:gridCol')), anchos): gc.set(qn('w:w'), str(int(w * 567)))
        for k, row in enumerate(t.rows):
            trPr = row._tr.get_or_add_trPr()
            ns = OxmlElement('w:cantSplit'); trPr.append(ns)                       # una fila no se parte entre páginas
            if k == 0: hd = OxmlElement('w:tblHeader'); trPr.append(hd)          # la cabecera se repite si la tabla cruza de página
            for c, w in zip(row.cells, anchos):
                c.width = Cm(w)
                for pp in c.paragraphs:
                    pp.paragraph_format.space_after = Pt(0)
                    if k == 0 or k < len(t.rows) - 1: pp.paragraph_format.keep_with_next = True
                    for r in pp.runs: r.font.size = Pt(8.5)
        self.d.add_paragraph().paragraph_format.space_after = Pt(0)
    def figs(self, items, alto=5.2):
        """items: [(ruta, pie)] en una fila sin bordes."""
        items = [(r, p) for r, p in items if r and os.path.exists(r)]
        if not items: return
        t = self.d.add_table(rows=2, cols=len(items)); t.alignment = WD_TABLE_ALIGNMENT.CENTER
        ancho = 17.0 / len(items) - 0.3
        for k, (ruta, pie) in enumerate(items):
            self.nf += 1
            c = t.cell(0, k); c.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
            from PIL import Image
            iw, ih = Image.open(ruta).size; w = min(ancho, alto * iw / ih)
            c.paragraphs[0].add_run().add_picture(ruta, width=Cm(w))
            c2 = t.cell(1, k); c2.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
            r = c2.paragraphs[0].add_run(f'Figura {self.letra}-{self.nf}. {pie}'); r.italic = True; r.font.size = Pt(8)
    def guardar(self, nombre):
        ruta = OUT + nombre + '.docx'; self.d.save(ruta)
        subprocess.run(['soffice', '--headless', '--convert-to', 'pdf', '--outdir', OUT, ruta], check=True, capture_output=True)
        n = subprocess.check_output(['pdfinfo', OUT + nombre + '.pdf']).decode()
        pags = int([l for l in n.splitlines() if l.startswith('Pages')][0].split()[-1])
        print(f'{nombre}: {pags} página(s)'); return pags

def jpg(src, dst):
    """Las .webp se convierten a .jpg para Word."""
    from PIL import Image
    if not os.path.exists(dst): Image.open(src).convert('RGB').save(dst, quality=88)
    return dst

os.makedirs(OUT + 'fig', exist_ok=True)

# ======================================================================== A
a = Anexo('A', 'Planos y componentes', 'Diseño mecánico, componentes y cableado del robot. La masa y el centro de gravedad son estimados. Pendiente: plano acotado del chasis, plano de la PCB, diagrama del cableado actual y medición real de la masa.')
a.figs([(FIG + 'chasis_cad_1.png', 'Diseño CAD (SolidWorks).'), (FIG + 'chasis_soldado.png', 'Chasis soldado, vista inferior.'),
        (WA[23] if len(WA) > 23 else None, 'Robot completo en Vitro Plantas (11-nov).')], alto=3.7)
a.tabla('Medidas del robot.', ['Medida', 'Valor', 'Medida', 'Valor'], [
    ('Ancho eje delantero (fijo)', '28 cm', 'Masa total', '12 kg (estimada)'),
    ('Ancho módulo trasero', '38 cm', 'Centro de gravedad', '0,20 m de altura (estimado)'),
    ('Distancia entre ejes', '37 cm', 'Pivote trasero', 'Libre, ±20°'),
    ('Longitud total', '49 cm', 'Altura del LIDAR', '37,5 cm')], [4.2, 3.4, 4.2, 5.2])
a.tabla('Componentes.', ['Subsistema', 'Componente', 'Conexión'], [
    ('Cómputo', 'NVIDIA Jetson Orin Nano Developer Kit (15 W)', '—'),
    ('Motores y servo', 'ESP32-C3 + 4 puentes TA6586 + servo de inclinación', 'Wi-Fi UDP y BLE con la Jetson'),
    ('LIDAR', 'Slamtec RPLIDAR C1 (360°, ~12 m)', 'USB (CP2102N)'),
    ('Cámara', 'NexiGo N60 (640×480 / 1280×720 con zoom)', 'USB'),
    ('Nodo de sensores', 'Heltec WiFi LoRa 32 V3 + MPU6050 + SCD40 (dañado)', 'Wi-Fi UDP a la Jetson; LoRaWAN'),
    ('Red del robot', 'Adaptador USB RTL8188FTV (punto de acceso JETSON_AP)', 'USB, driver rtl8xxxu'),
    ('Gateway y servidor', 'Heltec HT-M02 V2 + Raspberry Pi con ChirpStack', 'Wi-Fi de la red local')], [3.2, 8.3, 5.5])
a.figs([(FIG + 'esquematico_proteus.png', 'Esquemático de control (Proteus).'), (FIG + 'cableado_historico.png', 'Cableado de una etapa anterior.')], alto=3.5)
a.tabla('Cambios del robot actual frente a las figuras de etapas anteriores.', ['Elemento', 'Etapa anterior', 'Robot actual'], [
    ('Computador', 'Jetson Nano', 'Jetson Orin Nano'), ('LIDAR', 'RPLIDAR A1', 'RPLIDAR C1 por USB'),
    ('Control de motores', 'ESP32 LoRa V3', 'ESP32-C3'), ('Enlace con la Jetson', 'UART', 'Wi-Fi y Bluetooth'), ('Puentes de motor', 'Genéricos', 'TA6586')], [4.5, 5.5, 7])
pa = a.guardar('ANEXO_A')

# ======================================================================== B
b = Anexo('B', 'Firmware y comunicaciones', 'Versiones, pines y configuración al 12-nov-2026, tomadas del código. Las contraseñas y claves no se incluyen.')
b.tabla('Firmware y software por equipo.', ['Equipo', 'Software', 'Versión / commit'], [
    ('ESP32-C3', 'Firmware FUSIÓN (motores, servo, failsafe)', 'v5 · ebde6ea'),
    ('Heltec LoRa 32 V3', 'esp32_lora_v3: IMU por UDP a 20 Hz, LoRaWAN AU915 y OLED', '4f08332'),
    ('Jetson Orin Nano', 'ROS 2 Jazzy + backend aiohttp (dashboard :8080) + Ultralytics', '9fbe1fb'),
    ('Raspberry Pi', 'ChirpStack v4 en Docker + servidor del dashboard (:8092)', '—'),
    ('Gateway HT-M02', 'Packet forwarder Semtech UDP → Pi :1700', 'AU915, sub-banda 0')], [3.4, 9.6, 4])
b.tabla('Pines de la ESP32-C3 (los de motor se pueden cambiar desde el panel).', ['Función', 'GPIO', 'Función', 'GPIO'], [
    ('Delantera derecha FI / BI', '0 / 1', 'Trasera izquierda FI / BI', '4 / 5'), ('Delantera izquierda FI / BI', '3 / 2', 'Trasera derecha FI / BI', '6 / 7'),
    ('Foco LED', '8', 'Servo de cámara', '10')], [5, 2.5, 5, 2.5])
b.tabla('Pines del nodo Heltec V3.', ['Función', 'Pines'], [
    ('Sensores I2C (MPU6050 funciona; SCD40 dañado desde el 23-oct)', 'SDA 41, SCL 42'), ('Pantalla OLED', 'SDA 17, SCL 18, RST 21, Vext 36')], [11, 6])
b.tabla('Nodos de ROS 2 (paquete agrobot_navigation).', ['Nodo', 'Función'], [
    ('sllidar_node', 'Lee el LIDAR C1'), ('laser_filters / filtro_scan_slam', 'Quita el chasis y filtra el scan para el SLAM'),
    ('rf2o_laser_odometry', 'Odometría láser (velocidad de avance)'), ('robot_localization (EKF)', 'Fusiona LIDAR, cámara, IMU, órdenes y reposo a 20 Hz'),
    ('slam_toolbox', 'Mapa de ocupación (celda de 5 cm)'), ('Nav2 (planner + DWB)', 'Navegación a metas del mapa')], [5.5, 11.5])
b.tabla('Enlaces de comunicación.', ['Enlace', 'Medio', 'Uso', 'Seguridad ante fallas'], [
    ('Jetson ↔ C3', 'Wi-Fi UDP (JETSON_AP) y BLE', 'Órdenes a motores y estado', 'La C3 para a 1,2 s sin órdenes (Wi-Fi) o 0,4 s (USB)'),
    ('Nodo V3 → Jetson', 'Wi-Fi UDP 5001, 20 Hz', 'Giroscopio para el EKF', 'El vigía detecta datos viejos'),
    ('Nodo V3 → Pi', 'LoRaWAN AU915, OTAA, cada 30 s', 'Telemetría (17 bytes) y alertas', 'Independiente del control'),
    ('Operador → robot', 'Dashboard web y VPN Tailscale', 'Mando, misiones, datos', 'Parada de emergencia por WebSocket y HTTP')], [2.8, 4.3, 4.4, 5.5])
b.h('Puesta en marcha')
b.vin(['Revisar baterías, corte físico de motores y fijación del LIDAR.',
       'Encender la Jetson: agrobot-ros, agrobot-backend y el punto de acceso arrancan solos. Abrir 192.168.12.1:8080 en la red JETSON_AP.',
       'Encender la C3 y el nodo; comprobar todo en verde en la salud del panel. Elegir el perfil del cultivo (Coles o Arándano).',
       'Dejar el robot quieto ~10 s (el vigía de pose toma la primera pose). Para flashear la C3, detener antes agrobot-backend.'])
pb = b.guardar('ANEXO_B')

# ======================================================================== C
c = Anexo('C', 'Datos y modelos de visión', 'Datos, entrenamiento y métricas de los modelos del robot. Todos los modelos nuevos son YOLO11. Las métricas de test pueden ser optimistas porque fotos parecidas quedaron repartidas entre particiones; con las 5 fotos de campo del 12-nov, PLAGASX v8 confundía mosca blanca con mildiu y pudrición con pulgón.')
c.tabla('Modelos de visión.', ['', 'PLAGASX v8 (instalado)', 'ARANDANOS v1', 'Contexto', 'PLAGASX v10 (en curso)'], [
    ('Arquitectura', 'YOLOv8s', 'YOLO11s', 'YOLO11s', 'YOLO11m'), ('Clases', '11 (col)', '10', '9', '12 (+ trips)'),
    ('Entrenamiento', '80 ép. · 800 px · 28-oct', '157 ép. · 960 px · 11-nov', '81 ép. · 800 px · 9-nov', 'hasta 8,5 h · 960 px · 12-nov'),
    ('Resolución en el robot', '480 px', '960 px', '640 px', 'por medir'),
    ('Validación mAP50 / mAP50-95', '0,83 / 0,66', '0,79 / 0,46', '0,68 / 0,44', 'pendiente'),
    ('Tiempo en la Jetson', '41 ms', '135 ms', '63 ms', 'por medir')], [3.4, 3.4, 3.3, 3.2, 3.7])
c.tabla('Cajas de PLAGASX v8 por clase (entrenamiento / validación / prueba).', ['Clase', 'Cajas', 'Clase', 'Cajas'], [
    ('col_sana', '144 / 41 / 15', 'col_malformacion', '143 / 39 / 18'), ('col_amarillamiento', '16 / 4 / 5', 'col_pudricion', '0 / 0 / 0 → +13 en v10'),
    ('col_manchas_cafe', '142 / 41 / 30', 'col_mosca_blanca', '1169 / 241 / 230 → +27 en v10'), ('col_dano_insecto', '6555 / 884 / 718', 'col_pulgon', '1176 / 78 / 67'),
    ('col_mildiu_velloso', '199 / 64 / 27', 'col_oruga · col_pulguilla', '742 / 76 / 72 · 357 / 12 / 15')], [3.6, 3.4, 4.0, 6.0])
c.tabla('Origen de los datos.', ['Modelo', 'Fuentes', 'Calidad de las cajas'], [
    ('PLAGASX', 'Fotos propias de coles (170), cajas reales de daño y mosca blanca (2193), IP102 (2070), coliflor externa (1000), fondos (551); v10: +4 fotos de campo ×13', 'Propias automáticas por color; externas de imagen completa'),
    ('ARANDANOS', '1244 fotos de fruto (HANDset, Roboflow, HuggingFace; deduplicadas por hash), 8 plagas del IP102, hojas sanas y senderos propios; v2: +81 negativos de Vitro ×3', 'Cajas reales en frutos; madurez asignada por tono'),
    ('Contexto', 'Fotos de la cámara baja del robot (dataset agrobot-contexto-robot)', 'Cajas propias')], [2.4, 10.0, 4.6])
c.tabla('Aumentación del entrenamiento (v10 / v2).', ['Técnica', 'Valor', 'Técnica', 'Valor'], [
    ('Mosaico', '1,0 (sin él las últimas 15 épocas)', 'Rotación', '±10° (v10) · ±8° (v2)'), ('Mixup', '0,15 · 0,10', 'Traslación / escala', '12 % / 50–60 %'),
    ('Color HSV', 'H 0,015 · S 0,6–0,7 · V 0,4', 'Borrado aleatorio', '30 %'), ('Espejo', 'horizontal 0,5; vertical 0,3 (v10)', 'Fuera de línea (fotos de campo)', '12 variantes: recorte, giro, luz, desenfoque, ruido')], [3.2, 5.3, 3.6, 4.9])
c.figs([(H + '/.agrobot/pruebas/invernadero_arandano_20261111/inferencias/05.jpg', 'ARANDANOS en Vitro Plantas.'), (jpg(PL + 'orig/1.webp', OUT + 'fig/col_mosca.jpg'), 'Mosca blanca en col (12-nov).'),
        (PL + 'filtro/1_colonias.jpg', 'Filtro top-hat de colonias.')], alto=2.8)
pc = c.guardar('ANEXO_C')

# ======================================================================== D
dd = Anexo('D', 'Simulación', 'Simulador propio (simulador_pasivo/) que ejecuta los mismos módulos del robot con sensores y chasis simulados.')
dd.tabla('Qué simula.', ['Elemento', 'Modelo'], [
    ('Código', 'Los mismos módulos de la Jetson: freno por LIDAR, sendero, explorador, conductor por pulsos'),
    ('LIDAR', '720 rayos con ruido y alcance de 8 m; distingue lo alto (paredes) de lo que solo ve la cámara'),
    ('Cámara', 'Imagen sintética: tierra, plantas, paredes, paja; servo de inclinación'),
    ('Chasis', 'Masa 12,65 kg, fricción 0,80, eficiencias 0,62 adelante / 0,91 atrás, patinaje lateral'),
    ('SLAM', 'Con deriva; mundos generados o digitalizados desde mapas reales')], [3, 14])
dd.tabla('Calibración con pulsos reales (23-oct).', ['Pulso', 'Real: giro / avance', 'Simulado: giro / avance'], [
    ('Adelante', '+0,4° / +9,1 cm', '+0,0° / +9,0 cm'), ('Atrás', '+0,3° / −14,2 cm', '+0,1° / −13,8 cm'),
    ('Derecha', '−4,0° / +5,3 cm', '−3,0° / +2,8 cm'), ('Izquierda', '+2,3° / +0,0 cm', '+3,1° / +2,8 cm')], [4, 6.5, 6.5])
dd.figs([(SIM + 'calibracion.png', 'Pulsos reales frente a simulados.'), (R + 'docs/digitalizacion/plano_real_1618.png', 'Mapa real digitalizado para el simulador.')], alto=4.6)
dd.tabla('Resultados en simulación (semillas nuevas).', ['Experimento', 'Resultado'], [
    ('Entrenamiento CEM, 23-oct', 'Mejor candidato 78,2 % de éxito y 0,29 contactos/episodio, frente a 56,4 % y 0,78 de la configuración de ese día'),
    ('Entrenamiento de 7 h, 3-nov', 'Ganó la configuración actual (costo −2,20 frente a −2,09 y −1,97): no se cambió el robot'),
    ('Digitalización de mapas', 'Muros recuperados: 92,8 % sin deriva (error 4 cm), 82,2 % con 1 % de deriva, 71,3 % con 3 %')], [4.2, 12.8])
dd.tabla('Ejecución.', ['Acción', 'Comando o lugar'], [
    ('Abrir el simulador', 'Dashboard → pestaña Simulación → Iniciar (proceso aparte)'),
    ('Correr un escenario', 'python3 simulador_pasivo/correr.py --mundo <nombre>'),
    ('Entrenar parámetros', 'python3 simulador_pasivo/entrenar_intensivo.py (salida en ~/.agrobot/aprendizaje/)'),
    ('Mundo desde datos reales', 'Dashboard → «Subir datos recolectados a simulador»')], [4.2, 12.8])
dd.p('Validez: los resultados previos al 22-oct usaban giros invertidos y los del explorador previos al 2-nov tenían 4 diferencias con el robot; no se usan. '
     'El simulador no modela el giro de lazo cerrado; todo cambio se confirma en campo.', i=True, size=9)
pd_ = dd.guardar('ANEXO_D')

# ======================================================================== E
e = Anexo('E', 'Pruebas experimentales', 'Fichas resumidas de cada ensayo físico. P1–P12 se definieron después y se aplican como referencia; «No evaluable»: el ensayo no midió lo que pide el criterio. Limitaciones: sin registro de clima ni sensor de contacto; la Jetson no tiene reloj propio; CO₂, temperatura y humedad son simulados desde el 23-oct (SCD40 dañado).')
e.tabla('Ensayos interiores.', ['Ensayo', 'Fecha · lugar', 'Resultado', 'Dictamen'], [
    ('I-1 SLAM con regreso', '22-oct · sala', 'Cierre de 36 cm en 23 m (≈1,5 %)', 'No evaluable (P4)'),
    ('I-2 Plantas en maceta', '23-oct · laboratorio', 'Maceta 20/20; 0/30 falsas', 'Sin criterio'),
    ('I-3 Recorrido en L', '28-oct · 2,70 × 2,40 m', 'Especie 17/17; giro de 360° cortado a 158°', 'No evaluable (P3)'),
    ('I-4 L con armarios', '28-oct', '16–17 de 18 avisos falsos', 'No cumple P6; corregido')], [3.3, 3.6, 6.3, 3.8])
e.tabla('Ensayos exteriores.', ['Ensayo', 'Fecha · lugar', 'Resultado', 'Dictamen'], [
    ('E-1 Patio', '27-oct', '5 plantas inspeccionadas, 7 fallidas; giro 1,5° de 6°', 'No cumple el giro'),
    ('E-2 Huerta', '28-oct', '16,2 min en autonomía; 24,8 m', 'No cumple P1'),
    ('E-3 Tierra removida', '29-oct', '0 plagas falsas en 14 min; 77 % parado', 'No cumple P1'),
    ('E-4 Vitro Plantas', '11-nov · vivero', 'Mapa 9,5 × 12,1 m; ~7,5 m; detecciones falsas (sin fruto) → filtro 11 → 0', 'P6 no evaluable'),
    ('E-5 Huerta de coles', '11-nov', '15,8 min; ~41 m; mapa 13,8 × 9,1 m con vuelta cerrada; datos sin pérdidas', 'P1 no evaluable')], [3.3, 3.0, 7.0, 3.7])
e.tabla('Ensayos de LoRaWAN.', ['Ensayo', 'Fecha', 'Resultado', 'Dictamen'], [
    ('L-1 Banco', '3-nov', 'OTAA correcto; 2 mensajes, −29 y −31 dBm', 'Parcial (P8)'),
    ('L-2 Campo (E-5)', '11-nov', '34/34 paquetes; −53 a −82 dBm; SNR 9,2–14,5 dB', 'Parcial (P8)'),
    ('L-3 Falla de energía', '11-nov', 'Gateway caído ~6 h; tras reiniciar la Pi, el nodo volvió a unirse', 'Requiere respaldo de energía')], [3.3, 2.0, 7.9, 3.8])
e.figs([(AS + 'mapa_coles.png', 'Mapa SLAM de E-5 con la trayectoria.'), (AS + 'mapa_vitro.png', 'Mapa SLAM de E-4.'), (AS + 'lora_rssi.png', 'RSSI LoRa durante E-5.')], alto=3.3)
e.tabla('Dónde están los registros (Jetson, ~/.agrobot/).', ['Ensayo', 'Carpeta'], [
    ('I-1 · I-3 · I-4', 'campo/20261022_213805_5dde93 · pruebas/L_20261028 · pruebas/L2_20261028'),
    ('E-1 · E-2 · E-3', 'datos/20261027_215547 · datos/20261028_141932 · campo/20261029_142951_a81509'),
    ('E-4', 'datos/20261111_190208 · campo/20261111_190547_1a875c · maps/mision_20261111_1905'),
    ('E-5', 'datos/19700101_000140 (reloj sin hora) · campo/20261111_220339_4908f5 · maps/mision_20261111_2205 y _2210')], [3, 14])
pe = e.guardar('ANEXO_E')

# ======================================================================== F
f = Anexo('F', 'Trazabilidad', 'Relación entre versiones, cambios y pruebas. Repositorio git desde el 17-sep-2026 (183 commits al 12-nov), remoto github.com/DAvolmx/AGROBOT_V1, rama mejoras-auditoria-20261007.')
f.tabla('Commits clave.', ['Fecha', 'Commit', 'Cambio'], [
    ('23-oct', '11f6222', 'Simulador calibrado con pulsos reales y zoom automático'), ('23-oct', '41d3c4c', 'Entrenamiento ronda 2 (80 % en validación) y reporte'),
    ('26-oct', 'ba75ea6', 'Mapa del SLAM más limpio (registro en dos etapas)'), ('2-nov', '74afce5', 'Vigía de pose, giro a rumbo, atasco escalonado; simulador fiel'),
    ('3-nov', '4f08332', 'Red LoRaWAN en AU915 y OLED en el nodo'), ('8/9-nov', '4fef423 · 9fb6ec2', 'Primera versión de los anexos'),
    ('11-nov', 'a018391', 'ARANDANOS, filtros por cultivo, vigía tras reinicio, metas Nav2 registradas'),
    ('11-nov', '60009cc', 'Informe de redes de visión'), ('12-nov', '9fbe1fb', 'Informe final, presentación, video, fotos de plagas, reentrenamiento')], [2, 3.2, 11.8])
f.tabla('Modelos instalados y su huella (sha256, primeros 16 caracteres).', ['Archivo', 'sha256', 'Origen'], [
    ('agrobot_plagasx_det.pt', 'be961a0725e2fa6a', 'Kaggle plagasx v8 (28-oct)'), ('agrobot_plagasx_cls.pt', 'cb545d00e40c4b1e', 'Kaggle plagasx v8'),
    ('agrobot_arandanos_det.pt', '5277a41af9668620', 'Kaggle arandanos v1 (11-nov)'), ('agrobot_contexto.pt', '3810856dacdab2e7', 'Kaggle agrobot-contexto (9-nov)')], [5.2, 4.3, 7.5])
f.tabla('Prueba → versión → documento.', ['Prueba', 'Código vigente', 'Documento'], [
    ('I-1 a I-4, E-1 a E-3', 'antes de 74afce5', 'docs/ANEXOS_TESIS_20261108, PRUEBA_EXTERIOR_20261027, SESION_CAMPO_20261029'),
    ('L-1', '4f08332', 'TESIS_COMPLEMENTO_20261102'), ('E-4', 'a018391 (ARANDANOS v1)', 'MISION_FOTOS_ARANDANO_20261111'),
    ('E-5, L-2, L-3', 'a018391 + filtros', 'PRUEBA_COLES_EXTERIOR_20261111, INFORME_FINAL_TESIS_20261112'),
    ('Fotos 12-nov', '9fbe1fb', 'FOTOS_PLAGAS_20261112, INFORME_FINAL_TESIS_20261112')], [3.3, 4.2, 9.5])
f.tabla('Dónde está cada cosa.', ['Qué', 'Dónde'], [
    ('Registro de cambios', 'MANIFEST_1.md (raíz del repositorio), una entrada por cambio'), ('Datos de pruebas', 'Jetson: ~/.agrobot/{datos,campo,pruebas} (fuera de git por tamaño)'),
    ('Entrenamientos', 'Kaggle (davidgomez32): plagasx, arandanos, agrobot-contexto; salidas en ~/.agrobot/aprendizaje'),
    ('Fuera de git (>100 MB)', 'Presentación con videos y 4 datasets .npz (listados en .gitignore)')], [4, 13])
pf = f.guardar('ANEXO_F')

# ======================================================================== G
g = Anexo('G', 'Radio y normativa', 'Configuración de radio de la red LoRaWAN y su base normativa en Ecuador.')
g.tabla('Banda en Ecuador (Plan Nacional de Frecuencias v6.0, ARCOTEL, 14-jul-2021).', ['Rango', 'Atribución', 'Fuente'], [
    ('902–915 MHz', 'Telecomunicaciones móviles internacionales (IMT)', 'Nota EQA.40, p. 69'),
    ('915–928 MHz', 'Uso Determinado en Bandas Libres (UDBL), a título secundario', 'Nota EQA.45, p. 154; Res. ARCOTEL-2018-0661')], [3, 8, 6])
g.tabla('Plan de frecuencias usado: AU915 sub-banda 0 (canales 0–7 + 64).', ['Parámetro', 'Valor'], [
    ('Subida (canales 0–7, 125 kHz)', '915,2 – 916,6 MHz (máscara 0x00FF)'), ('Canal 64 (500 kHz)', '915,9 MHz'),
    ('Bajada RX1', '923,3 – 927,5 MHz'), ('Comparación US915 sub-banda 1', 'Subida 902,3–903,7 MHz: cae en banda IMT, no se usa'),
    ('Activación y periodo', 'OTAA, clase C, un mensaje cada 30 s, ADR activo'),
    ('Tiempo en el aire', '~72 ms por mensaje (30 bytes a SF7/125 kHz), bajo el límite de permanencia de 400 ms')], [5.5, 11.5])
g.tabla('Equipos de radio.', ['Equipo', 'Radio', 'Potencia de transmisión', 'Antena'], [
    ('Nodo Heltec LoRa 32 V3', 'SX1262, 902–928 MHz', 'Valor por defecto de la librería LoRaWAN (no fijado en el código)', 'Antena del kit; ganancia no documentada'),
    ('Gateway Heltec HT-M02 V2', 'SX1303 + SX1250, 8 canales', 'Hasta 26 ± 1 dBm (hoja de datos)', 'SMA, antena del kit'),
    ('Sensibilidad del gateway', '—', '—', '−125 dBm a SF7 · −139 dBm a SF12 (125 kHz)')], [3.6, 3.4, 5.5, 4.5])
g.tabla('Configuración por equipo.', ['Equipo', 'Ajuste'], [
    ('Nodo', 'Compilar con LORAWAN_REGION=4 (AU915); el código impide compilar en US915'),
    ('Gateway', 'Region AU915, sub-banda 0, servidor 192.168.0.20:1700 (packet forwarder UDP)'),
    ('ChirpStack', 'Región au915_0; perfil del dispositivo AU915 con OTAA')], [3, 14])
g.tabla('Mediciones de enlace.', ['Condición', 'RSSI', 'SNR', 'Entrega'], [
    ('Banco, 3-nov', '−29 a −31 dBm', '—', '2/2'), ('Casa, a pocos metros, 11-nov', '−31 a −42 dBm', '13,5 a 14,8 dB', '4/4 (SF7)'),
    ('Huerta, robot en movimiento, 11-nov', '−53 a −82 dBm', '9,2 a 14,5 dB', '34/34 (SF7)'), ('Huerto a 2–3 km', 'pendiente', 'pendiente', 'pendiente')], [6, 3.6, 3.6, 3.8])
g.p('Pendiente normativo: citar la norma técnica de ARCOTEL que fija la potencia máxima para UDBL en 915–928 MHz antes de afirmar un límite de potencia.', i=True, size=9)
pg = g.guardar('ANEXO_G')

subprocess.run(['pdfunite'] + [OUT + f'ANEXO_{x}.pdf' for x in 'ABCDEFG'] + [OUT + 'ANEXOS_AgroBot_A-G.pdf'], check=True)
print('páginas:', dict(zip('ABCDEFG', (pa, pb, pc, pd_, pe, pf, pg))))
