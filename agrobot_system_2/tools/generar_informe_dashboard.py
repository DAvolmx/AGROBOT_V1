"""Informe (Word) de todas las funcionalidades del dashboard de AgroBot, para la tesis. Reúne: las capturas anotadas con números rojos (tools/capturas_anotadas_dashboard.py),
el inventario de rutas/mensajes/estadísticas (tools/inventario_dashboard.py) y la especificación de cada control (tools/dashboard_spec.py).
Uso: python3 tools/generar_informe_dashboard.py   Salida: docs/INFORME_DASHBOARD/INFORME_DASHBOARD_20261027.docx"""
import json, sys
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from docx import Document
from docx.enum.section import WD_ORIENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dashboard_spec import TABS

RAIZ = Path(__file__).resolve().parents[1]
OUT = RAIZ / "docs" / "INFORME_DASHBOARD"
ANOT = OUT / "anotadas"
FIGS = OUT / "figs"; FIGS.mkdir(exist_ok=True)
INV = json.loads((OUT / "inventario.json").read_text())
ST = INV["stats"]
ROJO = RGBColor(0xC5, 0x30, 0x30)

GRUPOS = {
    "c3": "ESP32-C3 (actuador): pines, emparejado, BLE/WiFi, configuración UDP", "ws": "WebSocket de tiempo real", "video": "Flujo MJPEG y grabación de video", "drive": "Manejo manual por pulsos y modo de manejo",
    "emergencia": "Parada de emergencia", "paso": "Paso robótico (avance medido por pasos)", "sendero": "Vista del detector de senderos", "suelo": "Suelo libre visto por la cámara", "evasion": "Evasión de obstáculos",
    "nav": "Navegación con Nav2 (meta, reinicio de mapa)", "mapas": "Guardar, cargar y digitalizar mapas", "exploracion": "Exploración autónoma y regreso al inicio", "simulador": "Simulador visual",
    "giro_cerrado": "Giro de lazo cerrado con giroscopio", "maniobra_giro": "Maniobra de giro 180°/360°", "camara": "Zoom, cajas detectadas e imagen cruda", "misiones": "Misiones específicas (mapear, dataset, inspección)",
    "mision_plantas": "Misión de plantas en maceta", "vision_activa": "Visión activa (acercarse y escanear)", "mission": "Inicio/parada de la autonomía", "patrol": "Patrulla, recorrido y barridos",
    "saludo": "Saludo de demostración", "session-report": "Reporte de sesión", "models": "Modelos de detección y presets", "hardware": "Interfaces de hardware", "calibration": "Calibración de servo y giroscopio",
    "calibracion": "Calibración de la cámara", "entorno": "Clasificador de entorno", "params": "Parámetros persistentes", "lidar": "Estado del LIDAR", "obstacle_status": "Estado de la anticolisión", "costmap": "Costmap local",
    "ocupado": "¿Robot ocupado? (consulta del vigía del reloj)", "health": "Salud de componentes", "seguridad": "Modo seguro", "captures": "Capturas de plaga", "nav_cal": "Calibración de la navegación",
    "training": "Trazado de surcos, puntos de escaneo y rutas", "asistido": "Manejo asistido y aprendizaje", "comunicacion": "Calidad de la comunicación", "wifi": "Modo WiFi", "diagnostico": "Diagnóstico",
    "motor": "Pruebas de motores", "estado": "Estado completo", "etiquetado": "Herramienta de etiquetado de plagas", "recoleccion": "Recolección de campo (fotos, video, muestras)", "asistente": "Asistente", "raíz": "Páginas y archivos estáticos",
    "training.html": "Página heredada",
}
WS = {
    "snapshot": "Estado inicial completo al conectarse", "pose": "Pose del robot (SLAM o EKF)", "scan_mapa": "Scan del LIDAR en el marco del mapa, para el mapa y el minimapa", "map": "Mapa de ocupación del SLAM",
    "lidar_scan": "Scan crudo (ángulo, distancia) para el radar", "lidar_distancias": "Huecos libres adelante/atrás/izquierda/derecha", "obstacle_warning": "Aviso de frenado por obstáculo",
    "detection": "Detección de IA (clase, confianza)", "capture": "Nueva captura guardada en la galería", "captures_cleared": "La galería se vació", "env_telemetry": "Telemetría ambiental e IMU del ESP32 LoRa",
    "actuator_status": "Eco del actuador (v, ω, servo, LED, seq)", "actuator_setpoint": "Servo y LED pedidos", "drive_mode": "Modo de manejo vigente", "entorno": "Sugerencia del clasificador de entorno",
    "exploracion": "Estado del explorador (pulsos, giros, estado)", "exploracion_event": "Eventos de la exploración", "mission_state": "Misión activa/inactiva", "asistido": "Estado del manejo asistido",
    "seguridad": "Nivel del modo seguro", "health": "Salud de componentes", "system_stats": "CPU, RAM, GPU y temperaturas de la Jetson", "odom_webcam": "Odometría visual (cámara vs EKF)", "config_update": "La configuración cambió",
    "toast": "Aviso emergente para el operador", "mision_plantas": "Eventos de la misión de plantas", "misiones": "Eventos de las misiones específicas", "vision_activa": "Estado de la visión activa", "patrol_event": "Eventos de la patrulla",
    "evasion": "Estado de la evasión", "zoom_auto": "Zoom automático de la cámara", "error": "Error del backend", "emergency_stop": "Parada de emergencia (navegador → backend)", "actuator": "Servo/LED (navegador → backend)",
}


def tabla(doc, cab, filas, anchos=None, size=8.5):
    t = doc.add_table(rows=1, cols=len(cab)); t.style = "Light Grid Accent 1"
    t.autofit = False
    tblPr = t._tbl.tblPr                                           # diseño fijo: si no, Word/LibreOffice reparten el ancho a su manera e ignoran las columnas
    lay = OxmlElement("w:tblLayout"); lay.set(qn("w:type"), "fixed"); tblPr.append(lay)
    for i, c in enumerate(cab):
        t.rows[0].cells[i].text = str(c)
    trPr = t.rows[0]._tr.get_or_add_trPr(); th = OxmlElement("w:tblHeader"); th.set(qn("w:val"), "true"); trPr.append(th)     # la fila de títulos se repite en cada página
    for f in filas:
        r = t.add_row().cells
        for i, v in enumerate(f):
            r[i].text = str(v).replace("`", "")
    if anchos:
        for k, col in enumerate(t.columns):
            col.width = Cm(anchos[k])
    for row in t.rows:
        for k, c in enumerate(row.cells):
            if anchos:
                c.width = Cm(anchos[k])
            for p in c.paragraphs:
                for run in p.runs:
                    run.font.size = Pt(size)
    for c in t.rows[0].cells:
        for p in c.paragraphs:
            p.paragraph_format.keep_with_next = True
            for run in p.runs:
                run.bold = True
    for row in t.rows:
        cs = OxmlElement("w:cantSplit"); row._tr.get_or_add_trPr().append(cs)                                              # una fila no se parte entre páginas
    doc.add_paragraph()
    return t


def figura(doc, ruta, pie, ancho=16.0, alto_max=21.5):
    ruta = Path(ruta)
    if not ruta.exists():
        doc.add_paragraph(f"[FALTA la figura {ruta.name}]"); return
    im = cv2.imread(str(ruta)); h, w = im.shape[:2]
    ancho = min(ancho, alto_max * w / h)
    doc.add_picture(str(ruta), width=Cm(ancho))
    doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
    p = doc.add_paragraph(pie); p.alignment = WD_ALIGN_PARAGRAPH.CENTER; p.paragraph_format.keep_with_next = True
    p.runs[0].italic = True; p.runs[0].font.size = Pt(8.5)


def diagrama():
    """Arquitectura del dashboard y su entorno (figura propia)."""
    fig, ax = plt.subplots(figsize=(9.5, 5.6)); ax.set_xlim(0, 100); ax.set_ylim(0, 60); ax.axis("off")

    def caja(x, y, w, h, txt, c, tc="white", fs=8.5, bold=False):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4,rounding_size=1.2", fc=c, ec="#1a202c", lw=1))
        ax.text(x + w / 2, y + h / 2, txt, ha="center", va="center", color=tc, fontsize=fs, weight="bold" if bold else "normal")

    def flecha(a, b, txt="", off=(0, 1.4), both=True):
        ax.add_patch(FancyArrowPatch(a, b, arrowstyle="<|-|>" if both else "-|>", mutation_scale=11, lw=1.4, color="#2d3748"))
        ax.text((a[0] + b[0]) / 2 + off[0], (a[1] + b[1]) / 2 + off[1], txt, fontsize=7.2, ha="center", color="#2d3748")
    caja(2, 40, 26, 17, "Navegador del operador\n(PC, tablet o teléfono)\nindex.html — 4 259 líneas\nHTML + CSS + JavaScript\nsin dependencias externas", "#2f855a", bold=True)
    caja(2, 8, 26, 22, "Páginas auxiliares\netiquetar.html\nmanejo.html (móvil)\nmotores.html + motores.js\nwidget_redes.html", "#38a169")
    caja(40, 24, 26, 33, "Backend Jetson\njetson_server.py (aiohttp)\n5 346 líneas · 128 rutas\n\nexplorador · misiones\nconductor de pulsos\ncámara + IA · recolección\nmodo seguro · mapas", "#2b6cb0", bold=True)
    caja(40, 4, 26, 15, "Configuración persistente\nagrobot_config.json\n(config_store.py)", "#4a5568")
    caja(78, 46, 21, 11, "ROS 2 Jazzy\nSLAM · EKF · Nav2\nLIDAR (RPLIDAR C1)", "#6b46c1")
    caja(78, 31, 21, 11, "ESP32-C3\nmotores, servo, LED\n(UDP WiFi · USB · BLE)", "#c05621")
    caja(78, 16, 21, 11, "ESP32 LoRa V3\nCO₂, T, HR, IMU", "#b7791f")
    caja(78, 1, 21, 11, "Cámara USB\n640×480 · modelos YOLO\n(PLAGASX, macetas)", "#9b2c2c")
    flecha((28, 52), (40, 52), "HTTP · REST /api/*", (0, 1.6)); flecha((28, 47), (40, 47), "WebSocket /ws (push)", (0, -2.6)); flecha((28, 42), (40, 42), "MJPEG /video", (0, -2.6), both=False)
    flecha((28, 19), (40, 30), "REST /api/etiquetado…", (-1, -2))
    flecha((66, 52), (78, 52), "tópicos / TF", (0, 1.4)); flecha((66, 42), (78, 38), "UDP / BLE", (0, 1.4)); flecha((66, 32), (78, 22), "UDP :5001", (0, 1.4)); flecha((66, 27), (78, 7), "V4L2", (1.5, 0.5))
    flecha((53, 24), (53, 19), "", (0, 0))
    ax.text(50, 59.2, "Arquitectura del dashboard de AgroBot", ha="center", fontsize=11, weight="bold")
    fig.tight_layout(); fig.savefig(FIGS / "arquitectura.png", dpi=170); plt.close(fig)


def recortar_comparativa():
    hud = cv2.imread(str(ANOT / "01_dashboard_limpia.png"))
    sol = cv2.imread(str(RAIZ / "docs" / "PRUEBA_EXTERIOR_20261027" / "ui" / "00_dashboard_completo_sol.png"))
    if hud is None or sol is None:
        return
    a, b = hud[:1110], sol[:1110]
    cv2.imwrite(str(FIGS / "tema_hud.png"), a); cv2.imwrite(str(FIGS / "tema_sol.png"), b)


def main():
    diagrama(); recortar_comparativa()
    d = Document()
    for s in d.sections:
        s.left_margin = s.right_margin = Cm(2.0); s.top_margin = s.bottom_margin = Cm(1.8)
    st = d.styles["Normal"]; st.font.name = "Calibri"; st.font.size = Pt(10.5)
    d.add_heading("Informe funcional del dashboard de AgroBot", 0)
    d.add_paragraph("Análisis de todas las funcionalidades de la interfaz de operación · versión del 27 de octubre de 2026 · código: jetson/dashboard/index.html y jetson/backend/jetson_server.py")

    # ---------------- 1 Resumen
    d.add_heading("1. Resumen", 1)
    miles = lambda n: f"{n:,}".replace(",", " ")
    d.add_paragraph(
        "El dashboard es la única interfaz de operación del robot: una aplicación web de una sola página, servida por el propio backend de la Jetson y sin dependencias externas (funciona en la red aislada del punto de acceso, sin internet). "
        f"Tiene {len({t['id'] for t in TABS})} pestañas más 4 páginas auxiliares, {ST['botones']} botones, {ST['deslizadores']} deslizadores, {ST['interruptores']} interruptores y {ST['lienzos']} lienzos de dibujo en {miles(ST['lineas_index_html'])} líneas ({ST['tamano_kb']} KB). "
        f"Se comunica con el backend por WebSocket (tiempo real: {ST['mensajes_ws_manejados']} tipos de mensaje manejados) y por una API REST de {ST['rutas_rest_backend']} rutas, de las cuales {ST['rutas_usadas_por_alguna_pagina']} las usan las páginas del dashboard.")
    d.add_paragraph("Este informe describe, pestaña por pestaña, cada control (marcado con un número rojo en una captura real del dashboard con el robot conectado), qué hace y con qué ruta, mensaje y módulo del backend se conecta; "
                    "recoge la arquitectura, los canales de comunicación, los mecanismos de seguridad y robustez, y una evaluación crítica con los problemas encontrados y corregidos durante el análisis.")

    # ---------------- 2 Método
    d.add_heading("2. Alcance y método", 1)
    for t in ("Análisis estático de index.html (estructura, manejadores de eventos, llamadas fetch y mensajes WebSocket) y de las rutas registradas en jetson_server.py, recoleccion.py y etiquetado.py (tools/inventario_dashboard.py).",
              "Capturas reales de las 11 pestañas con el backend y el robot conectados, a 1500 px de ancho, mediante Chromium sin cabeza controlado por el protocolo CDP; la posición de cada control se mide en el navegador y se dibujan cuadros y números rojos (tools/capturas_anotadas_dashboard.py).",
              "Descripciones de cada control contrastadas con los textos de ayuda de la propia interfaz, los docstrings de los manejadores y el código; la especificación es única (tools/dashboard_spec.py).",
              "Límites: no se midió el rendimiento ni se probaron todos los botones con el robot en movimiento; los controles que mueven el robot (misiones, maniobras, calibraciones) se describen por el código, no por una prueba funcional de cada uno."):
        d.add_paragraph(t, style="List Bullet")

    # ---------------- 3 Arquitectura
    d.add_heading("3. Arquitectura", 1)
    figura(d, FIGS / "arquitectura.png", "Figura 1. Arquitectura del dashboard y de lo que controla.", 16.0)
    d.add_paragraph("El navegador descarga index.html (HTML, CSS y JavaScript en un solo archivo, sin librerías ni fuentes externas) y abre un WebSocket con el backend. Por ahí llegan, sin que el navegador pregunte, la pose, el mapa, el scan, las detecciones y la telemetría; "
                    "las órdenes del operador salen por peticiones REST (HTTP), a propósito: en el teléfono el WebSocket se cae al bloquear la pantalla y una petición HTTP o llega o devuelve un error. La excepción es el STOP, que va por las dos vías a la vez.")
    tabla(d, ["Indicador", "Valor"], [
        ["Líneas / tamaño de index.html", f"{ST['lineas_index_html']} / {ST['tamano_kb']} KB"], ["Pestañas + páginas auxiliares", f"{len({t['id'] for t in TABS})} + 4"],
        ["Botones / deslizadores / interruptores / lienzos", f"{ST['botones']} / {ST['deslizadores']} / {ST['interruptores']} / {ST['lienzos']}"], ["Elementos con identificador", ST["ids"]],
        ["Llamadas fetch / sondeos periódicos (setInterval)", f"{ST['llamadas_fetch']} / {ST['intervalos_setInterval']}"], ["Rutas REST registradas / usadas por alguna página", f"{ST['rutas_rest_backend']} / {ST['rutas_usadas_por_alguna_pagina']}"],
        ["Mensajes WebSocket manejados / emitidos por el backend", f"{ST['mensajes_ws_manejados']} / {ST['mensajes_ws_emitidos']}"], ["Consultas de ancho (@media) / atributos title / aria-label", f"{ST['media_queries']} / {ST['atributos_title']} / {ST['aria_label']}"],
        ["Diálogos de confirmación (confirm) / usos de localStorage", f"{ST['confirmaciones_confirm']} / {ST['usos_localstorage']}"],
    ], anchos=[10.5, 6.5])

    # ---------------- 4 Pestañas
    d.add_heading("4. Funcionalidades por pestaña", 1)
    d.add_paragraph("En cada captura, los números rojos remiten a la tabla que la acompaña. Las tablas indican la función, el canal con el backend (ruta REST o mensaje WebSocket) y el módulo del backend que responde.")
    for k, t in enumerate(TABS, 1):
        d.add_heading(f"4.{k} {t['titulo']}", 2)
        d.add_paragraph(t["intro"])
        figura(d, ANOT / f"{t['archivo']}.png", f"Figura {k + 1}. {t['titulo']}. Los números rojos se explican en la tabla siguiente.", 16.0, 20.0)
        tabla(d, ["N.º", "Control", "Qué hace", "Canal con el backend", "Módulo"], [[i, it[2], it[3], it[4], it[5]] for i, it in enumerate(t["items"], 1)], anchos=[0.9, 3.0, 7.2, 3.6, 2.3], size=8)

    # ---------------- 5 Páginas auxiliares
    d.add_heading("5. Páginas auxiliares", 1)
    aux = [("aux_etiquetar", "5.1 Etiquetar plagas (/etiquetar.html)", "Herramienta aparte para crear el dataset de plagas: se elige la fuente de fotos (mis fotos, capturas del robot, recolección de campo), se dibujan cajas arrastrando sobre la foto, se asigna la clase con el teclado, se guarda y pasa a la siguiente, y se exporta todo en un zip. Rutas: /api/etiquetado/lista, img, etiquetas, resumen y exportar (módulo etiquetado.py).", 14.0),
           ("aux_manejo_movil", "5.2 Manejo manual para el teléfono (/manejo.html)", "Versión mínima pensada para el móvil: cada toque mueve 0,3 s sin acumular órdenes, potencia limitada (máximo 60 % adelante y 35 % atrás), curvas, parar y comprobación de la conexión. Rutas: /api/drive, /api/emergencia y /api/estado/completo.", 7.0),
           ("aux_motores", "5.3 Motores (/motores.html + motores.js)", "Se incrusta en la pestaña «Motores y servo». Asigna a cada rueda su pin FI/BI del ESP32-C3, permite invertir el sentido, probar cada motor con pulsación mantenida e intercambiar dos ruedas; habla directamente con la C3. Dentro del dashboard se ve con tema claro (no sigue el tema HUD).", 14.0)]
    for n, tit, txt, w in aux:
        d.add_heading(tit, 2); d.add_paragraph(txt)
        figura(d, ANOT / f"{n}.png", tit + ".", w, 14.0)
    d.add_heading("5.4 Widget de redes (widget_redes.html)", 2)
    d.add_paragraph("Panel «Control de redes»: consulta el estado de la comunicación (GET /api/comunicacion/estado) y cambia el modo WiFi (POST /api/wifi/modo). Está pensado para integrarse en el dashboard; en esta versión no se enlaza desde index.html.")

    # ---------------- 6 Comunicación
    d.add_heading("6. Comunicación con el backend", 1)
    d.add_heading("6.1 WebSocket (tiempo real)", 2)
    maneja, emite = set(INV["ws_maneja"]), set(INV["ws_emite"])
    filas = []
    for m in sorted(maneja | emite | {"emergency_stop", "actuator"}):
        sentido = "navegador → backend" if m in ("emergency_stop", "actuator") else "backend → navegador"
        filas.append([m, sentido, "sí" if m in maneja else ("—" if m in ("emergency_stop", "actuator") else "no"), WS.get(m, "")])
    tabla(d, ["Mensaje", "Sentido", "Lo maneja el dashboard", "Contenido"], filas, anchos=[3.4, 3.0, 2.4, 8.2], size=8)
    sin = sorted(emite - maneja)
    d.add_paragraph(f"El backend emite {len(sin)} tipos de mensaje que el dashboard no trata de forma explícita: {', '.join(sin)}. Algunos son solo para trazas o para otros clientes, pero conviene revisarlos: los eventos de misión, patrulla y visión activa llegan al navegador y se descartan en silencio.")
    d.add_heading("6.2 API REST", 2)
    filas = [[g, n, GRUPOS.get(g, "")] for g, n in sorted(INV["rutas_por_grupo"].items(), key=lambda x: -x[1])]
    tabla(d, ["Grupo (/api/…)", "Rutas", "Para qué sirve"], filas, anchos=[3.6, 1.4, 12.0], size=8)
    d.add_paragraph(f"En total hay {ST['rutas_rest_backend']} rutas registradas ({sum(1 for r in INV['rutas'] if r['metodo'] == 'GET')} GET y {sum(1 for r in INV['rutas'] if r['metodo'] == 'POST')} POST); {ST['rutas_usadas_por_alguna_pagina']} las usa alguna página del dashboard y el resto sirve a scripts, herramientas y a la integración con otros módulos. La lista completa está en el Anexo A.")

    # ---------------- 7 Seguridad y robustez
    d.add_heading("7. Seguridad y robustez", 1)
    d.add_paragraph("Mecanismos verificados en el código:")
    for t in ("Parada de emergencia redundante: el botón STOP envía el mensaje `emergency_stop` por WebSocket y una petición POST a /api/emergencia con `keepalive`, y actúa en el primer toque (pointerdown), no al soltar. Detiene misión, exploración, patrulla, Nav2, pulsos y motores en un único sitio del backend (_parada_total).",
              "Reconexión automática: si el WebSocket se cierra, el indicador «enlace Jetson» se pone en rojo y se reintenta cada segundo.",
              f"Confirmaciones: {ST['confirmaciones_confirm']} diálogos de confirmación antes de acciones destructivas o que mueven el robot (vaciar la galería, resetear el mapa, iniciar misiones…).",
              "Acceso: HTTP Basic con clave y cookie de sesión (el WebSocket usa la cookie) cuando existe el archivo de clave del dashboard; las peticiones desde la propia Jetson (127.0.0.1) no la necesitan. Sin ese archivo el dashboard queda abierto, pensado para la red aislada del punto de acceso. En esta instalación el archivo existe.",
              "Perfil de demostración: con `control.solo_manual` activo, el backend rechaza (409) el inicio de misión, exploración, evasión, patrulla automática, metas de Nav2, paso robótico y calibración de navegación.",
              "Modo seguro: el indicador «Seguridad» muestra su nivel y permite rearmarlo (POST /api/seguridad/rearmar); lo activa el backend ante condiciones peligrosas como un vuelco.",
              "Tolerancia a fallos del navegador: localStorage siempre dentro de try/catch (modo privado o datos bloqueados) y la interfaz funciona sin él; las preferencias (tema, acelerador, alcance del minimapa) se recuerdan por navegador.",
              "Diseño pensado para el campo: tema claro para leer la tablet a pleno sol, tema HUD para poca luz, controles grandes y atajos de teclado."):
        d.add_paragraph(t, style="List Bullet")
    d.add_heading("Diseño visual: tema HUD y tema claro", 2)
    figura(d, FIGS / "tema_hud.png", "Figura 15. Tema HUD (verde sobre fondo oscuro), el predeterminado.", 15.0, 12.5)
    figura(d, FIGS / "tema_sol.png", "Figura 16. Tema claro («SOL») para leer a pleno sol; se alterna con el botón «TEMA» de la barra superior.", 15.0, 12.5)

    # ---------------- 8 Evaluación crítica
    d.add_heading("8. Evaluación crítica", 1)
    d.add_paragraph("Fortalezas", style="Heading 3")
    for t in ("Una sola pantalla da el estado completo (video, mapa, LIDAR, telemetría, detecciones) y el control: no hace falta abrir herramientas externas para operar.",
              "Funciona sin internet y sin instalar nada: abrir una URL basta, desde PC, tablet o móvil.",
              "Cada control está conectado a una ruta o mensaje concreto del backend, con módulos separados por responsabilidad; la configuración se persiste y se puede afinar sin tocar código (pestaña Parámetros).",
              "Seguridad pensada: STOP redundante, confirmaciones, perfil de demostración y modo seguro."):
        d.add_paragraph(t, style="List Bullet")
    d.add_paragraph("Limitaciones y mejoras posibles", style="Heading 3")
    for t in (f"Archivo monolítico: {ST['lineas_index_html']} líneas de HTML, CSS y JavaScript en un solo archivo, sin módulos ni pruebas automáticas del frontend; cada cambio se verifica a mano o con capturas.",
              f"{len(sin)} tipos de mensaje WebSocket emitidos no se manejan en el navegador ({', '.join(sin)}).",
              f"{ST['rutas_rest_backend'] - ST['rutas_usadas_por_alguna_pagina']} de las {ST['rutas_rest_backend']} rutas REST no las usa ninguna página: conviene documentarlas o retirar las muertas.",
              f"Accesibilidad limitada: {ST['aria_label']} etiquetas aria-label y {ST['atributos_title']} descripciones emergentes (title) para {ST['botones']} botones; muchos botones se entienden solo por su icono.",
              "Textos con mezcla de voseo y tuteo («tocá», «poné», «tú») y sin soporte de otros idiomas.",
              "La clave del dashboard es opcional: sin el archivo de clave cualquier equipo de la red del punto de acceso puede mover el robot.",
              "Las pestañas Simulación y Motores son páginas incrustadas con su propio estilo (Motores es claro y no sigue el tema HUD).",
              "Misiones, maniobra de giro 180°/360° y vigía del reloj se añadieron el 27-oct y siguen sin probarse en el robot real."):
        d.add_paragraph(t, style="List Bullet")
    d.add_paragraph("Problemas encontrados y corregidos durante este análisis", style="Heading 3")
    tabla(d, ["Problema", "Evidencia", "Corrección"], [
        ["Slider del LIDAR desaparecía en pantallas de menos de 900 px", "Revisión visual en Manejo asistido a 820 px", "Se mantiene visible junto al radar"],
        ["Título «AGROBOT» tapado por los indicadores a anchos medios", "Captura a 820 px", "Los indicadores pasan a una segunda fila y el título se acorta"],
        ["Lista de pestañas sin forma de desplazarse en ventanas bajas", "Petición del operador", "Scroll con deslizador verde siempre visible"],
        ["Banner «¡Plaga detectada!» saltaba con la clase «maceta»", "Captura del dashboard en vivo", "Las clases de planta se ignoran en el banner"],
        ["Cajas de plaga que ocupan media pantalla rodeaban el video", "Fotos de la prueba de exterior (4 de 4 cajas >60 % del cuadro) y una captura del ~40 %", "Las cajas de plaga de ≥30 % del cuadro se anotan como rótulo centrado abajo"],
    ], anchos=[5.2, 5.6, 6.2], size=8.5)

    # ---------------- 9 Conclusiones
    d.add_heading("9. Conclusiones", 1)
    d.add_paragraph("El dashboard cubre de punta a punta el ciclo de operación del robot: monitoreo en tiempo real, manejo manual y asistido, mapeo y navegación, autonomías y misiones, gestión de modelos de IA, calibración y parámetros, salud del sistema y recolección/etiquetado de datos para entrenar. "
                    "Su principal fortaleza es que todo está en una sola pantalla y se conecta de forma directa con módulos concretos del backend; sus principales debilidades son de mantenibilidad (archivo monolítico, sin pruebas) y de cobertura (mensajes sin tratar, rutas sin usar, accesibilidad). "
                    "Las mejoras más rentables serían separar el JavaScript en módulos con pruebas, tratar los eventos de misión y patrulla en la interfaz, y exigir clave por defecto.")

    # ---------------- Anexos
    d.add_heading("Anexo A. Rutas REST registradas", 1)
    filas = [[r["metodo"], r["ruta"], ", ".join(x.replace(".html", "") for x in r["usada_por"]) or "—", r["doc"][:110]] for r in sorted(INV["rutas"], key=lambda r: (r["ruta"], r["metodo"]))]
    tabla(d, ["Método", "Ruta", "Usada por", "Descripción (primera frase del docstring)"], filas, anchos=[1.4, 5.2, 2.2, 8.2], size=7)
    d.add_heading("Anexo B. Reproducibilidad", 1)
    for t in ("tools/inventario_dashboard.py — inventario de rutas, mensajes y estadísticas (docs/INFORME_DASHBOARD/inventario.json).",
              "tools/dashboard_spec.py — especificación de los controles marcados y su descripción.",
              "tools/capturas_anotadas_dashboard.py — capturas con números rojos (requiere el backend en marcha).",
              "tools/generar_informe_dashboard.py — este documento."):
        d.add_paragraph(t, style="List Bullet")
    destino = OUT / "INFORME_DASHBOARD_20261027.docx"
    d.save(destino); print("escrito", destino, destino.stat().st_size // 1024, "KB")


if __name__ == "__main__":
    main()
