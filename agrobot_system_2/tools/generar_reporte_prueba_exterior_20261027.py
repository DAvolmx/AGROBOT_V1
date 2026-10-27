"""Reporte (Word) de la prueba de EXTERIOR del 2026-10-27, para la tesis. Lee docs/PRUEBA_EXTERIOR_20261027/{metricas.json,figs,ui} que deja
tools/analizar_prueba_exterior_20261027.py y tools/capturas_dashboard.py. Uso: python3 tools/generar_reporte_prueba_exterior_20261027.py"""
import glob, json, os
from pathlib import Path

import cv2
from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.shared import Cm, Pt

RAIZ = Path(__file__).resolve().parents[1]
OUT = RAIZ / "docs" / "PRUEBA_EXTERIOR_20261027"
FIG, UI = OUT / "figs", OUT / "ui"
HOME = Path.home() / ".agrobot"
M = json.loads((OUT / "metricas.json").read_text())
SES = json.loads((HOME / "datos" / "20261027_215547" / "sesion.json").read_text())
A = SES["config"]["autonomia"]


def tabla(doc, cab, filas, size=8.5):
    t = doc.add_table(rows=1, cols=len(cab)); t.style = "Light Grid Accent 1"
    for i, c in enumerate(cab):
        t.rows[0].cells[i].text = str(c)
    for f in filas:
        r = t.add_row().cells
        for i, v in enumerate(f):
            r[i].text = str(v)
    for row in t.rows:
        for c in row.cells:
            for p in c.paragraphs:
                for run in p.runs:
                    run.font.size = Pt(size)
    doc.add_paragraph()
    return t


def figura(doc, ruta, pie, ancho=16.0):
    if Path(ruta).exists():
        doc.add_picture(str(ruta), width=Cm(ancho))
        doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
        p = doc.add_paragraph(pie); p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.runs[0].italic = True; p.runs[0].font.size = Pt(8.5)
    else:
        doc.add_paragraph(f"[FALTA la figura {Path(ruta).name}]")


def parrafo(doc, txt, negrita_ini=None):
    p = doc.add_paragraph()
    if negrita_ini:
        p.add_run(negrita_ini + " ").bold = True
    p.add_run(txt)
    return p


def main():
    d = Document()
    st = d.styles["Normal"]; st.font.name = "Calibri"; st.font.size = Pt(10.5)
    d.add_heading("Prueba de exterior de la misión de plantas en maceta", 0)
    d.add_paragraph("AgroBot — robot móvil de inspección de cultivos · Sesión del 27 de octubre de 2026 · Datos: sesión 20261027_215547")
    r, g, ek, rec = M["mision"], M.get("giros", {}), M.get("ekf", {}), M["recorrido"]

    # ---------------- 1. Resumen
    d.add_heading("1. Resumen", 1)
    d.add_paragraph(
        f"El 27 de octubre de 2026 se ejecutó la misión de plantas en maceta (explorar, mapear y escanear plantas) con el robot real en un patio abierto con macetas alineadas junto a un muro. "
        f"La misión duró {r['duracion_s']} s de reloj monótono, con intervención manual del operador en el {M['manual_pct_de_activo']:.0f} % del tiempo activo. "
        f"El robot detectó {r['detecciones_de_planta']} veces una maceta y completó el escaneo de {r['plantas_inspeccionadas']} plantas (más {r['inspecciones_fallidas']} inspecciones interrumpidas: "
        f"{r['motivos_fallo'].get('orden manual', 0)} por mando manual y {r['motivos_fallo'].get('no logré centrar el objetivo', 0)} por no lograr centrar el objetivo). "
        f"Se registraron {M['scans']['exterior']['n']} scans del LIDAR, 1 198 imágenes de cámara, 947 fotos y {len(M.get('videos', []))} videos de campo, además de 45 fotos de la visión activa y 5 videos de inspección de plantas.")
    d.add_paragraph("Resultados principales:")
    for t in (
        f"Las {r['plantas_inspeccionadas']} plantas escaneadas son suculentas y ornamentales (el reconocedor de especie CLIP las clasifica como «otra», p_col entre 0,02 y 0,32), no coles. "
        f"El detector de plagas, entrenado para col, emitió {sum(M['avisos_plaga_toast'].values())} avisos de plaga (amarillamiento, malformación, daño de insecto) sobre esas plantas: están fuera de su dominio y deben tratarse como falsos positivos.",
        f"Un salto del reloj de pared de +{M['salto_reloj']['magnitud_s']:.1f} s (el equipo no tiene RTC) a los {M['salto_reloj']['en_t_s']:.0f} s provocó saltos de la pose del SLAM de 30,1 m y 24,4 m y un pico del EKF "
        f"(vx = {ek.get('vx_max_abs_ms')} m/s, pose desplazada {ek.get('salto_de_pose_tras_el_salto_de_reloj_m')} m). Las plantas escaneadas después de ese instante quedaron ubicadas en un marco poco fiable.",
        f"Los pulsos de giro siguen siendo débiles: mediana de {g['giroscopio']['mediana_deg']}° (giroscopio) o {g['rumbo_ekf']['mediana_deg']}° (rumbo del EKF) por pulso, frente al objetivo de {g['objetivo_deg']:.0f}°; "
        f"entre {g['giroscopio']['menos_de_2deg_pct']} % y {g['rumbo_ekf']['menos_de_2deg_pct']} % de los pulsos giran menos de 2°.",
        f"El clasificador automático de entorno propuso «interior» en el {100 * M['entorno_auto']['sugerido'].get('interior', 0) / max(1, M['entorno_auto']['muestras']):.0f} % de sus {M['entorno_auto']['muestras']} muestras, aun estando el robot al aire libre; "
        "el operador tuvo que fijar el modo «exterior» a mano.",
        "No se registraron choques: el nivel de seguridad se mantuvo en «normal» en las 850 muestras de la sesión y la anticolisión actuó 3 veces (el robot no tiene sensor de contacto, así que un roce leve no queda descartado).",
    ):
        d.add_paragraph(t, style="List Bullet")

    # ---------------- 2. Objetivo
    d.add_heading("2. Objetivo y alcance", 1)
    d.add_paragraph("Verificar en exterior, con el robot real, la cadena completa de la misión de plantas (exploración reactiva con LIDAR y cámara, SLAM, detección de macetas, visión activa para acercarse y escanear, "
                    "detección de plagas) y recoger datos etiquetados como «exterior» para reentrenar el clasificador de entorno y para el análisis de la tesis. "
                    "Es una única prueba de unos 12 minutos sin referencia de posición externa (sin «verdad de terreno»), por lo que los resultados son descriptivos y no estadísticamente concluyentes.")

    # ---------------- 3. Sistema y configuración
    d.add_heading("3. Sistema y configuración durante la prueba", 1)
    d.add_paragraph("Plataforma: robot de cuatro ruedas con eje pasivo y tracción principal delantera, Jetson Orin Nano (ROS 2 Jazzy, slam_toolbox, robot_localization), LIDAR de plano único a 37,5 cm del suelo, "
                    "cámara USB de 640×480 a 24 cm de altura sobre un servo de inclinación, IMU y microcontrolador ESP32-C3 como actuador. "
                    f"Código: commit {SES.get('codigo')} más cambios locales sin consolidar (cámara como LIDAR, giro compensado, giro de lazo cerrado, misiones específicas, tema HUD del dashboard).")
    tabla(d, ["Parámetro", "Valor en la prueba", "Significado"], [
        ["autonomia.camara_como_lidar", A.get("camara_como_lidar"), "el frente de la cámara filtrado se fusiona con el scan del explorador"],
        ["autonomia.mx_giro_comp", A.get("mx_giro_comp"), "un pulso atrás cada N pulsos de giro seguidos (compensa el arco del eje pasivo)"],
        ["autonomia.giro_lazo_cerrado", A.get("giro_lazo_cerrado"), "cada pulso de giro intenta 6° medidos con el giroscopio"],
        ["autonomia.escala_velocidad", A.get("escala_velocidad"), "factor global de velocidad (0,13 = muy lento)"],
        ["autonomia.e11_activo / predictor", f"{A.get('e11_activo')} / {A.get('e11_predictor')}", "regreso A* y predictor de colisión"],
        ["autonomia.avance_libre_cm", A.get("avance_libre_cm"), "hueco mínimo para avanzar"],
        ["odometria_webcam (geometría de la cámara)", "altura 0,24 m · inclinación 1,9° · campo horizontal 55°", "calibrada el 27-oct con cajas de medida conocida"],
        ["Modelo de detección activo", SES["config"]["ai"]["active_model"], "detector de plagas de col (PLAGASX) + detector de macetas"],
    ])

    # ---------------- 4. Datos
    d.add_heading("4. Datos recopilados y método", 1)
    tabla(d, ["Conjunto", "Contenido", "Tamaño"], [
        ["datos/20261027_215547", "eventos.jsonl (8 324 eventos), telemetria.csv (≈16 400 filas), 116 archivos de scans LIDAR (6 447 scans de 720 rayos), 1 198 imágenes de cámara", "92 MB"],
        ["campo/20261027_215828_25f568", "747 fotos, variables a 5 Hz (3 322 muestras), video 640×480 a 10 fps de 434 s", "362 MB"],
        ["campo/20261027_221926_28a5d4", "200 fotos, variables a 5 Hz, video de 91 s (tramo cubierto/interior)", "43 MB"],
        ["vision_activa/fotos", "45 fotos de las vistas de inspección de plantas", "—"],
        ["videos/agrobot_planta_*.mp4", "5 videos de las inspecciones de plantas (archivos 221556 a 222114, posteriores al salto de reloj)", "—"],
        ["entorno/20261027.jsonl", "muestras del clasificador de entorno (rasgos de LIDAR y cámara, puntuación, modo, etiqueta del operador)", "—"],
    ])
    d.add_paragraph("Método. Todos los tiempos se expresan en segundos de sesión con el reloj monótono (t); el reloj de pared no es fiable porque saltó +711,5 s en t = 527 s. "
                    "La ventana de análisis va de t = 150 s (primer movimiento) a t = 880 s (fin de la misión). El estado del explorador se muestrea cada ≈2 s; la telemetría, a ≈10 Hz. "
                    "El ángulo por pulso de giro se estima de dos maneras independientes: integral del giroscopio y cambio del rumbo del EKF. "
                    "El script de análisis es tools/analizar_prueba_exterior_20261027.py y reproduce todas las cifras y figuras de este reporte.")
    d.add_paragraph("Etiquetado. A petición del operador la prueba quedó etiquetada como «exterior»: se escribió ETIQUETA.json en cada sesión y se marcaron como «exterior» las muestras del clasificador de entorno "
                    "del tramo al aire libre (t = 150–708 s) que no tenían etiqueta; los datos crudos no se modificaron y se guardó una copia previa del archivo de muestras. "
                    "A partir de t ≈ 708 s el robot entró en una zona cubierta (el brillo medio de la cámara cae de 0,57 a 0,3 y el video 20261027_221926 es de interior), por lo que esa sesión se etiquetó como «exterior_cubierto_o_interior».")

    # ---------------- 5. Resultados
    d.add_heading("5. Resultados", 1)
    d.add_heading("5.1 Desarrollo de la misión", 2)
    est = M["estados_pct"]
    tabla(d, ["Estado del explorador", "% del tiempo de la ventana"], [[k, v] for k, v in est.items()])
    d.add_paragraph(f"La misión inició en t = 212 s y terminó por orden del operador en t = 875 s ({r['duracion_s']} s). El robot estuvo «detenido» (esperando o en inspección) el {est.get('detenido', 0)} % del tiempo de la ventana "
                    f"y bajo mando manual el {est.get('mando manual', 0)} %; hubo {M['intervenciones_manuales']['eventos_asistido']} eventos de manejo asistido y {M['intervenciones_manuales']['toasts_autonomia_reanudada']} reanudaciones de la autonomía. "
                    "Cada intervención manual interrumpe la visión activa: seis de las siete inspecciones fallidas fueron «orden manual».")
    figura(d, FIG / "f2_linea_tiempo.png", "Figura 1. Línea de tiempo: estado del explorador, mando aplicado y huecos libres del LIDAR por dirección.", 16.5)

    d.add_heading("5.2 Mapa, trayectoria y LIDAR", 2)
    d.add_paragraph(f"Tiempo en movimiento: {rec['tiempo_en_movimiento_s']} s; camino recorrido ≈ {rec['camino_integral_v_m']} m (integral de la velocidad aplicada) y {rec['camino_slam_m']} m según la pose del SLAM sin contar los saltos. "
                    f"La pose del SLAM saltó {', '.join(f'{x[1]} m (t = {x[0]} s)' for x in rec['saltos_de_pose_slam'] if x[1] > 5)}; esos cuadros se omiten del dibujo. "
                    f"Hueco libre mínimo del LIDAR en la ventana: adelante {M['lidar_min_m']['adelante']} m, izquierda {M['lidar_min_m']['izquierda']} m, derecha {M['lidar_min_m']['derecha']} m (percentil 5: "
                    f"{M['lidar_p5_m']['adelante']}, {M['lidar_p5_m']['izquierda']} y {M['lidar_p5_m']['derecha']} m). El tiempo con menos de 20 cm libres fue del {M['lidar_pct_menos_20cm']['adelante']} % al frente, "
                    f"{M['lidar_pct_menos_20cm']['izquierda']} % a la izquierda y {M['lidar_pct_menos_20cm']['derecha']} % a la derecha. La lectura «atrás» (≈3 cm casi constante) corresponde al propio chasis y no se interpreta como obstáculo.")
    figura(d, FIG / "f1_trayectoria.png", "Figura 2. Recorrido según la pose del SLAM (color = tiempo), puntos LIDAR acumulados y plantas escaneadas (rombos verdes).", 12.5)
    figura(d, FIG / "f8_scans.png", "Figura 3. Scans crudos del LIDAR y fracción de rayos sin retorno por scan, exterior frente a interior del mismo día.", 15.5)
    d.add_paragraph(f"En exterior {M['scans']['exterior']['sin_retorno_pct']} % de los rayos no tuvo retorno (mediana de alcance {M['scans']['exterior']['mediana_alcance_m']} m), frente a "
                    f"{M['scans']['interior_mismo_dia']['sin_retorno_pct']} % en interior ({M['scans']['interior_mismo_dia']['mediana_alcance_m']} m): el espacio abierto da más rayos perdidos, pero la diferencia es moderada porque el patio es estrecho y está cerrado por muros.")

    d.add_heading("5.3 Escaneo de plantas", 2)
    tabla(d, ["Planta", "x (m)", "y (m)", "Especie (CLIP)", "p_col", "Vistas"], [[p["n"], p["x"], p["y"], p["especie"], p["p_col"], p["vistas"]] for p in r["plantas"]])
    figura(d, FIG / "f5_plantas.png", "Figura 4. Vista frontal de cada planta escaneada por la visión activa. P5 se tomó ya en la zona cubierta.", 15.5)
    d.add_paragraph("Las posiciones de P3–P5 se calcularon después del salto de reloj y de pose del SLAM; solo P1 y P2 (t < 527 s) se consideran bien ubicadas. "
                    "Las cinco plantas son suculentas y ornamentales, no coles, de modo que la prueba valida el flujo de detección-acercamiento-escaneo pero no el diagnóstico de col.")

    d.add_heading("5.4 Detección de plagas y falsos positivos", 2)
    dt = M["detecciones"]
    tabla(d, ["Clase detectada", "Detecciones", "Confianza media"], [[c, n, dt["confianza_media"].get(c)] for c, n in dt["por_clase"].items()])
    d.add_paragraph(f"El modelo de plagas generó {dt['total'] - dt['por_clase'].get('maceta', 0)} detecciones de plaga con confianza media entre 0,42 y 0,67, que activaron {M['capturas_plaga']} capturas y avisos "
                    f"({', '.join(f'{k}: {v}' for k, v in M['avisos_plaga_toast'].items())}). Sobre suculentas no hay coles que diagnosticar: son falsos positivos fuera de dominio. "
                    "Además se observó que el detector devuelve a veces la imagen completa como caja (4 de 4 cajas a más del 60 % del cuadro en las fotos analizadas) y que el panel mostraba el banner «¡Plaga detectada!» para la clase «maceta».")
    figura(d, FIG / "f4_detecciones.png", "Figura 5. Confianza de las detecciones en el tiempo y por clase.", 15.5)
    figura(d, FIG / "f5b_capturas_plaga.png", "Figura 6. Capturas guardadas por los avisos de plaga: plantas ornamentales sin relación con el cultivo de col.", 15.5)

    d.add_heading("5.5 Eficacia de los giros", 2)
    d.add_paragraph(f"Se analizaron {g['pulsos']} pulsos de giro (mediana de duración {g['duracion_mediana_s']} s). Con el giroscopio, la mediana es {g['giroscopio']['mediana_deg']}° por pulso "
                    f"(cuartiles {g['giroscopio']['p25_deg']}°–{g['giroscopio']['p75_deg']}°; {g['giroscopio']['menos_de_2deg_pct']} % por debajo de 2°); con el rumbo del EKF, {g['rumbo_ekf']['mediana_deg']}° "
                    f"(cuartiles {g['rumbo_ekf']['p25_deg']}°–{g['rumbo_ekf']['p75_deg']}°; {g['rumbo_ekf']['menos_de_2deg_pct']} % por debajo de 2°). Solo un {g['giroscopio']['sentido_contrario_pct']}–{g['rumbo_ekf']['sentido_contrario_pct']} % giró en sentido contrario, "
                    "y las señales del mando, el giroscopio y el SLAM coinciden en signo, así que no hay una inversión de giro. La causa más probable es la falta de par real: con 1,5 rad/s ordenados el giroscopio mide ≈5 °/s de media. "
                    "El giro de lazo cerrado activado en esta prueba no mejoró de forma apreciable lo ya conocido (≈3°/pulso con pulsos de tiempo fijo); la estimación depende del método (la telemetría a 10 Hz subestima los pulsos cortos).")
    figura(d, FIG / "f3_giros.png", "Figura 7. Ángulo girado por pulso (dos estimaciones) y su relación con la duración del pulso.", 15.5)

    d.add_heading("5.6 Clasificador automático de entorno", 2)
    e = M["entorno_auto"]
    d.add_paragraph(f"De {e['muestras']} muestras en la ventana, el clasificador sugirió «interior» en todas (puntuación media {e['puntuacion_media']}, donde +1 es interior seguro) a pesar de estar al aire libre. "
                    f"Los rasgos medios fueron brillo {e['rasgos_medios']['brillo']}, cielo {e['rasgos_medios']['cielo']}, verde {e['rasgos_medios']['verde']} y rugosidad {e['rasgos_medios']['rugosidad']}: el patio es un recinto estrecho entre muros, con poco cielo visible y casi sin vegetación verde, "
                    "y el LIDAR ve paredes cercanas, justo lo que el modelo asocia con interior. Por eso estas muestras, ya etiquetadas como exterior, son valiosas para reentrenar el clasificador "
                    "(tools/entorno_entrenar.py); hasta entonces el modo exterior debe fijarse a mano.")
    figura(d, FIG / "f7_entorno.png", "Figura 8. Rasgos de la cámara y puntuación automática durante la prueba (la caída de brillo en el minuto 9,3 marca la entrada a la zona cubierta).", 15.0)

    d.add_heading("5.7 Video", 2)
    d.add_paragraph("Los videos son 640×480 a 10 fps (434 s y 91 s). La figura muestra fotogramas equiespaciados: los de la primera sesión son del patio exterior; los de la segunda, de la zona cubierta.")
    figura(d, FIG / "f6_video.png", "Figura 9. Fotogramas de los videos de campo (filas 1–2: patio exterior; fila 3: zona cubierta/interior).", 16.0)

    # ---------------- 6. Incidentes
    d.add_heading("6. Incidentes y hallazgos de ingeniería", 1)
    tabla(d, ["Hallazgo", "Evidencia", "Estado"], [
        ["Salto de reloj de pared (+711,5 s en t = 527 s) corrompe la pose", f"Saltos de pose del SLAM de 30,1 y 24,4 m; EKF con vx = {ek.get('vx_max_abs_ms')} m/s y pose desplazada {ek.get('salto_de_pose_tras_el_salto_de_reloj_m')} m; telemetría con un «hueco» de 712 s en t_unix",
         "Pendiente: sincronizar la hora antes de iniciar o proteger el EKF/SLAM ante saltos del reloj (el equipo no tiene RTC)"],
        ["Banner «¡Plaga detectada!» para la clase «maceta»", "Captura del dashboard en vivo", "Corregido en el dashboard (se ignoran las clases de planta)"],
        ["Cajas de plaga de imagen completa cubren el video", "4 de 4 cajas del detector activo a >60 % del cuadro", "Corregido: se dibuja un rótulo en vez del marco (umbral 55 %)"],
        ["Falsos positivos de plaga sobre plantas que no son col", f"{M['capturas_plaga']} capturas/avisos sobre suculentas", "Pendiente: filtrar por especie (CLIP) antes de avisar"],
        ["Giros de 1,5–2,4° por pulso (objetivo 6°)", "Giroscopio y rumbo del EKF", "Pendiente: revisar el par de los motores y la fuerza del giro"],
        ["Clasificador de entorno falla en exterior estrecho", "100 % de las muestras sugeridas como interior", "Datos etiquetados; falta reentrenar"],
        ["Mando manual en el 40 % del tiempo activo", "37 eventos de manejo asistido; 6 inspecciones interrumpidas", "Observación de la operación"],
    ], size=8)

    # ---------------- 7. Interfaz
    d.add_heading("7. Interfaz del operador", 1)
    d.add_paragraph("Durante la sesión se rediseñó el dashboard con un tema de estilo HUD (verde sobre fondo oscuro): la cámara ocupa el ancho completo, el minimapa circular centrado en el robot (rumbo arriba) se superpone a la cámara en la esquina inferior izquierda "
                    "y un radar LIDAR reducido con control de alcance, en la superior derecha. El tema claro original sigue disponible con un botón para leer a pleno sol. Las capturas de todas las pestañas están en docs/PRUEBA_EXTERIOR_20261027/ui.")
    figura(d, UI / "00_dashboard_completo_hud.png", "Figura 10. Dashboard completo (tema HUD) con el robot conectado.", 14.5)
    for n, pie in (("04_navegacion_hud.png", "Figura 11. Pestaña de navegación: grabación de recorrido y mapa SLAM."), ("02_manejo_asistido_hud.png", "Figura 12. Manejo asistido: cámara, mapa y mando en una sola vista."),
                   ("03_galeria_hud.png", "Figura 13. Galería de capturas de la misión."), ("05_slam_telemetria_hud.png", "Figura 14. SLAM y telemetría.")):
        figura(d, UI / n, pie, 14.0)

    # ---------------- 8. Limitaciones y 9. Conclusiones
    d.add_heading("8. Limitaciones", 1)
    for t in ("Una sola prueba de ≈12 minutos, sin verdad de terreno de posición ni número real de macetas: no se pueden calcular exactitud ni recall.",
              "La pose posterior a t = 527 s no es fiable (salto de reloj), y con ella las posiciones de P3–P5.",
              "Las plantas no eran coles: el diagnóstico de plagas no se pudo validar.",
              "La distancia recorrida depende del método (integral de la velocidad aplicada frente a la pose del SLAM).",
              "Los ángulos por pulso proceden de telemetría a ≈10 Hz y de dos estimadores con sesgos distintos."):
        d.add_paragraph(t, style="List Bullet")
    d.add_heading("9. Conclusiones y trabajo siguiente", 1)
    d.add_paragraph("La misión completa funcionó de extremo a extremo en exterior: el robot detectó macetas, se acercó, las escaneó y guardó fotos, video y mapa sin choques registrados (aunque sin sensor de contacto que lo confirme). "
                    "Los puntos críticos son operativos y de robustez, no de concepto: (1) proteger el sistema ante saltos del reloj; (2) aumentar el par de giro para que la exploración no dependa tanto del mando manual; "
                    "(3) asegurar que los avisos de plaga solo se emitan sobre coles; (4) reentrenar el clasificador de entorno con estas muestras de exterior. Repetir la prueba con coles reales y una referencia de posición permitiría cuantificar la exactitud del diagnóstico y del mapa.")
    d.add_heading("Anexo A. Reproducibilidad", 1)
    for t in ("tools/analizar_prueba_exterior_20261027.py — métricas y figuras (docs/PRUEBA_EXTERIOR_20261027/metricas.json y figs/).",
              "tools/etiquetar_prueba_exterior_20261027.py — etiquetado como exterior (idempotente, con copia de seguridad).",
              "tools/capturas_dashboard.py — capturas de todas las pestañas del dashboard vía Chromium headless.",
              "tools/generar_reporte_prueba_exterior_20261027.py — este documento."):
        d.add_paragraph(t, style="List Bullet")
    destino = OUT / "REPORTE_PRUEBA_EXTERIOR_20261027.docx"
    d.save(destino)
    print("escrito", destino, os.path.getsize(destino) // 1024, "KB")


if __name__ == "__main__":
    main()
