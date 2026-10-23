"""Genera el REPORTE de simulación y entrenamiento (HTML con imágenes incrustadas + Markdown) en
agrobot_system_2/docs/REPORTE_SIMULACION_<fecha>/ a partir de los resultados guardados en ~/.agrobot (calibración, análisis de grabaciones,
validación del zoom, entrenamiento intensivo y comparativas). Uso: python3 generar_reporte.py [--entrenamiento <carpeta intensivo_...>]"""
import argparse, base64, glob, html, json, os, shutil, time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

AQUI = Path(__file__).resolve().parent
HOME = Path.home() / ".agrobot"
FECHA = "20261023"
DEST = AQUI.parent / "docs" / f"REPORTE_SIMULACION_{FECHA}"


def ultimo(patron):
    c = sorted(glob.glob(os.path.expanduser(patron)))
    return Path(c[-1]) if c else None


def cargar(path):
    try:
        return json.loads(Path(path).read_text())
    except Exception:
        return None


def img_b64(path):
    return "data:image/png;base64," + base64.b64encode(Path(path).read_bytes()).decode()


class Doc:
    def __init__(self):
        self.md, self.h = [], []

    def titulo(self, n, t):
        self.md.append(f"\n{'#' * n} {t}\n")
        self.h.append(f"<h{n}>{html.escape(t)}</h{n}>")

    def p(self, t):
        self.md.append(t + "\n")
        self.h.append(f"<p>{t}</p>")

    def tabla(self, cab, filas, pie=None):
        self.md.append("| " + " | ".join(cab) + " |\n|" + "---|" * len(cab) + "\n" + "\n".join("| " + " | ".join(str(c) for c in f) + " |" for f in filas) + "\n")
        t = "<table><thead><tr>" + "".join(f"<th>{html.escape(str(c))}</th>" for c in cab) + "</tr></thead><tbody>"
        t += "".join("<tr>" + "".join(f"<td>{html.escape(str(c))}</td>" for c in f) + "</tr>" for f in filas) + "</tbody></table>"
        self.h.append(t)
        if pie:
            self.p(pie)

    def figura(self, ruta, pie=""):
        nombre = Path(ruta).name
        shutil.copy(ruta, DEST / "figs" / nombre)
        self.md.append(f"\n![{pie}](figs/{nombre})\n*{pie}*\n")
        self.h.append(f'<figure><img src="{img_b64(ruta)}"><figcaption>{html.escape(pie)}</figcaption></figure>')


CSS = """body{font:15px/1.5 system-ui,sans-serif;max-width:1050px;margin:24px auto;padding:0 16px;color:#1c2b22}
h1,h2,h3{color:#14532d}table{border-collapse:collapse;margin:10px 0;width:100%}th,td{border:1px solid #cfe3d6;padding:5px 8px;text-align:left;font-size:13.5px}
th{background:#e8f5ec}figure{margin:14px 0}img{max-width:100%;border:1px solid #cfe3d6;border-radius:6px}figcaption{font-size:13px;color:#4b6b57}
.ok{color:#15803d;font-weight:600}.mal{color:#b91c1c;font-weight:600}code{background:#eef5f0;padding:1px 4px;border-radius:4px}"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--entrenamiento", default=None)
    a = ap.parse_args()
    (DEST / "figs").mkdir(parents=True, exist_ok=True)
    D = Doc()
    D.titulo(1, "AgroBot · Reporte de simulación, entrenamiento y mejoras — 2026-10-23")
    D.p(f"Generado {time.strftime('%Y-%m-%d %H:%M')} · datos reales de las pruebas de hoy (grabaciones de campo + SLAM) y simulación calibrada con ellos.")

    # ---------------- 1. grabaciones
    ana = ultimo("~/.agrobot/informes/analisis_*")
    D.titulo(2, "1. Qué pasó en las pruebas reales de hoy")
    if ana and (ana / "analisis.json").exists():
        d = cargar(ana / "analisis.json")
        D.tabla(["sesión", "duración (s)", "recorrido (m)", "toques probables", "toques/min", "% del tiempo a <5 cm", "% a <12 cm"],
                [[r["sesion"][-13:], r["duracion_s"], r.get("recorrido_m"), r["contactos_probables"], r["toques_por_min"], r["pct_menos_5cm"], r["pct_menos_12cm"]] for r in d],
                "«Toque probable» = el cuerpo llega a ≤ 2 cm de algo según el LIDAR (episodios separados > 1,5 s). Desde la primera prueba (01:28) a la última (05:41) "
                "la tasa baja de ~3,4-4,7 a ~1,0 toques/min con el freno activo, el 100 % continuo y la anticolisión trasera.")
        for r in d:
            m = r.get("mision", {})
            if m.get("inicios"):
                pass
        filas = []
        for r in d:
            m = r.get("mision", {})
            filas.append([r["sesion"][-13:], m.get("inicios", 0), m.get("detectadas", 0), m.get("inspeccionadas", 0), m.get("fallidas", 0), json.dumps(m.get("motivos_fallo", {}), ensure_ascii=False)])
        D.tabla(["sesión", "misiones", "plantas detectadas", "inspeccionadas", "inspecciones fallidas", "motivos"], filas)
        for png in sorted(ana.glob("*_trayectoria.png"))[-2:]:
            D.figura(png, f"Trayectoria y hueco libre — {png.name[:20]}")
        if (ana / "pulsos.png").exists():
            D.figura(ana / "pulsos.png", "Desplazamiento y giro por tipo de pulso (todas las sesiones). Izquierda gira menos y retrocede más que derecha (antes de compensar).")
    # ---------------- 2. calibración
    cal = cargar(AQUI / "calibracion_pulsos_20261023.json")
    D.titulo(2, "2. Simulador calibrado con los pulsos reales")
    if cal:
        obj, sim = cal["objetivo"], cal["simulado"]
        D.tabla(["pulso", "real: giro° / avance cm", "simulador calibrado: giro° / avance cm"],
                [[k, f"{obj[k][0]:+.1f} / {obj[k][1]:+.1f}", f"{sim[k][0]:+.1f} / {sim[k][1]:+.1f}"] for k in obj],
                "Sin calibrar el simulador recorría 18 cm hacia delante por pulso (real: 9) y casi no giraba en los arcos. Parámetros ajustados: "
                + ", ".join(f"{k}={v:.2f}" for k, v in cal["parametros"].items()) + ".")
        fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
        ks = list(obj)
        x = np.arange(len(ks))
        ax[0].bar(x - 0.2, [obj[k][1] for k in ks], 0.4, label="real"); ax[0].bar(x + 0.2, [sim[k][1] for k in ks], 0.4, label="simulado")
        ax[0].set_xticks(x); ax[0].set_xticklabels(ks); ax[0].set_title("Avance por pulso (cm)"); ax[0].legend()
        ax[1].bar(x - 0.2, [obj[k][0] for k in ks], 0.4, label="real"); ax[1].bar(x + 0.2, [sim[k][0] for k in ks], 0.4, label="simulado")
        ax[1].set_xticks(x); ax[1].set_xticklabels(ks); ax[1].set_title("Giro por pulso (°)"); ax[1].legend()
        fig.tight_layout(); fig.savefig(DEST / "figs" / "calibracion.png", dpi=110); plt.close(fig)
        D.figura(DEST / "figs" / "calibracion.png", "Real vs simulador calibrado")
    # ---------------- 3. zoom
    z = ultimo("~/.agrobot/informes/zoom_plagas_*")
    D.titulo(2, "3. Zoom automático y detección de plagas al acercarse")
    D.p("La cámara es de foco fijo (sin control de enfoque): «enfocar» = llevar la planta al centro (la visión activa gira el robot y mueve el tilt), "
        "acercarla con zoom digital (recorte del sensor a 1280×720, hasta 3×) y quedarse con el cuadro más nítido. Validación: hojas reales de col "
        "(sana, mildiu, mancha de alternaria) pegadas a escala sobre fondos reales del robot; el clasificador del robot (PLAGASX) con y sin zoom.")
    if z and (z / "resultados.json").exists():
        r = cargar(z / "resultados.json")
        modos = []
        for f in r:
            if f["modo"] not in modos:
                modos.append(f["modo"])
        escalas = sorted({f["escala"] for f in r})
        filas = []
        for s in escalas:
            fila = [s]
            for m in modos:
                f = [q for q in r if q["escala"] == s and q["modo"] == m][0]
                fila.append(f"{100 * f['exactitud']:.0f} % (z={f['zoom_medio']})" if m != "sin_zoom" else f"{100 * f['exactitud']:.0f} %")
            filas.append(fila)
        D.tabla(["tamaño de la planta (fracción del ancho)"] + modos, filas, "Exactitud del clasificador (3 clases, 90 imágenes por celda). El zoom ayuda sobre todo cuando la planta ocupa menos del 35 % de la imagen.")
        if (z / "exactitud_vs_distancia.png").exists():
            D.figura(z / "exactitud_vs_distancia.png", "Exactitud vs distancia con y sin zoom automático")
        if (z / "galeria.jpg").exists():
            shutil.copy(z / "galeria.jpg", DEST / "figs" / "galeria_zoom.jpg")
            D.md.append("\n![galería](figs/galeria_zoom.jpg)\n")
            D.h.append('<figure><img src="data:image/jpeg;base64,' + base64.b64encode((z / "galeria.jpg").read_bytes()).decode() + '"><figcaption>Lo que ve el clasificador, de lejos a cerca: sin zoom y con zoom automático (varios objetivos).</figcaption></figure>')
    # ---------------- 4. entrenamiento
    ent = Path(a.entrenamiento) if a.entrenamiento else ultimo("~/.agrobot/aprendizaje/intensivo_*")
    D.titulo(2, "4. Entrenamiento intensivo (simulación calibrada, interior 70 % · exterior 30 %)")
    inf = cargar(ent / "informe.json") if ent else None
    hist = (inf or cargar(ent / "historial.json") or {}).get("historial") if ent else None
    if hist:
        fig, ax = plt.subplots(figsize=(8.5, 3.8))
        g = [h["gen"] for h in hist]
        ax.plot(g, [h["mejor"] for h in hist], label="mejor de la generación"); ax.plot(g, [h["elite"] for h in hist], label="élite")
        ax.plot(g, [h["media"] for h in hist], label="media", alpha=0.6)
        hoy = [(h["gen"], h["hoy"]) for h in hist if h.get("hoy") is not None]
        if hoy:
            ax.scatter([x for x, _ in hoy], [y for _, y in hoy], color="k", label="config de hoy (referencia)", zorder=5)
        ax.set_xlabel("generación"); ax.set_ylabel("puntuación (menos = mejor)"); ax.set_title("Curva de aprendizaje"); ax.legend(); ax.grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(DEST / "figs" / "aprendizaje.png", dpi=110); plt.close(fig)
        D.figura(DEST / "figs" / "aprendizaje.png", f"{len(hist)} generaciones de {20} candidatos, cada una con semillas nuevas")
    if inf and inf.get("validacion"):
        filas = []
        for f in inf["validacion"]:
            t, i, e = f["todo"], f["interior"], f["exterior"]
            filas.append([f["nombre"], t["exito_pct"], t["contactos_por_ep"], t["puntos"], i["exito_pct"], i["contactos_por_ep"], e["exito_pct"], e["contactos_por_ep"]])
        D.tabla(["config", "éxito % (todo)", "contactos/ep", "puntos", "éxito % interior", "contactos int.", "éxito % exterior", "contactos ext."], filas,
                "Validación en semillas nunca vistas (8 de interior × 5 escenarios, 5 de exterior × 3). HOY = lo flasheado esta mañana; AYER = sin freno activo ni compensación.")
        m = inf["mejor"]
        from entrenar_intensivo import HOY
        D.tabla(["parámetro", "hoy", f"mejor ({m['nombre']})"], [[k, HOY[k], m["params"][k]] for k in HOY if k in m["params"]])
        mj = [f for f in inf["validacion"] if f["nombre"] == m["nombre"]][0]
        filas = [[k, v["exito_pct"], v["contactos_por_ep"], v["puntos"], [x for x in inf["validacion"] if x["nombre"] == "HOY"][0]["por_escenario"][k]["exito_pct"],
                  [x for x in inf["validacion"] if x["nombre"] == "HOY"][0]["por_escenario"][k]["contactos_por_ep"]] for k, v in mj["por_escenario"].items()]
        D.tabla(["escenario", "mejor: éxito %", "contactos/ep", "puntos", "hoy: éxito %", "hoy: contactos/ep"], filas)
        if inf.get("estres"):
            D.tabla(["config", "éxito % (estrés)", "contactos/ep", "puntos"], [[n, v["todo"]["exito_pct"], v["todo"]["contactos_por_ep"], v["todo"]["puntos"]] for n, v in inf["estres"].items()],
                    "Estrés: robot más flojo (más masa, más rozamiento, menos agarre).")
    else:
        D.p("<i>El entrenamiento aún no terminó: este reporte se regenera al acabar.</i>")
    extra = DEST / "figs"
    for png in sorted((HOME / "informes" / "episodios").glob("*.png")) if (HOME / "informes" / "episodios").exists() else []:
        D.figura(png, f"Episodio simulado: {png.stem}")
    D.titulo(2, "5. Qué se cambió hoy (resumen)")
    D.p("Firmware fusión con USB, 100 % continuo, freno activo y compensación del giro · mando asistido que PAUSA (el dashboard no sale de autónomo) · servo hacia abajo con macetas bajas · "
        "detección trasera con cono de 90° · bucle de escaneos de la misión corregido · zoom automático y cuadro más nítido al analizar · simulador calibrado con pulsos reales · "
        "mapa desfasado respaldado y reseteado. Detalle en MANIFEST_1.md.")
    D.titulo(2, "6. Límites honestos")
    D.p("El simulador reproduce el avance y el retroceso de los pulsos reales pero subestima el giro de los arcos (~⅔ del real) y NO reproduce que el giro a la izquierda retroceda más; "
        "por eso <code>comp_neg</code>/<code>comp_pos</code> se calibran con el robot (<code>tools/medir_giro.py</code>). La validación del zoom usa fotos de hoja de col de bases públicas y no hojas de tu cultivo. "
        "Los resultados de entrenamiento son sobre simulación: nada se desplegó sin tu OK.")
    (DEST / "REPORTE.md").write_text("\n".join(D.md))
    (DEST / "index.html").write_text(f"<!doctype html><meta charset=utf-8><title>AgroBot reporte {FECHA}</title><style>{CSS}</style>" + "\n".join(D.h))
    print("->", DEST)


if __name__ == "__main__":
    main()
