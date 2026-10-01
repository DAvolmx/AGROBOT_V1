"""Genera el informe HTML de la sesión del 2026-10-01 a partir de los datos
reales (log de entrenamiento, vigía, validaciones, mediciones, fotos).

    python3 tools/informe_sesion.py logs/sesion_20261001/informe.html
"""
import json
import re
import statistics as st
import sys
from html import escape
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent
SES = RAIZ / "logs" / "sesion_20261001"
APR = Path.home() / ".agrobot" / "aprendizaje"


def leer_datos():
    L = (RAIZ / "logs" / "entrenamiento_20261001.log").read_text(errors="ignore").splitlines()
    tandas = [(int(m.group(1)), int(m.group(2)), float(m.group(3)))
              for l in L for m in [re.search(r"tanda (\d+): (\d+) ejemplos acumulados, acierto ([\d.]+)%", l)] if m]
    evals = []
    tanda_actual = 0
    for l in L:
        m = re.search(r"--- tanda (\d+)", l)
        if m:
            tanda_actual = int(m.group(1))
        m = re.search(r"evaluación: desvío ([\d.]+) -> ([\d.]+) cm \| pisadas ([\d.]+) -> ([\d.]+)", l)
        if m:
            evals.append((tanda_actual, *map(float, m.groups())))
    episodios = sum(1 for l in L if re.search(r"\[\d+/10\]", l))
    descartados = sum(1 for l in L if "descartado" in l)
    inicio = next((l[:19] for l in L if "=== entrenamiento" in l), "")
    fin = next((l[:19] for l in L if "=== fin" in l), "")

    regs = [json.loads(l) for l in (SES / "estado.jsonl").read_text().splitlines() if l.strip()]
    sis = [r["sistema"] for r in regs]
    def serie(k):
        return [s[k] for s in sis if s.get(k) is not None]
    salud = {}
    for r in regs:
        for f in (r["salud"] if isinstance(r["salud"], list) else []):
            salud.setdefault(f["key"], {}).setdefault(f["status"], 0)
            salud[f["key"]][f["status"]] += 1
    niveles = {}
    for r in regs:
        s = r["seguridad"]
        n = s.get("nivel") if isinstance(s, dict) else "sin dato"
        niveles[n] = niveles.get(n, 0) + 1

    vals = []
    for l in (APR / "historial.jsonl").read_text().splitlines():
        try:
            e = json.loads(l)
        except ValueError:
            continue
        if e.get("tipo") == "validacion" and e.get("cuando", "").startswith("2026-10-01"):
            vals.append(e)
    # el último resultado de cada modelo
    por_modelo = {}
    for v in vals:
        por_modelo[v["modelo"]] = v
    fotos = json.loads((SES / "fotos_informe.json").read_text())
    return dict(tandas=tandas, evals=evals, episodios=episodios, descartados=descartados,
                inicio=inicio, fin=fin, regs=regs, tj=serie("tj_c"), ram=serie("ram_usada_mb"),
                pot=serie("potencia_mw"), gpu=serie("gpu_pct"), salud=salud, niveles=niveles,
                validaciones=list(por_modelo.values()), fotos=fotos)


def svg_acierto(tandas):
    """Línea de acierto de validación por tanda, a escala."""
    W, H, ml, mr, mt, mb = 640, 220, 46, 14, 14, 34
    xs = [t[0] for t in tandas]
    ys = [t[2] for t in tandas]
    y0, y1 = 80.0, 84.0
    def X(x): return ml + (x - min(xs)) / max(1, (max(xs) - min(xs))) * (W - ml - mr)
    def Y(y): return mt + (y1 - y) / (y1 - y0) * (H - mt - mb)
    puntos = " ".join(f"{X(x):.1f},{Y(y):.1f}" for x, y in zip(xs, ys))
    area = f"{X(xs[0]):.1f},{Y(y0):.1f} " + puntos + f" {X(xs[-1]):.1f},{Y(y0):.1f}"
    g = []
    for v in (80, 81, 82, 83, 84):
        g.append(f'<line x1="{ml}" x2="{W - mr}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" class="grid"/>'
                 f'<text x="{ml - 8}" y="{Y(v) + 4:.1f}" class="tick" text-anchor="end">{v} %</text>')
    for x in (1, 5, 10, 15, 20, 25, 29):
        if x in xs:
            g.append(f'<text x="{X(x):.1f}" y="{H - 12}" class="tick" text-anchor="middle">{x}</text>')
    ult = tandas[-1]
    return (f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="Acierto de validación por tanda">'
            + "".join(g)
            + f'<polygon points="{area}" class="area"/>'
            + f'<polyline points="{puntos}" class="line"/>'
            + f'<circle cx="{X(ult[0]):.1f}" cy="{Y(ult[2]):.1f}" r="4" class="dot"/>'
            + f'<text x="{X(ult[0]) - 6:.1f}" y="{Y(ult[2]) - 9:.1f}" class="lbl" text-anchor="end">{ult[2]:.1f} %</text>'
            + f'<text x="{(W + ml) / 2:.0f}" y="{H - 1}" class="tick" text-anchor="middle">tanda</text>'
            + "</svg>")


def tabla_validacion(vals):
    if not vals:
        return "<p>La validación no terminó a tiempo para este informe.</p>"
    esc = ["huerta_real", "bucle", "hileras", "pasillo", "callejon"]
    vals = sorted(vals, key=lambda v: v["puntos"])
    filas = []
    for i, v in enumerate(vals):
        actual = "tanda048" in v["modelo"]
        nombre = v["modelo"].replace("politica_", "").replace(".pt", "")
        etiqueta = "modelo anterior" if actual else ("ganador" if i == 0 else "")
        celdas = "".join(f'<td class="num">{v["escenarios"].get(e, float("nan")):.1f}</td>' for e in esc)
        filas.append(f'<tr class="{"gana" if i == 0 else ""}{" previo" if actual else ""}">'
                     f'<th scope="row">{escape(nombre)}'
                     f'{f"<span class=chip>{etiqueta}</span>" if etiqueta else ""}</th>'
                     f'{celdas}<td class="num total">{v["puntos"]:.2f}</td></tr>')
    cab = "".join(f'<th scope="col" class="num">{e.replace("_", " ")}</th>' for e in esc)
    return (f'<div class="tabla"><table><thead><tr><th scope="col">Modelo</th>{cab}'
            f'<th scope="col" class="num">Media</th></tr></thead><tbody>{"".join(filas)}</tbody></table></div>')


def generar(destino: Path, desplegado: str | None):
    d = leer_datos()
    t0, t1 = d["tandas"][0], d["tandas"][-1]
    salud_txt = "".join(
        f'<li><span class="k">{k}</span> <span class="v">{c.get("ok", 0)} OK'
        f'{" · " + str(c.get("warn")) + " aviso" if c.get("warn") else ""}'
        f'{" · " + str(c.get("err")) + " apagada" if c.get("err") else ""}</span></li>'
        for k, c in d["salud"].items())
    fotos = "".join(
        f'<figure><img src="data:image/jpeg;base64,{f["b64"]}" alt="{"Vista del sendero" if f.get("sendero") else "Cámara del robot"} a las {f["hora"]} UTC" loading="lazy">'
        f'<figcaption>{"Análisis de sendero" if f.get("sendero") else "Cámara"} · {f["hora"]} UTC</figcaption></figure>'
        for f in d["fotos"])
    evals = "".join(
        f'<tr><th scope="row">{t}</th><td class="num">{a:.1f} → {b:.1f}</td><td class="num">{c:.1f} → {e:.1f}</td></tr>'
        for t, a, b, c, e in d["evals"])
    ganador = desplegado or "ninguno (el anterior sigue)"
    html = PLANTILLA.format(
        inicio=d["inicio"], fin=d["fin"], tandas=len(d["tandas"]), episodios=d["episodios"],
        descartados=d["descartados"], ej0=f"{t0[1]:,}".replace(",", "."), ej1=f"{t1[1]:,}".replace(",", "."),
        acierto=f"{t1[2]:.1f}", amin=f"{min(t[2] for t in d['tandas']):.1f}", amax=f"{max(t[2] for t in d['tandas']):.1f}",
        grafico=svg_acierto(d["tandas"]), evals=evals, validacion=tabla_validacion(d["validaciones"]),
        ganador=escape(ganador), muestras=len(d["regs"]),
        v0=d["regs"][0]["hora"][11:16], v1=d["regs"][-1]["hora"][11:16],
        tjmin=f"{min(d['tj']):.0f}", tjmax=f"{max(d['tj']):.0f}", tjmed=f"{st.mean(d['tj']):.0f}",
        rammax=max(d["ram"]), rammed=f"{st.mean(d['ram']):.0f}",
        potmed=f"{st.mean(d['pot']) / 1000:.1f}", potmax=f"{max(d['pot']) / 1000:.1f}",
        gpumed=f"{st.mean(d['gpu']):.0f}", normal=d["niveles"].get("normal", 0), salud=salud_txt, fotos=fotos)
    destino.write_text(html)
    return destino


PLANTILLA = open(Path(__file__).with_name("informe_sesion.plantilla.html")).read()

if __name__ == "__main__":
    salida = Path(sys.argv[1] if len(sys.argv) > 1 else SES / "informe.html")
    desplegado = sys.argv[2] if len(sys.argv) > 2 else None
    print(generar(salida, desplegado))
