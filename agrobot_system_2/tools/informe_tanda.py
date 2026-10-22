#!/usr/bin/env python3
"""Informe FINAL de la tanda de entrenamiento con tablas y diagramas (2026-10-22). HTML con las imágenes incrustadas + Markdown.

  python3 tools/informe_tanda.py [--desde ARCHIVO_MARCA] [--salida CARPETA]
Reúne: calibración con datos reales (robot parado + pulsos), búsqueda de manejo (continuo y pulsos: embudo y escenarios), aprendizaje de fallas,
informe de geometría/fallas, dataset de SLAM y el diagrama de flujo. Solo usa archivos más nuevos que la marca de inicio de la tanda.
"""
import argparse
import base64
import json
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

APR = Path.home() / ".agrobot" / "aprendizaje"
CAL = Path.home() / ".agrobot" / "calibracion"
INF = Path.home() / ".agrobot" / "informes"
SLAM = Path.home() / ".agrobot" / "slam"


def nuevos(patron, carpeta, marca):
    return [p for p in sorted(carpeta.glob(patron), key=lambda p: p.stat().st_mtime) if marca is None or p.stat().st_mtime >= marca]


def img(ruta):
    return f'<img src="data:image/png;base64,{base64.b64encode(Path(ruta).read_bytes()).decode()}" style="max-width:100%">'


def tabla(cab, filas):
    h = "<table><tr>" + "".join(f"<th>{c}</th>" for c in cab) + "</tr>"
    for f in filas:
        h += "<tr>" + "".join(f"<td>{c}</td>" for c in f) + "</tr>"
    return h + "</table>"


def seccion_manejo(marca, out: Path):
    h, md = "", ""
    for f in nuevos("robusto_*.json", APR, marca):
        d = json.loads(f.read_text())
        if "prueba" not in d and "validacion" not in d:
            continue
        ctrl = d.get("control", "?")
        fig, ax = plt.subplots(figsize=(8.5, 5))
        for etapa, col in (("filtro", "#bbbbbb"), ("validacion", "#3b6ea5"), ("prueba", "#e76f51")):
            L = list(d.get(etapa, {}).items())
            if L:
                ax.scatter([v["contactos_total"] for _, v in L], [v["exito_pct"] for _, v in L], c=col, s=34, alpha=.8, label=etapa)
        ax.set_xlabel("contactos totales"); ax.set_ylabel("éxito [%]"); ax.set_title(f"Búsqueda de manejo · control {ctrl}: éxito contra contactos"); ax.grid(alpha=.3); ax.legend()
        png = out / f"manejo_{ctrl}_embudo.png"
        fig.tight_layout(); fig.savefig(png, dpi=105); plt.close(fig)
        h += f"<h3>Control {ctrl}</h3>" + img(png)
        p = d.get("prueba", {})
        el = d.get("elegido", {})
        nombre = el if isinstance(el, str) else (el.get("nombre") if isinstance(el, dict) else None)
        if p:
            esc = sorted({e for v in p.values() for e in v.get("por_escenario", {})})
            fig, ax = plt.subplots(figsize=(10, 4.2))
            w = 0.8 / max(len(p), 1)
            for i, (n, v) in enumerate(p.items()):
                ax.bar(np.arange(len(esc)) + i * w, [v["por_escenario"].get(e, {}).get("exito_pct", 0) if isinstance(v["por_escenario"].get(e), dict) else 0 for e in esc], w, label=n)
            ax.set_xticks(np.arange(len(esc)) + 0.4); ax.set_xticklabels(esc, rotation=20); ax.set_ylabel("éxito [%]"); ax.set_title(f"Prueba final · {ctrl}: éxito por escenario"); ax.legend(fontsize=7); ax.grid(alpha=.3)
            png = out / f"manejo_{ctrl}_escenarios.png"
            fig.tight_layout(); fig.savefig(png, dpi=105); plt.close(fig)
            h += img(png)
            filas = [[n, v["exito_pct"], v["peor_hueco_exito_pct"], v["contactos_total"], v["puntos"]] for n, v in p.items()]
            h += tabla(["config", "éxito %", "peor pasillo %", "contactos", "puntos"], filas)
        h += f"<p>Criterios: <code>{json.dumps(d.get('criterios'), ensure_ascii=False)}</code> · óptimo aceptado: <b>{d.get('optimo_aceptado')}</b> · {d.get('segundos', 0) // 60} min</p>"
        md += f"- {ctrl}: aceptado={d.get('optimo_aceptado')} criterios={d.get('criterios')}\n"
    return h or "<p>(sin resultados de manejo en esta tanda)</p>", md


def seccion_fallas(marca, out: Path):
    fs = nuevos("fallas_*.json", APR, marca)
    if not fs:
        return "<p>(sin resultados de fallas en esta tanda)</p>", ""
    d = json.loads(fs[-1].read_text())
    hist = d["historial"]
    fig, axs = plt.subplots(1, 2, figsize=(14, 4.2))
    axs[0].plot([r["ronda"] for r in hist], [r["train_pct"] for r in hist], "o-", label="mismas pruebas")
    axs[0].plot([r["ronda"] for r in hist], [r["val_pct"] for r in hist], "s-", label="semillas nuevas")
    axs[0].set_xlabel("ronda"); axs[0].set_ylabel("pasa [%]"); axs[0].set_title("Aprendizaje asistido de fallas"); axs[0].legend(); axs[0].grid(alpha=.3)
    pf = hist[-1]["por_fallo_val"]
    nombres = sorted(pf, key=lambda k: pf[k]["pasa_pct"])
    axs[1].barh(nombres, [pf[k]["pasa_pct"] for k in nombres], color="#2a9d8f"); axs[1].set_xlabel("pasa [%] (validación)"); axs[1].set_title("Por falla"); axs[1].grid(alpha=.3)
    png = out / "fallas.png"
    fig.tight_layout(); fig.savefig(png, dpi=105); plt.close(fig)
    filas = [[k, v["n"], v["pasa_pct"], v["contactos"], v["recuperaciones"], v["con_leccion"]] for k, v in sorted(pf.items(), key=lambda kv: kv[1]["pasa_pct"])]
    return img(png) + tabla(["falla", "pruebas", "pasa %", "contactos", "recuperaciones", "con lección"], filas) + f"<p>Lecciones aprendidas: {len(d.get('lecciones', []))}</p>", f"- fallas: {hist[0]['val_pct']} % -> {hist[-1]['val_pct']} % (validación)\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--desde", default=str(APR / "tanda.inicio"))
    ap.add_argument("--salida", default=None)
    a = ap.parse_args()
    marca = Path(a.desde).stat().st_mtime if Path(a.desde).exists() else None
    out = Path(a.salida) if a.salida else INF / ("tanda_" + time.strftime("%Y%m%d_%H%M"))
    out.mkdir(parents=True, exist_ok=True)
    partes = [f"<h1>Informe de la tanda de entrenamiento · {time.strftime('%Y-%m-%d %H:%M')}</h1>",
              "<p>Física del simulador calibrada con datos reales; los límites conocidos están en cada sección. Nada se despliega al robot automáticamente.</p>"]
    md = f"# Informe de la tanda · {time.strftime('%Y-%m-%d %H:%M')}\n"
    # calibración
    h = "<h2>1. Calibración con datos reales</h2>"
    for n in ("cal_01_ruido_lidar_giroscopio.png", "cal_02_acelerometro.png", "cal_03_pulsos_real_vs_simulador.png"):
        if (CAL / n).exists():
            h += img(CAL / n)
    try:
        rp = json.loads((CAL / "real_parado.json").read_text())
        h += f"<p>LIDAR parado: σ(r) = {rp['lidar']['sigma_m']['a'] * 1000:.2f} mm + {rp['lidar']['sigma_m']['b'] * 1000:.2f} mm/m · r · rayos válidos {rp['lidar']['rayos_validos_pct']} %.</p>"
        pr = json.loads((CAL / "pulsos_real.json").read_text())
        h += tabla(["", "p25", "p50", "p75", "p90", "<2°"], [[f"{t} {k}"] + [round(v[c], 2) for c in ("p25", "p50", "p75", "p90", "menor_2deg")]
                                                              for t in ("real", "sim_antes", "sim_despues") for k, v in pr[t].items()])
        h += f"<p>Parámetros ajustados: <code>{json.dumps(pr['parametros'])}</code>. El simulador no llega a los giros de 19° del robot real (la física satura ~13°).</p>"
    except Exception as e:
        h += f"<p>(no se pudo leer la calibración: {e})</p>"
    partes.append(h)
    h, m1 = seccion_manejo(marca, out)
    partes.append("<h2>2. Búsqueda de manejo (simulador calibrado)</h2>" + h); md += m1
    h, m2 = seccion_fallas(marca, out)
    partes.append("<h2>3. Fallas con manejo asistido</h2>" + h); md += m2
    # informe de geometría
    inf = [p for p in sorted(INF.glob("2026*"), key=lambda p: p.stat().st_mtime) if marca is None or p.stat().st_mtime >= marca]
    if inf:
        d = inf[-1]
        h = f"<h2>4. Escenarios y fallas (pruebas_geometria)</h2><p>Informe completo: <code>{d}/informe.html</code></p>"
        for png in sorted(d.glob("captura_*_75.png"))[:6] + sorted(d.glob("falla_*.png"))[:4]:
            h += f"<div style='display:inline-block;width:48%'>{img(png)}</div>"
        partes.append(h)
    # slam
    lg = SLAM / "generar.log"
    if lg.exists():
        partes.append("<h2>5. Dataset de SLAM calibrado</h2><pre>" + "".join(l for l in lg.read_text().splitlines(True) if "[dataset]" in l)[-1500:] + "</pre>")
    flujo = INF / "flujo_tanda.png"
    if flujo.exists():
        partes.append("<h2>6. Flujo</h2>" + img(flujo))
    css = "<style>body{font-family:sans-serif;max-width:1000px;margin:2em auto}table{border-collapse:collapse;margin:.6em 0}td,th{border:1px solid #bbb;padding:3px 8px;font-size:13px}img{margin:6px 0}</style>"
    (out / "informe_tanda.html").write_text("<meta charset=utf-8><title>Informe de la tanda</title>" + css + "".join(partes))
    (out / "informe_tanda.md").write_text(md)
    print("informe en", out)


if __name__ == "__main__":
    main()
