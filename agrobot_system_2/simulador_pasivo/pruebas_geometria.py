"""Batería de pruebas de geometría y de fallas con INFORME (tablas + capturas), 2026-10-21.

  1) Escenarios pedidos por el operador: ir RECTO A TODA VELOCIDAD, SALA CUADRADA, HILERAS de macetas (cuadrícula) y
     ZIGZAG a través de ellas, más los cuatro de siempre (sala, pasillo, eslalon, curva).
  2) TODAS las fallas del catálogo (episodio_fallas.FALLOS) en cada escenario donde aplican.
  3) Informe: ~/.agrobot/informes/<fecha>/informe.md + informe.html (con las imágenes incrustadas) + PNG de las
     trayectorias (la «captura» de cada prueba) y tablas CSV.

NO despliega nada. Uso:
  python3 pruebas_geometria.py [--config optimo|actual|interior] [--procs 6] [--semillas 4] [--rapido] [--sin-fallas]
"""
import argparse
import base64
import csv
import json
import math
import time
from dataclasses import asdict, replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import sim_logica_actual as S
import episodio_fallas as E

SALIDA = Path.home() / ".agrobot" / "aprendizaje"
INFORMES = Path.home() / ".agrobot" / "informes"
R = S.R_MACETA


def escenarios_extra(hueco=0.60) -> dict:
    """Los escenarios pedidos. Todos con el robot empezando en x=0 mirando a +x."""
    esc = {}
    # ir recto a toda velocidad: pasillo largo y estrecho (1,4 m), meta a 6 m
    esc["recto_tope"] = S.Escenario("recto_tope", S._sala(7.6, 1.4), [], goal_x=6.0, t_max=70.0,
                                    descripcion="Recto a toda velocidad: 6 m en un pasillo de 1,4 m, sin obstáculos.")
    # sala cuadrada 3x3 con una maceta en el centro: de una esquina a la esquina opuesta
    paredes = [(-0.6, -1.5, 2.4, -1.5), (-0.6, 1.5, 2.4, 1.5), (-0.6, -1.5, -0.6, 1.5), (2.4, -1.5, 2.4, 1.5)]
    esc["sala_cuadrada"] = S.Escenario("sala_cuadrada", paredes, [(1.0, 0.0, R)], goal_x=2.0, inicio=(0.0, -1.0, 0.0),
                                       t_max=140.0, meta_xy=(1.9, 1.0),
                                       descripcion="Sala cuadrada 3x3 m con una maceta al centro: de una esquina a la opuesta.")
    # hileras: tres filas de macetas (cuadrícula) y el robot por el pasillo del medio-arriba
    filas = []
    x = 0.7
    while x <= 3.0:
        for y in (-0.9, 0.0, 0.9):
            filas.append((x, y, R))
        x += 0.45
    esc["hileras_cuadricula"] = S.Escenario("hileras_cuadricula", S._sala(4.7, 3.4), filas, goal_x=3.0,
                                            inicio=(0.0, 0.45, 0.0), centro_y=0.45, t_max=150.0,
                                            descripcion="Tres hileras de macetas (cuadrícula 45 cm): por el pasillo entre dos hileras.")
    # zigzag a través de las hileras: cada hilera cierra el paso salvo un hueco que alterna de lado
    mac, lado, x = [], 1, 0.9
    for _ in range(3):
        for y in np.arange(-1.2, 1.21, 0.27):
            if abs(y - lado * 0.6) > hueco / 2 + R:
                mac.append((x, float(y), R))
        lado *= -1
        x += 0.9
    esc["zigzag_hileras"] = S.Escenario("zigzag_hileras", S._sala(5.0, 3.0), mac, goal_x=3.6, t_max=220.0,
                                        descripcion=f"Zigzag: tres hileras con un hueco de {hueco*100:.0f} cm que alterna de lado.")
    return esc


def todos_los_escenarios(hueco):
    d = dict(S.escenarios_interior(hueco))
    d.update(escenarios_extra(hueco))
    return d


def config_base(nombre):
    if nombre == "actual":
        return S.config_actual()
    if nombre == "optimo":
        fs = sorted(SALIDA.glob("modelo_manejo_optimo_*.json"))
        if fs:
            return S.Config(**json.loads(fs[-1].read_text())["config"])
    return S.config_interior_perfil()


def a_toda_velocidad(cfg):
    """Mismo controlador pero con el tope de velocidad del robot (0,55 m/s) y escala 1."""
    if cfg.control == "continuo":
        return replace(cfg, cc_v=S.V_SCALE, cc_nominal=S.V_SCALE, nombre=cfg.nombre + "+tope")
    return replace(cfg, escala_velocidad=1.0, nombre=cfg.nombre + "+tope")


def _job(a):
    cfg_d, esc_n, hueco, seed, fallo, traza = a
    cfg = S.Config(**cfg_d)
    esc = todos_los_escenarios(hueco)[esc_n]
    if esc_n == "recto_tope":
        cfg = a_toda_velocidad(cfg)
    # siempre episodio_fallas: tiene respuesta física a los choques (el robot no atraviesa paredes) y paradas para escanear
    ap = E.AP.Aprendizaje(Path("/nonexistent/m.json"), autoguardar=False)
    r = E.episodio_fallas(esc, cfg, seed, fallo, ap, False, traza=traza)
    r["fallo"] = fallo
    r["escenario"] = esc_n
    r["hueco"] = hueco
    r["semilla"] = seed
    r.setdefault("pasa", bool(r.get("exito")))
    r.setdefault("exito", bool(r["pasa"]))
    return r


def dibujar(esc, traza, titulo, ruta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7.5, 5.2), dpi=110)
    for (x0, y0, x1, y1) in esc.paredes:
        ax.plot([x0, x1], [y0, y1], color="#444", lw=2)
    for (x, y, r) in esc.macetas:
        ax.add_patch(plt.Circle((x, y), r, color="#6a994e", alpha=.8))
    if esc.meta_xy:
        ax.plot(*esc.meta_xy, marker="*", color="gold", ms=16, mec="k")
    else:
        ax.axvline(esc.goal_x, color="gold", ls="--")
    if traza:
        t = np.array([[p[1], p[2]] for p in traza])
        ch = np.array([bool(p[5]) for p in traza])
        ax.plot(t[:, 0], t[:, 1], color="#1d6fb8", lw=1.6, label="trayectoria del centro")
        if ch.any():
            ax.plot(t[ch, 0], t[ch, 1], "rx", ms=7, label="contacto")
        ax.plot(*t[0], "go", ms=8, label="inicio")
        ax.legend(loc="upper right", fontsize=7)
    ax.set_aspect("equal")
    ax.set_title(titulo, fontsize=9)
    ax.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(ruta)
    plt.close(fig)


def tabla_md(cab, filas):
    s = "| " + " | ".join(cab) + " |\n|" + "---|" * len(cab) + "\n"
    return s + "".join("| " + " | ".join(str(c) for c in f) + " |\n" for f in filas)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", choices=("optimo", "actual", "interior"), default="optimo")
    ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--semillas", type=int, default=4)
    ap.add_argument("--rapido", action="store_true")
    ap.add_argument("--sin-fallas", action="store_true")
    a = ap.parse_args()
    cfg = config_base(a.config)
    semillas = tuple(range(80, 80 + (1 if a.rapido else a.semillas)))
    huecos = (0.60,) if a.rapido else (0.50, 0.75)
    ts = time.strftime("%Y%m%d_%H%M")
    out = INFORMES / ts
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    nombres = list(todos_los_escenarios(0.6).keys())
    if a.rapido:
        nombres = ["recto_tope", "sala_cuadrada"]
    print(f"[informe] config {cfg.nombre} · {len(nombres)} escenarios · semillas {semillas} · huecos {huecos}", flush=True)
    cfg_d = asdict(cfg)
    md = [f"# Informe de pruebas de manejo · {time.strftime('%Y-%m-%d %H:%M')}\n",
          f"Configuración evaluada: **{cfg.nombre}** (control {cfg.control}, firmware {cfg.firmware}, geometría {S.GEOMETRIA}). "
          "Física del simulador = hipótesis (no calibrada): giro ajustado a ~10°/toque medido por el operador; avance por toque de giro "
          "del simulador menor que el real (1-3 cm vs ~10 cm).\n"]
    with Pool(a.procs) as pool:
        # ---- 1. escenarios sin fallas
        jobs = [(cfg_d, e, h, s, "ninguno", s == semillas[0]) for e in nombres for h in huecos for s in semillas]
        res = pool.map(_job, jobs, chunksize=1)
        filas, csv_f = [], []
        for e in nombres:
            for h in huecos:
                rs = [r for r in res if r["escenario"] == e and r["hueco"] == h]
                ex = 100 * sum(1 for r in rs if r["exito"]) / len(rs)
                lleg = 100 * sum(1 for r in rs if r.get("llego")) / len(rs)
                filas.append([e, f"{h*100:.0f}", f"{ex:.0f} %", f"{lleg:.0f} %", sum(r["contactos"] for r in rs),
                              f"{np.mean([r.get('progreso', 0) for r in rs])*100:.0f} %", f"{np.mean([r.get('t_s', 0) for r in rs]):.0f}",
                              f"{np.mean([r.get('pico_cm', 0) for r in rs]):.0f}", sum(1 for r in rs if r.get("atascado"))])
        cab = ["escenario", "hueco cm", "éxito (sin contactos)", "llega a la meta", "contactos", "progreso medio", "t medio s",
               "error lateral pico cm (solo rectas)", "atascos"]
        md += ["\n## 1. Escenarios sin fallas\n", tabla_md(cab, filas)]
        (out / "escenarios.csv").write_text("\n".join(",".join(map(str, f)) for f in [cab] + filas))
        md.append("\n### Capturas (trayectoria de la primera semilla)\n")
        for e in nombres:
            for h in huecos:
                r = next((r for r in res if r["escenario"] == e and r["hueco"] == h and r["semilla"] == semillas[0]), None)
                if r and r.get("traza"):
                    esc = todos_los_escenarios(h)[e]
                    png = out / f"captura_{e}_{int(h*100)}.png"
                    dibujar(esc, r["traza"], f"{e} · hueco {h*100:.0f} cm · éxito={r['exito']} contactos={r['contactos']} "
                                             f"progreso={r.get('progreso', 0)*100:.0f} %", png)
                    md.append(f"![{e}](./{png.name})\n")
        print(f"[informe] escenarios listos ({int(time.time()-t0)} s)", flush=True)
        # ---- 2. fallas
        if not a.sin_fallas:
            pares = E.combinaciones(("ninguno", "maceta_baja", "esquina_inicio") if a.rapido else None)
            jobs = [(cfg_d, e, 0.60, s, f, s == semillas[0]) for (f, e) in pares for s in semillas[:3]]
            fr = pool.map(_job, jobs, chunksize=1)
            por = {}
            for r in fr:
                por.setdefault(r["fallo"], []).append(r)
            filas = []
            for f, rs in por.items():
                filas.append([f, E.FALLOS[f].descripcion[:70], len(rs), f"{100*sum(1 for r in rs if r['pasa'])/len(rs):.0f} %",
                              sum(r["contactos"] for r in rs), sum(r.get("recuperaciones", 0) for r in rs),
                              sum(1 for r in rs if r.get("atascado"))])
            cab = ["falla", "descripción", "pruebas", "pasa", "contactos", "recuperaciones", "atascos"]
            md += ["\n## 2. Cada falla del sistema\n", tabla_md(cab, sorted(filas, key=lambda x: int(x[3].split()[0])))]
            (out / "fallas.csv").write_text("\n".join(",".join(map(str, f)) for f in [cab] + filas))
            md.append("\n### Capturas de fallas (primera semilla del peor escenario de cada falla)\n")
            for f, rs in por.items():
                malos = [r for r in rs if not r["pasa"] and r.get("traza")]
                if malos:
                    r = malos[0]
                    esc = todos_los_escenarios(0.60)[r["escenario"]]
                    png = out / f"falla_{f}_{r['escenario']}.png"
                    dibujar(esc, r["traza"], f"falla {f} · {r['escenario']} · contactos={r['contactos']} progreso={r.get('progreso', 0)*100:.0f} %", png)
                    md.append(f"![{f}](./{png.name})\n")
            print(f"[informe] fallas listas ({int(time.time()-t0)} s)", flush=True)
    md.append(f"\n_Generado en {int(time.time()-t0)} s._\n")
    texto = "".join(md)
    (out / "informe.md").write_text(texto)
    # HTML con imágenes incrustadas
    html_body = texto
    for png in out.glob("*.png"):
        b64 = base64.b64encode(png.read_bytes()).decode()
        html_body = html_body.replace(f"![", "![", 1)
        html_body = html_body.replace(f"(./{png.name})", f"(data:image/png;base64,{b64})")
    try:
        import markdown
        cuerpo = markdown.markdown(html_body, extensions=["tables"])
    except Exception:
        cuerpo = "<pre>" + html_body.replace("<", "&lt;") + "</pre>"
    (out / "informe.html").write_text("<meta charset=utf-8><title>Informe de manejo</title><style>body{font-family:sans-serif;max-width:980px;"
                                      "margin:2em auto}table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:3px 8px}"
                                      "img{max-width:100%}</style>" + cuerpo)
    print(f"Informe en {out}", flush=True)


if __name__ == "__main__":
    main()
