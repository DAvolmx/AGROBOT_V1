#!/usr/bin/env python3
"""Aplica al robot el MEJOR candidato de un entrenamiento intensivo (informe.json de ~/.agrobot/aprendizaje/intensivo_*/).

Sin --aplicar solo MUESTRA qué cambiaría (antes -> después) y cómo volver atrás. Con --aplicar:
  · límites del firmware de la C3 (por la API del backend, con la C3 enlazada): adel, pivote, tras, dif_del, acel, frena, freno_ms, comp_neg, comp_pos
  · parámetros de la Jetson: velocidad, troceo de pulsos, espacio libre y cono PARA EL ENTORNO ACTUAL (interior o exterior) vía /api/params/*
El backend y la C3 tienen que estar encendidos. Antes de aplicar guarda el estado actual en ~/.agrobot/aprendizaje/antes_de_aplicar_<fecha>.json
(con eso `--revertir <archivo>` lo deja como estaba).
Uso: python3 tools/aplicar_resultado_entrenamiento.py [--informe <informe.json>] [--entorno interior|exterior] [--aplicar] | --revertir <archivo>"""
import argparse, glob, json, os, sys, time, urllib.request
from pathlib import Path

BASE = "http://localhost:8080"
LIM = {"fu_adel": "adel", "fu_pivote": "pivote", "fu_tras": "tras", "fu_dif_del": "dif_del", "fu_acel": "acel", "fu_frena": "frena",
       "fu_freno_ms": "freno_ms", "fu_comp_neg": "comp_neg", "fu_comp_pos": "comp_pos"}


def api(ruta, datos=None, metodo=None):
    req = urllib.request.Request(BASE + ruta, data=None if datos is None else json.dumps(datos).encode(),
                                 headers={"Content-Type": "application/json"}, method=metodo or ("POST" if datos is not None else "GET"))
    return json.load(urllib.request.urlopen(req, timeout=10))


def jetson(p, entorno):
    i = entorno == "interior"
    return {"autonomia": {"escala_velocidad": round(p["escala_int" if i else "escala_ext"], 3), "avance_libre_cm": round(p["avance_libre_int" if i else "avance_libre_ext"]),
                          "sendero_tolerancia_cm": round(p["sendero_tol_cm"])},
            "control": {"suavidad_duty": round(p["suavidad_duty"], 3), "suavidad_periodo_s": round(p["suavidad_periodo_s"], 3)},
            "slam": {"cono_frontal_deg": round(p["cono_int" if i else "cono_ext"])}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--informe", default=None)
    ap.add_argument("--entorno", default="interior", choices=["interior", "exterior"])
    ap.add_argument("--aplicar", action="store_true")
    ap.add_argument("--revertir", default=None)
    a = ap.parse_args()
    if a.revertir:
        antes = json.loads(Path(a.revertir).read_text())
        for k, v in antes["limites"].items():
            print("C3", k, "->", v, api("/api/c3/limites", {"nombre": k, "valor": v}).get("ok"))
        for sec, vals in antes["secciones"].items():
            print(sec, "->", vals, "ok" if api(f"/api/params/{sec}", vals) else "")
        return
    inf = Path(a.informe) if a.informe else Path(sorted(glob.glob(os.path.expanduser("~/.agrobot/aprendizaje/intensivo_*/informe.json")))[-1])
    mejor = json.loads(inf.read_text())["mejor"]
    p = mejor["params"]
    print(f"informe: {inf} · candidato {mejor['nombre']} · entorno {a.entorno}")
    lim_nuevos = {LIM[k]: round(p[k]) for k in LIM}
    j = jetson(p, a.entorno)
    try:
        lim_ahora = api("/api/c3/limites")["limites"]
        cfg = api("/api/params")
    except Exception as e:
        sys.exit(f"el backend no responde ({e}): enciéndelo y enlaza la C3")
    print("\nFIRMWARE (C3):")
    for k, v in lim_nuevos.items():
        print(f"  {k:8s} {lim_ahora.get(k)} -> {v}")
    print("\nJETSON:")
    for sec, vals in j.items():
        for k, v in vals.items():
            print(f"  {sec}.{k:22s} {cfg[sec].get(k)} -> {v}")
    if not a.aplicar:
        print("\n(solo vista previa: añade --aplicar para aplicar)")
        return
    antes = {"limites": {k: lim_ahora[k] for k in lim_nuevos if k in lim_ahora},
             "secciones": {sec: {k: cfg[sec].get(k) for k in vals} for sec, vals in j.items()}}
    ruta = Path.home() / ".agrobot" / "aprendizaje" / f"antes_de_aplicar_{time.strftime('%Y%m%d_%H%M')}.json"
    ruta.write_text(json.dumps(antes, indent=1))
    print("\nguardado el estado anterior en", ruta)
    for k, v in lim_nuevos.items():
        for _ in range(6):
            r = api("/api/c3/limites", {"nombre": k, "valor": v}) if True else None
            if r.get("ok"):
                break
            time.sleep(1.5)
        print("C3", k, v, "ok" if r.get("ok") else r)
    for sec, vals in j.items():
        api(f"/api/params/{sec}", vals)
        print(sec, vals)
    print("\nlisto. Para volver atrás: python3 tools/aplicar_resultado_entrenamiento.py --revertir", ruta)


if __name__ == "__main__":
    main()
