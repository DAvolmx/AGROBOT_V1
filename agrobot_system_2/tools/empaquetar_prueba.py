#!/usr/bin/env python3
"""Reúne los datos de una prueba en ~/.agrobot/entrenamiento/<nombre>/ para simular y entrenar después:
  campo/ (enlace a la grabación), datos/ (enlace al lote de eventos + SLAM), intervenciones.jsonl (copia), config.json (copia),
  limites_c3.json (límites del firmware en ese momento), medida_giro.txt (tools/medir_giro.py) y manifest.json (qué hay y cuánto).
Uso: python3 tools/empaquetar_prueba.py <nombre> --campo <carpeta> --datos <carpeta> [--desde <unix>]"""
import argparse, json, os, shutil, subprocess, sys, time, urllib.request
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent
HOME = Path.home() / ".agrobot"


def contar(path, tipo_clave=None):
    n, tipos = 0, {}
    for l in open(path):
        n += 1
        if tipo_clave:
            try:
                t = json.loads(l).get(tipo_clave)
            except ValueError:
                continue
            tipos[t] = tipos.get(t, 0) + 1
    return n, tipos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("nombre")
    ap.add_argument("--campo", required=True)
    ap.add_argument("--datos", required=True)
    ap.add_argument("--desde", type=float, default=0.0, help="solo intervenciones con t >= este unix")
    a = ap.parse_args()
    dest = HOME / "entrenamiento" / a.nombre
    dest.mkdir(parents=True, exist_ok=True)
    campo, datos = Path(a.campo).expanduser().resolve(), Path(a.datos).expanduser().resolve()
    for nombre, destino in (("campo", campo), ("datos", datos)):
        enlace = dest / nombre
        if enlace.is_symlink() or enlace.exists():
            enlace.unlink()
        enlace.symlink_to(destino)
    man = {"nombre": a.nombre, "creado": time.strftime("%Y-%m-%d %H:%M:%S"), "campo": str(campo), "datos": str(datos)}
    n, _ = contar(campo / "variables.jsonl")
    man["campo_muestras_5hz"] = n
    n, tipos = contar(datos / "eventos.jsonl", "tipo")
    man["datos_eventos"], man["datos_tipos"] = n, tipos
    interv = [l for l in open(HOME / "aprendizaje" / "intervenciones" / time.strftime("%Y-%m-%d.jsonl"))
              if json.loads(l)["t"] >= a.desde]
    (dest / "intervenciones.jsonl").write_text("".join(interv))
    cuentas = {}
    for l in interv:
        t = json.loads(l)["tipo"]
        cuentas[t] = cuentas.get(t, 0) + 1
    man["intervenciones"] = {"total": len(interv), "por_tipo": cuentas}
    shutil.copy(RAIZ / "jetson" / "backend" / "agrobot_config.json", dest / "config.json")
    for nombre, url in (("limites_c3", "http://localhost:8080/api/c3/limites"), ("asistido", "http://localhost:8080/api/asistido")):
        try:
            (dest / f"{nombre}.json").write_text(json.dumps(json.load(urllib.request.urlopen(url, timeout=5)), indent=1, ensure_ascii=False))
        except Exception as exc:
            man[f"{nombre}_error"] = repr(exc)
    r = subprocess.run([sys.executable, str(RAIZ / "tools" / "medir_giro.py"), "--campo", str(campo), "--datos", str(datos)],
                       capture_output=True, text=True)
    (dest / "medida_giro.txt").write_text(r.stdout + r.stderr)
    man["tamano_mb"] = round(sum(f.stat().st_size for f in campo.rglob("*") if f.is_file()) / 1e6)
    (dest / "manifest.json").write_text(json.dumps(man, indent=1, ensure_ascii=False))
    print(json.dumps(man, indent=1, ensure_ascii=False))
    print("->", dest)


if __name__ == "__main__":
    main()
