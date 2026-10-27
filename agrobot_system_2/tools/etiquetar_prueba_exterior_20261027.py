"""Etiqueta como EXTERIOR la prueba del 2026-10-27 (sesión de datos 20261027_215547 + sesiones de campo 215828 y 221926), a petición del operador.
  · escribe ETIQUETA.json junto a cada sesión (no toca los datos crudos);
  · pone etiqueta «exterior» en las muestras del clasificador de entorno (~/.agrobot/entorno/20261027.jsonl) del TRAMO AL AIRE LIBRE (t = 150–708 s de la sesión;
    después el robot entró a una zona cubierta/interior: el brillo de la cámara cae de 0,57 a 0,3 y los fotogramas del segundo video son de interior) que aún no
    tengan etiqueta del operador, con copia de seguridad previa; las que ya tenían «exterior» no se cambian;
  · anota la prueba en ~/.agrobot/pruebas.json.
Idempotente: se puede repetir. Uso: python3 tools/etiquetar_prueba_exterior_20261027.py [--dry]"""
import argparse, json, shutil, time
from pathlib import Path

import numpy as np

HOME = Path.home() / ".agrobot"
SES = HOME / "datos" / "20261027_215547"
CAMPOS = [HOME / "campo" / "20261027_215828_25f568", HOME / "campo" / "20261027_221926_28a5d4"]
TRAMO_AIRE_LIBRE = (150.0, 708.0)          # s de sesión (reloj monótono)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--dry", action="store_true"); a = ap.parse_args()
    R = [json.loads(l) for l in open(SES / "eventos.jsonl") if l.startswith("{")]
    te, ue = np.array([r["t"] for r in R]), np.array([r["t_unix"] for r in R])
    off = ue - te
    salto = np.where(np.abs(np.diff(off)) > 1)[0]
    T0 = float(off[0]); T_SALTO = float(te[salto[0]]) if len(salto) else 1e18; D = float(off[salto[0] + 1] - off[salto[0]]) if len(salto) else 0.0
    mono = lambda u: u - T0 if u < T0 + T_SALTO + 1 else u - T0 - D
    f = HOME / "entorno" / "20261027.jsonl"
    lineas = f.read_text().splitlines()
    nuevas, cambiadas, ya = [], 0, 0
    for l in lineas:
        try:
            e = json.loads(l)
        except ValueError:
            nuevas.append(l); continue
        if e.get("rasgos") and TRAMO_AIRE_LIBRE[0] <= mono(e["t"]) <= TRAMO_AIRE_LIBRE[1]:
            if e.get("etiqueta") is None:
                e["etiqueta"], e["etiqueta_fuente"] = "exterior", "operador_retroactiva_20261027"; cambiadas += 1
            elif e.get("etiqueta") == "exterior":
                ya += 1
        nuevas.append(json.dumps(e, ensure_ascii=False))
    print(f"muestras de entorno: {cambiadas} etiquetadas ahora, {ya} ya estaban como exterior")
    if not a.dry:
        bak = f.with_suffix(".jsonl.bak_antes_etiqueta_exterior_20261027")
        if not bak.exists():
            shutil.copy2(f, bak)
        f.write_text("\n".join(nuevas) + "\n")
    meta = {"entorno": "exterior", "etiquetado": time.strftime("%Y-%m-%d %H:%M:%S"), "fuente": "indicación del operador (2026-10-27)",
            "descripcion": "Prueba de la misión de plantas en maceta en un patio/jardín al aire libre (macetas con suculentas y ornamentales junto a un muro), con mando manual intercalado.",
            "tramo_aire_libre_s": list(TRAMO_AIRE_LIBRE),
            "nota": "A partir de t≈708 s el robot entró a una zona cubierta/interior (brillo de cámara 0,57→0,3; el video 20261027_221926 es de interior). El reloj de pared saltó +711,5 s en t=527 s: usar el tiempo monótono `t`."}
    for d in [SES] + CAMPOS:
        m = dict(meta)
        if "221926" in d.name:
            m["entorno"] = "exterior_cubierto_o_interior"
            m["nota"] += " Esta sesión es el tramo cubierto/interior."
        if not a.dry:
            (d / "ETIQUETA.json").write_text(json.dumps(m, indent=1, ensure_ascii=False))
        print("ETIQUETA.json ->", d.name, m["entorno"])
    idx = HOME / "pruebas.json"
    reg = json.loads(idx.read_text()) if idx.exists() else {}
    reg["20261027_exterior_plantas"] = {"sesion_datos": SES.name, "sesiones_campo": [c.name for c in CAMPOS], **meta}
    if not a.dry:
        idx.write_text(json.dumps(reg, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
