"""Guarda TODO lo de la prueba del L (2026-10-28) en ~/.agrobot/pruebas/L_20261028/: estado de la misión, plantas CONSOLIDADAS (6 escaneos -> 4 plantas únicas, la verdad de terreno
del operador; los 6 escaneos crudos se conservan), diagnósticos de PLAGASX por foto, fotos de la visión activa, mapa de la misión, config, salud, referencias a las sesiones de datos y de
campo y ETIQUETA.json. No borra ni cambia nada de lo original. Uso: python3 tools/guardar_prueba_L_20261028.py"""
import glob, json, os, shutil, time, urllib.request
from pathlib import Path

HOME = Path.home() / ".agrobot"
DEST = HOME / "pruebas" / "L_20261028"; (DEST / "fotos").mkdir(parents=True, exist_ok=True)
RAIZ = Path(__file__).resolve().parents[1]
API = "http://localhost:8080"
SCR = Path("/tmp/claude-2002/-home-jetson-Desktop-AGROBOT/9301e484-84d6-44a6-9cad-04b2a7e3f293/scratchpad")


def get(ruta):
    try:
        return json.load(urllib.request.urlopen(API + ruta, timeout=8))
    except Exception as exc:
        return {"error": str(exc)}


mis = get("/api/mision_plantas")
(DEST / "mision_plantas_crudo.json").write_text(json.dumps(mis, indent=1, ensure_ascii=False))
(DEST / "salud.json").write_text(json.dumps(get("/api/health"), indent=1, ensure_ascii=False))
cfg = json.loads((RAIZ / "jetson" / "backend" / "agrobot_config.json").read_text())
(DEST / "config_snapshot.json").write_text(json.dumps({k: cfg.get(k) for k in ("autonomia", "slam", "odometria_webcam", "detection", "control", "ai")}, indent=1, ensure_ascii=False))

fotos = sorted(glob.glob(str(HOME / "vision_activa" / "fotos" / "20261028_*.jpg")))
for f in fotos:
    shutil.copy2(f, DEST / "fotos" / os.path.basename(f))
pl = json.loads((SCR / "plagasx_frente.json").read_text()) if (SCR / "plagasx_frente.json").exists() else []
diag = {d["escaneo"]: d["top"] for d in pl}

raw = {p["n"]: p for p in mis.get("plantas", [])}
def esc(n):
    p = raw.get(n, {})
    return {"escaneo": n, "x": p.get("x"), "y": p.get("y"), "especie": p.get("especie"), "p_col": p.get("p_col"), "plagasx_top3": diag.get(n)}
consolidadas = [
    {"planta": "A", "descripcion": "coliflor con hojas secas (la única col)", "escaneos": [esc(1), esc(3)], "criterio_fusion": "misma maceta, misma forma de hojas y misma mancha de la pared en las fotos frontales 1 y 3; es la única col",
     "diagnostico": "daño visible: PLAGASX da col_dano_insecto 0,54 y 0,89 (la clase más cercana a hojas secas)"},
    {"planta": "B", "descripcion": "planta con flores rojas en maceta acanalada, junto a la puerta de madera", "escaneos": [esc(2)], "criterio_fusion": "sola", "diagnostico": "no es col (p_col 0,32): sin diagnóstico de plaga"},
    {"planta": "C", "descripcion": "planta con flores magenta en maceta negra", "escaneos": [esc(4), esc(5)], "criterio_fusion": "mismas flores y maceta en las fotos 4 (de cerca) y 5 (de lejos); posiciones estimadas a 0,6 m",
     "diagnostico": "no es col (p_col 0,08 y 0,24): sin diagnóstico de plaga"},
    {"planta": "D", "descripcion": "planta de hojas pequeñas junto a un balde grande", "escaneos": [esc(6)], "criterio_fusion": "sola", "diagnostico": "no es col (p_col 0,04): sin diagnóstico de plaga"},
]
(DEST / "plantas_consolidadas.json").write_text(json.dumps({
    "verdad_de_terreno": "4 plantas (1 coliflor + 3 que no son col), dato del operador", "escaneos_crudos": len(raw), "plantas_unicas": len(consolidadas),
    "nota": "La fusión de escaneos es visual y apoyada en el total conocido (4); no hay medición independiente de posiciones: las estimaciones del robot tienen ~1 m de error por el SLAM recién reiniciado. "
            "#2 y #5 se parecen (tallo alto con flores) y se dejaron como plantas distintas por la puerta visible solo en #2.", "plantas": consolidadas}, indent=1, ensure_ascii=False))

for patron, nombre in (("maps/mision_20261028_*", "mapa"),):
    for d in sorted(glob.glob(str(RAIZ.parent / patron)) + glob.glob(str(RAIZ / patron)))[-1:]:
        shutil.copytree(d, DEST / nombre, dirs_exist_ok=True)

refs = {"datos": sorted(glob.glob(str(HOME / "datos" / "20261028_*")))[-1:], "campo": sorted(glob.glob(str(HOME / "campo" / "20261028_*")))[-2:],
        "monitor_10s": str(DEST / "monitor.jsonl"), "recoleccion_sesion": "20261028_014209_c2cc65", "vision_activa_fotos": len(fotos)}
(DEST / "referencias.json").write_text(json.dumps(refs, indent=1, ensure_ascii=False))
(DEST / "ETIQUETA.json").write_text(json.dumps({"escenario": "L de 2,70 × 2,40 m; pasillo de 0,70 m en el primer metro y 0,90 m el resto del tramo largo", "plantas_reales": 4, "coliflor": 1,
    "entorno": "por confirmar (interior/exterior)", "etiquetado": time.strftime("%Y-%m-%d %H:%M:%S"), "fuente": "operador"}, indent=1, ensure_ascii=False))
idx = HOME / "pruebas.json"
reg = json.loads(idx.read_text()) if idx.exists() else {}
reg["20261028_L_plantas"] = {"carpeta": str(DEST), **refs}
idx.write_text(json.dumps(reg, indent=1, ensure_ascii=False))
print("guardado en", DEST); print(sorted(os.listdir(DEST)), "| fotos:", len(fotos))
