#!/usr/bin/env python3
"""Aplica (o deshace) el perfil de manejo óptimo de la tanda del 2026-10-22 en el backend EN MARCHA, sin editar archivos.

  python3 tools/aplicar_perfil_optimo.py            # muestra qué cambiaría
  python3 tools/aplicar_perfil_optimo.py --aplicar  # lo aplica; guarda los valores previos en ~/.agrobot/perfil_previo.json
  python3 tools/aplicar_perfil_optimo.py --deshacer # restaura los valores previos
  --con-rumbo  además enciende autonomia.rumbo_fusion_activo (hazlo solo si tools/probar_recta.py mostró mejora)

Óptimo = pe1_37 de la búsqueda de pulsos (simulación con física calibrada y giro invertido como el real): 82 % de éxito en semillas nuevas,
44 contactos en 96 episodios (la config anterior: 49 % y 186). NO cumplió los criterios de aceptación (cero contactos, ≥90 % en el peor hueco).
No se toca escala_velocidad (el simulador comparó 0,2; el robot lleva 0,13 y no se acelera sin probar).
"""
import json
import sys
import urllib.request
from pathlib import Path

URL = "http://localhost:8080"
PREVIO = Path.home() / ".agrobot" / "perfil_previo.json"
PERFIL = {
    "control": {"max_omega_rps": 2.0, "suavidad_duty": 0.35, "suavidad_periodo_s": 0.2},
    "safety": {"max_angular_accel_rps2": 12},
    "autonomia": {"sendero_tolerancia_cm": 5, "sendero_pared_max_cm": 200, "rumbo_zona_muerta_deg": 6.0, "rumbo_aprende": 0.15,
                  "man_histeresis_cm": 12.0, "man_pausa_s": 0.0, "rumbo_pulso_escala": 0.3},
    "slam": {"cono_lateral_deg": 45, "cono_frontal_deg": 20},
}


def api(ruta, cuerpo=None):
    rq = urllib.request.Request(URL + ruta, data=None if cuerpo is None else json.dumps(cuerpo).encode(), headers={"Content-Type": "application/json"})
    return json.load(urllib.request.urlopen(rq, timeout=10))


def main():
    a = sys.argv[1:]
    actual = api("/api/params")
    actual = actual.get("params", actual)
    if "--deshacer" in a:
        for sec, kv in json.loads(PREVIO.read_text()).items():
            print(sec, api(f"/api/params/{sec}", kv))
        return
    perfil = json.loads(json.dumps(PERFIL))
    if "--con-rumbo" in a:
        perfil["autonomia"]["rumbo_fusion_activo"] = True
    previo = {sec: {k: actual.get(sec, {}).get(k) for k in kv} for sec, kv in perfil.items()}
    for sec, kv in perfil.items():
        for k, v in kv.items():
            print(f"{sec}.{k}: {previo[sec][k]} -> {v}")
    if "--aplicar" not in a:
        print("\n(solo vista previa: añade --aplicar)")
        return
    if not PREVIO.exists():
        PREVIO.write_text(json.dumps({s: {k: v for k, v in kv.items() if v is not None} for s, kv in previo.items()}, indent=1))
    for sec, kv in perfil.items():
        print(sec, api(f"/api/params/{sec}", kv))


if __name__ == "__main__":
    main()
