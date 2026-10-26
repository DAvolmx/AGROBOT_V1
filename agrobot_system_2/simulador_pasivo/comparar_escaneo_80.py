"""Escaneo a ~80 cm de cada planta sobre el planificador A* (2026-10-26). Mismas 10 misiones y semillas que comparar_estrategias.py.
  E6   A* ida y vuelta, sin escanear a propósito (referencia)
  E9   A* + escanear PASANDO: se detiene a escanear si una planta queda a <= 0,8 m (no se desvía)
  E10  A* + visitar: va hacia cada planta vista y se detiene a ~0,8 m a escanear (sin acercarse más)
  E11  E10 + repetir el escaneo una vez si no detectó nada
Salida: ../docs/estrategias/escaneo80/{resultados.json,CONCLUSION.md}"""
import json, sys, time
from multiprocessing import Pool
from pathlib import Path
import comparar_estrategias as CE
import misiones_sim as MS

CE.SAL = Path(__file__).resolve().parents[1] / "docs" / "estrategias" / "escaneo80"
BASE = dict(CE.PRED, mx_planner=True, mx_plan_inflar_m=0.22, mx_plan_mirada_m=1.0, mx_meta=True, mx_anti_dither=True)
CE.ESTRATEGIAS.clear()
CE.ESTRATEGIAS.update({
    "E6 A* ida y vuelta (referencia)": dict(BASE, mx_visitar=False),
    "E9 A* + escanear pasando (<= 0,8 m)": dict(BASE, mx_visitar=False, mis_dist_insp_m=0.8, mis_escaneo_s=6.4),
    "E10 A* + visitar y parar a ~0,8 m": dict(BASE, mx_visitar=True, mx_plan_tol_planta_m=0.78, mis_dist_visita_m=0.8, mis_dist_insp_m=0.8, mis_escaneo_s=6.4),
    "E11 E10 + repetir escaneo fallido": dict(BASE, mx_visitar=True, mx_plan_tol_planta_m=0.78, mis_dist_visita_m=0.8, mis_dist_insp_m=0.8, mis_escaneo_s=6.4, mis_reintentos=1),
})
if __name__ == "__main__":
    CE.main()
