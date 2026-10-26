"""Sonar vs seguir-el-hueco, con y sin planificador (2026-10-26). Mismas 10 misiones y semillas nuevas que comparar_estrategias.py, con las correcciones del
2026-10-26 ya en el simulador (barrido solo por delante de la cara frontal; giros por cuadrantes).
  S1 hueco + predictor            (E3: el «seguir el hueco» del scan de ahora)
  S2 sonar + predictor            (ecos sobre el mapa acumulado, 360°, con memoria de lo pisado)
  S3 A* + visitar + predictor     (E11)
  S4 A* + visitar + sonar         (E11 con el sonar como reactivo de respaldo)
Salida: ../docs/estrategias/sonar/{resultados.json,CONCLUSION.md}"""
from pathlib import Path

import comparar_estrategias as CE

CE.SAL = Path(__file__).resolve().parents[1] / "docs" / "estrategias" / "sonar"
PRED = dict(CE.PRED, mx_pred_giro_extra_m=0.04)
SON = dict(mx_sonar=True, mx_hueco=False, mx_meta=True, mx_anti_dither=True, mx_visitar=True, mx_memoria_w=1.0)
PLAN = dict(mx_planner=True, mx_plan_inflar_m=0.22, mx_plan_mirada_m=1.0, mx_plan_tol_planta_m=0.78, mis_dist_visita_m=0.8, mis_dist_insp_m=0.8, mis_escaneo_s=6.4,
            mis_reintentos=1)
CE.ESTRATEGIAS.clear()
CE.ESTRATEGIAS.update({
    "S1 hueco + predictor": dict(CE.HUECO, **PRED),
    "S2 sonar + predictor": dict(SON, **PRED),
    "S3 A* + visitar + predictor (E11)": dict(CE.HUECO, mx_hueco=False, mx_visitar=True, **PRED, **PLAN),
    "S4 A* + visitar + sonar": dict(SON, **PRED, **PLAN),
})
if __name__ == "__main__":
    CE.main()
