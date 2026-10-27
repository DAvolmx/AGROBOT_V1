"""Barrido de configuraciones del sonar y de la lógica por LADOS (2026-10-26). Mismas 10 misiones y semillas nuevas; parámetros de bajo nivel fijos (los de E11).
Cada configuración es un dict de Config sobre la base «sonar + predictor». Salida: ../docs/estrategias/sonar_barrido/{resultados.json,CONCLUSION.md}."""
from pathlib import Path

import comparar_estrategias as CE

CE.SAL = Path(__file__).resolve().parents[1] / "docs" / "estrategias" / "sonar_barrido"
PRED = dict(CE.PRED, mx_pred_giro_extra_m=0.04)
SON = dict(mx_sonar=True, mx_hueco=False, mx_meta=True, mx_anti_dither=True, mx_visitar=True, mx_memoria_w=1.0, **PRED)
B = dict(SON, mx_sonar_frente="barrido")
C = dict(B, mx_sonar_lado_w=1.0)
D = dict(C, mx_sonar_centrar=True, mx_sonar_tol_cm=6.0)
E = dict(D, mx_sonar_eje=True, mx_sonar_eje_deg=10.0)
CE.ESTRATEGIAS.clear()
CE.ESTRATEGIAS.update({
    "C00 hueco + predictor (referencia)": dict(CE.HUECO, **PRED),
    "C01 sonar base (cono)": dict(SON),
    "C02 sonar + frente barrido": dict(B),
    "C03 + castigo a girar hacia pared": dict(C),
    "C04 + centrado por lados": dict(D),
    "C05 + eje del pasillo": dict(E),
    "C06 C05 giro máx 90°": dict(E, mx_sonar_max_giro_deg=90.0),
    "C07 C05 profundidad 1,4 m": dict(E, mx_sonar_prof_m=1.4),
    "C08 C05 profundidad 0,8 m": dict(E, mx_sonar_prof_m=0.8),
    "C09 C05 36 rayos": dict(E, mx_sonar_n=36),
    "C10 C05 144 rayos": dict(E, mx_sonar_n=144),
    "C11 C05 solo scan (sin mapa)": dict(E, mx_sonar_modo="scan"),
    "C12 C05 sin memoria de visitas": dict(E, mx_sonar_w_visitas=0.0, mx_memoria_w=0.0),
    "C13 C05 meta pesa 2": dict(E, mx_sonar_w_meta=2.0),
    "C14 hueco + frente barrido + centrado": dict(CE.HUECO, **PRED, mx_sonar=True, mx_hueco=True, mx_sonar_frente="barrido", mx_sonar_centrar=True),
})
if __name__ == "__main__":
    CE.main()
