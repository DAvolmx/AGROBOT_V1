"""Segundo barrido del sonar (2026-10-26/27): el primero mostró que NINGUNA variante de la lógica por lados cambiaba el resultado (todas ~10 % vuelve): el fallo era el
número de pulsos de giro por decisión (este robot avanza ~5 cm por pulso de giro: 15 pulsos para 90° = 75 cm hacia la pared). Ahora se limitan los pulsos por
decisión (se re-decide con lo nuevo del mapa) y se retrocede antes de un giro grande con poco frente."""
from pathlib import Path

import barrido_sonar as B1
import comparar_estrategias as CE

CE.SAL = Path(__file__).resolve().parents[1] / "docs" / "estrategias" / "sonar_barrido2"
E = B1.E
CE.ESTRATEGIAS.clear()
CE.ESTRATEGIAS.update({
    "D00 hueco + predictor (referencia)": dict(CE.HUECO, **B1.PRED),
    "D01 sonar+lados, máx 3 pulsos": dict(E, mx_sonar_pulsos_max=3),
    "D02 sonar+lados, máx 2 pulsos": dict(E, mx_sonar_pulsos_max=2),
    "D03 sonar+lados, máx 5 pulsos": dict(E, mx_sonar_pulsos_max=5),
    "D04 máx 3 + atrás antes de giros > 60°": dict(E, mx_sonar_pulsos_max=3, mx_sonar_atras_antes_deg=60.0),
    "D05 máx 3 + atrás antes de giros > 40°": dict(E, mx_sonar_pulsos_max=3, mx_sonar_atras_antes_deg=40.0),
    "D06 D04 con profundidad 1,4 m": dict(E, mx_sonar_pulsos_max=3, mx_sonar_atras_antes_deg=60.0, mx_sonar_prof_m=1.4),
    "D07 D04 solo scan (sin mapa)": dict(E, mx_sonar_pulsos_max=3, mx_sonar_atras_antes_deg=60.0, mx_sonar_modo="scan"),
    "D08 D04 sin centrado ni eje": dict(B1.C, mx_sonar_pulsos_max=3, mx_sonar_atras_antes_deg=60.0),
    "D09 D04 giro máx 90°": dict(E, mx_sonar_pulsos_max=3, mx_sonar_atras_antes_deg=60.0, mx_sonar_max_giro_deg=90.0),
})
if __name__ == "__main__":
    CE.main()
