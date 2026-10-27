"""Tercer barrido del sonar (2026-10-27), ya SIN el error que hacía fallar todo episodio con meta (`margin` sin definir en sonar_mapa.mejor_rumbo): los dos barridos
anteriores del 26-oct midieron un sonar que se caía con una excepción (contactos 9 por episodio) y NO sirven. Mismas misiones y semillas."""
from pathlib import Path

import barrido_sonar as B1
import comparar_estrategias as CE

CE.SAL = Path(__file__).resolve().parents[1] / "docs" / "estrategias" / "sonar_barrido3"
E, C, D3 = B1.E, B1.C, dict(B1.E, mx_sonar_pulsos_max=3)
CE.ESTRATEGIAS.clear()
CE.ESTRATEGIAS.update({
    "F00 hueco + predictor (referencia)": dict(CE.HUECO, **B1.PRED),
    "F01 sonar base (cono)": dict(B1.SON),
    "F02 sonar + frente por corredor": dict(B1.B),
    "F03 + castigo a girar hacia pared": dict(C),
    "F04 + centrado por lados": dict(B1.D),
    "F05 + eje del pasillo": dict(E),
    "F06 F05 profundidad 1,4 m": dict(E, mx_sonar_prof_m=1.4),
    "F07 F05 solo scan (sin mapa)": dict(E, mx_sonar_modo="scan"),
    "F08 F05 máx 3 pulsos": dict(D3),
    "F09 F05 máx 3 pulsos + atrás antes de giros > 60°": dict(D3, mx_sonar_atras_antes_deg=60.0),
    "F10 F05 giro máx 90°": dict(E, mx_sonar_max_giro_deg=90.0),
    "F11 hueco + frente por corredor + centrado": dict(CE.HUECO, **B1.PRED, mx_sonar=True, mx_hueco=True, mx_sonar_frente="barrido", mx_sonar_centrar=True),
})
if __name__ == "__main__":
    CE.main()
