"""El sonar SOLO como escape cuando el robot se atasca en el mismo sitio (2026-10-27). Base: «seguir el hueco + predictor» (la mejor hasta ahora). Cuando el escape de nicho
se dispara, el sonar elige la salida mirando el mapa en 360° (meta, profundidad, lo menos pisado) en vez del giro al lado contrario de siempre.
G7 es el CONTROL: mismo disparo rápido pero con el escape viejo (para separar el efecto del disparo del efecto del sonar)."""
from pathlib import Path

import barrido_sonar as B1
import comparar_estrategias as CE

CE.SAL = Path(__file__).resolve().parents[1] / "docs" / "estrategias" / "sonar_escape"
REF = dict(CE.HUECO, **B1.PRED)
ESC = dict(REF, mx_sonar_escape=True)
CE.ESTRATEGIAS.clear()
CE.ESTRATEGIAS.update({
    "G0 hueco + predictor (referencia)": dict(REF),
    "G1 + escape con sonar (25 s, 8 bloqueos)": dict(ESC),
    "G2 + escape con sonar (15 s, 4 bloqueos)": dict(ESC, mx_ventana_s=15.0, mx_atasco_bloqueos=4),
    "G3 + escape con sonar (10 s, 3 bloqueos)": dict(ESC, mx_ventana_s=10.0, mx_atasco_bloqueos=3),
    "G4 G2 con máx 6 pulsos de giro": dict(ESC, mx_ventana_s=15.0, mx_atasco_bloqueos=4, mx_sonar_escape_pulsos=6),
    "G5 G2 sonar solo scan (sin mapa)": dict(ESC, mx_ventana_s=15.0, mx_atasco_bloqueos=4, mx_sonar_modo="scan"),
    "G6 G2 + memoria de visitas": dict(ESC, mx_ventana_s=15.0, mx_atasco_bloqueos=4, mx_memoria_w=1.0),
    "G7 CONTROL: disparo rápido, escape viejo (15 s, 4)": dict(REF, mx_ventana_s=15.0, mx_atasco_bloqueos=4),
})
if __name__ == "__main__":
    CE.main()
