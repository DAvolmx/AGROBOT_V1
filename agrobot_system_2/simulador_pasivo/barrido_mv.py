"""Media vuelta (K-turn) al llegar al final del camino, en las 15 misiones (10 + 5 duras) con «seguir el hueco + predictor» como base. 2026-10-27."""
from pathlib import Path

import barrido_sonar as B1
import comparar_estrategias as CE

CE.SAL = Path(__file__).resolve().parents[1] / "docs" / "estrategias" / "media_vuelta"
CE.DURAS = True
REF = dict(CE.HUECO, **B1.PRED)
MV = dict(REF, mx_mediavuelta=True)
CE.ESTRATEGIAS.clear()
CE.ESTRATEGIAS.update({
    "M0 referencia (sin media vuelta)": dict(REF),
    "M1 media vuelta 3 giros + 2 atrás": dict(MV, mx_mv_giros=3, mx_mv_atras=2),
    "M2 media vuelta 2 giros + 2 atrás": dict(MV, mx_mv_giros=2, mx_mv_atras=2),
    "M3 media vuelta 4 giros + 3 atrás": dict(MV, mx_mv_giros=4, mx_mv_atras=3),
    "M4 media vuelta 3 giros + 1 atrás": dict(MV, mx_mv_giros=3, mx_mv_atras=1),
})
if __name__ == "__main__":
    CE.main()
