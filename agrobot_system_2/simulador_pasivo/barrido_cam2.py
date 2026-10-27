"""Giro compensado en «seguir el hueco» (ahora sí actúa) × cámara como LIDAR, 30 % de macetas bajas, 15 misiones. 2026-10-27."""
from pathlib import Path

import barrido_sonar as B1
import comparar_estrategias as CE

CE.SAL = Path(__file__).resolve().parents[1] / "docs" / "estrategias" / "camara_lidar2"
CE.DURAS = True
BASE = dict(CE.HUECO, **B1.PRED, mx_mediavuelta=True, mx_mv_giros=4, mx_mv_atras=3, cam_bajos_frac=0.3, cam_lidar=True)
CE.ESTRATEGIAS.clear()
CE.ESTRATEGIAS.update({
    "G0 cámara, sin compensar": dict(BASE),
    "G1 cámara + compensado cada 1": dict(BASE, mx_giro_comp=1),
    "G2 cámara + compensado cada 2": dict(BASE, mx_giro_comp=2),
    "G3 cámara + compensado cada 3": dict(BASE, mx_giro_comp=3),
})
if __name__ == "__main__":
    CE.main()
