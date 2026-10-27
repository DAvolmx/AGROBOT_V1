"""Cámara como LIDAR frontal + giro compensado, con 30 % de macetas BAJAS (invisibles al LIDAR), en las 15 misiones sobre «seguir el hueco + predictor + media vuelta». 2026-10-27."""
from pathlib import Path

import barrido_sonar as B1
import comparar_estrategias as CE

CE.SAL = Path(__file__).resolve().parents[1] / "docs" / "estrategias" / "camara_lidar"
CE.DURAS = True
BASE = dict(CE.HUECO, **B1.PRED, mx_mediavuelta=True, mx_mv_giros=4, mx_mv_atras=3, cam_bajos_frac=0.3)
CE.ESTRATEGIAS.clear()
CE.ESTRATEGIAS.update({
    "H0 solo LIDAR": dict(BASE),
    "H1 LIDAR + giro compensado": dict(BASE, mx_giro_comp=1),
    "H2 LIDAR + cámara": dict(BASE, cam_lidar=True),
    "H3 LIDAR + cámara + giro compensado": dict(BASE, cam_lidar=True, mx_giro_comp=1),
})
if __name__ == "__main__":
    CE.main()
