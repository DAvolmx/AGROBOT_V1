"""Cámara como LIDAR con las MÉTRICAS MEDIDAS el 2026-10-27 (campo 55°, zona ciega 0,55 m, σ 2 cm + 6 %, sesgo +5 %) y giro compensado cada 3, 30 % macetas bajas, 15 misiones."""
from pathlib import Path

import barrido_sonar as B1
import comparar_estrategias as CE

CE.SAL = Path(__file__).resolve().parents[1] / "docs" / "estrategias" / "camara_lidar3"
CE.DURAS = True
BASE = dict(CE.HUECO, **B1.PRED, mx_mediavuelta=True, mx_mv_giros=4, mx_mv_atras=3, cam_bajos_frac=0.3)
CE.ESTRATEGIAS.clear()
CE.ESTRATEGIAS.update({
    "K0 solo LIDAR": dict(BASE),
    "K1 LIDAR + cámara medida": dict(BASE, cam_lidar=True),
    "K2 LIDAR + cámara medida + giro compensado 3": dict(BASE, cam_lidar=True, mx_giro_comp=3),
})
if __name__ == "__main__":
    CE.main()
