"""Calibración del simulador con DATOS REALES del robot (2026-10-22). Un solo sitio, con valores por defecto si no hay archivos.

Fuentes (todas en ~/.agrobot/calibracion/):
  real_parado.json      tools/caracterizar_parado.py: ruido real del LIDAR (σ(r) = a + b·r, pérdida de rayos) y del IMU con el robot PARADO.
  pulsos_real.json      simulador_pasivo/calibrar_pulsos.py: variabilidad real de los pulsos (giros muertos, dispersión del giro y de la deriva recta),
                        ajustada a los cuantiles medidos en 586 movimientos reales.
Variable de entorno AGROBOT_CAL=off -> comportamiento anterior (σ LIDAR 1 cm, pulsos sin variabilidad), para comparar.

HONESTIDAD: el ruido del robot PARADO es un mínimo; en marcha hay vibración y movimiento del haz. `lidar_factor_marcha` (HIPÓTESIS, sin medir)
lo multiplica para no entrenar con sensores mejores que los reales.
"""
import json
import os
from pathlib import Path

RUTA = Path.home() / ".agrobot" / "calibracion"

ANTIGUO = {"lidar_sigma_a": 0.01, "lidar_sigma_b": 0.0, "lidar_dropout": 0.0, "lidar_factor_marcha": 1.0,
           "giro_p_muerto": 0.0, "giro_mu": 0.0, "giro_sigma": 0.0, "deriva_p_grande": 0.0, "deriva_sd_chica": 0.0, "deriva_sd_grande": 0.0,
           "imu_gyro_sigma_dps": 0.3, "imu_gyro_sesgo_dps": 0.5, "imu_acc_sigma_g": 0.02}
DEFECTO_REAL = {"lidar_sigma_a": 0.00013, "lidar_sigma_b": 0.00163, "lidar_dropout": 0.02, "lidar_factor_marcha": 3.0,
                "giro_p_muerto": 0.0, "giro_mu": 0.0, "giro_sigma": 0.0, "deriva_p_grande": 0.0, "deriva_sd_chica": 0.0, "deriva_sd_grande": 0.0,
                "imu_gyro_sigma_dps": 0.17, "imu_gyro_sesgo_dps": 0.05, "imu_acc_sigma_g": 0.01}


def cargar() -> dict:
    if os.environ.get("AGROBOT_CAL", "").lower() == "off":
        return dict(ANTIGUO)
    c = dict(DEFECTO_REAL)
    try:
        p = json.loads((RUTA / "real_parado.json").read_text())
        c["lidar_sigma_a"] = float(p["lidar"]["sigma_m"]["a"])
        c["lidar_sigma_b"] = float(p["lidar"]["sigma_m"]["b"])
        d = [v["dropout_pct"] for v in p["lidar"]["por_distancia"].values()]
        c["lidar_dropout"] = round(sum(d) / len(d) / 100.0, 4)
        g = p["imu"]["giroscopio_dps"]
        c["imu_gyro_sigma_dps"] = float(max(g[k]["desv"] for k in g))
        c["imu_acc_sigma_g"] = float(max(p["imu"]["acelerometro_g"][k]["desv"] for k in p["imu"]["acelerometro_g"]))
    except (OSError, KeyError, ValueError):
        pass
    try:
        c.update(json.loads((RUTA / "pulsos_real.json").read_text())["parametros"])
    except (OSError, KeyError, ValueError):
        pass
    return c


CAL = cargar()
