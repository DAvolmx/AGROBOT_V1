#!/usr/bin/env python3
"""Activa/desactiva los FRENOS AUTOMÁTICOS del robot (reinicia el backend para aplicarlo).

  python3 scripts/frenos_autonomo.py off     # sin frenos automáticos (pedido del operador, 2026-10-16)
  python3 scripts/frenos_autonomo.py on      # frenos de seguridad activos (valores recomendados)
  python3 scripts/frenos_autonomo.py estado

Qué controla:
  safety.freno_lidar_autonomo                freno por LIDAR en autónomo (no avanzar contra algo / sin LIDAR fresco)
  slam.freno_camara                          freno por cámara (suelo libre)
  autonomia.sendero_parada_incierta_ciclos   la exploración se para si pierde el sendero (0 = nunca)
  autonomia.sendero_si_no_cabe               "aviso" = solo avisa; "frenar" = para
  safety.supervisor_activo                   protección final del supervisor
NO toca la PARADA DE EMERGENCIA (botón rojo) ni el failsafe de la C3 (500 ms sin órdenes).
Sin frenos, el robot NO se detiene solo ante un obstáculo: hay que vigilarlo con la mano en la emergencia.
"""
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jetson" / "backend"))
import config_store as c  # noqa: E402

OFF = {"safety": {"freno_lidar_autonomo": False, "supervisor_activo": False},
       "slam": {"freno_camara": False},
       "autonomia": {"sendero_parada_incierta_ciclos": 0, "sendero_si_no_cabe": "aviso"}}
ON = {"safety": {"freno_lidar_autonomo": True, "supervisor_activo": False},
      "slam": {"freno_camara": True},
      "autonomia": {"sendero_parada_incierta_ciclos": 5, "sendero_si_no_cabe": "aviso"}}


def estado():
    d = c.load()
    return {s: {k: d[s].get(k) for k in v} for s, v in OFF.items()}


if __name__ == "__main__":
    modo = sys.argv[1] if len(sys.argv) > 1 else "estado"
    if modo in ("on", "off"):
        subprocess.run(["sudo", "systemctl", "stop", "agrobot-backend"], check=False)
        for seccion, valores in (ON if modo == "on" else OFF).items():
            c.update_section(seccion, valores)
        subprocess.run(["sudo", "systemctl", "start", "agrobot-backend"], check=False)
    print(f"frenos automáticos: {modo.upper() if modo in ('on', 'off') else 'estado'}")
    for s, v in estado().items():
        print(f"  {s}: {v}")
