"""
Calibra el chasis simulado con lo medido conduciendo el robot real.
===================================================================

Lee los pulsos registrados (paso a paso, con LIDAR antes/después) y los giros
medidos con ICP de scans + giróscopo + EKF, y ajusta tres parámetros de
`simulador/robot.py` haciendo en el simulador el MISMO pulso que hace el
robot (modo interior, troceado de `control.suavidad_duty`, rampa de la config):

  decel_inercia     frenada al soltar   -> avance por pulso medido
  rendimiento_giro  giro real / pedido  -> grados por pulso medidos
  retroceso_giro    marcha atrás al girar -> cm de retroceso por pulso

Escribe `simulador/calibracion_real.json`, que `entrenar.aleatorizar()` usa
como centro de lo que sortea. Así el entrenamiento ve el robot que hay, no
el que se suponía.

    python3 tools/calibrar_con_pasillo.py logs/sesion_20261002_manejo
"""
import json
import math
import statistics as st
import sys
from pathlib import Path

import numpy as np

RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ / "simulador"))
sys.path.insert(0, str(RAIZ / "jetson" / "backend"))
import mundo as mundo_mod          # noqa: E402
import robot as robot_mod          # noqa: E402
from pulse_drive import MODOS      # noqa: E402

CFG = json.loads((RAIZ / "jetson" / "backend" / "agrobot_config.json").read_text())


def pulso_sim(v=0.0, w=0.0, *, decel=None, rend=None, retro=0.0, escala_pulso="interior",
              recto=True, T=3.0, dt=0.01):
    """Un pulso como el del robot; devuelve (avance_m con signo, giro_deg)."""
    m = MODOS[escala_pulso]
    dur = m["pulso_recto_s"] if recto else 2 * m["maniobra_medio_s"]
    duty = CFG["control"].get("suavidad_duty", 1.0)
    per = CFG["control"].get("suavidad_periodo_s", 0.3)
    mundo, _ = mundo_mod.pasillo_estrecho()
    r = robot_mod.RobotSim(mundo, (0.0, 0.0, 0.0), CFG, np.random.default_rng(1))
    r.latencia_mando = False
    if decel is not None:
        r.decel_inercia = decel
    if rend is not None:
        r.rendimiento_giro = rend
    r.retroceso_giro = retro
    t = 0.0
    while t < T:
        on = t < dur and (duty >= 0.99 or (t % per) < per * duty)
        r.ordenar(v if on else 0.0, w if on else 0.0)
        r.paso(dt)
        t += dt
    return r.x, math.degrees(r.theta)


def biseccion(f, objetivo, lo, hi, n=30):
    """f creciente o decreciente en [lo, hi]; busca f(x) = objetivo."""
    flo = f(lo)
    for _ in range(n):
        mid = (lo + hi) / 2
        if (f(mid) - objetivo) * (flo - objetivo) > 0:
            lo, flo = mid, f(mid)
        else:
            hi = mid
    return (lo + hi) / 2


def main(carpeta: Path):
    pasos = [json.loads(l) for l in (carpeta / "pasos.jsonl").read_text().splitlines() if l.strip()]
    giros = [json.loads(l) for l in (carpeta / "giros_medidos.jsonl").read_text().splitlines() if l.strip()]

    # ---- avance por pulso (escala 0,4, interior): hueco delantero antes - después
    avances = [p["lidar_antes_cm"]["adelante"] - p["lidar_despues_cm"]["adelante"]
               for p in pasos if p["accion"] == "adelante" and abs(p["escala"] - 0.4) < 1e-6]
    v_pedida = CFG["control"]["max_v_mps"] * 0.4 * MODOS["interior"]["vel_factor"]
    # La C3 recorta a V_MAX (0,55): el pico medido en el eco era 0,36.
    v_pedida = min(v_pedida, 0.55)
    av_med = st.mean(avances) / 100.0
    decel = biseccion(lambda d: pulso_sim(v=v_pedida, decel=d)[0], av_med, 0.15, 8.0)

    # ---- giro: solo los medidos en zona libre (no los apoyados en un poste)
    libres = [g for g in giros if "poste" not in g["lugar"]]
    grados = [abs(g["icp_giro_deg"]) for g in libres]
    retro_icp = [g["traslacion_m"] for g in libres]
    # Retroceso también de los pasos de giro ya con el sentido corregido (7, 9, 10):
    # el hueco delantero CRECE cuando el robot retrocede.
    retro_pasos = [(p["lidar_despues_cm"]["adelante"] - p["lidar_antes_cm"]["adelante"]) / 100.0
                   for p in pasos if p["accion"] in ("izquierda", "derecha") and p["paso"] >= 7]
    retro_med = st.mean(retro_icp + retro_pasos)
    w = libres[0]["omega_c3_rps"]
    giro_ideal = abs(pulso_sim(w=w, decel=decel, rend=1.0, recto=False)[1])
    rend = st.mean(grados) / giro_ideal
    retro_k = biseccion(lambda k: -pulso_sim(w=w, decel=decel, rend=rend, retro=k, recto=False)[0],
                        retro_med, 0.0, 1.0)

    # comprobación: el simulador calibrado reproduce lo medido
    av_sim = pulso_sim(v=v_pedida, decel=decel)[0]
    gx, gdeg = pulso_sim(w=w, decel=decel, rend=rend, retro=retro_k, recto=False)

    res = {
        "fuente": str(carpeta.relative_to(RAIZ)),
        "firmware_giro": libres[0]["firmware"],
        "medido": {
            "avance_cm_por_pulso_escala_0_4": {"media": round(st.mean(avances), 1),
                                                "desvio": round(st.pstdev(avances), 1), "n": len(avances),
                                                "valores": avances},
            "giro_deg_por_pulso": {"media": round(st.mean(grados), 2), "n": len(grados), "valores": grados},
            "retroceso_cm_por_pulso_de_giro": {"media": round(retro_med * 100, 1),
                                               "n": len(retro_icp) + len(retro_pasos),
                                               "valores": [round(x * 100, 1) for x in retro_icp + retro_pasos]},
        },
        "ajustado": {"decel_inercia": round(decel, 2), "rendimiento_giro": round(rend, 3),
                     "retroceso_giro": round(retro_k, 3)},
        "comprobacion_sim": {"avance_cm": round(av_sim * 100, 1), "giro_deg": round(abs(gdeg), 2),
                             "retroceso_cm": round(-gx * 100, 1),
                             "giro_ideal_deg_rend_1": round(giro_ideal, 1)},
        "config": {"suavidad_duty": CFG["control"].get("suavidad_duty"),
                   "max_linear_accel_mps2": CFG["safety"]["max_linear_accel_mps2"],
                   "max_angular_accel_rps2": CFG["safety"]["max_angular_accel_rps2"]},
    }
    salida = RAIZ / "simulador" / "calibracion_real.json"
    salida.write_text(json.dumps(res, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(res, indent=2, ensure_ascii=False))
    print(f"-> {salida.relative_to(RAIZ)}")


if __name__ == "__main__":
    main((RAIZ / sys.argv[1]).resolve() if len(sys.argv) > 1
         else RAIZ / "logs" / "sesion_20261002_manejo")
