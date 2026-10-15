#!/usr/bin/env python3
"""Compara giro CONTINUO vs ZIGZAG A PASOS siguiendo un arco, con la física del
eje pasivo (dinamica_pasiva.py). Valida lo que el operador vio en el robot:
el manejo a pasos traza mejor la curva porque no deja el pivote saturado.

    python3 comparar_zigzag.py
"""
import math
import numpy as np

from dinamica_pasiva import DinamicaPasiva, Parametros
from controlador_zigzag import ControladorZigzag


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def seguir_arco(tipo, R=1.2, v=0.5, t_max=14.0, dt=0.05, k=2.0):
    """Sigue un arco de radio R (centro en (0,R), giro a la izquierda desde el
    origen). Devuelve métricas de seguimiento y de uso del pivote."""
    p = Parametros()
    d = DinamicaPasiva(pose=(0.0, 0.0, 0.0), parametros=p)
    ctrl = ControladorZigzag(v=v) if tipo == "zigzag" else None
    cx, cy = 0.0, R
    err_ct, sat, delta_abs, recorrido = [], 0, [], 0.0
    ultimo = np.array([0.0, 0.0])
    n = int(t_max / dt)
    for i in range(n):
        x, y, psi, delta = d.q
        rumbo_des = math.atan2(y - cy, x - cx) + math.pi / 2  # tangente CCW
        err = _wrap(rumbo_des - psi)
        if tipo == "continuo":
            # Baseline justo: feedforward de curvatura (v/R) + corrección.
            v_cmd, omega = v, float(np.clip(v / R + k * err, -1.5, 1.5))
        else:
            v_cmd, omega = ctrl.ordenar(err, i * dt)
        d.step(dt, v_cmd, omega)
        x, y = d.q[:2]
        err_ct.append(abs(math.hypot(x - cx, y - cy) - R))
        delta_abs.append(abs(d.q[3]))
        if abs(d.q[3]) >= p.limit - 1e-3:
            sat += 1
        recorrido += float(np.hypot(x - ultimo[0], y - ultimo[1]))
        ultimo = np.array([x, y])
    return {
        "tipo": tipo,
        "error_seguimiento_cm": round(float(np.sqrt(np.mean(np.square(err_ct)))) * 100, 1),
        "error_max_cm": round(max(err_ct) * 100, 1),
        "pivote_medio_deg": round(math.degrees(float(np.mean(delta_abs))), 1),
        "pivote_saturado_%": round(100 * sat / n, 1),
        "recorrido_m": round(recorrido, 2),
    }


if __name__ == "__main__":
    print("Siguiendo un arco de radio 1.2 m a 0.5 m/s con el eje delantero pasivo:\n")
    cols = ["tipo", "error_seguimiento_cm", "error_max_cm",
            "pivote_medio_deg", "pivote_saturado_%", "recorrido_m"]
    etiqueta = {"error_seguimiento_cm": "error RMS", "error_max_cm": "error máx",
                "pivote_medio_deg": "pivote medio", "pivote_saturado_%": "pivote saturado",
                "recorrido_m": "recorrido"}
    res = [seguir_arco("continuo"), seguir_arco("zigzag")]
    print(f"{'':>16} " + " ".join(f"{r['tipo']:>12}" for r in res))
    for c in cols[1:]:
        print(f"{etiqueta[c]:>16} " + " ".join(f"{str(r[c]):>12}" for r in res))
