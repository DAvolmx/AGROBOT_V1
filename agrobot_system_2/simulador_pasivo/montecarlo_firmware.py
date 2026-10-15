#!/usr/bin/env python3
"""Monte Carlo de FIRMWARE: busca los límites de control más rápidos que NO
vuelcan, para validar un firmware sólido antes de tocar el robot real.

Complementa a `montecarlo.py` (que optimiza la CALIDAD de navegación: pisadas,
choques, desvío). Este optimiza el eje de SEGURIDAD que el otro no veía, con el
modelo 3D `fallos_3d.py`: dado un chasis (12 kg, CG alto, frente estrecho), ¿qué
combinación de v máx, omega máx, escala, aceleración y umbral de inclinación da
la MAYOR productividad sin volcar en la distribución de terreno de la huerta?

    python3 montecarlo_firmware.py                 # tabla
    python3 montecarlo_firmware.py --n 4000 --json  # JSON para el dashboard

Cada combinación se puntúa contra el MISMO conjunto de situaciones (misma
semilla), para que ninguna gane por terreno fácil. 'Menos puntos = mejor'
(como en montecarlo.py). Vuelcos pesan muchísimo: la seguridad manda.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))

from dataclasses import replace
from dinamica_pasiva import Parametros
import fallos_3d as F

V_TOPE = 1.5                 # techo físico del firmware real (max_v_mps)
R_FILA = 0.5                 # radio de giro que exige el final de hilera (m)

# Pesos del coste (menos es mejor). El vuelco domina: la seguridad no se negocia.
W_VUELCO = 50.0
W_LENTO = 3.0
W_MANIOBRA = 4.0

# Espacio de parámetros del firmware/control que se sortean.
RANGOS = {
    "max_v_mps": (0.3, 1.5),
    "max_omega_rps": (1.5, 4.0),
    "escala_velocidad": (0.3, 1.0),
    "max_linear_accel_mps2": (1.0, 3.0),
    "inclinacion_precaucion_deg": (15.0, 25.0),
}


def sortear(rng):
    return {k: float(rng.uniform(lo, hi)) for k, (lo, hi) in RANGOS.items()}


def generar_situaciones(n, semilla=0):
    """Terreno de huerta: cada 'tramo' tiene un bulto, a veces un giro y a
    veces una pendiente. Distribución fija (misma semilla para todas las
    configuraciones)."""
    rng = np.random.default_rng(semilla)
    sits = []
    for _ in range(n):
        u = rng.random()
        if u < 0.5:   h = rng.uniform(0.0, 0.02)     # suelo con grava: bultos chicos
        elif u < 0.8: h = rng.uniform(0.02, 0.05)    # terrones, medio surco
        elif u < 0.95: h = rng.uniform(0.05, 0.08)   # piedras
        else:          h = rng.uniform(0.08, 0.12)   # escalón grande, raro
        es_giro = rng.random() < 0.35
        omega_demanda = rng.uniform(1.0, 4.5)        # giro que pide el camino
        pendiente = abs(rng.normal(0.0, 7.0))        # cross-slope, mayoría suave
        sits.append((h, es_giro, omega_demanda, pendiente))
    return sits


def evaluar(cfg, sits, est_base):
    """Devuelve coste y desglose de una configuración sobre las situaciones."""
    est = est_base
    v_eff = cfg["max_v_mps"] * cfg["escala_velocidad"]
    vuelcos = man_fallos = 0
    suma_v = 0.0
    for h, es_giro, omega_dem, pendiente in sits:
        v = v_eff
        if pendiente >= cfg["inclinacion_precaucion_deg"]:
            v *= 0.5                                  # PRECAUCION: recorta en talud
        suma_v += v
        vuelca = F.vuelca_obstaculo_energia(h, v, est=est)
        if es_giro:
            omega = min(omega_dem, cfg["max_omega_rps"])
            if F.vuelca_giro_rapido(v, omega, est=est):
                vuelca = True
            if cfg["max_omega_rps"] < omega_dem:      # no pudo cerrar el giro
                man_fallos += 1
        if vuelca:
            vuelcos += 1
    n = len(sits)
    vuelco_rate = vuelcos / n
    v_media = suma_v / n
    maniobra_deficit = man_fallos / n
    # radio mínimo alcanzable a crucero; si no cabe en la hilera, penaliza
    radio_min = v_eff / cfg["max_omega_rps"] if cfg["max_omega_rps"] > 1e-6 else 9.9
    radio_pen = max(0.0, radio_min / R_FILA - 1.0)
    coste = (W_VUELCO * vuelco_rate
             + W_LENTO * (1 - v_media / V_TOPE)
             + W_MANIOBRA * (maniobra_deficit + radio_pen))
    return {"coste": coste, "vuelco_rate": vuelco_rate, "v_eff": v_eff,
            "v_media": v_media, "maniobra_deficit": maniobra_deficit,
            "radio_min_m": radio_min}


def buscar(n=3000, sits_n=500, semilla=0, p=None):
    est = F.Estabilidad(p)
    sits = generar_situaciones(sits_n, semilla)
    rng = np.random.default_rng(semilla + 1)
    cands = []
    # Incluye la config ACTUAL del robot como referencia.
    actual = {"max_v_mps": 1.5, "max_omega_rps": 4.0, "escala_velocidad": 0.5,
              "max_linear_accel_mps2": 3.0, "inclinacion_precaucion_deg": 22.0}
    cands.append(("actual", actual, evaluar(actual, sits, est)))
    for i in range(n):
        c = sortear(rng)
        cands.append((f"v{i}", c, evaluar(c, sits, est)))
    cands.sort(key=lambda x: x[2]["coste"])
    return cands, est


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=3000, help="combinaciones a sortear")
    ap.add_argument("--situaciones", type=int, default=500)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    cands, est = buscar(a.n, a.situaciones)
    mejor = next(c for c in cands if c[0] != "actual")
    actual = next(c for c in cands if c[0] == "actual")

    if a.json:
        print(json.dumps({
            "estabilidad": est.resumen(),
            "mejor": {"params": mejor[1], **mejor[2]},
            "actual": {"params": actual[1], **actual[2]},
            "top": [{"id": c[0], "params": c[1], **c[2]} for c in cands[:10]],
        }, indent=2, ensure_ascii=False))
        return

    r = est.resumen()
    print(f"Chasis: {r['masa_kg']} kg · CG {r['cg_altura_cm']} cm · "
          f"ángulo de vuelco lateral {r['ang_vuelco_lateral_deg']}°\n")
    print(f"{'config':>8} {'coste':>7} {'vuelco%':>8} {'v_ef':>6} "
          f"{'maniobra':>9}  (v,omega,escala,accel,incl)")
    for cid, c, m in [actual] + cands[:8]:
        marca = " <- ACTUAL" if cid == "actual" else ""
        print(f"{cid:>8} {m['coste']:>7.2f} {m['vuelco_rate']*100:>7.1f}% "
              f"{m['v_eff']:>6.2f} {m['maniobra_deficit']*100:>7.0f}%  "
              f"({c['max_v_mps']:.2f}, {c['max_omega_rps']:.2f}, "
              f"{c['escala_velocidad']:.2f}, {c['max_linear_accel_mps2']:.1f}, "
              f"{c['inclinacion_precaucion_deg']:.0f}°){marca}")

    mc = mejor[1]
    print("\n=== MEJOR CONFIGURACIÓN (para agrobot_config.json -> safety) ===")
    print(f"  max_v_mps               : {mc['max_v_mps']:.2f}")
    print(f"  max_omega_rps           : {mc['max_omega_rps']:.2f}")
    print(f"  escala_velocidad        : {mc['escala_velocidad']:.2f}")
    print(f"  max_linear_accel_mps2   : {mc['max_linear_accel_mps2']:.1f}")
    print(f"  (vuelco {mejor[2]['vuelco_rate']*100:.1f}% vs "
          f"actual {actual[2]['vuelco_rate']*100:.1f}% | "
          f"v efectiva {mejor[2]['v_eff']:.2f} vs {actual[2]['v_eff']:.2f} m/s)")
    print("  NO se aplica solo: revisa y confirma antes de tocar el robot real.")


if __name__ == "__main__":
    main()
