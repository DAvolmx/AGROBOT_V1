"""¿Cuánta holgura lateral necesita de verdad este chasis para no rozar?

Motivo (2026-10-14): `geometria_robot.HOLGURA_LATERAL_M` del backend valía
0,06 m, un número puesto a ojo. El banco lo desmiente: en el pasillo de 50 cm
de `paso_estrecho` (6,5 cm de holgura por lado, o sea MÁS que el umbral) el
robot choca en 0,67-0,83 episodios de cada uno. El chasis rústico deriva solo,
así que la holgura que hace falta no es una tolerancia de fabricación: es el
desvío que el control deja sin corregir.

Esto BARRE el ancho del pasillo y mide colisiones y progreso, para fijar la
holgura con una medida en vez de una corazonada. Sigue siendo el simulador:
masa, fricción e inercias son estimaciones (`contrato_robot.json`
`calibrated: false`), así que el número que salga es una cota del SIMULADOR,
no del robot. Vale para no ir por debajo; no para certificar que pasa.

Uso:  python3 calibrar_holgura.py [--semillas 8] [--anchos 0.44,0.50,...]
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

import evaluacion as E
from robot_rustico import parametros_rusticos

SALIDA = Path.home() / ".agrobot" / "aprendizaje" / "holgura.json"
VIA_TRASERA_M = 0.37


def _escenario(ancho_m):
    """Pasillo recto de ese ancho interior."""
    media = ancho_m / 2.0
    recta = [(x, 0.0) for x in np.linspace(0, 3.0, 20)]
    return {"ruta": recta, "mu": 0.65, "h_obs": 0.0, "obs_pos": None,
            "mundo": E._Mundo(paredes=[(0, media, 3, media),
                                       (0, -media, 3, -media)])}


def barrer(anchos, semillas, cfg_ctrl):
    env = E.envolvente_chasis(parametros_rusticos(0))
    filas = []
    for ancho in anchos:
        esc = _escenario(ancho)
        col, prog, rmse = [], [], []
        for s in semillas:
            ctrl = E.Controlador(**cfg_ctrl)
            ctrl.envolvente = env
            m = E.simular(esc, ctrl, parametros_rusticos(s), s)
            col.append(m["colisiones"]); prog.append(m["progreso"])
            rmse.append(m["rmse_cm"])

        holgura = (ancho - VIA_TRASERA_M) / 2.0
        filas.append({"ancho_m": round(ancho, 3),
                      "holgura_cm": round(holgura * 100, 1),
                      "colisiones_medias": round(float(np.mean(col)), 2),
                      "episodios_con_choque": round(float(np.mean([c > 0 for c in col])), 2),
                      "progreso": round(float(np.mean(prog)), 3),
                      "rmse_cm": round(float(np.mean(rmse)), 1)})
    return filas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--semillas", type=int, default=8)
    ap.add_argument("--anchos", type=str, default=None)
    ap.add_argument("--kp", type=float, default=4.0)
    ap.add_argument("--ki", type=float, default=2.5)
    ap.add_argument("--v", type=float, default=0.4)
    a = ap.parse_args()
    anchos = ([float(x) for x in a.anchos.split(",")] if a.anchos
              else [0.42, 0.45, 0.49, 0.53, 0.57, 0.61, 0.65, 0.75])
    semillas = list(range(a.semillas))
    # Por defecto las ganancias que el ajuste aceptó el 2026-10-14
    # (train 1,802 / val 2,056 / test 1,754). La holgura que hace falta
    # DEPENDE del controlador: es su pico de desvío, no una propiedad del
    # chasis, así que medirla con otras ganancias da otro número.
    cfg = {"kp": a.kp, "ki": a.ki, "v": a.v, "freno": True}

    print(f"vía trasera {VIA_TRASERA_M*100:.0f} cm · {len(semillas)} semillas "
          f"· controlador {cfg}\n", flush=True)
    filas = barrer(anchos, semillas, cfg)
    print(f"{'ancho':>7} {'holgura':>8} {'colis':>7} {'% con choque':>13} "
          f"{'progreso':>9} {'rmse':>7}")
    for f in filas:
        print(f"{f['ancho_m']*100:>6.0f}c {f['holgura_cm']:>7.1f}c "
              f"{f['colisiones_medias']:>7.2f} {f['episodios_con_choque']*100:>12.0f}% "
              f"{f['progreso']:>9.2f} {f['rmse_cm']:>6.1f}c")

    limpios = [f for f in filas if f["episodios_con_choque"] == 0.0]
    print()
    if limpios:
        f = limpios[0]
        print(f"Primera holgura SIN ningún choque: {f['holgura_cm']} cm/lado "
              f"(pasillo {f['ancho_m']*100:.0f} cm)")
    else:
        print("Ninguna de las holguras probadas evita todos los choques.")

    SALIDA.parent.mkdir(parents=True, exist_ok=True)
    SALIDA.write_text(json.dumps({"via_trasera_m": VIA_TRASERA_M,
                                  "controlador": cfg, "semillas": semillas,
                                  "filas": filas}, indent=2, ensure_ascii=False))
    print(f"Guardado en {SALIDA}")


if __name__ == "__main__":
    main()
