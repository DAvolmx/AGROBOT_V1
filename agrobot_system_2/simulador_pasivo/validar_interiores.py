"""Validación en INTERIORES: habitación con macetas, pasillos y eslalon.

Para la defensa de tesis (2026-10-16). Mide lo que una tabla de tesis necesita:
éxito de recorrido, contactos con macetas, error lateral (RMSE y PICO), tiempo
detenido y holgura necesaria.

Por qué hace falta un banco aparte del de huerta: en interiores el obstáculo NO
es un montículo que se cruza ni una hilera que se pisa, es una MACETA: un
cilindro sólido de 10-15 cm de radio que no se puede pisar ni cruzar, y que el
LIDAR a 18 cm del suelo sí ve (al contrario que el montículo de 5 cm). Y el
espacio es chico: los umbrales de huerta (100 cm libres delante para avanzar,
corte a 52 cm) dejan al robot clavado en una sala.

Escenarios:
  pasillo_macetas  dos filas de macetas formando un pasillo recto.
  eslalon_macetas  macetas alternas: obliga a corregir en los dos sentidos.
  esquina_maceta   giro de 90° con una maceta en el vértice interior.
  sala_vacia       referencia: la misma sala sin macetas.

LÍMITES: es el simulador, con `contrato_robot.json` `calibrated: false`. Las
macetas son cilindros perfectos y el suelo es uniforme; una sala real tiene
reflejos, patas de mesa y suelo resbaladizo. Sirve para fijar umbrales y
comparar configuraciones, no para certificar que el robot no toca nada.

Uso:
  python3 validar_interiores.py                       # tabla completa
  python3 validar_interiores.py --anchos 0.6,0.7,0.8  # barrido de pasillo
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

import evaluacion as E
import cinematica_pasiva as K
from robot_rustico import parametros_rusticos

SALIDA = Path.home() / ".agrobot" / "aprendizaje" / "interiores.json"

#: Largo de los recorridos de interior (m). En una sala de 4 x 3 m un tramo de
#: 1,5 m es lo que de verdad hay entre macetas; además, a la velocidad que
#: impone el frenado (0,195 m/s pedidos, ~0,06 reales) 3 m serían 50 s de
#: episodio y la rejilla no acaba en un tiempo razonable.
LARGO_INTERIOR_M = 1.5

#: Maceta tipo: radio del borde superior. Sólida: ni se pisa ni se cruza.
MACETA_R = 0.12
#: Formato de objeto del simulador: (x, y, radio, altura, solido>0.5)
MACETA_ALTURA = 0.25


def _maceta(x, y, r=MACETA_R):
    return (x, y, r, MACETA_ALTURA, 1.0)


def _sala(largo=4.0, ancho=3.0):
    """Paredes de una sala rectangular centrada en el eje de avance."""
    a = ancho / 2.0
    return [(-0.5, a, largo, a), (-0.5, -a, largo, -a),
            (-0.5, -a, -0.5, a), (largo, -a, largo, a)]


def _pasillo_macetas(ancho_libre, largo=LARGO_INTERIOR_M, paso=0.45):
    """Dos filas de macetas que dejan `ancho_libre` de paso entre bordes."""
    borde = ancho_libre / 2.0 + MACETA_R
    objetos = []
    x = 0.6
    while x <= largo:
        objetos.append(_maceta(x, borde))
        objetos.append(_maceta(x, -borde))
        x += paso
    return objetos


def _eslalon_macetas(ancho_libre, largo=LARGO_INTERIOR_M, paso=0.6):
    """Macetas alternas a un lado y a otro: obliga a corregir en los dos
    sentidos, que es donde el pivote pasivo lo pasa peor."""
    borde = ancho_libre / 2.0 + MACETA_R
    objetos, lado, x = [], 1, 0.5
    while x <= largo:
        objetos.append(_maceta(x, lado * borde))
        lado *= -1
        x += paso
    return objetos


def escenarios(ancho_libre=0.70):
    recta = [(x, 0.0) for x in np.linspace(0, LARGO_INTERIOR_M, 16)]
    # El giro usa radio_seguro(): con R_min el pivote va al tope y no queda
    # autoridad para corregir (ver cinematica_pasiva).
    r = K.radio_seguro()
    # Giro recortado: 90° completos con radio seguro (1,51 m) no caben en una
    # sala de 4 m. Se recorre el arco hasta LARGO_INTERIOR_M de recorrido.
    ang_max = min(math.pi / 2, LARGO_INTERIOR_M / r)
    esquina = [(r * math.sin(a), r - r * math.cos(a))
               for a in np.linspace(0, ang_max, 24)]
    base = dict(mu=0.8, h_obs=0.0, obs_pos=None)   # mu 0.8: suelo de interior
    return {
        "sala_vacia": {**base, "ruta": recta, "mundo": E._Mundo(_sala())},
        "pasillo_macetas": {**base, "ruta": recta,
                            "mundo": E._Mundo(_sala(), _pasillo_macetas(ancho_libre))},
        "eslalon_macetas": {**base, "ruta": recta,
                            "mundo": E._Mundo(_sala(), _eslalon_macetas(ancho_libre))},
        "esquina_maceta": {**base, "ruta": esquina,
                           "mundo": E._Mundo(_sala(largo=2.5, ancho=4.0),
                                             [_maceta(r * 0.75, r * 0.25)])},
    }


def _episodio(esc, semilla, cfg):
    """Un recorrido. Devuelve métricas con PICO además de RMSE: el pico es lo
    que decide si toca una maceta (ver geometria_robot)."""
    from dataclasses import replace
    from dinamica_pasiva import DinamicaPasiva
    import huella_pasiva as H

    ruta = esc["ruta"]
    p = replace(parametros_rusticos(semilla), mu=esc["mu"], substep=0.01)
    geo = H.geo_de_parametros(p)
    rng = np.random.default_rng(10_000 + semilla)
    largo = sum(math.dist(a, b) for a, b in zip(ruta, ruta[1:]))
    psi0 = math.atan2(ruta[1][1] - ruta[0][1], ruta[1][0] - ruta[0][0])
    d = DinamicaPasiva(pose=(ruta[0][0] - E.MID * math.cos(psi0),
                             ruta[0][1] - E.MID * math.sin(psi0), psi0),
                       parametros=p)
    ctrl = E.Controlador(**cfg)
    ctrl.envolvente = E.envolvente_chasis(parametros_rusticos(0))
    ctrl.reset()

    errs, pico, colis, s_max, quieto = [], 0.0, 0, 0.0, 0
    prev, dt = False, 0.08
    presupuesto = max(E.T_MAX_MIN_S, largo * SEGUNDOS_POR_METRO_INTERIOR)
    pasos = int(presupuesto / dt)
    usados = 0
    for _ in range(pasos):
        x, y, psi, delta = d.q
        xm, ym = x + E.MID * math.cos(psi), y + E.MID * math.sin(psi)
        dcc, s = E._proyecta(ruta, (xm, ym))
        errs.append(dcc); pico = max(pico, abs(dcc)); s_max = max(s_max, s)
        xe = xm + rng.normal(0, 0.01); ye = ym + rng.normal(0, 0.01)
        psie = psi + rng.normal(0, math.radians(1.0))
        _, s_e = E._proyecta(ruta, (xe, ye))
        obj = E._punto_adelante(ruta, s_e, lookahead=0.4)
        e = E._wrap(math.atan2(obj[1] - ye, obj[0] - xe) - psie)
        v, om = ctrl.comando(e, dt, hueco=None, altura=None)
        if abs(v) < 1e-3:
            quieto += 1
        d.step(dt, v, om)
        usados += 1
        xm2 = d.q[0] + E.MID * math.cos(d.q[2])
        ym2 = d.q[1] + E.MID * math.sin(d.q[2])
        if H.choca(esc["mundo"], xm2, ym2, d.q[2], d.q[3], geo=geo):
            if not prev:
                colis += 1
            prev = True
        else:
            prev = False
        if s_max >= largo - E.FIN_RUTA_M:
            break
    return {"progreso": min(1.0, s_max / largo),
            "rmse_cm": float(np.sqrt(np.mean(np.square(errs)))) * 100,
            "pico_cm": pico * 100,
            "contactos": colis,
            "detenido_pct": 100.0 * quieto / max(1, usados),
            "t_s": usados * dt}


def correr(escs, semillas, cfg):
    filas = []
    for nombre, esc in escs.items():
        ms = [_episodio(esc, s, cfg) for s in semillas]
        filas.append({
            "escenario": nombre,
            "exito_pct": round(100.0 * np.mean([m["progreso"] > 0.95 for m in ms]), 0),
            "progreso": round(float(np.mean([m["progreso"] for m in ms])), 3),
            "rmse_cm": round(float(np.mean([m["rmse_cm"] for m in ms])), 1),
            "pico_cm": round(float(np.max([m["pico_cm"] for m in ms])), 1),
            "contactos": round(float(np.mean([m["contactos"] for m in ms])), 2),
            "episodios_con_contacto_pct": round(
                100.0 * np.mean([m["contactos"] > 0 for m in ms]), 0),
            "detenido_pct": round(float(np.mean([m["detenido_pct"] for m in ms])), 1),
        })
    return filas


def _tabla(filas):
    print(f"  {'escenario':>17} {'éxito':>7} {'RMSE':>7} {'PICO':>7} "
          f"{'contactos':>10} {'% con':>7} {'detenido':>9}")
    for f in filas:
        print(f"  {f['escenario']:>17} {f['exito_pct']:>6.0f}% "
              f"{f['rmse_cm']:>6.1f}c {f['pico_cm']:>6.1f}c "
              f"{f['contactos']:>10.2f} {f['episodios_con_contacto_pct']:>6.0f}% "
              f"{f['detenido_pct']:>8.1f}%")


def puntuar(f):
    """Menos = mejor. Para interiores manda NO TOCAR la maceta y COMPLETAR:
    un robot que no llega no sirve, y uno que llega rozando, menos."""
    return (8.0 * f["contactos"] + 6.0 * (1.0 - f["progreso"])
            + 3.0 * (f["pico_cm"] / 100.0))


#: Para interior la velocidad NO es libre: el perfil del backend
#: (`perfiles_entorno`) la fija en 0,195 m/s porque por encima el recorrido
#: tras soltar el mando se dispara (14 cm a 0,2 m/s; 27,5 a 0,3; 86,6 a 0,4) y
#: el margen de parada deja de cubrirlo. Así que aquí se ajustan kp/ki a
#: velocidad FIJA. Mezclar velocidades en la rejilla sesgaba el resultado: el
#: presupuesto de tiempo es proporcional a la longitud (5 s/m = 0,2 m/s
#: medios), de modo que los candidatos lentos no llegaban y perdían por
#: `progreso`. Eso medía quién es más rápido, no quién no toca las macetas.
V_INTERIOR_MPS = 0.195
#: A esta velocidad el robot avanza mucho menos de lo que se le pide (medido:
#: ~0,06 m/s efectivos con 0,2 pedidos), así que el presupuesto de los
#: escenarios de interior es más generoso que el de huerta.
SEGUNDOS_POR_METRO_INTERIOR = 20.0


def ajustar(semillas, ancho, rejilla=None, mu_interior=0.8):
    """Rejilla sobre los escenarios de INTERIOR.

    Las ganancias de huerta (kp4,0 ki2,5 v0,4) se afinaron a mu 0,35-0,65. Un
    suelo de interior pulido agarra mucho más (mu 0,8-0,95) y ahí el chasis
    desalineado NO puede deslizar la desviación: la clava. Medido el
    2026-10-14 en recta de 3 m: éxito 100 % a mu 0,5 y 0 % a mu 0,95, con el
    pico de desvío de 7,3 a 36,5 cm. Así que interior necesita su propio
    ajuste, no el de huerta.
    """
    rejilla = rejilla or [(kp, ki, V_INTERIOR_MPS)
                          for kp in (2.0, 3.0, 4.0, 6.0, 8.0)
                          for ki in (1.0, 2.5, 4.0, 6.0)]
    escs = {k: {**e, "mu": mu_interior}
            for k, e in escenarios(ancho).items() if k != "sala_vacia"}
    resultados = []
    for kp, ki, vv in rejilla:
        cfg = {"kp": kp, "ki": ki, "v": vv, "freno": True}
        filas = correr(escs, semillas, cfg)
        pts = float(np.mean([puntuar(f) for f in filas]))
        resultados.append((pts, cfg, filas))
        print(f"  kp{kp} ki{ki} v{vv}: {pts:.3f}", flush=True)
    resultados.sort(key=lambda t: t[0])
    return resultados


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--semillas", type=int, default=8)
    ap.add_argument("--ancho", type=float, default=0.70,
                    help="paso libre entre macetas (m)")
    ap.add_argument("--anchos", type=str, default=None,
                    help="barrido: 0.55,0.65,0.75")
    ap.add_argument("--kp", type=float, default=4.0)
    ap.add_argument("--ki", type=float, default=2.5)
    ap.add_argument("--v", type=float, default=0.4)
    ap.add_argument("--ajustar", action="store_true",
                    help="rejilla de ganancias sobre los escenarios de interior")
    a = ap.parse_args()
    semillas = list(range(a.semillas))
    cfg = {"kp": a.kp, "ki": a.ki, "v": a.v, "freno": True}

    print(f"INTERIORES · maceta r={MACETA_R*100:.0f} cm · {len(semillas)} semillas "
          f"· controlador {cfg}")
    print(f"vía trasera 37 cm · radio de giro usado {K.radio_seguro():.2f} m\n")

    salida = {"controlador": cfg, "semillas": semillas, "maceta_r_m": MACETA_R}

    if a.ajustar:
        print("Ajustando para INTERIOR (mu 0,8, escenarios con macetas)...\n",
              flush=True)
        res = ajustar(semillas, a.ancho)
        mejor_pts, mejor_cfg, mejor_filas = res[0]
        print(f"\nMEJOR para interior: {mejor_cfg}  ({mejor_pts:.3f} puntos)")
        print(f"peor de la rejilla: {res[-1][1]} ({res[-1][0]:.3f})\n")
        _tabla(mejor_filas)
        salida["ajuste_interior"] = {
            "mejor": mejor_cfg, "puntos": round(mejor_pts, 3),
            "rejilla": [{"cfg": c, "puntos": round(pt, 3)} for pt, c, _ in res]}
        SALIDA.parent.mkdir(parents=True, exist_ok=True)
        SALIDA.write_text(json.dumps(salida, indent=2, ensure_ascii=False))
        print(f"\nGuardado en {SALIDA}")
        return

    if a.anchos:
        anchos = [float(x) for x in a.anchos.split(",")]
        salida["barrido"] = {}
        for w in anchos:
            print(f"--- paso libre entre macetas: {w*100:.0f} cm "
                  f"(holgura {(w-0.37)/2*100:.1f} cm/lado) ---")
            escs = {k: v for k, v in escenarios(w).items() if "maceta" in k}
            filas = correr(escs, semillas, cfg)
            _tabla(filas); print()
            salida["barrido"][f"{w:.2f}"] = filas
    else:
        filas = correr(escenarios(a.ancho), semillas, cfg)
        _tabla(filas)
        salida["ancho_m"] = a.ancho
        salida["filas"] = filas

    SALIDA.parent.mkdir(parents=True, exist_ok=True)
    SALIDA.write_text(json.dumps(salida, indent=2, ensure_ascii=False))
    print(f"\nGuardado en {SALIDA}")


if __name__ == "__main__":
    main()
