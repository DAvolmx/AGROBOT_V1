"""Entrenamiento ROBUSTO del manejo (2026-10-20): busca la configuración ÓPTIMA con el simulador del croquis.

Qué lo hace robusto (frente a `entrenar_interior_estricto.py`):
  · Cada candidato se juzga en TRES anchos de pasillo/hueco (0,50 / 0,60 / 0,75 m), no en uno, y se rankea por su
    PEOR caso: un óptimo que solo funciona con 60 cm no sirve.
  · Cada episodio sortea hardware (zona muerta, rozamiento, fuerza trasera, rodadura), el ancho del módulo del pivote
    (19 / 24 / 28 cm: el croquis dudoso) y la perturbación inicial: el óptimo no puede depender de una unidad ideal.
  · Búsqueda en EMBUDO (successive halving): muchos candidatos con pocas semillas -> los mejores con más semillas ->
    los 3 finalistas con semillas NUNCA usadas (50-59), prueba de estrés (peor hardware, hueco de 50 cm) y repetición
    con el paso de integración a la mitad. El ganador no se elige con las semillas con que se entrenó.
  · Orden de prioridad: 1) cero contactos, 2) mejor éxito en el peor hueco, 3) menor puntuación.
  · Valida el simulador primero (puerta 0: determinismo, convergencia, calibración con el único dato real).

Salida: ~/.agrobot/aprendizaje/robusto_<control>_<fecha>.json y, al final,
~/.agrobot/aprendizaje/modelo_manejo_optimo_<fecha>.json con la mejor configuración entre pulsos y continuo.
NO despliega nada. Se puede parar con ~/.agrobot/aprendizaje/robusto.STOP (acaba la etapa en curso).

Uso: python3 entrenar_robusto.py [--control pulsos|continuo|ambos] [--procs 6] [--n 64] [--horas 4]
"""
import argparse
import json
import random
import time
from dataclasses import asdict
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import entrenar_interior_estricto as T
import sim_logica_actual as S

SALIDA = Path.home() / ".agrobot" / "aprendizaje"
STOP = SALIDA / "robusto.STOP"
HUECOS = (0.50, 0.60, 0.75)
# Espacio AMPLIO (2026-10-22): además del anterior, las maniobras nuevas del explorador, el control de rumbo fusionado (giroscopio + paredes del
# LIDAR) y los demás parámetros del conductor que antes estaban fijos. Con --espacio amplio.
AMPLIO = {
    "rumbo_zm_deg": [0.0, 3.0, 4.0, 6.0],
    "rumbo_pulso_escala": [1.0, 0.5, 0.3],
    "rumbo_aprende": [0.15, 0.3],
    "man_retroceso_cm": [0.0, 35.0, 50.0],
    "man_atras_pulsos": [1, 2],
    "man_giro_pulsos": [1, 2],
    "man_desacel_cm": [0.0, 80.0, 120.0],
    "man_desacel_factor": [0.5, 0.7],
    "man_histeresis_cm": [0.0, 6.0, 12.0],
    "man_pausa_s": [0.0, 0.4],
    "man_escape_s": [0.0, 10.0, 20.0],
    "man_corr_doble": [False, True],
    "suavidad_periodo_s": [0.2, 0.3, 0.5],
    "cono_lateral_deg": [45, 60, 75],
    "sendero_pared_max_cm": [100, 150, 200],
    "max_omega_rps": [2.0, 3.0, 4.0],
    "accel_ang": [4.0, 8.0, 12.0],
    "accel_lin": [1.5, 3.0, 5.0],
}
PEOR_HW = {"dead": 40.0, "mu": 0.65, "drag": 0.10}


def evaluar(pool, cfgs, seeds, huecos, escs=T.ESCS, **kw):
    """Corre todas las configs en todos los huecos. Devuelve (episodios por config, resumen por hueco por config)."""
    todo, por_h = {}, {}
    for h in huecos:
        r = T.correr(pool, cfgs, seeds, escs=escs, hueco=h, **kw)
        for n, rs in r.items():
            todo.setdefault(n, []).extend(rs)
            por_h.setdefault(n, {})[h] = T.resumen(rs)
    return todo, por_h


def clave(n, todo, por_h):
    """Menor es mejor: (-éxito en el PEOR hueco, -éxito medio, contactos, puntuación media). Éxito primero: un robot que no
    avanza no choca, y eso no es un buen manejo."""
    r = T.resumen(todo[n])
    return (-min(v["exito_pct"] for v in por_h[n].values()), -r["exito_pct"], r["contactos_total"], r["puntos"])


def ficha(n, todo, por_h):
    r = T.resumen(todo[n])
    return {**r, "peor_hueco_exito_pct": min(v["exito_pct"] for v in por_h[n].values()),
            "por_hueco": {f"{h:.2f}": {"exito_pct": v["exito_pct"], "contactos": v["contactos_total"], "puntos": v["puntos"]}
                          for h, v in por_h[n].items()},
            "por_escenario": T.por_esc(todo[n])}


def parar():
    return STOP.exists()


def un_control(control, a, t_ini, presupuesto_s):
    ts = time.strftime("%Y%m%d_%H%M")
    esp, base = (T.ESPACIO_CC, T.BASE_CC) if control == "continuo" else (T.ESPACIO, {"pivote_en_sitio": True})
    # Firmware «MAX FRENTE»: las delanteras van siempre al 100 % de PWM, así que los parámetros del firmware «suave»
    # (su_*) no existen. Lo que regula la velocidad media es el troceado de pulsos de la Jetson (suavidad_duty) y la
    # ayuda trasera (tras_factor, constante del firmware).
    esp = {k: v for k, v in esp.items() if not k.startswith(("su_", "fu_"))}
    esp["tras_factor"] = [0.0, 0.5, 1.0]
    if control == "pulsos":
        esp["suavidad_duty"] = [1.0, 0.7, 0.5, 0.35]
        if a.espacio == "amplio":
            esp.update(AMPLIO)
    rng = random.Random(20261020 + (1 if control == "continuo" else 0))
    refs = [S.config_actual(), S.config_interior_perfil()]
    if control == "pulsos" and a.espacio == "amplio":      # un candidato con el rumbo fusionado, para que la evolución parta de algo bueno
        sem = S.Config(nombre="semilla_rumbo", modo="preciso", escala_velocidad=0.3, rumbo_zm_deg=4.0, rumbo_pulso_escala=0.5)
    cands, vistos = ([sem] if (control == "pulsos" and a.espacio == "amplio") else []), set()
    while len(cands) < a.n:
        d = {k: rng.choice(v) for k, v in esp.items()}
        if tuple(d.values()) in vistos:
            continue
        vistos.add(tuple(d.values()))
        cands.append(S.Config(nombre=f"{control[:1]}{len(cands):02d}", **base, **d))
    cfgn = {c.nombre: c for c in refs + cands}
    out = {"control": control, "fecha": ts, "geometria": S.GEOMETRIA, "huecos": HUECOS, "n_candidatos": a.n,
           "semillas": {"filtro": T.TRAIN[:2], "validacion": list(T.VAL), "prueba": list(T.TEST)}}
    fich = SALIDA / f"robusto_{control}_{ts}.json"

    def guardar():
        fich.write_text(json.dumps(out, indent=1, ensure_ascii=False))

    def quedan():
        return (time.time() - t_ini) < presupuesto_s and not parar()

    with Pool(a.procs) as pool:
        # ---- 1. FILTRO: todos, pocas semillas ----
        t0 = time.time()
        prev = None
        prev = None
        for f in (a.reanudar or "").split(","):          # varios JSON separados por coma: se usa el de este control
            if f and Path(f).exists():
                d = json.loads(Path(f).read_text())
                if d.get("control") == control and "filtro" in d:
                    prev, a.reanudar_usado = d, f
                    break
        if False:
            pass
        if prev is not None:
            cfgn = {n: S.Config(**v["cfg"]) for n, v in prev["filtro"].items()}
            refs = [cfgn["actual"], cfgn["perfil_interior"]]
            kf = lambda n: (-prev["filtro"][n]["peor_hueco_exito_pct"], -prev["filtro"][n]["exito_pct"], prev["filtro"][n]["contactos_total"], prev["filtro"][n]["puntos"])
            rank = sorted((n for n in cfgn if n not in ("actual", "perfil_interior")), key=kf)
            out["filtro"] = prev["filtro"]
            out["filtro_reanudado_de"] = a.reanudar_usado
            top = rank[:2] if a.rapido else rank[:(a.top or max(6, a.n // 6))]
            print(f"[{control}] 1/3 FILTRO reanudado de {Path(a.reanudar_usado).name}: {len(cfgn)} configs · mejor {top[0]} {kf(top[0])} · "
                  f"actual {kf('actual')} · perfil {kf('perfil_interior')}", flush=True)
        else:
            print(f"[{control}] 1/3 FILTRO: {len(cfgn)} configs x {len(T.TRAIN[:a.sem_filtro])} semillas x {len(T.ESCS)} esc x {len(HUECOS)} huecos", flush=True)
            todo, por_h = evaluar(pool, list(cfgn.values()), T.TRAIN[:a.sem_filtro], HUECOS)
            rank = sorted((n for n in todo if n not in ("actual", "perfil_interior")), key=lambda n: clave(n, todo, por_h))
            out["filtro"] = {n: {"cfg": asdict(cfgn[n]), **ficha(n, todo, por_h)} for n in todo}
            top = rank[:2] if a.rapido else rank[:(a.top or max(6, a.n // 6))]
            print(f"      {int(time.time() - t0)} s · mejor {top[0]} {clave(top[0], todo, por_h)} · actual {clave('actual', todo, por_h)} · "
                  f"perfil {clave('perfil_interior', todo, por_h)}", flush=True)
        # ---- 1b. EVOLUCIÓN (modo agresivo, --evolucion R): hijos = mutaciones de los mejores, evaluados igual que el filtro ----
        if a.evolucion and prev is None:
            ref_n = ("actual", "perfil_interior")
            for ronda in range(a.evolucion):
                if not quedan():
                    break
                t0 = time.time()
                elite = sorted((n for n in todo if n not in ref_n), key=lambda n: clave(n, todo, por_h))[:max(4, a.top or 8)]
                hijos = []
                intentos = 0
                while len(hijos) < a.hijos and intentos < a.hijos * 30:
                    intentos += 1
                    padre = asdict(cfgn[rng.choice(elite)])
                    padre.pop("nombre", None)
                    for k in rng.sample(list(esp), k=min(len(esp), rng.choice((1, 1, 2, 3)))):
                        padre[k] = rng.choice(esp[k])
                    huella = tuple(padre.get(k) for k in esp)
                    if huella in vistos:
                        continue
                    vistos.add(huella)
                    hijos.append(S.Config(nombre=f"{control[:1]}e{ronda}_{len(hijos):02d}", **padre))
                if not hijos:
                    break
                cfgn.update({c.nombre: c for c in hijos})
                t2, h2 = evaluar(pool, hijos, T.TRAIN[:a.sem_filtro], HUECOS)
                todo.update(t2)
                por_h.update(h2)
                rank = sorted((n for n in todo if n not in ref_n), key=lambda n: clave(n, todo, por_h))
                top = rank[:a.top or max(6, a.n // 6)]
                print(f"[{control}] 1b EVOLUCIÓN {ronda + 1}/{a.evolucion}: {len(hijos)} hijos · {int(time.time() - t0)} s · mejor {rank[0]} "
                      f"{clave(rank[0], todo, por_h)}", flush=True)
                out["filtro"] = {n: {"cfg": asdict(cfgn[n]), **ficha(n, todo, por_h)} for n in todo}
                out["evolucion_rondas"] = ronda + 1
                guardar()
        guardar()
        if not quedan():
            out["veredicto"] = "interrumpido tras el filtro"
            guardar()
            return out, cfgn

        # ---- 2. VALIDACIÓN: los mejores, 6 semillas nuevas ----
        t0 = time.time()
        print(f"[{control}] 2/3 VALIDACIÓN: {len(top)} + refs x {len(T.VAL)} semillas x 4 esc x {len(HUECOS)} huecos", flush=True)
        todo2, por_h2 = evaluar(pool, [cfgn[n] for n in top] + refs, T.VAL, HUECOS)
        rank2 = sorted((n for n in todo2 if n not in ("actual", "perfil_interior")), key=lambda n: clave(n, todo2, por_h2))
        out["validacion"] = {n: {"cfg": asdict(cfgn[n]), **ficha(n, todo2, por_h2)} for n in todo2}
        fin = rank2[:2] if a.rapido else rank2[:a.finalistas]
        print(f"      {int(time.time() - t0)} s · finalistas {fin} · mejor {clave(fin[0], todo2, por_h2)}", flush=True)
        guardar()
        if not quedan():
            out["veredicto"] = "interrumpido tras la validación"
            out["mejor_provisional"] = asdict(cfgn[fin[0]])
            guardar()
            return out, cfgn

        # ---- 3. PRUEBA FINAL: semillas nuevas + estrés + repetibilidad ----
        t0 = time.time()
        print(f"[{control}] 3/3 PRUEBA: {fin} + refs x {len(T.TEST)} semillas x 4 esc x {len(HUECOS)} huecos + estrés + repetición", flush=True)
        cfin = [cfgn[n] for n in fin] + refs
        todo3, por_h3 = evaluar(pool, cfin, T.TEST, HUECOS)
        elegido = min(fin, key=lambda n: clave(n, todo3, por_h3))
        e_cfg = cfgn[elegido]
        est_c, est_a = [], []
        for esc in ("pasillo_macetas", "eslalon_macetas"):
            est_c += T.correr(pool, [e_cfg], T.TEST[:4], escs=[esc], hueco=0.50, hip_over=PEOR_HW)[elegido]
            est_a += T.correr(pool, [S.config_actual()], T.TEST[:4], escs=[esc], hueco=0.50, hip_over=PEOR_HW)["actual"]
        rep0 = T.resumen(evaluar(pool, [e_cfg], T.TEST, (0.60,))[0][elegido])
        rep = T.resumen(evaluar(pool, [e_cfg], T.TEST, (0.60,), substep=T.SUB / 2)[0][elegido])
        out["prueba"] = {n: {"cfg": asdict(cfgn[n]), **ficha(n, todo3, por_h3)} for n in todo3}
        out["estres"] = {"elegido": T.resumen(est_c), "actual": T.resumen(est_a)}
        out["repetibilidad_substep_mitad"] = rep
        c, b = ficha(elegido, todo3, por_h3), ficha("actual", todo3, por_h3)
        p = ficha("perfil_interior", todo3, por_h3)
        crit = {
            "1_cero_contactos": c["contactos_total"] == 0,
            "2_exito_peor_hueco_ge_90": c["peor_hueco_exito_pct"] >= 90.0,
            "3_mejor_que_actual_y_perfil": c["puntos"] <= min(b["puntos"], p["puntos"]) and c["exito_pct"] >= max(b["exito_pct"], p["exito_pct"]),
            "4_estres_cero_contactos": T.resumen(est_c)["contactos_total"] == 0,
            "5_repetible": rep["contactos_total"] == 0 and abs(rep["exito_pct"] - rep0["exito_pct"]) <= 8.0,
        }
        out["elegido"] = {"nombre": elegido, "cfg": asdict(e_cfg), "prueba": c, "vs_actual": b, "vs_perfil_interior": p}
        out["criterios"] = crit
        out["optimo_aceptado"] = all(crit.values())
        out["segundos"] = int(time.time() - t_ini)
        print(f"      {int(time.time() - t0)} s · ELEGIDO {elegido}: éxito {c['exito_pct']}% (peor hueco {c['peor_hueco_exito_pct']}%) "
              f"contactos {c['contactos_total']} puntos {c['puntos']} | actual {b['exito_pct']}% {b['contactos_total']} cont | "
              f"perfil {p['exito_pct']}% {p['contactos_total']} cont", flush=True)
        print("      criterios:", json.dumps(crit, ensure_ascii=False), flush=True)
        guardar()
    return out, cfgn


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--control", choices=("pulsos", "continuo", "ambos"), default="ambos")
    ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--n", type=int, default=64, help="candidatos por control")
    ap.add_argument("--horas", type=float, default=4.0, help="presupuesto total; las etapas que no quepan se omiten")
    ap.add_argument("--rapido", action="store_true", help="prueba de humo: 1 hueco, 2 semillas por etapa, 2 finalistas")
    ap.add_argument("--huecos", default="0.50,0.60,0.75", help="anchos de pasillo, separados por comas")
    ap.add_argument("--sem-filtro", type=int, default=2)
    ap.add_argument("--sem-val", type=int, default=6)
    ap.add_argument("--sem-test", type=int, default=10)
    ap.add_argument("--top", type=int, default=0, help="finalistas del filtro (0 = n/6, mínimo 6)")
    ap.add_argument("--finalistas", type=int, default=3)
    ap.add_argument("--espacio", choices=("base", "amplio"), default="base", help="amplio = añade maniobras, rumbo fusionado y más parámetros")
    ap.add_argument("--evolucion", type=int, default=0, help="rondas de mutación de los mejores tras el filtro (modo agresivo)")
    ap.add_argument("--hijos", type=int, default=40, help="hijos por ronda de evolución")
    ap.add_argument("--reanudar", default="", help="robusto_<control>_*.json con un FILTRO ya calculado: se salta esa etapa")
    a = ap.parse_args()
    global HUECOS
    HUECOS = tuple(float(x) for x in a.huecos.split(","))
    T.VAL, T.TEST = T.VAL[:a.sem_val], T.TEST[:a.sem_test]
    if a.rapido:
        HUECOS = (0.60,)
        T.VAL, T.TEST = T.VAL[:2], T.TEST[:2]
    SALIDA.mkdir(parents=True, exist_ok=True)
    STOP.unlink(missing_ok=True)
    t_ini = time.time()
    print(f"[0] puerta de validez del simulador (geometría {S.GEOMETRIA}: pivote {S.PIVOTE_EXT_M} m atrás, eje fijo "
          f"{S.FIJO_EXT_M} m delante, LIDAR a {S.LIDAR_DESDE_EJE_M} m del eje fijo y {S.ALTURA_LIDAR_M} m del suelo; "
          f"firmware {S.Config().firmware})…", flush=True)
    ok0, det = T.puerta0()
    print(json.dumps(det, ensure_ascii=False), flush=True)
    if not ok0:
        raise SystemExit("PUERTA 0 FALLIDA: el simulador no es válido; no se entrena.")
    controles = ("continuo", "pulsos") if a.control == "ambos" else (a.control,)
    if a.control == "ambos" and a.espacio == "amplio":      # el robot real conduce con pulsos: va primero y con la mayor parte del tiempo
        controles = ("pulsos", "continuo")
    resultados = {}
    for k, ctl in enumerate(controles):
        restante = a.horas * 3600 - (time.time() - t_ini)
        if restante < 600 or parar():
            print(f"[{ctl}] sin tiempo/parada: se omite", flush=True)
            continue
        # el presupuesto se reparte entre los controles que faltan
        parte = restante / (len(controles) - k)
        if a.control == "ambos" and a.espacio == "amplio" and k == 0:
            parte = restante * 0.65
        resultados[ctl] = un_control(ctl, a, time.time(), parte)[0]
    # ---- óptimo global entre controles ----
    ok = {c: r for c, r in resultados.items() if "elegido" in r}
    if ok:
        mejor = min(ok, key=lambda c: (-ok[c]["elegido"]["prueba"]["peor_hueco_exito_pct"], -ok[c]["elegido"]["prueba"]["exito_pct"],
                                       ok[c]["elegido"]["prueba"]["contactos_total"], ok[c]["elegido"]["prueba"]["puntos"]))
        e = ok[mejor]["elegido"]
        final = {"fecha": time.strftime("%Y-%m-%d %H:%M"), "control_optimo": mejor, "config": e["cfg"], "prueba": e["prueba"],
                 "vs_actual": e["vs_actual"], "vs_perfil_interior": e["vs_perfil_interior"],
                 "aceptado": ok[mejor]["optimo_aceptado"], "criterios": ok[mejor]["criterios"],
                 "comparacion_controles": {c: ok[c]["elegido"]["prueba"] for c in ok},
                 "geometria": S.GEOMETRIA, "segundos": int(time.time() - t_ini),
                 "NO_DEMOSTRADO": "Es simulación con física HIPÓTESIS (calibrated=false): validar en el robot antes de fiarse."}
        sal = SALIDA / f"modelo_manejo_optimo_{time.strftime('%Y%m%d_%H%M')}.json"
        sal.write_text(json.dumps(final, indent=1, ensure_ascii=False))
        print(f"\nÓPTIMO: control {mejor} · éxito {e['prueba']['exito_pct']}% · peor hueco {e['prueba']['peor_hueco_exito_pct']}% · "
              f"contactos {e['prueba']['contactos_total']} · ACEPTADO={final['aceptado']}\nGuardado en {sal}", flush=True)
    print(f"Total {int(time.time() - t_ini)} s", flush=True)


if __name__ == "__main__":
    main()
