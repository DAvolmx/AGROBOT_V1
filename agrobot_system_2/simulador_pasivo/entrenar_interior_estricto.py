"""Tanda LIGERA y ESTRICTA de interiores con el firmware skid-steer v3 flasheado hoy.

Uso:  python3 entrenar_interior_estricto.py [--n 24] [--procs 6]
Salida: ~/.agrobot/aprendizaje/interior_estricto_<fecha>.json + resumen por pantalla.
NO despliega nada: produce una recomendación que se acepta solo si pasa TODO esto.

PUERTA 0 (validez del simulador, antes de gastar CPU en entrenar):
  a) determinismo: el mismo episodio dos veces da lo mismo;
  b) convergencia numérica: substep 0,005 y 0,0025 coinciden (el defecto que invalidó el
     banco anterior era un substep demasiado grande);
  c) calibración: el pulso del firmware antiguo (87 %, 1 s) debe reproducir el único dato real,
     26,2 ± 2,2 cm (n=4), dentro de ±6 cm. Para el v3 NO hay dato real: se muestra la
     predicción y es lo primero que hay que medir en el robot.
  Si a), b) o c) fallan, no se entrena.

ACEPTACIÓN (todas, sobre TEST con semillas nunca usadas, 4 escenarios x 10 semillas):
  1. CERO contactos y 0 % de atascos;  2. éxito >= 95 %;
  3. no peor que la config actual en puntuación NI en el éxito de ningún escenario;
  4. ESTRÉS: con el peor hardware plausible (zona muerta 40 %, mu 0,65, rozamiento 0,10: la mitad del calibrado, ruedan más lejos) y
     hueco de 50 cm entre macetas: cero contactos;
  5. REPETIBLE: el resultado de TEST se reproduce al repetirlo con el substep a la mitad.
Si ningún candidato cumple, se dice que no y se deja la config actual.
Semillas: train 40-43 · val 44-49 · test 50-59 (las de las rondas anteriores no se reusan).
"""
import argparse
import json
import math
import random
import time
from dataclasses import asdict
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import sim_logica_actual as S

SALIDA = Path.home() / ".agrobot" / "aprendizaje"
ESCS = ["sala_vacia", "pasillo_macetas", "eslalon_macetas", "curva_pasillo"]
SUB = 0.0025       # converge aquí (0,005 tenía hasta 6 % de error en un pulso; 0,00125 y 0,0006 = igual)
TRAIN, VAL, TEST = list(range(40, 44)), list(range(44, 50)), list(range(50, 60))
# Solo lo que importa con el skid-steer v3 (no tiene rampa, minp ni reducción interior).
ESPACIO = {
    "modo": ["interior", "preciso", "exterior"],
    "escala_velocidad": [0.2, 0.3, 0.5],
    "suavidad_duty": [1.0, 0.7],
    "sendero_tol_cm": [5, 7, 10, 15],
    "avance_libre_cm": [40, 70, 100],
    "cono_frontal_deg": [10, 20, 30],
}


def _job(a):
    cfg_d, esc, seed, hueco, hip_over, substep = a
    S.SUBSTEP = substep
    cfg = S.Config(**cfg_d)
    hip = S.hip_por_semilla(seed)
    if hip_over:
        for k, v in hip_over.items():
            setattr(hip, k, v)
    r = S.episodio(S.escenarios_interior(hueco)[esc], cfg, seed, hip)
    r["puntos"] = round(S.puntuar_v2(r), 3)
    return r


def correr(pool, cfgs, seeds, escs=ESCS, hueco=0.60, hip_over=None, substep=SUB):
    jobs = [(asdict(c), e, s, hueco, hip_over, substep) for c in cfgs for e in escs for s in seeds]
    por = {}
    for r in pool.map(_job, jobs, chunksize=2):
        por.setdefault(r["config"], []).append(r)
    return por


def wilson_inf(k, n, z=1.96):
    if n == 0:
        return 0.0
    p = k / n
    d = 1 + z * z / n
    return (p + z * z / (2 * n) - z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / d


def resumen(rs):
    n = len(rs)
    ok = sum(bool(r["exito"]) for r in rs)
    return {"n": n, "puntos": round(float(np.mean([r["puntos"] for r in rs])), 3),
            "exito_pct": round(100.0 * ok / n, 1), "exito_wilson_inf_pct": round(100 * wilson_inf(ok, n), 1),
            "contactos_total": int(sum(r["contactos"] for r in rs)),
            "atascos": int(sum(bool(r["atascado"]) for r in rs)),
            "progreso": round(float(np.mean([r["progreso"] for r in rs])), 3),
            "t_s": round(float(np.mean([r["t_s"] for r in rs])), 1)}


def por_esc(rs):
    return {e: resumen([r for r in rs if r["escenario"] == e]) for e in ESCS if any(r["escenario"] == e for r in rs)}


def puerta0():
    """(a) determinismo, (b) convergencia, (c) plausibilidad. Devuelve (ok, detalle)."""
    cfg = S.config_actual()
    esc = S.escenarios_interior()["pasillo_macetas"]
    out = {}
    S.SUBSTEP = SUB
    r1, r2 = S.episodio(esc, cfg, 50), S.episodio(esc, cfg, 50)
    out["determinismo"] = all(r1[k] == r2[k] for k in ("contactos", "progreso", "t_s", "dist_m", "rmse_cm"))
    # (b) convergencia de la DINÁMICA en lazo abierto (un episodio completo con contactos es
    # caótico: una decisión distinta cambia todo, así que no mide la estabilidad numérica).
    from dataclasses import replace as _rp
    from robot_rustico import parametros_rusticos as _pr

    def lazo_abierto(substep, accion):
        cfgq = S.Config(nombre="lo", modo="exterior", escala_velocidad=0.4)
        act, fwq = S.ActuadorJetson(cfgq), S.crear_firmware(cfgq)
        cnd = S.Conductor(cfgq, act)
        hp = S.hip_por_semilla(50)
        hp.drag = 0.17
        pr = _rp(_pr(hp.semilla_chasis), mu=hp.mu, front_drive_bias=hp.bias, rolling_drag=hp.drag, substep=substep)
        rb = S.RobotModelo(pr, (0.0, 0.0, 0.0))
        x0, ps0 = rb.centro()[0], rb.psi_real()
        t, cola, pc, pj = 0.0, [], 0.0, 0.0
        cnd.ordenar(accion, 0.0)
        while t < 4.0:
            if t >= pc:
                pc += S.TICK_CONDUCTOR
                cnd.paso(t)
            if t >= pj:
                pj += S.TICK_JETSON
                act.tick(t)
                cola.append((t + S.LATENCIA_ENLACE_S, S._pct(act.out_v, S.V_SCALE), S._pct(act.out_w, S.W_SCALE)))
            while cola and cola[0][0] <= t + 1e-9:
                _, pv, pw = cola.pop(0)
                fwq.recibir(pv, pw, t)
            rb.step(S.TICK_FW, fwq.tick(t), hp.dead)
            t += S.TICK_FW
        return (rb.centro()[0] - x0) * 100, math.degrees(rb.psi_real() - ps0)
    conv = {}
    for acc in ("adelante", "izquierda"):
        a1, a2 = lazo_abierto(SUB, acc), lazo_abierto(SUB / 2, acc)
        conv[acc] = {"substep_base": [round(a1[0], 2), round(a1[1], 2)], "substep_mitad": [round(a2[0], 2), round(a2[1], 2)],
                     "ok": bool(abs(a1[0] - a2[0]) <= max(1.0, 0.05 * abs(a1[0]))
                                and abs(a1[1] - a2[1]) <= max(1.0, 0.05 * abs(a1[1])))}
    S.SUBSTEP = SUB
    out["convergencia"] = {**conv, "ok": all(v["ok"] for v in conv.values()),
                           "nota": "[cm, grados] de un pulso en lazo abierto"}
    # (c) CALIBRACIÓN contra el único dato real: pulso del firmware antiguo (~87 % de duty
    # continuo, 1 s) = 26,2 ± 2,2 cm. Se exige reproducirlo dentro de ±6 cm con 3 chasis.
    from dataclasses import replace
    from robot_rustico import parametros_rusticos

    def pulso_viejo(seed, mu, bias):
        p = replace(parametros_rusticos(seed), mu=mu, front_drive_bias=bias, rolling_drag=0.17, substep=SUB)
        r = S.RobotModelo(p, (0.0, 0.0, 0.0))
        x0, t = r.centro()[0], 0.0
        while t < 4.0:
            r.step(0.01, [87.0] * 4 if t < 1.0 else [0.0] * 4, 0.0)
            t += 0.01
        return (r.centro()[0] - x0) * 100
    viejos = [pulso_viejo(sd, mu, b) for sd, mu, b in ((50, .8, 1.0), (51, .65, 2.0), (52, .95, 1.0))]
    medio = float(np.mean(viejos))
    # PREDICCIÓN para el firmware v3 (no hay dato real: es lo primero que hay que medir)
    cfgp = S.Config(nombre="pulso", modo="exterior", escala_velocidad=0.4)
    act = S.ActuadorJetson(cfgp)
    cond = S.Conductor(cfgp, act)
    fw = S.crear_firmware(cfgp)
    hip = S.hip_por_semilla(50)
    hip.drag = 0.17
    params = replace(parametros_rusticos(hip.semilla_chasis), mu=hip.mu, front_drive_bias=hip.bias,
                     rolling_drag=hip.drag, substep=S.SUBSTEP)
    rob = S.RobotModelo(params, (0.0, 0.0, 0.0))
    x0 = rob.centro()[0]
    t, cola, prox_c, prox_j = 0.0, [], 0.0, 0.0
    cond.ordenar("adelante", 0.0)
    while t < 4.0:
        if t >= prox_c:
            prox_c += S.TICK_CONDUCTOR
            cond.paso(t)
        if t >= prox_j:
            prox_j += S.TICK_JETSON
            act.tick(t)
            cola.append((t + S.LATENCIA_ENLACE_S, S._pct(act.out_v, S.V_SCALE), S._pct(act.out_w, S.W_SCALE)))
        while cola and cola[0][0] <= t + 1e-9:
            _, pv, pw = cola.pop(0)
            fw.recibir(pv, pw, t)
        rob.step(S.TICK_FW, fw.tick(t), hip.dead)
        t += S.TICK_FW
    out["calibracion_pulso_viejo_cm"] = {"sim_por_chasis": [round(v, 1) for v in viejos], "media": round(medio, 1),
                                         "real": "26,2 ± 2,2 (n=4)", "ok_pm6": bool(abs(medio - 26.2) <= 6.0)}
    out["prediccion_pulso_v3_cm"] = {"sim": round((rob.centro()[0] - x0) * 100, 1),
                                     "nota": "SIN dato real: medir con el robot; si difiere >2x, esta tanda no vale"}
    ok = out["determinismo"] and out["convergencia"]["ok"] and out["calibracion_pulso_viejo_cm"]["ok_pm6"]
    return ok, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=24)
    ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--top", type=int, default=5)
    a = ap.parse_args()
    t0 = time.perf_counter()
    salida = {"fecha": time.strftime("%Y-%m-%d %H:%M"), "firmware_modelado": "skidsteer_v3",
              "semillas": {"train": TRAIN, "val": VAL, "test": TEST}}
    print("[0] puerta de validez del simulador…", flush=True)
    ok0, det = puerta0()
    salida["puerta0"] = det
    print(json.dumps(det, ensure_ascii=False), flush=True)
    SALIDA.mkdir(parents=True, exist_ok=True)
    fich = SALIDA / f"interior_estricto_{time.strftime('%Y%m%d_%H%M')}.json"
    if not ok0:
        salida["veredicto"] = "SIMULADOR NO VALIDO: no se entrena"
        fich.write_text(json.dumps(salida, indent=1, ensure_ascii=False))
        print("PUERTA 0 FALLIDA — no se entrena.", flush=True)
        return

    rng = random.Random(20261016)
    actual, perfil = S.config_actual(), S.config_interior_perfil()
    cands, vistos = [], set()
    while len(cands) < a.n:
        d = {k: rng.choice(v) for k, v in ESPACIO.items()}
        if tuple(d.values()) in vistos:
            continue
        vistos.add(tuple(d.values()))
        cands.append(S.Config(nombre=f"c{len(cands):02d}", **d))
    cfgn = {c.nombre: c for c in [actual, perfil] + cands}

    with Pool(a.procs) as pool:
        print(f"[1/4] TRAIN: {a.n} configs x {len(TRAIN)} semillas x {len(ESCS)} esc", flush=True)
        r1 = correr(pool, [actual, perfil] + cands, TRAIN)
        t1 = {n: resumen(rs) for n, rs in r1.items()}
        # se descartan de entrada los que tocan algo: con cero contactos exigido no hay que mirarlos
        orden = sorted((n for n in t1 if n not in ("actual", "perfil_interior")),
                       key=lambda n: (t1[n]["contactos_total"], t1[n]["puntos"]))
        print(f"      actual {t1['actual']['puntos']} ({t1['actual']['contactos_total']} cont) · perfil "
              f"{t1['perfil_interior']['puntos']} · mejor {orden[0]} {t1[orden[0]]['puntos']}", flush=True)
        salida["train"] = {n: {"cfg": asdict(cfgn[n]), **t1[n]} for n in t1}

        top = [cfgn[n] for n in orden[:a.top]]
        print(f"[2/4] VAL: top {len(top)} + referencias x {len(VAL)} semillas", flush=True)
        r2 = correr(pool, [actual, perfil] + top, VAL)
        t2 = {n: resumen(rs) for n, rs in r2.items()}
        ranking = sorted((n for n in t2 if n not in ("actual", "perfil_interior")),
                         key=lambda n: (t2[n]["contactos_total"], t2[n]["puntos"]))
        salida["val"] = {n: {"cfg": asdict(cfgn[n]), **t2[n]} for n in t2}
        elegido = cfgn[ranking[0]]
        elegido.nombre = "candidato"
        print(f"      elegido por VAL: {ranking[0]} {t2[ranking[0]]['puntos']} "
              f"({t2[ranking[0]]['contactos_total']} contactos)", flush=True)

        print(f"[3/4] TEST (semillas nuevas): candidato vs actual vs perfil x {len(TEST)}", flush=True)
        r3 = correr(pool, [actual, perfil, elegido], TEST)
        t3 = {n: resumen(rs) for n, rs in r3.items()}
        salida["test"] = {n: {"cfg": asdict({'actual': actual, 'perfil_interior': perfil,
                                             'candidato': elegido}[n]), **t3[n], "por_escenario": por_esc(r3[n])}
                          for n in t3}
        c, b = t3["candidato"], t3["actual"]
        print(f"      candidato {c} \n      actual    {b}", flush=True)

        print("[4/4] ESTRÉS (peor hardware, hueco 50 cm) y repetibilidad (substep/2)", flush=True)
        peor = {"dead": 40.0, "mu": 0.65, "drag": 0.10}   # poco rozamiento = rueda más lejos = peor para frenar
        rs_e = []
        for esc in ("pasillo_macetas", "eslalon_macetas"):
            rs_e += correr(pool, [elegido], TEST[:6], escs=[esc], hueco=0.50, hip_over=peor)["candidato"]
        rs_a = []
        for esc in ("pasillo_macetas", "eslalon_macetas"):
            rs_a += correr(pool, [actual], TEST[:6], escs=[esc], hueco=0.50, hip_over=peor)["actual"]
        est_c, est_a = resumen(rs_e), resumen(rs_a)
        rr = correr(pool, [elegido], TEST, substep=SUB / 2)["candidato"]
        rep = resumen(rr)
        salida["estres"] = {"candidato": est_c, "actual": est_a}
        salida["repetibilidad"] = rep

    esc_c, esc_b = por_esc(r3["candidato"]), por_esc(r3["actual"])
    criterios = {
        "1_cero_contactos_y_atascos": c["contactos_total"] == 0 and c["atascos"] == 0,
        "2_exito_ge_95": c["exito_pct"] >= 95.0,
        "3_no_peor_que_actual": (c["puntos"] <= b["puntos"] and
                                 all(esc_c[e]["exito_pct"] >= esc_b[e]["exito_pct"] for e in esc_c)),
        "4_estres_cero_contactos": est_c["contactos_total"] == 0,
        "5_repetible": rep["contactos_total"] == 0 and abs(rep["exito_pct"] - c["exito_pct"]) <= 5.0,
    }
    salida["criterios"] = criterios
    salida["aceptado"] = all(criterios.values())
    salida["elegido"] = asdict(elegido)
    salida["segundos"] = round(time.perf_counter() - t0)
    fich.write_text(json.dumps(salida, indent=1, ensure_ascii=False))
    print("\nCRITERIOS:", json.dumps(criterios, ensure_ascii=False), flush=True)
    print("VEREDICTO:", "ACEPTADO" if salida["aceptado"] else "NO ACEPTADO (se deja la config actual)", flush=True)
    print(f"Guardado en {fich} ({salida['segundos']} s)", flush=True)


if __name__ == "__main__":
    main()
