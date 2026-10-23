"""Calibra el simulador con los PULSOS REALES medidos el 2026-10-23 (grabación de campo + poses del SLAM, comp_neg = 25):

  pulso       giro      avance
  adelante    +0,4°      +9,1 cm   (0,8 s, troceado 35 %)
  atras       +0,3°     -14,2 cm
  arco_der    -4,0°      +5,3 cm   (v 0,195 / omega -0,26, 0,4 s)
  arco_izq    +2,3°      +0,0 cm   (con comp_neg 25: antes de compensar retrocedía)

El simulador por defecto recorre ~2x más hacia delante y casi no gira en los arcos: aquí se buscan masa, rodadura, eficiencia de
avance/retroceso y diferencia de lados que lo reproducen. Salida: calibracion_pulsos_20261023.json. Uso: python3 calibrar_pulsos_reales.py [--n 400]"""
import argparse, json, math, sys, time
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path
import numpy as np
import sim_logica_actual as S
from robot_rustico import parametros_rusticos

OBJETIVO = {"adelante": (0.4, 9.1), "atras": (0.3, -14.2), "derecha": (-4.0, 5.3), "izquierda": (2.3, 0.0)}
ESCALA = (3.0, 4.0)     # tolerancia típica (grados, cm) para normalizar el error
S.SUBSTEP = 0.005


def pulso(accion, hp, comp_neg=25, freno=True):
    cfg = S.Config(nombre="cal", modo="interior", escala_velocidad=0.13, firmware="fusion", fu_adel=100, fu_atras=100, fu_pivote=100,
                   fu_minp=100, fu_acel=600, fu_frena=1000, fu_tras=150, fu_dif_del=100, fu_periodo=0, fu_refuerzo=100,
                   fu_comp_neg=comp_neg, fu_comp_pos=0, suavidad_duty=0.35, suavidad_periodo_s=0.2, pivote_en_sitio=True)
    act = S.ActuadorJetson(cfg)
    cond = S.Conductor(cfg, act)
    fw = S.crear_firmware(cfg)
    p = replace(parametros_rusticos(50), mu=hp["mu"], front_drive_bias=hp["bias"], rolling_drag=hp["drag"], substep=0.005,
                chassis_mass=hp["masa"], lateral=hp.get("lateral", 65.0), pivot_friction=hp.get("pfric", 0.02))
    r = S.RobotModelo(p, (0, 0, 0))
    lado = (hp["lado_der"], hp["lado_izq"], hp["lado_izq"], hp["lado_der"])
    r.efic = [(hp["efic_fwd"] * k, hp["efic_rev"] * k) for k in lado]
    x0, y0 = r.centro()
    ps0 = r.psi_real()
    t, cola, pc, pj = 0.0, [], 0.0, 0.0
    cond.ordenar(accion, 0.0, maniobra=False)
    while t < 2.6:
        if t >= pc:
            pc += S.TICK_CONDUCTOR
            cond.paso(t)
        if t >= pj:
            pj += S.TICK_JETSON
            act.tick(t)
            cola.append((t + S.LATENCIA_ENLACE_S, S._pct(act.out_v, S.V_SCALE), S._pct(act.out_w, S.W_SCALE)))
        while cola and cola[0][0] <= t + 1e-9:
            _, pv, pw = cola.pop(0)
            fw.recibir(pv, pw, t)
        r.step(S.TICK_FW, fw.tick(t), hp.get("dead", 0.0))
        t += S.TICK_FW
    x, y = r.centro()
    ps = r.psi_real()
    return (math.degrees(math.atan2(math.sin(ps - ps0), math.cos(ps - ps0))),
            ((x - x0) * math.cos(ps0) + (y - y0) * math.sin(ps0)) * 100)


def error(hp):
    e, res = 0.0, {}
    for acc, (g, d) in OBJETIVO.items():
        rg, rd = pulso(acc, hp)
        res[acc] = (round(rg, 1), round(rd, 1))
        e += ((rg - g) / ESCALA[0]) ** 2 + ((rd - d) / ESCALA[1]) ** 2
    return e, res


def _job(hp):
    try:
        return error(hp)[0], hp
    except Exception:
        return 1e9, hp


def muestrear(rng):
    return {"masa": float(rng.uniform(14, 24)), "bias": float(rng.uniform(1.2, 3.5)), "mu": float(rng.choice([0.65, 0.8, 0.95])),
            "drag": float(math.exp(rng.uniform(math.log(0.1), math.log(1.0)))), "efic_fwd": float(rng.uniform(0.3, 1.0)),
            "efic_rev": float(rng.uniform(0.4, 1.5)), "lado_der": float(rng.uniform(0.7, 1.3)), "lado_izq": float(rng.uniform(0.7, 1.3)),
            "dead": float(rng.choice([0.0, 25.0, 40.0])), "lateral": float(rng.uniform(8, 65)),
            "pfric": float(math.exp(rng.uniform(math.log(0.005), math.log(0.05))))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=400)
    ap.add_argument("--procs", type=int, default=5)
    ap.add_argument("--refinar", type=int, default=3)
    a = ap.parse_args()
    rng = np.random.default_rng(20261023)
    t0 = time.time()
    with Pool(a.procs) as pool:
        mejores = sorted(pool.map(_job, [muestrear(rng) for _ in range(a.n)]), key=lambda x: x[0])[:12]
        print(f"[1] {a.n} muestras en {time.time()-t0:.0f}s · mejor error {mejores[0][0]:.2f}", flush=True)
        for ronda in range(a.refinar):                               # refinamiento local alrededor de los mejores
            cand = []
            for _, hp in mejores[:6]:
                for _ in range(24):
                    h = dict(hp)
                    for k in ("masa", "bias", "drag", "efic_fwd", "efic_rev", "lado_der", "lado_izq", "lateral", "pfric"):
                        h[k] = float(max(0.05, h[k] * math.exp(rng.normal(0, 0.12))))
                    cand.append(h)
            mejores = sorted(pool.map(_job, cand) + mejores, key=lambda x: x[0])[:12]
            print(f"[{ronda+2}] refinado · mejor error {mejores[0][0]:.2f} · {time.time()-t0:.0f}s", flush=True)
    err, hp = mejores[0]
    _, res = error(hp)
    out = {"fecha": "2026-10-23", "objetivo": OBJETIVO, "error": round(err, 2), "parametros": hp, "simulado": res,
           "otros_buenos": [{"error": round(e, 2), **h} for e, h in mejores[1:5]]}
    Path(__file__).with_name("calibracion_pulsos_20261023.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
    print(json.dumps(out, indent=1, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
