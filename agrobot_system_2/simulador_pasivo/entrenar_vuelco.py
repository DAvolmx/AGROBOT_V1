"""ENTRENAMIENTO DEL DETECTOR DE VUELCO (2026-11-02). CEM sobre los parámetros de jetson/backend/detector_vuelco.py.

Objetivo: detectar TODOS los vuelcos simulados (bulto bajo una rueda, giro rápido, caída de morro) pronto, y NINGUNA falsa alarma ni en los casi-vuelcos,
taludes y baches simulados ni en la conducción real grabada (golpes de los pulsos). Separación entrenamiento / prueba:
  · reales: sesiones anteriores al 2026-10-27 para entrenar; del 27-oct en adelante (exteriores incluidos) solo para la prueba final;
  · simuladas: semillas distintas.
Se compara contra la REGLA ACTUAL de modo_seguro.py (inclinación instantánea >= 38°, o >= 22° con balanceo >= 90 °/s).

    nice python3 entrenar_vuelco.py                 # ~1,5 h en la Jetson (3 procesos); deja docs/VUELCO_DETECTOR_20261102/{resultado.json,INFORME.md,figuras}
    nice python3 entrenar_vuelco.py --rapido        # humo
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))

from detector_vuelco import DetectorVuelco, PARAMS_DEFECTO, VUELCO, PRECAUCION  # noqa: E402
import vuelco_datos as vd  # noqa: E402

SALIDA = AQUI.parent / "docs" / "VUELCO_DETECTOR_20261102"
CORTE_PRUEBA = "20261027"
MOV_RECIENTE_S = 3.0
VENTANA_ANTES_S, VENTANA_DESPUES_S = 1.0, 3.0      # una alarma cuenta como acierto entre t_tip - 1 s y t_tip + 3 s (lo que tarda en terminar de caer)

# rango de búsqueda (min, max, entero?)
ESPACIO = {
    "g_tol": (0.05, 0.40, False), "w_gate_dps": (20.0, 250.0, False), "ventana_s": (0.3, 2.0, False), "n_min": (2, 8, True),
    "lim_lateral_deg": (25.0, 60.0, False), "lim_frontal_deg": (25.0, 70.0, False), "persistencia_s": (0.1, 2.0, False),
}


# ----------------------------------------------------------------------------------------------- detectores
class ReglaActual:
    """La regla de modo_seguro.py hasta hoy (instantánea, sin filtro)."""
    def __init__(self, **_):
        self.estado = "normal"

    def paso(self, t, acc, gyro):
        ax, ay, az = acc
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        if not 0.6 <= mag <= 1.4:
            return self.estado
        inc = math.degrees(math.acos(max(-1.0, min(1.0, az / mag))))
        rate = abs(gyro[1])
        if inc >= 38.0 or (inc >= 22.0 and rate >= 90.0):
            self.estado = VUELCO                   # enclavado (como el modo seguro)
        elif self.estado != VUELCO:
            self.estado = PRECAUCION if inc >= 22.0 else "normal"
        return self.estado


def _correr(det, t, acc, gyro):
    """Instantes en que el detector ENTRA en vuelco y en precaución."""
    alarmas, precs = [], []
    previo = "normal"
    for i in range(len(t)):
        e = det.paso(float(t[i]), acc[i], gyro[i])
        if e == VUELCO and previo != VUELCO:
            alarmas.append(float(t[i]))
        if e == PRECAUCION and previo == "normal":
            precs.append(float(t[i]))
        previo = e
    return alarmas, precs


# ----------------------------------------------------------------------------------------------- evaluación
def eval_sim(fabrica, episodios) -> dict:
    pos = [e for e in episodios if e["t_tip"] is not None]
    neg = [e for e in episodios if e["t_tip"] is None]
    lat, fallos, fp_pos = [], 0, 0
    por_tipo: dict = {}
    for e in pos:
        al, _ = _correr(fabrica(), e["t"], e["acc"], e["gyro"])
        buenas = [a for a in al if e["t_tip"] - VENTANA_ANTES_S <= a <= e["t_tip"] + VENTANA_DESPUES_S]
        antes = [a for a in al if a < e["t_tip"] - VENTANA_ANTES_S]
        fp_pos += bool(antes)
        d = por_tipo.setdefault(e["tipo"], {"n": 0, "detectados": 0})
        d["n"] += 1
        if buenas and not antes:
            lat.append(buenas[0] - e["t_tip"])
            d["detectados"] += 1
        else:
            fallos += 1
    fp_neg = 0
    for e in neg:
        al, _ = _correr(fabrica(), e["t"], e["acc"], e["gyro"])
        d = por_tipo.setdefault(e["tipo"], {"n": 0, "falsas": 0})
        d["n"] += 1
        d["falsas"] = d.get("falsas", 0) + bool(al)
        fp_neg += bool(al)
    lat = np.array(lat) if lat else np.array([9.9])
    return {"positivos": len(pos), "negativos": len(neg), "detectados": len(pos) - fallos, "fallos": fallos,
            "falsas_en_negativos": fp_neg, "falsas_antes_del_vuelco": fp_pos,
            "lat_media_s": round(float(lat.mean()), 3), "lat_p90_s": round(float(np.percentile(lat, 90)), 3), "por_tipo": por_tipo}


def candidatos_reales(sesiones, margen_s=6.0, umbral_deg=20.0):
    """Trozos de las sesiones reales donde algún detector PODRÍA alarmar (alguna muestra con inclinación >= umbral), con margen para ventana y
    persistencia. Fuera de ellos ningún detector del espacio de búsqueda dispara (necesita gravedad inclinada >= 25°). Ahorra ~99 % del cómputo."""
    trozos, horas_mov, horas = [], 0.0, 0.0
    for nombre, a in sesiones:
        t = a[:, 0].astype(float)
        acc = a[:, 1:4].astype(float)
        gyr = a[:, 4:7].astype(float)
        mov = (np.abs(a[:, 7]) > 0.01) | (np.abs(a[:, 8]) > 0.01)
        horas += max(0.0, t[-1] - t[0]) / 3600
        dtm = np.diff(t, prepend=t[0])
        horas_mov += float(dtm[mov].clip(0, 1).sum()) / 3600
        mag = np.linalg.norm(acc, axis=1)
        inc = np.degrees(np.arccos(np.clip(acc[:, 2] / np.maximum(mag, 1e-6), -1, 1)))
        marcas = np.where(inc >= umbral_deg)[0]
        if not len(marcas):
            continue
        # movimiento reciente: último instante con orden de mover
        t_mov = np.where(mov, t, -1e9)
        t_mov = np.maximum.accumulate(t_mov)
        ini = None
        fin = None
        for k in marcas:
            a0 = t[k] - margen_s
            if ini is None:
                ini, fin = a0, t[k] + margen_s
            elif a0 <= fin:
                fin = t[k] + margen_s
            else:
                trozos.append(_trozo(nombre, t, acc, gyr, t_mov, ini, fin))
                ini, fin = a0, t[k] + margen_s
        trozos.append(_trozo(nombre, t, acc, gyr, t_mov, ini, fin))
    return trozos, horas_mov, horas


def _trozo(nombre, t, acc, gyr, t_mov, ini, fin):
    m = (t >= ini) & (t <= fin)
    return {"sesion": nombre, "t": t[m], "acc": acc[m], "gyro": gyr[m], "t_mov": t_mov[m]}


def eval_real(fabrica, trozos, horas_mov) -> dict:
    """Falsas alarmas reales: entradas en VUELCO con una orden de movimiento en los últimos MOV_RECIENTE_S (lo que pararía la autonomía)."""
    fp, fp_quieto, precs = 0, 0, 0
    detalle = []
    for tr in trozos:
        al, pr = _correr(fabrica(), tr["t"], tr["acc"], tr["gyro"])
        for a in al:
            k = int(np.searchsorted(tr["t"], a))
            k = min(k, len(tr["t"]) - 1)
            if a - tr["t_mov"][k] <= MOV_RECIENTE_S:
                fp += 1
                detalle.append({"sesion": tr["sesion"], "t": round(a, 1)})
            else:
                fp_quieto += 1
        for a in pr:
            k = min(int(np.searchsorted(tr["t"], a)), len(tr["t"]) - 1)
            precs += a - tr["t_mov"][k] <= MOV_RECIENTE_S
    return {"falsas_en_marcha": fp, "por_hora_de_marcha": round(fp / max(horas_mov, 1e-6), 3), "alarmas_quieto": fp_quieto,
            "precauciones_en_marcha": int(precs), "horas_marcha": round(horas_mov, 2), "detalle": detalle[:40]}


def coste(rs, rr) -> float:
    miss = rs["fallos"] / max(1, rs["positivos"])
    fpn = (rs["falsas_en_negativos"] + rs["falsas_antes_del_vuelco"]) / max(1, rs["negativos"])
    return 100 * miss + 40 * fpn + 25 * rr["por_hora_de_marcha"] + 4 * max(0.0, rs["lat_media_s"]) + 2 * max(0.0, rs["lat_p90_s"])


# ----------------------------------------------------------------------------------------------- CEM
def _muestrear(rng, mu, sd):
    p = {}
    for k, (lo, hi, entero) in ESPACIO.items():
        v = float(np.clip(rng.normal(mu[k], sd[k]), lo, hi))
        p[k] = int(round(v)) if entero else round(v, 3)
    return p


_CTX: dict = {}
PROCESOS = 3          # candidatos en paralelo (fork: los datos se comparten; ~250 MB de reales)


def _evaluar(c):
    fab = lambda: DetectorVuelco(**c)
    rs = eval_sim(fab, _CTX["sim"])
    rr = eval_real(fab, _CTX["trozos"], _CTX["horas_mov"])
    return (coste(rs, rr), c, rs, rr)


def cem(sim, trozos, horas_mov, iters, pop, elite, seed=0, log=print):
    rng = np.random.default_rng(seed)
    mu = {k: float(np.clip(PARAMS_DEFECTO[k], lo, hi)) for k, (lo, hi, _) in ESPACIO.items()}
    sd = {k: (hi - lo) / 4 for k, (lo, hi, _) in ESPACIO.items()}
    mejor, mejor_c, historia = None, 1e18, []
    for it in range(iters):
        cands = [dict(mu)] if it == 0 else []
        cands += [_muestrear(rng, mu, sd) for _ in range(pop - len(cands))]
        cands = [{k: (int(round(v)) if ESPACIO[k][2] else v) for k, v in c.items()} for c in cands]
        _CTX.update(sim=sim, trozos=trozos, horas_mov=horas_mov)
        if PROCESOS > 1:
            import multiprocessing as mp
            with mp.get_context("fork").Pool(PROCESOS) as pool:
                res = pool.map(_evaluar, cands)
        else:
            res = [_evaluar(c) for c in cands]
        res.sort(key=lambda x: x[0])
        el = res[:elite]
        for k in ESPACIO:
            vals = np.array([e[1][k] for e in el], float)
            mu[k] = float(vals.mean())
            sd[k] = max(float(vals.std()) * 1.1, (ESPACIO[k][1] - ESPACIO[k][0]) * 0.02)
        if res[0][0] < mejor_c:
            mejor_c, mejor = res[0][0], res[0]
        historia.append({"iter": it, "coste": round(res[0][0], 3), "media_elite": round(float(np.mean([e[0] for e in el])), 3)})
        log(f"[cem] iter {it + 1}/{iters} mejor {res[0][0]:.2f} (global {mejor_c:.2f}) det {res[0][2]['detectados']}/{res[0][2]['positivos']} "
            f"fp_sim {res[0][2]['falsas_en_negativos']} fp_real {res[0][3]['falsas_en_marcha']} lat {res[0][2]['lat_media_s']} s  {res[0][1]}")
    return mejor, historia


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rapido", action="store_true")
    ap.add_argument("--iters", type=int, default=10)
    ap.add_argument("--pop", type=int, default=18)
    ap.add_argument("--salida", default=None, help="carpeta de resultados (por defecto docs/VUELCO_DETECTOR_20261102)")
    a = ap.parse_args()
    global SALIDA
    if a.salida:
        SALIDA = Path(a.salida)
    t0 = time.time()
    SALIDA.mkdir(parents=True, exist_ok=True)
    log_f = open(SALIDA / "entrenamiento.log", "w")

    def log(msg):
        print(msg, flush=True)
        log_f.write(msg + "\n")
        log_f.flush()

    sesiones = vd.cargar_reales()
    ruido = vd.banco_ruido(sesiones)
    log(f"banco de ruido real: {len(ruido[0])} trozos de 6 s en marcha")
    tr_ses = [s for s in sesiones if s[0] < CORTE_PRUEBA]
    te_ses = [s for s in sesiones if s[0] >= CORTE_PRUEBA]
    n_sim = 80 if a.rapido else 360
    sim_tr = vd.lote(n_sim, 1, ruido)
    sim_te = vd.lote(n_sim, 2, ruido)
    tr_trozos, tr_hm, tr_h = candidatos_reales(tr_ses)
    te_trozos, te_hm, te_h = candidatos_reales(te_ses)
    log(f"reales: entrenamiento {len(tr_ses)} sesiones ({tr_h:.1f} h, {tr_hm:.1f} h en marcha, {len(tr_trozos)} trozos a revisar); "
        f"prueba {len(te_ses)} sesiones ({te_h:.1f} h, {te_hm:.1f} h en marcha, {len(te_trozos)} trozos)")
    log(f"simulados: {sum(e['t_tip'] is not None for e in sim_tr)} vuelcos + {sum(e['t_tip'] is None for e in sim_tr)} negativos por lote")

    iters, pop = (3, 8) if a.rapido else (a.iters, a.pop)
    mejor, historia = cem(sim_tr, tr_trozos, tr_hm, iters, pop, max(3, pop // 4), log=log)
    params = mejor[1]

    def informe(nombre, fab):
        return {"sim_entrenamiento": eval_sim(fab, sim_tr), "sim_prueba": eval_sim(fab, sim_te),
                "real_entrenamiento": eval_real(fab, tr_trozos, tr_hm), "real_prueba": eval_real(fab, te_trozos, te_hm)}

    res = {
        "fecha": time.strftime("%Y-%m-%d %H:%M"), "duracion_min": None,
        "datos": {"sesiones_entrenamiento": len(tr_ses), "sesiones_prueba": len(te_ses), "horas_marcha_entrenamiento": round(tr_hm, 2),
                  "horas_marcha_prueba": round(te_hm, 2), "simulados_por_lote": n_sim, "trozos_ruido": int(len(ruido[0]))},
        "params_entrenados": params, "params_defecto": PARAMS_DEFECTO, "historia": historia,
        "regla_actual": informe("regla", lambda: ReglaActual()),
        "detector_defecto": informe("defecto", lambda: DetectorVuelco()),           # lo desplegado antes de esta corrida
        "detector_entrenado": informe("entrenado", lambda: DetectorVuelco(**params)),
    }
    res["duracion_min"] = round((time.time() - t0) / 60, 1)
    (SALIDA / "resultado.json").write_text(json.dumps(res, indent=2, ensure_ascii=False))
    log(f"listo en {res['duracion_min']} min -> {SALIDA / 'resultado.json'}")
    for k in ("regla_actual", "detector_defecto", "detector_entrenado"):
        r = res[k]
        log(f"{k:20s} PRUEBA: detecta {r['sim_prueba']['detectados']}/{r['sim_prueba']['positivos']} (lat {r['sim_prueba']['lat_media_s']} s, "
            f"p90 {r['sim_prueba']['lat_p90_s']}), falsas sim {r['sim_prueba']['falsas_en_negativos']}/{r['sim_prueba']['negativos']}, "
            f"falsas reales en marcha {r['real_prueba']['falsas_en_marcha']} ({r['real_prueba']['por_hora_de_marcha']}/h)")


if __name__ == "__main__":
    main()
