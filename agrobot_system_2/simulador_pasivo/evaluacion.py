#!/usr/bin/env python3
"""Banco de evaluación por escenarios + ajuste (entrenamiento) del control.

Evalúa el control de manejo sobre la FÍSICA (dinamica_pasiva + chasis rústico),
de forma reproducible y honesta (directiva bloque 4):

  · Escenarios progresivos y de estrés: recta, curva, ese, paso estrecho,
    suelo con poca adherencia, obstáculo bajo.
  · El controlador usa POSE ESTIMADA (con ruido), nunca la verdad del sim.
  · Métricas por episodio: error de sendero, progreso, colisiones, contacto con
    cultivo, inestabilidad (vuelco por giro/obstáculo). La INMOVILIDAD puntúa
    MAL (penaliza el no avanzar), no bien.
  · Split por SEMILLA en train/val/test; se compara SIEMPRE contra una
    referencia bajo las mismas semillas.
  · Guarda métricas + config reproducibles en JSON.

No toca percepción/dataset/etiquetas (eso es de Codex) ni el robot real.

    python3 evaluacion.py            # evalúa referencia vs candidato (test)
    python3 evaluacion.py --ajustar   # busca ganancias en train, valida, reporta test
"""
import argparse
import json
import math
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from dinamica_pasiva import DinamicaPasiva
from robot_rustico import parametros_rusticos, ServoDiferencial
import fallos_3d as F
import huella_pasiva as huella
import cinematica_pasiva as K

SALIDA = Path.home() / ".agrobot" / "aprendizaje" / "evaluacion.json"
MID = 0.185          # del eje trasero (pose de la dinámica) al punto medio (huella)
#: Radio con margen de corrección sobre el límite del pivote pasivo (~1,51 m).
R_SEGURO = round(K.radio_seguro(), 2)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class _Mundo:
    def __init__(self, paredes=(), objetos=()):
        self.paredes = list(paredes)
        self.objetos = list(objetos)


def _arco(r, ang0, ang1, n=60, cx=0.0, cy=None):
    cy = r if cy is None else cy
    return [(cx + r * math.sin(a), cy - r * math.cos(a))
            for a in np.linspace(ang0, ang1, n)]


def escenarios():
    """Cada escenario: ruta objetivo, mundo (paredes/objetos), mu, obstáculo
    bajo (altura), y plantas que invaden (como objetos de 'cultivo')."""
    recta = [(x, 0.0) for x in np.linspace(0, 3.0, 20)]
    curva = _arco(1.2, 0, math.pi / 2, n=30)
    ese = _arco(1.0, 0, math.pi / 2, n=24) + _arco(1.0, math.pi / 2, 0, n=24, cx=2.0, cy=0.0)[1:]
    base = dict(mundo=_Mundo(), mu=0.65, h_obs=0.0, obs_pos=None)
    return {
        "recta": {**base, "ruta": recta},
        "curva": {**base, "ruta": curva},
        "ese": {**base, "ruta": ese, "mu": 0.6},
        "paso_estrecho": {**base, "ruta": recta,
                          "mundo": _Mundo(paredes=[(0, 0.25, 3, 0.25),
                                                   (0, -0.25, 3, -0.25)])},
        "suelo_flojo": {**base, "ruta": recta, "mu": 0.35},
        # Montículo bajo: TERRENO a cruzar (no objeto sólido). El peligro es el
        # VUELCO si se cruza rápido, no la colisión. Lo ve el sensor por `obs_pos`.
        "obstaculo_bajo": {**base, "ruta": recta, "mu": 0.6, "h_obs": 0.05,
                           "obs_pos": (1.5, 0.0)},
        # --- Curvas FACTIBLES (añadidas 2026-10-14) -----------------------
        # `curva` (r=1,2) y `ese` (r=1,0) piden 17,1° y 20,3° de pivote: la
        # primera pasa del límite útil (17°) y la segunda del TOPE MECÁNICO
        # (20°). El robot no las puede seguir, así que su progreso bajo mide
        # cinemática, no control, y su coste es un suelo irreducible: con un
        # tercio del objetivo clavado el ajuste no distingue candidatos (de
        # ahí las 4 h y 142 episodios sin aceptar ninguno). Estas dos usan
        # `radio_seguro()` = R_min_util × 1,25, que deja autoridad para
        # corregir en vez de ir con el pivote al tope.
        "curva_amplia": {**base, "ruta": _arco(R_SEGURO, 0, math.pi / 2, n=30)},
        "ese_amplia": {**base, "mu": 0.6,
                       "ruta": _arco(R_SEGURO, 0, math.pi / 2, n=24)
                       + _arco(R_SEGURO, math.pi / 2, 0, n=24,
                               cx=2 * R_SEGURO, cy=0.0)[1:]},
    }


#: Escenarios cuya ruta el eje pasivo NO puede seguir (ver `factibilidad`).
#: Se siguen midiendo y reportando, pero NO entran en el objetivo que se
#: optimiza: puntuarlos es puntuar la geometría del chasis, no el controlador.
def escenarios_factibles():
    """Los escenarios que el robot puede completar, con su veredicto."""
    out = {}
    for nombre, esc in escenarios().items():
        f = K.factibilidad(esc["ruta"])
        if f["factible"]:
            out[nombre] = esc
    return out


def _proyecta(ruta, p):
    """Distancia (cross-track) y arclength del punto p sobre la polilínea."""
    best_d, best_s, acc = 1e9, 0.0, 0.0
    for a, b in zip(ruta, ruta[1:]):
        a = np.array(a); b = np.array(b); ab = b - a
        L = np.hypot(*ab)
        if L < 1e-9:
            continue
        t = max(0.0, min(1.0, np.dot(np.array(p) - a, ab) / (L * L)))
        proj = a + t * ab
        d = float(np.hypot(*(np.array(p) - proj)))
        if d < best_d:
            best_d, best_s = d, acc + t * L
        acc += L
    return best_d, best_s


#: Presupuesto de tiempo por metro de ruta. Antes del 2026-10-14 `t_max` era
#: 10 s FIJO para todos los escenarios y `progreso = s_max/largo`: una ruta más
#: larga puntuaba peor aunque el robot fuese igual de bien, así que el banco
#: PENALIZABA rediseñar la ruta y los escenarios de distinta longitud no eran
#: comparables entre sí. Se detectó al sustituir `ese` (r=1,0, imposible para el
#: pivote) por `ese_amplia` (r=1,51, factible): la versión que el robot SÍ puede
#: seguir salía peor, solo por ser un 51 % más larga.
#: 5 s/m = 0,2 m/s medios, holgura sobre la velocidad nominal del contrato.
SEGUNDOS_POR_METRO = 5.0
T_MAX_MIN_S = 10.0
#: A cuánto del final se considera la ruta completada (m). Es del orden del
#: paso entre puntos de la ruta: exigir el último milímetro haría depender el
#: corte del remuestreo.
FIN_RUTA_M = 0.05


def presupuesto_s(ruta) -> float:
    """Tiempo que se le da a un episodio, proporcional a la longitud de la ruta."""
    largo = sum(math.dist(a, b) for a, b in zip(ruta, ruta[1:]))
    return max(T_MAX_MIN_S, largo * SEGUNDOS_POR_METRO)


def simular(esc, controlador, params, semilla, dt=0.08, t_max=None,
            ruido_pos=0.01, ruido_rumbo=math.radians(1.0)):
    """Un episodio. Devuelve métricas. El controlador ve pose ESTIMADA.

    substep=0.01: la dinámica de rueda es rígida; con pasos más gruesos el
    motor sobre-oscila y el robot casi no avanza (comprobado).

    t_max=None: presupuesto proporcional a la longitud (ver `presupuesto_s`).
    Pasar un número reproduce el comportamiento viejo de 10 s fijos."""
    ruta = esc["ruta"]
    rng = np.random.default_rng(10_000 + semilla)
    largo = sum(math.dist(a, b) for a, b in zip(ruta, ruta[1:]))
    if t_max is None:
        t_max = esc.get("t_max") or presupuesto_s(ruta)
    p = replace(params, mu=esc["mu"], substep=0.01)
    geo = huella.geo_de_parametros(p)           # huella con la vía REAL del chasis
    psi0 = math.atan2(ruta[1][1] - ruta[0][1], ruta[1][0] - ruta[0][0])
    d = DinamicaPasiva(pose=(ruta[0][0] - MID * math.cos(psi0),
                             ruta[0][1] - MID * math.sin(psi0), psi0), parametros=p)
    controlador.reset()
    errs, colis, contacto, inestable, s_max = [], 0, 0, False, 0.0
    choque_prev = False
    est = F.Estabilidad(p)
    pasos = int(t_max / dt)
    t_usado = 0.0
    for _ in range(pasos):
        x, y, psi, delta = d.q
        xm, ym = x + MID * math.cos(psi), y + MID * math.sin(psi)       # punto medio (verdad)
        dcc, s = _proyecta(ruta, (xm, ym))
        errs.append(dcc); s_max = max(s_max, s)
        # --- entrada del controlador: pose ESTIMADA (ruido), no la verdad ---
        xe = xm + rng.normal(0, ruido_pos); ye = ym + rng.normal(0, ruido_pos)
        psie = psi + rng.normal(0, ruido_rumbo)
        _, s_e = _proyecta(ruta, (xe, ye))
        objetivo = _punto_adelante(ruta, s_e, lookahead=0.4)
        e = _wrap(math.atan2(objetivo[1] - ye, objetivo[0] - xe) - psie)
        hueco = _hueco(esc, xm, ym, psi)
        altura = _altura_estim(esc, hueco, rng)      # sensor con ruido, no verdad
        v, omega = controlador.comando(e, dt, hueco=hueco, altura=altura)
        # --- inestabilidad (verdad del sim, por la velocidad REAL del robot,
        #     no la comandada: la física vuelca según lo que de verdad va) ---
        v_real = d.dq[0] * math.cos(psi) + d.dq[1] * math.sin(psi)
        if F.vuelca_giro_rapido(v_real, d.dq[2], est=est):
            inestable = True
        if esc["h_obs"] > 0 and _cerca_obstaculo(esc, xm, ym) and \
                F.vuelca_obstaculo_energia(esc["h_obs"], abs(v_real), est=est):
            inestable = True
        d.step(dt, v, omega)
        # colisión con paredes/objetos sólidos
        xm2 = d.q[0] + MID * math.cos(d.q[2]); ym2 = d.q[1] + MID * math.sin(d.q[2])
        choque = huella.choca(esc["mundo"], xm2, ym2, d.q[2], d.q[3], geo=geo)
        if choque and not choque_prev:
            if esc["h_obs"] > 0:
                contacto += 1              # contacto con planta/cultivo
            else:
                colis += 1
        choque_prev = choque
        t_usado += dt
        # Terminar al completar la ruta. Sin esto el episodio seguía corriendo
        # con la ruta ya hecha: `_proyecta` se satura en el último tramo y el
        # error crece mientras el robot se aleja, así que el RMSE medía
        # vagabundeo posterior, no seguimiento. Dejaba el RMSE —y la nota— a
        # merced del presupuesto de tiempo (detectado el 2026-10-14 al hacerlo
        # proporcional a la longitud: `recta` pasó de 0,18 a 0,66 sin que el
        # control cambiase en nada).
        if s_max >= largo - FIN_RUTA_M:
            break
    progreso = min(1.0, s_max / largo) if largo > 1e-6 else 0.0
    return {"rmse_cm": round(float(np.sqrt(np.mean(np.square(errs)))) * 100, 1),
            "progreso": round(progreso, 3), "largo_m": round(largo, 2),
            "t_max_s": round(t_max, 1), "t_usado_s": round(t_usado, 1),
            "colisiones": colis,
            "contacto_cultivo": contacto, "inestable": inestable}


def _punto_adelante(ruta, s0, lookahead=0.4):
    acc = 0.0
    for a, b in zip(ruta, ruta[1:]):
        L = math.dist(a, b)
        if acc + L >= s0 + lookahead:
            return b
        acc += L
    return ruta[-1]


def _hueco(esc, xm, ym, psi):
    """Distancia al montículo en el camino (sensor del robot), o None. frente
    puede ser algo negativo (robot sobre el montículo) para no soltar el freno
    justo al cruzarlo."""
    pos = esc.get("obs_pos")
    if pos is None:
        return None
    dx, dy = pos[0] - xm, pos[1] - ym
    frente = dx * math.cos(psi) + dy * math.sin(psi)
    lateral = abs(-dx * math.sin(psi) + dy * math.cos(psi))
    if frente > -0.1 and lateral < 0.2 and frente < 1.5:
        return max(0.0, frente)
    return None


def _altura_estim(esc, hueco, rng):
    """Altura ESTIMADA del montículo (sensor con ruido, como el estimador de
    altura del backend/percepción). None si no hay obstáculo a la vista. NO es
    la verdad del sim: lleva ruido y sesgo."""
    if hueco is None or esc["h_obs"] <= 0:
        return None
    return max(0.0, esc["h_obs"] + rng.normal(0.0, 0.012))     # ±1.2 cm de ruido


def _cerca_obstaculo(esc, xm, ym):
    pos = esc.get("obs_pos")
    return pos is not None and math.dist((xm, ym), pos) < 0.2


# ------------------------------------------------------------- controladores
def _v_segura_creencia(altura_m):
    """Envolvente de vuelco GENÉRICA que cree el controlador si no se le da una
    específica de su chasis (tabla conservadora)."""
    h = altura_m * 100.0
    if h >= 7.0:
        return 0.0
    return max(0.0, 0.32 * (1.0 - h / 7.0))       # 0cm->0.32, 5cm->~0.09, 7cm->0


def envolvente_chasis(params, margen=0.6):
    """Envolvente de velocidad segura CALIBRADA para un chasis concreto (lo que
    en el robot real sería una tabla medida por chasis). Deriva de la física de
    vuelco con un margen de seguridad. Un chasis de CG bajo da velocidades
    seguras MÁS altas -> cruza los montículos más rápido."""
    est = F.Estabilidad(params)

    def v_segura(altura_m):
        vc = F.v_critica_obstaculo_rapida(altura_m, est=est, v_max=1.6)
        if vc is None:
            return 1.6                 # no vuelca a ninguna velocidad útil
        if vc <= 0.05:
            return 0.0                 # vuelca casi parado: impasable
        return max(0.0, margen * vc)
    return v_segura


class Controlador:
    """Seguidor de sendero: rumbo con ServoDiferencial (PI que aprende la deriva
    del chasis rústico) + decisión frenar-a-seguro / PARAR ante un montículo.

    `parar`=True habilita parar/cruzar-despacio según la ALTURA estimada
    (sensor, no verdad del sim). `freno` sin `parar` = solo freno cauto por rango."""

    def __init__(self, kp=3.5, ki=2.0, v=0.5, freno=True, parar=True, envolvente=None):
        self.servo = ServoDiferencial(kp=kp, ki=ki, v=v)
        self.v = v
        self.freno = freno
        self.parar = parar
        # Envolvente de seguridad del chasis (si None, usa la genérica).
        self.envolvente = envolvente or _v_segura_creencia
        self._latch_v = 0.0
        self._latch_t = 0

    def reset(self):
        self.servo.reset()
        self._latch_v, self._latch_t = 0.0, 0

    def comando(self, error_rumbo, dt, hueco=None, altura=None):
        _, omega = self.servo.paso(error_rumbo, dt)
        v = self.v / (1 + 1.5 * abs(error_rumbo))            # frena en los giros
        if self.freno and hueco is not None and hueco < 1.0:
            if self.parar and altura is not None:
                vseg = self.envolvente(altura)
                if vseg <= 0.1:
                    # Impasable: creer que volcaría -> PARAR antes (mejor quieto).
                    v = 0.0 if hueco < 0.55 else min(v, 0.18)
                    self._latch_v, self._latch_t = 0.0, 45
                elif hueco < 0.5:
                    v = min(v, vseg)                 # transitable: cruzar a velocidad segura
                    self._latch_v, self._latch_t = vseg, 45
                else:
                    v = min(v, vseg + (self.v - vseg) * (hueco - 0.5) / 0.5)
            else:
                v = min(v, max(0.12, 0.45 * (hueco / 1.0)))   # freno cauto por rango
        elif self._latch_t > 0:
            # Mantiene la velocidad segura mientras CRUZA el montículo, aunque el
            # sensor ya no lo vea (si no, rebotaría la velocidad y volcaría).
            v = min(v, self._latch_v) if self._latch_v > 0.01 else 0.0
        if self._latch_t > 0:
            self._latch_t -= 1
        return v, omega


def puntuar(m):
    """Menos = mejor. Penaliza colisiones, cultivo, salirse e inestabilidad; la
    inmovilidad (progreso bajo) puntúa MAL para no premiar quedarse quieto."""
    return (8 * m["colisiones"] + 6 * m["contacto_cultivo"]
            + 10 * (1 if m["inestable"] else 0)
            + 3 * (m["rmse_cm"] / 100.0) + 6 * (1 - m["progreso"]))


# Chasis del Monte Carlo de diseño (montecarlo_diseno.py): CG bajo + frente algo
# más ancho + vía trasera mayor. OJO: la trasera de 43 cm roza en el sendero de
# 50 cm (lo revela la huella paramétrica).
CHASIS_MEJORADO = dict(front_outer=0.21, rear_outer=0.43, wheelbase=0.38,
                       cg_x=0.19, cg_height=0.12)
# Alternativa: SOLO bajar el CG, manteniendo el ancho actual (cabe en el sendero).
CHASIS_CG_BAJO = dict(cg_height=0.12)


def evaluar(hacer_ctrl, semillas, chasis_over=None):
    escs = escenarios()
    # Envolvente CALIBRADA para este chasis (la geometría/masa no dependen de la
    # semilla, así que se calcula una vez).
    env = envolvente_chasis(parametros_rusticos(0, **(chasis_over or {})))
    # Veredicto cinemático por escenario: no depende de la semilla.
    veredictos = {n: K.factibilidad(e["ruta"]) for n, e in escs.items()}
    filas, total = [], 0.0
    fact_total, fact_n = 0.0, 0
    for s in semillas:
        pr = parametros_rusticos(s, **(chasis_over or {}))
        for nombre, esc in escs.items():
            ctrl = hacer_ctrl()
            ctrl.envolvente = env               # el controlador conoce su chasis
            m = simular(esc, ctrl, pr, s)
            pts = puntuar(m)
            total += pts
            f = veredictos[nombre]
            # El objetivo que se OPTIMIZA excluye las rutas que el eje pasivo
            # no puede seguir: su coste es un suelo que ninguna ganancia baja,
            # y mezclarlo tapa las diferencias entre candidatos.
            if f["factible"]:
                fact_total += pts
                fact_n += 1
            filas.append({"semilla": s, "escenario": nombre, "puntos": round(pts, 2),
                          "veredicto": f["veredicto"],
                          "pivote_necesario_deg": f["pivote_necesario_deg"], **m})
    return {"puntos_medios": round(total / len(filas), 3),
            "puntos_factibles": round(fact_total / fact_n, 3) if fact_n else None,
            "veredictos": veredictos,
            "episodios": filas}


SPLIT = {"train": range(0, 12), "val": range(12, 18), "test": range(18, 24)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ajustar", action="store_true")
    ap.add_argument("--comparar-chasis", action="store_true")
    a = ap.parse_args()
    t0 = time.perf_counter()

    referencia = lambda: Controlador(kp=3.0, ki=0.0, v=0.5, freno=False)  # naive
    # Aceptado el 2026-10-14 por `--ajustar` sobre el objetivo ya SANO
    # (presupuesto proporcional + corte al acabar la ruta + sin las rutas que
    # el pivote no puede seguir): train 1,802 / val 2,056 / test 1,754, contra
    # 2,345 de las ganancias anteriores (kp3,5 ki2,0 v0,5) y 3,672 del naive.
    # Misma rejilla de búsqueda que ya había: la mejora viene de arreglar el
    # banco, no de buscar más. Baja el RMSE lateral de 6,0 a 3,6 cm y con ello
    # la holgura que hace falta en pasillo (19 -> 14 cm/lado).
    mejor = {"kp": 4.0, "ki": 2.5, "v": 0.4, "freno": True}

    if a.comparar_chasis:
        print("Mismo controlador, tres chasis...\n", flush=True)
        seeds = list(SPLIT["test"])
        variantes = [("actual (CG20, ancho actual)", None),
                     ("MC: CG12 + trasera 43", CHASIS_MEJORADO),
                     ("solo CG12 (ancho actual)", CHASIS_CG_BAJO)]
        res = [(nom, evaluar(lambda: Controlador(**mejor), seeds, ov)) for nom, ov in variantes]
        for nom, r in res:
            print(f"  {nom:<30} {r['puntos_medios']:.3f} puntos")
        print(f"\n  {'escenario':>16}" + "".join(f"{nom.split()[0]:>12}" for nom, _ in variantes))
        for nombre in escenarios():
            vals = [np.mean([e["puntos"] for e in r["episodios"] if e["escenario"] == nombre])
                    for _, r in res]
            print(f"  {nombre:>16}" + "".join(f"{v:>12.2f}" for v in vals))
        print(f"\n({time.perf_counter()-t0:.0f}s)")
        return

    if a.ajustar:
        print("Ajustando en TRAIN (busca ganancias), valida en VAL...\n", flush=True)
        cand, mejor_val = None, 1e9
        semillas_busqueda = list(SPLIT["train"])[:6]      # subconjunto para el barrido
        for kp in (2.5, 4.0):
            for ki in (1.0, 2.5):
                for v in (0.4, 0.55):
                    cfg = {"kp": kp, "ki": ki, "v": v, "freno": True}
                    tr = evaluar(lambda c=cfg: Controlador(**c), semillas_busqueda)
                    obj = tr["puntos_factibles"]
                    if obj < mejor_val:
                        mejor_val, mejor = obj, cfg
                    print(f"  probado {cfg}: {obj} (factibles) / "
                          f"{tr['puntos_medios']} (todos)", flush=True)
        val = evaluar(lambda: Controlador(**mejor), list(SPLIT["val"]))
        print(f"mejor en train: {mejor}  (val {val['puntos_factibles']} factibles / "
              f"{val['puntos_medios']} todos)", flush=True)

    # Reporte en TEST: candidato vs referencia, mismas semillas.
    cand = evaluar(lambda: Controlador(**mejor), list(SPLIT["test"]))
    ref = evaluar(referencia, list(SPLIT["test"]))
    print(f"\n=== TEST (mismas {len(list(SPLIT['test']))} semillas) ===")
    print(f"  REFERENCIA (naive): {ref['puntos_medios']} puntos")
    print(f"  CANDIDATO (ajustado+freno): {cand['puntos_medios']} puntos "
          f"({'mejora' if cand['puntos_medios'] < ref['puntos_medios'] else 'EMPEORA'})")
    print(f"  Solo rutas FACTIBLES -> ref {ref['puntos_factibles']} / "
          f"cand {cand['puntos_factibles']}")
    # desglose por escenario, con el veredicto cinemático
    print(f"\n  {'escenario':>16} {'ref':>7} {'cand':>7}  {'pivote':>7}  veredicto")
    for nombre in escenarios():
        rr = np.mean([e["puntos"] for e in ref["episodios"] if e["escenario"] == nombre])
        cc = np.mean([e["puntos"] for e in cand["episodios"] if e["escenario"] == nombre])
        f = cand["veredictos"][nombre]
        marca = "" if f["veredicto"] == "ok" else ("  <-- AL LIMITE" if f["veredicto"] == "al_limite"
                                                   else "  <-- IMPOSIBLE, no se optimiza")
        print(f"  {nombre:>16} {rr:>7.2f} {cc:>7.2f}  "
              f"{f['pivote_necesario_deg']:>6.1f}°  {f['veredicto']}{marca}")

    SALIDA.parent.mkdir(parents=True, exist_ok=True)
    SALIDA.write_text(json.dumps({"config": mejor, "test_candidato": cand,
                                  "test_referencia": ref,
                                  "segundos": round(time.perf_counter() - t0, 1)},
                                 indent=2, ensure_ascii=False))
    print(f"\nMétricas guardadas en {SALIDA}  ({time.perf_counter()-t0:.0f}s)")


if __name__ == "__main__":
    main()
