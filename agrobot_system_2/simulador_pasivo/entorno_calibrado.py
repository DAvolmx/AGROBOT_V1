"""Entornos de simulación CALIBRADOS con lo medido el 2026-10-23 (interior y exterior) para simular y entrenar.

· `hip(semilla)`: el robot PESADO (~12 kg), delanteras fuertes / traseras flojas, con los parámetros que reproducen los pulsos reales
  (calibracion_pulsos_20261023.json: avance 9 cm, retroceso -14 cm, arcos con su giro) más una variación del ±10 % por semilla.
· `escenarios(entorno)`: interior (sala, pasillo de macetas, eslalon, curva, sala en L, callejón de 1 m) y exterior (hileras de huerta,
  campo con rocas, camino curvo con plantas).
· `config_base(entorno)`: la Config del robot tal como está HOY (perfil interior o exterior de perfiles_entorno + firmware fusión).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

import sim_logica_actual as S

CAL = Path(__file__).with_name("calibracion_pulsos_20261023.json")


def _cal() -> dict:
    return json.loads(CAL.read_text())["parametros"]


def hip(semilla: int, extremo: bool = False) -> "S.Hip":
    c = _cal()
    rng = np.random.default_rng(91_000 + semilla)
    j = (lambda s: float(np.exp(rng.normal(0, s))))
    h = S.hip_por_semilla(semilla)
    h.masa = c["masa"] * (1.0 if extremo else j(0.10))
    h.bias = c["bias"] * (0.8 if extremo else j(0.10))
    h.mu = float(rng.choice([0.65, 0.8, 0.95])) if not extremo else 0.65
    h.drag = c["drag"] * (1.3 if extremo else j(0.10))
    h.dead = 0.0
    h.efic_fwd = c["efic_fwd"] * j(0.06)
    h.efic_rev = c["efic_rev"] * j(0.06)
    h.lado_der = c["lado_der"] * j(0.04)
    h.lado_izq = c["lado_izq"] * j(0.04)
    h.lateral = c["lateral"] * j(0.10)
    h.pivot_fric = c["pfric"] * j(0.15)
    return h


# ---------------------------------------------------------------- escenarios
R_PLANTA = 0.12


def _hileras(paso_filas, largo, sep=0.45, x_ini=0.8):
    """Dos hileras de plantas separadas `paso_filas` (hueco libre entre bordes)."""
    borde = paso_filas / 2 + R_PLANTA
    out, x = [], x_ini
    while x <= largo:
        out += [(x, borde, R_PLANTA), (x, -borde, R_PLANTA)]
        x += sep
    return out


def _rocas(rng, n, x0, x1, ancho):
    out = []
    while len(out) < n:
        x, y, r = rng.uniform(x0, x1), rng.uniform(-ancho / 2, ancho / 2), rng.uniform(0.12, 0.22)
        if all(math.hypot(x - a, y - b) > r + rr + 0.55 for a, b, rr in out) and abs(y) < ancho / 2 - r:
            out.append((x, y, r))
    return out


def escenarios(entorno: str, semilla: int = 0, extendido: bool = True) -> dict:
    E = S.Escenario
    if entorno == "interior":
        esc = dict(S.escenarios_interior(0.60))
        L = 2.6
        # (sala en L: pendiente — el robot simulado atraviesa la pared de esa geometría; la curva de 60° a la izquierda ya cubre el giro)
        esc["callejon_1m"] = E("callejon_1m", S._sala(L + 1.7, 3.0), _hileras(0.55, L, 0.6), goal_x=L, t_max=150.0,
                               descripcion="Callejón de plantas con 55 cm libres.")
        if extendido:
            import mapas_interiores as MI                     # laboratorios, laberintos, pasillos con esquinas, puertas, almacén
            esc.update(MI.mapas())
        return esc
    rng = np.random.default_rng(500 + semilla)
    return {
        "huerta_hileras": E("huerta_hileras", [], _hileras(0.90, 5.0, 0.55), goal_x=5.2, t_max=160.0,
                            descripcion="Dos hileras de plantas con 90 cm entre bordes, 5 m."),
        "campo_rocas": E("campo_rocas", [], _rocas(rng, 9, 1.2, 6.2, 4.0), goal_x=6.6, t_max=170.0,
                         descripcion="Campo abierto con 9 rocas dispersas, 6,6 m."),
        "camino_plantas": E("camino_plantas", [], [(x, 0.7 + 0.25 * math.sin(x / 1.4), R_PLANTA) for x in np.arange(0.8, 6.0, 0.5)]
                            + [(x, -0.7 + 0.25 * math.sin(x / 1.4), R_PLANTA) for x in np.arange(0.8, 6.0, 0.5)], goal_x=6.2,
                            t_max=170.0, descripcion="Camino de 1,4 m entre plantas con ondulación suave."),
    }


# ---------------------------------------------------------------- configuración del robot
def config_base(entorno: str, **extra) -> dict:
    """Campos de Config del robot HOY. interior = perfil interior; exterior = perfil de huerta."""
    base = dict(firmware="fusion", control="pulsos", pivote_en_sitio=True, invertir_giro=True,
                fu_adel=100, fu_atras=100, fu_pivote=100, fu_minp=100, fu_acel=600, fu_frena=1000, fu_tras=150, fu_dif_del=100,
                fu_periodo=0, fu_refuerzo=100, fu_freno_ms=80, fu_comp_neg=25, fu_comp_pos=0,
                suavidad_duty=0.35, suavidad_periodo_s=0.2)
    if entorno == "interior":
        base.update(modo="interior", escala_velocidad=0.13, avance_libre_cm=40,
                    sendero_pared_max_cm=120, cono_frontal_deg=45)
    else:
        base.update(modo="exterior", escala_velocidad=0.5, avance_libre_cm=100,
                    sendero_pared_max_cm=150, cono_frontal_deg=20)
    base.update(extra)
    return base


def correr(entorno: str, escenario: str, cfg_campos: dict, semilla: int, extremo: bool = False) -> dict:
    S.SUBSTEP = 0.005
    esc = escenarios(entorno, semilla)[escenario]
    cfg = S.Config(**cfg_campos)
    r = S.episodio(esc, cfg, semilla, hip(semilla, extremo))
    r["puntos"] = round(float(S.puntuar_v2(r)), 3)
    return r


if __name__ == "__main__":
    import time
    for ent in ("interior", "exterior"):
        for nom in escenarios(ent):
            t0 = time.perf_counter()
            r = correr(ent, nom, config_base(ent), 3)
            print(f"{ent:8s} {nom:16s} {time.perf_counter()-t0:5.1f}s  éxito={r['exito']} contactos={r['contactos']} progreso={r['progreso']:.2f} "
                  f"t={r['t_s']}s pulsos={r['pulsos']}", flush=True)
