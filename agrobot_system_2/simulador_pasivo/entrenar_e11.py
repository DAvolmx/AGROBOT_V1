"""Entrena la estrategia E11 (planificador A* + visitar plantas y parar a ~0,8 m + repetir escaneo fallido, con predictor de colisión) para ganar
RAPIDEZ y PRECISIÓN (2026-10-26). Reutiliza el CEM por rondas de entrenar_misiones.py, con:
  · espacio de búsqueda propio: planificador (inflado, mirada, replanificación, giro), predictor (márgenes, relajación), escaneo (duración, distancia de
    visita, reintentos), tolerancias del guardián y, además, la velocidad y el troceo de pulsos y el uso del hueco como respaldo reactivo;
  · costo con más peso al TIEMPO (0,02/s en vez de 0,008) y a la cobertura del escaneo (20): premia terminar antes y escanear todo;
  · éxito = vuelve al inicio + 0 choques + >= 90 % de las plantas vistas escaneadas + recall >= 0,8.
Referencias de la validación final (nombres heredados de entrenar_misiones): «HOY_con_guardian_y_maniobras» = E11 con los parámetros de ANTES del entrenamiento;
«HOY_con_guardian» = E6 (A* sin visitar plantas); «ROBOT_DE_HOY» = reactiva clásica sin guardián.
Uso: PYTHONPATH=~/.agrobot/numba_pkg python3 entrenar_e11.py [--rondas 4 --ronda-min 55 --pob 12 ...]   (acepta los argumentos de entrenar_misiones.py)"""
import json
import sys
from pathlib import Path

import entrenar_intensivo as EI
import entrenar_misiones as EM
import misiones_sim as MS

NUEVOS = {
    "mx_plan_inflar_m": ("u", 0.18, 0.30), "mx_plan_mirada_m": ("u", 0.5, 1.4), "mx_plan_cada_s": ("u", 1.0, 4.0), "mx_plan_giro_deg": ("u", 8.0, 25.0),
    "mx_plan_tol_planta_m": ("u", 0.60, 1.00), "mx_pred_relajar_s": ("u", 10.0, 60.0), "mx_pred_giro_extra_m": ("u", 0.02, 0.08),
    "mis_dist_visita_m": ("u", 0.60, 1.10), "mis_dist_insp_m": ("u", 0.30, 1.00), "mis_reintentos": ("u", 0, 2), "usar_hueco": ("u", 0, 1),
    # sonar sobre el mapa acumulado (2026-10-26)
    "usar_sonar": ("u", 0, 1), "mx_sonar_prof_m": ("u", 0.7, 1.5), "mx_sonar_w_meta": ("u", 0.3, 2.0), "mx_sonar_w_visitas": ("u", 0.0, 1.5),
}
E11 = {"mx_plan_inflar_m": 0.22, "mx_plan_mirada_m": 1.0, "mx_plan_cada_s": 2.0, "mx_plan_giro_deg": 14.0, "mx_plan_tol_planta_m": 0.78, "mx_pred_relajar_s": 30.0,
       "mx_pred_giro_extra_m": 0.04, "mis_dist_visita_m": 0.8, "mis_dist_insp_m": 0.8, "mis_reintentos": 1, "usar_hueco": 0, "mis_escaneo_s": 6.4,
       "usar_sonar": 0, "mx_sonar_prof_m": 1.0, "mx_sonar_w_meta": 1.0, "mx_sonar_w_visitas": 0.5}


def costo(r: dict) -> float:
    m = r["mision"]
    c = 35.0 * (1 - r["progreso"]) + 14.0 * min(r["contactos"], 5) + 20.0 * min(m["error_conteo"], 1.0) + 15.0 * (1 - m["recall"])
    c += 20.0 * (1 - m.get("cobertura_escaneo", 0.0)) + 0.02 * r["t_s"] + 10.0 * bool(r["atascado"]) + 10.0 * (not r["llego"])
    return round(c, 3)


def campos(p, entorno, guardian=True, mx=True):
    c = EI.campos(p, entorno)
    g = lambda k, d: p.get(k, E11.get(k, d))               # noqa: E731
    c.update(mx_regreso=True, mision=True, guardian=guardian, guardian_usar_memoria=False, guardian_tol_cm=p.get("guardian_tol_cm", 6.0),
             guardian_tol_deg=p.get("guardian_tol_deg", 4.0), mis_escaneo_s=p.get("mis_escaneo_s", 6.4))
    if not guardian and not mx:
        return c                                            # reactiva clásica
    c.update(mx_planner=True, mx_prediccion=True, mx_barrido=True, mx_meta=True, mx_anti_dither=True, mx_hueco=bool(round(g("usar_hueco", 0))),
             mx_plan_inflar_m=g("mx_plan_inflar_m", 0.22), mx_plan_mirada_m=g("mx_plan_mirada_m", 1.0), mx_plan_cada_s=g("mx_plan_cada_s", 2.0),
             mx_plan_giro_deg=g("mx_plan_giro_deg", 14.0), mx_pred_relajar_s=g("mx_pred_relajar_s", 30.0), mx_pred_giro_extra_m=g("mx_pred_giro_extra_m", 0.04),
             mx_pred_avance_m=p.get("mx_pred_avance_m", 0.30), mx_pred_atras_m=p.get("mx_pred_atras_m", 0.22), mx_pred_margen_m=p.get("mx_pred_margen_m", 0.05),
             mx_hueco_m=p.get("mx_hueco_m", 1.0), mx_grados_pulso=p.get("mx_grados_pulso", 6.0), mx_giro_largo=int(round(p.get("mx_giro_largo", 5))),
             mx_bordear_s=p.get("mx_bordear_s", 12.0), mx_rumbo_tol_deg=p.get("mx_rumbo_tol_deg", 25.0), mx_ventana_s=p.get("mx_ventana_s", 25.0),
             mx_ancho_half_m=p.get("mx_ancho_half_m", 0.26))
    if round(g("usar_sonar", 0)):                           # el sonar sustituye al «seguir el hueco» del scan de ahora
        c.update(mx_sonar=True, mx_hueco=False, mx_sonar_prof_m=g("mx_sonar_prof_m", 1.0), mx_sonar_w_meta=g("mx_sonar_w_meta", 1.0),
                 mx_sonar_w_visitas=g("mx_sonar_w_visitas", 0.5), mx_memoria_w=p.get("mx_memoria_w", 0.5))
    if mx:                                                  # E11: visitar cada planta vista y parar a ~0,8 m
        c.update(mx_visitar=True, mx_visitar_m=p.get("mx_visitar_m", 3.5), mx_plan_tol_planta_m=g("mx_plan_tol_planta_m", 0.78),
                 mis_dist_visita_m=g("mis_dist_visita_m", 0.8), mis_dist_insp_m=g("mis_dist_insp_m", 0.8), mis_reintentos=int(round(g("mis_reintentos", 1))))
    else:                                                   # E6: ir y volver sin visitar
        c.update(mx_visitar=False)
    return c


# ---- parcheo de entrenar_misiones para que use este costo y este espacio
MS.costo = costo
EM.campos = campos
EM.ESPACIO.update(NUEVOS)
EM.CLAVES[:] = list(EM.ESPACIO)
EI.ESPACIO, EI.CLAVES = EM.ESPACIO, EM.CLAVES
EI.ENTEROS = tuple(set(EI.ENTEROS) | {"mis_reintentos", "mx_giro_largo"})
EM.EXTRA_HOY.update(E11)
EM.HOY = dict(EI.HOY, **EM.EXTRA_HOY)
EM.MAN = dict(EM.HOY)

if __name__ == "__main__":
    inicio = Path.home() / ".agrobot" / "aprendizaje" / "e11_inicio.json"
    if "--desde" not in sys.argv:
        previos = sorted(Path.home().glob(".agrobot/aprendizaje/misiones_2026*/informe.json"))
        base = dict(EM.EXTRA_HOY)
        for q in previos[::-1]:                             # parámetros de bajo nivel del mejor entrenamiento anterior
            try:
                base.update(json.loads(q.read_text())["mejor"]["params"])
                break
            except (OSError, ValueError, KeyError):
                continue
        base.update(E11)
        inicio.write_text(json.dumps({"mejor": {"params": base}}))
        sys.argv += ["--desde", str(inicio)]
    EM.main()
