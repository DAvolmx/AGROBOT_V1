"""MONTE CARLO del SENDERO CORTO de la prueba de exterior 2 (2026-10-28): huerta con coles sembradas a ambos lados, sendero de poca longitud y ancho 0,70-1,2 m.
Compara el comportamiento CON marcha atrás (esquina: retrocedo 2 pulsos de ~30 cm + giro compensado, como el 28-oct) con la PRIORIDAD ADELANTE (sin marcha atrás), y barre el ancho.
Mide plantas tocadas (el robot no tiene sensores atrás), éxito y progreso. Uso: PYTHONPATH=~/.agrobot/numba_pkg python3 mc_exterior_sendero.py [--semillas 12] [--procs 5]
Salida: docs/exterior2_20261028/mc_sendero.json"""
import argparse, itertools, json, time
from multiprocessing import Pool
from pathlib import Path

import entorno_calibrado as EC
import entrenar_intensivo as EI
import sim_logica_actual as S

SAL = Path(__file__).resolve().parents[1] / "docs" / "exterior2_20261028"
E = S.Escenario
VARIANTES = {
    "con_atras (28-oct)": dict(man_retroceso_cm=29.8, man_atras_pulsos=2, mx_giro_comp=3),
    "prioridad_adelante": dict(man_retroceso_cm=0.0, man_atras_pulsos=1, mx_giro_comp=0),
}


def sendero(ancho, largo=3.2, sep=0.55):
    return E(f"sendero_{ancho:.2f}", [], EC._hileras(ancho, largo, sep, x_ini=0.6), goal_x=largo + 0.8, t_max=140.0, descripcion=f"Sendero de {ancho:.2f} m y {largo:.1f} m con coles a los lados")


def _job(a):
    var, ancho, semilla = a
    try:
        p = dict(EI.HOY)
        c = EI.campos(p, "exterior"); c.update(VARIANTES[var])
        S.SUBSTEP = 0.005
        r = S.episodio(sendero(ancho), S.Config(**c), semilla, EC.hip(semilla))
        return {"var": var, "ancho": ancho, "seed": semilla, "exito": bool(r["exito"]), "contactos": int(r["contactos"]), "progreso": float(r["progreso"]), "t_s": float(r["t_s"]),
                "atascado": bool(r["atascado"]), "puntos": float(S.puntuar_v2(r))}
    except Exception as e:                                                  # noqa: BLE001
        return {"var": var, "ancho": ancho, "seed": semilla, "error": repr(e), "exito": False, "contactos": 9, "progreso": 0.0, "t_s": 0.0, "atascado": True, "puntos": 40.0}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--semillas", type=int, default=12); ap.add_argument("--procs", type=int, default=5)
    ap.add_argument("--anchos", default="0.70,0.85,1.0,1.2"); a = ap.parse_args(); SAL.mkdir(parents=True, exist_ok=True)
    anchos = [float(x) for x in a.anchos.split(",")]
    trab = [(v, w, 700 + s) for v, w in itertools.product(VARIANTES, anchos) for s in range(a.semillas)]
    t0 = time.time()
    with Pool(a.procs) as p:
        rs = p.map(_job, trab, chunksize=1)
    filas = []
    for v, w in itertools.product(VARIANTES, anchos):
        l = [r for r in rs if r["var"] == v and r["ancho"] == w]; n = len(l)
        filas.append({"variante": v, "ancho_m": w, "n": n, "exito_pct": round(100 * sum(r["exito"] for r in l) / n, 1), "contactos_por_ep": round(sum(r["contactos"] for r in l) / n, 2),
                      "sin_contacto_pct": round(100 * sum(r["contactos"] == 0 for r in l) / n, 1), "progreso": round(sum(r["progreso"] for r in l) / n, 3), "atascado_pct": round(100 * sum(r["atascado"] for r in l) / n),
                      "t_s": round(sum(r["t_s"] for r in l) / n), "puntos": round(sum(r["puntos"] for r in l) / n, 2)})
    (SAL / "mc_sendero.json").write_text(json.dumps({"filas": filas, "episodios": rs, "segundos": round(time.time() - t0)}, indent=1, ensure_ascii=False))
    for f in filas:
        print(f)
    print("segundos", round(time.time() - t0))


if __name__ == "__main__":
    main()
