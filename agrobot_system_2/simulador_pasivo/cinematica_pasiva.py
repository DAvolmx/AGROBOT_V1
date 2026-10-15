"""Qué rutas puede seguir el eje pasivo, y cuáles no.

Creado el 2026-10-14 al diagnosticar por qué el banco de evaluación se
estancaba. Las delanteras forman un módulo con pivote PASIVO: no hay servo de
dirección, el ángulo lo impone la geometría, y hay un tope mecánico. Eso fija
un RADIO DE GIRO MÍNIMO que ninguna ganancia de control puede sortear:

    R_min = batalla / tan(pivote_max)

Con la batalla medida (0,37 m) y el contrato actual:

    límite útil    17°  ->  R_min = 1,21 m
    tope mecánico  20°  ->  R_min = 1,02 m

HALLAZGO que motivó el módulo: dos de los seis escenarios del banco pedían
radios por debajo de eso —`ese` con r=1,0 m exige 20,3° de pivote, MÁS que el
tope— así que el robot no las termina por cinemática, no por control. Su coste
(`6·(1−progreso)` sobre un progreso de 0,47) es un SUELO irreducible. Con un
tercio de la función objetivo clavado, un ajuste de controlador no puede
distinguir candidatos: es la explicación de las 4 h y 142 episodios sin
aceptar ninguno.

Esto NO es una medida nueva del robot: sale de aplicar Ackermann al
`wheelbase_m` y `steer_limit_deg` de `contrato_robot.json`, que siguen sin
calibrar físicamente (`calibrated: false`). El tope de 20° es el dato más
firme (es un tope físico). Al medir el pivote real, estos números se mueven.
"""
import json
import math
from pathlib import Path

CONTRATO = Path(__file__).parent / "contrato_robot.json"


def _contrato():
    return json.loads(CONTRATO.read_text())


def radio_minimo(pivote_deg: float, batalla_m: float | None = None) -> float:
    """Radio de giro mínimo (m) para ese ángulo de pivote. inf si es 0."""
    if batalla_m is None:
        batalla_m = _contrato()["wheelbase_m"]
    if pivote_deg <= 0:
        return math.inf
    return batalla_m / math.tan(math.radians(pivote_deg))


def pivote_para_radio(radio_m: float, batalla_m: float | None = None) -> float:
    """Ángulo de pivote (grados) que hace falta para ese radio."""
    if batalla_m is None:
        batalla_m = _contrato()["wheelbase_m"]
    if radio_m <= 0:
        return 90.0
    return math.degrees(math.atan(batalla_m / radio_m))


def radio_local(p0, p1, p2) -> float:
    """Radio del círculo que pasa por tres puntos (m). inf si son colineales.

    Es la curvatura que la ruta le PIDE al robot en ese tramo.
    """
    (x0, y0), (x1, y1), (x2, y2) = p0, p1, p2
    # Doble del área del triángulo: 0 => colineales.
    area2 = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
    if abs(area2) < 1e-12:
        return math.inf
    a = math.dist(p0, p1)
    b = math.dist(p1, p2)
    c = math.dist(p0, p2)
    return (a * b * c) / abs(2.0 * area2)


def factibilidad(ruta, batalla_m: float | None = None,
                 pivote_util_deg: float | None = None,
                 pivote_tope_deg: float | None = None) -> dict:
    """¿Puede el eje pasivo seguir esta ruta?

    Devuelve el tramo más cerrado, el pivote que exige y un veredicto:

      "ok"         cabe dentro del límite útil.
      "al_limite"  entre el límite útil y el tope mecánico: la sigue con el
                   pivote clavado al tope, sin margen para corregir.
      "imposible"  exige más que el tope: el robot NO la puede seguir, y su
                   progreso bajo NO es culpa del controlador.
    """
    c = _contrato()
    batalla_m = c["wheelbase_m"] if batalla_m is None else batalla_m
    util = c["steer_limit_deg"] if pivote_util_deg is None else pivote_util_deg
    tope = c["mechanical_stop_deg"] if pivote_tope_deg is None else pivote_tope_deg

    puntos = [tuple(p) for p in ruta]
    radio_min, donde = math.inf, None
    for i in range(len(puntos) - 2):
        r = radio_local(puntos[i], puntos[i + 1], puntos[i + 2])
        if r < radio_min:
            radio_min, donde = r, i + 1

    pivote = pivote_para_radio(radio_min, batalla_m)
    if pivote <= util:
        veredicto = "ok"
    elif pivote <= tope:
        veredicto = "al_limite"
    else:
        veredicto = "imposible"
    return {"radio_min_m": round(radio_min, 3) if radio_min != math.inf else None,
            "pivote_necesario_deg": round(pivote, 1),
            "pivote_util_deg": util, "pivote_tope_deg": tope,
            "indice": donde, "veredicto": veredicto,
            "factible": veredicto != "imposible"}


def radio_seguro(margen: float = 1.25, batalla_m: float | None = None) -> float:
    """Radio que deja margen de corrección sobre el límite ÚTIL.

    Seguir una curva al límite del pivote significa ir con la dirección
    clavada al tope: cualquier desvío ya no se puede corregir hacia dentro.
    Con el 25 % de margen por defecto el controlador conserva autoridad.
    """
    c = _contrato()
    batalla_m = c["wheelbase_m"] if batalla_m is None else batalla_m
    return radio_minimo(c["steer_limit_deg"], batalla_m) * margen


if __name__ == "__main__":
    c = _contrato()
    print(f"batalla {c['wheelbase_m']} m · util {c['steer_limit_deg']}° · "
          f"tope {c['mechanical_stop_deg']}°")
    print(f"R_min util  = {radio_minimo(c['steer_limit_deg']):.3f} m")
    print(f"R_min tope  = {radio_minimo(c['mechanical_stop_deg']):.3f} m")
    print(f"R seguro    = {radio_seguro():.3f} m")
