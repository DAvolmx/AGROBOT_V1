import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import entorno_auto as E


def _escaneo_sala(ancho=4.0, largo=3.0, n=360):
    """Rayos desde el centro de una sala rectangular."""
    pts = []
    for a in range(n):
        th = math.radians(a * 360 / n)
        c, s = math.cos(th), math.sin(th)
        tx = (ancho / 2) / abs(c) if abs(c) > 1e-9 else math.inf
        ty = (largo / 2) / abs(s) if abs(s) > 1e-9 else math.inf
        pts.append({"angle_deg": a * 360 / n, "distance_m": min(tx, ty)})
    return pts


def _escaneo_abierto(seed=3, plantas=8, n=360):
    """Campo abierto: unas pocas plantas (círculos) y el resto sin retorno."""
    rng = np.random.default_rng(seed)
    pl = [(rng.uniform(2, 7) * math.cos(t), rng.uniform(2, 7) * math.sin(t), rng.uniform(.1, .25))
          for t in rng.uniform(0, 2 * math.pi, plantas)]
    pts = []
    for a in range(n):
        th = math.radians(a)
        d = math.inf
        for (x, y, r) in pl:
            b = -(x * math.cos(th) + y * math.sin(th))
            disc = b * b - (x * x + y * y - r * r)
            if disc >= 0:
                t = b - math.sqrt(disc)
                if t > 0.05:
                    d = min(d, t)
        pts.append({"angle_deg": a, "distance_m": d})
    return pts


def test_una_sala_es_interior():
    r = E.rasgos_lidar(_escaneo_sala())
    assert r["cercano"] > 0.95 and r["rectitud"] > 0.6
    s, _ = E.puntuacion(r)
    assert s > 0.5


def test_campo_abierto_es_exterior():
    s, _ = E.puntuacion(E.rasgos_lidar(_escaneo_abierto()))
    assert s < -0.3


def test_escaneo_vacio_o_pobre():
    assert E.rasgos_lidar([]) is None
    r = E.rasgos_lidar([{"angle_deg": a, "distance_m": math.inf} for a in range(360)])
    assert r["cobertura"] == 0.0
    assert E.puntuacion(r)[0] < 0


def test_la_resolucion_del_escaneo_no_cambia_el_veredicto():
    a = E.puntuacion(E.rasgos_lidar(_escaneo_sala(n=360)))[0]
    b = E.puntuacion(E.rasgos_lidar(_escaneo_sala(n=720)))[0]
    assert a > 0.5 and b > 0.5


def test_camara_cielo_azul_empuja_a_exterior():
    cv2 = pytest.importorskip("cv2")
    azul = np.zeros((480, 640, 3), np.uint8)
    azul[:200] = (230, 150, 60)                     # BGR: cielo azul
    azul[200:] = (60, 120, 70)                      # verde
    beige = np.full((480, 640, 3), (150, 170, 190), np.uint8)   # pared de interior
    ca, cb = E.rasgos_camara(azul), E.rasgos_camara(beige)
    assert ca["cielo"] > 0.3 and cb["cielo"] < 0.05
    assert E.puntuacion_camara(ca) < E.puntuacion_camara(cb)
    assert E.rasgos_camara(None) is None


def _reloj():
    t = [0.0]
    f = lambda: t[0]
    f.avanzar = lambda s: t.__setitem__(0, t[0] + s)
    return f


def _correr(ent, rasgos, modo, segundos, **kw):
    ultimo = None
    for _ in range(int(segundos)):
        ent._reloj.avanzar(1.0)
        ultimo = ent.paso(rasgos, modo, **kw)
        if ultimo["cambiar_a"]:
            return ultimo
    return ultimo


EXT = E.rasgos_lidar(_escaneo_abierto())
INT = E.rasgos_lidar(_escaneo_sala())


def test_pasar_a_exterior_exige_mas_tiempo_que_pasar_a_interior():
    ent = E.EntornoAuto(reloj=_reloj())
    ent.activo = True
    r = _correr(ent, EXT, "interior", 20)          # 20 s de evidencia exterior: aún no
    assert r["cambiar_a"] is None
    r = _correr(ent, EXT, "interior", 40)
    assert r["cambiar_a"] == "exterior"

    ent2 = E.EntornoAuto(reloj=_reloj())
    ent2.activo = True
    r = _correr(ent2, INT, "exterior", 25)         # a interior bastan ~10 s sostenidos
    assert r["cambiar_a"] == "interior"


def test_apagado_solo_sugiere():
    ent = E.EntornoAuto(reloj=_reloj())
    r = _correr(ent, INT, "exterior", 60)
    assert r["cambiar_a"] is None and ent.sugerido == "interior" and "solo sugiere" in r["motivo"]


def test_no_cambia_en_movimiento_ni_con_obstaculo_ni_si_el_operador_manda():
    ent = E.EntornoAuto(reloj=_reloj())
    ent.activo = True
    assert _correr(ent, EXT, "interior", 80, puede_cambiar=False)["cambiar_a"] is None
    ent = E.EntornoAuto(reloj=_reloj())
    ent.activo = True
    assert _correr(ent, EXT, "interior", 80, hueco_frente_m=0.3)["cambiar_a"] is None
    ent = E.EntornoAuto(reloj=_reloj())
    ent.activo = True
    ent.fijar_operador("interior")
    assert ent.etiqueta_vigente() == "interior"
    assert _correr(ent, EXT, "interior", 80)["cambiar_a"] is None      # callado 600 s
    _correr(ent, EXT, "interior", 600)
    assert ent.etiqueta_vigente() is None


def test_preciso_es_decision_del_operador():
    ent = E.EntornoAuto(reloj=_reloj())
    ent.activo = True
    assert _correr(ent, EXT, "preciso", 80)["cambiar_a"] is None


def test_un_pico_suelto_no_cambia_nada():
    ent = E.EntornoAuto(reloj=_reloj())
    ent.activo = True
    _correr(ent, INT, "interior", 30)
    _correr(ent, EXT, "interior", 3)                # 3 escaneos de «exterior» entre 30 de interior
    r = _correr(ent, INT, "interior", 30)
    assert r["cambiar_a"] is None


def test_registro_y_modelo(tmp_path):
    E.registrar(tmp_path, {"t": 1, "x": 2})
    E.registrar(tmp_path, {"t": 2, "x": 3})
    linea = next(tmp_path.glob("*.jsonl")).read_text().strip().splitlines()
    assert len(linea) == 2 and json.loads(linea[1])["x"] == 3
    assert E.cargar_modelo(tmp_path) is None
    (tmp_path / "modelo.json").write_text(json.dumps(
        {"rasgos": ["cercano", "rectitud"], "media": [0.5, 0.3], "desv": [0.2, 0.2], "pesos": [1.0, 1.0], "sesgo": 0.0}))
    m = E.cargar_modelo(tmp_path)
    assert m is not None
    s, d = E.puntuacion({"cercano": 1.0, "rectitud": 0.8}, m)
    assert s > 0 and d["fuente"] == "modelo_entrenado"
    (tmp_path / "modelo.json").write_text('{"rasgos": ["inventado"], "media": [0], "desv": [1], "pesos": [1], "sesgo": 0}')
    assert E.cargar_modelo(tmp_path) is None
