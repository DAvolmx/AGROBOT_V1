"""Tests del renderizador sintético de interiores y del reparto del dataset."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI / "percepcion_sintetica"))
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))

import render as R                                   # noqa: E402
import escena as E                                   # noqa: E402
import generar_dataset as G                          # noqa: E402

GEO_ROBOT = {"camara_altura_m": 0.40, "camara_inclinacion_deg": 15.5,
             "camara_hfov_deg": 71, "camara_adelante_m": 0.42}


def _sala_vacia(**kw):
    paredes = [(-1, -2, 6, -2), (-1, 2, 6, 2), (-1, -2, -1, 2), (6, -2, 6, 2)]
    return R.Escena(paredes, [], [], dict(tipo="hormigon", color=(140, 140, 138)),
                    dict(ambiente=0.7, dir=(40, 60)), limites=(-1, -2, 6, 2), **kw)


def _render(esc, x=0.0, y=0.0, psi=0.0, **kw):
    return R.render(esc, x, y, psi, np.random.default_rng(0), efectos=False, **kw)


def test_geometria_igual_a_sendero_cenital():
    """Si esto divergiera, un píxel 'suelo libre' se proyectaría a otro sitio en el robot."""
    import sendero
    cen = sendero.Cenital(GEO_ROBOT)
    W, H = sendero.ANCHO_ANALISIS, sendero.ALTO_ANALISIS
    o = np.array([R.CAMARA["adelante"], 0.0, R.CAMARA["altura"]])
    comparados = 0
    for X, Y in ((1.0, 0.3), (1.0, -0.3), (1.5, 0.5), (1.2, -0.1), (0.9, 0.2)):
        fila = int(round((sendero.CENITAL_X[1] - X) / sendero.CENITAL_RES_M))
        col = int(round((sendero.CENITAL_Y[1] - Y) / sendero.CENITAL_RES_M))
        u_ref, v_ref = cen.map_u[fila, col], cen.map_v[fila, col]
        if v_ref < 0:
            continue                          # sendero.py lo marca como no visible
        u, v, _ = R._proyectar((X, Y, 0.0), o, 0.0, R.CAMARA, W, H)
        assert abs(u - u_ref) < 0.6 and abs(v - v_ref) < 0.6, (X, Y)
        comparados += 1
    assert comparados >= 4


def test_suelo_abajo_y_pared_arriba():
    r = _render(_sala_vacia())
    assert (r["clase"][-1] == 1).mean() > 0.95        # fila inferior: suelo
    assert (r["clase"][0] != 1).all()                  # fila superior: pared o techo
    assert r["rgb"].shape == (240, 320, 3)


def test_orientacion_izquierda_derecha():
    """Un objeto a la IZQUIERDA del robot (y>0) aparece en la mitad izquierda de la imagen."""
    esc = _sala_vacia()
    esc.cilindros.append(R.Cilindro(1.2, 0.5, 0.1, 0.3, 2, (180, 95, 60), id=0))
    cols = np.nonzero((_render(esc)["clase"] == 2).any(0))[0]
    assert cols.size and cols.mean() < 160
    esc2 = _sala_vacia()
    esc2.cilindros.append(R.Cilindro(1.2, -0.5, 0.1, 0.3, 2, (180, 95, 60), id=0))
    cols2 = np.nonzero((_render(esc2)["clase"] == 2).any(0))[0]
    assert cols2.mean() > 160


def test_la_maceta_tapa_el_suelo_y_es_clase_2():
    esc = _sala_vacia()
    esc.cilindros.append(R.Cilindro(1.0, 0.0, 0.12, 0.30, 2, (180, 95, 60), copa_r=0.18, copa_h=0.2, id=0))
    r = _render(esc)
    centro = r["clase"][:, 160]
    assert (centro == 2).any()
    # por debajo de la maceta (más cerca) hay suelo; por encima, pared/techo
    filas = np.nonzero(centro == 2)[0]
    assert centro[min(filas.max() + 15, 239)] == 1
    assert (r["instancia"][:, 160] == 0).any()


def test_un_bordillo_bajo_es_clase_3_y_no_clase_4():
    """Lo que el LIDAR a 18 cm no ve: 6 cm de alto, a 0,9 m."""
    esc = _sala_vacia()
    esc.cajas.append(R.Caja(0.9, 0.0, 0.1, 1.2, 0.06, 0.0, 3, (220, 200, 60), id=0))
    r = _render(esc)
    assert (r["clase"] == 3).sum() > 50
    assert (r["clase"] == 4).sum() == 0


def test_un_obstaculo_alto_es_clase_4():
    esc = _sala_vacia()
    esc.cajas.append(R.Caja(1.2, 0.0, 0.3, 0.3, 0.4, 0.0, 4, (190, 150, 100), id=0))
    assert (_render(esc)["clase"] == 4).sum() > 200


def test_misma_semilla_misma_escena():
    a = E.escena_aleatoria(np.random.default_rng(5))
    b = E.escena_aleatoria(np.random.default_rng(5))
    assert [(c.x, c.y, c.r) for c in a.cilindros] == [(c.x, c.y, c.r) for c in b.cilindros]


def test_las_poses_estan_libres_de_objetos():
    rng = np.random.default_rng(2)
    for _ in range(5):
        esc = E.escena_aleatoria(rng)
        for (x, y, *_r) in E.poses_sesion(esc, rng, 8):
            assert E._libre(x, y, esc.objetos(), 0.2)


def test_las_cajas_de_plaga_caen_dentro_de_la_imagen():
    rng = np.random.default_rng(9)
    vistas = 0
    for _ in range(30):
        esc = E.escena_aleatoria(rng, "vivero")
        for (x, y, psi, cab, bal, inc) in E.poses_sesion(esc, rng, 2):
            cam = dict(R.CAMARA, inclinacion=inc)
            r = R.render(esc, x, y, psi, rng, cam=cam, cabeceo_extra=cab, balanceo=bal, efectos=False)
            for (x0, y0, x1, y1, t) in r["plagas"]:
                assert -5 <= x0 < x1 <= 325 and -5 <= y0 < y1 <= 245 and 0 <= t < len(R.TIPOS_PLAGA)
                vistas += 1
    assert vistas > 0


def test_los_splits_usan_semillas_disjuntas():
    bases = sorted(G.BASE_SEMILLA.values())
    # cada split tiene hueco de >= 20000 sesiones antes del siguiente: no se pisan
    assert all(b - a >= 20_000 for a, b in zip(bases, bases[1:]))


@pytest.mark.skipif(not (G.DATOS / "train").exists() or not (G.DATOS / "test").exists(), reason="sin dataset")
def test_ninguna_sesion_se_comparte_entre_splits():
    def firmas(split):
        out = set()
        for f in sorted((G.DATOS / split).glob("*.npz"))[:10]:
            d = np.load(f, allow_pickle=True)
            out.add(d["rgb"][0].tobytes()[:5000])
        return out
    assert not (firmas("train") & firmas("test"))
    assert not (firmas("train") & firmas("val"))
