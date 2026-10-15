"""Corredor y decisión: se comprueban contra geometría CONOCIDA del render sintético."""
import sys
from pathlib import Path

import math
import numpy as np
import pytest

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI / "percepcion_sintetica"))
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))

import render as R          # noqa: E402
import corredor as C        # noqa: E402


def _escena(ancho=4.0, **kw):
    a = ancho / 2
    return R.Escena([(-1, -a, 8, -a), (-1, a, 8, a), (-1, -a, -1, a), (8, -a, 8, a)], [], [],
                    dict(tipo="hormigon", color=(140, 140, 138)), dict(ambiente=0.7, dir=(40, 60)),
                    limites=(-1, -a, 8, a), **kw)


def _mascara(esc, x=0.0, y=0.0, psi=0.0, inc=15.5):
    cam = dict(R.CAMARA, inclinacion=inc)
    r = R.render(esc, x, y, psi, np.random.default_rng(0), cam=cam, efectos=False)
    return r["clase"], inc


def test_sala_vacia_es_libre_y_se_avanza():
    m, inc = _mascara(_escena())
    c = C.corredor(m, inc)
    assert abs(c.centro_m[1.0]) < 0.06
    # El campo de visión limita el ancho visible a ~±0,47 m a 1 m: es una COTA INFERIOR, no la sala.
    assert c.ancho_m[1.0] > 0.85
    assert c.trunc[1.0] == (True, True)            # truncado por el campo de visión en los dos lados
    assert c.obstaculo_m is None
    assert C.decidir(c).accion == "avanzar"


def test_pasillo_desplazado_da_el_centro_del_lado_correcto():
    """Robot a +0,2 m del eje de un pasillo de 1,0 m: el centro está 0,2 m a su DERECHA (y<0).
    A 1,0 m el campo de visión trunca el hueco y sesga el centro hacia el eje (-8 cm en vez de -20);
    a 1,4 m ya se ven las dos paredes."""
    m, inc = _mascara(_escena(ancho=1.0), y=0.2)
    c = C.corredor(m, inc)
    assert c.trunc[1.0] != (False, False)           # a 1,0 m el hueco está truncado: no fiarse del centro
    assert c.trunc[1.4] == (False, False)
    assert c.centro_m[1.4] == pytest.approx(-0.2, abs=0.05)
    assert c.ancho_m[1.4] == pytest.approx(1.0, abs=0.1)


def test_maceta_delante_se_mide_a_su_distancia():
    esc = _escena()
    esc.cilindros.append(R.Cilindro(1.0, 0.0, 0.12, 0.30, 2, (180, 95, 60), id=0))
    m, inc = _mascara(esc)
    c = C.corredor(m, inc)
    assert c.clase_obstaculo == 2
    assert c.obstaculo_m == pytest.approx(0.88, abs=0.08)       # cara más cercana de la maceta
    d = C.decidir(c)
    assert d.accion in ("reducir", "parar")                      # la base queda justo en el borde de la zona ciega


def test_obstaculo_cercano_para():
    esc = _escena()
    esc.cilindros.append(R.Cilindro(0.75, 0.0, 0.12, 0.30, 2, (180, 95, 60), id=0))
    m, inc = _mascara(esc)
    assert C.decidir(C.corredor(m, inc)).accion == "parar"


def test_el_bordillo_bajo_se_detecta_aunque_el_lidar_no_lo_vea():
    esc = _escena()
    esc.cajas.append(R.Caja(1.3, 0.0, 0.1, 1.5, 0.06, 0.0, 3, (220, 200, 60), id=0))
    m, inc = _mascara(esc)
    c = C.corredor(m, inc)
    assert c.clase_obstaculo == 3
    assert c.obstaculo_m == pytest.approx(1.25, abs=0.12)


def test_objeto_fuera_del_pasillo_barrido_no_cuenta():
    esc = _escena()
    esc.cilindros.append(R.Cilindro(1.0, 0.7, 0.1, 0.3, 2, (180, 95, 60), id=0))    # a 70 cm del eje
    m, inc = _mascara(esc)
    assert C.corredor(m, inc).obstaculo_m is None


def test_datos_viejos_paran():
    m, inc = _mascara(_escena())
    assert C.decidir(C.corredor(m, inc), edad_s=0.8).accion == "parar"


def test_pared_de_frente_sin_suelo_visible_para():
    """Camara mirando a una pared pegada: 'no ver suelo' NO es 'camino libre'."""
    esc = _escena()
    m, inc = _mascara(esc, x=7.4, psi=0.0)          # cámara a 7,82 m: la pared (x=8) queda a 18 cm
    d = C.decidir(C.corredor(m, inc))
    assert d.accion == "parar"


def test_hueco_estrecho_no_admite_el_robot():
    esc = _escena()
    for lado in (1, -1):
        esc.cajas.append(R.Caja(1.1, lado * 0.33, 0.5, 0.3, 0.4, 0.0, 4, (190, 150, 100), id=(0 if lado > 0 else 1)))
    m, inc = _mascara(esc)
    c = C.corredor(m, inc)
    assert c.ancho_m[1.0] is not None and c.ancho_m[1.0] < 0.5
    assert C.decidir(c).accion in ("parar", "reducir")


def test_seguidor_pierde_tras_varias_ausencias():
    s = C.SeguidorCorredor(perdidas_max=3)
    vacio = C.Corredor(centro_m={1.0: None}, ancho_m={1.0: None})
    bueno = C.Corredor(centro_m={1.0: 0.1}, ancho_m={1.0: 0.8})
    assert s.actualizar(bueno, 0.0)["perdido"] is False
    for k in range(1, 4):
        est = s.actualizar(vacio, 0.1 * k)
    assert est["perdido"] is True


def test_el_filtro_suaviza_el_centro():
    s = C.SeguidorCorredor(alfa=0.5)
    s.actualizar(C.Corredor(centro_m={1.0: 0.0}, ancho_m={1.0: 1.0}), 0.0)
    est = s.actualizar(C.Corredor(centro_m={1.0: 0.2}, ancho_m={1.0: 1.0}), 0.1)
    assert est["centro_m"] == pytest.approx(0.1)


def test_zona_ciega_depende_de_la_inclinacion_y_se_declara():
    """Con 15,5° no se ve suelo a menos de ~89 cm; al bajar la cámara a 33° (servo 76°) se acerca."""
    assert C.zona_ciega(15.5) == pytest.approx(0.89, abs=0.03)
    assert C.zona_ciega(33.0) < C.zona_ciega(15.5) < C.zona_ciega(5.0)


def test_un_objeto_con_la_base_en_la_zona_ciega_no_se_subestima():
    """Maceta a 0,75 m: su base no se ve y la parte alta se proyectaría MÁS LEJOS. Debe frenar."""
    esc = _escena()
    esc.cilindros.append(R.Cilindro(0.85, 0.0, 0.1, 0.30, 2, (180, 95, 60), id=0))
    m, inc = _mascara(esc)
    c = C.corredor(m, inc)
    assert c.en_zona_ciega
    assert c.obstaculo_m <= C.zona_ciega(inc) + 1e-6
    assert C.decidir(c).accion == "parar"


# --- memoria de obstáculos (zona ciega) --------------------------------------
def _c(obst=None, ciega=False, clase=2):
    return C.Corredor(obstaculo_m=obst, en_zona_ciega=ciega, clase_obstaculo=clase if obst is not None else None,
                      libre_visible=0.9, visible_m=1.0, zona_ciega_m=0.89)


def test_la_memoria_resta_el_avance_y_sigue_viendo_lo_que_ya_no_se_ve():
    m = C.MemoriaObstaculo()
    assert m.actualizar(_c(1.0), 0.0, 0.0, 0.0) == pytest.approx(1.0)
    for k in range(1, 4):                              # el objeto sale de la vista: ya no hay observación
        d = m.actualizar(_c(None), 0.15, 0.0, 0.5 * k)
    assert d == pytest.approx(0.55, abs=1e-6)


def test_con_memoria_se_para_aunque_la_camara_ya_no_lo_vea():
    m = C.MemoriaObstaculo()
    m.actualizar(_c(0.95), 0.0, 0.0, 0.0)
    d = m.actualizar(_c(None), 0.45, 0.0, 1.0)         # recordado a 0,50 m
    assert C.decidir(_c(None), dist_recordada=d).accion == "parar"
    assert C.decidir(_c(None)).accion == "avanzar"     # sin memoria no se entera


def test_la_memoria_caduca_si_el_robot_gira_mucho():
    m = C.MemoriaObstaculo()
    m.actualizar(_c(1.0), 0.0, 0.0, 0.0)
    assert m.actualizar(_c(None), 0.0, math.radians(30), 0.5) is None


def test_la_memoria_caduca_por_tiempo():
    m = C.MemoriaObstaculo(caducidad_s=2.0)
    m.actualizar(_c(1.0), 0.0, 0.0, 0.0)
    assert m.actualizar(_c(None), 0.0, 0.0, 3.0) is None


def test_una_observacion_directa_corrige_a_la_memoria():
    m = C.MemoriaObstaculo()
    m.actualizar(_c(1.0), 0.0, 0.0, 0.0)
    assert m.actualizar(_c(0.7), 0.1, 0.0, 0.5) == pytest.approx(0.7)


def test_una_pared_cercana_con_suelo_visible_delante_frena():
    """Hallazgo del 2026-10-15: las 13 'pérdidas' de peligros VISIBLES eran paredes a 72-85 cm
    que se trataban como camino libre (la pared es clase `fondo`, que no bloqueaba)."""
    esc = _escena()
    # robot a 7,25 m: la pared (x=8) queda a 0,75 m del centro; con inclinación 33° el suelo se ve desde 0,67 m
    cam = dict(R.CAMARA, inclinacion=33.0)
    r = R.render(esc, 7.25, 0.0, 0.0, np.random.default_rng(0), cam=cam, efectos=False)
    c = C.corredor(r["clase"], 33.0)
    assert c.obstaculo_m is not None and c.clase_obstaculo == 0
    assert 0.6 < c.obstaculo_m < 0.95
    assert C.decidir(c).accion in ("reducir", "parar")


def test_una_pared_lejana_no_impide_avanzar():
    m, inc = _mascara(_escena(), x=0.0)            # pared a 8 m
    c = C.corredor(m, inc)
    assert C.decidir(c).accion == "avanzar"


def test_la_memoria_nunca_rebaja_un_parar_de_la_camara():
    """Fallo del 2026-10-15: la memoria devolvía antes de evaluar la cámara y rebajaba 'parar' a 'reducir'."""
    c = _c(0.40, ciega=False)                              # la cámara ve un obstáculo a 40 cm: PARAR
    assert C.decidir(c).accion == "parar"
    assert C.decidir(c, dist_recordada=0.80).accion == "parar"      # una memoria menos grave no lo rebaja


def test_la_memoria_puede_subir_de_avanzar_a_reducir():
    c = _c(None)
    assert C.decidir(c).accion == "avanzar"
    assert C.decidir(c, dist_recordada=0.80).accion == "reducir"


def test_la_memoria_lejana_no_cambia_nada():
    assert C.decidir(_c(None), dist_recordada=1.5).accion == "avanzar"
