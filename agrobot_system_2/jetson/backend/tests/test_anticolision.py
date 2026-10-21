import anticolision as A


class Reloj:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def _nuevo():
    r = Reloj()
    return A.Anticolision(r), r


S = {}      # defectos


def test_sin_obstaculo_no_toca_nada():
    a, _ = _nuevo()
    assert a.filtrar(0.2, 0.3, {"adelante": 1.0, "atras": 1.0}, 0.1, S) == (0.2, 0.3, None, False)


def test_obstaculo_delante_dispara_retroceso_y_cancela_avance():
    a, _ = _nuevo()
    v, w, ev, reset = a.filtrar(0.25, 0.0, {"adelante": 0.08, "atras": 1.0}, 0.1, S)
    assert v < 0 and w == 0 and ev["tipo"] == "retroceso" and reset and a.estado == A.RETROCEDIENDO


def test_retrocede_por_tiempo_y_luego_bloquea_el_avance():
    a, r = _nuevo()
    a.filtrar(0.25, 0.0, {"adelante": 0.08, "atras": 1.0}, 0.1, S)
    r.t += 0.5
    v, _, _, _ = a.filtrar(0.25, 0.0, {"adelante": 0.2, "atras": 0.8}, 0.1, S)
    assert v == -A.DEFECTOS["anticolision_v_retroceso"]
    r.t += 1.0                                         # 0,15 m a 0,15 m/s = 1 s
    v, _, ev, _ = a.filtrar(0.25, 0.0, {"adelante": 0.25, "atras": 0.6}, 0.1, S)
    assert v == 0.0 and ev["tipo"] == "fin_retroceso" and a.estado == A.BLOQUEADO
    v, _, _, _ = a.filtrar(0.25, 0.0, {"adelante": 0.25, "atras": 0.6}, 0.1, S)      # sigue apretando avanzar
    assert v == 0.0


def test_al_soltar_el_mando_se_libera():
    a, r = _nuevo()
    a.filtrar(0.25, 0.0, {"adelante": 0.08, "atras": 1.0}, 0.1, S)
    r.t += 2.0
    a.filtrar(0.0, 0.0, {"adelante": 0.25, "atras": 0.6}, 0.1, S)      # fin del retroceso
    r.t += 0.1
    a.filtrar(0.0, 0.0, {"adelante": 0.25, "atras": 0.6}, 0.1, S)      # mando suelto
    r.t += 0.5
    a.filtrar(0.0, 0.0, {"adelante": 0.25, "atras": 0.6}, 0.1, S)
    assert a.estado == A.LIBRE
    v, _, _, _ = a.filtrar(0.2, 0.0, {"adelante": 0.5, "atras": 0.6}, 0.1, S)
    assert v == 0.2


def test_sin_espacio_atras_solo_se_detiene():
    a, _ = _nuevo()
    v, _, ev, reset = a.filtrar(0.25, 0.0, {"adelante": 0.05, "atras": 0.10}, 0.1, S)
    assert v == 0.0 and ev["tipo"] == "alto_sin_espacio" and reset and a.estado == A.BLOQUEADO


def test_algo_detras_corta_el_retroceso():
    a, r = _nuevo()
    a.filtrar(0.25, 0.0, {"adelante": 0.08, "atras": 1.0}, 0.1, S)
    r.t += 0.2
    v, _, ev, _ = a.filtrar(0.0, 0.0, {"adelante": 0.2, "atras": 0.05}, 0.1, S)
    assert v == 0.0 and ev["motivo"] == "algo detrás"


def test_scan_viejo_no_actua_y_apagado_no_actua():
    a, _ = _nuevo()
    assert a.filtrar(0.25, 0.0, {"adelante": 0.05, "atras": 1.0}, 3.0, S)[0] == 0.25
    assert a.filtrar(0.25, 0.0, {"adelante": 0.05, "atras": 1.0}, 0.1, {"anticolision_activa": False})[0] == 0.25


def test_el_giro_y_la_marcha_atras_no_se_bloquean():
    a, _ = _nuevo()
    assert a.filtrar(0.0, 1.0, {"adelante": 0.05, "atras": 1.0}, 0.1, S) == (0.0, 1.0, None, False)
    assert a.filtrar(-0.15, 0.0, {"adelante": 0.05, "atras": 1.0}, 0.1, S)[0] == -0.15
