from servo_bajo import ServoBajo, maceta_baja, maceta_alta

H = 720


def test_caja_cortada_por_abajo_es_baja():
    assert maceta_baja([100, 500, 200, 719], H)
    assert maceta_baja([100, 400, 200, 700], H)          # centro 550/720 = 76 %
    assert not maceta_baja([100, 200, 200, 420], H)       # centro 43 %


def test_baja_el_servo_y_respeta_el_tope():
    s = ServoBajo()
    a = 98
    for t in range(10):
        n = s.paso([[100, 500, 200, 719]], H, a, float(t))
        if n is not None:
            a = n
    assert a == 62


def test_sube_si_la_maceta_esta_alta_y_el_servo_estaba_abajo():
    s = ServoBajo()
    assert s.paso([[100, 50, 200, 200]], H, 74, 1.0) == 77
    assert s.paso([[100, 50, 200, 200]], H, 98, 2.0) is None   # ya en reposo


def test_vuelve_al_reposo_tras_unos_segundos_sin_macetas():
    s = ServoBajo(reposo_s=4.0)
    s.paso([[100, 500, 200, 719]], H, 98, 0.0)
    assert s.paso([], H, 80, 2.0) is None                      # aún no
    assert s.paso([], H, 80, 5.0) == 84                        # empieza a volver
    assert s.paso([], H, 98, 9.0) is None


def test_maceta_a_media_altura_no_mueve_nada():
    s = ServoBajo()
    assert s.paso([[100, 250, 200, 450]], H, 80, 1.0) is None
