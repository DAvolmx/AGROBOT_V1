from zoom_auto import ZoomAuto, fraccion, zoom_maximo_que_cabe

W, H = 640, 480


def caja_centrada(f):                      # caja cuyo lado mayor ocupa la fracción f del ancho/alto, centrada
    w = f * W
    h = min(w * 1.1, f * H)
    return [W / 2 - w / 2, H / 2 - h / 2, W / 2 + w / 2, H / 2 + h / 2]


def test_planta_pequena_sube_el_zoom_con_paso_limitado():
    z = ZoomAuto()
    n = z.paso("encuadrando", caja_centrada(0.15), W, H, 1.0, 0.0)
    assert n == 1.3                                   # querría 3,67 pero sube ×1,3 por decisión


def test_converge_a_la_fraccion_objetivo():
    z, zoom, t = ZoomAuto(), 1.0, 0.0
    for _ in range(12):
        f = 0.15 * zoom                               # al subir el zoom la planta se ve proporcionalmente mayor
        n = z.paso("encuadrando", caja_centrada(min(f, 0.95)), W, H, zoom, t)
        t += 1.0
        if n:
            zoom = n
    assert zoom == 3.0                                # 0,80/0,15 = 5,3 -> tope 3,0


def test_zona_muerta_no_toca_nada():
    assert ZoomAuto().paso("analizando", caja_centrada(0.80), W, H, 2.0, 0.0) is None
    assert ZoomAuto().paso("analizando", caja_centrada(0.86), W, H, 2.0, 0.0) is None     # +7 %: dentro de la banda


def test_planta_demasiado_grande_baja_el_zoom():
    n = ZoomAuto().paso("analizando", caja_centrada(0.97), W, H, 3.0, 0.0)
    assert n is not None and n < 3.0 and n >= 3.0 * 0.8 - 1e-6


def test_no_hace_zoom_si_la_caja_se_saldria_del_recorte():
    caja = [400, 200, 620, 330]                       # descentrada a la derecha, pequeña
    tope = zoom_maximo_que_cabe(caja, W, H, 1.0, 0.08)
    assert tope < 1.4                                 # casi nada de zoom cabe sin perderla
    n = ZoomAuto().paso("encuadrando", caja, W, H, 1.0, 0.0)
    assert n is None or n <= tope + 1e-6


def test_vuelve_a_1x_al_salir_de_la_inspeccion():
    z = ZoomAuto()
    assert z.paso("alejando", None, W, H, 2.5, 0.0) == 1.0
    assert z.paso("buscando", None, W, H, 1.0, 5.0) is None


def test_periodo_minimo_entre_decisiones():
    z = ZoomAuto()
    assert z.paso("encuadrando", caja_centrada(0.15), W, H, 1.0, 0.0) is not None
    assert z.paso("encuadrando", caja_centrada(0.15), W, H, 1.3, 0.1) is None


def test_desactivable_por_config():
    assert ZoomAuto({"zoom_auto": False}).paso("encuadrando", caja_centrada(0.15), W, H, 1.0, 0.0) is None


def test_al_acercarse_no_hace_zoom_hasta_llegar():
    """2026-10-23: el zoom llenaba la imagen a 1,7 m y el robot dejaba de avanzar hacia la planta."""
    z = ZoomAuto()
    assert z.paso("centrando", caja_centrada(0.15), W, H, 1.0, 0.0) is None
    assert z.paso("acercando", caja_centrada(0.15), W, H, 1.0, 1.0) is None
    assert z.paso("encuadrando", caja_centrada(0.15), W, H, 1.0, 2.0) is not None
