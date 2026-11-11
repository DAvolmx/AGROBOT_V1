import modelos_preset as mp


def test_arandano_trae_sus_filtros():
    p = mp.perfil_deteccion("agrobot_arandanos_det.pt")
    assert p["plaga_solo_en_col"] is False and "arandano_maduro" in p["clases_planta"] and p["imgsz"] == 960


def test_coles_restaura_los_de_siempre():
    p = mp.perfil_deteccion("agrobot_plagasx_det.pt")
    assert p["plaga_solo_en_col"] is True and "arandano_maduro" not in p["clases_planta"] and p["imgsz"] == 480


def test_modelo_sin_perfil_no_toca_nada():
    assert mp.perfil_deteccion("ip102_pest_yolo11s.pt") is None


def test_caja_gigante_solo_se_descarta_en_arandano():
    assert mp.perfil_deteccion("agrobot_arandanos_det.pt")["plaga_caja_max_frac_descartar"] == 0.25
    assert mp.perfil_deteccion("agrobot_plagasx_det.pt")["plaga_caja_max_frac_descartar"] == 0
