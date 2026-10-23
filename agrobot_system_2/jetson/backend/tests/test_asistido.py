import json
from asistido import Asistido, clasificar


def test_clasificar():
    assert clasificar(0.2, 0, 0, 0) == "freno"
    assert clasificar(0.2, 0, -0.1, 0) == "freno"
    assert clasificar(0, 0, 0.2, 0) == "empuje"
    assert clasificar(0.2, 0, 0.2, 0) == "avance"
    assert clasificar(0.1, 0.5, 0.1, -0.8) == "giro_contrario"
    assert clasificar(0.1, 0.5, 0.0, 0.8) == "giro_mismo"
    assert clasificar(0.2, 0, 0.0, 0.8) == "giro_extra"


def _a(tmp_path):
    t = [100.0]
    a = Asistido(tmp_path, reloj=lambda: t[0])
    return a, t


def test_sin_toma_no_registra(tmp_path):
    a, t = _a(tmp_path)
    assert a.muestra(0.2, 0, None, None) is None
    assert not list(tmp_path.glob("*.jsonl"))


def test_registra_limita_frecuencia_y_persiste(tmp_path):
    a, t = _a(tmp_path)
    a.tomar("exploracion", auto_cmd=(0.2, 0.0))
    ev = a.muestra(0, 0, {"x": 1, "y": 2, "theta_deg": 90}, {"adelante": 0.4, "izquierda": None})
    assert ev["tipo"] == "freno" and ev["dist_cm"] == {"adelante": 40, "izquierda": None}
    assert a.muestra(0, 0, None, None) is None            # <0,2 s: descartada
    t[0] += 0.3
    assert a.muestra(0.2, 0, None, None)["tipo"] == "avance"
    filas = [json.loads(l) for f in tmp_path.glob("*.jsonl") for l in f.read_text().splitlines()]
    assert len(filas) == 2 and filas[0]["modo"] == "exploracion"
    assert a.soltar() == "exploracion" and not a.activa


def test_sugerencias_y_resumen(tmp_path):
    a, t = _a(tmp_path)
    a.tomar("patrulla", auto_cmd=(0.2, 0.0))
    for i in range(5):                                     # 5 frenos con algo cerca
        t[0] += 0.3
        a.muestra(0, 0, None, {"adelante": 0.5})
    cfg = {"slam": {"margen_parada_cm": 26}, "autonomia": {"escala_velocidad": 0.5}}
    s = a.sugerencias(cfg)
    assert [x["id"] for x in s] == ["margen_mas"] and s[0]["propuesto"] == 31
    r = a.resumen()
    assert r["total"] == 5 and r["por_tipo"]["freno"]["n"] == 5 and r["por_modo"] == {"patrulla": 5}


def test_pocos_eventos_no_sugieren(tmp_path):
    a, t = _a(tmp_path)
    a.tomar("exploracion", auto_cmd=(0.2, 0))
    a.muestra(0, 0, None, {"adelante": 0.3})
    assert a.sugerencias({"slam": {}, "autonomia": {}}) == []


def test_pose_se_registra_sin_intervencion(tmp_path):
    a, t = _a(tmp_path)
    a.evento_pose(1, 2, 90, {"x": 0, "y": 0, "theta_deg": 0})
    assert a.resumen()["por_tipo"]["pose"]["n"] == 1


def test_caduca_si_se_olvida(tmp_path):
    a, t = _a(tmp_path)
    a.tomar("patrulla", auto_cmd=(0.2, 0))
    t[0] += 301
    assert a.muestra(0, 0, None, None) is None and not a.activa


def test_segundos_sin_orden_y_reanudar_automatico(tmp_path):
    a, t = _a(tmp_path)
    a.tomar("exploracion", auto_cmd=(0.2, 0))
    t[0] += 1.0
    assert abs(a.segundos_sin_orden() - 1.0) < 1e-6
    a.muestra(0.1, 0, None, None)                 # una orden manual reinicia la cuenta
    assert a.segundos_sin_orden() == 0
    t[0] += 0.05
    a.muestra(0.1, 0, None, None)                 # descartada por frecuencia, pero cuenta como actividad
    assert abs(a.segundos_sin_orden()) < 1e-6


def test_aprende_solo_lo_cauto_y_no_repite(tmp_path):
    a, t = _a(tmp_path)
    a.tomar("patrulla", auto_cmd=(0.2, 0.0))
    for _ in range(5):
        t[0] += 0.3
        a.muestra(0, 0, None, {"adelante": 0.5})
    cfg = {"slam": {"margen_parada_cm": 26}, "autonomia": {}}
    (s,) = a.auto_aplicables(cfg)
    assert s["id"] == "margen_mas"
    a.marcar_aprendido(s)
    assert a.sugerencias(cfg) == [] and a.auto_aplicables(cfg) == []     # esas correcciones ya cuentan como aprendidas
    assert Asistido(tmp_path, reloj=lambda: t[0]).aprendido_hasta == a.aprendido_hasta   # y sobrevive al reinicio


def test_no_aprende_solo_lo_que_lo_hace_menos_cauto(tmp_path):
    a, t = _a(tmp_path)
    a.tomar("patrulla", auto_cmd=(0.0, 0.0))
    for _ in range(5):
        t[0] += 0.3
        a.muestra(0.2, 0, None, {"adelante": 0.9})
    cfg = {"slam": {"margen_parada_cm": 26}, "autonomia": {}}
    assert [x["id"] for x in a.sugerencias(cfg)] == ["margen_menos"] and a.auto_aplicables(cfg) == []
