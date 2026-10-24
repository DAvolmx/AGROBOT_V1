"""Guardián del sendero: errores, decisión, memoria por correlación de sensores y registro de lo hecho (sin robot).

    python3 -m pytest tests/test_guardian_sendero.py -q
"""
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from guardian_sendero import GuardianSendero, MemoriaCorrecciones, firma  # noqa: E402

SENS = {"lidar_izquierda_m": 0.5, "lidar_derecha_m": 0.5, "lidar_adelante_m": 2.0, "lidar_atras_m": 1.5,
        "sendero_lateral_m": 0.0, "sendero_confianza": 0.8, "imu_gz": 0.0, "imu_roll_deg": 0.0, "imu_pitch_deg": 0.0}


def _g(**kw):
    g = GuardianSendero(MemoriaCorrecciones(None), log_dir=None, **kw)
    g.fijar_recta({"x": 0.0, "y": 0.0, "theta_deg": 0.0})
    return g


def test_signos_de_los_errores():
    g = _g()
    e_y, e_h = g.errores({"x": 1.0, "y": 0.2, "theta_deg": 10.0})
    assert e_y > 0.19 and e_h > 0.17                     # a la izquierda y apuntando a la izquierda


def test_a_la_izquierda_corrige_a_la_derecha():
    d = _g().decidir({"x": 1.0, "y": 0.25, "theta_deg": 0.0}, SENS)
    assert d["accion"] == "derecha" and d["origen"] == "ley"


def test_dentro_de_la_tolerancia_no_corrige():
    assert _g().decidir({"x": 1.0, "y": 0.02, "theta_deg": 1.0}, SENS) is None


def test_no_gira_hacia_una_pared_pegada():
    s = dict(SENS, lidar_derecha_m=0.10)
    assert _g().decidir({"x": 1.0, "y": 0.25, "theta_deg": 0.0}, s) is None


def test_aprende_cuanto_gira_por_pulso():
    g = _g()
    pose = {"x": 1.0, "y": 0.25, "theta_deg": 0.0}
    d = g.decidir(pose, SENS)
    g.iniciar(d, pose, SENS)
    g.terminar({"x": 1.05, "y": 0.24, "theta_deg": -9.0 * d["escala"]}, SENS)
    assert g.efecto["derecha"] > 5.0                         # giró más que los 5° supuestos


def test_la_memoria_recomienda_lo_que_funciono_en_una_situacion_parecida():
    g = _g()
    x, w = firma(SENS, 0.25, 0.0)
    for _ in range(5):
        g.memoria.agregar(x, "derecha", 0.7, 0.8)
    d = g.decidir({"x": 1.0, "y": 0.25, "theta_deg": 0.0}, SENS)
    assert d["origen"] == "memoria" and abs(d["escala"] - 0.7) < 0.05 and "parecido" in d["explicacion"]


def test_la_memoria_no_repite_lo_que_empeoro():
    g = _g()
    x, w = firma(SENS, 0.25, 0.0)
    for _ in range(5):
        g.memoria.agregar(x, "derecha", 1.2, -0.6)
    assert g.decidir({"x": 1.0, "y": 0.25, "theta_deg": 0.0}, SENS)["origen"] == "ley"


def test_la_situacion_distinta_no_usa_esa_memoria():
    g = _g()
    x, w = firma(dict(SENS, lidar_izquierda_m=0.2, lidar_derecha_m=1.8, imu_roll_deg=12.0, sendero_lateral_m=0.4), 0.25, 0.0)
    for _ in range(5):
        g.memoria.agregar(x, "derecha", 0.7, 0.8)
    assert g.decidir({"x": 1.0, "y": 0.25, "theta_deg": 0.0}, SENS)["origen"] == "ley"


def test_apunta_la_correccion_real(tmp_path):
    g = GuardianSendero(MemoriaCorrecciones(None), log_dir=tmp_path)
    g.fijar_recta({"x": 0, "y": 0, "theta_deg": 0})
    pose = {"x": 1.0, "y": 0.25, "theta_deg": 0.0}
    d = g.decidir(pose, SENS)
    g.iniciar(d, pose, SENS)
    reg = g.terminar({"x": 1.1, "y": 0.15, "theta_deg": -3.0}, SENS)
    linea = json.loads(next(tmp_path.glob("guardian_*.jsonl")).read_text().splitlines()[0])
    assert linea["accion"] == "derecha" and linea["mejora"] > 0 and "antes" in linea and "despues" in linea and reg["giro_real_deg"] == -3.0


def test_el_explorador_usa_el_guardian_y_apunta(tmp_path):
    import asyncio
    import copy
    import explorador
    from config_store import DEFAULTS
    cfg = copy.deepcopy(DEFAULTS["autonomia"])
    cfg.update(escaneo_periodo_s=9999, avance_libre_cm=40, guardian_sendero_activo=True, rumbo_fusion_activo=False,
               correccion_sendero_preventiva=False)
    acciones = []
    d = {"adelante": 2.0, "atras": 1.5, "izquierda": 0.5, "derecha": 0.5, "cerca": None}

    async def escanear(_):
        pass
    e = explorador.ExploradorReactivo(
        distancias=lambda: d, get_pose=lambda: {"x": 1.0, "y": 0.3, "theta_deg": 0.0},
        conducir=lambda a, m=True, *x: acciones.append(a), parar=lambda: None, escanear=escanear, on_event=lambda _: None,
        grabar_recorrido=lambda b: (True, ""), edad_scan=lambda: 0.0, get_config=lambda: cfg)
    e.guardian = GuardianSendero(MemoriaCorrecciones(None), log_dir=tmp_path)
    e.guardian.fijar_recta({"x": 0.0, "y": 0.0, "theta_deg": 0.0})
    e.sensores_guardian = lambda: dict(SENS)

    async def go():
        e.activo = True
        e._ultimo_escaneo = 1e12
        t = asyncio.create_task(e._bucle())
        await asyncio.sleep(2.6)
        e.activo = False
        t.cancel()
        try:
            await t
        except asyncio.CancelledError:
            pass
    asyncio.run(go())
    assert acciones and acciones[0] == "derecha"
    assert list(tmp_path.glob("guardian_*.jsonl"))
