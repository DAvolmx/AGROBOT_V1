"""Maniobra de esquina del explorador (man_giro_pulsos / man_retroceso_cm / man_atras_pulsos), sin robot.

    python3 -m pytest tests/test_explorador_esquina.py -q
"""
import asyncio
import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import explorador  # noqa: E402
from config_store import DEFAULTS  # noqa: E402


def _correr(extra, frente=0.30, atras=1.5, ticks=3.0):
    cfg = copy.deepcopy(DEFAULTS["autonomia"])
    cfg.update(extra, escaneo_periodo_s=9999, avance_libre_cm=40, lado_libre_cm=30, retroceso_libre_cm=30,
               arco_frontal_si_giro_pasivo_bloqueado=False, retroceso_previo_giro_eje_pasivo=False,
               correccion_sendero_preventiva=False, rumbo_fusion_activo=False)
    acciones = []
    d = {"adelante": frente, "atras": atras, "izquierda": 2.0, "derecha": 0.25, "cerca": None}

    async def escanear(_):
        pass

    e = explorador.ExploradorReactivo(
        distancias=lambda: d, get_pose=lambda: {"x": 0, "y": 0, "theta_deg": 0}, conducir=lambda a, m=True, *x: acciones.append(a),
        parar=lambda: None, escanear=escanear, on_event=lambda _: None, grabar_recorrido=lambda b: (True, ""),
        edad_scan=lambda: 0.0, get_config=lambda: cfg)

    async def go():
        e.activo = True
        e._ultimo_escaneo = 1e12
        t = asyncio.create_task(e._bucle())
        await asyncio.sleep(ticks)
        e.activo = False
        t.cancel()
        try:
            await t
        except asyncio.CancelledError:
            pass
    asyncio.run(go())
    return acciones


def test_por_defecto_un_solo_giro():
    a = _correr({}, ticks=1.5)
    assert a.count("izquierda") >= 1 and "atras" not in a


def test_varios_pulsos_de_giro():
    a = _correr({"man_giro_pulsos": 3}, ticks=3.6)
    assert a[:3] == ["izquierda"] * 3


def test_retrocede_antes_de_girar():
    a = _correr({"man_retroceso_cm": 35, "man_atras_pulsos": 2, "man_giro_pulsos": 1}, ticks=3.6)
    assert a[:3] == ["atras", "atras", "izquierda"]


def test_no_retrocede_sin_sitio_detras():
    a = _correr({"man_retroceso_cm": 35, "man_atras_pulsos": 2}, atras=0.1, ticks=1.5)
    assert "atras" not in a[:2]
