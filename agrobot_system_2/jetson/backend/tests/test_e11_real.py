"""E11 en el explorador real: predictor de colisión, barrido de corredor y regreso al inicio (sin robot).   python3 -m pytest tests/test_e11_real.py -q"""
import asyncio
import copy
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import explorador  # noqa: E402
from config_store import DEFAULTS  # noqa: E402


def _scan(obst):
    """Scan de 360 rayos: libre a 6 m salvo `obst` = [(angulo_deg, distancia_m)] ensanchados ±3°."""
    out = []
    for a in range(360):
        d = 6.0
        for ang, dist in obst:
            if min(abs(a - ang), 360 - abs(a - ang)) <= 3:
                d = dist
        out.append({"angle_deg": float(a), "distance_m": d})
    return out


def _expl(scan, pose, extra=None, acciones=None):
    cfg = copy.deepcopy(DEFAULTS["autonomia"])
    cfg.update(extra or {}, escaneo_periodo_s=9999, avance_libre_cm=40, lado_libre_cm=30, retroceso_libre_cm=30, rumbo_fusion_activo=False,
               correccion_sendero_preventiva=False, guardian_sendero_activo=False)
    acciones = acciones if acciones is not None else []

    async def escanear(_):
        pass
    d = {"adelante": 3.0, "atras": 3.0, "izquierda": 2.0, "derecha": 2.0, "cerca": None}
    e = explorador.ExploradorReactivo(
        distancias=lambda: d, get_pose=lambda: dict(pose), conducir=lambda a, m=True, *x: acciones.append(a), parar=lambda: None, escanear=escanear,
        on_event=lambda _: None, grabar_recorrido=lambda b: (True, ""), edad_scan=lambda: 0.0, get_config=lambda: cfg)
    e.scan_cb = lambda: scan
    return e, cfg, acciones


def test_el_predictor_frena_un_pulso_que_chocaria():
    e, cfg, ac = _expl(_scan([(0, 0.45)]), {"x": 0, "y": 0, "theta_deg": 0})        # algo a 45 cm del LIDAR, de frente
    assert e._conducir("adelante") is False and ac == []
    assert e.pred_rechazados["adelante"] == 1


def test_el_predictor_deja_pasar_lo_despejado_y_lo_lateral():
    e, cfg, ac = _expl(_scan([(90, 0.30)]), {"x": 0, "y": 0, "theta_deg": 0})       # algo a 30 cm a la izquierda, no en la trayectoria
    assert e._conducir("adelante") is True and ac == ["adelante"]


def test_no_retrocede_si_hay_algo_atras():
    e, cfg, ac = _expl(_scan([(180, 0.45)]), {"x": 0, "y": 0, "theta_deg": 0})
    assert e._conducir("atras") is False and ac == []


def test_gira_al_lado_contrario_si_el_pedido_esta_bloqueado():
    e, cfg, ac = _expl(_scan([(60, 0.30)]), {"x": 0, "y": 0, "theta_deg": 0})        # algo cerca a la izquierda-adelante
    assert e._conducir("izquierda") is True and ac and ac[0] != "izquierda"


def test_sin_e11_el_explorador_es_el_de_antes():
    e, cfg, ac = _expl(_scan([(0, 0.45)]), {"x": 0, "y": 0, "theta_deg": 0}, {"e11_activo": False})
    assert e._conducir("adelante") is True and ac == ["adelante"]


def test_sin_scan_no_se_mueve():
    e, cfg, ac = _expl([], {"x": 0, "y": 0, "theta_deg": 0})
    assert e._conducir("adelante") is False


def test_regreso_apunta_al_inicio_y_termina_al_llegar():
    pose = {"x": 3.0, "y": 0.0, "theta_deg": 180.0}                                    # a 3 m del inicio, mirando hacia él
    ac = []
    e, cfg, _ = _expl(_scan([]), pose, acciones=ac)
    e.activo = True
    e._inicio_xy = (0.0, 0.0)
    from planificador import Planificador
    e.plan = Planificador(centro=(0, 0))
    assert e.iniciar_regreso()[0] is True
    asyncio.run(e._paso_regreso(cfg, {}))
    assert ac and ac[-1] == "adelante"                                                 # ya mira hacia el inicio: avanza
    pose.update(theta_deg=90.0)                                                        # mirando de lado: tiene que girar
    ac.clear()
    e._ruta = None
    asyncio.run(e._paso_regreso(cfg, {}))
    assert ac and ac[-1] in ("izquierda", "derecha")
    pose.update(x=0.2, y=0.1)                                                          # a 22 cm del inicio: terminó
    assert asyncio.run(e._paso_regreso(cfg, {})) is True and e.regreso_estado == "completo" and e.activo is False


def test_regreso_sin_planificador_se_rechaza():
    e, cfg, ac = _expl(_scan([]), {"x": 1, "y": 0, "theta_deg": 0})
    e.activo = True
    assert e.iniciar_regreso()[0] is False


def test_el_barrido_no_cuenta_una_pared_pegada_al_costado():
    """Pared a 22 cm del costado, junto al cuerpo (x entre 0 y la cara frontal): NO es un obstáculo de frente (error visto el 2026-10-26)."""
    scan = [{"angle_deg": 90.0 + a, "distance_m": 0.22 / max(math.sin(math.radians(90.0 + a)), 0.2)} for a in range(-30, 31)]   # pared recta a y = 0,22
    e, cfg, ac = _expl(scan, {"x": 0, "y": 0, "theta_deg": 0})
    assert e._frente_barrido(3.0, cfg) == 3.0


def test_el_barrido_si_cuenta_lo_que_esta_delante():
    e, cfg, ac = _expl(_scan([(0, 0.60)]), {"x": 0, "y": 0, "theta_deg": 0})
    assert 0.35 < e._frente_barrido(3.0, cfg) < 0.45


def test_giro_con_pared_en_la_esquina_segura_si_pasa():
    """Pared a la izquierda, a la altura de la cara delantera (x>0, y=+0,22): al girar a la DERECHA esa esquina se aleja (pasa); a la izquierda choca."""
    scan = [{"angle_deg": math.degrees(math.atan2(0.22, x)), "distance_m": math.hypot(x, 0.22)} for x in (0.02, 0.06, 0.10, 0.14)]
    e, cfg, ac = _expl(scan, {"x": 0, "y": 0, "theta_deg": 0})
    assert e._despejado("giro", cfg, "derecha") is True
    assert e._despejado("giro", cfg, "izquierda") is False


def test_el_sonar_elige_el_hueco_del_mapa_al_bloquearse():
    """Frente tapado a 30 cm; hueco libre a +60° (izquierda): el robot gira a la izquierda los pulsos que pide el ángulo."""
    from planificador import Planificador
    from sonar_mapa import Sonar
    scan = [{"angle_deg": a % 360, "distance_m": 5.0 if abs(a - 60) <= 22 else 0.3} for a in range(-180, 180)]
    ac = []
    e, cfg, _ = _expl(scan, {"x": 0, "y": 0, "theta_deg": 0}, {"e11_predictor": False, "e11_sonar": True, "mx_grados_pulso": 30.0}, ac)
    e.plan = Planificador(centro=(0, 0))
    e.sonar = Sonar(e.plan)
    e.activo = True
    e._inicio_xy = (0.0, 0.0)

    async def go():
        t = asyncio.create_task(e._bucle())
        await asyncio.sleep(3.0)
        e.activo = False
        t.cancel()
        try:
            await t
        except asyncio.CancelledError:
            pass
    d = {"adelante": 0.1, "atras": 3.0, "izquierda": 2.0, "derecha": 2.0, "cerca": None}
    e._distancias = lambda: d
    asyncio.run(go())
    assert ac and ac[0] == "izquierda" and ac[:2] == ["izquierda", "izquierda"]
