"""Autonomía robusta (2026-11-02): giro hasta un rumbo, atasco por pulso con salida escalonada, fuerza de giro aprendida e integración en el explorador.

    python3 -m pytest tests/test_autonomia_robusta.py -q
"""
import asyncio
import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import explorador  # noqa: E402
from atasco import DetectorAtasco  # noqa: E402
from config_store import DEFAULTS  # noqa: E402
from giro_cerrado import GiroCerrado  # noqa: E402
from giro_objetivo import GiroObjetivo, dif_deg  # noqa: E402
from pulse_drive import ConductorPulsos, W_GIRO_BASE, W_GIRO_MAX  # noqa: E402


# ---------------------------------------------------------------- giro_objetivo
def _girar(obj, theta0=0.0, rinde=1.0, limite=None):
    """Simula: cada tramo gira `rinde` × lo pedido (suelo que se come el giro). `limite` = el robot deja de girar del todo tras ese total."""
    g = GiroObjetivo(obj)
    th, total = theta0, 0.0
    for _ in range(20):
        o = g.siguiente(th)
        if o["tipo"] == "fin":
            return g, th
        paso = o["grados"] * rinde
        if limite is not None and abs(total + paso) > limite:
            paso = 0.0
        total += paso
        th = (th + paso) % 360
    return g, th


def test_dif_deg():
    assert dif_deg(10, 350) == 20
    assert dif_deg(350, 10) == -20
    assert dif_deg(180, 0) == 180


def test_alcanza_el_rumbo_en_un_tramo():
    g, th = _girar(20.0)
    assert g.motivo == "alcanzado" and abs(dif_deg(20.0, th)) <= 5.0 and g.tramos == 1


def test_suelo_que_se_come_el_giro_repite_tramos():
    """Exterior 27-oct: los pulsos daban la cuarta parte de lo pedido. Con tramos medidos llega igual."""
    g, th = _girar(90.0, rinde=0.5)
    assert g.motivo == "alcanzado" and abs(dif_deg(90.0, th)) <= 5.0 and g.tramos > 2


def test_se_pasa_y_vuelve():
    g, th = _girar(-60.0, theta0=10.0, rinde=1.4)
    assert g.motivo == "alcanzado" and abs(dif_deg(300.0, th)) <= 5.0


def test_cruza_el_cero():
    g, th = _girar(350.0, theta0=20.0)
    assert g.motivo == "alcanzado" and abs(dif_deg(350.0, th)) <= 5.0


def test_sin_avance_cuando_no_gira():
    g, _ = _girar(90.0, limite=10.0)
    assert g.motivo == "sin avance"


# ---------------------------------------------------------------- atasco
def _p(x, y=0.0, th=0.0):
    return {"x": x, "y": y, "theta_deg": th}


def test_pulsos_rectos_sin_moverse_es_atasco():
    a = DetectorAtasco()
    for _ in range(3):
        a.pulso("adelante", _p(1.0))
    assert not a.atascado                      # el tercero aún no se ha medido
    a.pulso("adelante", _p(1.01))
    assert a.atascado and a.declarados == 1


def test_un_pulso_que_avanza_reinicia_la_cuenta():
    a = DetectorAtasco()
    a.pulso("adelante", _p(0.0))
    a.pulso("adelante", _p(0.0))
    a.pulso("adelante", _p(0.20))
    a.pulso("adelante", _p(0.20))
    a.pulso("adelante", _p(0.20))
    assert not a.atascado and a.fallos_recto == 2


def test_giros_muertos():
    a = DetectorAtasco(fallos_giro=3)
    for k in range(4):
        a.pulso("izquierda", _p(0.0, th=0.5 * k))
    assert a.atascado and "giros" in a.motivo


def test_escalado_y_rendirse():
    a = DetectorAtasco()
    niveles = []
    for _ in range(4):
        a.giro_sin_avance(_p(0.0))
        plan = a.plan(lado_libre="izquierda", hueco_atras=True, hueco_delante=True)
        niveles.append(plan)
    assert niveles[0] == [("atras",), ("girar", 30.0)]
    assert niveles[1] == [("atras",), ("atras",), ("girar", -60.0)]
    assert [p[0] for p in niveles[2]] == ["adelante", "atras", "adelante", "atras", "girar"]
    assert niveles[3][0][0] == "rendirse"


def test_progreso_vuelve_al_nivel_1():
    a = DetectorAtasco(progreso_m=0.5)
    a.giro_sin_avance(_p(0.0))
    a.plan(lado_libre="derecha", hueco_atras=True, hueco_delante=True)
    a.giro_sin_avance(_p(0.0))
    a.plan(lado_libre="derecha", hueco_atras=True, hueco_delante=True)
    assert a.nivel == 2
    a.pulso("adelante", _p(0.0))
    a.pulso("adelante", _p(0.8))               # salió 80 cm del sitio del atasco
    assert a.nivel == 0
    a.giro_sin_avance(_p(0.8))
    assert a.plan(lado_libre="derecha", hueco_atras=True, hueco_delante=True)[-1] == ("girar", -30.0)


def test_sin_hueco_detras_no_retrocede():
    a = DetectorAtasco()
    a.giro_sin_avance(_p(0.0))
    assert a.plan(lado_libre="izquierda", hueco_atras=False, hueco_delante=True) == [("girar", 30.0)]


# ---------------------------------------------------------------- fuerza aprendida
def _cond():
    return ConductorPulsos(send_command=lambda *a: None, emergency_stop=lambda: None)


def test_fuerza_sube_si_el_lazo_tuvo_que_subirla():
    c = _cond()
    g = GiroCerrado(20.0, w_rps=W_GIRO_BASE)
    g.w, g.motivo, g.integ = 1.0, "alcanzado", 20.0
    c.aprender_fuerza(g)
    assert c.w_giro[1.0] > W_GIRO_BASE and c.w_giro[-1.0] == W_GIRO_BASE


def test_fuerza_al_maximo_si_no_giro_nada():
    c = _cond()
    g = GiroCerrado(-20.0)
    g.w, g.motivo, g.integ = 1.1, "tiempo", -1.0
    c.aprender_fuerza(g)
    assert c.w_giro[-1.0] == 1.1


def test_fuerza_baja_poco_a_poco_en_suelo_duro():
    c = _cond()
    c.w_giro[1.0] = W_GIRO_MAX
    for _ in range(10):
        g = GiroCerrado(20.0, w_rps=c.w_giro[1.0])
        g.motivo, g.integ = "alcanzado", 20.0
        c.aprender_fuerza(g)
    assert c.w_giro[1.0] < W_GIRO_MAX


# ---------------------------------------------------------------- integración en el explorador
def _explorador(extra, d, pose, conducir, girar_cb=None):
    cfg = copy.deepcopy(DEFAULTS["autonomia"])
    cfg.update(e11_activo=False, escaneo_periodo_s=9999, avance_libre_cm=40, lado_libre_cm=30, retroceso_libre_cm=30,
               arco_frontal_si_giro_pasivo_bloqueado=False, retroceso_previo_giro_eje_pasivo=False,
               correccion_sendero_preventiva=False, rumbo_fusion_activo=False)
    cfg.update(extra)
    eventos = []

    async def escanear(_):
        pass

    e = explorador.ExploradorReactivo(
        distancias=lambda: d, get_pose=lambda: dict(pose), conducir=conducir, parar=lambda: None, escanear=escanear,
        on_event=eventos.append, grabar_recorrido=lambda b: (True, ""), edad_scan=lambda: 0.0, get_config=lambda: cfg)
    e.girar_a_rumbo_cb = girar_cb
    e._seguidor = _SeguidorMudo()
    e._pausa_pulso = _sin_pausa
    return e, eventos


class _SeguidorMudo:
    """Sin cámara: el seguidor del sendero no mide nada."""
    def __getattr__(self, _nombre):
        return lambda *a, **k: None


async def _sin_pausa(*_a, **_k):
    await asyncio.sleep(0.01)


def _correr(e, segundos):
    async def go():
        e.activo = True
        e._ultimo_escaneo = 1e12
        t = asyncio.create_task(e._bucle())
        await asyncio.sleep(segundos)
        e.activo = False
        t.cancel()
        try:
            await t
        except asyncio.CancelledError:
            pass
    asyncio.run(go())


def test_esquina_gira_hasta_un_rumbo():
    pose = {"x": 0.0, "y": 0.0, "theta_deg": 10.0}
    pedidos = []

    async def girar(rumbo):
        pedidos.append(rumbo)
        pose["theta_deg"] = rumbo % 360
        return {"motivo": "alcanzado", "girado_deg": 30.0, "error_deg": 0.0, "tramos": 1}

    d = {"adelante": 0.30, "atras": 1.5, "izquierda": 2.0, "derecha": 0.25, "cerca": None}
    e, ev = _explorador({"giro_objetivo_activo": True}, d, pose, lambda *a, **k: None, girar)
    _correr(e, 0.4)
    assert pedidos and abs(pedidos[0] - 40.0) < 1e-6                 # 10° + 30° a la izquierda (sin scan: 30° al lado elegido)
    assert any(x.get("evento") == "giro_rumbo" for x in ev)


def test_sin_imu_usa_los_pulsos_de_siempre():
    pose = {"x": 0.0, "y": 0.0, "theta_deg": 0.0}
    acciones = []

    async def girar(_):
        return None

    d = {"adelante": 0.30, "atras": 1.5, "izquierda": 2.0, "derecha": 0.25, "cerca": None}
    e, _ = _explorador({"giro_objetivo_activo": True}, d, pose, lambda a, *x, **k: acciones.append(a), girar)
    _correr(e, 0.4)
    assert "izquierda" in acciones


def test_atasco_escalado_llega_a_rendirse_y_espera():
    """Frente libre pero la pose no se mueve nunca (rueda patinando): sube de nivel y acaba rindiéndose en espera, sin seguir empujando."""
    pose = {"x": 0.0, "y": 0.0, "theta_deg": 0.0}
    acciones = []
    d = {"adelante": 3.0, "atras": 1.5, "izquierda": 2.0, "derecha": 2.0, "cerca": None}
    e, ev = _explorador({"atasco_escalado": True}, d, pose, lambda a, *x, **k: acciones.append(a))
    _correr(e, 6.0)
    niveles = [x["nivel"] for x in ev if x.get("evento") == "desatasco"]
    assert niveles[:3] == [1, 2, 3]
    assert any(x.get("evento") == "atasco_sin_salida" for x in ev)
    assert "atasco" in e.bloqueos
    assert e.estado.startswith("en espera")
    n = len(acciones)
    _correr(e, 0.6)                              # bloqueado: no manda nada más
    assert len(acciones) == n


def test_bloqueo_de_localizacion_detiene_y_libera():
    pose = {"x": 0.0, "y": 0.0, "theta_deg": 0.0}
    acciones = []
    d = {"adelante": 3.0, "atras": 1.5, "izquierda": 2.0, "derecha": 2.0, "cerca": None}
    e, _ = _explorador({}, d, pose, lambda a, *x, **k: acciones.append(a))
    e.bloqueos["localizacion"] = "localización perdida"
    _correr(e, 0.6)
    assert acciones == [] and "localización" in e.estado
    e.bloqueos.clear()
    _correr(e, 0.6)
    assert "adelante" in acciones


def test_continuar_tras_el_operador_quita_el_bloqueo_de_atasco():
    e, _ = _explorador({}, {}, {"x": 0, "y": 0, "theta_deg": 0}, lambda *a, **k: None)
    e.activo = True
    e.bloqueos["atasco"] = "atascado"
    e.pausar()
    e.continuar()
    assert "atasco" not in e.bloqueos


def test_girar_a_rumbo_bucle_compartido():
    """giro_objetivo.girar_a_rumbo (backend y simulador): tramos hasta el rumbo con un conductor que rinde el 70 % de lo pedido."""
    import asyncio
    from giro_objetivo import girar_a_rumbo

    class Conductor:
        def __init__(self):
            self.activo, self.theta, self.w_giro, self.pedidos = False, 0.0, {1.0: 0.6, -1.0: 0.6}, []

        def girar_grados(self, grados, get_gyro, w_rps=None, t_max_s=2.0):
            self.pedidos.append(grados)
            self.theta += 0.7 * grados
            return True

    c = Conductor()
    pose = lambda: {"x": 0.0, "y": 0.0, "theta_deg": c.theta}
    r = asyncio.run(girar_a_rumbo(c, 60.0, pose, lambda: 0.0, {"giro_objetivo_tol_deg": 6.0, "giro_objetivo_tramo_deg": 30.0}))
    assert r["motivo"] == "alcanzado" and abs(r["error_deg"]) <= 6.0 and len(c.pedidos) >= 3
    assert asyncio.run(girar_a_rumbo(c, 60.0, pose, lambda: None, {})) is None          # sin IMU: los pulsos de siempre


def test_girar_a_rumbo_para_si_la_pose_salta():
    import asyncio
    from giro_objetivo import girar_a_rumbo

    class Conductor:
        def __init__(self):
            self.activo, self.theta, self.w_giro, self.n, self.ultimo_giro = False, 0.0, {1.0: 0.6, -1.0: 0.6}, 0, None

        def girar_grados(self, grados, get_gyro, w_rps=None, t_max_s=2.0):
            self.n += 1
            self.theta += grados + (-50.0 if self.n == 1 else 0.0)      # el SLAM cierra un bucle de -50° en mitad del primer tramo
            self.ultimo_giro = {"girado_deg": grados, "final_deg": grados}
            return True

    c = Conductor()
    r = asyncio.run(girar_a_rumbo(c, 90.0, lambda: {"theta_deg": c.theta}, lambda: 0.0, {}))
    assert r["motivo"] == "pose saltó" and c.n == 1


def test_atasco_sin_medida_fisica_no_se_rinde():
    """El criterio de 25 s («la pose no cambió») no es un atasco físico: tras el nivel 3 vuelve al 1 en vez de rendirse."""
    from atasco import DetectorAtasco
    d = DetectorAtasco()
    pose = {"x": 0.0, "y": 0.0, "theta_deg": 0.0}
    niveles = []
    for _ in range(5):
        d._declarar(pose, "la pose no cambió 20 cm en 25 s")
        plan = d.plan(lado_libre="izquierda", hueco_atras=True, hueco_delante=True)
        assert plan[0][0] != "rendirse"
        niveles.append(d.nivel)
    assert niveles == [1, 2, 3, 1, 2]


def test_atasco_fisico_se_rinde_tras_tres_niveles():
    from atasco import DetectorAtasco
    d = DetectorAtasco()
    pose = {"x": 0.0, "y": 0.0, "theta_deg": 0.0}
    for _ in range(3):
        d.giro_sin_avance(pose)
        assert d.plan(lado_libre="derecha", hueco_atras=True, hueco_delante=True)[0][0] != "rendirse"
    d.giro_sin_avance(pose)
    assert d.plan(lado_libre="derecha", hueco_atras=True, hueco_delante=True)[0][0] == "rendirse"
