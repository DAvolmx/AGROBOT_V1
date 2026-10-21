"""La anticolisión con retroceso, dentro de ActuatorController._aplicar (el punto por el que sale TODO lo que mueve el robot)."""
import time

import jetson_server as js


class _Link:
    connected = True

    def __init__(self):
        self.enviado = []

    def set_actuators(self, *a, **k):
        pass

    def send_command(self, v, w, scan):
        self.enviado.append((v, w))


def _scan(frente_m, atras_m=3.0):
    """Scan de 360 rayos con una pared a `frente_m` del LIDAR delante y otra a `atras_m` detrás; el resto lejos."""
    pts = []
    for a in range(-180, 180):
        d = 6.0
        if abs(a) <= 20:
            d = frente_m
        elif abs(abs(a) - 180) <= 20:
            d = atras_m
        pts.append({"angle_deg": float(a), "distance_m": d})
    return pts


def _preparar(frente_m, atras_m=3.0, activa=True):
    js.state.config["safety"]["freno_lidar_autonomo"] = False
    js.state.config["safety"]["anticolision_activa"] = activa
    js.state.config["control"]["pivote_en_sitio"] = True
    js.anticolision_global = js.anticolision.Anticolision()
    js.lidar_reactive.ingest_scan(_scan(frente_m, atras_m))
    link = _Link()
    ctrl = js.ActuatorController(link)
    ctrl._obj_v = ctrl._ramp_v = 0.25
    ctrl._obj_t = time.monotonic()
    return ctrl, link


def test_va_a_chocar_y_el_robot_retrocede():
    ctrl, link = _preparar(frente_m=0.28)            # 0,28 m del LIDAR - 0,185 de carrocería = 9,5 cm hasta tocar
    ctrl._aplicar(0)
    v, _ = link.enviado[-1]
    assert v < 0
    assert js.anticolision_global.estado == "retrocediendo"


def test_con_espacio_libre_sigue_avanzando():
    ctrl, link = _preparar(frente_m=2.0)
    ctrl._aplicar(0)
    assert link.enviado[-1][0] > 0


def test_apagada_no_hace_nada():
    ctrl, link = _preparar(frente_m=0.28, activa=False)
    ctrl._aplicar(0)
    assert link.enviado[-1][0] > 0


def test_sin_espacio_detras_solo_se_detiene():
    ctrl, link = _preparar(frente_m=0.28, atras_m=0.40)      # 0,40 - 0,305 = 9,5 cm detrás (menos que retroceso+margen)
    ctrl._aplicar(0)
    assert link.enviado[-1][0] == 0.0
