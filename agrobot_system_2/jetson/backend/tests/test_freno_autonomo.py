"""El freno por LIDAR actúa SOLO cuando el robot va solo (exploración/patrulla/misión), no en manual."""
import time
import types

import jetson_server as js


class _Link:
    connected = True

    def __init__(self):
        self.enviado = []

    def set_actuators(self, *a, **k):
        pass

    def send_command(self, v, w, scan):
        self.enviado.append((v, w))


def _preparar(autonomo, dist_frente_m, scan_fresco=True):
    link = _Link()
    ctrl = js.ActuatorController(link)
    js.explorador = types.SimpleNamespace(activo=autonomo)
    js.state.mission_active = False
    js.state.config["safety"]["freno_lidar_autonomo"] = True
    js.state.config["control"]["pivote_en_sitio"] = False
    pts = [{"angle_deg": float(a), "distance_m": dist_frente_m} for a in range(-5, 6)] if dist_frente_m else []
    js.lidar_reactive.ingest_scan(pts)
    if not scan_fresco:
        js.lidar_reactive.last_scan_at = time.monotonic() - 5.0
    ctrl._obj_v, ctrl._obj_omega = 0.3, 0.0
    ctrl._ramp_v = 0.3
    ctrl._obj_t = time.monotonic()
    return ctrl, link


def _ultimo_v(autonomo, dist, **kw):
    ctrl, link = _preparar(autonomo, dist, **kw)
    ctrl._aplicar(0)
    return link.enviado[-1][0]


def test_autonomo_frena_ante_un_obstaculo_cercano():
    assert _ultimo_v(True, 0.45) == 0.0            # 0,45 m al centro -> ~0,25 m libres < umbral


def test_autonomo_avanza_con_camino_libre():
    assert _ultimo_v(True, 3.0) > 0.0


def test_manual_no_se_toca_aunque_haya_obstaculo():
    assert _ultimo_v(False, 0.45) > 0.0


def test_autonomo_sin_datos_frescos_del_lidar_no_avanza():
    assert _ultimo_v(True, 3.0, scan_fresco=False) == 0.0
    assert _ultimo_v(False, 3.0, scan_fresco=False) > 0.0     # en manual sigue igual que antes
