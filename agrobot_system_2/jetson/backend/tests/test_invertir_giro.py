"""control.invertir_giro: el giro se niega SOLO en el borde hacia la C3 (todo lo que mueve el robot pasa por ahí)."""
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


def _enviar(invertir, omega, v=0.0):
    link = _Link()
    ctrl = js.ActuatorController(link)
    js.state.config["control"]["invertir_giro"] = invertir
    js.state.config["safety"]["freno_lidar_autonomo"] = False
    js.state.config["control"]["pivote_en_sitio"] = True      # otros tests lo dejan en False: sin v, el giro se anularía
    ctrl._obj_v, ctrl._obj_omega = v, omega
    ctrl._ramp_v, ctrl._ramp_omega = v, omega
    ctrl._obj_t = time.monotonic()
    ctrl._aplicar(0)
    return link.enviado[-1], ctrl


def test_por_defecto_vale_true_en_la_configuracion_guardada():
    from config_store import load
    assert load()["control"]["invertir_giro"] is True


def test_invertido_niega_omega_pero_no_v():
    (v, w), _ = _enviar(True, 1.0, v=0.2)
    assert w < 0 and abs(v - 0.2) < 1e-9


def test_izquierda_y_derecha_se_cruzan_y_sin_invertir_no_cambian():
    (_, wi), _ = _enviar(True, +1.0)       # izquierda lógica (omega>0, convención ROS)
    (_, wd), _ = _enviar(True, -1.0)
    assert wi < 0 < wd
    (_, wi), _ = _enviar(False, +1.0)
    (_, wd), _ = _enviar(False, -1.0)
    assert wi > 0 > wd


def test_la_odometria_sigue_con_el_giro_logico(monkeypatch):
    """pose_estimator recibe el omega LÓGICO: con el giro invertido en el borde, lo físico coincide con lo lógico."""
    vistos = []
    monkeypatch.setattr(js.pose_estimator, "on_command", lambda v, w: vistos.append(w))
    (_, w_c3), _ = _enviar(True, 1.0)
    assert vistos and vistos[-1] > 0 and w_c3 < 0


def test_actualizar_actuadores_tambien_respeta_la_inversion():
    (_, _), ctrl = _enviar(True, 1.0)
    ctrl.link.enviado.clear()
    ctrl.send_actuator_update()
    assert ctrl.link.enviado[-1][1] < 0


def test_las_distancias_del_lidar_restan_la_carroceria_correcta_a_cada_cara():
    """El LIDAR no está en el centro: a 18,5 cm de la cara delantera y a 30,5 cm de la trasera (croquis 2026-10-20)."""
    from slam import LidarReactive
    cfg = {"robot_largo_cm": 49, "robot_ancho_cm": 38, "cono_frontal_deg": 40, "cono_lateral_deg": 60,
           "lidar_adelante_cm": 18.5, "lidar_atras_cm": 30.5}
    lid = LidarReactive()
    lid.ingest_scan([{"angle_deg": 0.0, "distance_m": 1.0}, {"angle_deg": 180.0, "distance_m": 1.0}])
    d = lid.distancias(cfg)
    assert abs(d["adelante"] - 0.815) < 1e-6          # 1,00 - 0,185
    assert abs(d["atras"] - 0.695) < 1e-6             # 1,00 - 0,305
    cfg_sim = {k: v for k, v in cfg.items() if not k.startswith("lidar_")}
    lid2 = LidarReactive()
    lid2.ingest_scan([{"angle_deg": 0.0, "distance_m": 1.0}, {"angle_deg": 180.0, "distance_m": 1.0}])
    d2 = lid2.distancias(cfg_sim)                     # sin las claves nuevas: simétrico como antes
    assert abs(d2["adelante"] - 0.755) < 1e-6 and abs(d2["atras"] - 0.755) < 1e-6
