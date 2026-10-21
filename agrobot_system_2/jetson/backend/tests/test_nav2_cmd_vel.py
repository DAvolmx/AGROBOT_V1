"""Nav2 manda motores SOLO con una meta suya en curso, recortado, y sin pisar al operador."""
import types

import jetson_server as js


class _Ctrl:
    def __init__(self):
        self.enviado = []

    def send_command(self, v, w, scan):
        self.enviado.append((v, w))


def _montar(monkeypatch, meta="en_curso", conductor_activo=False, solo_manual=False):
    ctrl = _Ctrl()
    monkeypatch.setattr(js, "actuator_proto", ctrl)
    monkeypatch.setattr(js.ros_bridge, "estado_meta", lambda: meta)
    monkeypatch.setattr(js, "conductor", types.SimpleNamespace(activo=conductor_activo))
    js.state.config["control"]["solo_manual"] = solo_manual
    js.state.config["control"]["nav2_v_max_mps"] = 0.25
    js.state.config["control"]["nav2_w_max_rps"] = 0.6
    return ctrl


def test_con_meta_en_curso_pasa_y_se_recorta(monkeypatch):
    c = _montar(monkeypatch)
    js._ros_on_cmd_vel(0.4, 1.0)
    js._ros_on_cmd_vel(-0.2, -1.2)
    js._ros_on_cmd_vel(0.1, 0.2)
    assert c.enviado == [(0.25, 0.6), (-0.2, -0.6), (0.1, 0.2)]


def test_sin_meta_no_manda_nada(monkeypatch):
    for meta in ("sin_meta", "alcanzada", "rechazada"):
        c = _montar(monkeypatch, meta=meta)
        js._ros_on_cmd_vel(0.3, 0.3)
        assert c.enviado == [], meta


def test_el_operador_manda_y_el_perfil_demo_bloquea(monkeypatch):
    c = _montar(monkeypatch, conductor_activo=True)
    js._ros_on_cmd_vel(0.3, 0.0)
    assert c.enviado == []
    c = _montar(monkeypatch, solo_manual=True)
    js._ros_on_cmd_vel(0.3, 0.0)
    assert c.enviado == []
    js.state.config["control"]["solo_manual"] = False


def test_la_meta_enviada_tambien_cuenta(monkeypatch):
    c = _montar(monkeypatch, meta="enviada")
    js._ros_on_cmd_vel(0.1, 0.0)
    assert c.enviado == [(0.1, 0.0)]
