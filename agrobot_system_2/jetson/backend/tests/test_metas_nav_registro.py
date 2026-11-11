"""2026-11-11: cada meta de Nav2 deja un registro con su resultado (antes se perdía)."""
import collections
import types

import pytest

import ros_bridge

pytestmark = pytest.mark.skipif(not ros_bridge.ROS_AVAILABLE, reason="necesita rclpy (correr con ROS cargado)")


class _Futuro:
    def __init__(self, valor):
        self.valor, self.cbs = valor, []

    def result(self):
        return self.valor

    def add_done_callback(self, cb):
        self.cbs.append(cb)

    def resolver(self):
        for cb in self.cbs:
            cb(self)


class _Handle:
    accepted = True

    def __init__(self):
        self.res = _Futuro(types.SimpleNamespace(status=None))

    def get_result_async(self):
        return self.res

    def cancel_goal_async(self):
        pass


def _nodo(poses):
    N = ros_bridge._BridgeNode
    n = types.SimpleNamespace(_meta_id=0, _meta_reg=None, _goal_handle=None, estado_meta="sin_meta",
                              metas_terminadas=collections.deque(), envios=[])
    n.pose_en_mapa = lambda: poses.pop(0) if poses else None
    n.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None))
    n.get_logger = lambda: types.SimpleNamespace(warn=print)

    def enviar(goal):
        f = _Futuro(_Handle()); n.envios.append(f); return f
    n._nav_client = types.SimpleNamespace(wait_for_server=lambda timeout_sec: True, send_goal_async=enviar)
    for nombre in ("send_nav_goal", "_guardar_goal_handle", "_guardar_resultado", "_terminar_meta"):
        setattr(n, nombre, getattr(N, nombre).__get__(n))
    return n


def test_exito_queda_registrado_con_distancias():
    from action_msgs.msg import GoalStatus
    n = _nodo([{"x": 0.0, "y": 0.0, "theta_deg": 0}, {"x": 1.9, "y": 0.1, "theta_deg": 0}])
    assert n.send_nav_goal(2.0, 0.0, 0.0)
    f = n.envios[0]; f.resolver()
    h = f.valor; h.res.valor.status = GoalStatus.STATUS_SUCCEEDED; h.res.resolver()
    (r,) = list(n.metas_terminadas)
    assert r["resultado"] == "exito" and r["dist_inicio_m"] == 2.0 and r["dist_fin_m"] == 0.14


def test_meta_reemplazada_y_la_vieja_no_se_duplica():
    from action_msgs.msg import GoalStatus
    n = _nodo([])
    n.send_nav_goal(1.0, 0.0, 0.0); n.envios[0].resolver()
    n.send_nav_goal(2.0, 0.0, 0.0); n.envios[1].resolver()
    viejo = n.envios[0].valor; viejo.res.valor.status = GoalStatus.STATUS_ABORTED; viejo.res.resolver()
    nuevo = n.envios[1].valor; nuevo.res.valor.status = GoalStatus.STATUS_ABORTED; nuevo.res.resolver()
    assert [r["resultado"] for r in n.metas_terminadas] == ["reemplazada", "abortada"]
    assert n.metas_terminadas[0]["dist_fin_m"] is None          # sin TF no se inventa la distancia


def test_rechazada():
    n = _nodo([])
    n.send_nav_goal(1.0, 1.0, 0.0)
    f = n.envios[0]; f.valor.accepted = False; f.resolver()
    assert [r["resultado"] for r in n.metas_terminadas] == ["rechazada"]
