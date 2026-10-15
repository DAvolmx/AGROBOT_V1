import numpy as np
from seguimiento_sendero import SeguidorSendero


def test_straight_path_keeps_forward_command_and_near_zero_yaw():
    route=np.c_[np.linspace(0,5,101),np.zeros(101)]
    follower=SeguidorSendero(route,closed=False,progress_m=.5)
    v,w=follower.orden((.5,0.,0.),speed=.2,lookahead=.4)
    assert v > .15
    assert abs(w) < 1e-9


def test_nearby_parallel_branch_cannot_teleport_route_progress():
    route=np.array([[0,0],[5,0],[5,.3],[0,.3],[0,0]],dtype=float)
    follower=SeguidorSendero(route,closed=True,progress_m=1.0)
    follower.orden((1.,.3,0.),speed=.2,lookahead=.4)
    assert .85 <= follower.progress <= 2.4


def test_curved_route_requests_turn_and_reduces_speed():
    a=np.linspace(-.2,.9,80)
    route=np.c_[np.cos(a),np.sin(a)]
    follower=SeguidorSendero(route,closed=False,progress_m=0.)
    v,w=follower.orden((1.,0.,0.),speed=.5,lookahead=.4)
    assert w > 0
    assert v < .5
