import numpy as np
from ruta_ideal import Piloto, trazar


def test_closed_route_tracker_does_not_jump_to_nearby_return_row():
    route=trazar([(0,0),(2,0),(2,.3),(0,.3),(0,0)],paso=.1,cerrar=True)
    pilot=Piloto(route,mirada=.4,cerrado=True)
    pilot.objetivo(.5,0.)
    progress_before=pilot.i
    target=pilot.objetivo(.5,.29)
    assert pilot.i-progress_before <= 22
    assert target[0] > .5


def test_open_route_pilot_still_reaches_endpoint():
    route=trazar([(0,0),(1,0)],paso=.1,cerrar=False)
    pilot=Piloto(route,mirada=.2,cerrado=False)
    pilot.objetivo(.85,0.)
    target=pilot.objetivo(.99,0.)
    assert pilot.terminado
    np.testing.assert_allclose(target,route[-1])
