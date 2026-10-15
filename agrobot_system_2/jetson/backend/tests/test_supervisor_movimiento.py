"""Tests del supervisor de movimiento (Fase 2). Puros: sin hardware, sin ROS,
con reloj controlado. No importan jetson_server (no abren dispositivos)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from supervisor_movimiento import (  # noqa: E402
    SupervisorMovimiento, Modo, OK, R_EMERGENCIA, R_SIN_MANDO, R_CADUCA)


class Reloj:
    """Reloj monótono controlable para no depender del tiempo real."""
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def avanzar(self, dt):
        self.t += dt


def nuevo(deadman_s=0.5):
    reloj = Reloj()
    sup = SupervisorMovimiento(v_max=0.55, omega_max=1.5,
                               deadman_s=deadman_s, reloj=reloj)
    return sup, reloj


# ---------------------------------------------------------------- emergencia
def test_orden_rechazada_tras_emergencia():
    sup, _ = nuevo()
    sup.tomar_manual("manual")
    v, w, m = sup.filtrar("manual", 0.5, 0.0)
    assert (v, w, m) == (0.5, 0.0, OK)
    sup.emergencia()
    # Cualquier orden de movimiento se rechaza y da 0.
    v, w, m = sup.filtrar("manual", 0.5, 0.0)
    assert v == 0.0 and w == 0.0 and m == R_EMERGENCIA


def test_emergencia_enclava_hasta_rearme():
    sup, _ = nuevo()
    sup.emergencia()
    # Sin rearme, sigue rechazando aunque pidamos modo o manual.
    assert sup.tomar_manual("manual") == (False, R_EMERGENCIA)
    assert sup.pedir_modo(Modo.NAVEGACION, "nav2") == (False, R_EMERGENCIA)
    v, w, m = sup.filtrar("nav2", 0.3, 0.0)
    assert (v, w, m) == (0.0, 0.0, R_EMERGENCIA)


def test_rearme_requiere_salud_y_deja_detenido():
    sup, _ = nuevo()
    sup.tomar_manual("manual")
    sup.emergencia()
    assert sup.rearmar(salud_ok=False) == (False, "salud_insuficiente")
    assert sup.emergencia_activa is True
    ok, m = sup.rearmar(salud_ok=True)
    assert ok and m == OK
    assert sup.emergencia_activa is False
    assert sup.modo == Modo.DETENIDO
    # No restaura el propietario manual previo.
    assert sup.propietario is None


def test_parar_siempre_pasa_incluso_sin_mando():
    sup, _ = nuevo()
    sup.pedir_modo(Modo.NAVEGACION, "nav2")
    # Un (0,0) de cualquiera se acepta: nunca se rechaza frenar.
    v, w, m = sup.filtrar("manual", 0.0, 0.0)
    assert (v, w, m) == (0.0, 0.0, OK)


# ---------------------------------------------------------------- propiedad
def test_comando_de_productor_sin_mando_se_rechaza():
    sup, _ = nuevo()
    sup.pedir_modo(Modo.EXPLORACION, "exploracion")
    # El explorador manda; una orden tardía de Nav2 (que ya no manda) se cae.
    v, w, m = sup.filtrar("nav2", 0.4, 0.0)
    assert v == 0.0 and w == 0.0 and m == R_SIN_MANDO
    # La del propietario pasa.
    v, w, m = sup.filtrar("exploracion", 0.4, 0.0)
    assert m == OK and v == 0.4


def test_manual_arrebata_a_autonomo():
    sup, _ = nuevo()
    sup.pedir_modo(Modo.NAVEGACION, "nav2")
    assert sup.modo == Modo.NAVEGACION
    sup.tomar_manual("manual")
    assert sup.modo == Modo.MANUAL and sup.propietario == "manual"
    # Nav2 ya no manda: su orden se rechaza (cancelación en la puerta).
    v, w, m = sup.filtrar("nav2", 0.4, 0.0)
    assert m == R_SIN_MANDO and v == 0.0
    # Y un autónomo no puede arrebatarle el mando al manual.
    assert sup.pedir_modo(Modo.NAVEGACION, "nav2") == (False, R_SIN_MANDO)


def test_soltar_manual_no_reanuda_autonomia():
    sup, _ = nuevo()
    sup.pedir_modo(Modo.EXPLORACION, "exploracion")
    sup.tomar_manual("manual")
    sup.soltar_manual()
    assert sup.modo == Modo.DETENIDO
    assert sup.propietario is None
    # La autonomía NO se reanuda sola: queda "permitida" pero hay que pedirla.
    assert sup.autonomia_permitida() is True
    # Mientras no se pida, el explorador no manda.
    v, w, m = sup.filtrar("exploracion", 0.4, 0.0)
    assert m == R_SIN_MANDO and v == 0.0


# ---------------------------------------------------------------- caducidad
def test_orden_caduca_se_rechaza():
    sup, reloj = nuevo(deadman_s=0.5)
    sup.tomar_manual("manual")
    t_orden = reloj.t
    reloj.avanzar(0.6)   # la orden quedó vieja
    v, w, m = sup.filtrar("manual", 0.5, 0.0, t=t_orden)
    assert v == 0.0 and w == 0.0 and m == R_CADUCA
    # Una orden fresca del mismo propietario sí pasa.
    v, w, m = sup.filtrar("manual", 0.5, 0.0, t=reloj.t)
    assert m == OK and v == 0.5


# ---------------------------------------------------------------- límites
def test_limites_unificados_recortan():
    sup, _ = nuevo()
    sup.tomar_manual("manual")
    v, w, m = sup.filtrar("manual", 3.0, 9.0)   # pide más del tope físico
    assert m == OK
    assert v == 0.55 and w == 1.5
    v, w, m = sup.filtrar("manual", -3.0, -9.0)
    assert v == -0.55 and w == -1.5


def test_estado_reporta_rechazos():
    sup, _ = nuevo()
    sup.pedir_modo(Modo.NAVEGACION, "nav2")
    sup.filtrar("exploracion", 0.4, 0.0)   # sin mando
    est = sup.estado()
    assert est["modo"] == "navegacion"
    assert est["propietario"] == "nav2"
    assert est["rechazos"][R_SIN_MANDO] == 1


# ---------------------------------------------------------- protección final
def test_proteccion_final_recorta_y_no_mira_propiedad():
    sup, _ = nuevo()
    sup.pedir_modo(Modo.NAVEGACION, "nav2")
    # No depende del dueño: recorta a límites y pasa.
    v, w, m = sup.proteccion_final(3.0, 9.0)
    assert (v, w, m) == (0.55, 1.5, OK)


def test_proteccion_final_bloquea_en_emergencia():
    sup, _ = nuevo()
    sup.emergencia()
    v, w, m = sup.proteccion_final(0.5, 0.0)
    assert (v, w, m) == (0.0, 0.0, R_EMERGENCIA)


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
