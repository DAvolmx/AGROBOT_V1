"""Misión de plantas: reiniciar el explorador NO lanza barrido (bucle de escaneos) y se reanuda tras un mando manual."""
import mision_plantas as M


class FakeExpl:
    def __init__(self):
        self.activo = False
        self.escaneo_al_iniciar = True
        self.arranques = []

    def iniciar(self):
        self.activo = True
        self.arranques.append(self.escaneo_al_iniciar)     # ¿pediría el barrido inmediato?
        return True, "ok"

    def detener(self):
        self.activo = False


class FakeVision:
    estado = "inactiva"
    ultimo_resultado = None

    def __init__(self, ok=True):
        self.ok = ok

    def iniciar(self):
        return (True, "ok") if self.ok else (False, "no arrancó")

    def cancelar(self, m=""):
        pass


def _mision(vision, macetas):
    expl = FakeExpl()
    m = M.MisionPlantas(explorador=expl, vision=vision, get_macetas=lambda: macetas, get_pose=lambda: {"x": 0, "y": 0, "theta_deg": 0},
                        led=lambda b: None, grabar=lambda b: None, get_config=lambda: {"persistencia": 1})
    return m, expl


def test_si_la_vision_no_arranca_el_explorador_se_reinicia_sin_barrido():
    m, expl = _mision(FakeVision(ok=False), [{"conf": 0.9, "yaw_deg": 0, "dist_m": 1.0}])
    assert m.iniciar()[0]
    m.paso()                                     # ve la maceta, la visión falla -> reinicia el explorador
    assert expl.arranques == [True, False]       # el 1.º arranque sí barre; el reinicio NO
    assert expl.escaneo_al_iniciar is True       # el valor original se restaura


def test_reanudar_tras_mando_manual_conserva_las_plantas():
    m, expl = _mision(FakeVision(), [])
    m.iniciar()
    m.plantas.append({"n": 1})
    m.detener("mando manual")
    assert m.estado == M.FIN and not expl.activo
    ok, _ = m.reanudar()
    assert ok and m.estado == M.EXPLORANDO and m.plantas == [{"n": 1}] and expl.arranques[-1] is False


def test_reanudar_solo_si_se_interrumpio_por_mando_manual():
    m, _ = _mision(FakeVision(), [])
    m.iniciar()
    m.detener("operador")
    assert m.reanudar()[0] is False


def test_detener_dos_veces_no_duplica_el_evento_de_fin():
    eventos = []
    m, _ = _mision(FakeVision(), [])
    m._ev = eventos.append
    m.iniciar()
    m.detener("mando manual")
    m.detener("parada")
    assert [e["tipo"] for e in eventos].count("mision_fin") == 1


class VisionConPausa(FakeVision):
    pausa_externa = False

    def __init__(self):
        super().__init__()
        self.estado = "analizando"
        self.cancelada = False

    def cancelar(self, m=""):
        self.cancelada = True


def test_pausar_la_mision_pausa_la_inspeccion_en_vez_de_cancelarla():
    v = VisionConPausa()
    m, expl = _mision(v, [])
    m.iniciar()
    m.pausar()
    assert v.pausa_externa is True and v.cancelada is False
    m.continuar()
    assert v.pausa_externa is False
