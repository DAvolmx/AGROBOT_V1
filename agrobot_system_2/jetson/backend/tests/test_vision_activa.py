"""Visión activa: la lógica, contra un mundo simulado mínimo (robot que gira ~12° por pulso y avanza ~12 cm, planta a un rumbo y distancia)."""
import math

import pytest

import vision_activa as V
from calibracion_camara import GRADOS_POR_GRADO, INCLINACION_HOME_DEG, SERVO_HOME_DEG

W, H, HFOV = 640, 480, 71.0
F = (W / 2) / math.tan(math.radians(HFOV) / 2)


class Mundo:
    """Robot en el origen con rumbo `theta` (° CCW). Planta a `rumbo` (° CCW, mundo) y `dist` m del LIDAR."""

    def __init__(self, rumbo=40.0, dist=1.6, giro_por_pulso=12.0, avance=0.12, conf=0.8, clase="maceta_planta"):
        self.theta, self.rumbo, self.dist = 0.0, rumbo, dist
        self.giro, self.avance, self.conf, self.clase = giro_por_pulso, avance, conf, clase
        self.t = 100.0
        self.servo = 98.0
        self.pulsos, self.paradas, self.eventos = [], 0, []
        self.edad = 0.1
        self.enlace = True
        self.autonomia = False

    def reloj(self):
        return self.t

    # --- hardware simulado
    def conducir(self, accion):
        self.pulsos.append(accion)
        if accion == "izquierda":
            self.theta += self.giro
        elif accion == "derecha":
            self.theta -= self.giro
        elif accion == "adelante":
            self.dist -= self.avance

    def girar(self, grados):
        self.giros_cerrados = getattr(self, "giros_cerrados", []) + [grados]
        self.theta += grados * 0.93                                   # el giro cerrado casi acierta (7 % de error)
        return True

    def set_servo(self, a):
        self.servo = a

    def distancias(self):
        rel = self.rumbo - self.theta
        return {"adelante": round(self.dist - 0.185, 3) if abs(rel) <= 15 else None, "atras": 2.0, "izquierda": None, "derecha": None}

    def cajas(self):
        rel = self.rumbo - self.theta                      # + = planta a la izquierda
        yaw_err = -rel                                     # + = a la derecha en la imagen
        beta = math.degrees(math.atan(0.25 / max(self.dist, 0.05)))
        pitch_err = beta - (INCLINACION_HOME_DEG + GRADOS_POR_GRADO * (SERVO_HOME_DEG - self.servo))
        if abs(yaw_err) >= HFOV / 2 - 3 or abs(pitch_err) >= 24:
            return ([], self.t, W, H, HFOV)
        cx, cy = W / 2 + F * math.tan(math.radians(yaw_err)), H / 2 + F * math.tan(math.radians(pitch_err))
        return ([{"class": self.clase, "confidence": self.conf, "xyxy": (cx - 50, cy - 50, cx + 50, cy + 50)}], self.t, W, H, HFOV)

    def crear(self, cfg=None, tmp=None):
        return V.VisionActiva(get_cajas=self.cajas, get_servo=lambda: self.servo, set_servo=self.set_servo, conducir=self.conducir,
                              ocupado=lambda: False, parar=lambda: setattr(self, "paradas", self.paradas + 1), distancias=self.distancias,
                              edad_scan=lambda: self.edad, get_config=lambda: cfg or {"max_objetivos": 1}, on_event=self.eventos.append, girar=self.girar,
                              enlace_ok=lambda: self.enlace, autonomia_activa=lambda: self.autonomia, reloj=self.reloj, carpeta=tmp)

    def correr(self, va, pasos=400, hasta=None):
        for _ in range(pasos):
            self.t += 0.2
            va.paso()
            if va.estado == V.INACTIVA or (hasta and va.estado == hasta):
                break


def test_angulos_de_caja():
    yaw, pitch = V.angulos_de_caja((F + W / 2 - 10, H / 2 - 10, F + W / 2 + 10, H / 2 + 10), W, H, HFOV)
    assert yaw == pytest.approx(45.0, abs=0.5) and abs(pitch) < 0.1             # objetivo a la derecha, centrado en altura
    assert V.angulos_de_caja((W / 2 - 5, 300, W / 2 + 5, 340), W, H, HFOV)[1] > 0     # debajo del centro -> pitch positivo


def test_encuentra_centra_se_acerca_y_analiza(tmp_path):
    m = Mundo(rumbo=40.0, dist=1.6)
    va = m.crear(tmp=tmp_path)
    assert va.iniciar() == (True, "ok")
    m.correr(va)
    assert va.estado == V.INACTIVA
    assert "inspeccion" in [e["tipo"] for e in m.eventos]
    assert va.objetivos_hechos == 1 and "objetivos inspeccionados" in va.motivo_fin
    insp = [e for e in m.eventos if e["tipo"] == "inspeccion"][0]
    assert abs(insp["yaw_deg"]) <= 12.0                                             # al inspeccionar estaba mirando a la planta
    assert insp["hueco_m"] is not None and insp["hueco_m"] <= 0.55 + 0.08 + 0.12   # y cerca
    assert "adelante" in m.pulsos and "izquierda" in m.pulsos
    assert "atras" in m.pulsos                                                      # al terminar se aleja antes de girar (2026-10-22)
    assert (tmp_path / (__import__("time").strftime("%Y%m%d") + ".jsonl")).exists()


def test_el_servo_corrige_la_altura():
    m = Mundo(rumbo=0.0, dist=1.0)                                                   # planta de frente pero la cámara mira más abajo que ella
    va = m.crear(cfg={"max_objetivos": 1, "distancia_inspeccion_m": 5.0})
    va.iniciar()
    m.servo = 80.0                                                                   # mirando más abajo que la planta: ella queda ARRIBA en la imagen
    s0 = m.servo
    m.correr(va, pasos=60, hasta=V.ENCUADRANDO)
    assert m.servo > s0                                                              # sube el ángulo de servo (cámara hacia arriba)


def test_no_avanza_con_hueco_menor_al_minimo():
    m = Mundo(rumbo=0.0, dist=0.30)                                                  # hueco 0,115 m < 0,20
    va = m.crear()
    va.iniciar()
    m.correr(va)
    assert "adelante" not in m.pulsos


def test_ignora_baja_confianza_y_otras_clases():
    for conf, clase in ((0.1, "maceta_planta"), (0.9, "obstaculo")):
        m = Mundo(rumbo=0.0, dist=1.0, conf=conf, clase=clase)
        va = m.crear(cfg={"max_giros_busqueda": 2})
        va.iniciar()
        m.correr(va)
        assert "adelante" not in m.pulsos and va.objetivos_hechos == 0


def test_sin_objetivo_termina_tras_girar_la_vuelta():
    m = Mundo(rumbo=0.0, dist=1.0, clase="obstaculo")
    va = m.crear(cfg={"max_giros_busqueda": 3})
    va.iniciar()
    m.correr(va, pasos=2000)
    assert va.estado == V.INACTIVA and "sin objetivo" in va.motivo_fin


def test_si_no_logra_centrar_se_rinde():
    m = Mundo(rumbo=25.0, dist=1.6, giro_por_pulso=0.0)                              # los pulsos no giran nada
    va = m.crear(cfg={"max_pulsos_objetivo": 5})
    va.iniciar()
    m.correr(va, pasos=500)
    assert va.estado == V.INACTIVA and "centrar" in va.motivo_fin


def test_pausa_y_no_mueve_con_scan_viejo_c3_caida_o_autonomia():
    for atributo, valor in (("edad", 5.0), ("enlace", False), ("autonomia", True)):
        m = Mundo()
        va = m.crear()
        va.estado = V.BUSCANDO
        va._t_inicio = m.t
        setattr(m, atributo, valor)
        m.correr(va, pasos=30)
        assert va.estado == V.PAUSA and m.pulsos == [] and m.paradas == 1


def test_se_reanuda_cuando_vuelve_la_seguridad():
    m = Mundo()
    va = m.crear()
    va.iniciar()
    m.edad = 5.0
    m.correr(va, pasos=5)
    assert va.estado == V.PAUSA
    m.edad = 0.1
    m.correr(va, pasos=5)
    assert va.estado != V.PAUSA


def test_no_arranca_con_autonomia_y_cancelar_para_y_es_idempotente():
    m = Mundo()
    m.autonomia = True
    va = m.crear()
    assert va.iniciar()[0] is False
    m.autonomia = False
    assert va.iniciar()[0] is True
    va.cancelar("parada")
    va.cancelar("parada")
    assert va.estado == V.INACTIVA and m.paradas == 1 and va.motivo_fin == "parada"


def test_tiempo_maximo():
    m = Mundo(clase="obstaculo")
    va = m.crear(cfg={"tiempo_max_s": 10.0, "max_giros_busqueda": 1000})
    va.iniciar()
    m.correr(va, pasos=500)
    assert va.estado == V.INACTIVA and va.motivo_fin == "tiempo máximo"


def test_sin_mover_decide_pero_no_manda_pulsos():
    m = Mundo(rumbo=25.0, dist=1.6)
    va = m.crear(cfg={"sin_mover": True, "max_pulsos_objetivo": 4})
    va.iniciar()
    m.correr(va, pasos=300)
    assert m.pulsos == [] and any(e["tipo"] == "pulso_simulado" for e in m.eventos)
    assert va.estado == V.INACTIVA and "centrar" in va.motivo_fin


def test_caja_de_imagen_completa_no_sirve_para_centrar_y_avisa():
    m = Mundo(rumbo=0.0, dist=1.0)
    m.cajas_original = m.cajas
    m.cajas = lambda: ([{"class": "col_dano_insecto", "confidence": 0.9, "xyxy": (0, 0, W, H)}], m.t, W, H, HFOV)
    va = m.crear(cfg={"clases_objetivo": ["col"], "max_giros_busqueda": 1})
    va.iniciar()
    m.correr(va, pasos=300)
    assert "adelante" not in m.pulsos and "no localiza" in (va.aviso or "")


def test_clasificador_da_una_segunda_opinion_en_el_resultado(tmp_path):
    m = Mundo(rumbo=0.0, dist=0.7)
    va = m.crear(cfg={"max_objetivos": 1}, tmp=tmp_path)
    va._clasificar = lambda ruta: {"modelo": "falso", "top": [{"clase": "col_sana", "prob": 0.9}]}
    va._capturar = lambda info: "/tmp/foto.jpg"
    va.iniciar()
    m.correr(va, pasos=400)
    assert va.ultimo_resultado["diagnostico"]["top"][0]["clase"] == "col_sana"


def test_con_giro_cerrado_centra_en_menos_ordenes_que_con_pulsos(tmp_path):
    m1 = Mundo(rumbo=40.0, dist=1.6)
    v1 = m1.crear(cfg={"max_objetivos": 1, "giro_cerrado": False}, tmp=tmp_path)
    v1.iniciar(); m1.correr(v1)
    m2 = Mundo(rumbo=40.0, dist=1.6)
    v2 = m2.crear(cfg={"max_objetivos": 1, "giro_cerrado": True}, tmp=tmp_path)
    v2.iniciar(); m2.correr(v2)
    assert v2.objetivos_hechos == 1
    giros_pulsos = sum(1 for a in m1.pulsos if a in ("izquierda", "derecha"))
    assert len(getattr(m2, "giros_cerrados", [])) < giros_pulsos
    assert all(g > 0 for g in m2.giros_cerrados)                    # la planta está a la izquierda: giros positivos
