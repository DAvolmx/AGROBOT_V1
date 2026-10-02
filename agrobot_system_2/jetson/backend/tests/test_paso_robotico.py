"""
Prueba del movimiento robótico por pulsos (paso_robotico.py) contra un robot
falso con reloj virtual: cada pulso dura 0,8 s, mueve lo que diga el modelo
y el LIDAR ve los huecos de un pasillo recto.

    cd jetson/backend && python3 -m pytest tests/test_paso_robotico.py -q
"""
import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from paso_robotico import PasoRobotico  # noqa: E402


class RobotFalso:
    """Pasillo recto: pared delante a 3 m del origen, detrás a 0,5 m."""

    def __init__(self, avance_m=0.25, giro_deg=10.0, retro_m=0.0, frente=3.0, fondo=0.5):
        self.t = 0.0
        self.x = 0.0
        self.rumbo = 0.0
        self.avance_m, self.giro_deg, self.retro_m = avance_m, giro_deg, retro_m
        self.frente, self.fondo = frente, fondo
        self.fin_pulso = -1.0
        self.pendiente = None
        self.ordenes = []
        self.paradas = 0

    async def dormir(self, s):
        self.t += s
        if self.pendiente is not None and self.t >= self.fin_pulso:
            dx, dth = self.pendiente
            self.x += dx
            self.rumbo += dth
            self.pendiente = None
        await asyncio.sleep(0)

    def ordenar(self, accion, escala, maniobra):
        self.ordenes.append(accion)
        signo = {"adelante": 1, "atras": -1}.get(accion)
        if signo is not None:
            self.pendiente = (signo * self.avance_m, 0.0)
        else:
            g = self.giro_deg if accion == "izquierda" else -self.giro_deg
            self.pendiente = (-self.retro_m, g)
        self.fin_pulso = self.t + 0.8

    def activo(self):
        return self.pendiente is not None

    def distancias(self):
        return {"adelante": max(0.0, self.frente - self.x), "atras": max(0.0, self.fondo + self.x)}

    def hacer(self, tmp_path):
        return PasoRobotico(ordenar=self.ordenar, parar=lambda: setattr(self, "paradas", self.paradas + 1),
                            conductor_activo=self.activo, distancias=self.distancias,
                            rumbo_deg=lambda: self.rumbo, edad_scan_s=lambda: 0.1,
                            ruta_registro=tmp_path, reloj=lambda: self.t, dormir=self.dormir)


def correr(c):
    return asyncio.run(c)


def test_avanza_hasta_la_distancia_pedida(tmp_path):
    r = RobotFalso(avance_m=0.25)
    p = r.hacer(tmp_path)
    res = correr(p.avanzar(1.0))
    assert res["resultado"] == "completado"
    assert abs(r.x - 1.0) <= 0.13          # 4 pulsos de 25 cm
    assert r.ordenes == ["adelante"] * 4
    assert r.paradas >= 1                  # termina parado


def test_cada_pulso_queda_registrado(tmp_path):
    r = RobotFalso(avance_m=0.25)
    correr(r.hacer(tmp_path).avanzar(0.5))
    filas = [json.loads(l) for f in tmp_path.glob("*.jsonl") for l in f.read_text().splitlines()]
    assert len(filas) == 2
    assert filas[0]["avance_m"] == 0.25 and filas[0]["accion"] == "adelante"


def test_robot_que_no_se_mueve_se_rinde(tmp_path):
    """El caso real del pasillo: la C3 manda y el robot no avanza."""
    r = RobotFalso(avance_m=0.0)
    res = correr(r.hacer(tmp_path).avanzar(1.0))
    assert res["resultado"] == "sin_progreso"
    assert len(r.ordenes) == 3


def test_no_avanza_contra_un_obstaculo(tmp_path):
    r = RobotFalso(avance_m=0.25, frente=0.9)
    res = correr(r.hacer(tmp_path).avanzar(2.0))
    assert res["resultado"] == "obstaculo"
    assert r.distancias()["adelante"] > 0.3     # paró antes de llegar a la pared


def test_gira_midiendo_el_rumbo(tmp_path):
    r = RobotFalso(giro_deg=12.0)
    res = correr(r.hacer(tmp_path).girar(-90))
    assert res["resultado"] == "completado"
    assert -100 <= r.rumbo <= -85
    assert set(r.ordenes) == {"derecha"}


def test_giro_que_retrocede_para_antes_de_tocar(tmp_path):
    """Medido: cada pulso de giro retrocede ~8 cm. Con la pared detrás, para."""
    r = RobotFalso(giro_deg=2.0, retro_m=0.08, fondo=0.4)
    res = correr(r.hacer(tmp_path).girar(90))
    assert res["resultado"] in ("retroceso", "sin_progreso")
    assert r.distancias()["atras"] > 0.05


def test_giro_casi_nulo_se_rinde(tmp_path):
    r = RobotFalso(giro_deg=0.3)
    res = correr(r.hacer(tmp_path).girar(45))
    assert res["resultado"] == "sin_progreso"


def test_pulso_recto_que_gira_se_detiene(tmp_path):
    """Medido el 2026-10-02: cada pulso 'adelante' giraba 8-16° a la derecha."""
    r = RobotFalso(avance_m=0.25)
    orig = r.ordenar

    def ordenar_con_deriva(accion, escala, maniobra):
        orig(accion, escala, maniobra)
        dx, _ = r.pendiente
        r.pendiente = (dx, -15.0)
    r.ordenar = ordenar_con_deriva
    res = correr(r.hacer(tmp_path).avanzar(1.0))
    assert res["resultado"] == "deriva_rumbo"
    assert len(r.ordenes) == 1


def test_delante_y_detras_no_coinciden(tmp_path):
    r = RobotFalso(avance_m=0.25)
    d0 = r.distancias
    r.distancias = lambda: {"adelante": d0()["adelante"] + (1.6 if r.x > 0.1 else 0.0),
                            "atras": d0()["atras"]}
    res = correr(r.hacer(tmp_path).avanzar(1.0))
    assert res["resultado"] == "medida_incoherente"


def test_parada_de_emergencia_no_deja_mandar_otro_pulso(tmp_path):
    """El fallo del 2026-10-02: tras la parada, el lazo mandaba el siguiente
    pulso. Se detiene en medio del primero, sin cancelar la tarea (como
    cuando lo llama la evasión directamente)."""
    r = RobotFalso(avance_m=0.25)
    orig = r.ordenar
    caja = []

    def ordenar_y_parar(accion, escala, maniobra):
        orig(accion, escala, maniobra)
        caja[0].detener()                 # el operador pulsa STOP durante el pulso
    r.ordenar = ordenar_y_parar
    caja.append(r.hacer(tmp_path))
    res = correr(caja[0].avanzar(1.0))
    assert res["resultado"] == "detenido"
    assert len(r.ordenes) == 1
