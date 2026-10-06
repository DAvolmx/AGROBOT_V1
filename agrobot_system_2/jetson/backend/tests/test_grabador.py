"""Prueba del grabador de sesión con fuentes falsas y reloj virtual.

    cd jetson/backend && python3 -m pytest tests/test_grabador.py -q
"""
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import grabador as G  # noqa: E402


class Mundo:
    def __init__(self):
        self.t = 0.0
        self.v = 0.0
        self.t_scan = 0.0

    def leer(self):
        return {
            "velocidades": lambda: {"v_pedida": self.v, "omega_pedida": 0.0,
                                    "v_aplicada": self.v, "omega_aplicada": 0.0},
            "c3": lambda: {"v_cmd": self.v, "omega_cmd": 0.0, "failsafe": False},
            "pose": lambda: {"x": self.t * self.v, "y": 0.0, "theta_deg": 0.0},
            "distancias": lambda: {"adelante": 1.5, "atras": 0.4},
            "t_scan": lambda: self.t_scan,
            "scan": lambda: [{"angle_deg": a, "distance_m": 2.0} for a in range(-180, 180, 1)],
            "fallo": lambda: 1 / 0,           # una fuente rota no tumba nada
        }


def test_graba_telemetria_scans_y_eventos(tmp_path):
    m = Mundo()
    g = G.Grabador(leer=m.leer(), raiz=tmp_path, reloj=lambda: m.t, config={"x": 1})
    for k in range(100):                   # 10 s a 10 Hz
        m.t = k * 0.1
        m.t_scan = round(m.t, 1)           # un scan nuevo en cada tick
        m.v = 0.3 if 20 <= k < 40 else 0.0
        g.tick()
    g.evento({"type": "obstacle_warning", "data": {"lado": "adelante"}})
    g.evento({"type": "lidar_scan", "data": {}})          # flujo continuo: no se graba
    g.cerrar()

    filas = list(csv.DictReader(open(g.dir / "telemetria.csv")))
    assert len(filas) == 100
    assert filas[25]["v_pedida"] == "0.3"
    scans = [np.load(f) for f in sorted(g.dir.glob("scans_*.npz"))]
    n = sum(len(s["t"]) for s in scans)
    # en movimiento (20 ticks) + 3 s después (30) a 10 Hz, más 1 Hz en reposo
    assert 50 <= n <= 70, n
    assert scans[0]["distancias_m"].shape[1] == 720
    eventos = [json.loads(l) for l in open(g.dir / "eventos.jsonl")]
    assert [e["tipo"] for e in eventos] == ["obstacle_warning"]
    sesion = json.loads((g.dir / "sesion.json").read_text())
    assert sesion["contadores"]["filas"] == 100
