"""Subir datos del robot real al simulador (sin robot).   python3 -m pytest tests/test_subir_datos_simulador.py -q"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import subir_datos_simulador as SD  # noqa: E402

MAPA = Path(__file__).resolve().parents[3] / "maps" / "mision_20261023_1618"


def test_recorrido_se_simplifica_y_conserva_el_final():
    wp = [{"x": i * 0.05, "y": 0.0, "theta_deg": 0} for i in range(41)]
    pts = SD.simplificar_recorrido(wp, 0.3)
    assert len(pts) < 12 and pts[0] == [0.0, 0.0] and pts[-1] == [2.0, 0.0]


def test_mundo_tiene_el_formato_del_simulador():
    m = SD.construir_mundo("real_x", [[0, 0, 2, 0]], [{"x": 0, "y": 0}, {"x": 1, "y": 0}], [{"x": 1.0, "y": 0.5}, {"x": None, "y": None}])
    assert m["paredes"] == [[0, 0, 2, 0]] and len(m["plantas"]) == 1 and m["plantas"][0][:3] == [1.0, 0.5, SD.R_PLANTA]
    assert m["inicio"] == [0.0, 0.0] and len(m["limites"]) == 4 and m["regreso_al_inicio"]


def test_exportar_escribe_mundo_y_paquete(tmp_path):
    if not (MAPA / "mapa.yaml").exists():
        import pytest
        pytest.skip("no está el mapa de ejemplo")
    r = SD.exportar("prueba", MAPA, [{"x": 1.86, "y": 0.46}], [{"x": 0, "y": 0}, {"x": 1, "y": 0.2}], tmp_path / "mundos", tmp_path / "paq")
    assert r["muros"] > 3 and r["plantas"] == 1 and (tmp_path / "mundos" / "real_prueba.json").exists()
    assert (tmp_path / "paq" / "prueba" / "digital" / "plano.svg").exists() and (tmp_path / "paq" / "prueba" / "plantas.json").exists()
    d = json.loads((tmp_path / "mundos" / "real_prueba.json").read_text())
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "simulador_pasivo"))
    import mundo
    assert len(mundo.Mundo.desde_dict(d).a_dict()["paredes"]) == len(d["paredes"])      # el simulador lo carga
