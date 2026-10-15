import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sendero  # noqa: E402


def _frame(error, confidence=0.8):
    return {
        "estado": "test", "error_m": error,
        "lateral_m": error, "angulo_deg": 0.0,
        "ancho_m": None, "confianza": confidence,
    }


def test_filter_smooths_single_frame_jump_and_holds_short_dropout():
    vision = sendero.VisionSendero()
    responses = iter([_frame(0.10), _frame(0.90), _frame(None, 0.0),
                      _frame(None, 0.0), _frame(None, 0.0), _frame(None, 0.0)])
    vision.detectar = lambda *_args: next(responses)

    first = vision.analizar(None, None, {}, {})
    second = vision.analizar(None, None, {}, {})
    assert first == 0.10
    assert 0.10 < second < 0.30  # median + confidence-adaptive EMA reject jump

    assert vision.analizar(None, None, {}, {}) == second
    assert vision.analizar(None, None, {}, {}) == second
    assert vision.analizar(None, None, {}, {}) == second
    assert vision.analizar(None, None, {}, {}) is None


def test_angle_search_does_not_lock_onto_lateral_crop_row():
    import numpy as np
    perfiles = np.zeros((len(sendero.ANGULOS_DEG), 10), dtype=np.float32)
    perfiles[sendero.ANGULOS_DEG.index(0), :] = 1.0
    perfiles[sendero.ANGULOS_DEG.index(40), :] = 1.15
    chosen = sendero.ANGULOS_DEG[sendero._angulo_optimo(perfiles, 0.0)]
    assert chosen == 0


def test_rejects_false_lane_narrower_than_robot_corridor():
    assert not sendero._ancho_valido(0.20, {"sendero_ancho_min_cm": 40})
    assert sendero._ancho_valido(0.50, {"sendero_ancho_min_cm": 40})


def test_wide_crop_gap_uses_real_path_width_prior_without_shifting_center():
    left, right, width, adjusted = sendero._ancho_con_prior(-0.65, 0.65, 0.50)
    assert adjusted
    assert abs(width - 0.50) < 1e-9
    assert abs((left + right) / 2) < 1e-9
    assert abs((right - left) - 0.50) < 1e-9
