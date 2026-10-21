import time

import numpy as np
import pytest

import slam_correccion as SC


def _pesos(tmp_path, K=5, seed=0):
    rng = np.random.default_rng(seed)
    D = K * SC.N_BASE
    d = {"W1": rng.normal(0, .1, (128, D)), "b1": rng.normal(0, .1, 128), "W2": rng.normal(0, .1, (64, 128)), "b2": rng.normal(0, .1, 64),
         "W3": rng.normal(0, .1, (32, 64)), "b3": rng.normal(0, .1, 32), "W4": rng.normal(0, .1, (3, 32)), "b4": rng.normal(0, .1, 3),
         "x_mean": np.zeros(D), "x_scale": np.ones(D), "y_mean": np.zeros(3), "y_scale": np.full(3, 0.01), "K": np.array(K)}
    ruta = tmp_path / "pesos.npz"
    np.savez(ruta, **d)
    return ruta, d


def _scan():
    r = np.full(360, 3.0)
    r[60:121] = 0.5
    return r


def test_sectores():
    f, i, d = SC.dmin_sectores(_scan())
    assert f == 3.0 and i == 0.5 and d == 3.0
    assert SC.dmin_sectores(None) == (8.0, 8.0, 8.0)
    r = np.full(360, np.inf)
    assert SC.dmin_sectores(r) == (8.0, 8.0, 8.0)


def test_forward_igual_a_la_referencia(tmp_path):
    ruta, d = _pesos(tmp_path)
    c = SC.CorreccionSlam(ruta)
    corr = c.actualizar(0.2, 0.1, 0.0, 0.02, _scan(), (0.02, 0, 0), True, 0.25)
    f = np.array([0.2, 0.1, 0.0, 0.02, 3.0, 0.5, 3.0, 0.02, 0, 0, 1.0, 0.25])
    z = np.tile(f, 5)
    for i, k in enumerate(("1", "2", "3", "4")):
        z = z @ d["W" + k].T + d["b" + k]
        if i < 3:
            z = np.maximum(z, 0)
    esperado = np.clip(z * 0.01, -SC.LIMITE_CORR, SC.LIMITE_CORR)
    assert np.allclose(corr, esperado)


def test_fuera_de_distribucion_y_no_finito_dan_ceros(tmp_path):
    ruta, _ = _pesos(tmp_path)
    c = SC.CorreccionSlam(ruta)
    assert np.all(c.actualizar(1e6, 0, 0, 0, _scan(), (0, 0, 0), True, 0.25) == 0) and c.ultimo_ood
    assert np.all(c.actualizar(float("nan"), 0, 0, 0, _scan(), (0, 0, 0), True, 0.25) == 0)


def test_limite_de_seguridad(tmp_path):
    ruta, d = _pesos(tmp_path)
    d["y_scale"] = np.full(3, 100.0)
    np.savez(ruta, **d)
    c = SC.CorreccionSlam(ruta)
    corr = c.actualizar(0.2, 0.1, 0.0, 0.02, _scan(), (0.02, 0, 0), True, 0.25)
    assert np.all(np.abs(corr) <= SC.LIMITE_CORR + 1e-12)


def test_pesos_incompatibles(tmp_path):
    ruta, d = _pesos(tmp_path)
    d["K"] = np.array(4)
    np.savez(ruta, **d)
    with pytest.raises(ValueError):
        SC.CorreccionSlam(ruta)


def test_rapida(tmp_path):
    ruta, _ = _pesos(tmp_path)
    c = SC.CorreccionSlam(ruta)
    t0 = time.perf_counter()
    for _ in range(500):
        c.actualizar(0.2, 0.1, 0.0, 0.02, _scan(), (0.02, 0, 0), True, 0.25)
    assert (time.perf_counter() - t0) / 500 < 0.005     # < 5 ms por fotograma (el límite del documento es 50 ms)


def test_desde_config_apagado_y_con_pesos(tmp_path):
    assert SC.desde_config({"correccion_ml_activa": False}) is None
    ruta, _ = _pesos(tmp_path)
    assert SC.desde_config({"correccion_ml_activa": True, "correccion_ml_pesos": str(ruta)}) is not None
    assert SC.desde_config({"correccion_ml_activa": True, "correccion_ml_pesos": str(tmp_path / "no_existe.npz")}) is None


def test_pesos_reales_de_kaggle_cargan():
    from pathlib import Path
    ruta = Path(__file__).resolve().parents[1] / "models" / "slam_correccion" / "pesos_mlp.npz"
    c = SC.CorreccionSlam(ruta)
    corr = c.actualizar(0.2, 0.0, 0.0, 0.01, _scan(), (0.05, 0.0, 0.0), True, 0.25)
    assert corr.shape == (3,) and np.all(np.isfinite(corr)) and np.all(np.abs(corr) <= SC.LIMITE_CORR + 1e-12)
