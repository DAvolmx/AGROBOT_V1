import tempfile, pathlib, modelos_preset as m

def test_listar():
    with tempfile.TemporaryDirectory() as d:
        (pathlib.Path(d) / m.PRESETS["interior"]["archivo"]).write_bytes(b"x")
        r = {p["id"]: p for p in m.listar(pathlib.Path(d), m.PRESETS["interior"]["archivo"])}
        assert r["interior"]["disponible"] and r["interior"]["activo"]
        assert not r["exterior"]["disponible"] and not r["exterior"]["activo"]
