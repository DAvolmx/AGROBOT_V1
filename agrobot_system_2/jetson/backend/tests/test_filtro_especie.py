from camera import filtrar_plagas_por_especie

PLANTA = ["maceta", "col_planta"]


def _caja(clase, xyxy=(100, 100, 200, 300), conf=0.7):
    return {"class": clase, "confidence": conf, "xyxy": xyxy}


class Clip:
    def __init__(self, p):
        self.p, self.llamadas = p, 0

    def __call__(self, frame, xyxy):
        self.llamadas += 1
        return {"p_col": self.p}


def correr(cajas, nuevas, clip, cache=None, ahora=0.0):
    return filtrar_plagas_por_especie(None, cajas, nuevas, PLANTA, clip, {} if cache is None else cache, ahora)


def test_plaga_sobre_planta_que_no_es_col_se_descarta():
    cajas = [_caja("maceta"), _caja("col_amarillamiento", (110, 120, 150, 160))]
    c, n = correr(cajas, [("maceta", 0.7), ("col_amarillamiento", 0.5)], Clip(0.1))
    assert [x["class"] for x in c] == ["maceta"] and n == [("maceta", 0.7)]


def test_plaga_sobre_col_se_conserva():
    cajas = [_caja("maceta"), _caja("col_amarillamiento", (110, 120, 150, 160))]
    c, n = correr(cajas, [("maceta", 0.7), ("col_amarillamiento", 0.5)], Clip(0.8))
    assert len(c) == 2 and len(n) == 2


def test_sin_maceta_en_el_cuadro_no_se_filtra():
    cajas = [_caja("col_amarillamiento")]
    clip = Clip(0.0)
    c, n = correr(cajas, [("col_amarillamiento", 0.5)], clip)
    assert len(c) == 1 and n == [("col_amarillamiento", 0.5)] and clip.llamadas == 0


def test_si_clip_falla_no_se_filtra():
    def roto(frame, xyxy):
        raise RuntimeError("sin CLIP")
    cajas = [_caja("maceta"), _caja("col_malformacion")]
    c, n = correr(cajas, [("col_malformacion", 0.6)], roto)
    assert len(c) == 2 and n == [("col_malformacion", 0.6)]


def test_cache_evita_llamar_a_clip_en_cada_cuadro():
    clip, cache = Clip(0.1), {}
    cajas = [_caja("maceta"), _caja("col_amarillamiento")]
    for t in (0.0, 0.5, 1.0, 3.9):
        correr(cajas, [("col_amarillamiento", 0.5)], clip, cache, t)
    assert clip.llamadas == 1
    correr(cajas, [("col_amarillamiento", 0.5)], clip, cache, 5.0)       # pasada la vigencia, se vuelve a preguntar
    assert clip.llamadas == 2


def test_sin_plagas_no_llama_a_clip():
    clip = Clip(0.1)
    c, n = correr([_caja("maceta")], [("maceta", 0.7)], clip)
    assert clip.llamadas == 0 and len(c) == 1
