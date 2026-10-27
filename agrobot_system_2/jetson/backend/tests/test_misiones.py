import asyncio
import json

import pytest

import misiones as M


def diag(clase, prob, otros=(), p_col=0.9):
    top = [{"clase": clase, "prob": prob}] + [{"clase": c, "prob": p} for c, p in otros]
    return {"top": top, "especie": {"p_col": p_col}}


def test_veredicto_sana():
    assert M.veredicto(diag("col_sana", 0.8))["estado"] == M.SANA


def test_veredicto_alerta_y_malformacion():
    assert M.veredicto(diag("col_mildiu_velloso", 0.7))["estado"] == M.ALERTA
    assert M.veredicto(diag("col_malformacion", 0.7))["estado"] == M.MALFORMACION


def test_veredicto_sospecha_por_prob_media():
    v = M.veredicto(diag("col_manchas_cafe", 0.4))
    assert v["estado"] == M.SOSPECHA and "manchas" in v["motivo"]


def test_sana_dudosa_con_problema_segundo_es_sospecha():
    v = M.veredicto(diag("col_sana", 0.4, [("col_dano_insecto", 0.35)]))
    assert v["estado"] == M.SOSPECHA and v["clase"] == "col_dano_insecto"


def test_sin_diagnostico_o_no_col_se_revisa_a_mano():
    assert M.veredicto(None)["estado"] == M.REVISAR
    assert M.veredicto(diag("col_sana", 0.9, p_col=0.1))["estado"] == M.REVISAR
    assert M.veredicto(diag("col_mildiu_velloso", 0.1))["estado"] == M.REVISAR


class Expl:
    def __init__(self):
        self.activo = False
        self.regreso_estado = "no"
        self.log = []

    def iniciar(self):
        self.activo = True
        self.log.append("iniciar")
        return True, "ok"

    def detener(self):
        self.activo = False
        self.log.append("detener")

    def iniciar_regreso(self):
        self.regreso_estado = "en curso"
        self.log.append("regreso")
        return True, "ok"


class MP:
    def __init__(self):
        self.plantas, self.estado, self.cfg, self.log = [], "inactiva", None, []

    def iniciar(self):
        self.estado = "explorando"
        return True, "ok"

    def arrancar_tarea(self):
        self.log.append("tarea")

    def detener(self, m="detenida"):
        self.estado = "fin"
        self.log.append(m)


class Reloj:
    def __init__(self):
        self.t = 1_000_000.0

    def __call__(self):
        return self.t


def gestor(tmp_path, **kw):
    ajustes = []
    g = M.GestorMisiones(carpeta=tmp_path, mision_plantas=MP(), explorador=Expl(), vision=None,
                         aplicar_ajustes=lambda a, b: ajustes.append((a, b)), quitar_ajustes=lambda: ajustes.append("quitar"),
                         guardar_mapa=kw.pop("guardar_mapa", None), get_pose=lambda: None, reloj=Reloj(), **kw)
    return g, ajustes


def test_mision_desconocida_y_doble_inicio(tmp_path):
    g, _ = gestor(tmp_path)
    assert not g.iniciar("volar")[0]
    assert g.iniciar("dataset")[0]
    ok, msg = g.iniciar("inspeccion")
    assert not ok and "en curso" in msg


def test_dataset_guarda_fotos_y_manifest(tmp_path):
    g, ajustes = gestor(tmp_path)
    assert g.iniciar("dataset")[0]
    assert ajustes[0][0]["max_plantas"] == 40
    f = tmp_path / "foto.jpg"
    f.write_bytes(b"x")
    g.mp.plantas.append({})
    g.al_planta({"n": 1, "x": 1.0, "y": 2.0, "especie": "col", "p_col": 0.9}, {"vistas": [{"foto": str(f)}, {"foto": str(f)}], "clase": "maceta", "conf_media": 0.7})
    sub = tmp_path / "dataset_plantas" / g.sesion
    assert len(list(sub.glob("planta001_v*.jpg"))) == 2
    reg = json.loads((sub / "manifest.jsonl").read_text().splitlines()[0])
    assert reg["x"] == 1.0 and len(reg["fotos"]) == 2 and "veredicto" not in reg
    assert g.fotos_guardadas == 2
    g.detener("fin")
    assert ajustes[-1] == "quitar" and g.estado == "fin"


def test_inspeccion_veredicto_e_informe(tmp_path):
    g, _ = gestor(tmp_path)
    assert g.iniciar("inspeccion")[0]
    f = tmp_path / "f.jpg"
    f.write_bytes(b"x")
    g.al_planta({"n": 1, "x": 0.5, "y": 0.5}, {"foto": str(f), "diagnostico": diag("col_mildiu_velloso", 0.8)})
    g.al_planta({"n": 2, "x": 1.5, "y": 0.5}, {"foto": str(f), "diagnostico": diag("col_sana", 0.9)})
    g.al_planta({"n": 3, "x": None, "y": None}, {"foto": str(f)})
    g.detener("fin")
    inf = json.loads((tmp_path / "inspecciones" / g.sesion / "informe.json").read_text())
    assert inf["resumen"] == {"alerta": 1, "sana": 1, "revisar a mano": 1}
    md = (tmp_path / "inspecciones" / g.sesion / "informe.md").read_text()
    assert "mildiu" in md and "revisar a mano" in md


def test_al_planta_fuera_de_mision_no_hace_nada(tmp_path):
    g, _ = gestor(tmp_path)
    g.al_planta({"n": 1}, {})
    assert g.fotos_guardadas == 0 and not (tmp_path / "dataset_plantas").exists()


def test_fin_de_mision_plantas_cierra(tmp_path):
    g, ajustes = gestor(tmp_path)
    g.iniciar("dataset")
    g.al_fin_mision_plantas("tiempo máximo")
    assert g.estado == "fin" and g.motivo_fin == "tiempo máximo" and ajustes[-1] == "quitar"


def test_mapear_por_tiempo_regresa_y_guarda(tmp_path):
    guardados = []

    async def guardar(nombre):
        guardados.append(nombre)
        return {"nombre": nombre}

    async def correr():
        g, _ = gestor(tmp_path, guardar_mapa=guardar)
        reloj = g.reloj
        assert g.iniciar("mapear", {"tiempo_max_s": 5.0})[0]
        reloj.t += 10.0                        # pasa el tiempo máximo
        for _ in range(40):
            await asyncio.sleep(0.05)
            if g.expl.regreso_estado == "en curso":
                g.expl.regreso_estado = "completo"
            if g.estado == "fin":
                break
        return g

    import time as _t
    orig = asyncio.sleep

    async def rapido(s):
        await orig(0)
    asyncio.sleep, vuelve = rapido, orig
    try:
        g = asyncio.run(correr())
    finally:
        asyncio.sleep = vuelve
    assert g.estado == "fin" and g.motivo_fin == "tiempo máximo"
    assert g.expl.log == ["iniciar", "regreso", "detener"]
    assert len(guardados) == 1 and guardados[0].startswith("mapeo_")


def test_mapear_mapa_que_no_crece(tmp_path):
    async def guardar(n):
        return {"nombre": n}

    async def correr():
        celdas = [1000]
        g, _ = gestor(tmp_path, guardar_mapa=guardar, mapa_celdas=lambda: celdas[0])
        assert g.iniciar("mapear", {"sin_crecer_s": 20.0, "regresar": False})[0]
        for _ in range(60):
            await asyncio.sleep(0)
            g.reloj.t += 5.0
            if g.estado == "fin":
                break
        return g

    orig = asyncio.sleep

    async def rapido(s):
        await orig(0)
    asyncio.sleep = rapido
    try:
        g = asyncio.run(correr())
    finally:
        asyncio.sleep = orig
    assert g.motivo_fin == "el mapa ya no crece"
