"""MISIONES ESPECÍFICAS (2026-10-27): tres tareas de campo con un solo selector, sobre las piezas que ya existen (explorador E11, MisionPlantas, visión activa).

  mapear      Explora y construye el mapa del SLAM; al acabar (tiempo, explorador parado o mapa que ya no crece) vuelve al punto de salida
              por el A* de E11 y guarda el mapa con nombre `mapeo_AAAAMMDD_HHMM`.
  dataset     Va despacio por el recorrido; en cada planta que ve se para, la rodea con las vistas del barrido (8 fotos) y las GUARDA como
              dataset (`~/.agrobot/dataset_plantas/<sesión>/` + `manifest.jsonl`: foto, pose, caja, distancia, servo, especie, diagnóstico).
              No juzga las plantas: solo recoge material de entrenamiento.
  inspeccion  Recorre el cultivo y de cada planta emite un VEREDICTO con el clasificador de plagas (sana / sospecha / alerta / malformación /
              revisar a mano) y deja un informe `informe.json` + `informe.md` con las fotos y el mapa de dónde está cada hallazgo.

LÍMITE QUE HAY QUE SABER: el clasificador de plagas (PLAGASX) acierta ~30–36 % con fotos reales de col a varias distancias (validar_zoom_plagas.py). Por eso el
veredicto de la inspección es una SOSPECHA con su probabilidad, no un diagnóstico, y toda planta con duda queda marcada «revisar a mano» con su foto.

Lógica con dependencias inyectadas y reloj inyectable (se prueba sin robot). La acción física la hacen MisionPlantas / el explorador; este módulo elige el perfil,
recoge lo que producen y escribe los resultados."""
from __future__ import annotations

import asyncio
import json
import shutil
import time
from pathlib import Path

MAPEAR, DATASET, INSPECCION = "mapear", "dataset", "inspeccion"

# Ajustes de MisionPlantas / visión activa por misión. Se aplican SOLO mientras dura y se retiran al acabar (no tocan la configuración guardada).
PERFILES = {
    MAPEAR: {
        "titulo": "Explorar y mapear",
        "descripcion": "Recorre el lugar, construye el mapa del SLAM, vuelve al punto de salida y lo guarda.",
        "opciones": {"tiempo_max_s": 600.0, "regresar": True, "guardar_mapa": True, "sin_crecer_s": 120.0, "regreso_max_s": 240.0},
    },
    DATASET: {
        "titulo": "Recoger dataset de plantas",
        "descripcion": "Va despacio, se para en cada planta y guarda sus fotos con metadatos para entrenar.",
        "opciones": {"tiempo_max_s": 1500.0, "max_plantas": 40, "vistas": ["arriba", "abajo", "izquierda", "volver_izquierda", "derecha", "volver_derecha"]},
        "mision_plantas": {"max_plantas": 40, "tiempo_max_s": 1500.0, "persistencia": 4, "pausa_tras_planta_s": 8.0},
        "vision": {"distancia_inspeccion_m": 0.55, "max_objetivos": 1},
    },
    INSPECCION: {
        "titulo": "Inspección de plagas y malformaciones",
        "descripcion": "Recorre el cultivo, revisa cada planta con el clasificador de plagas y deja un informe con las sospechas.",
        "opciones": {"tiempo_max_s": 1500.0, "max_plantas": 40, "umbral_alerta": 0.55, "umbral_sospecha": 0.30},
        "mision_plantas": {"max_plantas": 40, "tiempo_max_s": 1500.0, "persistencia": 3, "pausa_tras_planta_s": 6.0},
        "vision": {"distancia_inspeccion_m": 0.55, "max_objetivos": 1},
    },
}

CLASE_SANA = "col_sana"
PROBLEMAS = {"col_amarillamiento": "amarillamiento", "col_dano_insecto": "daño de insecto", "col_malformacion": "malformación",
             "col_manchas_cafe": "manchas café", "col_mildiu_velloso": "mildiu velloso"}
SANA, SOSPECHA, ALERTA, MALFORMACION, REVISAR = "sana", "sospecha", "alerta", "malformacion", "revisar a mano"


def veredicto(diag: dict | None, umbral_alerta: float = 0.55, umbral_sospecha: float = 0.30) -> dict:
    """Veredicto de una planta a partir del diagnóstico de `clasificar` (top-3 de PLAGASX + especie CLIP).
    Sin diagnóstico o sin clase clara -> «revisar a mano». Una planta que CLIP no reconoce como col también se revisa a mano."""
    if not diag or not diag.get("top"):
        return {"estado": REVISAR, "motivo": "sin diagnóstico", "clase": None, "prob": None}
    top = diag["top"]
    clase, prob = top[0]["clase"], float(top[0]["prob"])
    esp = diag.get("especie") or {}
    if esp.get("p_col") is not None and float(esp["p_col"]) < 0.35:
        return {"estado": REVISAR, "motivo": f"no parece col (p_col {float(esp['p_col']):.2f})", "clase": clase, "prob": round(prob, 3)}
    if clase == CLASE_SANA:
        if prob >= umbral_alerta:
            return {"estado": SANA, "motivo": "", "clase": clase, "prob": round(prob, 3)}
        segunda = next((t for t in top[1:] if t["clase"] in PROBLEMAS and float(t["prob"]) >= umbral_sospecha), None)
        if segunda:
            return {"estado": SOSPECHA, "motivo": f"{PROBLEMAS[segunda['clase']]} ({float(segunda['prob']):.2f}) con «sana» dudosa", "clase": segunda["clase"],
                    "prob": round(float(segunda["prob"]), 3)}
        return {"estado": REVISAR, "motivo": f"«sana» solo {prob:.2f}", "clase": clase, "prob": round(prob, 3)}
    if clase in PROBLEMAS:
        nombre = PROBLEMAS[clase]
        if prob >= umbral_alerta:
            return {"estado": MALFORMACION if clase == "col_malformacion" else ALERTA, "motivo": nombre, "clase": clase, "prob": round(prob, 3)}
        if prob >= umbral_sospecha:
            return {"estado": SOSPECHA, "motivo": f"{nombre} ({prob:.2f})", "clase": clase, "prob": round(prob, 3)}
    return {"estado": REVISAR, "motivo": f"{clase} con {prob:.2f}", "clase": clase, "prob": round(prob, 3)}


class GestorMisiones:
    def __init__(self, *, carpeta: Path, mision_plantas, explorador, vision, aplicar_ajustes, quitar_ajustes, guardar_mapa, get_pose,
                 mapa_celdas=None, on_event=None, reloj=time.time):
        self.carpeta = Path(carpeta)
        self.mp = mision_plantas            # MisionPlantas: iniciar() -> (ok,msg), detener(m), arrancar_tarea(), .estado, .plantas
        self.expl = explorador              # iniciar() -> (ok,msg), detener(), iniciar_regreso(), .activo, .regreso_estado
        self.vision = vision                # .ultimo_resultado
        self._aplicar = aplicar_ajustes     # (mision_plantas: dict, vision: dict) -> None
        self._quitar = quitar_ajustes       # () -> None
        self._guardar_mapa = guardar_mapa   # async (nombre) -> dict   (guarda el mapa del SLAM)
        self._pose = get_pose
        self._celdas = mapa_celdas          # () -> int | None  (celdas conocidas del mapa, para saber si sigue creciendo)
        self._ev = on_event or (lambda e: None)
        self.reloj = reloj
        self.mision: str | None = None
        self.estado = "inactiva"            # inactiva | en curso | terminando | fin
        self.motivo_fin: str | None = None
        self.sesion: str | None = None
        self.opciones: dict = {}
        self.hallazgos: list[dict] = []
        self.fotos_guardadas = 0
        self._t0 = 0.0
        self._tarea: asyncio.Task | None = None

    # ---------------- catálogo y estado
    @staticmethod
    def catalogo() -> dict:
        return {k: {"titulo": v["titulo"], "descripcion": v["descripcion"], "opciones": v["opciones"]} for k, v in PERFILES.items()}

    def estado_dict(self) -> dict:
        return {"mision": self.mision, "estado": self.estado, "motivo_fin": self.motivo_fin, "sesion": self.sesion,
                "transcurrido_s": round(self.reloj() - self._t0, 1) if self.estado in ("en curso", "terminando") else None,
                "plantas": len(self.mp.plantas) if self.mision in (DATASET, INSPECCION) else None,
                "fotos_guardadas": self.fotos_guardadas, "hallazgos": self.hallazgos[-30:], "catalogo": self.catalogo()}

    # ---------------- ciclo de vida
    def iniciar(self, mision: str, opciones: dict | None = None) -> tuple[bool, str]:
        if mision not in PERFILES:
            return False, f"misión desconocida «{mision}» (válidas: {', '.join(PERFILES)})"
        if self.estado in ("en curso", "terminando"):
            return False, f"ya hay una misión en curso ({self.mision})"
        perfil = PERFILES[mision]
        self.opciones = {**perfil["opciones"], **(opciones or {})}
        self.mision, self.motivo_fin, self.hallazgos, self.fotos_guardadas = mision, None, [], 0
        self.sesion = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.reloj()))
        self._t0 = self.reloj()
        if mision == MAPEAR:
            ok, msg = self.expl.iniciar()
            if not ok and not getattr(self.expl, "activo", False):
                self.mision = None
                return False, f"no arrancó la exploración: {msg}"
            self.estado = "en curso"
            self._tarea = asyncio.create_task(self._bucle_mapear())
        else:
            mp_cfg = {**perfil.get("mision_plantas", {})}
            for k in ("max_plantas", "tiempo_max_s"):
                if k in self.opciones:
                    mp_cfg[k] = self.opciones[k]
            self._aplicar(mp_cfg, dict(perfil.get("vision", {})))
            ok, msg = self.mp.iniciar()
            if not ok:
                self._quitar()
                self.mision = None
                return False, msg
            self.mp.arrancar_tarea()
            self.estado = "en curso"
            (self.carpeta / ("dataset_plantas" if mision == DATASET else "inspecciones") / self.sesion).mkdir(parents=True, exist_ok=True)
        self._ev({"tipo": "mision_especifica_inicio", "mision": mision, "sesion": self.sesion})
        return True, "ok"

    def detener(self, motivo: str = "detenida") -> None:
        if self.estado not in ("en curso", "terminando"):
            return
        if self._tarea is not None and not self._tarea.done():
            self._tarea.cancel()
        if self.mision in (DATASET, INSPECCION):
            self.mp.detener(motivo)
        else:
            self.expl.detener()
        self._cerrar(motivo)

    def _cerrar(self, motivo: str):
        if self.estado == "fin":
            return                              # detener() -> MisionPlantas.detener() -> evento de fin -> aquí otra vez
        self.estado, self.motivo_fin = "fin", motivo
        self._quitar()
        if self.mision == INSPECCION:
            try:
                self._escribir_informe()
            except Exception as exc:
                self._ev({"tipo": "aviso", "msg": f"no se pudo escribir el informe: {exc}"})
        self._ev({"tipo": "mision_especifica_fin", "mision": self.mision, "motivo": motivo, "hallazgos": len(self.hallazgos), "fotos": self.fotos_guardadas})

    # ---------------- mapear
    async def _bucle_mapear(self):
        opt = self.opciones
        ult_celdas, t_crecio = None, self.reloj()
        try:
            while True:
                await asyncio.sleep(1.0)
                ahora = self.reloj()
                if ahora - self._t0 > opt["tiempo_max_s"]:
                    motivo = "tiempo máximo"
                    break
                if not getattr(self.expl, "activo", True):
                    motivo = "el explorador se detuvo"
                    break
                if self._celdas is not None:
                    n = self._celdas()
                    if n is not None and (ult_celdas is None or n > ult_celdas * 1.01):
                        ult_celdas, t_crecio = n, ahora
                    elif ahora - t_crecio > opt["sin_crecer_s"]:
                        motivo = "el mapa ya no crece"
                        break
            self.estado = "terminando"
            if opt.get("regresar"):
                ok, msg = self.expl.iniciar_regreso()
                t_reg = self.reloj()
                while ok and self.expl.regreso_estado == "en curso" and self.reloj() - t_reg < opt["regreso_max_s"] and getattr(self.expl, "activo", True):
                    await asyncio.sleep(1.0)
                self._ev({"tipo": "mapeo_regreso", "estado": self.expl.regreso_estado if ok else f"no pudo: {msg}"})
            self.expl.detener()
            if opt.get("guardar_mapa"):
                nombre = time.strftime("mapeo_%Y%m%d_%H%M", time.localtime(self.reloj()))
                info = await self._guardar_mapa(nombre)
                self._ev({"tipo": "mapa_guardado", "nombre": (info or {}).get("nombre", nombre)})
            self._cerrar(motivo)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.expl.detener()
            self._cerrar(f"error: {type(exc).__name__}: {exc}")

    # ---------------- dataset e inspección: lo llama el evento «planta_inspeccionada» de MisionPlantas
    def al_planta(self, planta: dict, resultado: dict | None) -> None:
        if self.estado != "en curso" or self.mision not in (DATASET, INSPECCION):
            return
        res = resultado or {}
        sub = self.carpeta / ("dataset_plantas" if self.mision == DATASET else "inspecciones") / self.sesion
        fotos = [v.get("foto") for v in (res.get("vistas") or []) if v.get("foto")] or ([res["foto"]] if res.get("foto") else [])
        copiadas = []
        for k, f in enumerate(fotos):
            try:
                dest = sub / f"planta{planta['n']:03d}_v{k + 1}{Path(f).suffix or '.jpg'}"
                shutil.copy2(f, dest)
                copiadas.append(dest.name)
            except OSError:
                continue
        self.fotos_guardadas += len(copiadas)
        reg = {"n": planta["n"], "t": round(self.reloj(), 2), "x": planta.get("x"), "y": planta.get("y"), "clase_detector": res.get("clase"),
               "conf": res.get("conf_media"), "dist_cam_m": res.get("dist_cam_m"), "yaw_deg": res.get("yaw_deg"), "servo": res.get("servo"),
               "pose": res.get("pose"), "especie": planta.get("especie"), "p_col": planta.get("p_col"), "fotos": copiadas,
               "diagnostico": res.get("diagnostico")}
        if self.mision == INSPECCION:
            v = veredicto(res.get("diagnostico"), self.opciones["umbral_alerta"], self.opciones["umbral_sospecha"])
            reg["veredicto"] = v
            self.hallazgos.append({"n": planta["n"], "x": planta.get("x"), "y": planta.get("y"), **v, "fotos": copiadas})
            self._ev({"tipo": "inspeccion_planta", "n": planta["n"], **v})
        with open(sub / "manifest.jsonl", "a", encoding="utf-8") as fh:
            fh.write(json.dumps(reg, ensure_ascii=False) + "\n")

    def al_fin_mision_plantas(self, motivo: str | None = None) -> None:
        """MisionPlantas terminó por sí sola (tiempo, nº de plantas, error o mando manual): se cierra la misión específica."""
        if self.estado == "en curso" and self.mision in (DATASET, INSPECCION):
            self._cerrar(motivo or "terminada")

    # ---------------- informe
    def _escribir_informe(self):
        sub = self.carpeta / "inspecciones" / self.sesion
        sub.mkdir(parents=True, exist_ok=True)
        cuenta: dict[str, int] = {}
        for h in self.hallazgos:
            cuenta[h["estado"]] = cuenta.get(h["estado"], 0) + 1
        data = {"sesion": self.sesion, "motivo_fin": self.motivo_fin, "plantas": len(self.hallazgos), "resumen": cuenta, "umbrales": {
            "alerta": self.opciones["umbral_alerta"], "sospecha": self.opciones["umbral_sospecha"]}, "hallazgos": self.hallazgos,
            "aviso": "El clasificador de plagas acierta ~30-36 % con fotos reales: toda sospecha o duda debe confirmarse a mano."}
        (sub / "informe.json").write_text(json.dumps(data, indent=1, ensure_ascii=False))
        lineas = [f"# Inspección {self.sesion}", "", f"Plantas revisadas: **{len(self.hallazgos)}** · terminó por: {self.motivo_fin}", "",
                  "Resumen: " + (", ".join(f"{k}: {v}" for k, v in sorted(cuenta.items())) or "sin plantas"), "",
                  "> " + data["aviso"], "", "| # | posición | estado | detalle | probabilidad | fotos |", "|---|---|---|---|---|---|"]
        for h in self.hallazgos:
            pos = "?" if h.get("x") is None else f"({h['x']:.2f}; {h['y']:.2f})"
            lineas.append(f"| {h['n']} | {pos} | {h['estado']} | {h.get('motivo') or ''} | {h.get('prob') if h.get('prob') is not None else ''} | "
                          f"{', '.join(h.get('fotos') or [])} |")
        (sub / "informe.md").write_text("\n".join(lineas) + "\n")
