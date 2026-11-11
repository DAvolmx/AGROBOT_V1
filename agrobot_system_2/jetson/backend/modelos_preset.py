"""Perfiles de modelo de visión INTERIOR / EXTERIOR, intercambiables con un botón.

Solo mapea perfil -> archivo de `models/`; la carga real la hace
`/api/models/load` (la misma de siempre, con su validación de tarea 'detect').
Nunca cambia el modelo activo por sí mismo.
"""
from pathlib import Path

# 2026-11-11: cada perfil puede traer su ajuste de FILTROS de detección (sección `detection` de la config), que se aplica al cargar
# su modelo. Los filtros estaban pensados para coles: con ARANDANOS descartaban todo (los frutos azules no pasan el filtro de
# «verde» y `plaga_solo_en_col` exige que CLIP vea una col). Cambiar de modelo deja siempre los filtros que le corresponden.
_CLASES_PLANTA_BASE = ["col_planta", "col_cogollo", "maceta_planta", "maceta", "planta"]
DETECCION_COLES = {"clases_planta": _CLASES_PLANTA_BASE, "plaga_solo_en_col": True, "plaga_usar_plantas": True,
                   "confidence_threshold": 0.3, "imgsz": 480,
                   "plaga_caja_max_frac_descartar": 0}   # PLAGASX da cajas de imagen completa: no se filtran por tamaño
DETECCION_ARANDANO = {
    "clases_planta": _CLASES_PLANTA_BASE + ["arandano_maduro", "arandano_inmaduro"],  # frutos: sin filtro de verde/textura
    "plaga_solo_en_col": False,      # en el arándano no hay col: el filtro de especie suprimía todas las plagas
    "plaga_usar_plantas": False,     # no corre el detector de coles (ahorra GPU)
    "confidence_threshold": 0.35,    # el umbral con el que se midió (0/40 hojas sanas con falsa alarma)
    "imgsz": 960,                    # la escala de entrenamiento: los frutos chicos crecen en la imagen (~125 ms en la Jetson)
    "plaga_caja_max_frac_descartar": 0.25,  # plaga > 25 % del cuadro = falsa (vivero 11-nov: todas ≥ 47 %)
}

PRESETS = {
    "interior": {"archivo": "agrobot_interior_obj_yolo11n.pt",
                 "titulo": "Interior (macetas/obstáculos)",
                 "nota": "YOLO11n entrenado con simulador sintético; sin validar en cámara real."},
    "col": {"archivo": "agrobot_col_yolo11n.pt",
            "titulo": "Col (planta y cogollo)",
            "nota": "Detecta col y cogollo (mAP50 0,71/0,77, otras fotos); el daño de hoja casi no (0,09). NO detecta mosca blanca."},
    "plagasx": {"archivo": "agrobot_plagasx_det.pt",
                "titulo": "Coles: enfermedades (PLAGASX)",
                "nota": "YOLOv8s entrenado en Kaggle (2026-10-21) con coles propias + datasets externos de coliflor/papa: sana, amarillamiento, "
                        "manchas café, daño de insecto, mildiu y malformación (pudrición no tiene ejemplos). Las cajas son de IMAGEN COMPLETA: "
                        "dice QUÉ enfermedad ve, no DÓNDE está la lesión. Clasificador en agrobot_plagasx_cls.pt (93 % en su test; sin validar en campo).",
                "deteccion": DETECCION_COLES},
    "arandano": {"archivo": "agrobot_arandanos_det.pt",
                 "titulo": "Arándano: frutos y plagas (ARANDANOS)",
                 "nota": "YOLO11s entrenado en Kaggle (2026-11-11) con fotos públicas: arándano maduro/inmaduro con cajas reales y 8 plagas "
                         "del IP102 (pulgón, trips, arañita roja, chanchito blanco, conchuela, oruga, mosca blanca, mosca de la fruta). "
                         "Validación mAP50 0,79; frutos en test: maduro 0,91, inmaduro 0,71 (el rosado cuenta como inmaduro). "
                         "0/40 hojas sanas con falsa alarma. ~170 ms/img en la Jetson. "
                         "Ninguna foto es del invernadero propio; NO detecta Drosophila suzukii. Sin validar en campo.",
                 "deteccion": DETECCION_ARANDANO},
    "exterior": {"archivo": "ip102_pest_yolo11s.pt",
                 "titulo": "Exterior (plagas IP102)",
                 "nota": "Modelo de plagas previo (IP102, 102 insectos). En arándano sirve mientras no esté ARANDANOS: "
                         "aphids/Thrips/red spider/Pseudococcus/whitefly son plagas del arándano; el resto de clases son de otros cultivos."},
}


def perfil_deteccion(archivo: str) -> dict | None:
    """Ajuste de filtros de detección del perfil cuyo modelo es `archivo` (None si no tiene)."""
    for v in PRESETS.values():
        if v["archivo"] == archivo and v.get("deteccion"):
            return dict(v["deteccion"])
    return None


def listar(models_dir: Path, activo: str) -> list[dict]:
    return [{"id": k, "titulo": v["titulo"], "archivo": v["archivo"], "nota": v["nota"],
             "disponible": (Path(models_dir) / v["archivo"]).is_file(),
             "activo": v["archivo"] == activo} for k, v in PRESETS.items()]
