"""Perfiles de modelo de visión INTERIOR / EXTERIOR, intercambiables con un botón.

Solo mapea perfil -> archivo de `models/`; la carga real la hace
`/api/models/load` (la misma de siempre, con su validación de tarea 'detect').
Nunca cambia el modelo activo por sí mismo.
"""
from pathlib import Path

PRESETS = {
    "interior": {"archivo": "agrobot_interior_obj_yolo11n.pt",
                 "titulo": "Interior (macetas/obstáculos)",
                 "nota": "YOLO11n entrenado con simulador sintético; sin validar en cámara real."},
    "col": {"archivo": "agrobot_col_yolo11n.pt",
            "titulo": "Col (planta y cogollo)",
            "nota": "Detecta col y cogollo (mAP50 0,71/0,77, otras fotos); el daño de hoja casi no (0,09). NO detecta mosca blanca."},
    "exterior": {"archivo": "ip102_pest_yolo11s.pt",
                 "titulo": "Exterior (plagas)",
                 "nota": "Modelo de plagas previo (IP102)."},
}


def listar(models_dir: Path, activo: str) -> list[dict]:
    return [{"id": k, "titulo": v["titulo"], "archivo": v["archivo"], "nota": v["nota"],
             "disponible": (Path(models_dir) / v["archivo"]).is_file(),
             "activo": v["archivo"] == activo} for k, v in PRESETS.items()]
