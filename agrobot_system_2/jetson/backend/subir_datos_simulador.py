"""Datos recolectados por el robot real -> mundo del simulador (editor de terrenos).

Toma lo que el robot reunió en una salida —el mapa del SLAM (PGM/YAML), el recorrido (waypoints), las plantas encontradas y escaneadas, el
punto de inicio— y arma un mundo `real_<nombre>.json` en `simulador_pasivo/mundos/` con el MISMO formato que lee el simulador:
  paredes (segmentos vectorizados con digitalizar_mapa), sendero (recorrido simplificado), plantas, escaneos, límites e inicio.
Además guarda un paquete crudo en `~/.agrobot/datos_simulador/<nombre>/` (mapa, plantas, recorrido, plano digitalizado, mundo) para no perder nada.
Desde el simulador el mundo se abre en el editor de terrenos, se corrige (paredes, plantas, sendero) y se usa para simular y entrenar.
Sin ROS ni red: solo archivos.
"""
from __future__ import annotations

import json
import math
import shutil
import time
from pathlib import Path

R_PLANTA = 0.12
DESTINO_PAQUETES = Path.home() / ".agrobot" / "datos_simulador"


def simplificar_recorrido(waypoints: list[dict], paso_m: float = 0.30) -> list[list[float]]:
    """Waypoints {x, y, theta_deg} -> polilínea [[x, y]] con un punto cada >= paso_m (el último siempre entra)."""
    pts: list[list[float]] = []
    for w in waypoints:
        try:
            x, y = float(w["x"]), float(w["y"])
        except (KeyError, TypeError, ValueError):
            continue
        if not pts or math.hypot(x - pts[-1][0], y - pts[-1][1]) >= paso_m:
            pts.append([round(x, 3), round(y, 3)])
    if waypoints and pts:
        try:
            ult = [round(float(waypoints[-1]["x"]), 3), round(float(waypoints[-1]["y"]), 3)]
            if ult != pts[-1]:
                pts.append(ult)
        except (KeyError, TypeError, ValueError):
            pass
    return pts


def construir_mundo(nombre: str, muros: list[list[float]], waypoints: list[dict], plantas: list[dict], extra: dict | None = None) -> dict:
    """Mundo en el formato de `simulador_pasivo/mundo.py` (campos extra se ignoran al cargarlo)."""
    sendero = simplificar_recorrido(waypoints)
    pl = [[round(float(p["x"]), 3), round(float(p["y"]), 3), R_PLANTA, 0, 0.30] for p in plantas if p.get("x") is not None and p.get("y") is not None]
    xs = [v for m in muros for v in (m[0], m[2])] + [p[0] for p in sendero] + [p[0] for p in pl]
    ys = [v for m in muros for v in (m[1], m[3])] + [p[1] for p in sendero] + [p[1] for p in pl]
    marg = 0.8
    limites = [round(min(xs) - marg, 2), round(min(ys) - marg, 2), round(max(xs) + marg, 2), round(max(ys) + marg, 2)] if xs else [-6, -6, 6, 6]
    mundo = {"nombre": nombre, "paredes": [[round(v, 3) for v in m] for m in muros], "sendero": sendero, "ancho_sendero": 0.8, "ancho_paja": 0.0,
             "plantas": pl, "escaneos": [[p[0], p[1]] for p in pl], "seco": 0.7, "borde": "paja", "superficie": "tierra", "objetos": [],
             "limites": limites, "entrenar": False, "origen": "datos reales del robot (SLAM + recorrido + plantas), subidos desde el dashboard",
             "inicio": sendero[0] if sendero else [0.0, 0.0], "regreso_al_inicio": True, "fecha": time.strftime("%Y-%m-%d %H:%M:%S")}
    if extra:
        mundo.update(extra)
    return mundo


def exportar(nombre: str, carpeta_mapa: Path, plantas: list[dict], waypoints: list[dict], mundos_dir: Path,
             paquetes_dir: Path = DESTINO_PAQUETES, extra: dict | None = None) -> dict:
    """Digitaliza el mapa de `carpeta_mapa` (mapa.yaml + mapa.pgm), arma el mundo y lo escribe en `mundos_dir` y en el paquete crudo."""
    import digitalizar_mapa
    muros: list[list[float]] = []
    plano = None
    yaml_path = Path(carpeta_mapa) / "mapa.yaml"
    paquete = Path(paquetes_dir) / nombre
    paquete.mkdir(parents=True, exist_ok=True)
    if yaml_path.exists():
        plano = digitalizar_mapa.digitalizar(yaml_path, paquete / "digital")
        muros = plano["muros"]
        for f in ("mapa.yaml", "mapa.pgm"):
            if (Path(carpeta_mapa) / f).exists():
                shutil.copy(Path(carpeta_mapa) / f, paquete / f)
    mundo = construir_mundo(f"real_{nombre}", muros, waypoints, plantas, extra)
    (paquete / "plantas.json").write_text(json.dumps(plantas, indent=1, ensure_ascii=False))
    (paquete / "waypoints.json").write_text(json.dumps(waypoints, indent=1))
    (paquete / "mundo.json").write_text(json.dumps(mundo, indent=1, ensure_ascii=False))
    mundos_dir = Path(mundos_dir)
    mundos_dir.mkdir(parents=True, exist_ok=True)
    (mundos_dir / f"real_{nombre}.json").write_text(json.dumps(mundo, indent=1, ensure_ascii=False))
    return {"mundo": f"real_{nombre}", "ruta_mundo": str(mundos_dir / f"real_{nombre}.json"), "paquete": str(paquete),
            "muros": len(muros), "plantas": len(mundo["plantas"]), "puntos_sendero": len(mundo["sendero"]),
            "mapa_digitalizado": plano is not None, "limites": mundo["limites"]}
