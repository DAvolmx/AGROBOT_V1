#!/usr/bin/env python3
"""SLAM en el SIMULADOR CENTRAL (2026-10-22): el mismo SLAM del robot (filtros de scan + rf2o + EKF + slam_toolbox) evaluado con datos REALES
reproducidos y con configuraciones candidatas, para dejarlo bien en interiores antes de pasar a exteriores.

  python3 simular_slam.py [--grabacion ~/.agrobot/campo/<sesion>] [--configs actual,caja,caja_interior] [--velocidad 2] [--desde 0] [--hasta 1e9]

Por cada configuración:
  1. lanza slam_ros/replay.launch.py en un ROS_DOMAIN_ID propio (no toca el ROS vivo del robot) con reloj simulado;
  2. slam_ros/reproducir.py publica los scans y el IMU grabados y guarda poses del SLAM, odometría del EKF y el mapa;
  3. se mide: saltos de pose (imposibles a 5 Hz con el robot real), celdas ocupadas del mapa (menos = paredes más finas) y manchas
     sueltas (ruido/fantasmas), y la divergencia final EKF vs SLAM.
Salida: ~/.agrobot/informes/slam_<fecha>/ con mapas PNG, trayectorias, resumen.json e informe.md.

Configuraciones:
  actual          los filtros y slam_toolbox_mapping.yaml del robot, tal cual
  caja            + filtro de caja que quita el propio chasis del scan (el LIDAR ve a 15-20 cm a -35° y la trasera a 30 cm en TODOS los scans)
  caja_interior   + slam_toolbox más fino para salas: actualiza cada 5 cm / 0,1 rad, búsqueda de bucles 3 m, respuesta mínima 0,1
SIN verdad terreno: con datos reales se mide consistencia, no exactitud. La exactitud la da el simulador de física (generar_dataset_slam.py).
"""
import argparse
import glob
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
ROS_WS = AQUI.parent / "ros2_ws"
CFG_ROBOT = ROS_WS / "src" / "agrobot_navigation" / "config" / "slam_toolbox_mapping.yaml"
DOMINIO = "77"
AJUSTES_INTERIOR = {"minimum_travel_distance": 0.05, "minimum_travel_heading": 0.10, "loop_search_maximum_distance": 3.0,
                    "link_match_minimum_response_fine": 0.10}
# "ayer": slam_toolbox como estaba antes del 2026-10-23 (scans cada 0,2 s y mapa cada 1 s): ¿el mapa se desfasó por haberlos acelerado?
CONFIGS = {"actual": (False, {}), "caja": (True, {}), "caja_interior": (True, AJUSTES_INTERIOR),
           "ayer": (True, {"minimum_time_interval": 0.2, "map_update_interval": 1.0}),
           "ayer_rapido": (True, {"minimum_time_interval": 0.2, "map_update_interval": 1.0, "minimum_travel_heading": 0.35,
                                  "correlation_search_space_dimension": 0.9}),
           # giros rápidos (a 100 % el robot gira ~15° entre dos scans): búsqueda angular más ancha, menos castigo al ángulo
           "giro_ancho": (True, {"coarse_search_angle_offset": 0.52, "angle_variance_penalty": 0.6, "minimum_travel_heading": 0.10}),
           "busca_ancha": (True, {"correlation_search_space_dimension": 0.9, "link_scan_maximum_distance": 2.5,
                                  "coarse_search_angle_offset": 0.52, "angle_variance_penalty": 0.6})}


def yaml_con(ajustes: dict, destino: Path) -> Path:
    """Copia el yaml del robot cambiando solo las claves pedidas (texto: conserva comentarios y orden)."""
    lineas = CFG_ROBOT.read_text().splitlines()
    out = []
    for l in lineas:
        k = l.strip().split(":")[0]
        if k in ajustes and not l.strip().startswith("#"):
            sangria = l[: len(l) - len(l.lstrip())]
            l = f"{sangria}{k}: {ajustes[k]}"
        out.append(l)
    destino.write_text("\n".join(out) + "\n")
    return destino


def correr(nombre, caja, ajustes, grabacion, carpeta, a):
    salida = carpeta / f"{nombre}.npz"
    yaml = yaml_con(ajustes, carpeta / f"slam_{nombre}.yaml")
    ros = f"source /opt/ros/jazzy/setup.bash && source {ROS_WS}/install/setup.bash"
    env = dict(os.environ, ROS_DOMAIN_ID=DOMINIO, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    lanz = subprocess.Popen(["bash", "-c", f"{ros} && exec ros2 launch {AQUI / 'slam_ros' / 'replay.launch.py'} caja:={str(caja).lower()} slam_yaml:={yaml} laser_girado:={str(not a.laser_robot).lower()}"],
                            env=env, stdout=open(carpeta / f"{nombre}_launch.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(7)
        subprocess.run(["bash", "-c", f"{ros} && exec python3 {AQUI / 'slam_ros' / 'reproducir.py'} {grabacion / 'variables.jsonl'} {salida} "
                        f"--velocidad {a.velocidad} --desde {a.desde} --hasta {a.hasta}" + (" --laser-robot" if a.laser_robot else "")], env=env, check=True)
    finally:
        os.killpg(lanz.pid, signal.SIGINT)
        try:
            lanz.wait(10)
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(lanz.pid, signal.SIGKILL)          # que no quede ningún nodo de la reproducción vivo
        except ProcessLookupError:
            pass
    return salida


def medir(npz: Path) -> dict:
    import cv2
    d = np.load(npz)
    P, E, M = d["poses"], d["ekf"], d["mapa"]
    r = {"poses": int(len(P))}
    if len(P) > 2:
        dxy = np.hypot(np.diff(P[:, 1]), np.diff(P[:, 2]))
        dth = np.degrees(np.abs((np.diff(P[:, 3]) + np.pi) % (2 * np.pi) - np.pi))
        r["saltos_pose"] = int(((dxy > 0.15) | (dth > 8)).sum())
        r["recorrido_m"] = round(float(dxy[dxy < 0.15].sum()), 1)
    if M.size:
        ocup, libre = (M >= 65), (M == 0)
        n, _, st, _ = cv2.connectedComponentsWithStats(ocup.astype(np.uint8), connectivity=8)
        r.update(celdas_ocupadas=int(ocup.sum()), celdas_libres=int(libre.sum()),
                 ocupadas_por_libre=round(float(ocup.sum()) / max(1, libre.sum()), 4),
                 manchas_sueltas=int(((st[1:, cv2.CC_STAT_AREA] <= 2)).sum()) if n > 1 else 0)
    if len(P) > 2:                                       # cierre: si el recorrido vuelve a la marca de salida, esto es la deriva acumulada
        r["fin_vs_inicio_cm"] = round(float(np.hypot(P[-1, 1] - P[0, 1], P[-1, 2] - P[0, 2])) * 100, 1)
        r["fin_vs_inicio_deg"] = round(float(np.degrees((P[-1, 3] - P[0, 3] + np.pi) % (2 * np.pi) - np.pi)), 1)
    if len(P) and len(E):
        r["ekf_vs_slam_final_m"] = round(float(np.hypot(P[-1, 1] - E[-1, 1], P[-1, 2] - E[-1, 2])), 2)
    return r


def coherencia_avance(npz: Path, grabacion: Path) -> dict:
    """¿La pose avanza hacia donde el LIDAR dice que se acerca lo de delante? Correlación entre el avance de la pose (SLAM y EKF) en 1 s
    y lo que se acorta la distancia frontal medida (±15°). Debe salir POSITIVA (≈ +0,8); negativa = odometría con el signo al revés
    (lo que pasó hasta el 2026-10-23 con rf2o)."""
    import json as _j
    t, fr = [], []
    for l in open(grabacion / "variables.jsonl", errors="ignore"):
        try:
            d = _j.loads(l)
        except ValueError:
            continue
        sc = d.get("lidar_scan") or []
        f = [p["distance_m"] for p in sc if p.get("distance_m", 0) > 0.05 and abs((p["angle_deg"] + 180) % 360 - 180) <= 15]
        if f and d.get("t_unix"):
            t.append(d["t_unix"]); fr.append(float(np.median(f)))
    t, fr = np.array(t), np.array(fr)
    d = np.load(npz)
    out = {}
    for nombre in ("poses", "ekf"):
        P = d[nombre]
        if len(P) < 20 or len(t) < 20:
            continue
        x, y, th = (np.interp(t, P[:, 0], P[:, i]) for i in (1, 2, 3))
        k = 5
        av = np.cos(th[:-k]) * (x[k:] - x[:-k]) + np.sin(th[:-k]) * (y[k:] - y[:-k])
        dfr = fr[k:] - fr[:-k]
        m = (np.abs(av) > 0.02) & (np.abs(av) < 0.5) & (np.abs(dfr) < 0.5) & (t[:-k] > P[0, 0]) & (t[k:] < P[-1, 0])
        if m.sum() > 10:
            out[f"coherencia_{nombre}"] = round(float(np.corrcoef(av[m], -dfr[m])[0, 1]), 2)
    return out


def dibujar(npz: Path, png: Path, titulo: str):
    import cv2
    d = np.load(npz)
    M, P = d["mapa"], d["poses"]
    if not M.size:
        return
    img = np.full(M.shape, 205, np.uint8)
    img[M == 0] = 255
    img[M >= 65] = 0
    img = cv2.cvtColor(np.flipud(img), cv2.COLOR_GRAY2BGR)
    res, org = float(d["resolucion"]), d["origen"]
    for p in P:
        u, v = int((p[1] - org[0]) / res), M.shape[0] - 1 - int((p[2] - org[1]) / res)
        if 0 <= u < img.shape[1] and 0 <= v < img.shape[0]:
            img[v, u] = (0, 0, 255)
    k = max(1, int(600 / max(img.shape[:2])))
    img = cv2.resize(img, (img.shape[1] * k, img.shape[0] * k), interpolation=cv2.INTER_NEAREST)
    cv2.putText(img, titulo, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 120, 0), 1)
    cv2.imwrite(str(png), img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grabacion", default=None, help="carpeta de ~/.agrobot/campo con variables.jsonl (por defecto la última)")
    ap.add_argument("--configs", default="actual,caja,caja_interior")
    ap.add_argument("--velocidad", type=float, default=2.0)
    ap.add_argument("--desde", type=float, default=0.0)
    ap.add_argument("--hasta", type=float, default=1e9)
    ap.add_argument("--laser-robot", action="store_true", help="modo antiguo: scans ya en el marco del robot (escondía el vx invertido de rf2o)")
    a = ap.parse_args()
    if a.grabacion:
        grab = Path(a.grabacion).expanduser()
    else:
        cand = [Path(p).parent for p in sorted(glob.glob(str(Path.home() / ".agrobot" / "campo" / "2026*" / "variables.jsonl")))]
        grab = cand[-1]
    carpeta = Path.home() / ".agrobot" / "informes" / f"slam_{time.strftime('%Y%m%d_%H%M')}"
    carpeta.mkdir(parents=True, exist_ok=True)
    print(f"[simular_slam] grabación {grab.name} -> {carpeta}", flush=True)
    resumen = {}
    for nombre in a.configs.split(","):
        caja, ajustes = CONFIGS[nombre]
        t0 = time.time()
        npz = correr(nombre, caja, ajustes, grab, carpeta, a)
        resumen[nombre] = {**medir(npz), **coherencia_avance(npz, grab), "segundos": int(time.time() - t0)}
        dibujar(npz, carpeta / f"mapa_{nombre}.png", nombre)
        print(f"[simular_slam] {nombre}: {resumen[nombre]}", flush=True)
    (carpeta / "resumen.json").write_text(json.dumps({"grabacion": str(grab), "configs": resumen}, indent=1, ensure_ascii=False))
    filas = ["| config | poses | saltos de pose | celdas ocupadas | ocupadas/libres | manchas sueltas | EKF vs SLAM final (m) |", "|---|---|---|---|---|---|---|"]
    for n, r in resumen.items():
        filas.append(f"| {n} | {r.get('poses')} | {r.get('saltos_pose')} | {r.get('celdas_ocupadas')} | {r.get('ocupadas_por_libre')} | "
                     f"{r.get('manchas_sueltas')} | {r.get('ekf_vs_slam_final_m')} |")
    (carpeta / "informe.md").write_text(f"# SLAM reproducido con datos reales ({grab.name})\n\n" + "\n".join(filas) +
                                        "\n\nMenos saltos, menos celdas ocupadas por celda libre (paredes finas) y menos manchas sueltas = mejor. "
                                        "Sin verdad terreno: mide consistencia, no exactitud.\n")
    print("\n".join(filas), flush=True)


if __name__ == "__main__":
    main()
