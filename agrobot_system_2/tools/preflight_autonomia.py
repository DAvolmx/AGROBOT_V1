#!/usr/bin/env python3
"""Lista de verificación ANTES de lanzar Explorar / Patrulla / Nav2. Solo LEE: no mueve nada.

  python3 tools/preflight_autonomia.py [--url http://localhost:8080] [--ros] [--suelo-ok]

  ✓ ok   ⚠ aviso (arranca, pero puede quedarse parado o ir peor)   ✗ BLOQUEA (la autonomía no arrancará o no se moverá)
Sale con código 1 si hay algún ✗. `--ros` añade el estado lifecycle de Nav2/SLAM (tarda ~30 s).
`--suelo-ok`: confirmas que el robot está en el SUELO y no al borde de una mesa (el LIDAR no ve el vacío).
"""
import argparse
import json
import shutil
import subprocess
import sys
import urllib.request

OK, AVISO, FALLO = "✓", "⚠", "✗"
filas = []


def r(estado, tema, detalle):
    filas.append((estado, tema, detalle))


def get(url, ruta):
    return json.load(urllib.request.urlopen(url + ruta, timeout=8))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8080")
    ap.add_argument("--ros", action="store_true")
    ap.add_argument("--suelo-ok", action="store_true")
    a = ap.parse_args()
    try:
        est = get(a.url, "/api/asistente/estado")
        par = get(a.url, "/api/params")
        sal = get(a.url, "/api/health")
        ent = get(a.url, "/api/entorno")
    except Exception as e:
        print(f"{FALLO} backend: no responde en {a.url} ({e}). `sudo systemctl start agrobot-backend`")
        return 1
    r(OK, "backend", "responde")

    # --- enlaces ---
    c3 = est["c3"]
    if not c3["conectada"]:
        r(FALLO, "C3 (motores)", "sin enlace: la autonomía «arranca» pero NO se mueve. ¿Alimentación de la C3? ¿AP JETSON_AP?")
    elif c3.get("failsafe"):
        r(FALLO, "C3 (motores)", "en FAILSAFE: no obedece órdenes hasta recibir tramas válidas")
    else:
        r(OK, "C3 (motores)", "conectada, sin failsafe")
    lid = est["lidar"]
    if lid["edad_scan_s"] > 1.0:
        r(FALLO, "LIDAR", f"scan de hace {lid['edad_scan_s']:.1f} s: el explorador se niega a arrancar (>1 s)")
    else:
        r(OK, "LIDAR", f"scan fresco ({lid['edad_scan_s']*1000:.0f} ms)")
    filas_h = sal if isinstance(sal, list) else (sal.get("filas") or sal.get("items") or sal.get("rows") or [])
    cam = next((x for x in filas_h if x.get("key") == "camera"), None)
    r(OK if cam and cam.get("status") == "ok" else AVISO, "cámara", "ok" if cam and cam.get("status") == "ok" else "sin imagen: sin sendero visual ni plagas")
    if est["pose_mapa"] is None:
        r(FALLO, "pose SLAM", "sin TF map→base_link: la patrulla/Nav2 no tienen pose (¿slam_toolbox?)")
    else:
        p = est["pose_mapa"]
        r(OK, "pose SLAM", f"x={p['x']:.2f} y={p['y']:.2f} θ={p['theta_deg']:.0f}°")
    r(OK if est["ros"] else FALLO, "ROS bridge", "disponible" if est["ros"] else "no disponible: sin Nav2 ni pose")

    # --- bloqueos de configuración ---
    ctl, seg, aut = par["control"], par["safety"], par["autonomia"]
    if ctl.get("solo_manual"):
        r(FALLO, "perfil demo", "control.solo_manual=true: el servidor RECHAZA misión, exploración, evasión, Nav2 y pasos (409)")
    else:
        r(OK, "perfil demo", "solo_manual=false: nada se rechaza por perfil")
    s = est["seguridad"]
    if s.get("enganchado") or s.get("nivel") == "seguro":
        r(FALLO, "modo seguro", f"ENGANCHADO ({'; '.join(s.get('motivos', []))}): para la autonomía. POST /api/seguridad/rearmar")
    else:
        r(OK, "modo seguro", "habilitado" if s.get("modo_seguro_habilitado") else "apagado (no frena nada)")
    if seg.get("freno_lidar_autonomo"):
        r(OK, "freno LIDAR autónomo", "activo: corta el avance a margen+deslizamiento de un obstáculo")
    else:
        r(AVISO, "freno LIDAR autónomo", "APAGADO (tu configuración): la evasión depende solo de la lógica del explorador/Nav2. "
                                          "Recomendado ACTIVO en las primeras pruebas autónomas (safety.freno_lidar_autonomo)")
    if seg.get("supervisor_activo"):
        r(AVISO, "supervisor", "activo: enclava la emergencia y exige rearme tras cada parada")

    # --- coherencia modo / entorno / espacio ---
    modo = ent.get("modo_actual") or ctl.get("modo_manejo")
    sug = ent.get("sugerido")
    avance, lado = aut["avance_libre_cm"], aut["lado_libre_cm"]
    if sug and modo in ("interior", "exterior") and sug != modo:
        r(AVISO, "modo vs entorno", f"el robot está en modo {modo} pero el LIDAR dice {sug}. Con avance_libre={avance} cm y lado_libre={lado} cm "
                                    "puede quedarse parado (en una sala) o ir demasiado rápido (en la huerta). Activa el entorno automático o pulsa el modo.")
    else:
        r(OK, "modo vs entorno", f"modo {modo}, entorno {sug or 'indeciso'} (EMA {ent.get('ema')}) · auto {'ACTIVO' if ent.get('activo') else 'apagado'}")
    h = lid["hueco_m"]
    cerca = {k: v for k, v in h.items() if v is not None and v < 0.15}
    if cerca:
        r(AVISO, "espacio libre", f"encajonado: {cerca} m. No podrá girar (lado_libre {lado} cm); sácalo a un sitio abierto")
    else:
        r(OK, "espacio libre", "hueco " + ", ".join(f"{k} {('∞' if v is None else f'{v:.2f}')}" for k, v in h.items()) + " m")
    frente = h.get("adelante")
    if frente is not None and frente * 100 < avance:
        r(AVISO, "avance libre", f"delante {frente*100:.0f} cm < {avance} cm exigidos: no avanza, gira o retrocede")

    # --- sensores ---
    imu = est["imu"]
    if not imu.get("fresco"):
        r(AVISO, "IMU", "sin telemetría fresca (¿nodo de sensores apagado?): sin cabeceo ni giroscopio; la pose usa solo rf2o")
    elif not imu.get("nivel_calibrado"):
        r(AVISO, "IMU", "sin calibrar el nivel: POST /api/calibracion/imu_nivel con el robot nivelado y quieto")
    else:
        r(OK, "IMU", "fresco y nivel calibrado")
    r(AVISO if est.get("autonomia", {}).get("activa") else OK, "autonomía previa",
      "ya hay algo en marcha: detenerlo antes" if est["autonomia"]["activa"] else "nada en marcha")
    wp = est["autonomia"]["waypoints"]
    r(OK, "recorrido", f"{wp} waypoints: la patrulla los sigue" if wp else "sin waypoints: «Iniciar patrulla» EXPLORA y mapea (y graba el recorrido)")
    try:
        z = get(a.url, "/api/camara/zoom")["zoom"]
        r(OK if z <= 1.0 else AVISO, "zoom", "1×" if z <= 1.0 else f"{z}× (se pone a 1× al iniciar la autonomía)")
    except Exception:
        pass
    try:
        t = max(int(open(f"/sys/class/thermal/thermal_zone{i}/temp").read()) for i in range(0, 8)) / 1000
        r(OK if t < 75 else AVISO, "temperatura", f"{t:.0f} °C")
    except OSError:
        pass
    libre = shutil.disk_usage("/").free / 1e9
    r(OK if libre > 5 else AVISO, "disco", f"{libre:.0f} GB libres")

    if a.ros:
        if shutil.which("ros2") is None:
            r(AVISO, "Nav2", "`ros2` no está en el PATH: ejecuta `source /opt/ros/jazzy/setup.bash` y repite con --ros")
        else:
            for n in ("slam_toolbox", "planner_server", "controller_server", "bt_navigator", "behavior_server"):
                try:
                    o = subprocess.run(["ros2", "lifecycle", "get", f"/{n}", "--spin-time", "5"], capture_output=True, text=True, timeout=25).stdout.strip()
                except Exception as e:
                    o = f"error {e}"
                r(OK if "active" in o else FALLO, f"ROS /{n}", o or "sin respuesta")
    if not a.suelo_ok:
        r(AVISO, "suelo", "CONFIRMA a mano que el robot está en el SUELO y no en una mesa: el LIDAR no detecta el borde (cae). "
                          "Repite con --suelo-ok para silenciar este aviso")

    ancho = max(len(t) for _, t, _ in filas)
    for e, t, d in filas:
        print(f" {e} {t:<{ancho}}  {d}")
    n = {k: sum(1 for e, _, _ in filas if e == k) for k in (OK, AVISO, FALLO)}
    print(f"\n{n[OK]} ok · {n[AVISO]} avisos · {n[FALLO]} bloqueos")
    print("LISTO para lanzar la autonomía." if not n[FALLO] else "NO LISTO: corrige los ✗ antes de lanzar.")
    return 1 if n[FALLO] else 0


if __name__ == "__main__":
    sys.exit(main())
