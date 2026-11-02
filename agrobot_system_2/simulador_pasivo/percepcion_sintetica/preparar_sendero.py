"""Dataset REAL de exterior para el SENDERO (2026-10-29): fotos de las sesiones de huerta etiquetadas con la trayectoria que el robot (o el operador) recorrió después.

Por qué: en la huerta el camino y las camas son la misma tierra; ni el color/textura ni las UNet sintéticas los separan. Donde el robot SÍ pasó es camino seguro.
Etiquetas (clases de render.CLASES; 255 = ignorar):
  · 1 camino   : franja de ±SEMI_ANCHO_M alrededor de las posiciones futuras del robot (de DIST_MIN_M a DIST_MAX_M por delante), proyectada a la imagen.
  · 4 obstáculo: banda alrededor de la base de cada punto del LIDAR (como preparar_real.py) cuando cae FUERA de la franja.
  · 2 planta   : puntos del LIDAR a más de LAT_CAMA_M de la franja: columna sobre su base (la cama / las coles vistas de lado).
  · 255        : todo lo demás.
Solo se usan instantes con el robot avanzando (la posición futura existe, pose fiable, sin saltos de reloj) y sesiones de exterior.
Uso: python3 preparar_sendero.py [--cada 4] [--max_por_sesion 500]   Salida: datos/real/sendero.npz (rgb, etq, sesion, servo)"""
import argparse, bisect, glob, json, math, sys
from pathlib import Path

import cv2
import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parents[1] / "jetson" / "backend"))
sys.path.insert(0, str(AQUI))
from barrido_tilt import inclinacion_por_servo      # noqa: E402
import preparar_real as PR                          # noqa: E402

CAMPO = Path.home() / ".agrobot" / "campo"
SESIONES_EXT = ["20261026_211854_a14049", "20261027_215828_25f568", "20261027_221926_28a5d4", "20261028_194540_ed7b3b",
                "20261028_195430_e352b3", "20261028_200037_32de40", "20261028_200833_d17537", "20261029_142951_a81509"]
H, W = PR.H, PR.W
SEMI_ANCHO_M = 0.17
DIST_MIN_M, DIST_MAX_M = 0.30, 2.2
LAT_CAMA_M = 0.40
VENTANA_S = 40.0
DT_MAX_S = 0.35


def trayectoria_futura(filas, i):
    """Posiciones futuras (x, y) en el marco del robot en la fila i. Solo mientras t_monotonic avanza sin saltos y la pose es fiable."""
    r0 = filas[i]
    p0 = r0["pose"]
    th = math.radians(p0["theta_deg"])
    c, s = math.cos(th), math.sin(th)
    pts, tm0, ult = [], r0["t_monotonic"], None
    for j in range(i, min(len(filas), i + 200)):
        r = filas[j]
        dt = r["t_monotonic"] - tm0
        if dt > VENTANA_S or dt < 0:
            break
        if not r.get("pose_reliable") or (r.get("pose_age_s") or 9) > 0.5:
            break
        dx, dy = r["pose"]["x"] - p0["x"], r["pose"]["y"] - p0["y"]
        x, y = c * dx + s * dy, -s * dx + c * dy
        if ult is not None and math.hypot(x - ult[0], y - ult[1]) > 0.5:         # salto de pose: se corta
            break
        ult = (x, y)
        pts.append((x, y))
    return np.array(pts)


def etiquetar_sendero(scan, incl, fut):
    """Devuelve la máscara de etiquetas o None si no hay trayectoria útil."""
    # se corta cuando el robot da la vuelta (x deja de crecer) o se sale del cuadro (>50° de la cámara)
    k = int(np.argmax(fut[:, 0]))
    fut = fut[:k + 1]
    fut = fut[(fut[:, 0] >= 0.12) & (fut[:, 0] <= DIST_MAX_M) & (np.abs(fut[:, 1]) < 1.2 * fut[:, 0])]
    if len(fut) < 4 or fut[:, 0].max() < 0.3 or np.ptp(fut[:, 0]) < 0.15:         # exige que avance de verdad
        return None
    # centro lateral del camino a cada x (interpolado sobre la trayectoria ordenada por x)
    o = np.argsort(fut[:, 0])
    fx, fy = fut[o, 0], fut[o, 1]
    etq = np.full((H, W), 255, np.uint8)
    xs = np.linspace(max(DIST_MIN_M, fx.min()), min(DIST_MAX_M, fx.max()), 40)
    yc = np.interp(xs, fx, fy)
    izq = np.stack([xs, yc + SEMI_ANCHO_M], 1)
    der = np.stack([xs, yc - SEMI_ANCHO_M], 1)
    pu, pv, ok1 = PR.a_pixel(izq[:, 0], izq[:, 1], incl)
    qu, qv, ok2 = PR.a_pixel(der[:, 0], der[:, 1], incl)
    if not (ok1.all() and ok2.all()):
        return None
    if not (np.isfinite(pu).all() and np.isfinite(qu).all()):
        return None
    poly = np.concatenate([np.stack([pu, pv], 1), np.stack([qu, qv], 1)[::-1]]).astype(np.int32)
    cv2.fillPoly(etq, [poly], 1)
    if (etq == 1).mean() < 0.01:
        return None
    # LIDAR: obstáculos (4) dentro del camino prohibido de la franja se ignoran; fuera: camas (2) o banda (4)
    if scan:
        ang = np.radians([p["angle_deg"] for p in scan])
        d = np.array([p["distance_m"] for p in scan])
        x, y = d * np.cos(ang), d * np.sin(ang)
        u, v, ok = PR.a_pixel(x, y, incl)
        for xi, yi, ui, vi, oki, di, ai in zip(x, y, u, v, ok, d, ang):
            if not (oki and 0 <= ui < W and 0 < vi < H + 40 and di < 2.0 and abs(math.degrees(ai)) < 60):
                continue
            lat = abs(yi - float(np.interp(xi, xs, yc)))
            c = int(ui)
            v0 = int(vi)
            if lat < SEMI_ANCHO_M + 0.05:
                continue                                                                  # dentro del camino recorrido: no se afirma nada
            if lat > LAT_CAMA_M:
                a, b = max(0, v0 - 45), min(H, v0 + 4)
                col = etq[a:b, max(0, c - 2):c + 3]
                col[col == 255] = 2
            else:
                a, b = max(0, v0 - PR.MARGEN_PX), min(H, v0 + PR.MARGEN_PX + 1)
                col = etq[a:b, max(0, c - 1):c + 2]
                col[col == 255] = 4
    return etq


def entorno_exterior():
    """(tiempos, es_exterior) del registro de entorno: la etiqueta del operador manda sobre el modo automático."""
    ts, ex = [], []
    for f in sorted(glob.glob(str(Path.home() / ".agrobot" / "entorno" / "2026*.jsonl"))):
        for l in open(f):
            if l.startswith("{"):
                try:
                    r = json.loads(l)
                except Exception:                                                          # noqa: BLE001
                    continue
                ts.append(r["t"]); ex.append((r.get("etiqueta") or r.get("modo")) == "exterior")
    o = np.argsort(ts)
    return np.array(ts)[o], np.array(ex)[o]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cada", type=int, default=4)
    ap.add_argument("--max_por_sesion", type=int, default=500)
    a = ap.parse_args()
    rgb, etq, ses, servo = [], [], [], []
    te, ee = entorno_exterior()
    for nom in SESIONES_EXT:
        d = CAMPO / nom
        if not (d / "frames.jsonl").exists():
            continue
        fotos = [json.loads(l) for l in open(d / "frames.jsonl") if '"photo"' in l]
        filas = []
        for l in open(d / "variables.jsonl"):
            if not l.startswith("{"):
                continue
            try:
                r = json.loads(l)
            except Exception:                                                              # noqa: BLE001
                continue
            if r.get("pose") and r.get("t_unix") and r.get("t_monotonic") is not None and r.get("lidar_scan"):
                filas.append(r)
        if not fotos or not filas:
            continue
        tm = np.array([r["t_monotonic"] for r in filas])
        tu = np.array([r["t_unix"] for r in filas])
        n = 0
        for f in fotos[::a.cada]:
            if n >= a.max_por_sesion:
                break
            i = int(np.argmin(np.abs(tu - f["observed_unix"])))
            r = filas[i]
            if abs(tu[i] - f["observed_unix"]) > DT_MAX_S or (r.get("lidar_age_s") or 9) > DT_MAX_S:
                continue
            st = r.get("actuator_status") or {}
            if st.get("servo_angle") is None or not r.get("pose_reliable"):
                continue
            k = int(np.argmin(np.abs(te - f["observed_unix"])))
            if abs(te[k] - f["observed_unix"]) > 15 or not ee[k]:
                continue                                                                   # tramos de interior dentro de una sesión de exterior
            fut = trayectoria_futura(filas, i)
            if len(fut) < 4:
                continue
            incl = PR.GEO["camara_inclinacion_deg"] + inclinacion_por_servo(st["servo_angle"])
            e = etiquetar_sendero(r["lidar_scan"], incl, fut)
            if e is None:
                continue
            im = cv2.imread(str(d / f["file"]))
            if im is None:
                continue
            rgb.append(cv2.cvtColor(cv2.resize(im, (W, H), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB))
            etq.append(e); ses.append(nom); servo.append(st["servo_angle"]); n += 1
        print(f"{nom}: {n} fotos etiquetadas", flush=True)
    out = AQUI / "datos" / "real"
    out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out / "sendero.npz", rgb=np.array(rgb), etq=np.array(etq), sesion=np.array(ses), servo=np.array(servo))
    E = np.array(etq)
    print(f"TOTAL {len(rgb)} fotos · camino {(E == 1).mean():.1%} cama {(E == 2).mean():.1%} obstáculo {(E == 4).mean():.1%} ignorado {(E == 255).mean():.1%}")


if __name__ == "__main__":
    main()
