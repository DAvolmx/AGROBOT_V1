"""Datos REALES para la transitabilidad (2026-10-27): fotos de las sesiones de campo (~/.agrobot/campo/*/) emparejadas con el scan del LIDAR y el ángulo del servo del
mismo instante -> etiquetas DÉBILES por columna, sin etiquetado manual:
  · libre (1): filas de la imagen entre el robot y el punto donde el LIDAR ve algo en esa columna (o hasta 2 m si no ve nada);
  · obstáculo (4): una franja de ±MARGEN_PX alrededor de la base del punto del LIDAR;
  · ignorar (255): todo lo demás (pared/fondo, zona ciega) y todo lo dudoso.
LÍMITE QUE HAY QUE SABER: el LIDAR está a 37,5 cm; un obstáculo bajo que no ve queda etiquetado como «libre». Por eso en el entrenamiento estas etiquetas solo valen donde el
modelo sintético NO contradice con seguridad (ver entrenar_trav.py --real). Videos del usuario (webcam de otro equipo, sin LIDAR) -> solo estadística de color.
Uso: python3 preparar_real.py [--cada 8] [--max_por_sesion 400]   Salida: datos/real/real.npz (rgb, etq, sesion, servo) + datos/real/estilo.npz"""
import argparse, glob, json, math, sys
from pathlib import Path

import cv2
import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parents[1] / "jetson" / "backend"))
from barrido_tilt import inclinacion_por_servo      # noqa: E402

CAMPO = Path.home() / ".agrobot" / "campo"
GEO = json.load(open(AQUI.parents[1] / "jetson" / "backend" / "agrobot_config.json"))["odometria_webcam"]
H, W = 240, 320
MARGEN_PX = 5
RANGO_LIBRE_M = 2.0
DT_MAX_S = 0.35


def a_pixel(x, y, incl_deg):
    """Punto del suelo (x adelante, y izquierda, marco del robot) -> (u, v) en la imagen de 320x240. Inversa de SueloLibre.a_suelo."""
    h, phi = GEO["camara_altura_m"], math.radians(incl_deg)
    f = (W / 2) / math.tan(math.radians(GEO["camara_hfov_deg"]) / 2)
    dx = x - GEO["camara_adelante_m"]
    zc = dx * math.cos(phi) + h * math.sin(phi)
    ok = zc > 1e-3
    t = (h * math.cos(phi) - dx * math.sin(phi)) / np.where(ok, zc, 1.0)
    return W / 2 - y * f / np.where(ok, zc, 1.0), H / 2 + f * t, ok


def etiquetar(scan, incl_deg):
    etq = np.full((H, W), 255, np.uint8)
    cols_hit = np.full(W, np.inf)
    if scan:
        ang = np.radians([p["angle_deg"] for p in scan])
        d = np.array([p["distance_m"] for p in scan])
        x, y = d * np.cos(ang), d * np.sin(ang)
        u, v, ok = a_pixel(x, y, incl_deg)
        for ui, vi, oki, di, ai in zip(u, v, ok, d, ang):
            if oki and 0 <= ui < W and 0 < vi < H + 40 and di < RANGO_LIBRE_M and abs(math.degrees(ai)) < 60:
                c = int(ui)
                cols_hit[c] = min(cols_hit[c], vi)
    # fila del suelo a RANGO_LIBRE_M (más allá no se afirma nada)
    _, v_lim, _ = a_pixel(np.array([RANGO_LIBRE_M]), np.array([0.0]), incl_deg)
    v_lim = int(np.clip(v_lim[0], 0, H - 1))
    # rellena columnas sin retorno con el vecino más cercano para no dejar «agujeros» (el LIDAR es disperso)
    hit = cols_hit.copy()
    idx = np.where(np.isfinite(hit))[0]
    if len(idx) < 8:
        return None
    for c in range(W):
        if not np.isfinite(hit[c]):
            j = idx[np.argmin(np.abs(idx - c))]
            if abs(j - c) <= 6:
                hit[c] = hit[j]
    for c in range(W):
        vh = hit[c]
        if np.isfinite(vh):
            v0 = int(vh)
            if v0 + MARGEN_PX + 1 < H:
                etq[v0 + MARGEN_PX + 1:, c] = 1
            etq[max(0, v0 - MARGEN_PX):min(H, v0 + MARGEN_PX + 1), c] = 4
        else:
            etq[v_lim:, c] = 1
    return etq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cada", type=int, default=8)
    ap.add_argument("--max_por_sesion", type=int, default=400)
    a = ap.parse_args()
    rgb, etq, ses, servo = [], [], [], []
    for d in sorted(glob.glob(str(CAMPO / "2026*/"))):
        d = Path(d)
        if not (d / "frames.jsonl").exists() or not (d / "variables.jsonl").exists():
            continue
        fotos = [json.loads(l) for l in open(d / "frames.jsonl") if '"photo"' in l]
        var = []
        for l in open(d / "variables.jsonl"):
            try:
                r = json.loads(l)
                if r.get("lidar_scan") and r.get("t_unix"):
                    var.append((float(r["t_unix"]), r))
            except Exception:                                                     # noqa: BLE001
                pass
        if not fotos or not var:
            continue
        tv = np.array([t for t, _ in var])
        n = 0
        for f in fotos[::a.cada]:
            if n >= a.max_por_sesion:
                break
            i = int(np.argmin(np.abs(tv - f["observed_unix"])))
            if abs(tv[i] - f["observed_unix"]) > DT_MAX_S:
                continue
            r = var[i][1]
            if (r.get("lidar_age_s") or 9) > DT_MAX_S:
                continue
            st = r.get("actuator_status") or {}
            if st.get("servo_angle") is None:
                continue
            im = cv2.imread(str(d / f["file"]))
            if im is None:
                continue
            incl = GEO["camara_inclinacion_deg"] + inclinacion_por_servo(st["servo_angle"])
            e = etiquetar(r["lidar_scan"], incl)
            if e is None:
                continue
            rgb.append(cv2.cvtColor(cv2.resize(im, (W, H), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB))
            etq.append(e)
            ses.append(d.name)
            servo.append(st["servo_angle"])
            n += 1
        print(f"{d.name}: {n} fotos con etiqueta", flush=True)
    out = AQUI / "datos" / "real"
    out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out / "real.npz", rgb=np.array(rgb), etq=np.array(etq), sesion=np.array(ses), servo=np.array(servo))
    # estilo: media/desv. por canal de fotos reales + videos del usuario (para llevar el color sintético al real)
    est = [f.reshape(-1, 3) for f in rgb[::5]]
    for v in glob.glob(str(Path.home() / ".agrobot" / "datos_usuario" / "*" / "*.mp4")):
        c = cv2.VideoCapture(v)
        n = int(c.get(7) or 0)
        for k in range(0, n, max(1, n // 40)):
            c.set(1, k)
            ok, fr = c.read()
            if ok:
                est.append(cv2.cvtColor(cv2.resize(fr, (W, H)), cv2.COLOR_BGR2RGB).reshape(-1, 3))
    E = np.array([[e.mean(0), e.std(0)] for e in (x.astype(np.float32) for x in est)])
    np.savez(out / "estilo.npz", mu=E[:, 0], sd=E[:, 1])
    libre = np.mean([(e == 1).mean() for e in etq])
    print(f"TOTAL {len(rgb)} fotos reales etiquetadas (libre {libre:.0%} de los píxeles) · {len(E)} muestras de estilo")


if __name__ == "__main__":
    main()
