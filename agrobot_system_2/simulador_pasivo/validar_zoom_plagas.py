"""Valida el ZOOM AUTOMÁTICO sobre el clasificador de plagas/enfermedades de col (agrobot_plagasx_cls.pt) con imágenes REALES.

Simula el acercamiento del robot: una foto de hoja de col (dataset real, con su etiqueta) se pega, más pequeña, sobre un FONDO REAL
del robot (fotos de la cámara tomadas en las pruebas). Cuanto más lejos, más pequeña (fracción de la imagen). Se compara:
  SIN zoom : la cámara a 1× entrega 640×480  (la planta, a escala `s`, mide s·640 px)
  CON zoom : la lógica REAL de zoom_auto.ZoomAuto decide el zoom con la caja de la planta; la cámara a 1280×720 recorta el centro
             (camera._recorte_zoom, la misma función) y lo lleva a 640×480
y se mide la exactitud del clasificador y la probabilidad de la clase correcta. Salida en ~/.agrobot/informes/zoom_plagas_<fecha>/.
Uso: python3 validar_zoom_plagas.py [--por-clase 30]"""
import argparse, glob, json, os, random, sys, time
from pathlib import Path

import cv2
import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))
from zoom_auto import ZoomAuto, fraccion          # noqa: E402
from camera import _recorte_zoom                  # noqa: E402

RAIZ_IMG = Path.home() / ".agrobot" / "val_zoom" / "cols" / "orig" / "ORIGINAL_ALL_COMBINE_CABBAGE_DISEASES"
CLASES = {"Cabbage - Healthy": "col_sana", "Cabbage downy mildew - Original": "col_mildiu_velloso",
          "Cabbage_Altenaria_spot": "col_manchas_cafe"}
ESCALAS = [0.10, 0.15, 0.22, 0.32, 0.45, 0.65, 1.0]      # fracción del ancho de la imagen que ocupa la planta (a 1×)


def fondo(rng, fondos, ancho, alto):
    f = cv2.imread(str(rng.choice(fondos)))
    return cv2.resize(f, (ancho, alto), interpolation=cv2.INTER_AREA)


def pegar(bg, img, frac_ancho):
    """Pega `img` centrada con ancho frac_ancho·ancho_del_lienzo. Devuelve (lienzo, caja xyxy)."""
    H, W = bg.shape[:2]
    w = max(8, int(frac_ancho * W))
    h = max(8, int(img.shape[0] * w / img.shape[1]))
    if h > H:
        h = H
        w = int(img.shape[1] * h / img.shape[0])
    chico = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
    out = bg.copy()
    x0, y0 = (W - w) // 2, (H - h) // 2
    out[y0:y0 + h, x0:x0 + w] = chico
    return out, [x0, y0, x0 + w, y0 + h]


def vista_con_zoom(lienzo, caja, zoom):
    """Lo que ve el modelo con zoom z: recorte central del lienzo 1280×720 llevado a 640×480 (y la caja en esa vista)."""
    H, W = lienzo.shape[:2]
    vista = _recorte_zoom(cv2, lienzo, zoom) if zoom > 1.0 else cv2.resize(lienzo, (640, 480), interpolation=cv2.INTER_AREA)
    ch = max(60, int(H / zoom)) if zoom > 1.0 else H
    cw = min(W, int(ch * 4 / 3)) if zoom > 1.0 else W
    x0, y0 = (W - cw) // 2, (H - ch) // 2
    sx, sy = 640.0 / cw, 480.0 / ch
    x1, y1, x2, y2 = caja
    return vista, [(x1 - x0) * sx, (y1 - y0) * sy, (x2 - x0) * sx, (y2 - y0) * sy]


def zoom_final(lienzo, caja, cfg=None):
    """Itera ZoomAuto (como el lazo del servidor) hasta que se estabiliza: devuelve (zoom, vista final, caja en la vista)."""
    za = ZoomAuto(cfg)
    z, t = 1.0, 0.0
    for _ in range(14):
        vista, cv = vista_con_zoom(lienzo, caja, z)
        n = za.paso("acercando", cv, 640, 480, z, t)
        t += 1.0
        if n is None:
            break
        z = n
    vista, cv = vista_con_zoom(lienzo, caja, z)
    return z, vista, cv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--por-clase", type=int, default=30)
    a = ap.parse_args()
    from ultralytics import YOLO
    modelo = YOLO(str(AQUI.parent / "jetson" / "backend" / "models" / "agrobot_plagasx_cls.pt"))
    nombres = {v: k for k, v in modelo.names.items()}
    rng = random.Random(7)
    fondos = sorted(glob.glob(os.path.expanduser("~/.agrobot/vision_activa/fotos/*.jpg")))
    assert fondos, "faltan fotos de fondo del robot"
    muestras = []
    for carpeta, clase in CLASES.items():
        fs = sorted(glob.glob(str(RAIZ_IMG / carpeta / "*")))
        rng.shuffle(fs)
        muestras += [(f, clase) for f in fs[: a.por_clase]]
    print(len(muestras), "imágenes reales,", len(fondos), "fondos del robot", flush=True)
    filas = []
    galeria = {}
    for s in ESCALAS:
        for modo in ("sin_zoom", "con_zoom"):
            ok = 0
            pp = []
            zs = []
            for k, (f, clase) in enumerate(muestras):
                img = cv2.imread(f)
                if img is None:
                    continue
                r2 = random.Random(1000 + k)
                if modo == "sin_zoom":                      # cámara a 1×: 640×480 nativo
                    bg = fondo(r2, fondos, 640, 480)
                    vista, _ = pegar(bg, img, s)
                    z = 1.0
                else:                                       # cámara a 1280×720 (necesaria para el zoom) + ZoomAuto
                    bg = fondo(r2, fondos, 1280, 720)
                    lienzo, caja = pegar(bg, img, s)
                    z, vista, _ = zoom_final(lienzo, caja)
                p = modelo.predict(vista, verbose=False, device="cpu")[0].probs.data.cpu().numpy()
                ok += int(int(p.argmax()) == nombres[clase])
                pp.append(float(p[nombres[clase]]))
                zs.append(z)
                if k == 0:
                    galeria[(s, modo)] = vista
            n = len(pp)
            filas.append({"escala": s, "modo": modo, "n": n, "exactitud": round(ok / n, 3), "prob_clase_correcta": round(float(np.mean(pp)), 3),
                          "zoom_medio": round(float(np.mean(zs)), 2)})
            print(filas[-1], flush=True)
    dest = Path.home() / ".agrobot" / "informes" / ("zoom_plagas_" + time.strftime("%Y%m%d_%H%M"))
    dest.mkdir(parents=True, exist_ok=True)
    (dest / "resultados.json").write_text(json.dumps(filas, indent=1, ensure_ascii=False))
    # galería: para cada escala, sin zoom / con zoom
    fil = []
    for s in ESCALAS:
        a1, a2 = galeria[(s, "sin_zoom")], galeria[(s, "con_zoom")]
        for im, txt in ((a1, f"sin zoom  s={s}"), (a2, f"con zoom  s={s}")):
            cv2.putText(im, txt, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        fil.append(np.hstack([cv2.resize(a1, (320, 240)), cv2.resize(a2, (320, 240))]))
    cv2.imwrite(str(dest / "galeria.jpg"), np.vstack(fil))
    print("->", dest)


if __name__ == "__main__":
    main()
