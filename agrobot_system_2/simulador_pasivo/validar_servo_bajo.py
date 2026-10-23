"""Valida servo_bajo.py con las FOTOS REALES de la grabación de campo: detecta las macetas (modelo del robot) en cada foto y cuenta en cuántas
la maceta está «baja» (cortada por el borde inferior o con el centro en el 38 % inferior). Esas son las que el robot perdía de vista justo antes de tocarlas
y las que ahora provocan que el servo baje. Además reproduce la secuencia de decisiones del servo sobre la grabación.
Uso: python3 validar_servo_bajo.py [--campo <carpeta>]   Salida: ~/.agrobot/informes/servo_bajo_<fecha>/"""
import argparse, glob, json, os, sys, time
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))
from servo_bajo import ServoBajo, maceta_baja   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--campo", default=None)
    ap.add_argument("--paso", type=int, default=1)
    a = ap.parse_args()
    from ultralytics import YOLO
    campo = Path(a.campo) if a.campo else Path(sorted(glob.glob(os.path.expanduser("~/.agrobot/campo/20261023_*[0-9a-f]")))[-1])
    fotos = sorted(glob.glob(str(campo / "fotos" / "*.jpg")), key=lambda f: float(Path(f).stem.split("_")[1]))[::a.paso]
    modelo = YOLO(str(AQUI.parent / "jetson" / "backend" / "models" / "agrobot_maceta_det.pt"))
    sb = ServoBajo()
    servo, serie, bajas, con_maceta, ejemplos = 98, [], 0, 0, []
    cache = []
    t0 = float(Path(fotos[0]).stem.split("_")[1])
    for i in range(0, len(fotos), 16):
        lote = fotos[i:i + 16]
        for f, r in zip(lote, modelo.predict(lote, verbose=False, conf=0.35, imgsz=640)):
            t = float(Path(f).stem.split("_")[1]) - t0
            h = r.orig_shape[0]
            cajas = [b.tolist() for b in r.boxes.xyxy.cpu().numpy()] if len(r.boxes) else []
            if cajas:
                con_maceta += 1
            cache.append((t, h, cajas))
            baja = any(maceta_baja(c, h) for c in cajas)
            bajas += int(baja)
            nuevo = sb.paso(cajas, h, servo, t)
            if nuevo is not None:
                servo = nuevo
            serie.append((t, servo, int(baja), len(cajas)))
            if baja and len(ejemplos) < 6:
                im = r.plot()
                cv2.putText(im, f"maceta BAJA -> servo {servo}", (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                ejemplos.append(im)
    dest = Path.home() / ".agrobot" / "informes" / ("servo_bajo_" + time.strftime("%Y%m%d_%H%M"))
    dest.mkdir(parents=True, exist_ok=True)
    res = {"fotos": len(fotos), "con_maceta": con_maceta, "con_maceta_baja": bajas, "pct_baja_de_las_que_tienen_maceta": round(100 * bajas / max(1, con_maceta), 1),
           "servo_min": min(s[1] for s in serie), "decisiones_servo": sum(1 for a_, b_ in zip(serie, serie[1:]) if a_[1] != b_[1])}
    (dest / "resultado.json").write_text(json.dumps(res, indent=1))
    (dest / "cajas.json").write_text(json.dumps(cache))
    fig, ax = plt.subplots(2, 1, figsize=(11, 5), sharex=True)
    ax[0].plot([s[0] for s in serie], [s[1] for s in serie]); ax[0].set_ylabel("servo (°)"); ax[0].set_title("Servo propuesto sobre la grabación (menos = mira más abajo)")
    ax[1].fill_between([s[0] for s in serie], [s[2] for s in serie], step="pre", alpha=0.5, color="r"); ax[1].set_ylabel("maceta baja"); ax[1].set_xlabel("s")
    fig.tight_layout(); fig.savefig(dest / "servo.png", dpi=110); plt.close(fig)
    if ejemplos:
        cv2.imwrite(str(dest / "ejemplos_macetas_bajas.jpg"), cv2.vconcat([cv2.hconcat([cv2.resize(e, (480, 360)) for e in ejemplos[i:i + 3]]) for i in range(0, len(ejemplos) - len(ejemplos) % 3, 3)]) if len(ejemplos) >= 3 else ejemplos[0])
    print(json.dumps(res, indent=1), "->", dest)


if __name__ == "__main__":
    main()
