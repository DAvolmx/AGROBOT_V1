"""Figuras de RESULTADOS (necesitan los modelos entrenados): segmentación, corredor, aproximación,
detección de objetos, plagas y prueba con fotos reales.

Uso: python3 figuras_resultados.py [--modelo modelos/seg_interior_v1.pt]
"""
import argparse
import glob
import json
import sys
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import corredor as C
import entrenar_seg as T
import evaluar_corredor as EV
import render as R

AQUI = Path(__file__).resolve().parent
SAL = AQUI.parents[1] / "docs" / "capturas_percepcion_20261015"
SAL.mkdir(parents=True, exist_ok=True)
COL = {"actual": "#c0392b", "lidar": "#c0392b", "camara": "#e67e22", "camara_mem": "#2980b9", "camara+lidar": "#1f7a4d"}


def fig_seg_iou(meta):
    m = meta["metricas"]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    w = 0.26
    for k, (split, col) in enumerate((("val", "#7f8c8d"), ("test", "#1f7a4d"), ("test_nuevo", "#e67e22"))):
        if split not in m:
            continue
        v = [m[split]["iou"][c] or 0 for c in R.CLASES]
        ax.bar(np.arange(5) + (k - 1) * w, v, w, color=col, label=f"{split} (mIoU {m[split]['miou']:.3f})")
    ax.set_xticks(range(5))
    ax.set_xticklabels(R.CLASES)
    ax.set_ylabel("IoU")
    ax.set_ylim(0, 1.02)
    ax.set_title("Segmentación: IoU por clase. test_nuevo = otros suelos y luces NO vistos en entrenamiento")
    ax.legend(loc="lower left")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(SAL / "03_segmentacion_iou.png", dpi=125)
    plt.close(fig)


def _frames(split, n, rng, exigir=(3, 4)):
    fs = sorted(glob.glob(str(AQUI / "datos" / split / "*.npz")))
    out = []
    for f in rng.permutation(fs)[:60]:
        d = np.load(f, allow_pickle=True)
        k = int(rng.integers(len(d["rgb"])))
        if any((d["clase"][k] == c).sum() > 150 for c in exigir):
            out.append((d["rgb"][k], d["clase"][k], float(d["poses"][k][5])))
        if len(out) >= n:
            break
    return out


def fig_seg_ejemplos(net, dev, split="test_nuevo"):
    rng = np.random.default_rng(5)
    fr = _frames(split, 6, rng)
    pred = EV.segmentar(net, np.stack([f[0] for f in fr]), dev)
    fig, ax = plt.subplots(3, 6, figsize=(17, 7.2))
    for i, (rgb, gt, inc) in enumerate(fr):
        ax[0, i].imshow(rgb); ax[1, i].imshow(R.COLORES_CLASE[gt]); ax[2, i].imshow(R.COLORES_CLASE[pred[i]])
        err = (pred[i] != gt).mean() * 100
        ax[2, i].set_title(f"error de píxel {err:.1f}%", fontsize=9)
        for a in ax[:, i]:
            a.axis("off")
    for r, t in enumerate(("imagen", "verdad del simulador", "predicción")):
        ax[r, 0].text(-0.05, 0.5, t, transform=ax[r, 0].transAxes, rotation=90, va="center", ha="right", fontsize=11)
    fig.suptitle(f"Segmentación en {split} (entorno NO visto): imagen, verdad y predicción", fontsize=13)
    fig.tight_layout()
    fig.savefig(SAL / "04_segmentacion_ejemplos.png", dpi=110)
    plt.close(fig)


def _suelo_rgb(mascara, inc):
    G, cen = C.a_suelo(mascara, inc)
    img = np.full(G.shape + (3,), 235, np.uint8)
    for cl in range(5):
        img[G == cl + 1] = R.COLORES_CLASE[cl]
    return img, cen


def fig_corredor(net, dev, split="test_nuevo"):
    rng = np.random.default_rng(8)
    fr = _frames(split, 4, rng, exigir=(2, 3, 4))
    pred = EV.segmentar(net, np.stack([f[0] for f in fr]), dev)
    fig, ax = plt.subplots(2, 4, figsize=(17, 8.6))
    for i, (rgb, gt, inc) in enumerate(fr):
        cp = C.corredor(pred[i], inc)
        dec = C.decidir(cp)
        ax[0, i].imshow(rgb)
        ov = R.COLORES_CLASE[pred[i]]
        ax[0, i].imshow(ov, alpha=0.4)
        ax[0, i].axis("off")
        ax[0, i].set_title(f"inclinación {inc:.0f}° · zona ciega {cp.zona_ciega_m*100:.0f} cm", fontsize=9)
        img, cen = _suelo_rgb(pred[i], inc)
        ax[1, i].imshow(img, extent=[1.0, -1.0, 0.30, 1.80], aspect="equal")
        ax[1, i].plot(0, 0.30, "k^", ms=9)
        for la in C.MIRADAS_M:
            if cp.izq_m.get(la) is not None:
                ax[1, i].plot([cp.izq_m[la], cp.der_m[la]], [la, la], "w-", lw=2.2)
                ax[1, i].plot(cp.centro_m[la], la, "wo", mec="k")
        if cp.obstaculo_m:
            ax[1, i].axhline(cp.obstaculo_m, color="#c0392b", ls="--", lw=2)
        ax[1, i].axhline(cp.zona_ciega_m, color="k", ls=":", lw=1)
        ax[1, i].set_title(f"{dec.accion.upper()}: {dec.motivo[:48]}", fontsize=9,
                           color={"avanzar": "#1f7a4d", "reducir": "#e67e22", "parar": "#c0392b"}[dec.accion])
        ax[1, i].set_xlabel("y (m)  ← izquierda | derecha →")
        ax[1, i].set_ylabel("x delante (m)")
    fig.suptitle("De la máscara al corredor: vista cenital del suelo (azul libre, verde planta, rojo obstáculo, amarillo borde, "
                 "blanco = no visible). Línea punteada negra = zona ciega", fontsize=11)
    fig.tight_layout()
    fig.savefig(SAL / "05_corredor_cenital.png", dpi=105)
    plt.close(fig)


def fig_aproximacion(ruta):
    d = json.load(open(ruta))["resumen"]
    sis = ["lidar", "camara", "camara_mem", "camara+lidar"]
    rot = {"lidar": "solo LIDAR (hoy)", "camara": "cámara", "camara_mem": "cámara + memoria", "camara+lidar": "cámara + memoria + LIDAR"}
    pel = ["todos", "maceta_alta", "maceta_baja", "caja", "bordillo", "patas_silla"]
    fig, ax = plt.subplots(figsize=(12.5, 4.8))
    w = 0.2
    for k, s in enumerate(sis):
        ax.bar(np.arange(len(pel)) + (k - 1.5) * w, [d[s][p]["contactos_pct"] for p in pel], w,
               color=COL[s if s != "lidar" else "lidar"] if s != "camara+lidar" else COL["camara+lidar"], label=rot[s])
    ax.set_xticks(range(len(pel)))
    ax.set_xticklabels(pel)
    ax.set_ylabel("aproximaciones con CONTACTO (%)")
    ax.set_title("Aproximación a un peligro en lazo cerrado (cinemática; deslizamiento y odometría NO medidos)")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    ax.set_ylim(0, 105)
    fig.tight_layout()
    fig.savefig(SAL / "06_aproximacion_contactos.png", dpi=125)
    plt.close(fig)


def fig_deteccion(carpeta, modelo, salida, titulo, n=6, split="test_nuevo", imgsz=320, nombres=None, conf=0.25):
    from ultralytics import YOLO
    net = YOLO(str(modelo))
    fs = sorted(glob.glob(str(carpeta / "images" / split / "*.jpg")))
    rng = np.random.default_rng(3)
    sel = [fs[i] for i in rng.choice(len(fs), n, replace=False)]
    fig, ax = plt.subplots(2, 3, figsize=(16, 8.4))
    for a, f in zip(ax.ravel(), sel):
        im = cv2.imread(f)[..., ::-1].copy()
        h, w = im.shape[:2]
        lab = Path(str(carpeta / "labels" / split / Path(f).stem) + ".txt")
        if lab.exists():
            for ln in lab.read_text().split("\n"):
                if ln.strip():
                    c, x, y, bw, bh = map(float, ln.split())
                    cv2.rectangle(im, (int((x - bw / 2) * w), int((y - bh / 2) * h)), (int((x + bw / 2) * w), int((y + bh / 2) * h)),
                                  (255, 255, 255), 1)
        r = net.predict(f, imgsz=imgsz, conf=conf, verbose=False)[0]
        for b in r.boxes:
            x0, y0, x1, y1 = [int(v) for v in b.xyxy[0]]
            cv2.rectangle(im, (x0, y0), (x1, y1), (255, 40, 40), 2)
            cv2.putText(im, f"{r.names[int(b.cls)]} {float(b.conf):.2f}", (x0, max(10, y0 - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 0), 1)
        a.imshow(im)
        a.axis("off")
    fig.suptitle(titulo + "  (blanco = verdad, rojo = detección)", fontsize=12)
    fig.tight_layout()
    fig.savefig(SAL / salida, dpi=105)
    plt.close(fig)


def fig_real(net, dev):
    fs = sorted(glob.glob(str(AQUI.parents[1] / "logs" / "sesion_20261001" / "fotos" / "camara_*.jpg")))
    sel = fs[::max(1, len(fs) // 12)][:12]
    ims = [cv2.resize(cv2.imread(f)[..., ::-1], (320, 240)) for f in sel]
    pred = EV.segmentar(net, np.stack(ims), dev)
    decs = [C.decidir(C.corredor(p, 15.5)).accion for p in pred]
    todas = [C.decidir(C.corredor(p, 15.5)).accion for p in EV.segmentar(net, np.stack(
        [cv2.resize(cv2.imread(f)[..., ::-1], (320, 240)) for f in fs]), dev)]
    fig, ax = plt.subplots(2, 4, figsize=(15, 6.2))
    for i in range(4):
        ax[0, i].imshow(ims[i * 3])
        ax[1, i].imshow(R.COLORES_CLASE[pred[i * 3]])
        ax[1, i].set_title(f"decisión: {decs[i*3].upper()}", fontsize=10, color="#c0392b" if decs[i*3] == "parar" else "#1f7a4d")
        ax[0, i].axis("off"); ax[1, i].axis("off")
    cnt = {a: todas.count(a) for a in ("avanzar", "reducir", "parar")}
    (AQUI / "modelos" / "fotos_reales_pared.json").write_text(json.dumps({"n": len(todas), "decisiones": cnt}))
    fig.suptitle(f"FOTOS REALES del robot (1-oct, cámara mirando a una pared, sin suelo a la vista): decisiones en las {len(todas)} fotos {cnt}",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(SAL / "09_fotos_reales_pared.png", dpi=110)
    plt.close(fig)
    return cnt, len(todas)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--modelo", default=str(AQUI / "modelos" / "seg_interior_v1.pt"))
    a = ap.parse_args()
    dev = torch.device("cuda")
    net, meta = EV.cargar_modelo(a.modelo, dev)
    if "metricas" in meta:
        fig_seg_iou(meta)
    fig_seg_ejemplos(net, dev)
    fig_corredor(net, dev)
    ap_json = AQUI / "modelos" / f"{Path(a.modelo).stem}_aproximacion.json"
    if ap_json.exists():
        fig_aproximacion(ap_json)
    mo = AQUI / "modelos" / "yolo" / "obj" / "weights" / "best.pt"
    if mo.exists():
        fig_deteccion(AQUI / "yolo_obj", mo, "07_deteccion_objetos.png", "Detección de objetos (navegación), entorno NO visto", imgsz=320)
    print("real:", fig_real(net, dev))
    print("figuras:", sorted(p.name for p in SAL.iterdir()))


if __name__ == "__main__":
    main()
