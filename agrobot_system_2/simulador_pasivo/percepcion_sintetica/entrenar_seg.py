"""Entrena la segmentación de navegación (5 clases) con el dataset sintético de interiores.

Uso:  python3 entrenar_seg.py [--epocas 24] [--batch 16] [--sin_preentrenar]

POR QUÉ ESTE ENTRENAMIENTO ES DISTINTO AL DE CODEX
  · El checkpoint de Codex era un `state_dict` desnudo: sin media/desviación, sin resolución,
    sin nombres de clase, y su preprocesado de 6 canales (RGB + a* + ExGR + relieve) no está en
    el repo. Aquí el checkpoint lleva TODO lo necesario para reproducirlo (`meta`).
  · Entrada de 3 canales (RGB, normalización de ImageNet): sin canales hechos a mano que
    reconstruir. Si el contraste de color importa, la red lo aprende.
  · Se parte de un encoder preentrenado en ImageNet (descargado).
  · El modelo que se guarda es el MEJOR EN VALIDACIÓN, no el último (el último suele ser peor).

HONESTIDAD SOBRE QUÉ DEMUESTRA: mide si la red aprende la geometría y el contraste de un
mundo SINTÉTICO. Que funcione en el huerto real o en una sala real NO está demostrado: nunca se
ha evaluado aquí sobre fotos reales. El `test_nuevo` (otros suelos y luces) mide cuánto se cae
al cambiar de dominio DENTRO del simulador, que es un indicio, no una garantía.
"""
import argparse
import glob
import json
import math
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
import segmentation_models_pytorch as smp

import render as R

AQUI = Path(__file__).resolve().parent
DATOS = AQUI / "datos"
MODELOS = AQUI / "modelos"
MEDIA = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)
PAD = 8                                   # 240 -> 256 (múltiplo de 32 para la UNet)
N_CLASES = len(R.CLASES)


def cargar(split, max_sesiones=None):
    rgb, cl, info = [], [], []
    for i, f in enumerate(sorted(glob.glob(str(DATOS / split / "*.npz")))):
        if max_sesiones and i >= max_sesiones:
            break
        d = np.load(f, allow_pickle=True)
        rgb.append(d["rgb"])
        cl.append(d["clase"])
        meta = json.loads(str(d["meta"]))
        info += [meta] * len(d["rgb"])
    return np.concatenate(rgb), np.concatenate(cl), info


def modelo(preentrenar=True):
    return smp.Unet("timm-mobilenetv3_large_100", encoder_weights="imagenet" if preentrenar else None,
                    in_channels=3, classes=N_CLASES)


def preparar(x_uint8, dev):
    """(B,H,W,3) uint8 -> (B,3,256,W) normalizado, con relleno reflejado arriba y abajo."""
    x = torch.from_numpy(x_uint8).to(dev).permute(0, 3, 1, 2).float() / 255.0
    return normalizar(x)


def normalizar(x):
    m = torch.tensor(MEDIA, device=x.device).view(1, 3, 1, 1)
    s = torch.tensor(STD, device=x.device).view(1, 3, 1, 1)
    x = (x - m) / s
    return F.pad(x, (0, 0, PAD, PAD), mode="reflect")


def predecir(net, x_norm):
    out = net(x_norm)
    return out[:, :, PAD:-PAD, :]


@torch.no_grad()
def evaluar(net, rgb, cl, dev, batch=32, info=None):
    net.eval()
    conf = np.zeros((N_CLASES, N_CLASES), np.int64)
    por_grupo = {}
    for i in range(0, len(rgb), batch):
        x = preparar(rgb[i:i + batch], dev)
        with torch.autocast("cuda", dtype=torch.float16):
            p = predecir(net, x).argmax(1).cpu().numpy()
        y = cl[i:i + batch]
        conf += np.bincount((y.astype(np.int64) * N_CLASES + p).ravel(), minlength=N_CLASES ** 2).reshape(N_CLASES, N_CLASES)
        if info is not None:
            for k in range(len(y)):
                g = info[i + k]["tipo"] + "/" + info[i + k]["suelo"]
                c = por_grupo.setdefault(g, np.zeros((N_CLASES, N_CLASES), np.int64))
                c += np.bincount((y[k].astype(np.int64) * N_CLASES + p[k]).ravel(), minlength=N_CLASES ** 2).reshape(N_CLASES, N_CLASES)
    return metricas(conf), {g: metricas(c) for g, c in por_grupo.items()}


def metricas(conf):
    tp = np.diag(conf).astype(float)
    fp, fn = conf.sum(0) - tp, conf.sum(1) - tp
    with np.errstate(divide="ignore", invalid="ignore"):
        iou = tp / (tp + fp + fn)
        prec = tp / (tp + fp)
        rec = tp / (tp + fn)
    presentes = conf.sum(1) > 0
    return {"iou": {c: (None if not presentes[i] else round(float(iou[i]), 4)) for i, c in enumerate(R.CLASES)},
            "precision": {c: (None if not presentes[i] else round(float(prec[i]), 4)) for i, c in enumerate(R.CLASES)},
            "recall": {c: (None if not presentes[i] else round(float(rec[i]), 4)) for i, c in enumerate(R.CLASES)},
            "miou": round(float(np.nanmean(iou[presentes])), 4),
            "pixeles": {c: int(conf.sum(1)[i]) for i, c in enumerate(R.CLASES)},
            "exactitud": round(float(tp.sum() / max(conf.sum(), 1)), 4)}


def aumentar(x, y, gen):
    """x (B,3,H,W) 0..1 en GPU; y (B,H,W) long. Volteo, escala/desplazamiento, color, ruido."""
    B, _, H, W = x.shape
    dev = x.device
    # volteo horizontal (simétrico para esta tarea)
    f = torch.rand(B, device=dev, generator=gen) < 0.5
    x = torch.where(f.view(B, 1, 1, 1), x.flip(3), x)
    y = torch.where(f.view(B, 1, 1), y.flip(2), y)
    # escala y desplazamiento ligeros (no se gira: la geometría cámara-suelo no lo permite)
    s = 1.0 + (torch.rand(B, device=dev, generator=gen) - 0.5) * 0.25
    tx = (torch.rand(B, device=dev, generator=gen) - 0.5) * 0.16
    ty = (torch.rand(B, device=dev, generator=gen) - 0.5) * 0.10
    th = torch.zeros(B, 2, 3, device=dev)
    th[:, 0, 0], th[:, 1, 1], th[:, 0, 2], th[:, 1, 2] = 1 / s, 1 / s, tx, ty
    grid = F.affine_grid(th, x.shape, align_corners=False)
    x = F.grid_sample(x, grid, mode="bilinear", padding_mode="reflection", align_corners=False)
    y = F.grid_sample(y.unsqueeze(1).float(), grid, mode="nearest", padding_mode="reflection",
                      align_corners=False).squeeze(1).long()
    # color
    gan = 0.6 + torch.rand(B, 1, 1, 1, device=dev, generator=gen) * 0.8
    can = 0.7 + torch.rand(B, 1, 1, 1, device=dev, generator=gen) * 0.6
    bal = 0.85 + torch.rand(B, 3, 1, 1, device=dev, generator=gen) * 0.3
    m = x.mean((1, 2, 3), keepdim=True)
    x = ((x - m) * can + m) * gan * bal
    gris = x.mean(1, keepdim=True).expand_as(x)
    g = (torch.rand(B, device=dev, generator=gen) < 0.10).view(B, 1, 1, 1)
    x = torch.where(g, gris, x)
    perm = (torch.rand(B, device=dev, generator=gen) < 0.2).view(B, 1, 1, 1)      # barajar canales
    x = torch.where(perm, x[:, [2, 0, 1]], x)
    x = x + torch.randn(x.shape, device=dev, generator=gen) * (torch.rand(B, 1, 1, 1, device=dev, generator=gen) * 0.04)
    return x.clamp(0, 1), y


def dice_loss(logits, y, n=N_CLASES, eps=1.0):
    p = logits.softmax(1)
    oh = F.one_hot(y, n).permute(0, 3, 1, 2).float()
    inter = (p * oh).sum((0, 2, 3))
    den = p.sum((0, 2, 3)) + oh.sum((0, 2, 3))
    presente = oh.sum((0, 2, 3)) > 0
    d = 1 - (2 * inter + eps) / (den + eps)
    return d[presente].mean()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epocas", type=int, default=24)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--lr", type=float, default=1.5e-3)
    ap.add_argument("--sin_preentrenar", action="store_true")
    ap.add_argument("--max_sesiones", type=int, default=None, help="solo para pruebas rápidas")
    ap.add_argument("--nombre", default="seg_interior_v1")
    ap.add_argument("--semilla", type=int, default=0)
    a = ap.parse_args()
    torch.manual_seed(a.semilla)
    np.random.seed(a.semilla)
    dev = torch.device("cuda")
    MODELOS.mkdir(exist_ok=True)

    t0 = time.perf_counter()
    rgb_tr, cl_tr, _ = cargar("train", a.max_sesiones)
    rgb_va, cl_va, info_va = cargar("val", a.max_sesiones and max(2, a.max_sesiones // 5))
    print(f"train {len(rgb_tr)} fotogramas · val {len(rgb_va)} · carga {time.perf_counter()-t0:.0f} s", flush=True)
    frec = np.bincount(cl_tr.ravel(), minlength=N_CLASES).astype(float)
    frec /= frec.sum()
    print("frecuencia de píxeles en train:", {c: f"{100*f:.2f}%" for c, f in zip(R.CLASES, frec)}, flush=True)
    pesos = np.clip((np.median(frec) / np.maximum(frec, 1e-9)) ** 0.5, 0.5, 6.0)
    print("pesos de clase:", dict(zip(R.CLASES, np.round(pesos, 2))), flush=True)
    w = torch.tensor(pesos, dtype=torch.float32, device=dev)

    net = modelo(not a.sin_preentrenar).to(dev)
    opt = torch.optim.AdamW(net.parameters(), lr=a.lr, weight_decay=1e-4)
    pasos = a.epocas * math.ceil(len(rgb_tr) / a.batch)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=pasos, pct_start=0.15)
    scaler = torch.amp.GradScaler()
    gen = torch.Generator(device=dev)
    gen.manual_seed(a.semilla)

    mejor, mejor_ep, hist = -1.0, -1, []
    for ep in range(a.epocas):
        net.train()
        orden = np.random.permutation(len(rgb_tr))
        perd_ep, t_ep = 0.0, time.perf_counter()
        for i in range(0, len(orden) - a.batch + 1, a.batch):
            idx = np.sort(orden[i:i + a.batch])
            x = torch.from_numpy(rgb_tr[idx]).to(dev).permute(0, 3, 1, 2).float() / 255.0
            y = torch.from_numpy(cl_tr[idx]).to(dev).long()
            x, y = aumentar(x, y, gen)
            xn = normalizar(x)
            with torch.autocast("cuda", dtype=torch.float16):
                lg = predecir(net, xn)
            lg = lg.float()
            loss = F.cross_entropy(lg, y, weight=w) + 0.5 * dice_loss(lg, y)
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            sched.step()
            perd_ep += float(loss)
        m, _ = evaluar(net, rgb_va, cl_va, dev)
        n_b = len(orden) // a.batch
        hist.append({"epoca": ep + 1, "perdida": round(perd_ep / n_b, 4), "val_miou": m["miou"], "val_iou": m["iou"]})
        marca = ""
        if m["miou"] > mejor:
            mejor, mejor_ep = m["miou"], ep + 1
            torch.save(net.state_dict(), MODELOS / f"{a.nombre}.mejor.pt")
            marca = "  <-- mejor"
        print(f"  época {ep+1:2d}/{a.epocas}  pérdida {perd_ep/n_b:.3f}  val mIoU {m['miou']:.3f}  "
              f"IoU {{{', '.join(f'{k[:5]} {v}' for k, v in m['iou'].items())}}}  ({time.perf_counter()-t_ep:.0f} s){marca}", flush=True)

    # --- evaluación final del MEJOR en val
    net.load_state_dict(torch.load(MODELOS / f"{a.nombre}.mejor.pt"))
    resultados = {"val": evaluar(net, rgb_va, cl_va, dev, info=info_va)}
    for split in ("test", "test_nuevo"):
        if (DATOS / split).exists():
            rgb, cl, info = cargar(split, a.max_sesiones and max(2, a.max_sesiones // 4))
            resultados[split] = evaluar(net, rgb, cl, dev, info=info)
            print(f"  {split:11s} mIoU {resultados[split][0]['miou']:.3f}  "
                  f"{ {k: v for k, v in resultados[split][0]['iou'].items()} }", flush=True)
    base = np.zeros(N_CLASES)
    base[1] = 1
    meta = {"nombre": a.nombre, "arquitectura": "smp.Unet", "encoder": "timm-mobilenetv3_large_100",
            "encoder_weights": None if a.sin_preentrenar else "imagenet", "in_channels": 3,
            "clases": R.CLASES, "n_clases": N_CLASES, "media": MEDIA.tolist(), "std": STD.tolist(),
            "preprocesado": "RGB uint8 -> /255 -> (x-media)/std -> relleno reflejado 8 px arriba y abajo "
                            "(240->256); la salida se recorta 8 px por arriba y por abajo",
            "entrada": {"alto": 240, "ancho": 320, "alto_con_relleno": 256}, "pad": PAD,
            "geometria_camara": R.CAMARA, "mejor_epoca": mejor_ep, "epocas": a.epocas,
            "val_miou_mejor": mejor, "semilla": a.semilla,
            "dataset": {"sesiones_train": len(glob.glob(str(DATOS / 'train' / '*.npz'))),
                        "fotogramas_train": int(len(rgb_tr))},
            "fecha": time.strftime("%Y-%m-%d %H:%M"),
            "metricas": {k: v[0] for k, v in resultados.items()},
            "NO_DEMOSTRADO": "Entrenado y evaluado SOLO con imágenes sintéticas. No hay evaluación sobre "
                             "fotos reales. No desplegar sin evaluación independiente real y reversión."}
    torch.save({"state_dict": net.state_dict(), "meta": meta}, MODELOS / f"{a.nombre}.pt")
    (MODELOS / f"{a.nombre}_metricas.json").write_text(json.dumps(
        {"meta": meta, "historial": hist, "por_grupo": {k: v[1] for k, v in resultados.items()}},
        indent=1, ensure_ascii=False))
    print(f"\nmejor época {mejor_ep}, val mIoU {mejor:.3f}. Guardado en {MODELOS}/{a.nombre}.pt  ({time.perf_counter()-t0:.0f} s)")


if __name__ == "__main__":
    main()
