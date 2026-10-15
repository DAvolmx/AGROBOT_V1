#!/usr/bin/env python3
"""Inferencia cualitativa de la U-Net Kaggle sobre cámaras del simulador.

Replica el preprocesado de la celda baseline del notebook: RGB/255 + a*(LAB)/255
+ ExGR reescalado + top-hat/black-hat. No calcula IoU: el render actual no da
máscara semántica de verdad. El control seguro sigue usando los límites LIDAR.
"""
from __future__ import annotations
import json, math, os, sys, time
from pathlib import Path
import cv2
import numpy as np
import torch
import segmentation_models_pytorch as smp

HERE = Path(__file__).resolve().parent
SIM = HERE.parent
sys.path.insert(0, str(SIM))
import mundo

ROOT = Path.home() / ".agrobot" / "kaggle_notebook972b3bbf33"
MODEL = Path(os.environ.get("AGROBOT_SEGMENTATION_MODEL", ROOT / "modelo_agrobot_unet.pth"))
MUNDO_JSON = SIM / "mundos" / "huerta_fotos_50cm.json"
CONFIG = SIM / "config_huerta.json"
OUT = HERE / "salidas_kaggle_base"
H, W = 384, 512
NAMES = ["fondo", "sendero", "cultivo", "bancal", "obstaculo"]
COLORS_BGR = np.array([[80,80,80],[40,120,230],[60,180,60],[40,200,230],[40,40,220]], np.uint8)


def to_6ch(rgb: np.ndarray) -> np.ndarray:
    """Canales exactos usados en el entrenamiento baseline de Kaggle."""
    rgb = np.ascontiguousarray(rgb)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).astype(np.float32)
    b, g, r = cv2.split(bgr)
    s = b + g + r + 1e-6
    a = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB)[..., 1].astype(np.float32) / 255.0
    exgr = (2*(g/s) - (r/s) - (b/s)) - (1.4*(r/s) - (g/s))
    exgr = np.clip((exgr + 1.0) / 2.0, 0.0, 1.0)
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    relief = cv2.add(cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kernel),
                     cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kernel)).astype(np.float32) / 255.0
    x = np.concatenate([rgb.astype(np.float32)/255.0,
                        np.stack([a, exgr, relief], axis=-1)], axis=-1)
    return np.ascontiguousarray(x.transpose(2, 0, 1), dtype=np.float32)


def load_model():
    if not MODEL.is_file():
        raise FileNotFoundError(f"No existe el modelo base: {MODEL}")
    net = smp.Unet("timm-mobilenetv3_large_100", encoder_weights=None,
                   in_channels=6, classes=5)
    state = torch.load(MODEL, map_location="cpu", weights_only=True)
    net.load_state_dict(state, strict=True)
    device = torch.device("cuda" if torch.cuda.is_available() and os.environ.get("AGROBOT_FORCE_CPU") != "1" else "cpu")
    net.to(device).eval()
    return net, device


def render_poses(points, n=6):
    pts = np.asarray(points, dtype=float)
    for idx in np.linspace(0, len(pts)-2, n).astype(int):
        x, y = pts[idx]
        dx, dy = pts[idx+1] - pts[idx]
        yield float(x), float(y), math.atan2(dy, dx)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    net, device = load_model()
    world_data = json.loads(MUNDO_JSON.read_text(encoding="utf-8"))
    cfg = json.loads(CONFIG.read_text(encoding="utf-8"))
    geo = dict(cfg["odometria_webcam"])
    geo["camara_inclinacion_deg"] = 33.0
    world = mundo.Mundo.desde_dict(world_data)
    coverage = np.zeros(5, dtype=np.int64)
    composites, times = [], []
    for i, (x, y, theta) in enumerate(render_poses(world_data["sendero"])):
        bgr = world.camara(x, y, theta, geo, 640, 480)
        rgb = cv2.cvtColor(cv2.resize(bgr, (W, H), interpolation=cv2.INTER_LINEAR), cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(to_6ch(rgb)).unsqueeze(0).to(device)
        start = time.perf_counter()
        with torch.inference_mode():
            pred = net(tensor).argmax(1)[0].cpu().numpy().astype(np.uint8)
        if device.type == "cuda": torch.cuda.synchronize()
        times.append(time.perf_counter() - start)
        pred_full = cv2.resize(pred, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)
        overlay = cv2.addWeighted(bgr, 0.52, COLORS_BGR[pred_full], 0.48, 0)
        comp = np.hstack([bgr, overlay])
        cv2.imwrite(str(OUT / f"frame_{i:02d}.png"), comp)
        composites.append(comp)
        vals, counts = np.unique(pred, return_counts=True)
        for label, count in zip(vals, counts): coverage[label] += count
        print(f"frame {i+1}: " + ", ".join(f"{NAMES[k]}={v/ pred.size:.1%}" for k,v in zip(vals, counts)), flush=True)
    rows = [np.hstack(composites[i:i+2]) for i in range(0, len(composites), 2)]
    cv2.imwrite(str(OUT / "montaje.png"), np.vstack(rows))
    pct = (coverage / max(int(coverage.sum()), 1) * 100).round(2).tolist()
    report = {
        "model": str(MODEL), "architecture": "segmentation_models_pytorch.Unet(timm-mobilenetv3_large_100)",
        "input": {"width": W, "height": H, "channels": ["R/255","G/255","B/255","Lab-a/255","ExGR remapped 0..1","morph top-hat + black-hat /255"]},
        "classes": NAMES, "device": str(device), "frames": len(composites),
        "mean_inference_ms": round(float(np.mean(times))*1000, 2),
        "predicted_pixel_share_pct": dict(zip(NAMES, pct)),
        "kaggle_validation_reference": {"baseline_mIoU_val": 0.7104, "baseline_mIoU_real": 0.7386, "baseline_mIoU_synth": 0.5645, "v3_mIoU_val": 0.3877},
        "limitations": ["Sim frames have no semantic ground truth: this is visual inspection, not IoU.", "The obstacle class had no labeled pixels in Kaggle validation; do not use it as a collision detector.", "Keep LIDAR veto and low-object camera checks active; do not deploy model outputs as motor commands."],
        "outputs": str(OUT)
    }
    (OUT / "resumen.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)

if __name__ == "__main__":
    main()
