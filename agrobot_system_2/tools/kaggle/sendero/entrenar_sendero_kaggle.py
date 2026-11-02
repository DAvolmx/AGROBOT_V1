"""AGROBOT · segmentación del SENDERO con fotos reales de huerta (Kaggle, GPU). 2026-10-29.

Afina la UNet de transitabilidad (trav_20261028; clases 0 fondo, 1 camino libre, 2 planta/maceta, 3 borde bajo, 4 obstáculo) con:
  · sendero.npz : 790 fotos reales de exterior; camino = franja que el robot recorrió después, obstáculo/planta = puntos del LIDAR fuera de la franja, 255 = no se sabe.
  · sintetico.npz: repaso de imágenes sintéticas (para no olvidar obstáculos) + test_nuevo para medir ese olvido.
Se valida con SESIONES COMPLETAS que no se usan para entrenar. Se mide la base y el modelo afinado con las mismas métricas y se guarda el MEJOR.
Salida en /kaggle/working: sendero_mejor.pt, sendero_final.pt, resultados.json."""
import json, subprocess, sys, time
from pathlib import Path

try:
    import segmentation_models_pytorch as smp
except Exception:                                                                    # noqa: BLE001
    subprocess.run([sys.executable, "-m", "pip", "install", "-q", "segmentation-models-pytorch"], check=False)
    import segmentation_models_pytorch as smp
import numpy as np
import torch
import torch.nn.functional as F

IN = next(p for p in Path("/kaggle/input").glob("**/sendero.npz")).parent
OUT = Path("/kaggle/working")
dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
EPOCAS, BATCH_R, BATCH_S, LR = 14, 8, 8, 1.2e-4
VAL_SESIONES = ["20261028_200833_d17537", "20261029_142951_a81509"]
MEDIA = torch.tensor([0.485, 0.456, 0.406], device=dev).view(1, 3, 1, 1)
STD = torch.tensor([0.229, 0.224, 0.225], device=dev).view(1, 3, 1, 1)
PAD = 8
torch.manual_seed(0); np.random.seed(0)

real = np.load(IN / "sendero.npz")
syn = np.load(IN / "sintetico.npz")
ses = real["sesion"]
mval = np.isin(ses, VAL_SESIONES)
Rtr, Etr, Rva, Eva = real["rgb"][~mval], real["etq"][~mval], real["rgb"][mval], real["etq"][mval]
Str, Ctr, Ste, Cte = syn["rgb"], syn["clase"], syn["rgb_test"], syn["clase_test"]
print(f"real train {len(Rtr)} val {len(Rva)} · sintético {len(Str)} test_nuevo {len(Ste)}", flush=True)


def cargar(ruta):
    net = smp.Unet("timm-mobilenetv3_large_100", encoder_weights=None, in_channels=3, classes=5).to(dev)
    ck = torch.load(ruta, map_location=dev, weights_only=False)
    net.load_state_dict(ck["state_dict"] if "state_dict" in ck else ck)
    return net


def predecir(net, x01):
    x = (x01 - MEDIA) / STD
    x = F.pad(x, (0, 0, PAD, PAD), mode="reflect")
    return net(x)[:, :, PAD:-PAD, :]


def a_tensor(rgb):
    return torch.from_numpy(np.ascontiguousarray(rgb)).to(dev).permute(0, 3, 1, 2).float() / 255.0


def aumentar(x, y):
    """Volteo horizontal, brillo/contraste/tono, desenfoque leve y niebla (la huerta del 29 tenía bruma)."""
    B = x.shape[0]
    flip = torch.rand(B, device=dev) < 0.5
    x = torch.where(flip.view(B, 1, 1, 1), x.flip(3), x)
    y = torch.where(flip.view(B, 1, 1), y.flip(2), y)
    g = 1 + 0.35 * (torch.rand(B, 1, 1, 1, device=dev) - 0.5) * 2
    b = 0.12 * (torch.rand(B, 1, 1, 1, device=dev) - 0.5) * 2
    tono = 1 + 0.12 * (torch.rand(B, 3, 1, 1, device=dev) - 0.5) * 2
    x = ((x - 0.5) * g + 0.5 + b) * tono
    niebla = (torch.rand(B, 1, 1, 1, device=dev) < 0.3).float() * torch.rand(B, 1, 1, 1, device=dev) * 0.35
    x = x * (1 - niebla) + 0.75 * niebla
    return x.clamp(0, 1), y


@torch.no_grad()
def evaluar(net):
    """Sobre las sesiones reservadas: de los píxeles etiquetados, recall de camino, camino falso (en planta/obstáculo = peligroso) y IoU de camino.
    Sobre test_nuevo sintético: obstáculo dado por libre (olvido de lo viejo)."""
    net.eval()
    tp = fn = fp_pel = n_cam = n_pel = u = 0
    bajo = tot_bajo = 0                                  # suelo justo delante (abajo al centro): debe ser camino
    for i in range(0, len(Rva), 32):
        with torch.autocast(dev.type, dtype=torch.float16):
            p = predecir(net, a_tensor(Rva[i:i + 32])).argmax(1).cpu().numpy()
        e = Eva[i:i + 32]
        cam, pel = e == 1, (e == 2) | (e == 4)
        bajo += int((p[:, 175:, 100:220] == 1).sum()); tot_bajo += p[:, 175:, 100:220].size
        tp += int((cam & (p == 1)).sum()); n_cam += int(cam.sum()); n_pel += int(pel.sum()); fp_pel += int((pel & (p == 1)).sum())
        u += int(((p == 1) & (cam | pel)).sum() + (cam & (p != 1)).sum())
    obst = libre_falso = 0
    for i in range(0, len(Ste), 32):
        with torch.autocast(dev.type, dtype=torch.float16):
            p = predecir(net, a_tensor(Ste[i:i + 32])).argmax(1).cpu().numpy()
        o = Cte[i:i + 32] >= 2
        obst += int(o.sum()); libre_falso += int((o & (p == 1)).sum())
    r = {"camino_recall": round(tp / max(n_cam, 1), 4), "peligro_dado_por_camino": round(fp_pel / max(n_pel, 1), 4),
         "camino_iou_etiquetado": round(tp / max(u, 1), 4), "camino_bajo_centro": round(bajo / max(tot_bajo, 1), 4), "sintetico_libre_falso": round(libre_falso / max(obst, 1), 4)}
    r["puntaje"] = round(r["peligro_dado_por_camino"] + r["sintetico_libre_falso"] + (1 - r["camino_recall"]) * 0.5 + (1 - r["camino_bajo_centro"]), 4)
    return r


net = cargar(IN / "base_trav_20261028.pt")
base = evaluar(net)
print("BASE:", base, flush=True)
w5 = torch.tensor([0.5, 1.2, 1.0, 1.2, 1.5], device=dev)
opt = torch.optim.AdamW(net.parameters(), lr=LR, weight_decay=1e-4)
pasos = EPOCAS * (len(Rtr) // BATCH_R)
sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=LR, total_steps=pasos, pct_start=0.1)
scaler = torch.amp.GradScaler()
mejor, hist = 9.0, []
for ep in range(EPOCAS):
    net.train()
    orden = np.random.permutation(len(Rtr))
    perd, t0 = 0.0, time.time()
    for i in range(0, len(orden) - BATCH_R + 1, BATCH_R):
        idx = np.sort(orden[i:i + BATCH_R])
        xr, yr = aumentar(a_tensor(Rtr[idx]), torch.from_numpy(Etr[idx]).to(dev).long())
        js = np.sort(np.random.choice(len(Str), BATCH_S, replace=False))
        xs, ys = aumentar(a_tensor(Str[js]), torch.from_numpy(Ctr[js]).to(dev).long())
        with torch.autocast(dev.type, dtype=torch.float16):
            lr_ = predecir(net, xr)
            ls_ = predecir(net, xs)
        # reales: ignorar 255; el camino recorrido es verdad firme, los obstáculos pesan más
        loss = F.cross_entropy(lr_.float(), yr, weight=w5, ignore_index=255) + 0.7 * F.cross_entropy(ls_.float(), ys, weight=w5)
        opt.zero_grad(set_to_none=True)
        scaler.scale(loss).backward(); scaler.step(opt); scaler.update(); sched.step()
        perd += float(loss.detach())
    ev = evaluar(net)
    hist.append({"epoca": ep + 1, "perdida": round(perd / (len(orden) // BATCH_R), 4), **ev})
    marca = ""
    if ev["puntaje"] < mejor:
        mejor = ev["puntaje"]; torch.save(net.state_dict(), OUT / "sendero_mejor.pt"); marca = "  <-- mejor"
    print(f"época {ep+1}/{EPOCAS} pérdida {hist[-1]['perdida']} {ev} ({time.time()-t0:.0f} s){marca}", flush=True)
torch.save(net.state_dict(), OUT / "sendero_final.pt")
net.load_state_dict(torch.load(OUT / "sendero_mejor.pt"))
final = evaluar(net)
(OUT / "resultados.json").write_text(json.dumps({"base": base, "final": final, "historial": hist, "val_sesiones": VAL_SESIONES,
                                                 "n_real_train": int(len(Rtr)), "n_real_val": int(len(Rva))}, indent=1, ensure_ascii=False))
print("BASE :", base, "\nFINAL:", final)
