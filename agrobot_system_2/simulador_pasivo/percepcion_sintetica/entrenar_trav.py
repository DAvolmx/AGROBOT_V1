"""Modelo de TRANSITABILIDAD: ¿camino libre u obstáculo? (2026-10-27). Afina la UNet de 5 clases (seg_10h) con una pérdida binaria ASIMÉTRICA: dar por libre un obstáculo
(peligroso) cuesta TRAV_W_PELIGRO veces más que dar por obstáculo un suelo libre (solo frena de más), y con más variación de dominio (sombras duras, tono, brillo, desenfoque).
Binario = libre (clase 1) vs obstáculo (plantas/macetas 2, borde bajo 3, obstáculo 4); el fondo (0, sobre el horizonte) no cuenta.
Mide, antes y después y bajo perturbaciones, en val / test / test_nuevo: recall de obstáculo (por clase), tasa de «libre falso» (obstáculo dado por libre) y de «obstáculo falso».
Con --real usa también las FOTOS REALES de campo (preparar_real.py: etiquetas débiles del LIDAR; las contradice el modelo base seguro de sí mismo → se ignoran) y lleva el color sintético al de las fotos y videos reales.
Uso: python3 entrenar_trav.py [--epocas 10] [--real] [--base modelos/seg_10h_20261021_0213.mejor.pt] [--solo_eval]
Salida: modelos/trav_20261027.{mejor.pt,pt,_metricas.json}. NO se despliega. Solo sintético: no demuestra nada sobre fotos reales."""
import argparse, json, math, time
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

import entrenar_seg as T
import evaluar_corredor as EV

AQUI = Path(__file__).resolve().parent
TRAV_W_PELIGRO = 4.0
PERTURBACIONES = (None, "oscuro", "brillante", "ruido", "desenfoque", "oclusion")


def aumentar_dominio(x, y, gen):
    x, y = T.aumentar(x, y, gen)
    B, _, H, W = x.shape
    dev = x.device
    # sombra dura: un semiplano oscurecido con borde inclinado (como la de una ventana o una silla)
    ang = torch.rand(B, 1, 1, device=dev, generator=gen) * math.pi
    off = (torch.rand(B, 1, 1, device=dev, generator=gen) - 0.5) * 1.2
    yy = torch.linspace(-1, 1, H, device=dev).view(1, H, 1)
    xx = torch.linspace(-1, 1, W, device=dev).view(1, 1, W)
    plano = (xx * torch.cos(ang) + yy * torch.sin(ang) - off) > 0
    fuerza = 1.0 - torch.rand(B, 1, 1, 1, device=dev, generator=gen) * 0.55
    usar = (torch.rand(B, 1, 1, 1, device=dev, generator=gen) < 0.4)
    x = torch.where(usar & plano.unsqueeze(1), x * fuerza, x)
    # tono: rota ligeramente los canales (otro suelo, otra luz)
    tono = 1.0 + (torch.rand(B, 3, 1, 1, device=dev, generator=gen) - 0.5) * 0.35
    x = (x * tono).clamp(0, 1)
    # desenfoque ocasional
    d = (torch.rand(B, 1, 1, 1, device=dev, generator=gen) < 0.2)
    k = torch.ones(3, 1, 5, 5, device=dev) / 25.0
    xb = F.conv2d(F.pad(x, (2, 2, 2, 2), mode="reflect"), k, groups=3)
    return torch.where(d, xb, x), y


def estilo_real(x, gen, mu, sd, prob=0.5):
    """Transferencia de color de Reinhard simplificada: lleva media/desv. por canal de cada imagen sintética a las de una foto/video real elegida al azar."""
    B = x.shape[0]
    k = torch.randint(0, len(mu), (B,), device=x.device, generator=gen)
    m0 = x.mean((2, 3), keepdim=True)
    s0 = x.std((2, 3), keepdim=True) + 1e-3
    m1 = (mu[k] / 255.0).view(B, 3, 1, 1)
    s1 = (sd[k] / 255.0).view(B, 3, 1, 1)
    usar = (torch.rand(B, 1, 1, 1, device=x.device, generator=gen) < prob)
    return torch.where(usar, ((x - m0) / s0 * s1 + m1).clamp(0, 1), x)


def perdida_real(lg, etq, teacher_lg):
    """Etiquetas débiles del LIDAR (1 libre, 4 obstáculo, 255 ignorar). Se ignoran donde el modelo base, seguro de sí mismo, las contradice (obstáculos bajos que el LIDAR no ve)."""
    pt = teacher_lg.softmax(1)
    conflicto = ((etq == 1) & (pt[:, 2:].sum(1) > 0.8)) | ((etq == 4) & (pt[:, 1] > 0.9))
    y = torch.where(conflicto, torch.full_like(etq, 255), etq)
    return F.cross_entropy(lg, y.long(), ignore_index=255), float(conflicto.float().mean())


def perdida(lg, y, w5):
    ce = F.cross_entropy(lg, y, weight=w5)
    p = lg.softmax(1)
    p_libre = p[:, 1]
    valido = (y >= 1)                                          # el fondo no cuenta
    objetivo = (y == 1).float()
    bce = -(objetivo * torch.log(p_libre + 1e-6) + TRAV_W_PELIGRO * (1 - objetivo) * torch.log(1 - p_libre + 1e-6))
    return ce + 0.5 * T.dice_loss(lg, y) + 0.7 * (bce * valido).sum() / valido.sum().clamp(min=1)


@torch.no_grad()
def eval_binaria(net, rgb, cl, dev, modo=None, batch=32):
    net.eval()
    rng = np.random.default_rng(0)
    c = {"obst_pix": 0, "obst_dado_libre": 0, "libre_pix": 0, "libre_dado_obst": 0, "por_clase": {k: [0, 0] for k in (2, 3, 4)}}
    for i in range(0, len(rgb), batch):
        x = rgb[i:i + batch]
        if modo:
            x = np.stack([EV.perturbar(f, modo, rng) for f in x])
        with torch.autocast("cuda", dtype=torch.float16):
            p = T.predecir(net, T.preparar(np.ascontiguousarray(x), dev)).argmax(1).cpu().numpy()
        y = cl[i:i + batch]
        libre_pred = p == 1
        for k in (2, 3, 4):
            m = y == k
            c["por_clase"][k][0] += int(m.sum())
            c["por_clase"][k][1] += int((m & ~libre_pred).sum())
        o = (y >= 2)
        c["obst_pix"] += int(o.sum())
        c["obst_dado_libre"] += int((o & libre_pred).sum())
        l = y == 1
        c["libre_pix"] += int(l.sum())
        c["libre_dado_obst"] += int((l & ~libre_pred & (p >= 2)).sum())
    return {"libre_falso": round(c["obst_dado_libre"] / max(c["obst_pix"], 1), 4), "obstaculo_falso": round(c["libre_dado_obst"] / max(c["libre_pix"], 1), 4),
            "recall_obstaculo_por_clase": {T.R.CLASES[k]: round(v[1] and 1 - v[1] / max(v[0], 1) or (1.0 if v[0] else None), 4) for k, v in c["por_clase"].items()}}


@torch.no_grad()
def eval_real(net, rgb, etq, dev, batch=32):
    """Contra las etiquetas débiles: «obstáculo dado por libre» (peligro) en la banda del LIDAR y «libre dado por obstáculo» en el suelo libre."""
    net.eval()
    ob = ol = fl = fo = 0
    for i in range(0, len(rgb), batch):
        with torch.autocast("cuda", dtype=torch.float16):
            p = T.predecir(net, T.preparar(np.ascontiguousarray(rgb[i:i + batch]), dev)).argmax(1).cpu().numpy()
        e = etq[i:i + batch]
        o, l = e == 4, e == 1
        ob += int(o.sum()); ol += int((o & (p == 1)).sum())
        fl += int(l.sum()); fo += int((l & (p >= 2)).sum())
    return {"libre_falso": round(ol / max(ob, 1), 4), "obstaculo_falso": round(fo / max(fl, 1), 4), "pix_obst": ob, "pix_libre": fl}


def evaluar_todo(net, dev, datos):
    r = {sp: {str(m): eval_binaria(net, *datos[sp], dev, m) for m in PERTURBACIONES} for sp in datos if sp != "real_val"}
    if "real_val" in datos:
        r["real_val"] = eval_real(net, *datos["real_val"], dev)
    return r


def cargar_modelo(ruta, dev):
    net = T.modelo(False).to(dev)
    ck = torch.load(ruta, map_location=dev, weights_only=False)
    net.load_state_dict(ck["state_dict"] if "state_dict" in ck else ck)
    return net


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epocas", type=int, default=10)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--lr", type=float, default=4e-4)
    ap.add_argument("--base", default=str(AQUI / "modelos" / "seg_10h_20261021_0213.mejor.pt"))
    ap.add_argument("--nombre", default="trav_20261027")
    ap.add_argument("--solo_eval", action="store_true")
    ap.add_argument("--real", action="store_true", help="usa las fotos reales de campo (preparar_real.py)")
    ap.add_argument("--lote_real", type=int, default=6)
    ap.add_argument("--datos", default="", help="carpeta de dataset sintético alternativa (p. ej. datos_cam_real, con la cámara real); test_nuevo sigue siendo el de siempre")
    a = ap.parse_args()
    torch.manual_seed(0)
    np.random.seed(0)
    dev = torch.device("cuda")
    t0 = time.perf_counter()
    datos = {}
    if a.datos:
        datos["test_nuevo"] = T.cargar("test_nuevo")[:2]
        T.DATOS = AQUI / a.datos
    rgb_tr, cl_tr, _ = T.cargar("train")
    datos["val"], datos["test"] = T.cargar("val")[:2], T.cargar("test")[:2]
    if not a.datos:
        datos["test_nuevo"] = T.cargar("test_nuevo")[:2]
    real_tr = mu = sd = None
    if a.real:
        d = np.load(AQUI / "datos" / "real" / "real.npz")
        ses = d["sesion"]
        val_s = sorted(set(ses))[-3:]                                   # las 3 sesiones más recientes quedan fuera: validan lo real
        mval = np.isin(ses, val_s)
        real_tr = (d["rgb"][~mval], d["etq"][~mval])
        datos["real_val"] = (d["rgb"][mval], d["etq"][mval])
        e = np.load(AQUI / "datos" / "real" / "estilo.npz")
        mu, sd = torch.tensor(e["mu"], device=dev), torch.tensor(e["sd"], device=dev)
        print(f"real: {len(real_tr[0])} fotos de entrenamiento · {int(mval.sum())} de validación ({val_s})", flush=True)
    print(f"datos cargados ({time.perf_counter()-t0:.0f} s)", flush=True)
    net = cargar_modelo(a.base, dev)
    teacher = cargar_modelo(a.base, dev).eval().half() if a.real else None
    antes = evaluar_todo(net, dev, datos)
    if "real_val" in antes: print("ANTES en fotos reales (etiquetas débiles):", antes["real_val"], flush=True)
    print("ANTES (libre_falso / obstáculo_falso, test_nuevo):", {m: (v["libre_falso"], v["obstaculo_falso"]) for m, v in antes["test_nuevo"].items()}, flush=True)
    if a.solo_eval:
        print(json.dumps(antes, indent=1))
        return
    frec = np.bincount(cl_tr.ravel(), minlength=T.N_CLASES).astype(float)
    frec /= frec.sum()
    w5 = torch.tensor(np.clip((np.median(frec) / np.maximum(frec, 1e-9)) ** 0.5, 0.5, 6.0), dtype=torch.float32, device=dev)
    opt = torch.optim.AdamW(net.parameters(), lr=a.lr, weight_decay=1e-4)
    pasos = a.epocas * (len(rgb_tr) // a.batch)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=pasos, pct_start=0.1)
    scaler = torch.amp.GradScaler()
    gen = torch.Generator(device=dev)
    gen.manual_seed(0)
    mejor, hist = 9.0, []
    for ep in range(a.epocas):
        net.train()
        orden = np.random.permutation(len(rgb_tr))
        perd, t_ep = 0.0, time.perf_counter()
        for i in range(0, len(orden) - a.batch + 1, a.batch):
            idx = np.sort(orden[i:i + a.batch])
            x = torch.from_numpy(rgb_tr[idx]).to(dev).permute(0, 3, 1, 2).float() / 255.0
            y = torch.from_numpy(cl_tr[idx]).to(dev).long()
            x, y = aumentar_dominio(x, y, gen)
            if mu is not None:
                x = estilo_real(x, gen, mu, sd)
            with torch.autocast("cuda", dtype=torch.float16):
                lg = T.predecir(net, T.normalizar(x))
            loss = perdida(lg.float(), y, w5)
            if real_tr is not None:
                ir = np.random.randint(0, len(real_tr[0]), a.lote_real)
                xr = torch.from_numpy(real_tr[0][ir]).to(dev).permute(0, 3, 1, 2).float() / 255.0
                er = torch.from_numpy(real_tr[1][ir]).to(dev)
                xr, er = T.aumentar(xr, er, gen)
                with torch.autocast("cuda", dtype=torch.float16):
                    lgr = T.predecir(net, T.normalizar(xr))
                    with torch.no_grad():
                        lgt = T.predecir(teacher, T.normalizar(xr).half())
                lr_, _ = perdida_real(lgr.float(), er, lgt.float())
                loss = loss + 0.5 * lr_
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            sched.step()
            perd += float(loss.detach())
        # criterio de selección: peligro (libre falso) + 0,25·obstáculo falso, sobre val y test_nuevo con y sin perturbar
        ev = {sp: {str(m): eval_binaria(net, *datos[sp], dev, m) for m in (None, "oscuro", "ruido")} for sp in ("val", "test_nuevo")}
        puntaje = float(np.mean([v["libre_falso"] + 0.25 * v["obstaculo_falso"] for s in ev.values() for v in s.values()]))
        if "real_val" in datos:
            rv = eval_real(net, *datos["real_val"], dev)
            puntaje += rv["libre_falso"] + 0.25 * rv["obstaculo_falso"]
        hist.append({"epoca": ep + 1, "perdida": round(perd / (len(orden) // a.batch), 4), "puntaje": round(puntaje, 4)})
        marca = ""
        if puntaje < mejor:
            mejor = puntaje
            torch.save(net.state_dict(), AQUI / "modelos" / f"{a.nombre}.mejor.pt")
            marca = "  <-- mejor"
        print(f"  época {ep+1}/{a.epocas} pérdida {hist[-1]['perdida']} puntaje {puntaje:.4f} ({time.perf_counter()-t_ep:.0f} s){marca}", flush=True)
    net.load_state_dict(torch.load(AQUI / "modelos" / f"{a.nombre}.mejor.pt"))
    despues = evaluar_todo(net, dev, datos)
    meta = dict(torch.load(a.base, map_location="cpu", weights_only=False).get("meta", {})) if "state_dict" in torch.load(a.base, map_location="cpu", weights_only=False) else {}
    meta.update(nombre=a.nombre, afinado_de=Path(a.base).name, tipo="transitabilidad (libre/obstaculo)", peso_peligro=TRAV_W_PELIGRO, fecha=time.strftime("%Y-%m-%d %H:%M"),
                NO_DEMOSTRADO="Solo imágenes sintéticas; sin evaluación en fotos reales.")
    torch.save({"state_dict": net.state_dict(), "meta": meta}, AQUI / "modelos" / f"{a.nombre}.pt")
    (AQUI / "modelos" / f"{a.nombre}_metricas.json").write_text(json.dumps({"antes": antes, "despues": despues, "historial": hist, "meta": meta}, indent=1, ensure_ascii=False, default=str))
    if "real_val" in despues: print("DESPUÉS en fotos reales (etiquetas débiles):", despues["real_val"], flush=True)
    print("DESPUÉS (libre_falso / obstáculo_falso, test_nuevo):", {m: (v["libre_falso"], v["obstaculo_falso"]) for m, v in despues["test_nuevo"].items()}, flush=True)
    print(f"listo ({time.perf_counter()-t0:.0f} s)")


if __name__ == "__main__":
    main()
