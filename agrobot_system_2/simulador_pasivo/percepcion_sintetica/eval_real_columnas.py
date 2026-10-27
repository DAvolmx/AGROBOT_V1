"""Evaluación en FOTOS REALES por columna, en METROS (lo que usa suelo_libre/camara_lidar): para cada columna con un punto del LIDAR a <2 m, ¿a qué distancia pone el
modelo el primer obstáculo (de abajo hacia arriba, racha ≥4 filas)? Se compara con la distancia del LIDAR (etiqueta débil: no ve lo bajo). Sesiones de validación = las 3 últimas.
 seguro = el modelo no sobreestima el hueco (> +0,10 m); alarma = ve el obstáculo >0,30 m antes de lo que dice el LIDAR (frena de más); mediana |error|."""
import json, math, sys
import numpy as np, torch
import entrenar_seg as T
import preparar_real as PR

d = np.load(PR.AQUI / "datos" / "real" / "real.npz")
ses = d["sesion"]; val = np.isin(ses, sorted(set(ses))[-3:])
rgb, etq, servo = d["rgb"][val], d["etq"][val], d["servo"][val]
dev = torch.device("cuda")
from barrido_tilt import inclinacion_por_servo as inc_servo


def dist_filas(incl):
    h, phi = PR.GEO["camara_altura_m"], math.radians(incl)
    f = (PR.W / 2) / math.tan(math.radians(PR.GEO["camara_hfov_deg"]) / 2)
    t = (np.arange(PR.H) - PR.H / 2) / f
    den = math.sin(phi) + t * math.cos(phi)
    with np.errstate(divide="ignore", invalid="ignore"):
        dx = h * (math.cos(phi) - t * math.sin(phi)) / den
    return np.where(den > 1e-3, dx + PR.GEO["camara_adelante_m"], np.nan)


def primera_obstaculo(mask_obst, dist):
    out = np.full(mask_obst.shape[1], np.nan)
    for c in range(mask_obst.shape[1]):
        r = 0
        for v in range(PR.H - 1, -1, -1):
            if np.isnan(dist[v]) or dist[v] > 3.0:
                break
            r = r + 1 if mask_obst[v, c] else 0
            if r >= 4:
                out[c] = dist[v + 3]
                break
    return out


def evaluar(ruta):
    net = T.modelo(False).to(dev)
    ck = torch.load(ruta, map_location=dev, weights_only=False)
    net.load_state_dict(ck["state_dict"] if "state_dict" in ck else ck)
    net.eval().half()
    err = []; seguro = alarma = n = 0
    for i in range(len(rgb)):
        with torch.no_grad():
            p = T.predecir(net, T.preparar(rgb[i:i + 1], dev).half()).argmax(1)[0].cpu().numpy()
        dist = dist_filas(PR.GEO["camara_inclinacion_deg"] + inc_servo(int(servo[i])))
        e = etq[i]
        cols = np.where((e == 4).any(0))[0]
        if not len(cols):
            continue
        dl = np.array([dist[np.where(e[:, c] == 4)[0].max()] for c in cols])        # base del obstáculo según el LIDAR
        dm = primera_obstaculo(p[:, cols] >= 2, dist)
        ok = np.isfinite(dl) & (dl < 2.0)
        dm_ = np.where(np.isfinite(dm), dm, 3.0)
        n += int(ok.sum())
        seguro += int((ok & (dm_ <= dl + 0.10)).sum())
        alarma += int((ok & (dm_ < dl - 0.30)).sum())
        err += list(np.abs(dm_ - dl)[ok])
    return {"columnas": n, "seguro_pct": round(100 * seguro / max(n, 1), 1), "alarma_pct": round(100 * alarma / max(n, 1), 1), "mediana_err_m": round(float(np.median(err)), 2)}


if __name__ == "__main__":
    res = {}
    for r in sys.argv[1:]:
        res[r] = evaluar(r)
        print(r.split("/")[-1], res[r], flush=True)
