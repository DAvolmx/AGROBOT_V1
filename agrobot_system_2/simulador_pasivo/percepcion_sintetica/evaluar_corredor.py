"""Evalúa segmentación + corredor + decisión contra la GEOMETRÍA REAL de la escena.

Uso:  python3 evaluar_corredor.py [--modelo modelos/seg_interior_v1.pt] [--max 60]

La referencia NO es la máscara de verdad (que comparte con la predicha la zona ciega de la
cámara y el campo de visión): se reconstruye cada escena por semilla y se calcula la distancia
REAL al primer obstáculo dentro del pasillo que barre el robot. Así se mide también lo que la
cámara no puede ver, que es lo peligroso.

MÉTRICAS DE SEGURIDAD (las que importan para navegar):
  · peligro_no_visto: había un obstáculo a < D_PELIGRO y la decisión fue "avanzar"  (el peor error)
  · paradas_inutiles : camino libre y la decisión fue "parar"
  · error de distancia al obstáculo (cm), con signo: + = lo ve MÁS LEJOS de lo que está (peligroso)
  · error de centro y de ancho del corredor (solo donde se ven las dos paredes, sin truncar)
  · latencia por fotograma
Y robustez: brillo, desenfoque, ruido, error de inclinación (±3°), parche de oclusión y fotograma
negro/saturado.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import torch

import corredor as C
import entrenar_seg as T
import escena as E
import generar_dataset as G
import render as R

AQUI = Path(__file__).resolve().parent
D_PELIGRO = 0.85          # m: un obstáculo más cerca que esto en el pasillo barrido DEBE frenar
MEDIO = C.ANCHO_ROBOT_M / 2 + 0.05


def reconstruir(split, idx, n=24):
    """Escena y poses de una sesión, idénticas a las de generar_dataset._sesion."""
    rng = np.random.default_rng(G.BASE_SEMILLA[split] + idx)
    esc = E.escena_aleatoria(rng)
    if split == "test_nuevo":
        if rng.random() < 0.5:
            esc.suelo = dict(tipo="baldosa", color=(90, 130, 90), color2=(70, 105, 75), escala=0.55, junta=0.02)
        else:
            esc.suelo = dict(tipo="madera", color=(225, 205, 160), escala=0.2)
        esc.luz["ambiente"] = float(rng.choice([0.32, 0.42, 0.95]))
    return esc, E.poses_sesion(esc, rng, n)


def distancia_real(esc, x, y, psi):
    """Distancia (m, desde el CENTRO del robot) al primer obstáculo dentro del pasillo barrido, o None."""
    c, s = math.cos(psi), math.sin(psi)
    mejor, clase = None, None
    for o in esc.objetos():
        if isinstance(o, R.Cilindro):
            rad = max(o.r, o.copa_r)
            ang = np.linspace(0, 2 * math.pi, 90, endpoint=False)
            px, py = o.x + rad * np.cos(ang), o.y + rad * np.sin(ang)
        else:
            u = np.linspace(-0.5, 0.5, 20)
            lx, ly = np.meshgrid(u * o.lx, u * o.ly)
            ca, sa = math.cos(o.ang), math.sin(o.ang)
            px, py = (o.x + lx * ca - ly * sa).ravel(), (o.y + lx * sa + ly * ca).ravel()
        dx, dy = px - x, py - y
        fx, fy = dx * c + dy * s, -dx * s + dy * c          # x adelante, y izquierda
        ok = (fx > 0.0) & (np.abs(fy) <= MEDIO) & (fx < 3.0)
        if ok.any():
            d = float(fx[ok].min())
            if mejor is None or d < mejor:
                mejor, clase = d, {2: "maceta", 3: "borde_bajo", 4: "obstaculo"}[o.clase]
    for (ax, ay, bx, by) in esc.paredes:
        for t in np.linspace(0, 1, 120):
            px, py = ax + t * (bx - ax), ay + t * (by - ay)
            dx, dy = px - x, py - y
            fx, fy = dx * c + dy * s, -dx * s + dy * c
            if fx > 0 and abs(fy) <= MEDIO and fx < 3.0 and (mejor is None or fx < mejor):
                mejor, clase = fx, "pared"
    return mejor, clase


def decision_real(dist):
    if dist is None:
        return "avanzar"
    return "parar" if dist < D_PELIGRO else "avanzar"


def cargar_modelo(ruta, dev):
    ck = torch.load(ruta, map_location=dev, weights_only=False)
    net = T.modelo(False).to(dev)
    if "state_dict" in ck:
        net.load_state_dict(ck["state_dict"])
        meta = ck["meta"]
    else:                                   # checkpoint parcial (.mejor.pt): solo pesos, sin metadatos
        net.load_state_dict(ck)
        meta = {"nombre": Path(ruta).stem + "_parcial"}
    net.eval().half()
    return net, meta


@torch.no_grad()
def segmentar(net, rgbs, dev):
    x = T.preparar(np.ascontiguousarray(rgbs), dev).half()
    out = T.predecir(net, x).argmax(1).cpu().numpy().astype(np.uint8)
    return out


def perturbar(rgb, modo, rng):
    x = rgb.astype(np.float32)
    if modo == "oscuro":
        x *= 0.45
    elif modo == "brillante":
        x = np.clip(x * 1.7, 0, 255)
    elif modo == "ruido":
        x += rng.normal(0, 18, x.shape)
    elif modo == "desenfoque":
        import cv2
        x = cv2.GaussianBlur(x, (0, 0), 2.5)
    elif modo == "oclusion":
        h, w = x.shape[:2]
        y0, x0 = int(rng.integers(0, h // 2)), int(rng.integers(0, w // 2))
        x[y0:y0 + h // 3, x0:x0 + w // 3] = 30
    elif modo == "negro":
        x[:] = 0
    elif modo == "saturado":
        x[:] = 255
    return np.clip(x, 0, 255).astype(np.uint8)


def evaluar(net, split, dev, max_ses, modo=None, error_incl=0.0, rng=None):
    rng = rng or np.random.default_rng(0)
    filas = []
    lat = []
    carpeta = G.DATOS / split
    archivos = sorted(carpeta.glob("*.npz"))[:max_ses]
    for f in archivos:
        idx = int(f.stem.split("_")[1])
        d = np.load(f, allow_pickle=True)
        esc, poses = reconstruir(split, idx, len(d["rgb"]))
        rgb = d["rgb"]
        if modo:
            rgb = np.stack([perturbar(r, modo, rng) for r in rgb])
        t0 = time.perf_counter()
        pred = segmentar(net, rgb, dev)
        t_seg = (time.perf_counter() - t0) / len(rgb)
        for k, (x, y, psi, cab, bal, inc) in enumerate(poses[:len(rgb)]):
            t1 = time.perf_counter()
            cp = C.corredor(pred[k], inc + error_incl)
            dec = C.decidir(cp)
            lat.append(t_seg + (time.perf_counter() - t1))
            dreal, clase_real = distancia_real(esc, x, y, psi)
            cg = C.corredor(d["clase"][k], inc)
            filas.append({"clase_real": clase_real, "zona_ciega": C.zona_ciega(inc), "dist_real": dreal, "dist_pred": cp.obstaculo_m, "dec": dec.accion,
                          "dec_real": decision_real(dreal), "motivo": dec.motivo,
                          "clase_obs": cp.clase_obstaculo, "inc": inc,
                          "centro_p": cp.centro_m.get(1.4), "ancho_p": cp.ancho_m.get(1.4),
                          "centro_g": cg.centro_m.get(1.4), "ancho_g": cg.ancho_m.get(1.4),
                          "trunc_g": cg.trunc.get(1.4, (True, True)), "tipo": esc.nombre})
    return filas, float(np.mean(lat)) * 1000


def resumen(filas, lat_ms):
    n = len(filas)
    # Peligro = obstáculo en el pasillo barrido entre 0,5 y 0,85 m del centro. Por debajo de 0,5 m el robot
    # ya está pegado: ningún sensor de visión sirve y no debería haber llegado (no es un fallo de percepción).
    peligro = [f for f in filas if f["dist_real"] is not None and 0.5 <= f["dist_real"] < D_PELIGRO]
    libres = [f for f in filas if f["dec_real"] == "avanzar"]
    no_visto = [f for f in peligro if f["dec"] == "avanzar"]
    # Dos causas distintas: (a) el peligro está en la ZONA CIEGA de la cámara (límite físico);
    # (b) el peligro era visible y el modelo/regla no lo frenó (esto sí es del sistema).
    p_visibles = [f for f in peligro if f["dist_real"] >= f["zona_ciega"] + 0.03]
    p_ciegos = [f for f in peligro if f["dist_real"] < f["zona_ciega"] + 0.03]
    nv_vis = [f for f in p_visibles if f["dec"] == "avanzar"]
    nv_cie = [f for f in p_ciegos if f["dec"] == "avanzar"]
    por_clase = {}
    for cl in ("maceta", "obstaculo", "borde_bajo", "pared"):
        pc = [f for f in peligro if f["clase_real"] == cl]
        por_clase[cl] = {"peligros": len(pc), "no_vistos": sum(f["dec"] == "avanzar" for f in pc)}
    inutiles = [f for f in libres if f["dec"] == "parar"]
    err = [100 * (f["dist_pred"] - f["dist_real"]) for f in filas
           if f["dist_real"] is not None and f["dist_pred"] is not None and f["dist_real"] < 1.6]
    err_c = [100 * (f["centro_p"] - f["centro_g"]) for f in filas
             if f["centro_p"] is not None and f["centro_g"] is not None and not any(f["trunc_g"])]
    err_a = [100 * (f["ancho_p"] - f["ancho_g"]) for f in filas
             if f["ancho_p"] is not None and f["ancho_g"] is not None and not any(f["trunc_g"])]
    q = lambda v, p: None if not v else round(float(np.percentile(v, p)), 1)
    # CURVA DE DETECCIÓN: por franja de distancia REAL y clase, qué fracción VE el obstáculo
    # (alguna celda bloqueada en el pasillo barrido) y qué fracción frena (decisión != avanzar).
    # Por debajo de 0,5 m el robot ya está pegado: ningún sensor de visión sirve y no debería llegar ahí.
    bandas = {}
    for lo, hi in ((0.3, 0.5), (0.5, 0.7), (0.7, 0.9), (0.9, 1.1), (1.1, 1.3), (1.3, 1.6)):
        for cl in ("maceta", "obstaculo", "borde_bajo", "pared", "todos"):
            sub = [f for f in filas if f["dist_real"] is not None and lo <= f["dist_real"] < hi
                   and (cl == "todos" or f["clase_real"] == cl)]
            if len(sub) >= 5:
                bandas[f"{lo:.1f}-{hi:.1f}|{cl}"] = {
                    "n": len(sub),
                    "visto_pct": round(100 * float(np.mean([f["dist_pred"] is not None for f in sub])), 1),
                    "frena_pct": round(100 * float(np.mean([f["dec"] != "avanzar" for f in sub])), 1)}
    return {"n": n, "con_peligro": len(peligro), "bandas": bandas,
            "peligro_no_visto_pct": round(100 * len(no_visto) / max(len(peligro), 1), 1),
            "peligro_no_visto": len(no_visto),
            "peligro_visible": {"n": len(p_visibles), "no_visto": len(nv_vis),
                                "pct": round(100 * len(nv_vis) / max(len(p_visibles), 1), 1)},
            "peligro_en_zona_ciega": {"n": len(p_ciegos), "no_visto": len(nv_cie),
                                      "pct": round(100 * len(nv_cie) / max(len(p_ciegos), 1), 1)},
            "por_clase": por_clase,
            "paradas_inutiles_pct": round(100 * len(inutiles) / max(len(libres), 1), 1),
            "err_dist_cm": {"mediana": q(err, 50), "p10": q(err, 10), "p90": q(err, 90), "n": len(err),
                            "sobreestima_pct": round(100 * np.mean([e > 5 for e in err]), 1) if err else None},
            "err_centro_cm": {"mae": None if not err_c else round(float(np.mean(np.abs(err_c))), 1), "n": len(err_c)},
            "err_ancho_cm": {"mae": None if not err_a else round(float(np.mean(np.abs(err_a))), 1), "n": len(err_a)},
            "latencia_ms": round(lat_ms, 1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--modelo", default=str(AQUI / "modelos" / "seg_interior_v1.pt"))
    ap.add_argument("--max", type=int, default=40)
    a = ap.parse_args()
    dev = torch.device("cuda")
    net, meta = cargar_modelo(a.modelo, dev)
    out = {"modelo": meta["nombre"], "d_peligro_m": D_PELIGRO, "resultados": {}, "robustez": {}}
    for split in ("test", "test_nuevo"):
        filas, lat = evaluar(net, split, dev, a.max)
        out["resultados"][split] = resumen(filas, lat)
        r = out["resultados"][split]
        print(f"{split:10s} n={r['n']}  peligros={r['con_peligro']}  NO VISTOS={r['peligro_no_visto']} ({r['peligro_no_visto_pct']}%)  "
              f"(visibles {r['peligro_visible']['no_visto']}/{r['peligro_visible']['n']}, "
              f"zona ciega {r['peligro_en_zona_ciega']['no_visto']}/{r['peligro_en_zona_ciega']['n']})  "
              f"paradas inútiles={r['paradas_inutiles_pct']}%  err dist mediana={r['err_dist_cm']['mediana']} cm "
              f"(sobreestima {r['err_dist_cm']['sobreestima_pct']}%)  centro MAE={r['err_centro_cm']['mae']} cm  "
              f"{r['latencia_ms']} ms/frame", flush=True)
        # por tipo de escena
        out["resultados"][split]["por_tipo"] = {
            t: resumen([f for f in filas if f["tipo"] == t], lat) for t in ("sala", "pasillo", "vivero")}
    print("-- robustez (split test) --", flush=True)
    for modo, err_incl in (("oscuro", 0), ("brillante", 0), ("ruido", 0), ("desenfoque", 0), ("oclusion", 0),
                           ("negro", 0), ("saturado", 0), (None, 3.0), (None, -3.0)):
        filas, lat = evaluar(net, "test", dev, max(8, a.max // 3), modo, err_incl)
        r = resumen(filas, lat)
        etiqueta = modo or f"inclinacion {err_incl:+.0f} grados mal"
        out["robustez"][etiqueta] = r
        print(f"  {etiqueta:22s} peligros no vistos {r['peligro_no_visto']}/{r['con_peligro']} ({r['peligro_no_visto_pct']}%)  "
              f"paradas inútiles {r['paradas_inutiles_pct']}%  err dist mediana {r['err_dist_cm']['mediana']} cm", flush=True)
    (AQUI / "modelos" / f"{meta['nombre']}_corredor.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
    print("guardado en", AQUI / "modelos" / f"{meta['nombre']}_corredor.json")


if __name__ == "__main__":
    main()
