"""Aproximación en LAZO CERRADO a un peligro: ¿frena el robot antes de tocarlo?

Uso:  python3 aproximacion.py [--modelo modelos/seg_interior_v1.pt] [--reps 2] [--procs 3]

Un robot (movimiento CINEMÁTICO: avanza en línea recta y frena con un deslizamiento dado; NO usa
la dinámica de ruedas, que no es fiable todavía) se acerca a un peligro colocado en su pasillo.
En cada paso de 0,2 s se renderiza lo que ve la cámara, se segmenta con el modelo, se calcula el
corredor y se decide. La orden llega 0,25 s tarde (percepción + enlace). Se compara:

  lidar          solo el LIDAR a 18 cm: ve lo que mide >= 18 cm de alto; frena si queda < 52 cm
                 de hueco (margen_parada 12 + deslizamiento 40, la config actual). Es lo que hay hoy.
  camara         segmentación + corredor, sin memoria
  camara_mem     lo mismo con memoria del obstáculo por odometría (cruza la zona ciega)
  camara+lidar   la orden más conservadora de las dos fuentes (lo que se recomendaría)

NO ESTÁN MEDIDOS y por eso se barren: el deslizamiento tras soltar el mando (0,15 y 0,40 m a 0,3
m/s: lo medido por el operador fue ~80 cm con el firmware antiguo y a otra velocidad) y la
odometría (aquí perfecta; en el robot la del EKF dio velocidades absurdas). Contacto = el
frente del robot (centro + 0,28 m) alcanza la huella del peligro dentro de su ancho.
"""
import argparse
import json
import math
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np
import torch

import corredor as C
import escena as E
import render as R

AQUI = Path(__file__).resolve().parent
FRENTE_M = 0.28            # del centro del robot al punto más adelantado (eje delantero + rueda)
MEDIO_ANCHO = 0.185
DT = 0.2
LATENCIA_S = 0.25
ALTURA_LIDAR = 0.18
SISTEMAS = ("lidar", "camara", "camara_mem", "camara+lidar")
PELIGROS = ("maceta_alta", "maceta_baja", "caja", "bordillo", "patas_silla")


def escena_peligro(tipo, rng, lateral, x_peligro=2.7):
    esc = E.escena_aleatoria(rng, "sala")
    esc.cilindros, esc.cajas = [], []
    esc.paredes = [(-0.8, -1.8, 5.5, -1.8), (-0.8, 1.8, 5.5, 1.8), (-0.8, -1.8, -0.8, 1.8), (5.5, -1.8, 5.5, 1.8)]
    esc.limites = (-0.8, -1.8, 5.5, 1.8)
    y = lateral
    if tipo == "maceta_alta":
        esc.cilindros.append(R.Cilindro(x_peligro, y, 0.12, 0.30, 2, (180, 95, 60), copa_r=0.2, copa_h=0.2, id=0))
        borde, alto = x_peligro - 0.2, 0.5            # contacto: la COPA (zona a no rozar)
        borde_lidar = x_peligro - 0.12                  # el haz a 18 cm solo toca la MACETA
    elif tipo == "maceta_baja":
        esc.cilindros.append(R.Cilindro(x_peligro, y, 0.10, 0.10, 2, (235, 235, 230), id=0))
        borde, alto = x_peligro - 0.10, 0.10
        borde_lidar = borde
    elif tipo == "caja":
        esc.cajas.append(R.Caja(x_peligro, y, 0.3, 0.3, 0.35, 0.2, 4, (190, 150, 100), id=0))
        borde, alto = x_peligro - 0.16, 0.35
        borde_lidar = borde
    elif tipo == "bordillo":
        esc.cajas.append(R.Caja(x_peligro, 0.0, 0.08, 1.4, 0.06, 0.0, 3, (220, 200, 60), id=0))
        borde, alto = x_peligro - 0.04, 0.06
        borde_lidar = borde
    else:                                                      # patas de silla: 4 cilindros finos
        for k, (sx, sy) in enumerate(((-0.18, -0.18), (-0.18, 0.18), (0.18, -0.18), (0.18, 0.18))):
            esc.cilindros.append(R.Cilindro(x_peligro + sx, y + sy, 0.02, 0.42, 4, (60, 45, 35), id=k))
        borde, alto = x_peligro - 0.2, 0.42
        borde_lidar = borde
    return esc, borde, alto, borde_lidar


def lidar_ve(alto):
    return alto >= ALTURA_LIDAR


@torch.no_grad()
def _segmentar(net, rgb, dev, T):
    x = T.preparar(rgb[None], dev).half()
    return T.predecir(net, x).argmax(1)[0].cpu().numpy().astype(np.uint8)


def correr(args):
    (tipo, v0, slide, inc, rep, ruta_modelo) = args
    import entrenar_seg as T
    dev = torch.device("cuda")
    cache = _MODELO.get("net")
    if cache is None:
        ck = torch.load(ruta_modelo, map_location=dev, weights_only=False)
        net = T.modelo(False).to(dev)
        net.load_state_dict(ck["state_dict"] if "state_dict" in ck else ck)
        net.eval().half()
        _MODELO["net"] = net
    net = _MODELO["net"]
    import zlib
    # hash() de cadenas cambia en cada ejecución (PYTHONHASHSEED): crc32 sí es estable y reproducible
    semilla = 7000 + zlib.crc32(f"{tipo}|{v0:.2f}|{slide:.2f}|{inc:.1f}|{rep}".encode()) % 100_000
    rng = np.random.default_rng(semilla)
    lateral = float(rng.uniform(-0.10, 0.10))
    esc, borde, alto, borde_lidar = escena_peligro(tipo, rng, lateral)
    yaw = float(rng.normal(0, math.radians(1.5)))
    out = {}
    for sistema in SISTEMAS:
        out[sistema] = _simular(sistema, esc, borde, alto, borde_lidar, v0, slide, inc, yaw, rng, net, dev, T)
    return {"tipo": tipo, "v0": v0, "slide": slide, "inc": inc, "rep": rep, "lateral": lateral, "res": out}


_MODELO: dict = {}


def _simular(sistema, esc, borde, alto, borde_lidar, v0, slide, inc, yaw, rng, net, dev, T):
    x, v, t = 0.0, v0, 0.0
    memoria = C.MemoriaObstaculo()
    cola = []                                    # (t_efecto, accion)
    accion = "avanzar"
    parada_orden = False
    a_frenado = 0.0
    t_parado = 0.0
    contacto = False
    n = 0
    decs = []
    while t < 40.0:
        # --- percepción (cada DT)
        cam = dict(R.CAMARA, inclinacion=inc)
        if sistema == "lidar":
            hueco = borde_lidar - (x + FRENTE_M)
            dec = "parar" if (lidar_ve(alto) and hueco < 0.52) else "avanzar"
        else:
            r = R.render(esc, x, 0.0, yaw, rng, cam=cam, efectos=True, pegar_plagas=False)
            mask = _segmentar(net, r["rgb"], dev, T)
            cp = C.corredor(mask, inc)
            dist_rec = None
            if sistema in ("camara_mem", "camara+lidar"):
                dist_rec = memoria.actualizar(cp, 0.0 if n == 0 else v * DT, 0.0, t)
            d = C.decidir(cp, dist_recordada=dist_rec)
            dec = d.accion
            if sistema == "camara+lidar":
                hueco = borde_lidar - (x + FRENTE_M)
                if lidar_ve(alto) and hueco < 0.52:
                    dec = "parar"
        n += 1
        cola.append((t + LATENCIA_S, dec))
        # --- aplicar órdenes que ya llegaron
        while cola and cola[0][0] <= t + 1e-9:
            accion = cola.pop(0)[1]
        # --- cinemática
        if accion == "parar" and not parada_orden:
            parada_orden = True
            # El deslizamiento al soltar el mando es proporcional al cuadrado de la velocidad
            # (referido a v0). La deceleración se fija UNA vez al dar la orden: frenar recorre una
            # distancia determinada. (Recalcularla cada paso con el hueco restante daba una
            # velocidad que decae sin llegar a cero y el robot se arrastraba hasta tocar.)
            d_frenado = max(slide * (v / v0) ** 2, 1e-3) if v0 > 0 else 1e-3
            a_frenado = v * v / (2 * d_frenado)
        if accion != "parar":
            parada_orden = False
            objetivo = v0 if accion == "avanzar" else 0.5 * v0
            v += max(-0.6 * DT, min(0.6 * DT, objetivo - v))
        else:
            v = max(0.0, v - a_frenado * DT)
        x += v * DT
        t += DT
        decs.append((t, x, v, dec, accion))
        if x + FRENTE_M >= borde - 1e-6:                      # el frente alcanzó el borde del peligro
            contacto = True
            break
        if v < 0.005 and accion == "parar":
            t_parado += DT
            if t_parado > 3.0:
                break
        else:
            t_parado = 0.0
    return {"contacto": contacto, "holgura_cm": round(100 * (borde - (x + FRENTE_M)), 1),
            "t_s": round(t, 1), "x_final": round(x, 2), "trazas": decs[:: max(1, len(decs) // 40)] if sistema != "lidar" else decs[::3]}


def resumir(res):
    out = {}
    for s in SISTEMAS:
        for tipo in (None,) + PELIGROS:
            filas = [r["res"][s] for r in res if tipo is None or r["tipo"] == tipo]
            if not filas:
                continue
            out.setdefault(s, {})[tipo or "todos"] = {
                "n": len(filas), "contactos_pct": round(100 * float(np.mean([f["contacto"] for f in filas])), 1),
                "holgura_mediana_cm": round(float(np.median([f["holgura_cm"] for f in filas])), 1)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--modelo", default=str(AQUI / "modelos" / "seg_interior_v1.pt"))
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--procs", type=int, default=3)
    ap.add_argument("--rapido", action="store_true", help="menos combinaciones (prueba)")
    a = ap.parse_args()
    t0 = time.perf_counter()
    velos = (0.30,) if a.rapido else (0.15, 0.30)
    slides = (0.40,) if a.rapido else (0.15, 0.40)
    incs = (15.5,) if a.rapido else (15.5, 33.0)
    trabajos = [(t, v, s, i, r, a.modelo) for t in PELIGROS for v in velos for s in slides for i in incs
                for r in range(a.reps)]
    print(f"{len(trabajos)} aproximaciones x {len(SISTEMAS)} sistemas", flush=True)
    res = []
    with Pool(a.procs) as pool:
        for k, r in enumerate(pool.imap_unordered(correr, trabajos)):
            res.append(r)
            if (k + 1) % 10 == 0:
                print(f"  {k+1}/{len(trabajos)}  ({time.perf_counter()-t0:.0f} s)", flush=True)
    resumen = resumir(res)
    (AQUI / "modelos").mkdir(exist_ok=True)
    nombre = Path(a.modelo).stem
    (AQUI / "modelos" / f"{nombre}_aproximacion.json").write_text(json.dumps(
        {"resumen": resumen, "corridas": res}, indent=1, default=float))
    print("\nCONTACTOS (%) por sistema y peligro")
    print(f"{'peligro':14s}" + "".join(f"{s:>14s}" for s in SISTEMAS))
    for tipo in ("todos",) + PELIGROS:
        print(f"{tipo:14s}" + "".join(f"{resumen[s][tipo]['contactos_pct']:14.1f}" for s in SISTEMAS))
    print(f"({time.perf_counter()-t0:.0f} s)")


if __name__ == "__main__":
    main()
