#!/usr/bin/env python3
"""
Entrenador: cuanto más corre, mejor conduce el robot.

    python3 simulador/entrenar.py --todo --episodios 20
    python3 simulador/entrenar.py --recolectar 30      # solo juntar datos
    python3 simulador/entrenar.py --entrenar           # solo entrenar
    python3 simulador/entrenar.py --evaluar            # con política vs sin ella

Tres pasos, que se pueden correr por separado:

  1. RECOLECTAR. En cada episodio se genera un escenario al azar (sala,
     sendero, obstáculos y plantas distintos) y el robot lo recorre con el
     piloto de la ruta ideal, que va por el centro del sendero. Cada pulso se
     guarda como un ejemplo: "percibiendo esto, había que hacer esto". Variar
     el escenario es lo que evita aprender un mapa en vez de a conducir.

  2. ENTRENAR. Una red pequeña (ver `politica.py`) aprende a imitar esos
     ejemplos. Sale `politica.pt`, que el robot real puede cargar.

  3. EVALUAR. Se corren los mismos escenarios con la política y sin ella, y
     se comparan desvío de la ruta, plantas pisadas y choques. Si no mejora,
     se dice; no se da por bueno un modelo porque sí.

Los datos se acumulan en `~/.agrobot/aprendizaje/dataset.csv`: cada tanda de
entrenamiento suma a la anterior.
"""
import argparse
import asyncio
import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))

import mundo as mundo_mod          # noqa: E402
import politica as pol             # noqa: E402
import correr                      # noqa: E402

CARPETA = Path.home() / ".agrobot" / "aprendizaje"
DATASET = CARPETA / "dataset.csv"
MODELO = CARPETA / "politica.pt"
COLUMNAS = list(pol.ENTRADAS) + ["accion", "escenario", "origen"]


# --------------------------------------------------------------- recolectar
async def _episodio_ideal(escenario, cfg, segundos, semilla):
    """Un recorrido ideal del que salen ejemplos de conducción buena."""
    ep = correr.Episodio(escenario, cfg, segundos + 5, semilla,
                         tiempo_real=False, usar_slam=False)
    ep.calibrar_servo = False          # en recolección interesa el volumen
    tarea = asyncio.create_task(ep._correr())
    # En modo rápido el episodio entero corre entre dos esperas, así que el
    # recorrido ideal se arranca desde dentro del propio bucle de física.
    for _ in range(2000):
        await ep.reloj._sleep(0)
        if getattr(ep, "explorador", None) is not None and ep.reloj.t > 0.5:
            break
    ep.explorador.detener()
    ep.iniciar_recorrido_ideal()
    await tarea
    return ep


def recolectar(episodios, cfg, segundos=120, semilla0=0, escenarios=None):
    """Genera ejemplos en escenarios al azar y los añade al dataset."""
    CARPETA.mkdir(parents=True, exist_ok=True)
    nuevo = not DATASET.exists()
    total, resumen = 0, []
    with DATASET.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNAS)
        if nuevo:
            w.writeheader()
        for k in range(episodios):
            semilla = semilla0 + k
            esc = (escenarios[k % len(escenarios)] if escenarios else "aleatorio")
            t0 = time.perf_counter()
            ep = asyncio.run(_episodio_ideal(esc, cfg, segundos, semilla))
            filas = 0
            for m in ep.registro_ideal:
                fila = dict(zip(pol.ENTRADAS,
                                pol.caracteristicas(m, m.get("desvio_cm", 0) / 100)))
                fila["accion"] = m["accion"]
                fila["escenario"] = ep.mundo.nombre
                fila["origen"] = "ideal"
                w.writerow(fila)
                filas += 1
            total += filas
            resumen.append({"escenario": ep.mundo.nombre, "ejemplos": filas,
                            "recorrido_m": round(ep.robot.recorrido, 1)})
            print(f"  [{k+1}/{episodios}] {ep.mundo.nombre:22s} {filas:4d} ejemplos "
                  f"({ep.robot.recorrido:5.1f} m, {time.perf_counter()-t0:4.1f} s)", flush=True)
    print(f"-> {total} ejemplos nuevos en {DATASET}")
    return resumen


def espejar(x, accion_idx):
    """Refleja un ejemplo de izquierda a derecha.

    El robot es simétrico: si en una situación había que girar a la
    izquierda, en su imagen especular hay que girar a la derecha. Sin esto
    los datos salen torcidos (los bucles aleatorios giran casi siempre para
    el mismo lado: 1184 "adelante", 318 "izquierda" y 7 "derecha" en la
    primera tanda) y el modelo no aprende a girar al otro lado."""
    e = list(pol.ENTRADAS)
    x = list(x)
    x[e.index("lidar_izquierda")], x[e.index("lidar_derecha")] = \
        x[e.index("lidar_derecha")], x[e.index("lidar_izquierda")]
    for campo in ("hueco_relativo", "sendero_lateral", "sendero_rumbo", "omega_actual"):
        x[e.index(campo)] = -x[e.index(campo)]
    accion = pol.ACCIONES[accion_idx]
    if accion == "izquierda":
        accion = "derecha"
    elif accion == "derecha":
        accion = "izquierda"
    return x, pol.ACCIONES.index(accion)


def cargar_dataset(espejo=True):
    if not DATASET.exists():
        sys.exit(f"no hay datos todavía: correr --recolectar (falta {DATASET})")
    X, y, escenarios = [], [], set()
    with DATASET.open() as f:
        for fila in csv.DictReader(f):
            if fila["accion"] not in pol.ACCIONES:
                continue
            x = [float(fila[c]) for c in pol.ENTRADAS]
            a_idx = pol.ACCIONES.index(fila["accion"])
            X.append(x); y.append(a_idx)
            escenarios.add(fila.get("escenario", ""))
            if espejo:
                xe, ye = espejar(x, a_idx)
                X.append(xe); y.append(ye)
    return np.array(X, np.float32), np.array(y, np.int64), sorted(escenarios)


# ------------------------------------------------------------------ entrenar
def entrenar(epocas=60, lote=256, oculta=64, semilla=0):
    import torch
    import torch.nn as nn

    # En CPU a propósito: la rueda de PyTorch instalada (cu130) no trae
    # kernels para la GPU de esta Jetson, y esta red es diminuta.
    torch.set_num_threads(4)
    X, y, escenarios = cargar_dataset()
    if len(X) < 200:
        sys.exit(f"hacen falta más ejemplos ({len(X)}): correr --recolectar")
    torch.manual_seed(semilla)
    cuenta = np.bincount(y, minlength=len(pol.ACCIONES))
    print(f"{len(X)} ejemplos de {len(escenarios)} escenarios | por acción: "
          + ", ".join(f"{a}={c}" for a, c in zip(pol.ACCIONES, cuenta)))

    medias, desv = X.mean(axis=0), X.std(axis=0) + 1e-6
    Xn = (X - medias) / desv
    idx = np.random.default_rng(semilla).permutation(len(Xn))
    corte = int(len(idx) * 0.8)
    ent, val = idx[:corte], idx[corte:]
    Xe = torch.tensor(Xn[ent]); ye = torch.tensor(y[ent])
    Xv = torch.tensor(Xn[val]); yv = torch.tensor(y[val])

    red = pol.crear_red(X.shape[1], len(pol.ACCIONES), oculta)
    opt = torch.optim.Adam(red.parameters(), lr=2e-3)
    # Las clases están desbalanceadas (se avanza mucho más de lo que se gira):
    # sin pesos, la red aprendería a decir "adelante" siempre.
    pesos = torch.tensor((cuenta.sum() / np.maximum(cuenta, 1)) ** 0.5, dtype=torch.float32)
    perdida = nn.CrossEntropyLoss(weight=pesos)

    mejor, mejores_pesos = 0.0, None
    for e in range(epocas):
        red.train()
        for i in range(0, len(Xe), lote):
            opt.zero_grad()
            salida = red(Xe[i:i + lote])
            p = perdida(salida, ye[i:i + lote])
            p.backward()
            opt.step()
        red.eval()
        with torch.no_grad():
            acierto = float((red(Xv).argmax(1) == yv).float().mean())
        if acierto > mejor:
            mejor, mejores_pesos = acierto, {k: v.clone() for k, v in red.state_dict().items()}
        if (e + 1) % 15 == 0:
            print(f"  época {e+1:3d}: acierto en validación {acierto*100:.1f}%")

    red.load_state_dict(mejores_pesos)
    MODELO.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"pesos": red.state_dict(), "medias": medias.tolist(),
                "desviaciones": desv.tolist(), "entradas": list(pol.ENTRADAS),
                "acciones": list(pol.ACCIONES), "oculta": oculta,
                "version_entradas": pol.VERSION_ENTRADAS,
                "entrenado": time.strftime("%Y-%m-%d %H:%M"), "ejemplos": len(X),
                "acierto": round(mejor, 4), "escenarios": escenarios}, MODELO)
    print(f"-> {MODELO} | acierto en validación {mejor*100:.1f}% con {len(X)} ejemplos")
    return {"ejemplos": len(X), "acierto": mejor}


# ------------------------------------------------------------------- evaluar
def evaluar(episodios=4, cfg=None, segundos=90, semilla0=1000):
    """Mismos escenarios con la política y sin ella."""
    politica = pol.Politica(MODELO)
    if not politica.disponible:
        sys.exit(f"no hay modelo en {MODELO}: correr --entrenar")
    print(f"modelo: {politica.info}")
    filas = []
    for k in range(episodios):
        semilla = semilla0 + k
        medidas = {}
        for usar in (False, True):
            ep = correr.Episodio("aleatorio", cfg, segundos, semilla)
            ep.calibrar_servo = False
            # `montar()` se la pasa al explorador al crearlo dentro de correr().
            ep.politica = pol.Politica(MODELO) if usar else None
            r = ep.correr()
            medidas["con" if usar else "sin"] = r
        filas.append(medidas)
        c, s = medidas["con"], medidas["sin"]
        print(f"  [{k+1}/{episodios}] {c['escenario']:20s} "
              f"desvío {s['desvio_ruta_medio_cm']:6.1f} -> {c['desvio_ruta_medio_cm']:6.1f} cm | "
              f"pisadas {s['plantas_pisadas']:2d} -> {c['plantas_pisadas']:2d} | "
              f"choques {s['colisiones']:2d} -> {c['colisiones']:2d}", flush=True)

    def media(lado, clave):
        return float(np.mean([f[lado][clave] for f in filas]))

    resumen = {m: {"desvio_cm": round(media(m, "desvio_ruta_medio_cm"), 1),
                   "pisadas": round(media(m, "plantas_pisadas"), 2),
                   "choques": round(media(m, "colisiones"), 2),
                   "recorrido_m": round(media(m, "recorrido_m"), 2)}
               for m in ("sin", "con")}
    mejora = (resumen["sin"]["desvio_cm"] - resumen["con"]["desvio_cm"])
    print(json.dumps(resumen, ensure_ascii=False, indent=1))
    print(f"=> la política {'MEJORA' if mejora > 0 else 'NO mejora'} el desvío en "
          f"{abs(mejora):.1f} cm de media")
    (CARPETA / "evaluacion.json").write_text(json.dumps(resumen, indent=1, ensure_ascii=False))
    return resumen


def main():
    p = argparse.ArgumentParser(description="Entrenador del AgroBot")
    p.add_argument("--recolectar", type=int, metavar="N", help="episodios de recolección")
    p.add_argument("--entrenar", action="store_true")
    p.add_argument("--evaluar", type=int, nargs="?", const=4, metavar="N")
    p.add_argument("--todo", action="store_true", help="recolectar + entrenar + evaluar")
    p.add_argument("--episodios", type=int, default=10, help="episodios con --todo")
    p.add_argument("--segundos", type=float, default=120)
    p.add_argument("--epocas", type=int, default=60)
    p.add_argument("--config", default=str(AQUI.parent / "jetson" / "backend" / "agrobot_config.json"))
    a = p.parse_args()

    cfg = json.loads(Path(a.config).read_text())
    cfg["autonomia"].update(sendero_activo=True, sendero_paja=True)
    cfg["autonomia"].setdefault("escala_velocidad", 0.5)

    if a.todo:
        print("== 1. recolectando ==")
        recolectar(a.episodios, cfg, a.segundos)
        print("== 2. entrenando ==")
        entrenar(a.epocas)
        print("== 3. evaluando ==")
        evaluar(max(2, a.episodios // 4), cfg, a.segundos)
        return
    if a.recolectar:
        recolectar(a.recolectar, cfg, a.segundos)
    if a.entrenar:
        entrenar(a.epocas)
    if a.evaluar:
        evaluar(a.evaluar, cfg, a.segundos)
    if not (a.recolectar or a.entrenar or a.evaluar):
        p.print_help()


if __name__ == "__main__":
    main()
