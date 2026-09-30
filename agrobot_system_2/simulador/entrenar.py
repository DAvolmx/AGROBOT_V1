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
     ejemplos. Sale `politica_candidata.pt` (y una copia por tanda); el
     robot real carga `politica.pt`, que solo cambia al desplegar desde el
     dashboard (pestaña Entrenamiento → "Desplegar el mejor").

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

MUNDOS_DIR = AQUI / "mundos"
CARPETA = Path.home() / ".agrobot" / "aprendizaje"
DATASET = CARPETA / "dataset.csv"
# El entrenamiento escribe un CANDIDATO. `politica.pt` es el que carga el
# robot real (politica.RUTA_POR_DEFECTO) y solo se cambia desplegando a
# propósito (botón "Desplegar el mejor" del dashboard): antes cada tanda lo
# sobrescribía sin validar y el robot usaba el último al reiniciar.
MODELO = CARPETA / "politica_candidata.pt"
COLUMNAS = list(pol.ENTRADAS) + ["accion", "escenario", "origen"]
RECORRIDO_MINIMO_M = 1.0
ORIGENES = ("ideal_dr",)         # ver cargar_dataset; --todos-los-datos = None


# --------------------------------------------------------------- variaciones
def mundos_disponibles():
    """Escenarios incluidos (menos el aleatorio, que ya se usa aparte) y
    mapas dibujados en el editor del dashboard."""
    incluidos = [n for n in mundo_mod.ESCENARIOS if n != "aleatorio"]
    dibujados = sorted(p.stem for p in MUNDOS_DIR.glob("*.json"))
    return incluidos + dibujados


def _dist_a_ruta(x, y, ruta) -> float:
    a, b = ruta[:-1], ruta[1:]
    ab = b - a
    t = np.clip(((x - a[:, 0]) * ab[:, 0] + (y - a[:, 1]) * ab[:, 1])
                / np.maximum((ab ** 2).sum(axis=1), 1e-9), 0, 1)
    return float(np.hypot(x - a[:, 0] - t * ab[:, 0], y - a[:, 1] - t * ab[:, 1]).min())


def mundo_variado(nombre, rng):
    """Carga un escenario o un mapa dibujado y lo varía en cada episodio.

    Entrenar siempre con el mapa EXACTO enseña ese mapa, no a conducir. Se
    cambia lo que de verdad cambia en el campo de un día para otro: lo seco
    que está el suelo, de qué es el borde del surco, alguna rama o maceta
    tirada, y desde qué punto del recorrido se empieza."""
    import mundo as M

    if nombre in M.ESCENARIOS:
        m, _ = M.ESCENARIOS[nombre]()
        datos = {"ancho_sendero": m.ancho_sendero, "ancho_paja": m.ancho_paja}
        base = [{"tipo": t, "x": float(o[0]), "y": float(o[1])} for o, t in zip(m.objetos, m.tipos)]
    else:
        datos = json.loads((MUNDOS_DIR / f"{nombre}.json").read_text())
        m = M.Mundo.desde_dict(datos)
        base = list(datos.get("objetos", []))
    m.seco = float(rng.uniform(0.0, 1.0))
    m.borde = str(rng.choice(["paja", "vegetacion", "mixto", "tierra"]))
    m.ancho_sendero = max(0.35, datos.get("ancho_sendero", 0.65) * float(rng.uniform(0.85, 1.2)))
    m.ancho_paja = max(0.15, datos.get("ancho_paja", 0.35) * float(rng.uniform(0.7, 1.3)))

    # Estorbos sueltos: ramas que el LIDAR ve pero se atraviesan, y macetas
    # que sí son obstáculo. Se ponen cerca del sendero, que es donde molestan.
    ruta = np.asarray(m.sendero, np.float64)
    objetos = base
    if len(ruta) > 2:
        # Al lado del sendero, no encima: si cae en medio, el recorrido ideal
        # se queda frenado y el episodio no enseña nada. Se mide contra TODO
        # el sendero y no solo contra el punto de partida: en las curvas en U
        # un objeto "al lado" de un tramo caía encima del de vuelta (el
        # 2026-09-30 un episodio arrancó dentro de una maceta: 24 choques).
        libre = m.ancho_sendero / 2 + 0.35
        for _ in range(int(rng.integers(0, 5))):
            i = int(rng.integers(0, len(ruta)))
            ang = float(rng.uniform(0, 2 * math.pi))
            r = m.ancho_sendero / 2 + 0.45 + float(rng.uniform(0, 0.7))
            x, y = float(ruta[i][0] + r * math.cos(ang)), float(ruta[i][1] + r * math.sin(ang))
            if _dist_a_ruta(x, y, ruta) < libre:
                continue
            objetos.append({"tipo": str(rng.choice(["rama", "arbusto", "maceta", "piedra"])),
                            "x": x, "y": y})
    m.poner_objetos(objetos)

    # Arrancar en un punto cualquiera del recorrido, mirando hacia donde sigue.
    if len(ruta) > 2:
        i = int(rng.integers(0, len(ruta) - 1))
        j = (i + 1) % len(ruta)
        pose = (float(ruta[i][0]), float(ruta[i][1]),
                math.atan2(ruta[j][1] - ruta[i][1], ruta[j][0] - ruta[i][0]))
    else:
        x0, y0, x1, y1 = m.limites
        pose = ((x0 + x1) / 2, (y0 + y1) / 2, 0.0)
    return m, pose


def escenario_para(k, mundos, mezcla, rng):
    """Uno de los mapas pedidos (variado), o un escenario al azar."""
    if mundos and rng.random() < mezcla:
        return mundo_variado(str(rng.choice(mundos)), rng)
    return "aleatorio"


def aleatorizar(ep, rng) -> dict:
    """Sortea las condiciones del robot para este episodio.

    Es la "aleatorización de dominio": en vez de UN chasis simulado ideal,
    uno distinto cada vez dentro de lo que se midió o se puede esperar en el
    campo. Así la red aprende a conducir con cualquiera de ellos, y el robot
    real, que es uno más de esa familia, no la pilla por sorpresa."""
    r = ep.robot
    r.rendimiento_giro = float(rng.uniform(0.50, 0.80))    # patina más o menos al girar
    r.decel_inercia = float(rng.uniform(0.15, 0.30))       # desliza 60-100 cm al soltar
    r.escala_latencia = float(rng.uniform(0.6, 2.0))       # BLE más o menos cargado
    ep.slam.ruido = float(rng.uniform(0.5, 1.5))
    ep.fallos_lidar.cambiar(str(rng.choice(["usb", "usb", "uart", "ninguna"])))
    return {"giro": round(r.rendimiento_giro, 2), "desliz": round(r.decel_inercia, 2),
            "latencia": round(r.escala_latencia, 2), "slam": round(ep.slam.ruido, 2),
            "lidar": ep.fallos_lidar.conexion}


# --------------------------------------------------------------- recolectar
async def _episodio_ideal(escenario, cfg, segundos, semilla, rng=None):
    """Un recorrido ideal del que salen ejemplos de conducción buena."""
    ep = correr.Episodio(escenario, cfg, segundos + 5, semilla,
                         tiempo_real=False, usar_slam=False)
    ep.condiciones = aleatorizar(ep, rng) if rng is not None else {}
    # Calibrar la inclinación al arrancar: si la cámara no ve el sendero, los
    # ejemplos salen sin datos de cámara y el modelo no aprende a corregir.
    ep.calibrar_servo = True
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


def recolectar(episodios, cfg, segundos=120, semilla0=0, escenarios=None,
               mundo_nombre=None, mezcla=0.7, mundos=None):
    """Genera ejemplos y los añade al dataset.

    Con `mundos` (o `mundo_nombre`, uno solo) se entrena sobre esos mapas
    con variaciones, mezclados con escenarios al azar para que lo aprendido
    no valga solo ahí. Cada episodio sortea además las condiciones del
    robot (ver `aleatorizar`)."""
    mundos = list(mundos or ([mundo_nombre] if mundo_nombre else []))
    CARPETA.mkdir(parents=True, exist_ok=True)
    nuevo = not DATASET.exists()
    total, resumen = 0, []
    with DATASET.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNAS)
        if nuevo:
            w.writeheader()
        for k in range(episodios):
            semilla = semilla0 + k
            rng = np.random.default_rng(semilla)
            if escenarios:
                esc = escenarios[k % len(escenarios)]
            else:
                esc = escenario_para(k, mundos, mezcla, rng)
            t0 = time.perf_counter()
            try:
                ep = asyncio.run(_episodio_ideal(esc, cfg, segundos, semilla, rng))
            except Exception as exc:
                # Una noche entera de entrenamiento no se puede caer por un
                # episodio raro: se anota y se sigue.
                print(f"  [{k+1}/{episodios}] episodio descartado: "
                      f"{type(exc).__name__}: {exc}", flush=True)
                continue
            if ep.robot.recorrido < RECORRIDO_MINIMO_M:
                # Atascado desde el principio: sus ejemplos dirían "adelante"
                # con el robot quieto contra algo, y enseñarían a empujar.
                print(f"  [{k+1}/{episodios}] {ep.mundo.nombre:22s} descartado: solo "
                      f"{ep.robot.recorrido:.1f} m recorridos ({ep.robot.colisiones} choques)",
                      flush=True)
                continue
            filas = 0
            for m in ep.registro_ideal:
                fila = dict(zip(pol.ENTRADAS,
                                pol.caracteristicas(m, m.get("desvio_cm", 0) / 100)))
                fila["accion"] = m["accion"]
                fila["escenario"] = ep.mundo.nombre
                # "ideal_dr": recorrido ideal con aleatorización de dominio y
                # el simulador realista (latencia, SLAM con fallos, cámara
                # nueva). Se distingue de lo recolectado antes del 2026-09-30.
                fila["origen"] = "ideal_dr" if ep.condiciones else "ideal"
                w.writerow(fila)
                filas += 1
            total += filas
            resumen.append({"escenario": ep.mundo.nombre, "ejemplos": filas,
                            "recorrido_m": round(ep.robot.recorrido, 1)})
            print(f"  [{k+1}/{episodios}] {ep.mundo.nombre:22s} {filas:4d} ejemplos "
                  f"({ep.robot.recorrido:5.1f} m, {time.perf_counter()-t0:4.1f} s) "
                  f"{ep.condiciones}", flush=True)
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


def cargar_dataset(espejo=True, origenes=("ideal_dr",)):
    """`origenes`: de qué recolecciones se entrena. Por defecto solo las del
    simulador realista (2026-09-30 en adelante): las anteriores se hicieron
    con el explorador peleando el mando al piloto ideal, y tienen casi tantos
    giros como avances. None = todo."""
    if not DATASET.exists():
        sys.exit(f"no hay datos todavía: correr --recolectar (falta {DATASET})")
    X, y, escenarios = [], [], set()
    with DATASET.open() as f:
        for fila in csv.DictReader(f):
            if fila["accion"] not in pol.ACCIONES:
                continue
            if origenes is not None and fila.get("origen") not in origenes:
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
    X, y, escenarios = cargar_dataset(origenes=ORIGENES)
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
    # Exponente 0,75, no 0,5: con 0,5 el empuje se quedaba corto para las
    # clases muy raras ("atrás" es el 1 % de los ejemplos).
    pesos = torch.tensor((cuenta.sum() / np.maximum(cuenta, 1)) ** 0.75, dtype=torch.float32)
    perdida = nn.CrossEntropyLoss(weight=pesos)

    def notas(salida, cierto):
        """Acierto llano y acierto EQUILIBRADO (media de aciertos por clase).

        Quedarse con la época de mejor acierto llano fue lo que rompió el
        modelo anterior: con 4000 "adelante" y 40 "atrás", la red que dice
        siempre "adelante" saca un 79 % y gana. Y eso fue exactamente lo que
        salió: el .pt del 2026-09-20 respondía "adelante" ante una pared a
        40 cm y en un callejón sin salida. Los pesos de la pérdida empujaban
        al equilibrio y el criterio de selección deshacía el empujón.

        El acierto equilibrado promedia el acierto de CADA clase, así que una
        red que ignora los giros saca 25 % por mucho que acierte el resto."""
        pred = salida.argmax(1)
        llano = float((pred == cierto).float().mean())
        recalls = []
        for c in range(len(pol.ACCIONES)):
            hay = cierto == c
            if int(hay.sum()):
                recalls.append(float((pred[hay] == c).float().mean()))
        return llano, (sum(recalls) / len(recalls) if recalls else 0.0)

    mejor, mejor_llano, mejores_pesos = 0.0, 0.0, None
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
            llano, equilibrado = notas(red(Xv), yv)
        if equilibrado > mejor:
            mejor, mejor_llano = equilibrado, llano
            mejores_pesos = {k: v.clone() for k, v in red.state_dict().items()}
        if (e + 1) % 15 == 0:
            print(f"  época {e+1:3d}: acierto {llano*100:.1f}% | equilibrado {equilibrado*100:.1f}%")

    red.load_state_dict(mejores_pesos)
    # Última barrera: una red que responde casi siempre lo mismo no sirve, y
    # es mejor enterarse aquí que en el campo.
    with torch.no_grad():
        reparto = np.bincount(red(Xv).argmax(1).numpy(), minlength=len(pol.ACCIONES))
    dominante = reparto.max() / max(reparto.sum(), 1)
    if dominante > 0.92:
        print(f"  ¡CUIDADO! el modelo responde '{pol.ACCIONES[int(reparto.argmax())]}' "
              f"en el {dominante*100:.0f} % de los casos: ha colapsado a una sola acción. "
              f"Hacen falta más ejemplos de giro (--recolectar con más escenarios).")
    MODELO.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"pesos": red.state_dict(), "medias": medias.tolist(),
                "desviaciones": desv.tolist(), "entradas": list(pol.ENTRADAS),
                "acciones": list(pol.ACCIONES), "oculta": oculta,
                "version_entradas": pol.VERSION_ENTRADAS,
                "entrenado": time.strftime("%Y-%m-%d %H:%M"), "ejemplos": len(X),
                "acierto": round(mejor_llano, 4), "acierto_equilibrado": round(mejor, 4),
                "reparto_predicho": reparto.tolist(), "escenarios": escenarios}, MODELO)
    print(f"-> {MODELO} | acierto {mejor_llano*100:.1f}% | equilibrado {mejor*100:.1f}% "
          f"con {len(X)} ejemplos")
    return {"ejemplos": len(X), "acierto": mejor_llano, "acierto_equilibrado": mejor}


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


def toda_la_noche(horas, cfg, segundos, mundo_nombre, mezcla, por_tanda=8, epocas=80,
                  mundos=None):
    """Entrena sin parar durante `horas`, guardando por el camino.

    Pensado para dejarlo corriendo: cada tanda recolecta, reentrena con TODO
    lo acumulado y evalúa. Si algo falla, lo anota y sigue. Se puede cortar
    con Ctrl-C sin perder nada: los datos se escriben episodio a episodio y
    el modelo se guarda en cada tanda."""
    fin = time.time() + horas * 3600
    registro = CARPETA / "entrenamiento.log"
    CARPETA.mkdir(parents=True, exist_ok=True)
    tanda, historial = 0, []

    def anotar(texto):
        linea = f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {texto}"
        print(linea, flush=True)
        with registro.open("a") as f:
            f.write(linea + "\n")

    mundos = list(mundos or ([mundo_nombre] if mundo_nombre else []))
    anotar(f"=== entrenamiento de {horas} h | mapas: {', '.join(mundos) or 'solo aleatorios'} "
           f"| mezcla {mezcla:.0%} | {por_tanda} episodios por tanda | aleatorización de dominio ===")
    try:
        while time.time() < fin:
            tanda += 1
            queda = (fin - time.time()) / 3600
            anotar(f"--- tanda {tanda} (quedan {queda:.1f} h) ---")
            try:
                recolectar(por_tanda, cfg, segundos, semilla0=tanda * 1000,
                           mezcla=mezcla, mundos=mundos)
                r = entrenar(epocas)
                anotar(f"tanda {tanda}: {r['ejemplos']} ejemplos acumulados, "
                       f"acierto {r['acierto']*100:.1f}%")
                historial.append({"tanda": tanda, **r})
                # Copia de seguridad del modelo de cada tanda: si una tanda
                # posterior empeora, se puede volver a esta.
                copia = CARPETA / f"politica_tanda{tanda:03d}.pt"
                copia.write_bytes(MODELO.read_bytes())
                # Ficha en el historial: con varias noches de tandas, el .pt
                # más nuevo NO es el mejor, y sin registro no hay manera de
                # saber cuál elegir. Ver simulador/historial.py.
                try:
                    import historial as hist
                    hist.registrar_modelo(copia, tanda=tanda, mundo=",".join(mundos) or None,
                                          mezcla=mezcla, segundos=segundos, **r)
                except Exception as exc:
                    anotar(f"no se pudo fichar el modelo: {type(exc).__name__}: {exc}")
                if tanda % 3 == 0:
                    ev = evaluar(2, cfg, segundos, semilla0=90000 + tanda)
                    anotar(f"evaluación: desvío {ev['sin']['desvio_cm']} -> "
                           f"{ev['con']['desvio_cm']} cm | pisadas "
                           f"{ev['sin']['pisadas']} -> {ev['con']['pisadas']}")
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                anotar(f"tanda {tanda} falló: {type(exc).__name__}: {exc}")
                time.sleep(5)
    except KeyboardInterrupt:
        anotar("cortado a mano")
    anotar(f"=== fin: {tanda} tandas | registro en {registro} ===")
    (CARPETA / "historial_entrenamiento.json").write_text(
        json.dumps(historial, indent=1, ensure_ascii=False))
    return historial


def main():
    p = argparse.ArgumentParser(description="Entrenador del AgroBot")
    p.add_argument("--recolectar", type=int, metavar="N", help="episodios de recolección")
    p.add_argument("--entrenar", action="store_true")
    p.add_argument("--evaluar", type=int, nargs="?", const=4, metavar="N")
    p.add_argument("--todo", action="store_true", help="recolectar + entrenar + evaluar")
    p.add_argument("--horas", type=float, metavar="H",
                   help="entrenar sin parar durante H horas (para dejarlo toda la noche)")
    p.add_argument("--mundo", metavar="NOMBRE",
                   help="mapa dibujado en el dashboard (simulador/mundos/NOMBRE.json)")
    p.add_argument("--mundos", metavar="A,B,C",
                   help="varios mapas: escenarios incluidos y/o dibujados, separados por "
                        "comas; 'todos' = todos los que haya")
    p.add_argument("--todos-los-datos", action="store_true",
                   help="entrenar también con lo recolectado antes del simulador realista")
    p.add_argument("--mezcla", type=float, default=0.7,
                   help="fracción de episodios en ese mapa; el resto, aleatorios")
    p.add_argument("--por-tanda", type=int, default=8, help="episodios por tanda con --horas")
    p.add_argument("--episodios", type=int, default=10, help="episodios con --todo")
    p.add_argument("--segundos", type=float, default=120)
    p.add_argument("--epocas", type=int, default=60)
    p.add_argument("--config", default=str(AQUI.parent / "jetson" / "backend" / "agrobot_config.json"))
    a = p.parse_args()

    cfg = json.loads(Path(a.config).read_text())
    cfg["autonomia"].update(sendero_activo=True, sendero_paja=True)
    cfg["autonomia"].setdefault("escala_velocidad", 0.5)

    global ORIGENES
    if a.todos_los_datos:
        ORIGENES = None
    disponibles = mundos_disponibles()
    mundos = [a.mundo] if a.mundo else []
    if a.mundos:
        mundos = disponibles if a.mundos == "todos" else [n.strip() for n in a.mundos.split(",") if n.strip()]
    faltan = [n for n in mundos if n not in disponibles]
    if faltan:
        sys.exit(f"no existen: {', '.join(faltan)}\nhay: {', '.join(disponibles)}\n"
                 f"(los mapas se dibujan y guardan en el dashboard, pestaña Editor de mapas)")

    if a.horas:
        toda_la_noche(a.horas, cfg, a.segundos, None, a.mezcla, a.por_tanda, a.epocas,
                      mundos=mundos)
        return
    if a.todo:
        print("== 1. recolectando ==")
        recolectar(a.episodios, cfg, a.segundos, mezcla=a.mezcla, mundos=mundos)
        print("== 2. entrenando ==")
        entrenar(a.epocas)
        print("== 3. evaluando ==")
        evaluar(max(2, a.episodios // 4), cfg, a.segundos)
        return
    if a.recolectar:
        recolectar(a.recolectar, cfg, a.segundos, mezcla=a.mezcla, mundos=mundos)
    if a.entrenar:
        entrenar(a.epocas)
    if a.evaluar:
        evaluar(a.evaluar, cfg, a.segundos)
    if not (a.recolectar or a.entrenar or a.evaluar):
        p.print_help()


if __name__ == "__main__":
    main()
