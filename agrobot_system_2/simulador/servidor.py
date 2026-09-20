#!/usr/bin/env python3
"""
Dashboard del simulador: ver, manejar y enseñar al robot desde el navegador.

    python3 simulador/servidor.py                 # http://localhost:8090
    python3 simulador/servidor.py --escenario callejon --puerto 8090

Sirve `dashboard.html` y expone:

  WS   /ws                     estado a 10 Hz (pose, sensores, métricas, eventos)
  GET  /camara.jpg             lo que ve la cámara del robot
  GET  /sendero.jpg            lo que detecta el seguidor de senderos
  GET  /vista3d.jpg?yaw&pitch&distancia    vista 3D orbital
  GET  /mundo                  geometría del escenario (para dibujar el 2D)
  POST /control                {"accion": "adelante|atras|izquierda|derecha|parar"}
  POST /modo                   {"modo": "auto"|"manual"}
  POST /velocidad              {"factor": 0.5..5}
  POST /reset                  {"escenario": ..., "semilla": n}
  GET  /muestras.csv           pares (sensores -> acción) del operador

El episodio corre en TIEMPO REAL para poder verlo y manejarlo; `factor`
acelera o frena la simulación.
"""
import argparse
import asyncio
import csv
import io
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from aiohttp import web

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))

import mundo as mundo_mod          # noqa: E402
import vista3d                     # noqa: E402
from correr import Episodio        # noqa: E402

ACCIONES = ("adelante", "atras", "izquierda", "derecha", "parar")
MUNDOS_DIR = AQUI / "mundos"                      # mapas dibujados en el dashboard
MAPAS_ROBOT = AQUI.parent / "maps"                # mapas reales del SLAM
DURACION_LARGA_S = 10 ** 6          # el episodio del dashboard no se acaba solo


class Simulacion:
    """Mantiene el episodio vivo y permite reiniciarlo con otro escenario."""

    def __init__(self, cfg, escenario, semilla=0):
        self.cfg = cfg
        self.escenario = escenario
        self.semilla = semilla
        self.ep: Episodio | None = None
        self.tarea: asyncio.Task | None = None
        self.enviados = 0            # eventos ya mandados al navegador

    async def arrancar(self, escenario=None, semilla=None):
        await self.parar()
        if escenario is not None:
            self.escenario = escenario
        self.semilla = self.semilla if semilla is None else semilla
        self.ep = Episodio(self.escenario, self.cfg, DURACION_LARGA_S,
                           self.semilla, tiempo_real=True)
        self.enviados = 0
        self.tarea = asyncio.create_task(self.ep._correr())
        for _ in range(100):         # esperar a que monte los módulos reales
            await asyncio.sleep(0.05)
            if getattr(self.ep, "explorador", None) is not None:
                break
        return self.ep

    async def parar(self):
        if self.ep is not None:
            self.ep.parar_pedido = True
        if self.tarea is not None:
            try:
                await asyncio.wait_for(self.tarea, timeout=3)
            except (asyncio.TimeoutError, asyncio.CancelledError, Exception):
                self.tarea.cancel()
        self.tarea = None

    def estado(self) -> dict:
        ep = self.ep
        if ep is None or getattr(ep, "explorador", None) is None:
            return {"listo": False}
        r = ep.robot
        nuevos = ep.eventos[self.enviados:]
        self.enviados = len(ep.eventos)
        return {
            "listo": True, "escenario": ep.mundo.nombre, "modo": ep.modo,
            "t": round(ep.reloj.t, 1), "factor": ep.factor,
            "pose": {"x": round(r.x, 3), "y": round(r.y, 3),
                     "theta_deg": round(r.pose()["theta_deg"], 1)},
            "velocidad": {"v": round(r.v, 3), "omega": round(r.omega, 3),
                          "v_pedida": round(r.v_obj, 3), "omega_pedida": round(r.omega_obj, 3)},
            "sensores": ep.sensores(),
            "sendero": ep.seguidor.snapshot(),
            "explorador": ep.explorador.snapshot(),
            "metricas": {
                "recorrido_m": round(r.recorrido, 2), "colisiones": r.colisiones,
                "contacto_s": round(r.contacto_s, 1),
                "desvio_cm": round(r.desvio_del_sendero() * 100, 1),
                "en_sendero_pct": round(100 * ep.t_en_sendero / max(ep.t_total, 1e-6), 1),
                "escaneos": ep.escaneos, "muestras": len(ep.muestras),
                "bucles": ep.bucles, "intervenciones": ep.intervenciones,
                "aprendidas_usadas": ep.explorador.aprendidas,
            },
            "aprendizaje": ep.aprendizaje.resumen(),
            "slam": {**ep.slam.resumen((ep.robot.x, ep.robot.y, ep.robot.theta)),
                     "usar": ep.usar_slam, "ruido": ep.slam.ruido,
                     "mapa": ep.slam.calidad_mapa(ep.mundo)},
            "ruta": [[round(float(x), 2), round(float(y), 2)] for x, y in ep.ruta[::3]],
            "recorrido_ideal": ep.modo_recorrido == "ideal",
            "filtro_ideal": ep.filtro_ideal,
            "correcciones_filtro": ep.correcciones_filtro,
            "desvio_ruta_cm": round(100 * __import__("ruta_ideal").desvio(
                ep.ruta, (ep.robot.x, ep.robot.y)), 1) if len(ep.ruta) else None,
            "comparacion": ep.comparacion,
            "traza_slam": ep.traza_slam[-400:],
            "traza": ep.traza[-600:],
            "lidar": [{"a": round(p["angle_deg"], 1), "d": round(p["distance_m"], 2)}
                      for p in ep.lidar.last_scan[::6]],
            "eventos": nuevos[-12:],
        }


def _jpeg(img, calidad=80):
    ok, datos = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, calidad])
    if not ok:
        raise web.HTTPInternalServerError(text="no se pudo codificar la imagen")
    return web.Response(body=datos.tobytes(), content_type="image/jpeg",
                        headers={"Cache-Control": "no-store"})


def crear_app(sim: Simulacion) -> web.Application:
    app = web.Application()

    async def index(_):
        return web.FileResponse(AQUI / "dashboard.html")

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        try:
            while not ws.closed:
                await ws.send_json(sim.estado())
                await asyncio.sleep(0.1)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return ws

    async def camara(_):
        return _jpeg(await asyncio.to_thread(sim.ep.robot.frame, sim.ep.geometria(), 480, 360))

    async def sendero_vista(_):
        import sendero as sd

        def render():
            frame = sim.ep.robot.frame(sim.ep.geometria(), 480, 360)
            res = sim.ep.seguidor.vision.detectar(cv2, frame, sim.ep.cfg["autonomia"],
                                                  sim.ep.geometria())
            return sd.dibujar(cv2, frame, res)

        return _jpeg(await asyncio.to_thread(render))

    async def vista_3d(request):
        q = request.query
        ep = sim.ep
        cam = vista3d.Camara(
            objetivo=(ep.robot.x, ep.robot.y) if q.get("seguir", "1") == "1" else (0.0, 0.0),
            yaw=float(q.get("yaw", -60)), pitch=float(q.get("pitch", 32)),
            distancia=float(q.get("distancia", 7)),
            ancho=int(q.get("ancho", 760)), alto=int(q.get("alto", 460)))
        img = await asyncio.to_thread(vista3d.render, ep.mundo,
                                      (ep.robot.x, ep.robot.y, ep.robot.theta), cam, ep.traza)
        return _jpeg(img)

    async def geometria_mundo(_):
        m = sim.ep.mundo
        return web.json_response({
            "nombre": m.nombre, "limites": list(m.limites),
            "paredes": m.paredes.tolist(), "sendero": m.sendero.tolist(),
            "ancho_sendero": m.ancho_sendero, "ancho_paja": m.ancho_paja,
            "plantas": m.plantas.tolist(), "escaneos": m.escaneos.tolist(),
            "seco": m.seco,
            "escenarios": list(mundo_mod.ESCENARIOS),
        })

    async def control(request):
        cuerpo = await request.json()
        accion = cuerpo.get("accion")
        if accion not in ACCIONES:
            return web.json_response({"error": f"acción desconocida: {accion}"}, status=400)
        # Se puede corregir TAMBIÉN en automático: ese es el entrenamiento.
        n = sim.ep.conducir_manual(accion, bool(cuerpo.get("maniobra", True)))
        return web.json_response({"ok": True, "muestras": n, "modo": sim.ep.modo})

    async def modo(request):
        cuerpo = await request.json()
        if cuerpo.get("modo") not in ("auto", "manual"):
            return web.json_response({"error": "modo debe ser auto o manual"}, status=400)
        return web.json_response({"ok": True, "modo": sim.ep.cambiar_modo(cuerpo["modo"])})

    async def velocidad(request):
        cuerpo = await request.json()
        sim.ep.factor = max(0.1, min(5.0, float(cuerpo.get("factor", 1.0))))
        return web.json_response({"ok": True, "factor": sim.ep.factor})

    def _mundo_pedido(cuerpo):
        """De la petición sale: un escenario de siempre, uno dibujado, o un
        mapa real del robot."""
        if cuerpo.get("mundo"):
            m = mundo_mod.Mundo.desde_dict(cuerpo["mundo"])
            return (m, _pose_inicial(m))
        if cuerpo.get("guardado"):
            datos = json.loads((MUNDOS_DIR / f"{_seguro(cuerpo['guardado'])}.json").read_text())
            m = mundo_mod.Mundo.desde_dict(datos)
            return (m, _pose_inicial(m))
        if cuerpo.get("mapa_real"):
            nombre = _seguro(cuerpo["mapa_real"])
            carpeta = MAPAS_ROBOT / nombre
            m = mundo_mod.desde_ocupacion(carpeta / "mapa.pgm",
                                          (carpeta / "mapa.yaml").read_text(), nombre,
                                          sendero=cuerpo.get("sendero", []))
            return (m, _pose_inicial(m))
        return cuerpo.get("escenario", sim.escenario)

    async def reset(request):
        cuerpo = await request.json()
        try:
            esc = _mundo_pedido(cuerpo)
        except FileNotFoundError as exc:
            return web.json_response({"error": str(exc)}, status=404)
        if isinstance(esc, str) and esc not in mundo_mod.ESCENARIOS:
            return web.json_response({"error": f"escenario desconocido: {esc}"}, status=400)
        muestras = list(sim.ep.muestras) if sim.ep else []
        await sim.arrancar(esc, cuerpo.get("semilla"))
        sim.ep.muestras.extend(muestras)     # el aprendizaje no se pierde al cambiar de escenario
        return web.json_response({"ok": True, "escenario": sim.ep.mundo.nombre})

    async def mundos(request):
        """GET: qué hay para cargar. POST: guarda el mapa dibujado."""
        if request.method == "GET":
            MUNDOS_DIR.mkdir(exist_ok=True)
            return web.json_response({
                "guardados": sorted(p.stem for p in MUNDOS_DIR.glob("*.json")),
                "mapas_reales": sorted(p.parent.name for p in MAPAS_ROBOT.glob("*/mapa.pgm")),
                "escenarios": list(mundo_mod.ESCENARIOS),
            })
        cuerpo = await request.json()
        nombre = _seguro(cuerpo.get("nombre", ""))
        if not nombre:
            return web.json_response({"error": "hace falta un nombre"}, status=400)
        MUNDOS_DIR.mkdir(exist_ok=True)
        datos = dict(cuerpo["mundo"], nombre=nombre)
        (MUNDOS_DIR / f"{nombre}.json").write_text(json.dumps(datos, indent=1, ensure_ascii=False))
        return web.json_response({"ok": True, "nombre": nombre})

    async def mundo_de_mapa_real(request):
        """El mapa del SLAM convertido a paredes, para dibujarle el sendero."""
        nombre = _seguro(request.match_info["nombre"])
        carpeta = MAPAS_ROBOT / nombre
        try:
            m = mundo_mod.desde_ocupacion(carpeta / "mapa.pgm",
                                          (carpeta / "mapa.yaml").read_text(), nombre)
        except (FileNotFoundError, OSError) as exc:
            return web.json_response({"error": str(exc)}, status=404)
        return web.json_response(m.a_dict())

    async def recorrido(request):
        """Recorrido ideal, vuelta al inicio y comparación con el autónomo."""
        cuerpo = await request.json()
        ep, accion = sim.ep, cuerpo.get("accion")
        if accion == "ideal":
            return web.json_response(ep.iniciar_recorrido_ideal())
        if accion == "parar":
            return web.json_response(ep.terminar_recorrido())
        if accion == "volver":
            ep.volver_al_inicio(bool(cuerpo.get("reiniciar_slam", True)))
            return web.json_response({"ok": True})
        if accion == "comparar":
            # Vuelve al inicio y lanza el autónomo midiendo el mismo tramo.
            ep.volver_al_inicio()
            ep.iniciar_comparacion()
            ep.cambiar_modo("auto")
            return web.json_response({"ok": True, "ideal": ep.resultado_ideal})
        if accion == "cerrar_comparacion":
            return web.json_response(ep.cerrar_comparacion())
        return web.json_response({"error": f"acción desconocida: {accion}"}, status=400)

    async def filtro(request):
        cuerpo = await request.json()
        sim.ep.filtro_ideal = bool(cuerpo.get("activo", True))
        return web.json_response({"ok": True, "activo": sim.ep.filtro_ideal})

    async def slam_opciones(request):
        cuerpo = await request.json()
        ep = sim.ep
        if "usar_slam" in cuerpo:
            ep.usar_slam = bool(cuerpo["usar_slam"])
        if "ruido" in cuerpo:
            ep.slam.ruido = max(0.0, min(3.0, float(cuerpo["ruido"])))
        if cuerpo.get("corregir"):
            ep.slam.corregir(ep.robot.x, ep.robot.y, ep.robot.theta)
        return web.json_response({"ok": True, "usar_slam": ep.usar_slam,
                                  "ruido": ep.slam.ruido})

    async def mapa_slam(_):
        """El mapa tal como lo está construyendo el SLAM, con las dos poses."""
        ep = sim.ep

        def render():
            img = ep.slam.mapa_imagen()
            img = cv2.resize(img, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
            x0, y0, x1, y1 = ep.mundo.limites
            k = 3 / ep.slam.resolucion
            a_px = lambda x, y: (int((x - x0) * k), int((y1 - y) * k))
            # OpenCV usa BGR: verde real, azul para lo que cree el SLAM.
            for traza, color in ((ep.traza, (60, 190, 60)), (ep.traza_slam, (230, 130, 40))):
                if len(traza) > 1:
                    pts = np.array([a_px(x, y) for x, y in traza], np.int32)
                    cv2.polylines(img, [pts], False, color, 2, cv2.LINE_AA)
            if len(ep.ruta):
                pts = np.array([a_px(x, y) for x, y in ep.ruta], np.int32)
                cv2.polylines(img, [pts], False, (0, 150, 255), 2, cv2.LINE_AA)   # naranja
            cv2.circle(img, a_px(ep.robot.x, ep.robot.y), 7, (60, 190, 60), -1)
            cv2.circle(img, a_px(ep.slam.x, ep.slam.y), 7, (230, 130, 40), 2)
            cv2.putText(img, "verde: donde ESTA   azul: donde CREE estar   naranja: ruta ideal",
                        (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1)
            return img

        return _jpeg(await asyncio.to_thread(render))

    async def aprendizaje(request):
        """GET: qué aprendió. POST: olvidar una lección o dejar de aplicarlas."""
        ap = sim.ep.aprendizaje
        if request.method == "GET":
            return web.json_response({**ap.resumen(), "usar": sim.ep.usar_aprendido})
        cuerpo = await request.json()
        if "usar" in cuerpo:
            sim.ep.usar_aprendido = bool(cuerpo["usar"])
            sim.ep.explorador._aprendizaje = ap if sim.ep.usar_aprendido else None
        if cuerpo.get("olvidar"):
            ap.olvidar(cuerpo["olvidar"])
        return web.json_response({"ok": True, "usar": sim.ep.usar_aprendido, **ap.resumen()})

    async def muestras_csv(_):
        muestras = sim.ep.muestras if sim.ep else []
        if not muestras:
            return web.Response(text="sin muestras todavía: manejá en modo manual\n")
        buf = io.StringIO()
        w = csv.DictWriter(buf, fieldnames=list(muestras[0].keys()))
        w.writeheader()
        w.writerows(muestras)
        return web.Response(body=buf.getvalue().encode(), content_type="text/csv",
                            headers={"Content-Disposition": "attachment; filename=muestras.csv"})

    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/camara.jpg", camara)
    app.router.add_get("/sendero.jpg", sendero_vista)
    app.router.add_get("/vista3d.jpg", vista_3d)
    app.router.add_get("/mundo", geometria_mundo)
    app.router.add_get("/muestras.csv", muestras_csv)
    app.router.add_post("/control", control)
    app.router.add_post("/modo", modo)
    app.router.add_post("/velocidad", velocidad)
    app.router.add_post("/reset", reset)
    app.router.add_post("/recorrido", recorrido)
    app.router.add_post("/filtro", filtro)
    app.router.add_post("/slam", slam_opciones)
    app.router.add_get("/mapa_slam.jpg", mapa_slam)
    app.router.add_get("/aprendizaje", aprendizaje)
    app.router.add_post("/aprendizaje", aprendizaje)
    app.router.add_get("/mundos", mundos)
    app.router.add_post("/mundos", mundos)
    app.router.add_get("/mapa_real/{nombre}", mundo_de_mapa_real)
    return app


def _seguro(nombre: str) -> str:
    """Solo letras, números, guion y guion bajo: el nombre acaba en una ruta."""
    return "".join(c for c in str(nombre) if c.isalnum() or c in "-_")[:40]


def _pose_inicial(m) -> tuple:
    """Arranca al principio del sendero dibujado, mirando hacia el siguiente
    punto. Sin sendero, en el centro del mapa."""
    if len(m.sendero) >= 2:
        (x, y), (x2, y2) = m.sendero[0], m.sendero[1]
        return (float(x), float(y), math.atan2(float(y2 - y), float(x2 - x)))
    x0, y0, x1, y1 = m.limites
    return ((x0 + x1) / 2, (y0 + y1) / 2, 0.0)


def main():
    p = argparse.ArgumentParser(description="Dashboard del simulador AgroBot")
    p.add_argument("--escenario", default="bucle", choices=list(mundo_mod.ESCENARIOS))
    p.add_argument("--puerto", type=int, default=8090)
    p.add_argument("--semilla", type=int, default=0)
    p.add_argument("--config", default=str(AQUI.parent / "jetson" / "backend" / "agrobot_config.json"))
    a = p.parse_args()

    cfg = json.loads(Path(a.config).read_text())
    cfg["autonomia"].setdefault("sendero_paja", True)
    cfg["autonomia"]["sendero_activo"] = True
    sim = Simulacion(cfg, a.escenario, a.semilla)

    async def arrancar(app):
        await sim.arrancar()

    async def cerrar(app):
        await sim.parar()

    app = crear_app(sim)
    app.on_startup.append(arrancar)
    app.on_cleanup.append(cerrar)
    print(f"Dashboard del simulador en http://localhost:{a.puerto}  (escenario: {a.escenario})")
    web.run_app(app, port=a.puerto, print=None)


if __name__ == "__main__":
    main()
