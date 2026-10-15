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
import os
from pathlib import Path as _SandboxPath
os.environ["HOME"] = str(_SandboxPath(__file__).resolve().parent / "laboratorio")
_SandboxPath(os.environ["HOME"]).mkdir(exist_ok=True)
import argparse
import asyncio
import csv
import io
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from aiohttp import web

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))

import historial                   # noqa: E402
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
        # El terreno se pinta una vez (3-5 s). Hecho aquí, en un hilo, y no
        # en el primer fotograma: ahí congelaba la física y el WebSocket.
        await asyncio.to_thread(self.ep.mundo.raster_suelo)
        self.enviados = 0
        self._calidad_cache = (0.0, None)
        self._fps_ref = (time.monotonic(), 0, 0.0)
        self.tarea = asyncio.create_task(self.ep._correr())
        for _ in range(100):         # esperar a que monte los módulos reales
            await asyncio.sleep(0.05)
            if getattr(self.ep, "explorador", None) is not None:
                break
        # El panel es simulación aislada: arrancar en auto evita que parezca
        # bloqueado al abrirlo; el botón Manual sigue disponible en la UI.
        self.ep.cambiar_modo("auto")
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

    def _fps_camara(self) -> float:
        """Fotogramas que analizó el detector por segundo de reloj real."""
        ahora = time.monotonic()
        n = self.ep.frames_camara
        t0, n0, fps = getattr(self, "_fps_ref", (ahora, n, 0.0))
        if ahora - t0 >= 1.0:
            fps = (n - n0) / (ahora - t0)
            t0, n0 = ahora, n
        self._fps_ref = (t0, n0, fps)
        return round(fps, 1)

    def _calidad_mapa(self) -> dict:
        """Compara todas las celdas ocupadas contra todas las paredes: con
        el mapa lleno son decenas de miles de distancias. A 10 Hz por
        cliente era trabajo tirado; basta con recalcularla cada 2 s."""
        ahora = time.monotonic()
        t, valor = getattr(self, "_calidad_cache", (0.0, None))
        if valor is None or ahora - t > 2.0:
            valor = self.ep.slam.calidad_mapa(self.ep.mundo)
            self._calidad_cache = (ahora, valor)
        return valor

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
                     "fallos_casamiento": ep.slam.fallos_casamiento,
                     "cierres": ep.slam.cierres,
                     "mapa": self._calidad_mapa()},
            "lidar_estado": {"caido": ep.slam.sin_lidar, "conexion": ep.fallos_lidar.conexion,
                             "caidas": ep.fallos_lidar.caidas,
                             "segundos_sin": round(ep.fallos_lidar.t_caido, 1),
                             "edad_scan_s": round(ep.lidar.edad_scan_s(), 2)},
            "camara_fps": self._fps_camara(),
            "servo": round(ep.robot.servo),
            "plantas_pisadas": r.plantas_pisadas,
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

    def _ultimo_frame():
        """El fotograma que vio el robot por última vez. Antes se renderizaba
        otro distinto por petición, desde otro hilo y con el mismo generador
        aleatorio que la física: ni era lo que analizaba el detector ni era
        seguro. Solo si el robot aún no miró se renderiza uno."""
        ep = sim.ep
        frame = ep.ultimo_frame
        if frame is None:
            frame = ep.robot.mundo.camara(ep.robot.x, ep.robot.y, ep.robot.theta,
                                          ep.geometria(), 320, 240)
        return frame

    async def camara(_):
        return _jpeg(_ultimo_frame(), calidad=85)

    async def sendero_vista(_):
        import sendero as sd

        def render():
            ep = sim.ep
            frame = _ultimo_frame()
            seguidor = getattr(ep, "seguidor", None)
            if seguidor is None:  # el episodio puede estar reiniciándose
                return frame
            res = seguidor.vision.detectar(cv2, frame, ep.cfg["autonomia"],
                                           ep.geometria())
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

    def _suelo_visor():
        """El terreno pintado (el mismo que proyecta la cámara del robot),
        recortado a los límites del mapa y sin las sombras horneadas: en el
        3D las pone three.js según el sol. Tope de 4096 px, lo que admite
        cualquier GPU."""
        m = sim.ep.mundo
        niveles, (x0r, y1r) = m.raster_suelo()
        tex, res = niveles[0]
        clave = (id(tex), res)
        cache = getattr(sim, "_textura_visor", None)
        if cache is not None and cache[0] == clave:
            return cache[1]
        x0, y0, x1, y1 = m.limites
        c0, c1 = int((x0 - x0r) / res), int((x1 - x0r) / res)
        f0, f1 = int((y1r - y1) / res), int((y1r - y0) / res)
        img = np.ascontiguousarray(tex[f0:f1, c0:c1, :3])
        escala = 4096 / max(img.shape[:2])
        if escala < 1:
            img = cv2.resize(img, None, fx=escala, fy=escala, interpolation=cv2.INTER_AREA)
        sim._textura_visor = (clave, img)
        return img

    async def textura_terreno(request):
        """El suelo del escenario como imagen, para el visor 3D.

        Es la MISMA que ve la cámara del robot: bandas de paja, plantas,
        humedad y piedras. Así el 3D no es una decoración aparte, sino el
        terreno de verdad."""
        img = await asyncio.to_thread(_suelo_visor)
        return _jpeg(img, calidad=90)

    async def relieve_terreno(request):
        """Mapa de relieve (normales) sacado del propio terreno.

        Con esto la luz marca los surcos y la paja en vez de verse el suelo
        como una calcomanía plana, y no hace falta descargar ningún paquete
        de texturas."""

        def render():
            base = _suelo_visor()
            gris = cv2.cvtColor(base, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255
            gris = cv2.GaussianBlur(gris, (0, 0), 1.2)
            gx = cv2.Sobel(gris, cv2.CV_32F, 1, 0, ksize=3)
            gy = cv2.Sobel(gris, cv2.CV_32F, 0, 1, ksize=3)
            fuerza = 3.0
            n = np.dstack([-gx * fuerza, -gy * fuerza, np.ones_like(gris)])
            n /= np.linalg.norm(n, axis=2, keepdims=True)
            # Normales a color, como cualquier mapa de normales: RGB = XYZ.
            return ((n * 0.5 + 0.5) * 255).astype(np.uint8)[:, :, ::-1]

        return _jpeg(await asyncio.to_thread(render), calidad=88)

    async def validar_mundo(request):
        """El editor pregunta si el mapa sirve para entrenar."""
        cuerpo = await request.json()
        try:
            m = mundo_mod.Mundo.desde_dict(cuerpo["mundo"])
        except (KeyError, ValueError, TypeError) as exc:
            return web.json_response({"error": f"mapa mal formado: {exc}"}, status=400)
        if cuerpo.get("quitar_plantas_del_sendero"):
            m.plantas = m._normalizar_plantas(mundo_mod._fuera_del_sendero(
                m.plantas.tolist(), m.sendero.tolist(), m.ancho_sendero))
        return web.json_response({"problemas": mundo_mod.validar(m),
                                  "plantas": m.plantas.tolist()})

    async def geometria_mundo(_):
        m = sim.ep.mundo
        return web.json_response({
            "nombre": m.nombre, "limites": list(m.limites),
            "paredes": m.paredes.tolist(), "sendero": m.sendero.tolist(),
            "ancho_sendero": m.ancho_sendero, "ancho_paja": m.ancho_paja,
            "plantas": m.plantas.tolist(), "escaneos": m.escaneos.tolist(),
            "seco": m.seco, "borde": m.borde,
            "objetos": [{"tipo": t, "x": float(o[0]), "y": float(o[1]), "radio": float(o[2]),
                         "altura": float(o[3]), "solido": bool(o[4])}
                        for o, t in zip(m.objetos, m.tipos)],
            "altura_pared": 0.6, "altura_lidar": mundo_mod.ALTURA_LIDAR_M,
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
        escenario = cuerpo.get("escenario", sim.escenario)
        if isinstance(escenario, str) and escenario not in mundo_mod.ESCENARIOS:
            guardado = MUNDOS_DIR / f"{_seguro(escenario)}.json"
            if guardado.is_file():
                m = mundo_mod.Mundo.desde_dict(json.loads(guardado.read_text()))
                return (m, _pose_inicial(m))
        return escenario

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
        if "fallos_casamiento" in cuerpo:
            ep.slam.fallos_casamiento = bool(cuerpo["fallos_casamiento"])
        if "conexion" in cuerpo:
            ep.fallos_lidar.cambiar(str(cuerpo["conexion"]))
        if "cierres" in cuerpo:
            ep.slam.cierres = bool(cuerpo["cierres"])
        if cuerpo.get("corregir"):
            ep.slam.corregir(ep.robot.x, ep.robot.y, ep.robot.theta)
        return web.json_response({"ok": True, "usar_slam": ep.usar_slam,
                                  "ruido": ep.slam.ruido,
                                  "fallos_casamiento": ep.slam.fallos_casamiento,
                                  "conexion": ep.fallos_lidar.conexion,
                                  "cierres": ep.slam.cierres})

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

    async def hist_handler(request):
        import subprocess, re
        entrenando = False
        try:
            out = subprocess.check_output(["pgrep", "-af", "entrenar.py"], text=True, timeout=2)
            entrenando = bool(out.strip())
        except Exception:
            pass
        # Parsear el log de entrenamiento para gráficos
        log_path = Path.home() / ".agrobot" / "aprendizaje" / "entrenamiento.log"
        tandas = []
        sesion_actual = None
        if log_path.exists():
            for linea in log_path.read_text().splitlines():
                if "=== entrenamiento de" in linea:
                    sesion_actual = linea.split("===")[1].strip() if "===" in linea else ""
                    continue
                m = re.search(r"tanda (\d+): (\d+) ejemplos.*acierto ([\d.]+)%", linea)
                if m:
                    ts = linea[:19]
                    tandas.append({"tanda": int(m.group(1)), "ejemplos": int(m.group(2)),
                                   "acierto": float(m.group(3)), "cuando": ts})
                m2 = re.search(r"evaluación:.*desvío ([\d.]+) -> ([\d.]+).*pisadas ([\d.]+) -> ([\d.]+)", linea)
                if m2:
                    tandas.append({"tipo": "eval", "desvio_sin": float(m2.group(1)),
                                   "desvio_con": float(m2.group(2)),
                                   "pisadas_sin": float(m2.group(3)),
                                   "pisadas_con": float(m2.group(4)),
                                   "cuando": linea[:19]})
        return web.json_response({
            "ranking": historial.ranking(solo_validados=False),
            "eventos": historial.leer(limite=50),
            "entrenando": entrenando,
            "tandas": tandas[-60:],
        })

    async def desplegar_modelo(request):
        return web.json_response({"error": "Este laboratorio no despliega modelos al robot"}, status=403)

    async def vision_training(_):
        path = Path(os.environ.get('AGROBOT_VISION_STATUS',
                    '/home/jetson/.agrobot/mision_20261014/training/status.json'))
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return web.json_response({'status': 'unavailable'})
        fields = ('status', 'started_utc', 'deadline_utc', 'finished_utc', 'planned_seconds',
                  'elapsed_s', 'epochs', 'device', 'baseline_validation', 'best_validation',
                  'candidate_test', 'baseline_test', 'test_delta_path_iou', 'candidate_accepted', 'limits')
        return web.json_response({k: data[k] for k in fields if k in data})

    app.router.add_get('/api/vision-training', vision_training)
    app.router.add_get("/historial", hist_handler)
    app.router.add_post("/desplegar", desplegar_modelo)
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/camara.jpg", camara)
    app.router.add_get("/sendero.jpg", sendero_vista)
    app.router.add_get("/vista3d.jpg", vista_3d)
    app.router.add_get("/mundo", geometria_mundo)
    app.router.add_get("/textura.jpg", textura_terreno)
    app.router.add_get("/relieve.jpg", relieve_terreno)
    app.router.add_static("/static", AQUI / "static", show_index=False)
    app.router.add_get("/muestras.csv", muestras_csv)
    app.router.add_post("/control", control)
    async def controlador_pasivo(request):
        candidate=AQUI/'resultados_pasivo'/'controlador_candidato.json'
        if not candidate.exists():return web.json_response({'error':'Entrenamiento pendiente'},status=409)
        ep=sim.ep
        ep.cambiar_modo('manual')
        ep.conductor.parar()
        ep.piloto=None
        ep.slam.cierres=False
        ep.controlador_pasivo=json.loads(candidate.read_text())
        ep.modo='ajustado'
        return web.json_response({'ok':True,'controlador':ep.controlador_pasivo,'simulacion':True})

    async def resultados_pasivo(request):
        file=AQUI/'resultados_pasivo'/'metrics.json'
        if not file.exists():return web.json_response({'estado':'entrenando'})
        return web.json_response(json.loads(file.read_text()))

    app.router.add_post('/controlador_pasivo',controlador_pasivo)
    app.router.add_get('/resultados_pasivo',resultados_pasivo)
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
    app.router.add_post("/validar", validar_mundo)
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
    guardados = [f.stem for f in MUNDOS_DIR.glob("*.json")]
    p.add_argument("--escenario", default="huerta_fotos_giro20",
                   choices=list(mundo_mod.ESCENARIOS) + guardados)
    p.add_argument("--puerto", type=int, default=8093)
    p.add_argument("--semilla", type=int, default=0)
    p.add_argument("--config", default=str(AQUI / "config_huerta.json"))
    a = p.parse_args()

    cfg = json.loads(Path(a.config).read_text())
    cfg["autonomia"].setdefault("sendero_paja", True)
    cfg["autonomia"]["sendero_activo"] = True
    escenario = a.escenario
    guardado = MUNDOS_DIR / f"{escenario}.json"
    if escenario not in mundo_mod.ESCENARIOS and guardado.exists():
        m = mundo_mod.Mundo.desde_dict(json.loads(guardado.read_text()))
        s = m.sendero
        pose = (float(s[0][0]), float(s[0][1]), math.atan2(float(s[1][1]-s[0][1]), float(s[1][0]-s[0][0]))) if len(s)>1 else (0, 0, 0)
        escenario = (m, pose)
    sim = Simulacion(cfg, escenario, a.semilla)

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
