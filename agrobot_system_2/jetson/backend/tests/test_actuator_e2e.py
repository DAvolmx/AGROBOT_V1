"""
Prueba end-to-end real del control de actuadores de AgroBot.

Levanta el backend de verdad (jetson_server.py, aiohttp + UDP), se conecta
por WebSocket como lo haría el dashboard, manda comandos de servo/LED, y
CAPTURA LOS BYTES UDP REALES que salen hacia el ESP32-C3 para verificar
que el protocolo binario es correcto byte a byte.

No mockea el backend: usa el código de producción tal cual, solo apunta
el AP a loopback (la Jetson no tiene el AP levantado ahora mismo) y el
actuador a un socket UDP de prueba que hace de ESP32-C3.
"""
import asyncio
import socket
import struct
import sys
from pathlib import Path

BACKEND = Path("/home/jetson/Desktop/AGROBOT/agrobot_system_2/jetson/backend")
sys.path.insert(0, str(BACKEND))

# `jetson_server` es un nombre único en todo el proyecto (antes esto era
# `import server` y podía cargar el backend de la Raspberry Pi por error).
import jetson_server as server
import protocol

FAKE_ACTUATOR_PORT = 15002
fails = []


async def _fake_json(valor):
    """Simula el body JSON de un POST para probar la validación de entrada."""
    return {"name": valor}


def check(label, got, want):
    ok = got == want
    print(f"  {'OK  ' if ok else 'FALLA'} {label}: {got!r}" + ("" if ok else f"  (esperado {want!r})"))
    if not ok:
        fails.append(label)


async def main():
    # --- tamaños del protocolo vs. los structs #pragma pack(1) del .ino ---
    print("\n[1] Tamaño del protocolo binario (debe coincidir con el firmware)")
    # v4: se quito vy de ambos (el robot es diferencial, no mecanum)
    # CmdPacket:    u8 + f32*2 + u8*3 + u16
    check("CMD_SIZE", protocol.CMD_SIZE, 1 + 4 + 4 + 1 + 1 + 1 + 2)
    # StatusPacket: u8 + f32*2 + u8*4 + u16
    check("STATUS_SIZE", protocol.STATUS_SIZE, 1 + 4 + 4 + 1 + 1 + 1 + 1 + 2)

    # --- el backend real, con el AP apuntado a loopback ---
    server.JETSON_AP_IP = "127.0.0.1"
    server.LORA_UDP_PORT = 15001
    server.ACTUATOR_UDP_PORT = FAKE_ACTUATOR_PORT
    server.state.config["hardware"]["actuator_ip"] = "127.0.0.1"

    # socket que hace de ESP32-C3 escuchando comandos. En producción la
    # Jetson y el ESP32 son máquinas distintas y ambas usan el 5002; acá
    # está todo en loopback, así que el destino se separa del puerto de
    # escucha después de que el backend ya hizo su bind.
    esp32 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp32.bind(("127.0.0.1", FAKE_ACTUATOR_PORT + 1))
    esp32.settimeout(2.0)

    app = server.create_app()
    runner = server.web.AppRunner(app)
    await runner.setup()
    server.ACTUATOR_UDP_PORT = FAKE_ACTUATOR_PORT + 1
    site = server.web.TCPSite(runner, "127.0.0.1", 18080)
    await site.start()

    import aiohttp
    async with aiohttp.ClientSession() as sess:
        async with sess.ws_connect("http://127.0.0.1:18080/ws") as ws:
            snap = await ws.receive_json()
            print("\n[2] Snapshot inicial trae el setpoint de actuadores")
            check("actuator_setpoint presente", "actuator_setpoint" in snap["data"], True)
            check("servo inicial", snap["data"]["actuator_setpoint"]["servo_angle"], 0)
            check("led inicial", snap["data"]["actuator_setpoint"]["led"], False)

            loop = asyncio.get_running_loop()

            async def recv_cmd():
                # en executor: un recvfrom bloqueante congelaría el event
                # loop del backend y el paquete nunca llegaría a salir
                data, _ = await loop.run_in_executor(None, esp32.recvfrom, 64)
                (magic, v, omega, scan, servo,
                 led, seq) = struct.unpack(protocol.CMD_FORMAT, data)
                return {"len": len(data), "magic": magic, "v": v,
                        "omega": omega, "scan": scan, "servo": servo,
                        "led": led, "seq": seq}

            print("\n[3] Slider del servo -> paquete UDP real al ESP32-C3")
            await ws.send_json({"type": "actuator", "servo_angle": 120})
            pkt = await recv_cmd()
            check("tamaño del datagrama", pkt["len"], protocol.CMD_SIZE)
            check("magic", hex(pkt["magic"]), hex(protocol.CMD_MAGIC))
            check("servo_angle en el paquete", pkt["servo"], 120)
            check("led sigue apagado", pkt["led"], 0)

            print("\n[4] Toggle del LED -> no pisa el ángulo del servo (estado pegajoso)")
            await ws.send_json({"type": "actuator", "led": True})
            pkt = await recv_cmd()
            check("led encendido", pkt["led"], 1)
            check("servo se mantiene en 120", pkt["servo"], 120)

            print("\n[5] Mover el robot NO reposiciona la cámara ni apaga la luz")
            await ws.send_json({"type": "control", "v": 0.5, "omega": 0.0, "scan": False})
            pkt = await recv_cmd()
            check("servo intacto durante marcha", pkt["servo"], 120)
            check("led intacto durante marcha", pkt["led"], 1)
            # la rampa no puede entregar más de max_accel * RAMP_MAX_DT en un
            # solo paso, sin importar cuánto tiempo llevara el stream parado
            techo = (server.state.config["safety"]["max_linear_accel_mps2"]
                     * server.RAMP_MAX_DT)
            check(f"v del primer paso <= {techo:.3f} m/s (tope de rampa)",
                  pkt["v"] <= techo + 1e-6, True)
            check("y no llega al objetivo de una", pkt["v"] < 0.5, True)

            print("\n[5b] El protocolo v4 NO transporta velocidad lateral")
            # El robot es diferencial: pedir desplazamiento lateral no es
            # algo que deba "ignorarse en silencio", directamente no existe
            # como campo. Esto lo deja fijado: si alguien re-agrega vy al
            # protocolo sin adaptar el firmware, este check falla.
            campos = struct.unpack(protocol.CMD_FORMAT,
                                   protocol.pack_cmd(0.5, 0.1, 0, 0, False, 8))
            check("el paquete tiene 7 campos (magic,v,omega,scan,servo,led,seq)",
                  len(campos), 7)
            check("v viaja en el paquete", round(campos[1], 2), 0.5)
            check("omega viaja en el paquete", round(campos[2], 2), 0.1)

            print("\n[5c] Un cliente que manda `vy` no mueve el robot de costado")
            # Un dashboard viejo (o un cliente hecho a mano) puede seguir
            # mandando vy. El backend lo tiene que ignorar sin romperse ni
            # contaminar las otras componentes.
            for _ in range(4):
                await asyncio.sleep(0.06)
                await ws.send_json({"type": "control", "v": 0.0, "vy": 0.3,
                                    "omega": 0.0, "scale": 1.0})
                pkt = await recv_cmd()
            check("el backend acepta el mensaje sin caerse", pkt["len"],
                  protocol.CMD_SIZE)
            check("y no contamina el avance", abs(pkt["v"]) < 1e-6, True)
            check("ni el giro", abs(pkt["omega"]) < 1e-6, True)

            print("\n[5d] Evasion reactiva: cubre adelante Y atras")
            # obstaculo ATRAS (180 grados) mas cerca que el umbral de frenado
            server.lidar_reactive.ingest_scan([{"angle_deg": 180.0, "distance_m": 0.20}])
            v_atras, obs = server.lidar_reactive.apply_safety_cap(
                -0.3, server.state.config["slam"])
            check("obstaculo detectado al retroceder", obs is not None, True)
            check("frena el retroceso hacia el obstaculo", v_atras, 0.0)
            # el mismo obstaculo NO debe frenar la marcha hacia adelante
            v_ade, obs2 = server.lidar_reactive.apply_safety_cap(
                0.3, server.state.config["slam"])
            check("pero NO frena el avance (el obstaculo esta detras)",
                  round(v_ade, 3), 0.3)
            # y girar en el lugar tampoco se frena: el robot no se acerca
            server.lidar_reactive.ingest_scan([{"angle_deg": 0.0, "distance_m": 0.20}])
            v_giro, _ = server.lidar_reactive.apply_safety_cap(
                0.0, server.state.config["slam"])
            check("girar en el lugar no se frena por un obstaculo enfrente",
                  v_giro, 0.0)
            server.lidar_reactive.ingest_scan([])

            print("\n[5e] Dead-reckoning diferencial")
            pe = server.PoseEstimator(gyro_weight=0.0)
            import time as _t
            pe._last_t = 0.0
            pe.on_command(0.4, 0.0)           # avance recto, sin girar
            pe._last_t = _t.time() - 0.5      # simula medio segundo
            pe.on_gyro(0.0)
            check("avanzar con theta=0 mueve la pose en X", pe.x > 0.15, True)
            check("y nada en Y (no hay deriva lateral)", abs(pe.y) < 1e-9, True)

            print("\n[5f] Acelerador: el mando manda FRACCION, no m/s")
            maxv = server.state.config["control"]["max_v_mps"]

            async def avanzar(**extra):
                """Manda avance a fondo sostenido y devuelve el paquete que
                sale hacia el ESP32-C3, ya con la rampa levantada del todo."""
                pkt = None
                for _ in range(20):
                    await asyncio.sleep(0.06)
                    await ws.send_json({"type": "control", "v": 1.0,
                                        "omega": 0.0, **extra})
                    pkt = await recv_cmd()
                return pkt

            pkt = await avanzar(scale=1.0)
            # Antes se mandaba v=1 crudo: la UI decía 1.0 m/s y el firmware
            # lo recortaba a V_MAX en silencio. Ahora el techo se aplica acá.
            check(f"acelerador al 100% da max_v_mps ({maxv})",
                  abs(pkt["v"] - maxv) < 0.02, True)

            pkt = await avanzar(scale=0.4)
            check("acelerador al 40% da 40% de esa velocidad",
                  abs(pkt["v"] - maxv * 0.4) < 0.02, True)

            # Un cliente WebSocket es entrada no confiable.
            for basura in (None, "rapido", float("nan"), 99):
                await asyncio.sleep(0.06)
                await ws.send_json({"type": "control", "v": basura,
                                    "omega": 0.0, "scale": 1.0})
                pkt = await recv_cmd()
            check("un valor basura nunca acelera por encima del techo",
                  abs(pkt["v"]) <= maxv + 1e-6, True)

            # y se vuelve a dejar el robot quieto para los bloques siguientes
            for _ in range(20):
                await asyncio.sleep(0.06)
                await ws.send_json({"type": "control", "v": 0.0, "omega": 0.0})
                pkt = await recv_cmd()
            check("soltar el mando devuelve la velocidad a cero",
                  abs(pkt["v"]) < 1e-6, True)

            print("\n[6] Offset de calibración del servo se aplica de verdad")
            server.state.config["calibration"]["servo_offset_deg"] = -20
            await ws.send_json({"type": "actuator", "servo_angle": 100})
            pkt = await recv_cmd()
            check("100 - 20 = 80 enviado al hardware", pkt["servo"], 80)

            print("\n[7] Saturación del servo (uint8 0-180, no puede desbordar)")
            server.state.config["calibration"]["servo_offset_deg"] = 60
            await ws.send_json({"type": "actuator", "servo_angle": 175})
            pkt = await recv_cmd()
            check("175 + 60 se recorta a 180", pkt["servo"], 180)
            server.state.config["calibration"]["servo_offset_deg"] = 0

            print("\n[8] Parada de emergencia: inmediata y apaga el LED")
            await ws.send_json({"type": "emergency_stop"})
            pkt = await recv_cmd()
            check("v exactamente 0 (sin rampa)", pkt["v"], 0.0)
            check("omega exactamente 0", pkt["omega"], 0.0)
            check("led apagado por la emergencia", pkt["led"], 0)

            print("\n[9] Status del ESP32-C3 -> parseo del eco real")
            st = struct.pack(protocol.STATUS_FORMAT, protocol.STATUS_MAGIC,
                             0.33, -0.12, 1, 0, 145, 1, 4242)
            parsed = protocol.unpack_status(st)
            check("v_cmd del eco", round(parsed["v_cmd"], 2), 0.33)
            check("omega_cmd del eco", round(parsed["omega_cmd"], 2), -0.12)
            check("servo_angle del eco", parsed["servo_angle"], 145)
            check("led_state del eco", parsed["led_state"], True)
            check("scanning del eco", parsed["scanning"], True)
            check("seq_echo", parsed["seq_echo"], 4242)

            print("\n[10b] El video NO puede bloquear el event loop")
            # Este es el bug de control que motivó camera.py: cap.read() y
            # cv2.imencode() son BLOQUEANTES (~37 ms por frame). Corriendo
            # dentro del handler async paraban el loop entero, y ese mismo
            # loop es el que manda los comandos UDP al C3 a 20 Hz. Con YOLO
            # (>300 ms) el watchdog del firmware disparaba el failsafe y el
            # robot se frenaba solo en marcha.
            import ast, inspect, textwrap, camera as camera_mod
            # Se analiza el AST, no el texto: buscar la cadena "VideoCapture"
            # en el código fuente también encuentra los COMENTARIOS que
            # explican por qué ya no se usa, y el test daba falso positivo.
            arbol = ast.parse(textwrap.dedent(inspect.getsource(server.video_handler)))
            llamadas = {n.func.attr for n in ast.walk(arbol)
                        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
            check("video_handler no abre la cámara", "VideoCapture" in llamadas, False)
            check("video_handler no comprime frames", "imencode" in llamadas, False)
            check("ni siquiera importa cv2",
                  any(isinstance(n, (ast.Import, ast.ImportFrom)) for n in ast.walk(arbol)), False)
            check("la captura corre en un hilo aparte",
                  "threading.Thread" in inspect.getsource(camera_mod.CameraWorker.start), True)

            # Un cliente lento no puede hacer crecer la memoria: la cola es
            # de 1 y se descarta el frame VIEJO, no el nuevo.
            cw = camera_mod.CameraWorker(asyncio.get_running_loop(),
                                         lambda *a: None, lambda: server.state.config,
                                         lambda *a: None)
            cola = cw.subscribe()
            for i in range(5):
                cw._publicar(bytes([i]))
            await asyncio.sleep(0.05)
            check("la cola del cliente nunca crece", cola.qsize(), 1)
            check("y guarda el frame MÁS NUEVO, no el más viejo",
                  (await cola.get()), bytes([4]))
            cw.unsubscribe(cola)
            check("darse de baja libera al cliente", len(cw._suscriptores), 0)

            print("\n[10d] Cargar un modelo llega DE VERDAD al hilo de inferencia")
            # Al mover YOLO a camera.py, `models_load_handler` seguía dejando
            # el modelo solo en app["yolo_model"]: la config se guardaba, el
            # dashboard decía "Modelo cargado" y la detección no corría nunca,
            # sin ningún error. Fallo silencioso, el peor tipo.
            import ast as _ast, inspect as _insp, textwrap as _tw
            src = _tw.dedent(_insp.getsource(server.models_load_handler))
            metodos = {n.func.attr for n in _ast.walk(_ast.parse(src))
                       if isinstance(n, _ast.Call) and isinstance(n.func, _ast.Attribute)}
            check("el handler avisa al worker de cámara", "set_model" in metodos, True)

            # El nombre viene de un POST: no puede escaparse de models/
            for malo in ("../../etc/passwd", "a/b.pt", None, "", ".oculto"):
                req = type("R", (), {"json": lambda self, v=malo: _fake_json(v),
                                     "app": {}})()
                resp = await server.models_load_handler(req)
                if resp.status != 400:
                    check(f"nombre {malo!r} rechazado", resp.status, 400)
            check("nombres de modelo maliciosos rechazados con 400", True, True)

            # Las rutas no pueden depender del directorio de trabajo
            check("MODELS_DIR es absoluta", server.MODELS_DIR.is_absolute(), True)
            check("CAPTURES_DIR es absoluta", server.CAPTURES_DIR.is_absolute(), True)

            print("\n[10c] El scan se diezma y redondea SOLO para el dashboard")
            crudo = [{"angle_deg": -180.00000500895632 + i * 0.6,
                      "distance_m": 0.14875000715255737} for i in range(600)]
            dec = server._decimar(crudo, server.RADAR_MAX_PUNTOS)
            check("se recorta al máximo de puntos", len(dec), server.RADAR_MAX_PUNTOS)
            check("distancia redondeada a mm", dec[0]["distance_m"], 0.149)
            import json as _json
            antes, ahora = len(_json.dumps(crudo)), len(_json.dumps(dec))
            check(f"el mensaje encoge ({antes//1024} KB -> {ahora//1024} KB)",
                  ahora < antes / 3, True)
            # La evasión reactiva NO debe perder resolución: un solo rayo
            # contra una rama fina es la diferencia entre frenar y chocar.
            server.lidar_reactive.ingest_scan(crudo)
            check("pero la evasión conserva el scan completo",
                  len(server.lidar_reactive.last_scan), 600)
            server.lidar_reactive.ingest_scan([])

            print("\n[10] Bias del giroscopio se resta de la telemetría entrante")
            server.state.config["calibration"]["gyro_bias_z_dps"] = 1.5
            lora = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            lora.sendto(b'{"gyroX":0.0,"gyroY":0.0,"gyroZ":4.0,"co2":800}',
                        ("127.0.0.1", 15001))
            await asyncio.sleep(0.3)
            check("gyroZ 4.0 - bias 1.5", round(server.state.env_telemetry["gyroZ"], 3), 2.5)
            lora.close()

    esp32.close()
    await runner.cleanup()

    print("\n" + "=" * 58)
    if fails:
        print(f"RESULTADO: {len(fails)} FALLA(S) -> {fails}")
        return 1
    print("RESULTADO: todas las comprobaciones pasaron")
    return 0


sys.exit(asyncio.run(main()))
