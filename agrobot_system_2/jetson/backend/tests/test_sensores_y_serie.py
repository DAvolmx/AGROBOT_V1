"""
Pruebas de:
  1. Normalización de sensores caídos -> 0.0 (Jetson y Raspberry Pi).
  2. Compatibilidad byte a byte del protocolo BLE entre el firmware del
     ESP32-C3 y el dashboard Web Bluetooth (se leen los DOS archivos y se
     compara lo que declara cada uno, en vez de confiar en que coinciden).

Correr desde jetson/backend/:   python3 tests/test_sensores_y_ble.py
"""
import importlib.util
import math
import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]          # agrobot_system_2/


def cargar(nombre: str, ruta: Path):
    """Carga un módulo por RUTA explícita, sin depender de sys.path.

    Los módulos ya tienen nombres únicos (`jetson_server.py` /
    `gateway_server.py`), así que un import normal alcanzaría. Se carga por
    ruta igual porque este test toca archivos de DOS máquinas distintas y
    así queda escrito sin ambigüedad cuál se está probando."""
    # el módulo importa vecinos suyos (jetson_server.py hace `from protocol
    # import ...`), así que su carpeta tiene que estar en sys.path al cargar
    carpeta = str(ruta.parent)
    if carpeta not in sys.path:
        sys.path.insert(0, carpeta)
    spec = importlib.util.spec_from_file_location(nombre, ruta)
    modulo = importlib.util.module_from_spec(spec)
    sys.modules[nombre] = modulo
    spec.loader.exec_module(modulo)
    return modulo


fails = []


def check(label, got, want):
    ok = got == want
    print(f"  {'OK  ' if ok else 'FALLA'} {label}: {got!r}" + ("" if ok else f"  (esperado {want!r})"))
    if not ok:
        fails.append(label)


# =====================================================================
print("\n[1] Jetson — sensores que no inicializaron -> 0.0")
# =====================================================================
server = cargar("jetson_server", ROOT / "jetson" / "backend" / "jetson_server.py")

# trama con: un campo ausente (co2), uno nulo (tempSCD), uno NaN (gyroZ),
# uno como texto no numérico (humidity) y el resto sano
crudo = {
    "accelX": 0.1, "accelY": 0.2, "accelZ": 0.98,
    "gyroX": 1.5, "gyroY": -0.5, "gyroZ": float("nan"),
    "tempSCD": None, "humidity": "n/a", "gyroYextra": 0.0,
}
out = server.normalize_env(dict(crudo))

check("co2 ausente -> 0.0", out["co2"], 0.0)
check("tempSCD nulo -> 0.0", out["tempSCD"], 0.0)
check("gyroZ NaN -> 0.0", out["gyroZ"], 0.0)
check("humidity no numérico -> 0.0", out["humidity"], 0.0)
check("los sanos NO se tocan (gyroX)", out["gyroX"], 1.5)
check("los sanos NO se tocan (accelZ)", out["accelZ"], 0.98)
check("se listan los caídos", sorted(out["_sensors_missing"]),
      ["co2", "gyroZ", "humidity", "tempSCD"])
check("todos los campos quedan finitos",
      all(math.isfinite(float(out[f])) for f in server.ENV_FIELDS), True)

# una trama sana no debe reportar nada caído
sana = {f: 1.0 for f in server.ENV_FIELDS}
check("trama sana -> lista vacía", server.normalize_env(sana)["_sensors_missing"], [])


# =====================================================================
print("\n[2] Raspberry Pi — mismo criterio sobre el payload LoRaWAN")
# =====================================================================
schema = cargar("rpi_schema", ROOT / "rpi" / "server" / "schema.py")

decodificado = {
    "accelX": 0.1, "accelY": 0.2, "accelZ": 0.9,
    "gyroX": 1.0, "gyroY": 2.0, "gyroZ": float("inf"),   # sensor mudo
    "co2": 900.0, "tempSCD": 9999.0, "humidity": 40.0,   # conectado pero delirando
}
v = schema.validate(decodificado)
check("gyroZ infinito -> 0.0", v["gyroZ"], 0.0)
check("gyroZ listado como ausente", v["_sensors_missing"], ["gyroZ"])
check("tempSCD fuera de rango se marca aparte", v["_out_of_range"], ["tempSCD"])
check("un campo no aparece en las dos listas a la vez",
      set(v["_sensors_missing"]) & set(v["_out_of_range"]), set())


# =====================================================================
print("\n[3] Protocolo serie - firmware vs. enlace de la Jetson")
# =====================================================================
# Este bloque comparo antes el firmware contra el dashboard Web Bluetooth, y
# luego contra el enlace BLE. Los dos transportes se retiraron el 2026-09-04
# (ver la cabecera del .ino). El otro lado del protocolo vive ahora en
# jetson/backend/uart_link.py.
ino = (ROOT / "firmware" / "esp32c3_actuator" / "esp32c3_actuator.ino").read_text()
py = (ROOT / "jetson" / "backend" / "uart_link.py").read_text()

TIPOS = {"uint8_t": "B", "int8_t": "b", "uint16_t": "H"}


def formato_de(nombre):
    """Deriva el formato de struct de Python de la declaracion en C."""
    cuerpo = re.search(rf"struct {nombre} \{{(.*?)\}};", ino, re.S).group(1)
    campos = re.findall(r"^\s*(uint8_t|int8_t|uint16_t)\s+(\w+);", cuerpo, re.M)
    return "<" + "".join(TIPOS[t] for t, _ in campos)


fmt = formato_de("CmdFrame")
check("CmdFrame son 8 bytes (7 de datos + CRC)", struct.calcsize(fmt), 8)

# Comparar los FORMATOS y no solo los tamanos es lo que detecta dos campos
# intercambiados: la trama mide igual y el robot obedece el valor equivocado.
fmt_py = re.search(r'CMD_FORMAT\s*=\s*"([^"]+)"', py).group(1)
check("CMD_FORMAT de uart_link.py coincide con struct CmdFrame", fmt_py, fmt)

fmt_st = formato_de("StatusFrame")
fmt_st_py = re.search(r'STATUS_FORMAT\s*=\s*"([^"]+)"', py).group(1)
check("STATUS_FORMAT de uart_link.py coincide con struct StatusFrame",
      fmt_st_py, fmt_st)

# magics y banderas iguales en ambos lados
for macro, pyname in (("CMD_MAGIC", "CMD_MAGIC"), ("STATUS_MAGIC", "STATUS_MAGIC")):
    fw = int(re.search(rf"const uint8_t {macro}\s*=\s*(0x[0-9A-Fa-f]+)", ino).group(1), 16)
    jet = int(re.search(rf"^{pyname}\s*=\s*(0x[0-9A-Fa-f]+)", py, re.M).group(1), 16)
    check(f"magic {macro}", jet, fw)

for macro in ("FLAG_LED", "FLAG_SCAN", "ST_LED", "ST_SCANNING", "ST_FAILSAFE"):
    fw = int(re.search(rf"#define {macro}\s+(0x[0-9A-Fa-f]+)", ino).group(1), 16)
    jet = int(re.search(rf"^{macro}\s*=\s*(0x[0-9A-Fa-f]+)", py, re.M).group(1), 16)
    check(f"bandera {macro}", jet, fw)

# Los pines del cableado, tomados del firmware, contra los que documenta el
# modulo de la Jetson. Un desfase aqui es una tarde entera con el multimetro.
for macro in ("MANDO_RX_PIN", "MANDO_TX_PIN"):
    pin = int(re.search(rf"#define {macro}\s+(\d+)", ino).group(1))
    check(f"{macro} documentado en uart_link.py", f"GPIO{pin}" in py, True)

baud_fw = int(re.search(r"MANDO_BAUD\s*=\s*(\d+)", ino).group(1))
baud_py = int(re.search(r"^DEFAULT_BAUD\s*=\s*(\d+)", py, re.M).group(1))
check("los baudios coinciden en los dos lados", baud_py, baud_fw)

# El C3 ya NO debe traer radio: es la causa por la que los dos transportes
# anteriores fallaron. Si alguien la reintroduce, esto avisa.
check("el firmware del C3 no incluye WiFi", "WiFi.h" in ino, False)
check("el firmware del C3 no incluye BLE", "BLEDevice.h" in ino, False)

# El CRC de las dos implementaciones tiene que dar lo mismo. Se usa el vector
# de prueba estandar del CRC-8/ATM.
sys.path.insert(0, str(ROOT / "jetson" / "backend"))
import uart_link
check("crc8 con el vector estandar '123456789'", uart_link.crc8(b"123456789"), 0xF4)
check("uart_link empaqueta el tamano que espera el firmware",
      uart_link.CMD_SIZE, struct.calcsize(fmt))

# Round-trip por el MISMO empaquetador que usa el backend en vivo, no por una
# copia del formato en el test: si uart_link.py se desincroniza, esto lo ve.
enlace = uart_link.UartActuatorLink(v_max=0.55, omega_max=1.5)
enlace.send_command(-0.55, 1.5)
enlace.set_actuators(servo_angle=105, led=True)
trama = enlace.pack()
check("la trama mide lo que dice el struct", len(trama), struct.calcsize(fmt))
check("el CRC de la trama generada es valido",
      uart_link.crc8(trama[:-1]), trama[-1])
magic, v_pct, om_pct, servo, flags, seq, crc = struct.unpack(fmt, trama)
check("magic de la trama", magic, 0xA5)
check("v a fondo atras -> -100", v_pct, -100)
check("omega a fondo -> 100", om_pct, 100)
check("servo dentro del recorrido pasa tal cual", servo, 105)

# El recorte al recorrido util (25-110) se prueba aparte: un angulo fuera de
# la ventana no debe salir nunca por el cable, porque el servo montado topa
# mecanicamente y zumbaria contra el tope.
enlace.set_actuators(servo_angle=180)
check("servo por encima del maximo se recorta a 110",
      struct.unpack(fmt, enlace.pack())[3], 110)
enlace.set_actuators(servo_angle=0)
check("servo por debajo del minimo se recorta a 25",
      struct.unpack(fmt, enlace.pack())[3], 25)
enlace.set_actuators(servo_angle=105, led=True)
check("bandera LED encendida", flags & uart_link.FLAG_LED, uart_link.FLAG_LED)

# Un magic que aparezca DENTRO de los datos no debe poder hacerse pasar por
# el principio de una trama. Es justo lo que el CRC esta ahi para evitar.
falsa = bytes([0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5])
check("una trama de puro magic no pasa el CRC",
      uart_link.crc8(falsa[:-1]) == falsa[-1], False)


print("\n" + "=" * 60)
if fails:
    print(f"RESULTADO: {len(fails)} FALLA(S) -> {fails}")
    sys.exit(1)
print("RESULTADO: todas las comprobaciones pasaron")
sys.exit(0)
