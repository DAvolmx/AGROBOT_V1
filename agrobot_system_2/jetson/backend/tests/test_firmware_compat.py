"""Compatibilidad Jetson <-> firmware demo-trasero (1.2), comprobada con el
codigo C++ REAL del firmware, no con una copia en Python.

Motivo: el 2026-10-15 el servo se amplio a 130 en `protocol.py` pero no en
`calibracion_camara.py`, y esa clase de divergencia solo se descubre con la
placa. Aqui las constantes se LEEN de los fuentes del firmware.
"""
import random
import re
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import protocol
from ble_link import BleActuatorLink, CMD_FORMAT, crc8

RAIZ = Path(__file__).resolve().parents[3]
FW = RAIZ / "firmware" / "agrobot_c3_demo_trasero_1_2"
INO = FW / "agrobot_c3_demo_trasero_1_2.ino"
HDR = FW / "control_model.h"

pytestmark = pytest.mark.skipif(not INO.exists(), reason="firmware 1.2 no esta en este checkout")


def _constante(texto, nombre):
    m = re.search(rf"{nombre}\s*=\s*([0-9.]+)f?", texto)
    assert m, f"no encontre {nombre}"
    return float(m.group(1))


def test_el_servo_de_la_jetson_coincide_con_el_del_firmware():
    ino = INO.read_text()
    assert protocol.SERVO_MIN_DEG == _constante(ino, "SERVO_MIN")
    assert protocol.SERVO_MAX_DEG == _constante(ino, "SERVO_MAX")
    assert protocol.SERVO_HOME_DEG == _constante(ino, "SERVO_HOME")


def test_las_escalas_v_omega_coinciden():
    """La Jetson manda v/v_max en %; el firmware lo reinterpreta con V_SCALE.
    Si v_max != V_SCALE el duty real no es el pedido."""
    hdr = HDR.read_text()
    vs, ws = _constante(hdr, "V_SCALE"), _constante(hdr, "W_SCALE")
    server = (RAIZ / "jetson" / "backend" / "jetson_server.py").read_text()
    m = re.search(r"BleActuatorLink\(\s*v_max=([0-9.]+),\s*omega_max=([0-9.]+)", server)
    assert m, "no encontre la construccion de BleActuatorLink"
    assert float(m.group(1)) == vs
    assert float(m.group(2)) == ws


def test_el_firmware_reporta_la_version_que_el_backend_exige():
    import c3_setup
    assert 'demo-trasero-1.2' in INO.read_text()
    assert c3_setup.CONFIG_FIRMWARE == "demo-trasero-1.2"
    assert c3_setup.CONFIG_FIRMWARE in c3_setup.ACCEPTED


def test_los_limites_del_backend_son_los_del_firmware():
    """CONFIG_BOUNDS (backend) debe ser igual a `defs[]` de /config (firmware)."""
    import c3_setup
    ino = INO.read_text()
    bloque = re.search(r"defs\[\]=\{(.*?)\};", ino, re.S).group(1)
    del_firmware = {k: (int(a), int(b)) for k, a, b in
                    re.findall(r'\{"(\w+)",(-?\d+),(-?\d+)\}', bloque)}
    assert del_firmware == c3_setup.CONFIG_BOUNDS


@pytest.mark.skipif(not shutil.which("g++"), reason="sin g++")
def test_crc8_de_la_jetson_es_igual_al_del_firmware(tmp_path):
    fuente = tmp_path / "crc.cpp"
    fuente.write_text('''#include <cstdio>
#include "control_model.h"
int main(){char l[64];while(fgets(l,64,stdin)){uint8_t b[16];int n=0;unsigned x;char*p=l;int k;
while(sscanf(p,"%2x%n",&x,&k)==1&&n<16){b[n++]=(uint8_t)x;p+=k;}printf("%02x\\n",agro::crc8(b,n));}}''')
    exe = tmp_path / "crc"
    subprocess.run(["g++", "-std=c++17", f"-I{FW}", str(fuente), "-o", str(exe)], check=True)
    random.seed(11)
    datos = [bytes(random.randrange(256) for _ in range(7)) for _ in range(1500)]
    salida = subprocess.run([str(exe)], input="\n".join(d.hex() for d in datos) + "\n",
                            capture_output=True, text=True, check=True).stdout.split()
    assert len(salida) == len(datos)
    for d, o in zip(datos, salida):
        assert crc8(d) == int(o, 16), d.hex()


def _acepta_el_firmware(t):
    """Las mismas reglas que udpTick() del firmware."""
    if len(t) != 8:
        return False
    magic, v, w, servo, flags, seq, crc = struct.unpack(CMD_FORMAT, t)
    return (magic == 0xA5 and crc8(t[:7]) == crc and -100 <= v <= 100 and -100 <= w <= 100
            and 25 <= servo <= 130 and not (flags & ~3))


def test_toda_trama_que_arma_la_jetson_la_acepta_el_firmware():
    enlace = BleActuatorLink(v_max=0.55, omega_max=1.5)
    for v in (-0.55, -0.2, 0.0, 0.195, 0.33, 0.55, 5.0, -5.0):       # incluye fuera de escala
        for om in (-1.5, 0.0, 1.5, 9.0):
            for servo in (0, 24, 25, 76, 98, 130, 131, 255):          # incluye fuera de rango
                enlace._v, enlace._omega = v, om
                enlace.set_actuators(servo_angle=servo, led=bool(servo % 2))
                assert _acepta_el_firmware(enlace.pack()), (v, om, servo)
