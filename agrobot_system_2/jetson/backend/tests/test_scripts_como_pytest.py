"""Ejecuta como subproceso los tests que son SCRIPTS (hacen sys.exit al importarse).

Motivo (2026-10-15): `test_sensores_y_serie.py` termina con `sys.exit()`, asi que
bajo pytest solo aparecia "SystemExit: 0" y TODOS sus checks quedaban ocultos.
Cuando Codex amplio el servo a 130, el check "se recorta a 110" empezo a fallar
y ninguna pasada de pytest lo mostro durante dias. Aqui se ejecuta de verdad y se
exige exito.

NO incluido a proposito: `test_actuator_e2e.py` valida el protocolo ANTIGUO (v4,
14 bytes con floats, servo inicial 0) levantando el backend real; el protocolo
real hoy es la trama compacta de 8 bytes, cubierta por test_firmware_compat.py.
Esta OBSOLETO y hay que reescribirlo, no arreglarlo.
"""
import subprocess
import sys
from pathlib import Path

AQUI = Path(__file__).resolve().parent


def _script(nombre):
    r = subprocess.run([sys.executable, str(AQUI / nombre)], capture_output=True,
                       text=True, timeout=120, cwd=str(AQUI.parent))
    return r.returncode, r.stdout + r.stderr


def test_sensores_y_serie_pasa_entero():
    codigo, salida = _script("test_sensores_y_serie.py")
    assert "FALLA" not in salida, salida[-1500:]
    assert "todas las comprobaciones pasaron" in salida, salida[-1500:]
    assert codigo == 0
