"""Lanza y detiene el simulador visual (simulador_pasivo/servidor.py) desde el dashboard.

El simulador corre en su propio proceso y puerto (`PUERTO`): el dashboard lo muestra en un iframe de la pestaña
«Simulación». Sobrevive a un reinicio del backend (sesión propia); el PID queda en ~/.agrobot/simulador.pid.
Consume CPU/GPU: con el robot en marcha conviene pararlo al terminar.
"""
from __future__ import annotations

import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

PUERTO = 8093
RAIZ = Path(__file__).resolve().parents[2]
SERVIDOR = RAIZ / "simulador_pasivo" / "servidor.py"
PID = Path.home() / ".agrobot" / "simulador.pid"
LOG = Path.home() / ".agrobot" / "logs" / "simulador.log"


def _pid_vivo() -> int | None:
    try:
        pid = int(PID.read_text())
        cmd = Path(f"/proc/{pid}/cmdline").read_bytes()
    except (OSError, ValueError):
        return None
    return pid if b"servidor.py" in cmd else None


def _puerto_abierto() -> bool:
    with socket.socket() as s:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", PUERTO)) == 0


def estado() -> dict:
    pid = _pid_vivo()
    return {"corriendo": pid is not None, "escuchando": pid is not None and _puerto_abierto(), "pid": pid, "puerto": PUERTO}


def iniciar(escenario: str | None = None) -> dict:
    if _pid_vivo() is not None:
        return {"ok": True, **estado(), "nota": "ya estaba corriendo"}
    if not SERVIDOR.exists():
        return {"ok": False, "error": f"no existe {SERVIDOR}"}
    LOG.parent.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, str(SERVIDOR), "--puerto", str(PUERTO)]
    if escenario:
        cmd += ["--escenario", str(escenario)]
    with LOG.open("ab") as log:
        p = subprocess.Popen(cmd, cwd=SERVIDOR.parent, stdout=log, stderr=log, start_new_session=True)
    PID.write_text(str(p.pid))
    return {"ok": True, **estado()}


def detener() -> dict:
    pid = _pid_vivo()
    if pid is None:
        return {"ok": True, **estado(), "nota": "no estaba corriendo"}
    try:
        os.killpg(os.getpgid(pid), signal.SIGTERM)
    except OSError:
        pass
    for _ in range(30):
        if not Path(f"/proc/{pid}").exists():
            break
        time.sleep(0.1)
    else:
        try:
            os.killpg(os.getpgid(pid), signal.SIGKILL)
        except OSError:
            pass
    PID.unlink(missing_ok=True)
    return {"ok": True, **estado()}
