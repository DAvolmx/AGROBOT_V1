"""
Vigía de una sesión larga (entrenamiento + robot parado).

Cada PERIODO_S registra el estado completo del sistema en
`<carpeta>/estado.jsonl`, guarda fotos de la cámara y escribe en
`<carpeta>/alertas.log` todo lo que se salga de lo normal. Sirve para dejar
el robot horas solo y tener después el informe con datos, no con recuerdos.

    python3 tools/vigia_sesion.py --horas 5 --carpeta logs/sesion_20261001

Solo lee: no mueve el robot ni cambia la configuración. Va por 127.0.0.1,
que el backend deja pasar sin la clave del dashboard.
"""
import argparse
import json
import re
import subprocess
import time
import urllib.request
from pathlib import Path

BACKEND = "http://127.0.0.1:8080"
PERIODO_S = 300
LOG_ENTRENAMIENTO = Path(__file__).resolve().parent.parent / "logs" / "entrenamiento_20261001.log"


def _get(url, timeout=6):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def _json(ruta):
    try:
        return json.loads(_get(BACKEND + ruta))
    except Exception as exc:
        return {"error": f"{type(exc).__name__}: {exc}"}


def _foto_video(destino: Path) -> bool:
    """Primer cuadro del MJPEG /video (la cámara tal cual)."""
    try:
        with urllib.request.urlopen(BACKEND + "/video", timeout=8) as r:
            buf = b""
            while len(buf) < 2_000_000:
                buf += r.read(16384)
                a = buf.find(b"\xff\xd8")
                b = buf.find(b"\xff\xd9", a + 2)
                if a >= 0 and b > a:
                    destino.write_bytes(buf[a:b + 2])
                    return True
    except Exception:
        pass
    return False


def _tegrastats():
    try:
        p = subprocess.run(["timeout", "3", "tegrastats", "--interval", "1000"],
                           capture_output=True, text=True)
        linea = p.stdout.strip().splitlines()[0]
    except Exception:
        return {}
    out = {"crudo": linea}
    m = re.search(r"RAM (\d+)/(\d+)MB", linea)
    if m:
        out["ram_usada_mb"], out["ram_total_mb"] = int(m.group(1)), int(m.group(2))
    m = re.search(r"GR3D_FREQ (\d+)%", linea)
    if m:
        out["gpu_pct"] = int(m.group(1))
    m = re.search(r"tj@([\d.]+)C", linea)
    if m:
        out["tj_c"] = float(m.group(1))
    m = re.search(r"VDD_IN (\d+)mW", linea)
    if m:
        out["potencia_mw"] = int(m.group(1))
    cpus = re.search(r"CPU \[([^\]]*)\]", linea)
    if cpus:
        out["cpu_pct"] = [int(x.split("%")[0]) for x in cpus.group(1).split(",") if "%" in x]
    return out


def _entrenamiento():
    vivo = subprocess.run(["pgrep", "-f", "entrenar.py"], capture_output=True).returncode == 0
    tanda, ultima = None, ""
    try:
        lineas = LOG_ENTRENAMIENTO.read_text(errors="ignore").splitlines()
        for l in reversed(lineas):
            m = re.search(r"--- tanda (\d+)", l)
            if m and tanda is None:
                tanda = int(m.group(1))
            if not ultima and l.strip():
                ultima = l.strip()[:200]
            if tanda is not None and ultima:
                break
    except Exception:
        pass
    return {"vivo": vivo, "tanda": tanda, "ultima_linea": ultima}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--horas", type=float, default=5.0)
    ap.add_argument("--carpeta", default="logs/sesion")
    ap.add_argument("--periodo", type=float, default=PERIODO_S)
    a = ap.parse_args()
    carpeta = Path(a.carpeta)
    (carpeta / "fotos").mkdir(parents=True, exist_ok=True)
    estado_f = carpeta / "estado.jsonl"
    alertas_f = carpeta / "alertas.log"
    fin = time.time() + a.horas * 3600
    n = 0

    def alerta(msg):
        linea = f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {msg}"
        print("ALERTA", linea, flush=True)
        with alertas_f.open("a") as f:
            f.write(linea + "\n")

    while time.time() < fin:
        n += 1
        t = time.strftime("%Y%m%d_%H%M%S")
        reg = {"t": time.time(), "hora": time.strftime("%Y-%m-%d %H:%M:%S"),
               "salud": _json("/api/health"), "seguridad": _json("/api/seguridad"),
               "sistema": _tegrastats(), "entrenamiento": _entrenamiento()}
        foto = carpeta / "fotos" / f"camara_{t}.jpg"
        reg["foto_camara"] = foto.name if _foto_video(foto) else None
        try:
            (carpeta / "fotos" / f"sendero_{t}.jpg").write_bytes(_get(BACKEND + "/api/sendero/vista.jpg"))
            reg["foto_sendero"] = f"sendero_{t}.jpg"
        except Exception:
            reg["foto_sendero"] = None
        with estado_f.open("a") as f:
            f.write(json.dumps(reg, ensure_ascii=False) + "\n")

        # ---- anomalías ----
        salud = reg["salud"] if isinstance(reg["salud"], list) else []
        if not salud:
            alerta(f"backend sin responder /api/health: {reg['salud']}")
        for fila in salud:
            # La C3 y la LoRa pueden estar apagadas a propósito (robot parado).
            if fila.get("key") in ("lidar", "camera", "slam", "ros", "ekf") and fila.get("status") == "err":
                alerta(f"{fila['key']} en error: {fila.get('detail', '')}")
        seg = reg["seguridad"]
        if isinstance(seg, dict) and seg.get("nivel") not in (None, "normal"):
            alerta(f"modo seguro en {seg.get('nivel')}: {seg.get('motivos')}")
        s = reg["sistema"]
        if s.get("tj_c", 0) >= 80:
            alerta(f"Jetson caliente: {s['tj_c']} °C")
        if s.get("ram_usada_mb") and s["ram_total_mb"] - s["ram_usada_mb"] < 600:
            alerta(f"poca RAM libre: {s['ram_total_mb'] - s['ram_usada_mb']} MB")
        if not reg["entrenamiento"]["vivo"] and n > 1:
            alerta("el entrenamiento ya no está corriendo")
        print(f"[{reg['hora']}] muestra {n} · tanda {reg['entrenamiento']['tanda']} · "
              f"tj {s.get('tj_c')} °C · RAM {s.get('ram_usada_mb')} MB · "
              f"seguridad {seg.get('nivel') if isinstance(seg, dict) else '?'}", flush=True)
        time.sleep(max(5.0, a.periodo))


if __name__ == "__main__":
    main()
