import http.server
import subprocess
import threading
import time
from pathlib import Path

from reloj import DetectorSalto

VIGIA = Path(__file__).resolve().parents[2] / "systemd" / "sbin" / "agrobot-reloj-vigia.sh"


def test_sin_salto_no_avisa():
    d = DetectorSalto()
    for k in range(20):
        assert d.paso(1000.0 + k, 50.0 + k) is None
    assert d.saltos == []


def test_salto_adelante_de_711_s():
    d = DetectorSalto()
    d.paso(1000.0, 50.0)
    d.paso(1001.0, 51.0)
    assert d.paso(1002.0 + 711.5, 52.0) == 711.5 - 0.0 or abs(d.saltos[-1]["delta_s"] - 711.5) < 0.2
    assert d.paso(1003.0 + 711.5, 53.0) is None            # tras el salto, de nuevo estable


def test_salto_atras_y_pequenas_deriva():
    d = DetectorSalto(umbral_s=2.0)
    d.paso(1000.0, 0.0)
    assert d.paso(1001.5, 1.0) is None                      # 0,5 s de deriva: no es salto
    assert d.paso(1000.0, 2.0) < -2.0                       # 1,5 s hacia atrás respecto a la deriva: sí
    

class _Servidor:
    """Backend de mentira: responde ocupado=true las primeras `n` veces y luego false."""
    def __init__(self, n):
        self.n, self.llamadas = n, 0
        outer = self

        class H(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                outer.llamadas += 1
                cuerpo = ('{"ocupado": true, "motivos": ["autonomía: mision"]}' if outer.llamadas <= outer.n else '{"ocupado": false, "motivos": []}').encode()
                self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers(); self.wfile.write(cuerpo)

            def log_message(self, *a):
                pass
        self.srv = http.server.HTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.srv.server_port}/api/ocupado"
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def cerrar(self):
        self.srv.shutdown()


def _esperar(url, **env):
    e = {"VIGIA_LIB": "1", "VIGIA_URL": url, "PATH": "/usr/bin:/bin", **env}
    t0 = time.time()
    r = subprocess.run(["bash", "-c", f"source {VIGIA}; esperar_reposo"], env=e, capture_output=True, text=True, timeout=60)
    return time.time() - t0, r


def test_vigia_espera_mientras_el_robot_esta_ocupado():
    s = _Servidor(n=4)
    try:
        dt, r = _esperar(s.url, IDLE_S="1", SONDEO_S="0.2")
    finally:
        s.cerrar()
    assert r.returncode == 0
    assert "ocupado" in r.stdout
    assert s.llamadas >= 4 + 1                              # sondeó todo el rato ocupado y luego el reposo
    assert dt >= 4 * 0.2 + 1 - 0.3                          # 4 sondeos ocupados + 1 s seguido en reposo


def test_vigia_no_espera_si_el_backend_no_responde():
    dt, r = _esperar("http://127.0.0.1:9/api/ocupado", IDLE_S="1", SONDEO_S="0.2")
    assert r.returncode == 0 and dt < 3.0


def test_vigia_tiene_tope_de_espera():
    s = _Servidor(n=10 ** 6)
    try:
        dt, r = _esperar(s.url, IDLE_S="30", SONDEO_S="0.2", ESPERA_MAX_S="1")
    finally:
        s.cerrar()
    assert r.returncode == 0 and dt < 6.0
