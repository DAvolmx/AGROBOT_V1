"""
Registro de salud de cada fuente de datos ("tópicos" en el sentido ROS,
aunque este backend no corre sobre ROS): LiDAR, cámara, ESP32-C3, ESP32
LoRa V3, SLAM. Cada fuente reporta un "latido" (heartbeat) cuando produce
un dato nuevo; este módulo solo calcula, para cada una, si el último
latido es reciente (OK), viejo (WARN) o nunca llegó (ERR) — igual al
panel "QUÉ ESTÁ VIVO" del mockup de referencia.

Los umbrales de WARN/ERR son configurables (ver config_store.DEFAULTS,
sección "safety") en vez de estar fijos en código.
"""
import time

TOPICS = {
    "lidar": {"label": "LiDAR (/scan)", "icon": "📡"},
    "camera": {"label": "Cámara (/image_raw)", "icon": "📷"},
    "esp32c3": {"label": "ESP32-C3 (/c3/status)", "icon": "⚙️"},
    "lora_v3": {"label": "ESP32 LoRa V3 (/env/sensors)", "icon": "📶"},
    "slam": {"label": "SLAM (/map)", "icon": "🗺️"},
}

_last_seen: dict[str, float] = {}


def beat(topic: str) -> None:
    """Llamar cada vez que llega un dato nuevo de esa fuente."""
    _last_seen[topic] = time.time()


def snapshot(warn_s: float, err_s: float) -> list[dict]:
    now = time.time()
    out = []
    for key, meta in TOPICS.items():
        last = _last_seen.get(key)
        if last is None:
            status, age = "err", None
        else:
            age = now - last
            status = "ok" if age <= warn_s else ("warn" if age <= err_s else "err")
        out.append({
            "key": key,
            "label": meta["label"],
            "icon": meta["icon"],
            "status": status,
            "age_s": age,
        })
    return out


class ContadorSecuencia:
    """Calidad de un enlace por su número de secuencia (el `seq` del nodo
    LoRa V3). Un hueco en la secuencia es un paquete perdido; un `seq` que
    vuelve atrás de golpe es la placa que se reinició (el contador arranca
    en cero). Así se ven los reinicios por caída de tensión sin tocar el
    firmware, que es lo que el 2026-09-28 hubo que deducir a mano."""

    VENTANA = 2000      # paquetes para el % de pérdida (~100 s a 20 Hz)

    def __init__(self):
        self.ultimo = None
        self.recibidos = 0
        self.perdidos = 0
        self.reinicios = 0
        self.ultimo_reinicio = None
        self._ventana = []          # (recibidos, perdidos) por paquete

    def registrar(self, seq: int) -> bool:
        """Devuelve True si este paquete indica un reinicio de la placa."""
        reinicio = False
        perdidos = 0
        if self.ultimo is not None:
            salto = seq - self.ultimo
            if salto <= 0:
                # Duplicado o reinicio. Un reinicio vuelve a valores bajos.
                if seq < 50 or salto < -1000:
                    reinicio = True
                    self.reinicios += 1
                    self.ultimo_reinicio = time.time()
            elif salto > 1:
                perdidos = min(salto - 1, 1000)
        self.ultimo = seq
        self.recibidos += 1
        self.perdidos += perdidos
        self._ventana.append(perdidos)
        if len(self._ventana) > self.VENTANA:
            del self._ventana[:len(self._ventana) - self.VENTANA]
        return reinicio

    def resumen(self) -> dict:
        perd = sum(self._ventana)
        total = len(self._ventana) + perd
        return {"perdida_pct": round(100.0 * perd / total, 1) if total else None,
                "reinicios": self.reinicios,
                "ultimo_reinicio": self.ultimo_reinicio}
