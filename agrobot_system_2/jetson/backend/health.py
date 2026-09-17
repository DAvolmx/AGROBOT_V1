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
