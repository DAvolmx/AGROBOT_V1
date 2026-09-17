"""
Esquema de la telemetría LoRaWAN de AgroBot — versión 1.

Antes cada campo vivía suelto (un dict con claves sueltas, sin unidades
ni rango documentado). Esto centraliza: nombre, unidad, rango físico
válido y a qué sensor pertenece cada campo — así el dashboard, el CSV
exportado y la tesis citan siempre la misma fuente de verdad, y un valor
fuera de rango (sensor desconectado, ruido I2C) se puede marcar como
sospechoso en vez de graficarse como si fuera bueno.

El payload binario de 18 bytes en sí (ver decoder.py) no cambia — este
esquema es la capa de METADATA sobre esos mismos campos.
"""

import math

SCHEMA_VERSION = 1

FIELDS = {
    "accelX": {"unit": "g", "sensor": "MPU6050", "range": (-2.0, 2.0), "label": "Aceleración X"},
    "accelY": {"unit": "g", "sensor": "MPU6050", "range": (-2.0, 2.0), "label": "Aceleración Y"},
    "accelZ": {"unit": "g", "sensor": "MPU6050", "range": (-2.0, 2.0), "label": "Aceleración Z"},
    "gyroX": {"unit": "°/s", "sensor": "MPU6050", "range": (-250.0, 250.0), "label": "Giroscopio X (roll rate)"},
    "gyroY": {"unit": "°/s", "sensor": "MPU6050", "range": (-250.0, 250.0), "label": "Giroscopio Y (pitch rate)"},
    "gyroZ": {"unit": "°/s", "sensor": "MPU6050", "range": (-250.0, 250.0), "label": "Giroscopio Z (yaw rate)"},
    "co2": {"unit": "ppm", "sensor": "SCD40", "range": (400, 5000), "label": "CO2"},
    "tempSCD": {"unit": "°C", "sensor": "SCD40", "range": (-10, 60), "label": "Temperatura (SCD40)"},
    "humidity": {"unit": "%", "sensor": "SCD40", "range": (0, 100), "label": "Humedad relativa"},
}

METADATA_FIELDS = {
    "rssi": {"unit": "dBm", "sensor": "gateway", "range": (-140, 0), "label": "RSSI"},
    "snr": {"unit": "dB", "sensor": "gateway", "range": (-20, 15), "label": "SNR"},
    "sf": {"unit": "", "sensor": "gateway", "range": (7, 12), "label": "Spreading Factor"},
}


def validate(decoded: dict) -> dict:
    """
    Devuelve el mismo dict con dos claves extra:

      `_sensors_missing`: campos que faltaban, venían nulos o no eran un
          número finito (sensor que no inicializó, I2C mudo, payload
          corrupto). Se sustituyen por `0.0` para que nada aguas abajo
          reviente, pero se listan acá para no hacer pasar un cero de
          "sensor caído" por una medición real de cero.

      `_out_of_range`: campos con un valor numérico válido pero fuera del
          rango físico posible (sensor conectado pero devolviendo basura).
          El dato NO se descarta, solo se marca.
    """
    result = dict(decoded)
    missing = []
    for name in FIELDS:
        try:
            number = float(result.get(name))
        except (TypeError, ValueError):
            number = None
        if number is None or not math.isfinite(number):
            result[name] = 0.0
            missing.append(name)
        else:
            result[name] = number

    out_of_range = []
    for name, meta in FIELDS.items():
        if name in missing:
            continue  # ya está marcado como ausente, no ensuciar las dos listas
        lo, hi = meta["range"]
        if not (lo <= result[name] <= hi):
            out_of_range.append(name)

    # El firmware v2 dice explícitamente qué sensores no respondieron. Esa
    # información gana sobre cualquier deducción por rango: un sensor
    # desconectado manda 0.0, y 0.0 cae fuera de rango, así que sin la
    # máscara acababa en `_out_of_range` — "sensor conectado devolviendo
    # basura" — cuando la realidad es que no hay sensor. Son dos problemas
    # distintos y llevan a revisar dos cosas distintas.
    for name in decoded.get("_ausentes", []):
        if name in FIELDS and name not in missing:
            missing.append(name)
            if name in out_of_range:
                out_of_range.remove(name)

    result["_sensors_missing"] = missing
    result["_out_of_range"] = out_of_range
    return result
