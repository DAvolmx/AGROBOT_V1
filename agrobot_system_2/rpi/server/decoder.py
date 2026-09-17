"""
Decodificador del payload LoRaWAN que arma `prepareTxFrame()` en el firmware
ESP32 LoRa V3 (ver firmware/esp32_lora_v3/esp32_lora_v3.ino). Debe coincidir
byte a byte.

Layout (big-endian por campo, tal como lo empaqueta el firmware con
desplazamiento manual de bytes):
  0-5   accelX,Y,Z   int16 * 100
  6-11  gyroX,Y,Z    int16 * 10
  12-13 co2          uint16
  14-15 tempSCD      int16 * 100
  16    humidity     uint8
  17    válidos      uint8, máscara de bits (sólo en las versiones que la
        llevan; el firmware en uso no la manda)

Cuatro tamaños aceptados a propósito:

  17 bytes (v3) — EL QUE MANDA EL ROBOT AHORA. Es el firmware que se sabe
      que hace JOIN en campo, con el BMP280 quitado. No lleva máscara de
      validez: un sensor ausente y uno midiendo cero llegan idénticos, y es
      `schema.validate()` quien intenta distinguirlos por rango.

  18 bytes (v3b) — variante con máscara de validez en el byte 17. No está
      en uso. Se sigue aceptando por si se vuelve a ella: distinguir "sin
      dato" de "cero medido" sólo se puede hacer con ese byte.

  22 bytes (v2) — firmware anterior. Traía presión y tempBMP en los bytes
      17-20 y la máscara en el 21. Se decodifica el resto y esos cuatro
      bytes se descartan.

  21 bytes (v1) — el más antiguo, sin máscara. Se marcan todos los sensores
      como válidos y es `schema.validate()` quien intenta distinguir un
      sensor ausente de uno midiendo cero, que es lo mejor que se puede
      hacer sin la información.

Se aceptan los tres para poder actualizar el firmware del robot y el servidor
en momentos distintos: en campo no siempre se puede reprogramar el nodo el
mismo día que se despliega el gateway.
"""
import struct

PAYLOAD_SIZE = 21        # v1, sin máscara de validez
PAYLOAD_SIZE_V2 = 22     # v2, con máscara y con BMP280
PAYLOAD_SIZE_V3 = 17     # v3, sin BMP280 y sin máscara — el que está en uso
PAYLOAD_SIZE_V3B = 18    # v3b, sin BMP280 y CON máscara — no está en uso

VAL_MPU = 0x01
VAL_SCD40 = 0x02
# 0x04 era el BMP280. No se reutiliza para otro sensor: un nodo con firmware
# viejo lo sigue poniendo a 1 y el bit acabaría validando un dato ajeno.

# Qué campos deja de tener sentido cada sensor cuando no responde.
CAMPOS_POR_SENSOR = {
    VAL_MPU: ("accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ"),
    VAL_SCD40: ("co2", "tempSCD", "humidity"),
}

_TAMANOS = {PAYLOAD_SIZE: 1, PAYLOAD_SIZE_V2: 2,
            PAYLOAD_SIZE_V3: 3, PAYLOAD_SIZE_V3B: 3}
# Dónde está la máscara en cada versión que la lleva. Las que no aparecen
# aquí no la traen, y se asume todo presente.
_POS_MASCARA = {PAYLOAD_SIZE_V2: 21, PAYLOAD_SIZE_V3B: 17}


def decode(raw: bytes) -> dict | None:
    version = _TAMANOS.get(len(raw))
    if version is None:
        return None

    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, co2, temp_scd_raw = struct.unpack(
        ">hhhhhhHh", raw[0:16]
    )
    humidity = raw[16]

    # Sin máscara (v1 y v3) se asume todo presente, que es lo que hacía el
    # sistema entero antes de que la máscara existiera.
    pos = _POS_MASCARA.get(len(raw))
    validos = raw[pos] if pos is not None else 0xFF
    ausentes = [c for bit, campos in CAMPOS_POR_SENSOR.items()
                if not (validos & bit) for c in campos]

    return {
        "_ausentes": ausentes,
        "_payload_version": version,
        "accelX": accel_x / 100.0,
        "accelY": accel_y / 100.0,
        "accelZ": accel_z / 100.0,
        "gyroX": gyro_x / 10.0,
        "gyroY": gyro_y / 10.0,
        "gyroZ": gyro_z / 10.0,
        "co2": co2,
        "tempSCD": temp_scd_raw / 100.0,
        "humidity": float(humidity),
    }
