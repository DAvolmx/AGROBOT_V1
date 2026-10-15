"""
Protocolo binario Jetson <-> ESP32-C3 Mini (actuador).

Debe coincidir byte a byte con los structs #pragma pack(1) del firmware
esp32c3_actuator.ino. Formato little-endian, sin padding.

VERSIÓN 4 (2026-09-02) — se ELIMINÓ `vy_ref` (velocidad lateral) que había
traído la v3. La v3 partía de una deducción equivocada: los sentidos por
rueda del sketch de prueba coincidían con la cinemática mecanum, pero eso
era una coincidencia de patrones, no una prueba. El usuario confirmó que el
robot **no tiene ruedas mecanum**: es DIFERENCIAL (skid-steer) y gira por
patinaje. No puede desplazarse de costado.

Se quita el campo en vez de dejarlo en cero: un campo fantasma que el robot
nunca puede ejecutar termina apareciendo en la UI, en la odometría y en
Nav2 como si fuera una capacidad real.

  CmdPacket    18 bytes -> 14 bytes  (- vy_ref float)
  StatusPacket 19 bytes -> 15 bytes  (- vy_cmd float)

Convención de signos (la misma de ROS): +x adelante, +omega antihorario
(giro a la izquierda).

Cambiar cualquiera de estos structs OBLIGA a reflashear el ESP32-C3: el
firmware descarta todo paquete cuyo tamaño no coincida exactamente con
`sizeof(CmdPacket)`, así que una Jetson nueva contra un firmware viejo
simplemente no mueve el robot (falla segura, no comportamiento raro).
"""
import struct

PROTOCOL_VERSION = 4

CMD_MAGIC = 0xA5
STATUS_MAGIC = 0x5A

# uint8 magic, float v_ref, float omega_ref,
# uint8 scan_cmd, uint8 servo_angle, uint8 led_cmd, uint16 seq
CMD_FORMAT = "<BffBBBH"
CMD_SIZE = struct.calcsize(CMD_FORMAT)

# uint8 magic, float v_cmd, float omega_cmd,
# uint8 scanning, uint8 failsafe, uint8 servo_angle, uint8 led_state,
# uint16 seq_echo
STATUS_FORMAT = "<BffBBBBH"
STATUS_SIZE = struct.calcsize(STATUS_FORMAT)

# Recorrido util del servo de camara. Debe coincidir con SERVO_MIN_ANGLE /
# SERVO_MAX_ANGLE del firmware: fuera de este arco el conjunto montado topa
# mecanicamente y el servo zumba contra el tope sin llegar a la posicion.
# Se recorta en los dos lados a proposito — el firmware es la ultima linea
# de defensa, pero la Jetson no debe mandar nunca un angulo imposible.
SERVO_MIN_DEG = 25
SERVO_MAX_DEG = 110

# Reposo: camara mirando al frente.
SERVO_HOME_DEG = 98


def clamp_servo(angle: float) -> int:
    """Recorta al recorrido util (25°-110°). Cualquier valor fuera de rango
    —por ejemplo tras sumarle el offset de calibración— se corta acá y no en
    el firmware, que es actuador puro."""
    return int(max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, round(angle))))


def pack_cmd(v_ref: float, omega_ref: float, scan_cmd: int,
             servo_angle: int, led_cmd: int, seq: int) -> bytes:
    return struct.pack(
        CMD_FORMAT, CMD_MAGIC, v_ref, omega_ref,
        scan_cmd & 0xFF, clamp_servo(servo_angle), 1 if led_cmd else 0,
        seq & 0xFFFF,
    )


def unpack_status(data: bytes) -> dict | None:
    if len(data) != STATUS_SIZE:
        return None
    (magic, v_cmd, omega_cmd, scanning, failsafe,
     servo_angle, led_state, seq_echo) = struct.unpack(STATUS_FORMAT, data)
    if magic != STATUS_MAGIC:
        return None
    return {
        "v_cmd": v_cmd,
        "omega_cmd": omega_cmd,
        "scanning": bool(scanning),
        "failsafe": bool(failsafe),
        "servo_angle": servo_angle,
        "led_state": bool(led_state),
        "seq_echo": seq_echo,
    }
