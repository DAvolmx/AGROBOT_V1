// =============================================================================
// Codec de ChirpStack para el AgroBot — pegar en:
//   Device profiles -> US915_PROFILE -> pestana "Payload codec"
//   -> Codec = "JavaScript functions"
//
// Reemplaza al codec que había, que pertenecía a OTRA versión del firmware:
// inventaba campos (lidar_min, plaga_conf, estado_str, servo…) leyendo los
// mismos bytes con un layout distinto. No fallaba nunca, simplemente
// devolvía números plausibles y falsos: su "hum": 0.93 era en realidad el
// eje Z del acelerómetro.
//
// Este espeja byte a byte prepareTxFrame() de firmware/esp32_lora_v3 y
// decoder.py del gateway. Los tres tienen que cambiar juntos.
//
// Layout, 17 bytes (v3, el que manda el robot), big-endian por campo:
//   0-5   accelX,Y,Z   int16 / 100   (g)
//   6-11  gyroX,Y,Z    int16 / 10    (deg/s)
//   12-13 co2          uint16        (ppm)
//   14-15 tempSCD      int16 / 100   (degC)
//   16    humidity     uint8         (%)
//
// El firmware en uso NO manda mascara de validez, asi que el payload
// termina en el byte 16. Se aceptan ademas tres tamanos mas: 18 = la
// variante con mascara en el byte 17 (bit0 MPU, bit1 SCD40), hoy sin usar;
// 22 = v2, traia presion y tempBMP en 17-20 y la mascara en el 21; 21 = v1.
// El BMP280 se retiro del robot, asi que esos bytes se descartan.
// =============================================================================

function i16(b, i) {
  var v = (b[i] << 8) | b[i + 1];
  return v > 32767 ? v - 65536 : v;   // complemento a dos
}

function u16(b, i) {
  return (b[i] << 8) | b[i + 1];
}

function decodeUplink(input) {
  var b = input.bytes;

  // 17 = firmware actual (sin presion, sin mascara). 18 = la variante con
  // mascara. 22 = v2. 21 = v1. Se aceptan los cuatro para poder actualizar
  // nodo y servidor en momentos distintos sin quedarse sin telemetria.
  if (b.length !== 17 && b.length !== 18 && b.length !== 21 && b.length !== 22) {
    return { errors: ["payload de " + b.length + " bytes, se esperaban 17, 18, 21 o 22"] };
  }

  // Sin mascara se asume todo presente: es lo unico que se puede hacer sin
  // esa informacion.
  var validos = 0xFF;
  var version = 1;
  if (b.length === 17)      { version = 3; }
  else if (b.length === 18) { validos = b[17]; version = 3; }
  else if (b.length === 22) { validos = b[21]; version = 2; }

  var ausentes = [];
  if (!(validos & 0x01)) ausentes.push("mpu6050");
  if (!(validos & 0x02)) ausentes.push("scd40");

  return {
    data: {
      _sensores_ausentes: ausentes,
      _payload_version: version,
      accelX: i16(b, 0) / 100,
      accelY: i16(b, 2) / 100,
      accelZ: i16(b, 4) / 100,
      gyroX: i16(b, 6) / 10,
      gyroY: i16(b, 8) / 10,
      gyroZ: i16(b, 10) / 10,
      co2: u16(b, 12),
      tempSCD: i16(b, 14) / 100,
      humidity: b[16]
    }
  };
}

// Downlinks: 1 byte en el puerto 10. Los códigos son los de DOWNLINK_CODES
// en gateway_server.py.
function encodeDownlink(input) {
  var codigos = { EMERGENCY_STOP: 0x01, PARK: 0x02, PING: 0x03, ACK: 0x04 };
  var c = codigos[input.data.command];
  if (c === undefined) {
    return { errors: ["comando desconocido: " + input.data.command] };
  }
  return { bytes: [c], fPort: 10 };
}
