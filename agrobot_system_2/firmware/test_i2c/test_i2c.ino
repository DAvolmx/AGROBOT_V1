/*
 * test_i2c.ino — diagnostico del bus I2C del ESP32 LoRa V3 (Heltec, S3).
 *
 * Sube ESTE sketch, abre el monitor serie a 115200 y lee. No toca LoRa ni
 * WiFi: si el problema fuese de alimentacion, meter las radios en la
 * ecuacion lo volveria intermitente y no se podria concluir nada.
 *
 * ORDEN DE LAS PRUEBAS — importa, y es la leccion aprendida a golpes:
 *
 *   Barrer las 126 direcciones deja el periferico I2C del ESP32 en estado de
 *   error, y a partir de ahi TODO lo que venga despues lee 0xFF, incluidos
 *   sensores que estan perfectamente. Un escaneo puesto al principio no
 *   diagnostica la averia: la fabrica.
 *
 *   Por eso aqui se prueban primero las direcciones CONOCIDAS sobre un bus
 *   limpio, y el barrido completo va AL FINAL, cuando ya no queda nada que
 *   pueda contaminar. Para repetir el barrido, reinicia la placa.
 */
#include <Wire.h>

#define I2C_SDA 41
#define I2C_SCL 42

static bool responde(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Lee un registro. Devuelve false si el sensor no contesta, en vez de
// devolver 0xFF, que es indistinguible de un registro que valga 0xFF.
static bool leerReg(uint8_t addr, uint8_t reg, uint8_t* valor) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) < 1) return false;
  *valor = Wire.read();
  return true;
}

void probarMPU() {
  Serial.println("\n--- MPU6050 (acelerometro + giroscopio) ---");
  Serial.println("    De aqui sale TODA la odometria del robot: sin este");
  Serial.println("    sensor no hay TF odom->base_link y el SLAM no mapea.");
  uint8_t id;
  for (uint8_t addr : {(uint8_t)0x68, (uint8_t)0x69}) {
    if (!leerReg(addr, 0x75, &id)) {          // WHO_AM_I
      Serial.printf("  0x%02X: no contesta\n", addr);
      continue;
    }
    Serial.printf("  0x%02X: WHO_AM_I = 0x%02X", addr, id);
    if (id == 0x68)      Serial.println("   OK, es un MPU6050");
    else if (id == 0x70) Serial.println("   es un MPU6500/9250 (compatible)");
    else if (id == 0xFF) Serial.println("   0xFF: nadie tira de la linea,"
                                        " sensor sin alimentar o sin cable");
    else                 Serial.println("   desconocido");
  }
}

void probarSCD40() {
  Serial.println("\n--- SCD40 (CO2, temperatura, humedad) ---");
  if (!responde(0x62)) {
    Serial.println("  0x62 no contesta. El sensor no esta en el bus.");
    return;
  }
  Serial.println("  0x62 responde.");

  // stop_periodic_measurement. Es el paso que le faltaba al firmware viejo:
  // tras un reinicio en caliente el SCD40 sigue midiendo de la sesion
  // anterior y en ese estado RECHAZA el start. El firmware no miraba el
  // error y daba el sensor por bueno, asi que devolvia ceros para siempre.
  Wire.beginTransmission(0x62); Wire.write(0x3F); Wire.write(0x86);
  Wire.endTransmission();
  delay(500);

  Wire.beginTransmission(0x62); Wire.write(0x36); Wire.write(0x82);   // serial
  if (Wire.endTransmission() == 0) {
    delay(2);
    if (Wire.requestFrom((uint8_t)0x62, (uint8_t)9) >= 9) {
      Serial.print("  numero de serie: ");
      for (int i = 0; i < 9; i++) Serial.printf("%02X", Wire.read());
      Serial.println("   <- el sensor esta vivo y habla");
    } else {
      Serial.println("  pide el numero de serie y no devuelve datos");
    }
  }

  Wire.beginTransmission(0x62); Wire.write(0x21); Wire.write(0xB1);
  Serial.printf("  start_periodic_measurement -> %s\n",
                Wire.endTransmission() == 0 ? "aceptado" : "RECHAZADO");
  Serial.println("  esperando 6 s a la primera medida...");
  delay(6000);

  // data_ready. El firmware viejo tampoco lo consultaba: leia a ciegas 5 ms
  // despues de pedir la medida, cuando el sensor entrega una cada 5 s. De
  // ahi que 2 de cada 3 paquetes llegaran a cero.
  Wire.beginTransmission(0x62); Wire.write(0xE4); Wire.write(0xB8);
  Wire.endTransmission();
  delay(2);
  if (Wire.requestFrom((uint8_t)0x62, (uint8_t)3) >= 3) {
    uint16_t st = ((uint16_t)Wire.read() << 8) | Wire.read(); Wire.read();
    Serial.printf("  data_ready = %s\n", (st & 0x07FF) ? "SI" : "NO");
  }

  Wire.beginTransmission(0x62); Wire.write(0xEC); Wire.write(0x05);
  Wire.endTransmission();
  delay(2);
  if (Wire.requestFrom((uint8_t)0x62, (uint8_t)9) >= 9) {
    uint8_t d[9];
    for (int i = 0; i < 9; i++) d[i] = Wire.read();
    uint16_t co2 = ((uint16_t)d[0] << 8) | d[1];
    int16_t  tr  = (int16_t)(((uint16_t)d[3] << 8) | d[4]);
    uint16_t hr  = ((uint16_t)d[6] << 8) | d[7];
    Serial.printf("  CO2 = %u ppm | T = %.2f C | HR = %.1f %%\n",
                  co2, -45.0 + 175.0 * (tr / 65536.0), 100.0 * (hr / 65536.0));
    if (co2 == 0) Serial.println("  CO2 = 0 significa medida aun invalida");
  } else {
    Serial.println("  read_measurement no devuelve nada");
  }
}

// El barrido va al final por lo explicado en la cabecera: deja el periferico
// en error y contaminaria cualquier prueba posterior.
void escanearAlFinal() {
  Serial.println("\n--- Barrido completo del bus (ULTIMA prueba) ---");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (responde(a)) {
      Serial.printf("  0x%02X responde", a);
      switch (a) {
        case 0x68: Serial.print("   <- MPU6050"); break;
        case 0x69: Serial.print("   <- MPU6050 con AD0 en alto"); break;
        case 0x62: Serial.print("   <- SCD40"); break;
        case 0x3C: Serial.print("   <- OLED de la placa"); break;
      }
      Serial.println();
      n++;
    }
    delay(2);
  }
  Serial.printf("  total: %d dispositivo(s)\n", n);
  if (n == 0) {
    Serial.println();
    Serial.println("  NINGUNA direccion contesta. Eso NO es un sensor roto:");
    Serial.println("  es el bus entero mudo. En orden de probabilidad:");
    Serial.println("    1. Faltan los pull-up. SDA y SCL necesitan 4k7 a 3V3.");
    Serial.println("       Muchos modulos los traen, pero si ninguno de los");
    Serial.println("       conectados los tiene, las lineas flotan y todo se");
    Serial.println("       lee como 0xFF.");
    Serial.println("    2. GND no comun entre la placa y los sensores.");
    Serial.println("    3. SDA y SCL cruzados, o en otros pines. Este sketch");
    Serial.println("       usa SDA=41 y SCL=42.");
    Serial.println("    4. Sensores sin 3V3. Medilo con el tester, no a ojo.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2500);                        // margen para abrir el monitor serie
  Serial.println("\n\n===== test_i2c — diagnostico de sensores AgroBot =====");
  Serial.printf("pines: SDA=%d  SCL=%d\n", I2C_SDA, I2C_SCL);

  if (!Wire.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("✗ Wire.begin() FALLO: no se pudieron tomar esos pines.");
    return;
  }
  // Sin timeout, una linea pegada a masa bloquea el sketch entero y no se
  // llega a imprimir ni el primer resultado: parece que la placa esta
  // muerta cuando en realidad esta esperando a un bus que no va a soltarse.
  Wire.setTimeOut(50);
  delay(300);

  probarMPU();                        // primero, sobre un bus limpio
  probarSCD40();
  escanearAlFinal();                  // destructivo: siempre el ultimo

  Serial.println("\n===== fin — reinicia la placa para repetir =====");
}

void loop() { delay(10000); }
