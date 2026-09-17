/*
 * AGROBOT — ESP32-C3 Mini — NODO ACTUADOR (sin encoder, sin PID)
 * ================================================================
 * Rol único: recibir comandos y aplicar PWM directo a motores/servo/LED.
 * Cero lógica de control aquí — eso vive en la Jetson (robot_localization
 * + nav2). Este firmware solo ejecuta y protege (watchdog/failsafe).
 *
 * UNA SOLA VIA DE MANDO: UART por cable desde la Jetson.
 *
 * Historia corta de por que, que importa para no repetirla:
 *   - WiFi + UDP: el C3 tiene UNA sola radio de 2.4 GHz. La asociacion al AP
 *     se caia sola y no se recuperaba. Eliminado el 2026-09-04.
 *   - BLE: quitar el WiFi le dejaba la radio entera, pero la placa dejo de
 *     anunciarse y el enlace nunca llego a establecerse. Eliminado el mismo
 *     dia.
 *   - UART: sin radio. La Jetson y el C3 van atornillados al mismo robot, a
 *     veinte centimetros uno del otro, asi que un enlace inalambrico entre
 *     ellos nunca compro nada y si aporto todos los modos de fallo.
 *
 * CABLEADO (tres hilos, la masa NO es opcional):
 *   Jetson pin 8  (TXD, /dev/ttyTHS1)  ->  C3 GPIO20 (RX)
 *   Jetson pin 10 (RXD, /dev/ttyTHS1)  <-  C3 GPIO21 (TX)
 *   Jetson pin 6  (GND)                 -  C3 GND
 * Los dos lados son de 3,3 V: se conectan directo, sin adaptador de nivel.
 * Sin la masa comun las señales no tienen referencia y no funciona nada;
 * ya paso una vez con el driver de motores (2026-09-04).
 *
 * Reparto de responsabilidades, que no cambia:
 *   Jetson  -> toda la logica de control (nav2, robot_localization) y el
 *              dashboard web que se sirve sobre su propio AP.
 *   Este C3 -> actuador puro: aplica PWM y protege con watchdog/failsafe.
 *
 * El AP de la Jetson SIGUE existiendo: lo usan el dashboard web y el nodo
 * LoRa. Lo que ya no lo usa es este actuador.
 *
 * PROTOCOLO SERIE (8 bytes por trama, con magic y CRC-8). Debe coincidir
 * byte a byte con jetson/backend/uart_link.py.
 *
 * COMPILAR CON "USB CDC On Boot: Enabled". Los logs salen por el USB y las
 * tramas por GPIO20/21; si el USB CDC esta apagado, `Serial` es ESE MISMO
 * UART y los logs se mezclan con los datos.
 *
 * ---------------------------------------------------------------------
 * PINOUT Y LADOS — CONFIRMADOS EN EL ROBOT REAL (2026-09-02)
 * No son supuestos: salen del sketch de prueba del usuario, que se probó
 * moviendo el robot y funciona.
 *
 *   LADO DERECHO:   M1 FI=GPIO0 BI=GPIO1   +   M4 FI=GPIO6 BI=GPIO7
 *   LADO IZQUIERDO: M2 FI=GPIO3 BI=GPIO2   +   M3 FI=GPIO4 BI=GPIO5
 *   GPIO8  -> FOCO (LED)     GPIO10 -> SERVO-CAM
 *   GPIO9, 20, 21 libres (9 = BOOT, 20/21 = UART0 de consola)
 *
 * El M2 tiene FI y BI al revés que los demás. Está así a propósito: es lo
 * que quedó cableado y verificado en la placa. No "arreglarlo".
 *
 * ⚠️ El ENCABEZADO del sketch de prueba dice "M1 delantero derecho, M4
 * trasero IZQUIERDO", pero los comentarios dentro de girarIzquierda() dicen
 * "motor(1) y motor(4) = derecha". Se contradicen, y manda el segundo: con
 * ese agrupamiento el robot gira bien, que es lo que el usuario verificó.
 * Para control diferencial alcanza con saber DE QUÉ LADO está cada motor —
 * delante o atrás dentro de un mismo lado da igual, reciben lo mismo.
 *
 * ---------------------------------------------------------------------
 * ⚠️  EL ROBOT ES DIFERENCIAL (SKID-STEER). NO TIENE RUEDAS MECANUM.
 * Confirmado por el usuario (2026-09-02). Gira por patinaje: los dos lados
 * a distinta velocidad. Tiene DOS grados de libertad: vx y omega.
 *
 * NO puede desplazarse de costado. Por eso la v4 del protocolo elimina
 * `vy_ref`: ofrecer un control que el robot no puede ejecutar es peor que
 * no ofrecerlo — hace patinar las ruedas y ensucia la odometría.
 */


// El enlace con la Jetson es un UART por cable: no hace falta ninguna
// libreria de radio. Ver el bloque PROTOCOLO SERIE mas abajo.


// =============================================
// ===== PINES Y RUEDAS (confirmados en el robot real) =====
// =============================================
// Cada motor son DOS pines del TA6586: FI (adelante) y BI (reversa).
// Los nombres son por LADO, porque el control diferencial manda por lado:
// las dos ruedas de un mismo lado SIEMPRE reciben el mismo comando.
//
//   R1/R2 = los dos motores del lado DERECHO  (M1 y M4 del sketch)
//   L1/L2 = los dos motores del lado IZQUIERDO (M2 y M3 del sketch)
#define R1_FI  0
#define R1_BI  1
#define L1_FI  3   // ⚠️ invertidos respecto de los otros: así está
#define L1_BI  2   //    cableado y verificado. GPIO2 es pin de strapping.
#define L2_FI  4
#define L2_BI  5
#define R2_FI  6
#define R2_BI  7

#define LED_PIN    8    // FOCO. ⚠️ pin de strapping — ver aviso de cabecera

// POLARIDAD DEL FOCO. En la ESP32-C3 SuperMini el LED de la placa cuelga de
// este mismo GPIO8 y es de LOGICA INVERTIDA: se enciende con el pin en BAJO.
// Poner el pin a bajo para "apagar" lo que hacia era encenderlo.
//
// Si tu etapa de potencia del foco es de logica normal (se enciende con el
// pin en ALTO), cambia esto a false. Es el UNICO sitio donde hay que
// tocarlo: todo el resto del firmware razona en terminos de encendido y
// apagado, no de niveles.
// ⚠️ ESTE VALOR NO ES SOLO ESTETICO: DECIDE SI LA PLACA ARRANCA.
//
// GPIO8 es un PIN DE STRAPPING del ESP32-C3. En el reset el chip lo muestrea
// y **tiene que estar en ALTO** para arrancar desde la flash. Si esta en
// bajo, el chip NO ARRANCA: ni ejecuta el firmware, ni inicializa el USB, o
// sea que el ordenador tampoco lo reconoce. Parece una placa muerta.
//
// Y aqui esta la trampa: el estado de REPOSO del foco (apagado) es el que se
// queda en el pin la mayor parte del tiempo, incluido el momento del proximo
// reset.
//
//   LED_ACTIVO_EN_BAJO = false -> apagar escribe LOW  -> GPIO8 en bajo -> NO ARRANCA
//   LED_ACTIVO_EN_BAJO = true  -> apagar escribe HIGH -> GPIO8 en alto -> arranca
//
// El 2026-09-06 se puso en `false` y la placa dejo de arrancar y de ser
// reconocida por USB. Recuperacion: mantener BOOT pulsado mientras se conecta
// el USB (fuerza modo descarga y salta el strapping), o desconectar el cable
// del foco de GPIO8.
//
// Asi que `true` no es una preferencia, es un requisito. Y ademas coincide
// con lo que se observo al principio: con `false`, setup() escribia LOW y la
// lampara se ENCENDIA, o sea que la etapa es de logica invertida.
//
// SI ALGUN DIA HAY QUE PONERLO EN `false` porque la etapa se recablee a
// logica normal, entonces HAY QUE MOVER EL FOCO A OTRO PIN. GPIO8 y un
// consumidor que reposa en bajo son incompatibles, y no es negociable.
//
// Queda un detalle aparte: durante los primeros milisegundos, antes de que
// setup() escriba el pin, GPIO8 no lo gobierna nadie y el foco puede dar un
// destello. Eso se corrige con un pull-up en la puerta de la etapa, que
// ademas refuerza el arranque.
const bool LED_ACTIVO_EN_BAJO = true;

// Nivel fisico que hay que escribir para cada estado logico.
#define LED_NIVEL(encendido) \
  ((LED_ACTIVO_EN_BAJO ? !(encendido) : (encendido)) ? HIGH : LOW)
#define SERVO_PIN 10    // SERVO-CAM

// Si una rueda gira al revés de lo esperado (motor montado en espejo),
// poné su INVERT en true en vez de recablear.
const bool INVERT_R1 = false;
const bool INVERT_R2 = false;
const bool INVERT_L1 = false;
const bool INVERT_L2 = false;

// Frecuencia de PWM de los motores. Se fija EXPLÍCITAMENTE en vez de
// confiar en el default de analogWrite(), que cambió entre versiones del
// core de ESP32 (1 kHz en unas, 5 kHz en otras) — con el default, el mismo
// código se comporta distinto según con qué versión se compile.
// FRECUENCIA: 1 kHz, NO 8 kHz (bajada el 2026-09-08 por falta de par).
//
// A 8 kHz el robot movia poco aunque el duty fuera alto. El motivo es el
// driver: el TA6586 es un puente H de conmutacion LENTA —sus flancos de
// subida y bajada duran decenas de microsegundos—, asi que a 8 kHz (125 us
// de periodo) una parte grande de cada ciclo se la come la transicion, en
// la que el puente ni conduce del todo ni corta del todo. Esa parte se
// disipa como calor en el driver en vez de llegar al motor: el duty que se
// pide no es el par que sale.
//
// A 1 kHz el periodo es 1 ms, ocho veces mas largo, y la misma transicion
// pasa a ser una fraccion despreciable. El motor ve practicamente toda la
// tension pedida. Ademas 1 kHz sigue estando muy por encima de la constante
// de tiempo electrica del motorreductor, asi que la corriente sigue siendo
// continua y no se pierde par por rizado.
//
// El precio es el chillido audible de 1 kHz. Es el compromiso correcto: un
// robot que se mueve y zumba es mejor que uno silencioso que no arranca.
// Si molesta mucho, 2000 Hz es el siguiente escalon a probar; por encima de
// ~4 kHz vuelve a notarse la perdida de par.
const uint32_t MOTOR_PWM_HZ = 1000;
// 8 bits -> duty 0..255, que es justo el rango de PWM_MAX de más abajo.
const uint8_t MOTOR_PWM_BITS = 8;

// =============================================
// ===== PARÁMETROS FÍSICOS (de AGROBOT - MODELO MATEMATICO Y CONTROL) =====
// =============================================
const float WHEEL_RADIUS = 0.06f;   // m
const float TRACK_WIDTH  = 0.33f;   // m, separación izquierda-derecha
// Semi-trocha: es lo único que hace falta para diferencial. La separación
// delante-atrás NO entra en la cinemática de un skid-steer.
const float K_ROT        = TRACK_WIDTH / 2.0f;

const float V_MAX        = 0.55f;   // m/s  adelante/atrás
const float OMEGA_MAX    = 1.5f;    // rad/s
const int   PWM_MAX      = 255;

// =============================================
// ===== ARRANQUE: ZONA MUERTA + PULSOS DE TORQUE =====
// =============================================
// Ambas cosas salen del sketch de prueba del usuario, que rinde MEJOR que
// el PWM proporcional puro. La razón es física, no de código:
//
// (a) ZONA MUERTA. Un motorreductor con carga no arranca con cualquier
//     duty: por debajo de cierto umbral el TA6586 conmuta pero el par no
//     alcanza para vencer el rozamiento estático, y el motor solo zumba.
//     Todo el rango 0..PWM_MIN era, en la práctica, "parado con ruido".
//     Ahora la fracción pedida se mapea a [PWM_MIN..PWM_MAX], así el
//     primer escalón de velocidad ya mueve el robot de verdad.
//
// (b) PULSOS DE TORQUE. El rozamiento estático (arranque) es bastante
//     mayor que el dinámico (ya en movimiento). Golpear con duty completo
//     en pulsos cortos rompe la adherencia inicial y hace que el robot
//     salga parejo; con rampa suave desde cero, la rueda con más carga
//     arranca después que las otras y el robot sale torcido.
//     Es exactamente lo que hacía pulsos() del sketch de prueba —
//     3 pulsos de 200 ms con 30 ms de pausa— pero SIN delay(): acá corre
//     como máquina de estados por rueda, así el loop sigue atendiendo
//     el serie y el watchdog mientras dura el arranque. Con delay() el
//     failsafe de 300 ms saltaría en mitad de cada arranque.
//
// RANGO PEDIDO POR EL USUARIO: 200..255. Es angosto a propósito —abajo de
// ~200 estos motores con carga no mueven el robot— y tiene una consecuencia
// que conviene tener presente: la velocidad va del 78% al 100% de la
// potencia, así que el "paso lento" del acelerador sigue siendo bastante
// rápido. Es el compromiso correcto para este chasis: mejor poca gama de
// velocidad que una gama grande cuya mitad inferior no mueve nada.
const int PWM_MIN = 200;

const uint8_t       KICK_PULSOS  = 3;    // 0 = desactivar los pulsos
const unsigned long KICK_ON_MS   = 200;  // duración de cada pulso a fondo
const unsigned long KICK_OFF_MS  = 30;   // pausa entre pulsos
// Solo se re-arranca por pulsos si la rueda estuvo quieta al menos esto.
// Sin esta guarda, ir a paso lento con el duty oscilando cerca del umbral
// dispararía pulsos todo el tiempo y el robot avanzaría a los tirones.
const unsigned long KICK_REARME_MS = 400;

// DESFASE ENTRE LADOS AL ARRANCAR.
//
// Antes las cuatro ruedas daban su primer pulso a fondo EN EL MISMO
// INSTANTE. Cuatro motores pasando de parados a duty maximo a la vez es el
// peor caso de corriente de todo el robot, y es justo el momento en que se
// veia caer la tension de la bateria y reiniciarse la electronica.
//
// Escalonar un lado respecto del otro parte ese pico en dos mitades
// separadas en el tiempo. El robot no lo nota: 60 ms es menos de lo que
// tarda una rueda en empezar a moverse de verdad, asi que sigue arrancando
// recto. La bateria si lo nota.
//
// No se escalonan las dos ruedas DE UN MISMO LADO entre si a proposito: van
// mecanicamente unidas por el suelo y arrancarlas desfasadas solo las hace
// pelearse. El pico se parte por lados, que es la division que tiene sentido.
//
// Si aun asi cae la tension, subir este valor es lo primero que hay que
// probar; despues, bajar KICK_PULSOS de 3 a 2.
const uint16_t KICK_DESFASE_MS = 60;

// =============================================
// ===== GIRO: PISO DE DUTY + TREN DE PULSOS SOSTENIDO =====
// =============================================
// Girar sobre el sitio es la maniobra MAS exigente de este chasis, y por
// mucho. En un skid-steer las cuatro ruedas tienen que ARRASTRARSE de lado
// contra el suelo: no ruedan sobre su propio eje, derrapan. El rozamiento
// que hay que vencer es varias veces el de ir recto, y no se vence una sola
// vez al arrancar —dura todo el giro.
//
// Por eso el giro necesita DOS cosas que no necesita la marcha recta:
//
// (a) PISO DE DUTY PROPIO. La cinematica de un giro puro pide una velocidad
//     de rueda bastante menor que la de ir a fondo (con OMEGA_MAX=1.5 sale
//     ~45% de la rueda de referencia), asi que el mapeo normal daba un duty
//     de mitad de tabla: justo el que no alcanza para arrastrar las ruedas.
//     Girando, el piso pasa a ser PWM_MIN_GIRO.
//
// (b) TREN DE PULSOS SOSTENIDO. Los pulsos de arranque (KICK_*) son de UNA
//     VEZ, al empezar a moverse. Sirven para romper la inercia inicial pero
//     se acaban, y en un giro el rozamiento sigue ahi despues. Mientras el
//     comando sea de giro dominante, el duty alterna entre el pedido y el
//     maximo con este ritmo. Cada golpe rompe la adherencia estatica de las
//     ruedas que se hayan clavado, y en la fase baja el robot sigue tirando
//     (NO baja a 0: eso lo frenaria en seco cada 140 ms).
//
// Como el consumo medio de un giro sube con esto, es lo primero que hay que
// aflojar si vuelve a caer la tension: bajar PWM_MIN_GIRO, luego alargar
// GIRO_OFF_MS.
const int PWM_MIN_GIRO = 235;

// Un comando cuenta como "giro dominante" cuando la componente de rotacion
// pesa al menos tanto como la de avance en la velocidad de las ruedas. Con
// esto, girar sobre el sitio y las curvas cerradas reciben los pulsos, y una
// correccion suave de rumbo yendo recto NO (ahi los pulsos serian tirones
// molestos y consumo para nada).
const float GIRO_UMBRAL = 1.0f;
const unsigned long GIRO_ON_MS  = 140;  // golpe a fondo
const unsigned long GIRO_OFF_MS = 90;   // vuelta al duty pedido

bool          giroActivo    = false;  // el comando de ahora es giro dominante
bool          giroAlto      = false;  // fase del tren: golpe (true) o base
unsigned long giroFaseHasta = 0;

// =============================================
// ===== SEGURIDAD =====
// =============================================
// Sin comando fresco de la Jetson, el robot para. Es el unico watchdog que
// queda: el UART es la unica via de mando.
//
// 300 ms y no los 700 que hacian falta con BLE: un cable no renegocia
// intervalos ni se duerme, asi que el margen puede volver a ser el ajustado.
// La Jetson escribe a 20 Hz, o sea que hacen falta 6 tramas perdidas
// seguidas para que salte.
const unsigned long CMD_TIMEOUT_MS = 300;
unsigned long lastCmdMillis = 0;
bool inFailsafe = true;

// =============================================
// ===== ESTADO DEL ENLACE =====
// =============================================
// Ya no hay arbitraje: el UART es la unica via de mando, asi que no hay dos
// fuentes que puedan pelearse por los mismos motores.

// =============================================
// ===== SERVO DE CÁMARA + LED =====
// =============================================
// ── SERVO POR LEDC EN CRUDO, SIN ESP32Servo ────────────────────────────────
// La libreria se retiro el 2026-09-06 despues de aislarla en
// firmware/test_servo_minimo. Lo medido EN ESTA PLACA:
//
//   · `ledcAttachChannel(GPIO10, 50 Hz, 14 bits, canal 0)` engancha y el
//     servo SE MUEVE. Tambien valen 10 y 12 bits.
//   · Con 8 bits el core lo rechaza: "requested frequency 50 and duty
//     resolution 8 can not be achieved... div_param=0" — a 50 Hz el divisor
//     se queda sin margen. Con 16 tampoco: el C3 admite 14 como maximo
//     (SOC_LEDC_TIMER_BIT_WIDTH = 14).
//   · `ESP32Servo::attach(GPIO10)` devuelve 0 SIEMPRE y, al fallar, deja el
//     pin medio tomado: desde ese momento ningun ledcAttachChannel funciona
//     sobre ese GPIO en toda la sesion. Por eso el servo llevaba dias sin
//     moverse y los barridos de prueba "se saltaban" sin explicacion.
//
// 14 bits a 50 Hz: periodo de 20000 us, duty de 0 a 16383, o sea 0.819
// cuentas por microsegundo. De sobra para un servo, que distingue del orden
// de 10 us.
const int SERVO_LEDC_CANAL   = 0;      // 0 y 1 libres; los motores usan 2..5
const int SERVO_LEDC_BITS    = 14;
const int SERVO_LEDC_HZ      = 50;
const int SERVO_PULSO_MIN_US = 500;    // 0 grados
const int SERVO_PULSO_MAX_US = 2400;   // 180 grados
bool servoListo = false;

// (servoDuty y servoEscribir se definen mas abajo, junto a setServo: tienen
//  que quedar DESPUES de `struct Rueda` — ver la nota de los prototipos.)
bool scanning = false;
unsigned long scanStartMillis = 0;
const unsigned long SCAN_SWEEP_MS = 2000; // barrido completo en 2s

// Recorrido util del servo de camara, en grados. NO es 0-180: el conjunto
// montado topa mecanicamente fuera de este arco y forzarlo hace que el
// servo zumbe contra el tope, se caliente y tire de corriente sin llegar
// nunca a la posicion pedida. Todo angulo, venga del barrido o del slider
// del operador, se recorta a esta ventana en setServo().
const int SERVO_MIN_ANGLE  = 25;
const int SERVO_MAX_ANGLE  = 110;
// Reposo = camara mirando AL FRENTE. Es la posicion en la que arranca la
// placa y a la que vuelve al terminar cada barrido.
const int SERVO_PARK_ANGLE = 98;

// Estado real aplicado al hardware (lo que se ecoa en el status).
int  currentServoAngle = SERVO_PARK_ANGLE;
bool ledOn = false;

// ---------------------------------------------------------------------------
// TIPOS DE LAS RUEDAS — definidos AQUI, antes de la primera funcion.
//
// El IDE de Arduino genera prototipos automaticos y los inserta justo antes
// de la primera funcion del .ino. Si `struct Rueda` se define despues de ese
// punto, los prototipos de aplicarDuty(), setRueda() y setLado() nombran un
// tipo que todavia no existe y el error sale en la linea de la FUNCION, no
// en la del struct: parece que falta el struct cuando en realidad esta mas
// abajo. Definirlo aqui elimina la dependencia de donde decida el IDE poner
// los prototipos.
// ---------------------------------------------------------------------------
// CANALES 2..5, NO 0..3. El canal 0 es del servo (ver el bloque de LEDC mas
// arriba) y el 1 se deja libre. Cuando el servo lo gestionaba ESP32Servo, esa
// libreria repartia canales empezando por el 0 y el primer motor que se movia
// se lo robaba: `ledcAttachChannel(pin, 8000 Hz, 8 bits, 0)` reprogramaba el
// canal del servo a 8 kHz. La libreria ya no esta, pero la separacion se
// mantiene porque sigue siendo lo correcto: el servo va a 50 Hz y 14 bits y
// los motores a 8 kHz y 8 bits, y dos configuraciones asi no pueden compartir
// canal.
enum RuedaCanal : uint8_t { CH_R1 = 2, CH_R2 = 3, CH_L1 = 4, CH_L2 = 5 };

struct Rueda {
  const uint8_t pinFI;
  const uint8_t pinBI;
  const uint8_t canal;
  const bool    invertida;
  const uint16_t kickDesfaseMs;// cuanto espera esta rueda antes de su 1er pulso
  int8_t        sentido;      // +1 FI activo, -1 BI activo, 0 ninguno
  int           dutyPedido;   // lo que pide la cinemática, ya con zona muerta
  uint8_t       kickRestantes;// pulsos de arranque que faltan
  bool          kickAlto;     // fase actual del pulso: a fondo o en pausa
  unsigned long kickFaseHasta;// millis() en que termina la fase actual
  unsigned long quietaDesde;  // millis() en que esta rueda se detuvo
  bool          dcCompleto;   // pin activo en HIGH fijo, sin PWM (100% real)
};

static uint32_t servoDuty(int angulo) {
  int us = map(angulo, 0, 180, SERVO_PULSO_MIN_US, SERVO_PULSO_MAX_US);
  return (uint32_t)((float)us * ((float)(1UL << SERVO_LEDC_BITS) / 20000.0f));
}

static void servoEscribir(int angulo) {
  if (!servoListo) return;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(SERVO_LEDC_CANAL, servoDuty(angulo));
#else
  ledcWrite(SERVO_LEDC_CANAL, servoDuty(angulo));
#endif
}

void setServo(int angle) {
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  if (angle != currentServoAngle) {
    currentServoAngle = angle;
    servoEscribir(angle);
  }
}

void setLed(bool on) {
  if (on != ledOn) {
    ledOn = on;
    digitalWrite(LED_PIN, LED_NIVEL(on));
  }
}

// =============================================
// ===== PROTOCOLO SERIE (8 bytes por trama, packed) =====
// =============================================
// Se usan enteros con signo en PORCENTAJE en vez de floats: la trama es
// chica y no hay que preocuparse por como representa cada lado un float.
// Debe coincidir byte a byte con jetson/backend/uart_link.py.
//
// DIFERENCIA CLAVE CON UDP Y BLE: un UART es un CHORRO DE BYTES, no una
// secuencia de paquetes. Nadie marca donde empieza y termina una trama, asi
// que hacen falta dos cosas que los otros dos transportes regalaban:
//
//   1. Un byte de INICIO (el magic) para poder resincronizar. Si se pierde
//      un byte por ruido, o si se empieza a leer a mitad de trama, el
//      receptor descarta hasta encontrar el siguiente magic.
//   2. Un CHECKSUM. Sin el, un magic que aparezca por casualidad dentro de
//      los datos se tomaria por el principio de una trama y el robot
//      obedeceria basura. Con 8 bytes a 20 Hz el coste es irrelevante.
//
// Y hay una fuente de basura garantizada: el bootloader del ESP32-C3 escupe
// su mensaje de arranque por estos mismos pines en cada reset. La Jetson lo
// va a leer siempre; el tramado es lo que hace que no importe.
#pragma pack(push, 1)
struct CmdFrame {
  uint8_t  magic;        // 0xA5
  int8_t   v_pct;        // -100..100  -> se escala a V_MAX
  int8_t   omega_pct;    // -100..100  -> se escala a OMEGA_MAX
  uint8_t  servo_angle;  // 0..180
  uint8_t  flags;        // bit0 LED, bit1 SCAN
  uint16_t seq;
  uint8_t  crc;          // CRC-8 de los 7 bytes anteriores
};
struct StatusFrame {
  uint8_t  magic;        // 0x5A
  int8_t   v_pct;
  int8_t   omega_pct;
  uint8_t  servo_angle;
  uint8_t  flags;        // bit0 LED, bit1 escaneando, bit2 failsafe
  uint16_t seq_echo;
  uint8_t  crc;
};
#pragma pack(pop)

const uint8_t CMD_MAGIC    = 0xA5;
const uint8_t STATUS_MAGIC = 0x5A;

#define FLAG_LED     0x01
#define FLAG_SCAN    0x02

#define ST_LED       0x01
#define ST_SCANNING  0x02
#define ST_FAILSAFE  0x04

// CRC-8 con el polinomio 0x07 (el "CRC-8/ATM" clasico). Sin tabla: son 7
// bytes por trama a 20 Hz, el bucle no se nota y ahorra 256 bytes de flash.
static uint8_t crc8(const uint8_t *datos, size_t n) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < n; i++) {
    crc ^= datos[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// ===== PUERTO SERIE DE MANDO =====
// GPIO20 y GPIO21 son el UART0 del C3. Se usan para hablar con la Jetson.
//
// Los logs NO pueden compartir estos pines: irian a parar al parser de la
// Jetson mezclados con las tramas. Van por el USB, que en el C3 es un
// periferico aparte. En el IDE hay que dejar "USB CDC On Boot: Enabled"
// para que `Serial` sea el USB y no este mismo UART.
#define MANDO_RX_PIN 20
#define MANDO_TX_PIN 21
const uint32_t MANDO_BAUD = 115200;
HardwareSerial MandoUart(1);

// MANDO POR USB - solo para probar en el banco.
//
// Con esto en 1 las tramas viajan por el MISMO USB con el que se flashea, y
// no hace falta cablear nada: la Jetson abre /dev/ttyACM0 y manda. Sirve
// para verificar el protocolo antes de tener los tres hilos puestos.
//
// En 0 (el modo normal, el del robot montado) las tramas van por GPIO20/21
// y el USB queda libre para los logs.
//
// (!) Con MANDO_POR_USB en 1 los logs y las tramas COMPARTEN el USB. El
// receptor de la Jetson resincroniza por magic y descarta lo que no cuadre,
// asi que funciona, pero se ven tramas descartadas por CRC cada vez que se
// imprime una linea. Es esperado en este modo y NO significa cable malo.
#define MANDO_POR_USB 0

#if MANDO_POR_USB
  #define Mando Serial
#else
  #define Mando MandoUart
#endif

// Ultimo comando ya aplicado
float lastV = 0, lastOmega = 0;
uint8_t lastServo = 0;
bool lastLed = false;
uint16_t lastSeq = 0;
bool enlaceVivo = false;

// Contador de tramas descartadas por CRC. Se imprime junto a la traza de
// motores: unas pocas al arrancar son normales (el bootloader escribe por
// estos pines), pero un contador que sube sin parar mientras se conduce
// significa cable, masa o baudios mal.
unsigned long tramasMalas = 0;

// =============================================
// ===== CINEMÁTICA MECANUM + MOTORES (TA6586, sin encoder, sin PID) =====
// =============================================
// TABLA DEL TA6586:  FI=H,BI=L adelante | FI=L,BI=H reversa
//                    FI=L,BI=L libre    | FI=H,BI=H FRENADO
// La velocidad se hace con PWM sobre la entrada de dirección ACTIVA y la
// otra en LOW. Simétrico en los dos sentidos: mismo modo de decaimiento
// (impulso/rueda libre) yendo adelante y atrás, así el robot responde
// igual en ambos y la odometría comandada no queda sesgada.
//
// -------------------------------------------------------------------
// RESTRICCIÓN DE HARDWARE Y CÓMO SE RESUELVE
// Harían falta 8 salidas PWM (2 por motor) + 1 del servo = 9, y el
// ESP32-C3 tiene solo 6 canales LEDC.
//
// Solución: UN canal por motor (4 en total), que se RE-APUNTA al pin FI o
// al BI según el sentido. El cambio ocurre solo cuando el motor invierte
// la marcha —no en cada ciclo de PWM—, y se hace con el duty en 0, así que
// no genera glitches. Quedan 4 canales + 1 del servo = 5 de 6.
//
// En diferencial los dos motores de un lado reciben SIEMPRE lo mismo, así
// que se les podría compartir un solo canal y bajar a 2. No se hace: con
// un canal por motor cada uno conserva su propio `INVERT` y su propio
// arranque por pulsos, y sobran canales igual.
// (`enum RuedaCanal` y `struct Rueda` se definen mas arriba, antes de la
//  primera funcion del archivo — ver el comentario alli.)

// El lado IZQUIERDO arranca primero (desfase 0) y el DERECHO espera
// KICK_DESFASE_MS. Cual va primero da igual; lo que importa es que no sean
// los cuatro a la vez.
Rueda ruedaR1 = {R1_FI, R1_BI, CH_R1, INVERT_R1, KICK_DESFASE_MS, 0, 0, 0, false, 0, 0, false};
Rueda ruedaR2 = {R2_FI, R2_BI, CH_R2, INVERT_R2, KICK_DESFASE_MS, 0, 0, 0, false, 0, 0, false};
Rueda ruedaL1 = {L1_FI, L1_BI, CH_L1, INVERT_L1, 0,               0, 0, 0, false, 0, 0, false};
Rueda ruedaL2 = {L2_FI, L2_BI, CH_L2, INVERT_L2, 0,               0, 0, 0, false, 0, 0, false};
Rueda *RUEDAS[4]     = {&ruedaR1, &ruedaR2, &ruedaL1, &ruedaL2};
// Agrupadas por lado: es como manda el control diferencial.
Rueda *LADO_DER[2]   = {&ruedaR1, &ruedaR2};
Rueda *LADO_IZQ[2]   = {&ruedaL1, &ruedaL2};

// La API de LEDC cambió entre el core 2.x y el 3.x de ESP32. Se encapsula
// acá para que el resto del firmware no se entere.
static void pwmAttach(uint8_t pin, uint8_t canal) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, MOTOR_PWM_HZ, MOTOR_PWM_BITS, canal);
#else
  ledcSetup(canal, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcAttachPin(pin, canal);
#endif
}

static void pwmDetach(uint8_t pin) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(pin);
#else
  ledcDetachPin(pin);
#endif
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

static void pwmWriteCanal(uint8_t canal, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(canal, duty);
#else
  ledcWrite(canal, duty);
#endif
}

// Escribe en el hardware el duty que corresponde AHORA a esta rueda:
// durante el arranque manda el pulso a fondo (o su pausa), y el resto del
// tiempo el duty que pidió la cinemática.
static void aplicarDuty(Rueda &r) {
  if (r.sentido == 0) {                // libre: las dos entradas en bajo
    digitalWrite(r.pinFI, LOW);
    digitalWrite(r.pinBI, LOW);
    r.dcCompleto = false;
    return;
  }
  int duty = r.dutyPedido;
  if (r.kickRestantes > 0) {
    // Arranque: pulso a fondo o su pausa. Tiene prioridad sobre todo.
    duty = r.kickAlto ? PWM_MAX : 0;
  } else if (giroActivo && giroAlto) {
    // Giro: golpe a fondo por encima del duty pedido. En la fase baja se
    // deja r.dutyPedido tal cual, para que el robot no frene en cada hueco.
    duty = PWM_MAX;
  }

  // ── DUTY MAXIMO = CORRIENTE CONTINUA, NO "PWM AL 255" ──────────────────
  // Con LEDC a 8 bits, escribir 255 NO deja el pin en alto: deja 255 de
  // cada 256 cuentas en alto y una en bajo. En media es el 99,6% —
  // despreciable— pero el driver SIGUE CONMUTANDO 1000 veces por segundo, y
  // en el TA6586 cada conmutacion cuesta par (ver la nota de MOTOR_PWM_HZ).
  // O sea que el "a fondo" nunca era a fondo del todo, justo en los
  // instantes en que mas par hace falta: los pulsos de arranque y los de
  // giro, que son exactamente los que piden PWM_MAX.
  //
  // Cuando el duty pedido es el maximo, se suelta el canal LEDC y el pin se
  // maneja como salida digital en HIGH. Ahi el puente conduce entero, sin
  // una sola conmutacion, y entrega toda la tension de la bateria.
  uint8_t pinActivo = (r.sentido > 0) ? r.pinFI : r.pinBI;
  if (duty >= PWM_MAX) {
    if (!r.dcCompleto) {
      pwmDetach(pinActivo);            // deja el pin como salida en LOW
      r.dcCompleto = true;
    }
    digitalWrite(pinActivo, HIGH);
  } else {
    if (r.dcCompleto) {
      pwmAttach(pinActivo, r.canal);   // vuelve al PWM
      r.dcCompleto = false;
    }
    pwmWriteCanal(r.canal, duty);
  }
}

// duty 0..PWM_MAX, sentido +1 adelante / -1 atrás / 0 libre
void setRueda(Rueda &r, int duty, int sentido) {
  duty = constrain(duty, 0, PWM_MAX);
  if (r.invertida) sentido = -sentido;
  if (duty == 0) sentido = 0;

  if (sentido != r.sentido) {
    // Se apaga ANTES de mover el canal de pin: si no, el pin viejo podría
    // quedar en alto junto con el nuevo y eso en el TA6586 es FRENADO.
    if (r.sentido != 0) {
      // Si venia en DC completo el pin ya no cuelga del canal LEDC, asi que
      // escribir 0 en el canal no lo bajaria: hay que bajarlo a mano.
      if (r.dcCompleto) {
        digitalWrite(r.sentido > 0 ? r.pinFI : r.pinBI, LOW);
        r.dcCompleto = false;
      } else {
        pwmWriteCanal(r.canal, 0);
      }
      pwmDetach(r.sentido > 0 ? r.pinFI : r.pinBI);
    }
    if (sentido != 0) {
      uint8_t pinPasivo = (sentido > 0) ? r.pinBI : r.pinFI;
      pinMode(pinPasivo, OUTPUT);
      digitalWrite(pinPasivo, LOW);
      pwmAttach(sentido > 0 ? r.pinFI : r.pinBI, r.canal);

      // ARRANQUE POR PULSOS: solo al empezar a moverse tras estar quieta
      // el tiempo suficiente. Invertir la marcha de golpe (+1 -> -1, sin
      // pasar por 0) también recibe pulsos, y está bien: hay que frenar la
      // inercia y volver a vencer el rozamiento estático en el otro
      // sentido, que es el caso MÁS exigente de todos.
      if (KICK_PULSOS > 0 && millis() - r.quietaDesde >= KICK_REARME_MS) {
        r.kickRestantes = KICK_PULSOS;
        // Se arranca en fase BAJA durante el desfase de esta rueda, y
        // updateKicks() la pasa a alta cuando le toca. Para el lado que va
        // primero el desfase es 0, asi que su pulso empieza en la vuelta
        // siguiente del loop, practicamente al instante.
        //
        // Importante: durante la espera el duty es 0, NO el que pidio la
        // cinematica. Un lado tirando a media potencia mientras el otro
        // espera haria girar el robot al arrancar.
        r.kickAlto      = false;
        r.kickFaseHasta = millis() + r.kickDesfaseMs;
      }
    } else {
      r.kickRestantes = 0;             // se detuvo: no queda arranque pendiente
      r.quietaDesde   = millis();
    }
    r.sentido = sentido;
  }

  r.dutyPedido = duty;
  aplicarDuty(r);
}

// Máquina de estados del arranque por pulsos. Va en loop(), no en
// setRueda(): los comandos llegan a 20 Hz (50 ms) y la pausa entre pulsos
// es de 30 ms, así que si el avance dependiera de que llegue un comando la
// pausa duraría 50 ms en vez de 30 y el tren de pulsos saldría deformado.
void updateKicks() {
  unsigned long ahora = millis();
  for (int i = 0; i < 4; i++) {
    Rueda &r = *RUEDAS[i];
    if (r.kickRestantes == 0) continue;
    if ((long)(ahora - r.kickFaseHasta) < 0) continue;   // fase en curso

    if (r.kickAlto) {                  // terminó un pulso -> pausa
      r.kickAlto = false;
      r.kickRestantes--;
      r.kickFaseHasta = ahora + KICK_OFF_MS;
    } else {                           // terminó la pausa -> otro pulso
      r.kickAlto = true;
      r.kickFaseHasta = ahora + KICK_ON_MS;
    }
    aplicarDuty(r);
  }
}

// Tren de pulsos SOSTENIDO mientras el comando sea de giro dominante.
//
// A diferencia de updateKicks(), que se agota tras KICK_PULSOS golpes, este
// no se acaba: el rozamiento de arrastrar cuatro ruedas de costado tampoco
// se acaba mientras dure el giro (ver la nota de PWM_MIN_GIRO).
//
// La fase alta manda PWM_MAX a las cuatro ruedas A LA VEZ, sin escalonar por
// lados. Aca no se puede escalonar: en un giro los dos lados tiran en
// sentidos OPUESTOS, asi que golpear uno solo no hace girar el robot, lo
// empuja de lado. El pico de corriente de estos golpes es, por eso, el mas
// alto del robot despues del arranque.
void updateGiro() {
  if (!giroActivo) {
    if (giroAlto) {                    // se salio del giro en fase alta
      giroAlto = false;
      for (int i = 0; i < 4; i++) aplicarDuty(*RUEDAS[i]);
    }
    return;
  }
  unsigned long ahora = millis();
  if ((long)(ahora - giroFaseHasta) < 0) return;   // fase en curso

  giroAlto      = !giroAlto;
  giroFaseHasta = ahora + (giroAlto ? GIRO_ON_MS : GIRO_OFF_MS);
  // Las ruedas que estan en su arranque por pulsos no se tocan: aplicarDuty()
  // le da prioridad al kick, que ya va a fondo.
  for (int i = 0; i < 4; i++) aplicarDuty(*RUEDAS[i]);
}

// Aplica a los DOS motores de un lado la misma velocidad de rueda.
// La fracción pedida se mapea a [PWM_MIN..PWM_MAX], no a [0..PWM_MAX]: por
// debajo de PWM_MIN el motor no vence su propio rozamiento y solo zumba,
// así que ese tramo era velocidad pedida que nunca se entregaba.
static void setLado(Rueda *lado[2], float w, float escala, float wRef) {
  float frac = constrain(fabs(w) * escala / wRef, 0.0f, 1.0f);
  // Girando el piso sube: la cinematica de un giro puro pide poca velocidad
  // de rueda, y con el piso normal salia un duty de mitad de tabla que no
  // alcanza para arrastrar las ruedas de costado (ver PWM_MIN_GIRO).
  const int piso = giroActivo ? PWM_MIN_GIRO : PWM_MIN;
  int duty = 0;
  if (frac > 0.02f) duty = piso + (int)lroundf(frac * (PWM_MAX - piso));
  int sentido = (w >= 0) ? 1 : -1;
  for (int i = 0; i < 2; i++) setRueda(*lado[i], duty, sentido);
}

// -------------------------------------------------------------------
// §CINEMÁTICA — diferencial / skid-steer (convención ROS: +x adelante,
// +omega antihorario = giro a la IZQUIERDA):
//
//   wIzq = (vx - K*omega) / R        wDer = (vx + K*omega) / R
//           con K = trocha / 2
//
// Girar a la izquierda (omega > 0) manda el lado DERECHO adelante y el
// IZQUIERDO atrás — exactamente lo que hace girarIzquierda() del sketch de
// prueba que ya anda en el robot.
//
// ⚠️ NO hay desplazamiento lateral: el robot no tiene ruedas mecanum. El
// protocolo v4 ni siquiera transporta `vy`, para no ofrecer un control
// inejecutable.
void applyMotors(float vx, float omega) {
  vx    = constrain(vx,    -V_MAX,     V_MAX);
  omega = constrain(omega, -OMEGA_MAX, OMEGA_MAX);

  float wIzq = (vx - K_ROT * omega) / WHEEL_RADIUS;
  float wDer = (vx + K_ROT * omega) / WHEEL_RADIUS;

  // NORMALIZACIÓN CON ESCALADO DINÁMICO.
  //
  // Lo obvio sería dividir siempre por la velocidad de rueda del peor caso
  // (avanzar a tope Y girar a tope a la vez). Pero eso deja al robot
  // lentísimo en el caso normal: yendo derecho a V_MAX no hay nada que
  // reservar, porque omega vale 0.
  //
  // En vez de eso: la referencia es la rueda a fondo, y SOLO si un lado se
  // pasa se escalan LOS DOS por el mismo factor. Escalar por igual conserva
  // la relación entre lados, así que el radio de giro no se deforma — el
  // robot va más lento pero describe la misma curva.
  const float wRef = V_MAX / WHEEL_RADIUS;
  float wPico = fmaxf(fabs(wIzq), fabs(wDer));
  float escala = 1.0f;
  if (wPico > wRef && wPico > 0.0f) escala = wRef / wPico;

  // ¿ES UN GIRO DOMINANTE? Se compara cuanto pesa la rotacion contra cuanto
  // pesa el avance, las dos ya en velocidad de rueda (m/s en la llanta), que
  // es la unica forma de compararlas: omega esta en rad/s y vx en m/s.
  // Tiene que decidirse ANTES de setLado(), porque setLado() consulta
  // giroActivo para elegir el piso de duty.
  bool giroAntes = giroActivo;
  giroActivo = (fabsf(K_ROT * omega) >= GIRO_UMBRAL * fabsf(vx))
               && (fabsf(omega) > 0.05f);
  if (giroActivo && !giroAntes) {
    // Entrada al giro: se empieza por la fase de golpe, que es la que rompe
    // la inercia. Empezar por la baja desperdiciaria los primeros 90 ms
    // justo cuando mas falta hace.
    giroAlto      = true;
    giroFaseHasta = millis() + GIRO_ON_MS;
  }

  setLado(LADO_IZQ, wIzq, escala, wRef);
  setLado(LADO_DER, wDer, escala, wRef);
}

void stopMotors() {
  // Salir del modo giro ANTES de parar: si no, un failsafe en plena fase
  // alta dejaria giroAlto en true y el proximo comando de marcha recta
  // arrancaria con un golpe a fondo que nadie pidio.
  giroActivo = false;
  giroAlto   = false;
  for (int i = 0; i < 4; i++) setRueda(*RUEDAS[i], 0, 0);
}

// =============================================
// ===== ESCANEO AUTOMÁTICO (barrido del servo + LED) =====
// =============================================
// Mientras dura el barrido el SERVO lo manda esta rutina y se ignora el
// `servo_angle` del comando, para que un barrido en curso no se corte a
// mitad porque el operador mueva el slider.
//
// El FOCO ya no. Antes el barrido lo encendia a la fuerza y lo restauraba al
// terminar, y bastaba con que el barrido no llegara a terminar —failsafe,
// reinicio, enlace caido— para que el foco se quedara encendido sin que
// nadie lo hubiera pedido. Un actuador de potencia no debe tener ninguna via
// de encendido automatico: el foco obedece SOLO al comando de la Jetson.
void startScan() {
  scanning = true;
  scanStartMillis = millis();
}

void updateScan() {
  if (!scanning) return;
  unsigned long elapsed = millis() - scanStartMillis;
  if (elapsed >= SCAN_SWEEP_MS) {
    scanning = false;
    setServo(SERVO_PARK_ANGLE);
    return;
  }
  setServo(map(elapsed, 0, SCAN_SWEEP_MS, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE));
}

// =============================================
// ===== ENTRAR EN FAILSAFE =====
// =============================================
void enterFailsafe(const char *motivo) {
  inFailsafe = true;
  lastV = 0;
  lastOmega = 0;
  stopMotors();
  // se corta el barrido y se APAGA el foco. Apagarlo es la unica accion
  // automatica que se le permite al foco, y es la segura: ahorra bateria y
  // hace visible la perdida de enlace. El servo se deja DONDE ESTÁ: si se
  // perdió la comunicación, mover la cámara a ciegas puede golpear algo —
  // detenerse es más seguro que reposicionar sin supervisión.
  scanning = false;
  setLed(false);
  lastLed = false;
  Serial.print("⚠️  FAILSAFE: ");
  Serial.println(motivo);
}







// =============================================
// ===== ENLACE SERIE CON LA JETSON =====
// =============================================
// Maquina de estados byte a byte. NO se usa readBytes() con timeout: eso
// bloquearia el loop y con el los motores. Se consume lo que haya llegado y
// se sigue de largo; si la trama esta a medias se retoma en la vuelta
// siguiente.
//
// La resincronizacion es lo que hace que el mensaje de arranque del
// bootloader, o cualquier byte perdido por ruido, no dejen el enlace roto
// para siempre: todo lo que no encaje se descarta hasta el proximo magic.
void procesarSerie() {
  static uint8_t buf[sizeof(CmdFrame)];
  static uint8_t n = 0;

  while (Mando.available() > 0) {
    uint8_t b = (uint8_t)Mando.read();

    if (n == 0) {
      if (b != CMD_MAGIC) continue;   // basura: se tira hasta el magic
      buf[n++] = b;
      continue;
    }

    buf[n++] = b;
    if (n < sizeof(CmdFrame)) continue;

    n = 0;
    CmdFrame c;
    memcpy(&c, buf, sizeof(c));
    if (crc8(buf, sizeof(CmdFrame) - 1) != c.crc) {
      // Trama corrupta. NO se aplica y NO se refresca el watchdog: si el
      // cable falla de verdad, el robot tiene que parar, no seguir con el
      // ultimo comando bueno.
      tramasMalas++;
      continue;
    }

    lastCmdMillis = millis();
    enlaceVivo = true;
    lastSeq = c.seq;

    lastV     = (constrain((int)c.v_pct,     -100, 100) / 100.0f) * V_MAX;
    lastOmega = (constrain((int)c.omega_pct, -100, 100) / 100.0f) * OMEGA_MAX;
    lastServo = c.servo_angle;
    lastLed   = (c.flags & FLAG_LED) != 0;

    inFailsafe = false;
    applyMotors(lastV, lastOmega);
    if ((c.flags & FLAG_SCAN) && !scanning) startScan();
    // El foco se aplica SIEMPRE, tambien durante un barrido: es lo unico
    // que garantiza que se pueda apagar en cualquier momento. Solo el servo
    // queda tomado por el barrido mientras dura.
    setLed(lastLed);
    if (!scanning) {
      setServo(lastServo);
    }
  }
}

void enviarEstado() {
  StatusFrame st;
  st.magic = STATUS_MAGIC;
  st.v_pct = (int8_t)constrain((int)lroundf(lastV / V_MAX * 100.0f), -100, 100);
  st.omega_pct = (int8_t)constrain((int)lroundf(lastOmega / OMEGA_MAX * 100.0f), -100, 100);
  st.servo_angle = (uint8_t)currentServoAngle;
  st.flags = 0;
  if (ledOn)      st.flags |= ST_LED;
  if (scanning)   st.flags |= ST_SCANNING;
  if (inFailsafe) st.flags |= ST_FAILSAFE;
  st.seq_echo = lastSeq;
  st.crc = crc8((uint8_t *)&st, sizeof(st) - 1);
  Mando.write((uint8_t *)&st, sizeof(st));
}

// =============================================
// ===== SETUP =====
// =============================================
void setup() {
  // El foco, apagado ANTES QUE NADA — antes incluso de Serial.begin().
  // GPIO8 es pin de strapping y el arranque lo deja en alto. Este bloque
  // estaba DESPUES de la espera del USB CDC, que dura hasta 1,7 s: en cada
  // reset el foco se encendia solo durante ese rato. Apagarlo es lo primero
  // que hace la placa; los logs pueden esperar.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_NIVEL(false));
  ledOn = false;

  Serial.begin(115200);
  // Con "USB CDC On Boot" el puerto tarda en enumerarse tras el reset: sin
  // esta espera las primeras lineas (pinout, PWM, cinematica) se pierden y
  // el monitor arranca en blanco. El tope evita colgarse cuando el robot va
  // con bateria y no hay nadie escuchando al otro lado.
  unsigned long t0Serial = millis();
  while (!Serial && millis() - t0Serial < 1500) { delay(10); }
  delay(200);
  // Sello de compilacion. Existe porque el 2026-09-06 los sintomas no
  // cambiaban tras flashear y no habia forma de distinguir "el arreglo no
  // sirve" de "la placa sigue con el firmware viejo". __DATE__ y __TIME__
  // los pone el compilador: si esta linea no coincide con el momento en que
  // compilaste, lo que corre en la placa es otra cosa.
  Serial.println();
  Serial.printf("=== AGROBOT C3 — compilado el %s a las %s ===\n",
                __DATE__, __TIME__);

  // ── AUTOPRUEBA DEL FOCO ──────────────────────────────────────────────────
  // Existe porque la polaridad se ha diagnosticado mal dos veces (una en cada
  // sentido) y no hay forma de deducirla desde el codigo: depende de como este
  // montada la etapa de potencia. Esto lo convierte en una medida, no en una
  // suposicion.
  //
  // Se conduce el pin a los dos estados logicos anunciando cada uno por serie.
  // MIRA LA LAMPARA y compara con lo que dice la linea:
  //   · si coinciden, LED_ACTIVO_EN_BAJO esta bien
  //   · si van al reves, cambia LED_ACTIVO_EN_BAJO al valor contrario
  //     (esta arriba del todo, es la UNICA linea que hay que tocar)
  //
  // Se puede desactivar poniendo FOCO_AUTOPRUEBA en 0 cuando ya este ajustado.
  #define FOCO_AUTOPRUEBA 1
#if FOCO_AUTOPRUEBA
  Serial.printf("[foco] autoprueba — LED_ACTIVO_EN_BAJO = %s\n",
                LED_ACTIVO_EN_BAJO ? "true" : "false");
  for (int i = 0; i < 2; i++) {
    Serial.println("[foco] deberia estar ENCENDIDO ahora (2 s)");
    digitalWrite(LED_PIN, LED_NIVEL(true));
    delay(2000);
    Serial.println("[foco] deberia estar APAGADO ahora (2 s)");
    digitalWrite(LED_PIN, LED_NIVEL(false));
    delay(2000);
  }
  ledOn = false;
  Serial.println("[foco] fin de la autoprueba, queda apagado");
#endif

  // Las 8 entradas de driver a OUTPUT/LOW ANTES de nada: una entrada
  // flotando puede hacer arrancar el motor sola mientras arranca la placa.
  const int pinesMotor[] = {R1_FI, R1_BI, R2_FI, R2_BI,
                            L1_FI, L1_BI, L2_FI, L2_BI};
  for (int p : pinesMotor) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  // Los canales LEDC no se enganchan acá: setRueda() apunta cada canal al
  // pin FI o BI la primera vez que esa rueda se mueve, según el sentido.
  stopMotors();

  Serial.print("[PWM] motores a ");
  Serial.print(MOTOR_PWM_HZ);
  Serial.print(" Hz, ");
  Serial.print(MOTOR_PWM_BITS);
  Serial.println(" bits, 1 canal LEDC por motor (4 en total)");
  Serial.println("[CINEMATICA] diferencial / skid-steer: vx, omega");
  Serial.print("[GIRO] piso PWM_MIN_GIRO=");
  Serial.print(PWM_MIN_GIRO);
  Serial.print(", pulsos ");
  Serial.print(GIRO_ON_MS);
  Serial.print("/");
  Serial.print(GIRO_OFF_MS);
  Serial.println(" ms mientras el giro domine");

  Serial.print("[ARRANQUE] zona muerta PWM_MIN=");
  Serial.print(PWM_MIN);
  Serial.print(", pulsos de torque: ");
  Serial.print(KICK_PULSOS);
  Serial.print(" x ");
  Serial.print(KICK_ON_MS);
  Serial.println(" ms");

  stopMotors();

  // ── SERVO ────────────────────────────────────────────────────────────────
  // Enganche directo por LEDC. Arriba, en el bloque de constantes, esta el
  // detalle de por que no se usa ESP32Servo y de donde salen 50 Hz y 14 bits.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  servoListo = ledcAttachChannel(SERVO_PIN, SERVO_LEDC_HZ, SERVO_LEDC_BITS,
                                 SERVO_LEDC_CANAL);
#else
  servoListo = (ledcSetup(SERVO_LEDC_CANAL, SERVO_LEDC_HZ, SERVO_LEDC_BITS) != 0);
  if (servoListo) ledcAttachPin(SERVO_PIN, SERVO_LEDC_CANAL);
#endif
  if (servoListo) {
    Serial.printf("[servo] OK — GPIO%d, canal LEDC %d, %d Hz, %d bits\n",
                  SERVO_PIN, SERVO_LEDC_CANAL, SERVO_LEDC_HZ, SERVO_LEDC_BITS);
  } else {
    Serial.printf("[servo] ⚠️  FALLO al enganchar GPIO%d en LEDC. El servo no se "
                  "movera. Sube Core Debug Level a Error para ver el motivo.\n",
                  SERVO_PIN);
  }
  servoEscribir(SERVO_PARK_ANGLE);
  currentServoAngle = SERVO_PARK_ANGLE;


  // El puerto de mando. En el modo normal los logs siguen yendo por USB
  // (`Serial`), que es un periferico distinto: hay que compilar con
  // "USB CDC On Boot: Enabled".
#if MANDO_POR_USB
  Serial.println("[serie] MODO BANCO: mando por USB, mezclado con los logs");
#else
  MandoUart.begin(MANDO_BAUD, SERIAL_8N1, MANDO_RX_PIN, MANDO_TX_PIN);
  Serial.printf("[serie] mando por UART1 a %lu baudios (RX=GPIO%d, TX=GPIO%d)\n",
                (unsigned long)MANDO_BAUD, MANDO_RX_PIN, MANDO_TX_PIN);
#endif

  lastCmdMillis = millis();
}

// =============================================
// ===== LOOP (no bloqueante, alta frecuencia) =====
// =============================================
unsigned long ultimoEstado = 0;
const unsigned long ESTADO_PERIODO_MS = 100;  // 10 Hz hacia la Jetson

void loop() {

  // --- comandos de la Jetson por el UART (unica via de mando) ---
  procesarSerie();

  // --- watchdog del enlace serie ---
  // Si la Jetson deja de escribir, el robot para. No hay arbitraje que hacer
  // porque ya no hay dos fuentes compitiendo.
  if (!inFailsafe && millis() - lastCmdMillis > CMD_TIMEOUT_MS) {
    enterFailsafe("sin comando de la Jetson por el serie");
    enlaceVivo = false;
  }

  updateScan();
  updateKicks();   // arranque por pulsos, sin bloquear el loop
  updateGiro();    // tren de pulsos sostenido mientras se gira

  // --- TRAZA DE MOTORES ---
  // Imprime a 2 Hz mientras haya mando o movimiento, y una sola vez al
  // quedar todo a cero. Separa los tres fallos que dan el mismo sintoma (las
  // ruedas no giran): mando que no llega, failsafe que corta, o duty que si
  // sale y entonces el problema es del driver o del cableado. Este ultimo
  // caso ya se dio una vez: un GND suelto del driver (2026-09-04).
  {
    static unsigned long ultimaTraza = 0;
    static bool huboActividad = false;
    float vT = lastV;
    float wT = lastOmega;
    bool  activo = (fabsf(vT) > 0.001f) || (fabsf(wT) > 0.001f);
    for (int i = 0; i < 4 && !activo; i++) activo = (RUEDAS[i]->sentido != 0);

    if ((activo || huboActividad) && millis() - ultimaTraza >= 500) {
      ultimaTraza = millis();
      huboActividad = activo;
      const char *src = enlaceVivo ? "SERIE" : "SIN ENLACE";
      Serial.printf("[mot] src=%s fs=%d crc_err=%lu v=%.2f w=%.2f | R1 %d/%d R2 %d/%d L1 %d/%d L2 %d/%d\n",
                    src, inFailsafe ? 1 : 0, tramasMalas, vT, wT,
                    ruedaR1.sentido, ruedaR1.dutyPedido,
                    ruedaR2.sentido, ruedaR2.dutyPedido,
                    ruedaL1.sentido, ruedaL1.dutyPedido,
                    ruedaL2.sentido, ruedaL2.dutyPedido);
    }
  }

  // --- estado hacia la Jetson (10 Hz, no en cada vuelta del loop) ---
  if (millis() - ultimoEstado >= ESTADO_PERIODO_MS) {
    ultimoEstado = millis();
    enviarEstado();
  }

  // sin delay() bloqueante: el loop corre lo más rápido posible para
  // minimizar la latencia de reacción a un nuevo paquete
}


