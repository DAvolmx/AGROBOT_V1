/*
 * AGROBOT — SKETCH DE PRUEBA DE MOTORES (no es el firmware de producción)
 * =======================================================================
 * Sirve para verificar el CABLEADO antes de cargar esp32c3_actuator.ino.
 * A propósito NO tiene BLE, ni UDP, ni el protocolo binario: si algo no
 * anda acá, el problema está en los motores o el cableado, no en el
 * sistema. Lo único que sí trae es WiFi, para poder probar desde el
 * celular en vez de estar atado al cable del monitor serie.
 *
 * ⚠️  LEVANTÁ EL ROBOT ANTES DE PROBAR — las ruedas tienen que girar en el
 *     aire. Los botones mueven los motores a velocidad plena.
 *
 * ---------------------------------------------------------------------
 * CÓMO SE USA
 *   1. Cargar con la placa "ESP32C3 Dev Module".
 *   2. Abrir el Monitor Serie a 115200 -> imprime la IP que le tocó.
 *   3. Desde un celular o PC EN LA MISMA RED, abrir esa IP en el navegador.
 *   4. Mantener presionado un botón para mover; al soltar, frena.
 *
 * También sigue andando el menú por Serial (mandá '?' para la ayuda), por
 * si el WiFi no conecta.
 * ---------------------------------------------------------------------
 * SEGURIDAD — WATCHDOG DE LA INTERFAZ WEB
 * Un control remoto tiene un problema que el monitor serie no tiene: si se
 * corta el WiFi, se cierra el navegador o se traba el celular con un motor
 * andando, el motor SIGUE GIRANDO para siempre.
 * Por eso la página manda el comando repetido mientras tenés el botón
 * apretado, y este firmware frena solo si no llega uno nuevo en
 * WEB_TIMEOUT_MS. Es el mismo criterio que usa el firmware de producción.
 * ---------------------------------------------------------------------
 *
 * Pinout (igual al firmware real, del esquemático de la PCB):
 *     m1 = GPIO0 (FI) / GPIO1 (BI)      m3 = GPIO4 / GPIO5
 *     m2 = GPIO2 (FI) / GPIO3 (BI)      m4 = GPIO6 / GPIO7
 *
 * Drivers TA6586:  FI=H,BI=L -> adelante | FI=L,BI=H -> reversa
 *                  FI=L,BI=L -> libre    | FI=H,BI=H -> FRENADO
 */

#include <WiFi.h>
#include <WebServer.h>

// =============================================
// ===== RED =====
// =============================================
// Red normal de la casa/taller, NO el AP de la Jetson: este sketch es para
// probar en el banco, con el celular conectado a la misma red.
#define WIFI_SSID     "LoRa_Wan"
#define WIFI_PASSWORD "12345678"

WebServer server(80);

// Si no llega un comando nuevo en este tiempo, se frena solo (ver §SEGURIDAD)
const unsigned long WEB_TIMEOUT_MS = 700;
unsigned long ultimoComandoWeb = 0;
bool moviendoPorWeb = false;

// =============================================
// ===== PINES (del esquemático de la PCB) =====
// =============================================
#define M1_FI 0
#define M1_BI 1
#define M2_FI 2
#define M2_BI 3
#define M3_FI 4
#define M3_BI 5
#define M4_FI 6
#define M4_BI 7

// Reparto que ASUME el firmware real. Esto es justamente lo que venís a
// confirmar con esta prueba.
const int IZQ[2][2] = { {M1_FI, M1_BI}, {M2_FI, M2_BI} };  // m1, m2
const int DER[2][2] = { {M3_FI, M3_BI}, {M4_FI, M4_BI} };  // m3, m4

const int TODOS[4][2] = {
  {M1_FI, M1_BI}, {M2_FI, M2_BI}, {M3_FI, M3_BI}, {M4_FI, M4_BI}
};

const unsigned long PULSO_MS = 700;   // duración de cada movimiento (Serial)
const unsigned long PAUSA_MS = 500;

// =============================================
// ===== CONTROL BÁSICO (sin PWM: solo on/off) =====
// =============================================
// La identificación usa digitalWrite puro. Es lo más simple posible y no
// depende de LEDC ni de la versión del core: si un motor no se mueve acá,
// es cableado o alimentación, punto.
void motorLibre(int fi, int bi)    { digitalWrite(fi, LOW);  digitalWrite(bi, LOW); }
void motorAdelante(int fi, int bi) { digitalWrite(bi, LOW);  digitalWrite(fi, HIGH); }
void motorReversa(int fi, int bi)  { digitalWrite(fi, LOW);  digitalWrite(bi, HIGH); }

// sentido: 1 adelante, -1 reversa, 0 libre
void motorSentido(const int m[2], int sentido) {
  if (sentido > 0)      motorAdelante(m[0], m[1]);
  else if (sentido < 0) motorReversa(m[0], m[1]);
  else                  motorLibre(m[0], m[1]);
}

void pararTodo() {
  for (auto &m : TODOS) motorLibre(m[0], m[1]);
  moviendoPorWeb = false;
}

// =============================================
// ===== PRUEBAS AUTOMÁTICAS (menú por Serial) =====
// =============================================
void probarMotor(int n) {
  const int *m = TODOS[n - 1];
  Serial.printf("\n== MOTOR m%d  (FI=GPIO%d, BI=GPIO%d) ==\n", n, m[0], m[1]);

  Serial.println("   -> ADELANTE");
  motorAdelante(m[0], m[1]);
  delay(PULSO_MS);
  motorLibre(m[0], m[1]);
  delay(PAUSA_MS);

  Serial.println("   -> REVERSA");
  motorReversa(m[0], m[1]);
  delay(PULSO_MS);
  motorLibre(m[0], m[1]);

  Serial.println("   Anotá: ¿se movió? ¿en qué sentido físico giró primero?");
}

// El array va por REFERENCIA (&lado)[2][2], no como `const int lado[2][2]`:
// escrito de la segunda forma decae a puntero, se pierde el tamaño y el
// `for` por rango de abajo ni siquiera compila.
void probarLado(const int (&lado)[2][2], const char *nombre) {
  Serial.printf("\n== LADO %s (los dos motores juntos) ==\n", nombre);
  Serial.println("   -> ADELANTE");
  for (auto &m : lado) motorAdelante(m[0], m[1]);
  delay(PULSO_MS * 2);
  for (auto &m : lado) motorLibre(m[0], m[1]);
  delay(PAUSA_MS);

  Serial.println("   -> REVERSA");
  for (auto &m : lado) motorReversa(m[0], m[1]);
  delay(PULSO_MS * 2);
  for (auto &m : lado) motorLibre(m[0], m[1]);
  Serial.println("   Los DOS motores de este lado deben girar para el MISMO lado.");
  Serial.println("   Si uno va al reves, tiene FI/BI cruzados en el conector.");
}

void probarTodosAdelante() {
  Serial.println("\n== LOS CUATRO ADELANTE ==");
  for (auto &m : TODOS) motorAdelante(m[0], m[1]);
  delay(PULSO_MS * 2);
  pararTodo();
}

void probarGiro() {
  Serial.println("\n== GIRO EN EL LUGAR ==");
  for (auto &m : IZQ) motorReversa(m[0], m[1]);
  for (auto &m : DER) motorAdelante(m[0], m[1]);
  delay(PULSO_MS * 2);
  pararTodo();
}

// =============================================
// ===== PRUEBA DE PWM (mismo esquema que el firmware real) =====
// =============================================
// El firmware de producción comparte UN canal LEDC entre los dos motores de
// cada lado, porque el ESP32-C3 solo tiene 6 canales y harían falta 9.
// Esta prueba usa exactamente ese esquema para confirmar que funciona.
const uint32_t PWM_HZ   = 8000;
const uint8_t  PWM_BITS = 8;
const int      PWM_MAX  = 255;

enum { CH_IZQ_FI = 0, CH_IZQ_BI = 1, CH_DER_FI = 2, CH_DER_BI = 3 };
bool pwmListo = false;
volatile bool abortarRampa = false;

static void pwmAttach(uint8_t pin, uint8_t canal) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, PWM_HZ, PWM_BITS, canal);
#else
  ledcSetup(canal, PWM_HZ, PWM_BITS);
  ledcAttachPin(pin, canal);
#endif
}

static void pwmWriteCanal(uint8_t canal, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(canal, duty);
#else
  ledcWrite(canal, duty);
#endif
}

void iniciarPwm() {
  if (pwmListo) return;
  pwmAttach(M1_FI, CH_IZQ_FI);  pwmAttach(M2_FI, CH_IZQ_FI);
  pwmAttach(M1_BI, CH_IZQ_BI);  pwmAttach(M2_BI, CH_IZQ_BI);
  pwmAttach(M3_FI, CH_DER_FI);  pwmAttach(M4_FI, CH_DER_FI);
  pwmAttach(M3_BI, CH_DER_BI);  pwmAttach(M4_BI, CH_DER_BI);
  pwmListo = true;
  Serial.println("   [LEDC: 4 canales, 2 pines por canal]");
}

void probarRampa() {
  Serial.println("\n== RAMPA DE PWM (0 -> 100% -> 0, hacia adelante) ==");
  iniciarPwm();
  abortarRampa = false;
  pwmWriteCanal(CH_IZQ_BI, 0);
  pwmWriteCanal(CH_DER_BI, 0);

  for (int d = 0; d <= PWM_MAX && !abortarRampa; d += 5) {
    pwmWriteCanal(CH_IZQ_FI, d);
    pwmWriteCanal(CH_DER_FI, d);
    // se atiende la web DURANTE la rampa: si no, el STOP no llegaría
    // hasta que la rampa terminara sola
    server.handleClient();
    delay(25);
  }
  for (int d = PWM_MAX; d >= 0 && !abortarRampa; d -= 5) {
    pwmWriteCanal(CH_IZQ_FI, d);
    pwmWriteCanal(CH_DER_FI, d);
    server.handleClient();
    delay(25);
  }
  pwmWriteCanal(CH_IZQ_FI, 0);
  pwmWriteCanal(CH_DER_FI, 0);
  if (abortarRampa) Serial.println("   (rampa cortada por el operador)");
  Serial.println("   Si arranca recien pasada la mitad, el motor necesita");
  Serial.println("   mas tension de arranque: normal en DC chicos.");
}

// Después de usar PWM, los pines quedan tomados por LEDC y digitalWrite ya
// no los controla. Esto los devuelve a modo digital para que las pruebas
// de identificación sigan funcionando.
void soltarPwm() {
  if (!pwmListo) return;
  for (auto &m : TODOS) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcDetach(m[0]);
    ledcDetach(m[1]);
#else
    ledcDetachPin(m[0]);
    ledcDetachPin(m[1]);
#endif
    pinMode(m[0], OUTPUT);
    pinMode(m[1], OUTPUT);
    digitalWrite(m[0], LOW);
    digitalWrite(m[1], LOW);
  }
  pwmListo = false;
}

// =============================================
// ===== PÁGINA WEB =====
// =============================================
// Sin CDN ni dependencias externas: tiene que abrir aunque la red no tenga
// salida a internet (igual que los dashboards del proyecto).
const char PAGINA[] PROGMEM = R"HTML(<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>AgroBot - Prueba de motores</title><style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:#0e1210;color:#e6ebe7;font-family:system-ui,sans-serif;padding:12px}
h1{font-size:16px;margin:0 0 2px;color:#4be277}
.sub{font-size:11px;color:#8e9a93;margin-bottom:12px}
.aviso{background:#3a3218;border:1px solid #fbbf24;color:#ffe4a3;border-radius:8px;
       padding:10px;font-size:12px;line-height:1.5;margin-bottom:12px}
.card{background:#161b18;border:1px solid #2b342e;border-radius:10px;padding:12px;margin-bottom:10px}
.card h2{font-size:11px;text-transform:uppercase;letter-spacing:.06em;color:#8e9a93;margin:0 0 10px}
.fila{display:flex;gap:8px}
button{flex:1;padding:16px 8px;border-radius:9px;border:1px solid #2b342e;background:#1e2521;
       color:#e6ebe7;font-size:15px;font-family:inherit;cursor:pointer;touch-action:none;user-select:none}
button:active,button.on{background:#16311f;border-color:#4be277;color:#4be277}
.stop{background:#3a1a1a;border-color:#ff5c5c;color:#ff5c5c;font-weight:700}
.grande{width:100%;padding:20px;font-size:17px;margin-top:4px}
.est{font-size:11px;color:#8e9a93;margin-top:8px;min-height:15px}
</style></head><body>
<h1>AgroBot &middot; Prueba de motores</h1>
<div class="sub">manten&eacute; presionado para mover &middot; al soltar, frena</div>
<div class="aviso"><b>Levant&aacute; el robot.</b> Las ruedas tienen que girar en el aire:
los botones mueven los motores a velocidad plena.</div>
<div id="motores"></div>
<div class="card"><h2>Lados completos</h2>
<div class="fila" style="margin-bottom:8px">
<button data-u="/side?s=L&d=-1">&#9664; IZQ</button><button data-u="/side?s=L&d=1">IZQ &#9654;</button></div>
<div class="fila">
<button data-u="/side?s=R&d=-1">&#9664; DER</button><button data-u="/side?s=R&d=1">DER &#9654;</button></div></div>
<div class="card"><h2>Movimiento completo</h2>
<div class="fila" style="margin-bottom:8px">
<button data-u="/all?d=1">&#9650; Adelante</button><button data-u="/all?d=-1">&#9660; Atr&aacute;s</button></div>
<div class="fila">
<button data-u="/turn?d=1">&#8634; Giro izq</button><button data-u="/turn?d=-1">Giro der &#8635;</button></div></div>
<div class="card"><h2>PWM (esquema del firmware real)</h2>
<button class="grande" id="rampa">Rampa 0 &rarr; 100% &rarr; 0</button>
<div class="est" id="est-rampa"></div></div>
<button class="stop grande" id="stop">&#9209; PARAR TODO</button>
<div class="est" id="est"></div>
<script>
var cont=document.getElementById('motores');
for(var i=1;i<=4;i++){
  cont.insertAdjacentHTML('beforeend',
   '<div class="card"><h2>Motor m'+i+'</h2><div class="fila">'+
   '<button data-u="/m?i='+i+'&d=-1">&#9664; Rev</button>'+
   '<button data-u="/m?i='+i+'&d=1">Adel &#9654;</button></div></div>');
}
var est=document.getElementById('est'),timer=null,activo=null;
function pedir(u){return fetch(u).catch(function(){est.textContent='sin conexion con el robot';});}
// Mientras el boton esta apretado se repite el comando. Si el navegador se
// cierra o se corta el WiFi, dejan de llegar y el firmware frena solo.
function apretar(b){
  activo=b;b.classList.add('on');
  pedir(b.dataset.u);est.textContent='moviendo: '+b.textContent.trim();
  clearInterval(timer);timer=setInterval(function(){pedir(b.dataset.u);},200);
}
function soltar(){
  clearInterval(timer);timer=null;
  if(activo){activo.classList.remove('on');activo=null;}
  pedir('/stop');est.textContent='parado';
}
document.querySelectorAll('button[data-u]').forEach(function(b){
  b.addEventListener('pointerdown',function(e){e.preventDefault();apretar(b);});
  ['pointerup','pointerleave','pointercancel'].forEach(function(ev){
    b.addEventListener(ev,function(e){e.preventDefault();soltar();});});
});
document.getElementById('stop').addEventListener('click',function(){soltar();pedir('/stop');});
document.getElementById('rampa').addEventListener('click',function(){
  document.getElementById('est-rampa').textContent='rampa en curso (~13s)...';
  pedir('/ramp').then(function(){document.getElementById('est-rampa').textContent='rampa terminada';});
});
// Hombre muerto: si la pagina deja de estar visible, frena.
document.addEventListener('visibilitychange',function(){if(document.hidden)soltar();});
window.addEventListener('blur',soltar);
</script></body></html>)HTML";

// =============================================
// ===== HANDLERS HTTP =====
// =============================================
void marcarComandoWeb() {
  ultimoComandoWeb = millis();
  moviendoPorWeb = true;
}

void handleRaiz() { server.send_P(200, "text/html", PAGINA); }

void handleMotor() {
  soltarPwm();
  int i = server.arg("i").toInt();
  int d = server.arg("d").toInt();
  if (i < 1 || i > 4) { server.send(400, "text/plain", "motor invalido"); return; }
  motorSentido(TODOS[i - 1], d);
  marcarComandoWeb();
  server.send(200, "text/plain", "ok");
}

void handleLado() {
  soltarPwm();
  String s = server.arg("s");
  int d = server.arg("d").toInt();
  const int (&lado)[2][2] = (s == "L") ? IZQ : DER;
  for (auto &m : lado) motorSentido(m, d);
  marcarComandoWeb();
  server.send(200, "text/plain", "ok");
}

void handleTodos() {
  soltarPwm();
  int d = server.arg("d").toInt();
  for (auto &m : TODOS) motorSentido(m, d);
  marcarComandoWeb();
  server.send(200, "text/plain", "ok");
}

void handleGiro() {
  soltarPwm();
  int d = server.arg("d").toInt();   // 1 = giro a la izquierda
  for (auto &m : IZQ) motorSentido(m, -d);
  for (auto &m : DER) motorSentido(m,  d);
  marcarComandoWeb();
  server.send(200, "text/plain", "ok");
}

void handleStop() {
  abortarRampa = true;
  pararTodo();
  server.send(200, "text/plain", "parado");
}

void handleRampa() {
  // se responde ANTES de arrancar: la rampa bloquea ~13s y el navegador
  // no debería quedarse esperando la respuesta todo ese rato
  server.send(200, "text/plain", "rampa iniciada");
  probarRampa();
  soltarPwm();
}

// =============================================
// ===== AYUDA (Serial) =====
// =============================================
void ayuda() {
  Serial.println("\n---------------------------------------------------");
  Serial.println(" PRUEBA DE MOTORES AGROBOT   (robot LEVANTADO!)");
  Serial.println("---------------------------------------------------");
  Serial.println("  1 2 3 4  un motor solo (adelante y reversa)");
  Serial.println("  t        los cuatro, uno por uno");
  Serial.println("  i / d    lado izquierdo / derecho completo");
  Serial.println("  a        los cuatro adelante (deberia ir recto)");
  Serial.println("  g        giro en el lugar");
  Serial.println("  p        rampa de PWM (esquema del firmware real)");
  Serial.println("  s        parar todo");
  Serial.println("  ?        esta ayuda");
  Serial.println("---------------------------------------------------");
}

// =============================================
// ===== SETUP / LOOP =====
// =============================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Todas las entradas de driver en bajo ANTES que nada: si quedan
  // flotando, el motor puede arrancar solo mientras la placa bootea.
  for (auto &m : TODOS) {
    pinMode(m[0], OUTPUT);
    pinMode(m[1], OUTPUT);
    digitalWrite(m[0], LOW);
    digitalWrite(m[1], LOW);
  }

  Serial.printf("\nConectando a la red \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    server.on("/",     handleRaiz);
    server.on("/m",    handleMotor);
    server.on("/side", handleLado);
    server.on("/all",  handleTodos);
    server.on("/turn", handleGiro);
    server.on("/stop", handleStop);
    server.on("/ramp", handleRampa);
    server.begin();
    Serial.println("======================================");
    Serial.print("  ABRI ESTA DIRECCION EN EL CELULAR:\n    http://");
    Serial.println(WiFi.localIP());
    Serial.println("  (el celular tiene que estar en la MISMA red)");
    Serial.println("======================================");
  } else {
    // No es fatal: el menú por Serial sigue estando.
    Serial.println("WiFi NO conecto. Revisa el SSID/clave.");
    Serial.println("Podes probar igual con el menu por Serial.");
  }

  ayuda();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) server.handleClient();

  // --- watchdog de la interfaz web ---
  // Si la pagina dejo de mandar comandos (navegador cerrado, WiFi caido,
  // celular bloqueado) y hay motores andando por web, se frena solo.
  if (moviendoPorWeb && millis() - ultimoComandoWeb > WEB_TIMEOUT_MS) {
    pararTodo();
    Serial.println("[watchdog] sin comandos de la web: motores detenidos");
  }

  // --- menú por Serial ---
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;

  soltarPwm();   // las pruebas por Serial usan digitalWrite

  switch (c) {
    case '1': case '2': case '3': case '4':
      probarMotor(c - '0');
      break;
    case 't':
      for (int i = 1; i <= 4; i++) { probarMotor(i); delay(PAUSA_MS); }
      break;
    case 'i': probarLado(IZQ, "IZQUIERDO (m1+m2)"); break;
    case 'd': probarLado(DER, "DERECHO (m3+m4)");   break;
    case 'a': probarTodosAdelante(); break;
    case 'g': probarGiro();          break;
    case 'p': probarRampa(); soltarPwm(); break;
    case 's': pararTodo(); Serial.println("   [parado]"); break;
    case '?': ayuda();               break;
    default:
      Serial.printf("Tecla '%c' desconocida. Manda '?' para la ayuda.\n", c);
      return;
  }
  Serial.println("\nListo. Otra tecla, o '?' para la ayuda.");
}
