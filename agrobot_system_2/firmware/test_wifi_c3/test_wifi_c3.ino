/*
 * test_wifi_c3.ino — por que el ESP32-C3 Mini no entra en el AP de la Jetson.
 *
 * Sube ESTE sketch al C3, monitor serie a 115200. No arranca BLE ni motores
 * ni UDP: el C3 tiene UNA sola radio de 2.4 GHz compartida entre WiFi y BLE,
 * asi que con BLE activo no se puede saber si un fallo de asociacion es de
 * la radio o del reparto de tiempo entre las dos. Aqui solo hay WiFi.
 *
 * QUE SABEMOS YA, medido en el lado del AP:
 *   hostapd lleva todo el dia registrando UNA sola estacion, el ESP32-S3.
 *   Del C3 no hay ni un intento de autenticacion ni un rechazo: no aparece.
 *   El AP esta bien (WPA2-PSK/CCMP, canal 1, sin PMF, sin filtro de MAC) y
 *   otro ESP32 se asocia sin problema con la misma clave.
 *
 * Es decir: el C3 no esta siendo rechazado, es que no llega a hablar. Este
 * sketch separa las tres causas posibles, que desde fuera son identicas:
 *   a) no VE la red            -> antena / alcance / canal
 *   b) la ve y no asocia       -> clave, modo, o pila WiFi
 *   c) asocia y pierde la IP   -> WiFi.config() / DHCP
 */
#include <WiFi.h>
#include <WiFiUdp.h>

#define WIFI_SSID     "JETSON_AP"
#define WIFI_PASSWORD "12345678"

IPAddress STATIC_IP (192, 168, 12,  50);
IPAddress GATEWAY_IP(192, 168, 12,   1);
IPAddress SUBNET_MSK(255, 255, 255,  0);

// WiFi.status() devuelve numeros. Traducirlos importa: "status=1" no dice
// nada, "no se encontro la red" manda directo a mirar la antena.
const char* textoEstado(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "IDLE (aun probando)";
    case WL_NO_SSID_AVAIL:   return "NO SE ENCUENTRA LA RED (antena/alcance/canal)";
    case WL_SCAN_COMPLETED:  return "escaneo terminado";
    case WL_CONNECTED:       return "CONECTADO";
    case WL_CONNECT_FAILED:  return "FALLO LA CONEXION (clave incorrecta?)";
    case WL_CONNECTION_LOST: return "conexion perdida";
    case WL_DISCONNECTED:    return "desconectado";
    default:                 return "desconocido";
  }
}

void escanear() {
  Serial.println("\n--- 1) ¿VE el C3 la red? ---");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(200);

  int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.println("  NO SE VE NINGUNA RED.");
    Serial.println("  Ni la del vecino. Eso apunta a la radio del propio C3:");
    Serial.println("   - Muchos 'C3 SuperMini' traen la antena PCB mal");
    Serial.println("     adaptada y su alcance real es de centimetros.");
    Serial.println("     Prueba con la placa PEGADA a la Jetson.");
    Serial.println("   - Si la placa tiene conector u.FL, puede estar puesto");
    Serial.println("     el jumper hacia la antena externa y sin antena.");
    Serial.println("   - Alimentacion floja por USB: la radio necesita picos");
    Serial.println("     de ~350 mA para transmitir.");
    return;
  }

  Serial.printf("  %d redes visibles:\n", n);
  bool encontrada = false;
  for (int i = 0; i < n; i++) {
    bool esLaNuestra = (WiFi.SSID(i) == WIFI_SSID);
    if (esLaNuestra) encontrada = true;
    Serial.printf("   %-24s RSSI %4d dBm  canal %2d  %s%s\n",
                  WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
                  WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "abierta" : "cifrada",
                  esLaNuestra ? "   <<< ESTA ES" : "");
  }

  if (!encontrada) {
    Serial.println("\n  Ve otras redes pero NO ve JETSON_AP.");
    Serial.println("  El AP emite en canal 1 y comparte radio con el enlace");
    Serial.println("  cliente de la Jetson. Comprueba en la Jetson:");
    Serial.println("    iw dev ap0 info");
    Serial.println("  Si el AP cambio de canal, el C3 igual lo encuentra, asi");
    Serial.println("  que descarta primero distancia y antena.");
  } else {
    Serial.println("\n  La red SE VE. Si el RSSI es peor que -80 dBm el enlace");
    Serial.println("  va a ser inestable aunque llegue a asociarse.");
  }
  WiFi.scanDelete();
}

bool intentar(const char* etiqueta, bool conIpFija) {
  Serial.printf("\n--- %s ---\n", etiqueta);
  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_STA);

  if (conIpFija) {
    // Se prueba DESPUES del intento con DHCP a proposito. Si WiFi.config()
    // esta mal (mascara, gateway) la asociacion ocurre igual pero el
    // dispositivo queda incomunicado, y eso se confunde facil con "no
    // conecta". Separar los dos casos es justo el objetivo.
    if (!WiFi.config(STATIC_IP, GATEWAY_IP, SUBNET_MSK)) {
      Serial.println("  WiFi.config() devolvio false: la IP fija NO se aplico.");
    }
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("  conectando");
  unsigned long t0 = millis();
  wl_status_t st = WL_IDLE_STATUS;
  while (millis() - t0 < 20000) {
    st = (wl_status_t)WiFi.status();
    if (st == WL_CONNECTED) break;
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("  estado final: %s\n", textoEstado(st));

  if (st == WL_CONNECTED) {
    Serial.printf("  IP        : %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  gateway   : %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("  RSSI      : %d dBm\n", WiFi.RSSI());
    Serial.printf("  canal     : %d\n", WiFi.channel());
    // Prueba de ruta real: se abre un UDP hacia la Jetson y se manda un
    // byte. No se usa WiFi.ping() porque no existe en todas las versiones
    // del core de ESP32 y romperia la compilacion justo en la herramienta
    // que sirve para diagnosticar. Asociado pero sin ruta es un estado
    // frecuente cuando la IP fija esta mal puesta.
    WiFiUDP prueba;
    prueba.begin(50000);
    bool enviado = prueba.beginPacket(GATEWAY_IP, 5002) &&
                   prueba.write((const uint8_t*)"x", 1) && prueba.endPacket();
    prueba.stop();
    Serial.printf("  UDP hacia la Jetson (%s:5002): %s\n",
                  GATEWAY_IP.toString().c_str(),
                  enviado ? "enviado" : "NO se pudo enviar (sin ruta)");
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println("\n\n===== test_wifi_c3 — diagnostico del ESP32-C3 =====");
  Serial.printf("chip     : %s rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("MAC      : %s\n", WiFi.macAddress().c_str());
  Serial.println("  ^ apunta esta MAC: si el AP nunca la registra, el C3 no");
  Serial.println("    esta llegando a transmitir (problema de radio, no de clave).");
  Serial.printf("SSID     : %s\n", WIFI_SSID);

  escanear();

  if (intentar("2) conexion con DHCP (sin IP fija)", false)) {
    Serial.println("\n  => Con DHCP SI funciona. El problema esta en");
    Serial.println("     WiFi.config() del firmware: revisa IP/gateway/mascara,");
    Serial.println("     o quitala y deja que el AP asigne la IP.");
  } else if (intentar("3) conexion con IP fija 192.168.12.50", true)) {
    Serial.println("\n  => Con IP fija SI y con DHCP no. Raro, pero el");
    Serial.println("     firmware puede quedarse con la IP fija.");
  } else {
    Serial.println("\n  => NO conecta de ninguna de las dos formas.");
    Serial.println("     Mira el estado final de arriba: si dice que no");
    Serial.println("     encuentra la red, es antena o alcance; si dice que");
    Serial.println("     fallo la conexion, es la clave o la pila WiFi.");
  }

  Serial.println("\n===== fin — reinicia para repetir =====");
}

void loop() {
  static unsigned long t = 0;
  if (millis() - t > 10000) {
    t = millis();
    Serial.printf("[vivo] estado=%s  RSSI=%d dBm  heap=%u\n",
                  textoEstado((wl_status_t)WiFi.status()), WiFi.RSSI(),
                  ESP.getFreeHeap());
  }
  delay(50);
}
