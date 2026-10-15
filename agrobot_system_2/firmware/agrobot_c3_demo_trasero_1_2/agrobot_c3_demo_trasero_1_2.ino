/* AGROBOT C3 demo-trasero 1.2 — base 1.1 + limites configurables + pines definidos al soltar.
   AGROBOT C3 v6 — UDP + web minimo; Arduino ESP32 3.x; SIN BLE.
   Pivote trasero PASIVO, modulo unico +/-20 grados. No tiene servo de direccion.
   Modelo nominal sin encoders, sensor del pivote ni corriente: NO mide velocidad/atasco.
   Motores solo a traves de puente H compatible IN/IN, nunca desde GPIO.
   Arranque SIN movimientos. Mapeo debe confirmarse en la web antes de conducir.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_system.h>
#include <atomic>
#include <esp_task_wdt.h>
#include "control_model.h"
#include "web_panel.h"

// AP .240 fuera del DHCP .50-.200 verificado. Reservar LAN .50 en router.
struct NetworkProfile {const char *ssid,*pass;IPAddress ip,gateway,jetson;};
NetworkProfile networks[]={
 {"JETSON_AP","12345678",IPAddress(192,168,12,240),IPAddress(192,168,12,1),IPAddress(192,168,12,1)}
};
constexpr char WEB_USER[]="agrobot",WEB_PASS[]="12345678"; // cambiar antes de compartir red
constexpr uint32_t DEADMAN_MS=500;
constexpr uint8_t SERVO_PIN=10,LED_PIN=8,SERVO_CHANNEL=4;
constexpr bool LED_ACTIVE_LOW=true;
constexpr uint8_t SERVO_MIN=25,SERVO_MAX=130,SERVO_HOME=98;
constexpr uint32_t MOTOR_HZ=5000;
// Parametros ajustables en caliente desde /config y guardados en NVS. Los
// valores por defecto son los de la 1.1. Todos con limites duros: una orden
// invalida se rechaza entera y no cambia nada.
struct Cfg {float fwd=agro::MAX_DUTY,rev=agro::REVERSE_MAX_DUTY,minp=0,inner=65,slew=60;uint32_t gap=25;};
Cfg cfg;
constexpr float DRIVE_LIMIT_MPS=.55f; // escala heredada NO medida; recta permite 100% en las 4 ruedas
constexpr int TEST_MAX_PCT=50;

struct Motor {const char *name;uint8_t fi,bi;bool invert;int8_t attached,dir;float duty;uint32_t switchAt;};
Motor motors[4]={{"R1",0,1,false,-1,0,0,0},{"L1",3,2,false,-1,0,0,0},
                {"L2",4,5,false,-1,0,0,0},{"R2",6,7,false,-1,0,0,0}};
static_assert(SERVO_CHANNEL==4,"Motor channels 0..3, servo channel 4");
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
uint8_t roles[4]={0,1,2,3}; // physical channel -> logical wheel R1,L1,L2,R2
float targets[4]={},actual[4]={};uint32_t refreshed=0;bool timeoutMotor=true;
volatile bool hwFault=false;
bool verified=false,manualMode=false,jetsonArmed=false;
bool udpStarted=false,staWasUp=false;
uint32_t token=0,webLast=0,jetsonLast=0,attemptAt=0,servoAt=0,statusAt=0;
uint16_t webSeq=0,jetsonSeq=0;bool haveWebSeq=false,haveJetsonSeq=false;
uint32_t badPackets=0;int profile=0;
std::atomic<unsigned> wifiReason{0},wifiDrops{0};
uint32_t reconnectAt=0;
float appliedV=0,appliedW=0,servoPosition=SERVO_HOME;
uint8_t servoGoal=SERVO_HOME;bool led=false,servoOK=false;
WiFiUDP udp;WebServer server(80);Preferences prefs;

#pragma pack(push,1)
struct Command {uint8_t magic;int8_t v,w;uint8_t servo,flags;uint16_t seq;uint8_t crc;};
#pragma pack(pop)
static_assert(sizeof(Command)==8,"Jetson BLE/UDP compact frame = 8 bytes");

void setTargets(const float *values){
 portENTER_CRITICAL(&mux);for(int i=0;i<4;i++)targets[i]=values[i];refreshed=millis();timeoutMotor=false;portEXIT_CRITICAL(&mux);
}
void stopAll(){float z[4]={};setTargets(z);appliedV=appliedW=0;}
void ledSet(bool on){led=on;digitalWrite(LED_PIN,LED_ACTIVE_LOW?!on:on);}
// 1.2: tras ledcDetach el pin YA NO es GPIO para el gestor de perifericos y un
// digitalWrite posterior no hace nada (el core lo rechaza). En la 1.1 el
// "ambos LOW = rueda libre" dependia por tanto de un registro que el codigo no
// controlaba. Se devuelve el pin a GPIO OUTPUT explicitamente antes de ponerlo a LOW.
void detachMotor(int i){
 Motor &m=motors[i];
 if(m.attached>=0){
  uint8_t pin=m.attached;
  ledcWriteChannel(i,0);ledcDetach(pin);m.attached=-1;
  pinMode(pin,OUTPUT);digitalWrite(pin,LOW);
 }
 digitalWrite(m.fi,LOW);digitalWrite(m.bi,LOW);m.duty=0;
}
// Only this task touches motor PWM after setup. Timeout works even with slow HTTP clients.
void motorTask(void *){
 TickType_t wake=xTaskGetTickCount();
 for(;;){
  uint32_t now=millis();float wanted[4];bool expire;float slew;uint32_t gap;
  portENTER_CRITICAL(&mux);slew=cfg.slew;gap=cfg.gap;expire=(uint32_t)(now-refreshed)>DEADMAN_MS;
  timeoutMotor=expire;
  for(int i=0;i<4;i++){if(expire||hwFault)targets[i]=0;wanted[i]=targets[i];}
  portEXIT_CRITICAL(&mux);
  for(int i=0;i<4;i++){
   Motor &m=motors[i];float value=wanted[i];int dir=value>0?1:value<0?-1:0;
   if(hwFault)dir=0;
   if(!dir){detachMotor(i);m.dir=0;}
   else{
    if(m.dir!=dir){detachMotor(i);m.dir=dir;m.switchAt=now;}
    if((uint32_t)(now-m.switchAt)>=gap){
     int pin=dir>0?m.fi:m.bi;
     if(m.attached<0){
      if(!ledcAttachChannel(pin,MOTOR_HZ,8,i)){portENTER_CRITICAL(&mux);hwFault=true;portEXIT_CRITICAL(&mux);detachMotor(i);}
      else m.attached=pin;
     }
     if(m.attached>=0){float goal=fabsf(value);m.duty+=agro::clip(goal-m.duty,-slew*.01f,slew*.01f);
      ledcWriteChannel(i,lroundf(agro::clip(m.duty,0,100)*255/100));}
    }
   }
   portENTER_CRITICAL(&mux);actual[i]=m.dir*m.duty;portEXIT_CRITICAL(&mux);
  }
  vTaskDelayUntil(&wake,pdMS_TO_TICKS(10));
 }
}
void drive(float v,float w){
 agro::Limits L;float fwd,rev,inner,minp;
 portENTER_CRITICAL(&mux);fwd=cfg.fwd;rev=cfg.rev;inner=cfg.inner;minp=cfg.minp;portEXIT_CRITICAL(&mux);
 L.fwd=fwd;L.rev=rev;L.inner=inner*.01f;L.minp=minp;
 agro::Mix m=agro::mix(v,w,DRIVE_LIMIT_MPS,L);float values[4];
 for(int i=0;i<4;i++)values[i]=motors[i].invert?-m.duty[roles[i]]:m.duty[roles[i]];
 setTargets(values);appliedV=m.v;appliedW=m.w;
}
void servoTick(){
 uint32_t now=millis();if((uint32_t)(now-servoAt)<20)return;
 float dt=fminf((now-servoAt)/1000.f,.1f);servoAt=now;
 servoPosition+=agro::clip(servoGoal-servoPosition,-60*dt,60*dt);
 if(servoOK){float us=500+servoPosition*(1900.f/180);ledcWriteChannel(SERVO_CHANNEL,lroundf(us*16384/20000));}
}
void beginProfile(){
 if(udpStarted){udp.stop();udpStarted=false;}
 WiFi.disconnect(false,false);
 if(!WiFi.config(networks[profile].ip,networks[profile].gateway,IPAddress(255,255,255,0)))Serial.println("ERROR WiFi.config");
 WiFi.begin(networks[profile].ssid,networks[profile].pass);attemptAt=millis();staWasUp=false;
}
void networkTick(){
 uint32_t now=millis();bool up=WiFi.status()==WL_CONNECTED;
 if(up&&!staWasUp){
  udpStarted=udp.begin(5002);staWasUp=true;reconnectAt=now;
  Serial.printf("STA conectada %s IP %s UDP %s\n",networks[profile].ssid,WiFi.localIP().toString().c_str(),udpStarted?"OK":"ERROR");
 }
 if(!up&&staWasUp){
  if(!manualMode){stopAll();jetsonArmed=false;}
  udp.stop();udpStarted=false;staWasUp=false;attemptAt=reconnectAt=now;
  Serial.printf("STA perdida motivo=%u\n",wifiReason.load());
 }
 if(up&&!udpStarted&&(uint32_t)(now-reconnectAt)>3000){reconnectAt=now;udpStarted=udp.begin(5002);}
 if(!up){
  bool activeWeb=manualMode&&token&&(uint32_t)(now-webLast)<=DEADMAN_MS;
  if(!activeWeb&&(uint32_t)(now-reconnectAt)>5000){reconnectAt=now;beginProfile();}
 }
}
void udpTick(){
 if(!udpStarted)return;
 for(int n=0;n<4;n++){
  int size=udp.parsePacket();if(size<=0)break;
  Command c{};bool source=udp.remoteIP()==networks[profile].jetson&&udp.remotePort()==5002;
  int got=udp.read((uint8_t*)&c,sizeof(c));while(udp.available())udp.read();
  if(!source||size!=8||got!=8||c.magic!=0xA5||agro::crc8((uint8_t*)&c,7)!=c.crc||c.v<-100||c.v>100||c.w<-100||c.w>100||c.servo<SERVO_MIN||c.servo>SERVO_MAX||(c.flags&~3)){badPackets++;continue;}
  bool stale=!jetsonLast||(uint32_t)(millis()-jetsonLast)>DEADMAN_MS;
  bool zero=c.v==0&&c.w==0;
  if(haveJetsonSeq&&!agro::newer(c.seq,jetsonSeq)&&!(stale&&zero)){badPackets++;continue;}
  jetsonSeq=c.seq;haveJetsonSeq=true;jetsonLast=millis();
  if(manualMode)continue; // local control exclusively owns motors + actuators
  if(stale){stopAll();jetsonArmed=false;}
  if(!verified||hwFault){stopAll();continue;}
  if(!jetsonArmed){if(!zero){stopAll();continue;}jetsonArmed=true;}
  drive(c.v*.01f*agro::V_SCALE,c.w*.01f*agro::W_SCALE);
  servoGoal=c.servo;ledSet(c.flags&1);
  // FLAG_SCAN accepted for compatibility; camera sweep remains on Jetson.
 }
}
void sendStatus(){
 if(!udpStarted||WiFi.status()!=WL_CONNECTED)return;
 bool expired;
 portENTER_CRITICAL(&mux);expired=timeoutMotor;portEXIT_CRITICAL(&mux);
 Command s{};s.magic=0x5A;
 s.v=expired?0:lroundf(appliedV/agro::V_SCALE*100);s.w=expired?0:lroundf(appliedW/agro::W_SCALE*100);
 s.servo=lroundf(servoPosition);s.seq=jetsonSeq;s.flags=(led?1:0)|0x40;
 if(expired||hwFault||(manualMode?!token:(!jetsonArmed||!verified)))s.flags|=4;
 if(esp_reset_reason()==ESP_RST_BROWNOUT)s.flags|=8;
 if(millis()<15000)s.flags|=16;
 if(!manualMode)s.flags|=32;
 s.crc=agro::crc8((uint8_t*)&s,7);udp.beginPacket(networks[profile].jetson,5002);udp.write((uint8_t*)&s,8);udp.endPacket();
}
bool auth(){if(server.authenticate(WEB_USER,WEB_PASS))return true;server.requestAuthentication();return false;}
bool intArg(const char *key,int low,int high,int &out){
 if(!server.hasArg(key))return false;
 String s=server.arg(key);if(!s.length()||s.length()>7)return false;
 char *end;long v=strtol(s.c_str(),&end,10);if(*end||v<low||v>high)return false;out=v;return true;
}
void replyError(int status,const char *message){server.send(status,"text/plain",message);}
bool lease(){
 if(!manualMode||!token||!server.hasArg("token")||(uint32_t)(millis()-webLast)>DEADMAN_MS)return false;
 String value=server.arg("token");char *end;uint32_t t=strtoul(value.c_str(),&end,16);
 if(!value.length()||value.length()>8||*end||t!=token)return false;
 int seq;if(!intArg("seq",0,65535,seq)|| (haveWebSeq&&!agro::newer(seq,webSeq)))return false;
 webSeq=seq;haveWebSeq=true;webLast=millis();return true;
}
void invalidate(){stopAll();ledSet(false);token=0;haveWebSeq=false;jetsonArmed=false;}
void statusWeb(){
 if(!auth())return;
 char json[2200];float duty[4];bool timed,fault;Cfg c;
 portENTER_CRITICAL(&mux);for(int i=0;i<4;i++)duty[i]=actual[i];timed=timeoutMotor;fault=hwFault;c=cfg;portEXIT_CRITICAL(&mux);
 int n=snprintf(json,sizeof(json),"{\"mode\":\"%s\",\"ip\":\"%s\",\"ap\":%s,\"verified\":%s,\"armed\":%s,\"timeout\":%s,\"fault\":%s,\"heap\":%u,\"jetson_age_ms\":%lu,\"bad_packets\":%lu,\"servo\":%u,\"motors\":[",
 manualMode?"WEB":"JETSON",WiFi.localIP().toString().c_str(),"false",verified?"true":"false",jetsonArmed?"true":"false",timed?"true":"false",fault?"true":"false",(unsigned)ESP.getFreeHeap(),(unsigned long)(jetsonLast?millis()-jetsonLast:UINT32_MAX),(unsigned long)badPackets,servoGoal);
 for(int i=0;i<4;i++)n+=snprintf(json+n,sizeof(json)-n,"%s{\"name\":\"%s\",\"fi\":%u,\"bi\":%u,\"role\":%u,\"invert\":%s,\"duty\":%.1f}",i?",":"",motors[i].name,motors[i].fi,motors[i].bi,roles[i],motors[i].invert?"true":"false",duty[i]);
 snprintf(json+n,sizeof(json)-n,"],\"cfg\":{\"fwd\":%.0f,\"rev\":%.0f,\"minp\":%.0f,\"inner\":%.0f,\"slew\":%.0f,\"gap\":%lu},\"uptime_ms\":%lu,\"reset_reason\":%d,\"wifi_reason\":%u,\"wifi_drops\":%u,\"rssi\":%d,\"firmware\":\"demo-trasero-1.2\"}",c.fwd,c.rev,c.minp,c.inner,c.slew,(unsigned long)c.gap,(unsigned long)millis(),(int)esp_reset_reason(),wifiReason.load(),wifiDrops.load(),WiFi.RSSI());server.send(200,"application/json",json);
}
void setupWeb(){
 server.on("/",HTTP_GET,[](){if(auth())server.send_P(200,"text/html",PAGE);});
 server.on("/index.html",HTTP_GET,[](){if(auth())server.send_P(200,"text/html",PAGE);});
 server.on("/status",HTTP_GET,statusWeb);
 server.on("/halt",HTTP_POST,[](){if(!auth())return;invalidate();server.send(200,"text/plain","PARADO");});
 server.on("/stop",HTTP_POST,[](){if(!auth())return;invalidate();manualMode=true;ledSet(false);server.send(200,"text/plain","PARADO; vuelva a mantener un boton");});
 server.on("/claim",HTTP_POST,[](){if(!auth())return;invalidate();manualMode=true;webLast=millis();token=esp_random();if(!token)token=1;char t[12];snprintf(t,sizeof(t),"%08lx",(unsigned long)token);server.send(200,"text/plain",t);});
 server.on("/jetson",HTTP_POST,[](){if(!auth())return;invalidate();manualMode=false;server.send(200,"text/plain","JETSON: espera orden cero fresca para rearmar");});
 server.on("/drive",HTTP_POST,[](){
  if(!auth())return;int v,w;
  if(!lease()||!verified||hwFault||!intArg("v",-100,100,v)||!intArg("w",-100,100,w)){invalidate();replyError(409,"lease/mapeo/parametros invalidos");return;}
  drive(v*.01f*DRIVE_LIMIT_MPS,w*.01f*agro::W_SCALE);server.send(200,"text/plain","OK");
 });
 server.on("/motor",HTTP_POST,[](){
  if(!auth())return;int i,d,p;
  if(!lease()||hwFault||!intArg("m",0,3,i)||!intArg("d",-1,1,d)||!d||!intArg("p",0,TEST_MAX_PCT,p)){invalidate();replyError(409,"prueba invalida");return;}
  float a[4]={};a[i]=p*d*(motors[i].invert?-1:1);setTargets(a);appliedV=appliedW=0;server.send(200,"text/plain","OK");
 });
 server.on("/invert",HTTP_POST,[](){
  if(!auth())return;invalidate();manualMode=true;int i;
  if(!intArg("m",0,3,i)){replyError(400,"motor invalido");return;}
  motors[i].invert=!motors[i].invert;verified=false;prefs.putBool("verified",false);
  char k[8];snprintf(k,sizeof(k),"inv%d",i);prefs.putBool(k,motors[i].invert);server.send(200,"text/plain","Guardado; confirmar mapeo de nuevo");
 });
 server.on("/map",HTTP_POST,[](){
  if(!auth())return;invalidate();manualMode=true;
  uint8_t proposed[4];unsigned mask=0;
  for(int i=0;i<4;i++){char key[4];snprintf(key,sizeof(key),"r%d",i);int role;
   if(!intArg(key,0,3,role)||(mask&(1u<<role))){replyError(400,"Cada rueda debe tener un canal distinto");return;}
   proposed[i]=role;mask|=1u<<role;
  }
  verified=false;prefs.putBool("verified",false);
  if(prefs.putBytes("roles",proposed,4)!=4){replyError(500,"No se guardo el mapeo");return;}
  for(int i=0;i<4;i++)roles[i]=proposed[i];
  server.send(200,"text/plain","Mapeo guardado; confirmar ruedas antes de conducir");
 });
 server.on("/verify",HTTP_POST,[](){if(!auth())return;invalidate();manualMode=true;verified=true;prefs.putBool("verified",true);server.send(200,"text/plain","Mapeo confirmado por operador");});
 server.on("/config",HTTP_POST,[](){
  if(!auth())return;
  // Todo o nada: se valida cada parametro presente ANTES de tocar nada.
  struct P{const char *k;int lo,hi;} defs[]={{"fwd",10,100},{"rev",10,100},{"minp",0,60},{"inner",0,90},{"slew",20,300},{"gap",10,200}};
  int val[6];bool has[6];bool any=false;
  for(int i=0;i<6;i++){
   has[i]=server.hasArg(defs[i].k);
   if(has[i]){if(!intArg(defs[i].k,defs[i].lo,defs[i].hi,val[i])){replyError(400,"parametro fuera de rango");return;}any=true;}
  }
  if(!any){replyError(400,"sin parametros");return;}
  Cfg n=cfg;
  if(has[0])n.fwd=val[0];if(has[1])n.rev=val[1];if(has[2])n.minp=val[2];
  if(has[3])n.inner=val[3];if(has[4])n.slew=val[4];if(has[5])n.gap=val[5];
  if(n.minp>n.fwd){replyError(400,"minp no puede superar fwd");return;}
  invalidate();   // parar antes de cambiar limites
  portENTER_CRITICAL(&mux);cfg=n;portEXIT_CRITICAL(&mux);
  prefs.putFloat("c_fwd",n.fwd);prefs.putFloat("c_rev",n.rev);prefs.putFloat("c_minp",n.minp);
  prefs.putFloat("c_inner",n.inner);prefs.putFloat("c_slew",n.slew);prefs.putUInt("c_gap",n.gap);
  server.send(200,"text/plain","Limites guardados");
 });
 server.on("/config/reset",HTTP_POST,[](){
  if(!auth())return;invalidate();Cfg d;
  portENTER_CRITICAL(&mux);cfg=d;portEXIT_CRITICAL(&mux);
  prefs.remove("c_fwd");prefs.remove("c_rev");prefs.remove("c_minp");prefs.remove("c_inner");prefs.remove("c_slew");prefs.remove("c_gap");
  server.send(200,"text/plain","Limites por defecto");
 });
 server.on("/actuator",HTTP_POST,[](){
  if(!auth())return;int s,l;
  if(!lease()||!intArg("servo",SERVO_MIN,SERVO_MAX,s)||!intArg("led",0,1,l)){invalidate();replyError(409,"actuador invalido");return;}
  stopAll();servoGoal=s;ledSet(l);server.send(200,"text/plain","OK");
 });
 server.onNotFound([](){replyError(404,"Ruta inexistente");});server.begin();
}
void setup(){
 for(Motor &m:motors){digitalWrite(m.fi,LOW);digitalWrite(m.bi,LOW);pinMode(m.fi,OUTPUT);pinMode(m.bi,OUTPUT);}
 digitalWrite(LED_PIN,LED_ACTIVE_LOW?HIGH:LOW);pinMode(LED_PIN,OUTPUT);
 Serial.begin(115200);prefs.begin("agro-demo1",false);verified=false;
 for(int i=0;i<4;i++){char k[8];snprintf(k,sizeof(k),"inv%d",i);motors[i].invert=prefs.getBool(k,false);}
 if(prefs.getBytesLength("roles")==4){
  uint8_t saved[4];prefs.getBytes("roles",saved,4);unsigned mask=0;bool valid=true;
  for(int i=0;i<4;i++){if(saved[i]>3||(mask&(1u<<saved[i]))){valid=false;break;}mask|=1u<<saved[i];}
  if(valid)for(int i=0;i<4;i++)roles[i]=saved[i];
 }
 {
  // Config guardada: se valida TODO; si algo esta fuera de rango se usan los defectos.
  Cfg c;c.fwd=prefs.getFloat("c_fwd",c.fwd);c.rev=prefs.getFloat("c_rev",c.rev);c.minp=prefs.getFloat("c_minp",c.minp);
  c.inner=prefs.getFloat("c_inner",c.inner);c.slew=prefs.getFloat("c_slew",c.slew);c.gap=prefs.getUInt("c_gap",c.gap);
  bool ok=c.fwd>=10&&c.fwd<=100&&c.rev>=10&&c.rev<=100&&c.minp>=0&&c.minp<=60&&c.minp<=c.fwd&&c.inner>=0&&c.inner<=90&&c.slew>=20&&c.slew<=300&&c.gap>=10&&c.gap<=200;
  if(ok)cfg=c;else Serial.println("Config NVS invalida: defectos");
 }
 servoOK=ledcAttachChannel(SERVO_PIN,50,14,SERVO_CHANNEL);
 if(!servoOK)hwFault=true;
 if(xTaskCreate(motorTask,"motor-safe",3072,nullptr,3,nullptr)!=pdPASS)hwFault=true;
 servoAt=millis();servoTick();
 WiFi.setHostname("agrobot-c3");WiFi.mode(WIFI_STA);WiFi.setSleep(false);WiFi.setAutoReconnect(true);
 WiFi.onEvent([](WiFiEvent_t event,WiFiEventInfo_t info){
  if(event==ARDUINO_EVENT_WIFI_STA_DISCONNECTED){wifiReason.store(info.wifi_sta_disconnected.reason);wifiDrops.fetch_add(1);}
 });
 beginProfile();WiFi.setTxPower(WIFI_POWER_8_5dBm);
 setupWeb();
 esp_task_wdt_config_t wdt={.timeout_ms=8000,.idle_core_mask=0,.trigger_panic=true};
 if(esp_task_wdt_reconfigure(&wdt)!=ESP_OK)esp_task_wdt_init(&wdt);
 enableLoopWDT();
 Serial.println("AGROBOT demo-trasero-1.2. Sin auto-test. Web usuario agrobot. PWM canal 0..3; servo 4.");
}
void loop(){
 udpTick();
 if(!manualMode&&jetsonLast&&(uint32_t)(millis()-jetsonLast)>DEADMAN_MS){stopAll();jetsonArmed=false;ledSet(false);}
 if(manualMode&&token&&(uint32_t)(millis()-webLast)>DEADMAN_MS){invalidate();ledSet(false);}
 networkTick();server.handleClient();servoTick();
 if((uint32_t)(millis()-statusAt)>=100){statusAt=millis();sendStatus();}
 delay(2);
}
