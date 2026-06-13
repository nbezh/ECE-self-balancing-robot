/* ============================================================
   Self-Balancing Robot  —  ESP32 + MPU6050 + TB6612FNG
   FreeRTOS version: 3 tasks, mutex-protected shared state.
   + Drive control (WiFi) and battery / RSSI telemetry.
   ------------------------------------------------------------
   RTOS DESIGN
     controlTask  : core 1, prio 3, fixed 200 Hz (vTaskDelayUntil)
                    MPU -> filter -> PID -> motors (+ drive bias/steer).
     commsTask    : core 0, prio 1, 25 Hz. Telemetry + battery + RSSI.
     distanceTask : core 0, prio 1, 10 Hz. HC-SR04 (blocking pulseIn).
     Shared state guarded by a mutex (stMutex).

   DRIVE: web D-pad leans the setpoint (fwd/back) and adds a
          differential to the wheels (left/right). The robot keeps
          balancing while it drives.

   BATTERY: divider on GPIO35 -> 100k (batt+ to pin) + 47k (pin to GND).

   LIBRARIES: ESP32Async/ESPAsyncWebServer + ESP32Async/AsyncTCP
   ============================================================ */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ---------------- Mode ----------------
#define DIAGNOSTIC 0     // 1 = axis-finding (motors OFF). 0 = balance.

// ---------------- Pin map ----------------
#define PWMA 4
#define AIN2 16
#define AIN1 17
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 23
#define TRIG 26
#define ECHO 25
#define BATT_PIN 35              // ADC1, input-only, safe with WiFi

#define MPU_ADDR 0x68
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
#define PWM_CH_A 0
#define PWM_CH_B 1

const float FALL_LIMIT = 45.0;
const float ARM_WINDOW = 3.0;
const float CTRL_DT    = 0.005;          // 200 Hz control period (s)
const float DRIVE_TILT = 1.5;            // deg of setpoint lean per fwd/back press (gentle = keeps balance)
const float DRIVE_RAMP = 0.03;           // how fast the drive lean eases in (per 5 ms cycle)
const int   STEER_PWM  = 40;             // wheel differential per left/right press
const float BATT_DIV   = 0.319;          // R2/(R1+R2) = 47/(100+47); Vbat = Vpin/BATT_DIV
const float OBST_NEAR  = 20.0;           // cm: start backing away below this
const float OBST_FAR   = 30.0;           // cm: stop avoiding once clear past this

// ---------------- Shared state (guarded by stMutex) ----------------
struct State {
  float Kp, Ki, Kd, offset;   // params: web -> control
  int   minPwm, motorDir;
  bool  run;
  int   fb, lr;               // drive: forward/back, left/right (-1,0,1)
  float angle;                // telemetry: control -> comms
  bool  armed;
  float distanceCm;           // distanceTask -> comms
  float battV;                // commsTask
  int   rssi;                 // commsTask
};
State st = { 20.0, 0.0, 0.8, 0.0, 40, -1, false, 0, 0, 0.0, false, -1.0, 0.0, 0 };
SemaphoreHandle_t stMutex;

// ---------------- WiFi AP ----------------
const char* AP_SSID = "BalanceBot";
const char* AP_PASS = "balance123";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences prefs;

// ---------------- Web page ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>BalanceBot</title><style>
*{box-sizing:border-box}
body{margin:0 auto;padding:20px 14px 40px;max-width:460px;
  font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;color:#6b3350;
  background:linear-gradient(165deg,#fff5f9 0%,#ffe8f1 55%,#ffdbe9 100%);min-height:100vh;
  -webkit-tap-highlight-color:transparent}
h1{font-size:23px;font-weight:800;margin:2px 0;color:#e84a8a;text-align:center;letter-spacing:.3px}
.sub{text-align:center;font-size:12px;color:#cf83a8;margin:2px 0 16px;letter-spacing:1px;text-transform:uppercase}
.card{background:#fff;border:1px solid #ffd9e7;border-radius:20px;
  box-shadow:0 8px 22px rgba(255,140,175,.18);padding:15px 16px;margin-bottom:14px}
#stat{font-size:13px;color:#b06088;line-height:1.6;text-align:center}
#stat b{color:#e84a8a;font-size:20px;font-weight:800}
canvas{width:100%;height:150px;background:#fff8fb;border-radius:14px;display:block;border:1px solid #ffe1ec}
.glabel{font-size:11px;color:#cf83a8;text-align:center;margin-top:8px;letter-spacing:1.5px;text-transform:uppercase}
.ttl{font-size:11px;font-weight:800;color:#cf83a8;text-transform:uppercase;letter-spacing:1.5px;margin:0 0 12px;text-align:center}
.row{margin:14px 0}
.row:first-of-type{margin-top:0}
.row:last-of-type{margin-bottom:0}
label{display:flex;justify-content:space-between;font-size:13px;font-weight:700;margin-bottom:9px;color:#9a4f72}
label span{color:#ff5e9a;font-weight:800}
input[type=range]{width:100%;height:8px;border-radius:8px;-webkit-appearance:none;appearance:none;
  background:#ffe0ed;accent-color:#ff5e9a;outline:none;cursor:pointer}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;border-radius:50%;
  background:#ff5e9a;border:3px solid #fff;box-shadow:0 2px 7px rgba(255,94,154,.55)}
input[type=range]::-moz-range-thumb{width:20px;height:20px;border-radius:50%;background:#ff5e9a;border:3px solid #fff}
button{width:100%;padding:15px;font-size:17px;font-weight:800;border:0;border-radius:15px;margin:0;color:#fff;
  background:linear-gradient(135deg,#ff95bd,#ff5e9a);box-shadow:0 6px 15px rgba(255,94,154,.32);cursor:pointer;
  -webkit-user-select:none;-moz-user-select:none;-ms-user-select:none;user-select:none;
  -webkit-touch-callout:none;-webkit-tap-highlight-color:transparent;transition:transform .05s,filter .15s}
button:active{transform:translateY(1px);filter:brightness(.97)}
#go{margin-bottom:14px;letter-spacing:1px}
#go.on{background:linear-gradient(135deg,#ff7a92,#e8417a)}
#save{background:#fff;color:#e84a8a;border:2px solid #ffc6dd;box-shadow:none;font-size:15px}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:230px;margin:0 auto}
.pad button{padding:15px 0;font-size:21px;color:#d6336c;box-shadow:0 4px 11px rgba(255,140,175,.32);
  background:linear-gradient(135deg,#ffd3e3,#ffbcd6);touch-action:none}
.pad #bc{background:linear-gradient(135deg,#ffb0cd,#ff8fb6);color:#fff}
.pad .sp{visibility:hidden;box-shadow:none}
</style></head><body>
<h1>BalanceBot</h1>
<div class=sub>control &amp; tuning</div>
<div class=card><div id=stat>connecting…</div></div>
<div class=card><canvas id=g width=480 height=150></canvas><div class=glabel>tilt angle</div></div>
<button id=go>GO</button>
<div class=card><div class=ttl>drive</div>
<div class=pad>
 <div class=sp></div><button id=bf>&#9650;</button><div class=sp></div>
 <button id=bl>&#9664;</button><button id=bc>&#9632;</button><button id=br>&#9654;</button>
 <div class=sp></div><button id=bb>&#9660;</button><div class=sp></div>
</div></div>
<div class=card><div class=ttl>tuning</div>
<div class=row><label>Kp <span id=vkp></span></label><input type=range id=kp min=0 max=80 step=0.5></div>
<div class=row><label>Ki <span id=vki></span></label><input type=range id=ki min=0 max=400 step=1></div>
<div class=row><label>Kd <span id=vkd></span></label><input type=range id=kd min=0 max=6 step=0.05></div>
<div class=row><label>Offset (deg) <span id=voff></span></label><input type=range id=off min=-12 max=12 step=0.1></div>
<div class=row><label>Min PWM (deadband) <span id=vmp></span></label><input type=range id=mp min=0 max=120 step=1></div>
</div>
<button id=save>Save to flash</button>
<script>
let ws,buf=[],MAXN=240,curRun=false;
const cv=document.getElementById('g'),cx=cv.getContext('2d');
function setS(id,v){const e=document.getElementById(id);e.value=v;document.getElementById('v'+id).textContent=(+v).toFixed(2);}
function send(k,v){if(ws&&ws.readyState==1)ws.send(k+':'+v);}
function draw(){const w=cv.width,h=cv.height,S=30;cx.clearRect(0,0,w,h);
 cx.strokeStyle='#f6cadb';cx.lineWidth=1;cx.beginPath();cx.moveTo(0,h/2);cx.lineTo(w,h/2);cx.stroke();
 cx.strokeStyle='#ff4e8a';cx.lineWidth=2.5;cx.lineJoin='round';cx.beginPath();
 for(let i=0;i<buf.length;i++){const x=i/MAXN*w,y=h/2-(buf[i]/S)*(h/2);i?cx.lineTo(x,y):cx.moveTo(x,y);}cx.stroke();}
function connect(){ws=new WebSocket('ws://'+location.host+'/ws');
 ws.onmessage=e=>{const d=e.data;
  if(d[0]=='s'){const p=d.split(';'),a=parseFloat(p[1]);curRun=p[2]=='1';const armed=p[3]=='1';
   document.getElementById('stat').innerHTML='<b>'+a.toFixed(1)+'\u00B0</b><br>'+(curRun?(armed?'BALANCING':'waiting upright'):'STOPPED')+'<br>dist '+p[4]+'cm &nbsp;&middot;&nbsp; batt '+p[5]+'V &nbsp;&middot;&nbsp; rssi '+p[6]+'dBm';
   buf.push(a);if(buf.length>MAXN)buf.shift();draw();
   const go=document.getElementById('go');go.textContent=curRun?'STOP':'GO';go.className=curRun?'on':'';}
  else if(d[0]=='c'){const p=d.split(';');setS('kp',p[1]);setS('ki',p[2]);setS('kd',p[3]);setS('off',p[4]);setS('mp',p[5]);}};
 ws.onclose=()=>setTimeout(connect,1000);}
['kp','ki','kd','off','mp'].forEach(id=>{const e=document.getElementById(id);
 e.oninput=()=>{document.getElementById('v'+id).textContent=(+e.value).toFixed(2);send(id,e.value);};});
document.getElementById('go').onclick=()=>send('run',curRun?0:1);
document.getElementById('save').onclick=()=>{if(ws&&ws.readyState==1)ws.send('save');};
function hold(id,k,v){const e=document.getElementById(id);
 const on=ev=>{ev.preventDefault();send(k,v);};
 const off=ev=>{ev.preventDefault();send(k,0);};
 e.addEventListener('pointerdown',on);e.addEventListener('pointerup',off);
 e.addEventListener('pointerleave',off);e.addEventListener('pointercancel',off);}
hold('bf','fb',1);hold('bb','fb',-1);hold('bl','lr',-1);hold('br','lr',1);
document.getElementById('bc').onclick=()=>{send('fb',0);send('lr',0);};
connect();
</script></body></html>
)rawliteral";

// ============================================================
//  MPU6050
// ============================================================
void mpuWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
void mpuInit(){
  mpuWrite(0x6B,0x00); delay(100);
  mpuWrite(0x1A,0x03); mpuWrite(0x1B,0x00); mpuWrite(0x1C,0x00);
}
void mpuRead(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz){
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14,true);
  ax=(Wire.read()<<8)|Wire.read(); ay=(Wire.read()<<8)|Wire.read(); az=(Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read();
  gx=(Wire.read()<<8)|Wire.read(); gy=(Wire.read()<<8)|Wire.read(); gz=(Wire.read()<<8)|Wire.read();
}

// ============================================================
//  Motors  (PWM works on core 2.x channel API and 3.x pin API)
// ============================================================
void pwmSetup(){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
#else
  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES); ledcAttachPin(PWMA, PWM_CH_A);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES); ledcAttachPin(PWMB, PWM_CH_B);
#endif
}
inline void pwmA(int d){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWMA, d);
#else
  ledcWrite(PWM_CH_A, d);
#endif
}
inline void pwmB(int d){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWMB, d);
#else
  ledcWrite(PWM_CH_B, d);
#endif
}
void motorsEnable(bool on){ digitalWrite(STBY, on?HIGH:LOW); }
void motorA(int speed){ bool f=(speed>=0); int s=constrain(abs(speed),0,255);
  digitalWrite(AIN1,f?HIGH:LOW); digitalWrite(AIN2,f?LOW:HIGH); pwmA(s); }
void motorB(int speed){ bool f=(speed>=0); int s=constrain(abs(speed),0,255);
  digitalWrite(BIN1,f?HIGH:LOW); digitalWrite(BIN2,f?LOW:HIGH); pwmB(s); }

// ============================================================
//  Preferences
// ============================================================
void loadPrefs(){
  prefs.begin("balance", true);
  st.Kp=prefs.getFloat("kp",st.Kp); st.Ki=prefs.getFloat("ki",st.Ki);
  st.Kd=prefs.getFloat("kd",st.Kd); st.offset=prefs.getFloat("off",st.offset);
  st.minPwm=prefs.getInt("mp",st.minPwm);
  prefs.end();
}
void savePrefs(){
  float kp,ki,kd,off; int mp;
  xSemaphoreTake(stMutex,portMAX_DELAY);
  kp=st.Kp; ki=st.Ki; kd=st.Kd; off=st.offset; mp=st.minPwm;
  xSemaphoreGive(stMutex);
  prefs.begin("balance", false);
  prefs.putFloat("kp",kp); prefs.putFloat("ki",ki); prefs.putFloat("kd",kd);
  prefs.putFloat("off",off); prefs.putInt("mp",mp);
  prefs.end();
}

// ============================================================
//  Telemetry helpers
// ============================================================
float readBattery(){
  uint32_t sum=0; const int N=32;
  for(int i=0;i<N;i++) sum += analogReadMilliVolts(BATT_PIN);  // oversample
  float vbat = ((sum/(float)N)/1000.0f) / BATT_DIV;
  static float filt = -1;
  filt = (filt < 0) ? vbat : (filt*0.9f + vbat*0.1f);          // EMA smoothing
  return filt;
}
int apRssi(){                       // RSSI of first connected station (we are the AP)
  wifi_sta_list_t sl;
  if(esp_wifi_ap_get_sta_list(&sl) == ESP_OK && sl.num > 0) return sl.sta[0].rssi;
  return 0;
}

// ============================================================
//  WebSocket
// ============================================================
void handleMsg(const String &m){
  int c=m.indexOf(':');
  if(c<0){ if(m=="save") savePrefs(); return; }
  String k=m.substring(0,c); float v=m.substring(c+1).toFloat();
  xSemaphoreTake(stMutex,portMAX_DELAY);
  if(k=="kp")st.Kp=v; else if(k=="ki")st.Ki=v; else if(k=="kd")st.Kd=v;
  else if(k=="off")st.offset=v; else if(k=="mp")st.minPwm=(int)v;
  else if(k=="fb")st.fb=(int)v; else if(k=="lr")st.lr=(int)v;
  else if(k=="run"){ st.run=(v!=0); if(!st.run){ st.fb=0; st.lr=0; } }
  xSemaphoreGive(stMutex);
}
void onWsEvent(AsyncWebSocket*s,AsyncWebSocketClient*client,AwsEventType type,void*arg,uint8_t*data,size_t len){
  if(type==WS_EVT_CONNECT){
    float kp,ki,kd,off; int mp;
    xSemaphoreTake(stMutex,portMAX_DELAY);
    kp=st.Kp;ki=st.Ki;kd=st.Kd;off=st.offset;mp=st.minPwm;
    xSemaphoreGive(stMutex);
    String cfg="c;"+String(kp,2)+";"+String(ki,2)+";"+String(kd,2)+";"+String(off,2)+";"+String(mp);
    client->text(cfg);
  } else if(type==WS_EVT_DATA){
    AwsFrameInfo*info=(AwsFrameInfo*)arg;
    if(info->final && info->index==0 && info->len==len && info->opcode==WS_TEXT){
      String msg; msg.reserve(len);
      for(size_t i=0;i<len;i++) msg+=(char)data[i];
      handleMsg(msg);
    }
  }
}

// ============================================================
//  TASKS
// ============================================================

// --- Control task: hard real-time, core 1, 200 Hz ---
void controlTask(void *pv){
  static float angle=0, integral=0;
  static bool armed=false;
  static float gyBias=0;

  // gyro-bias calibration: keep the robot STILL for ~1.6 s at boot
  Serial.println("Calibrating gyro - hold still...");
  { long gsum=0; const int N=800; int16_t ax,ay,az,gx,gy,gz;
    for(int i=0;i<N;i++){ mpuRead(ax,ay,az,gx,gy,gz); gsum+=gy; delay(2); }
    gyBias=(float)gsum/N;
    Serial.printf("gyro bias gy=%.1f\n", gyBias);
    angle = atan2((float)az, -(float)ax) * 57.29578f; }     // seed
  TickType_t last = xTaskGetTickCount();
  for(;;){
    int16_t ax,ay,az,gx,gy,gz; mpuRead(ax,ay,az,gx,gy,gz);

#if DIAGNOSTIC
    motorsEnable(false); motorA(0); motorB(0);
    static uint32_t lr=0;
    if(millis()-lr>100){ lr=millis();
      Serial.printf("ax=%d ay=%d az=%d | gx=%d gy=%d gz=%d\n",ax,ay,az,gx,gy,gz); }
#else
    // [AXIS] gravity=X, tilt=Z, rotation=Y (gyro sign flipped: -gy)
    float accAngle = atan2((float)az, -(float)ax) * 57.29578f;
    float gyroRate = -((float)gy - gyBias) / 131.0f;
    angle = 0.98f*(angle + gyroRate*CTRL_DT) + 0.02f*accAngle;

    // snapshot params + drive commands
    float Kp,Ki,Kd,offset; int minPwm,dir,fb,lr; bool run; float dist;
    xSemaphoreTake(stMutex,portMAX_DELAY);
    Kp=st.Kp; Ki=st.Ki; Kd=st.Kd; offset=st.offset;
    minPwm=st.minPwm; dir=st.motorDir; run=st.run; fb=st.fb; lr=st.lr;
    dist=st.distanceCm;
    xSemaphoreGive(stMutex);

    // obstacle avoidance: back away from anything close ahead, overriding the
    // radio command. Hysteresis (NEAR/FAR) stops it chattering at the edge.
    static bool avoiding=false;
    if(dist>0 && dist<OBST_NEAR)       avoiding=true;
    else if(dist<0 || dist>OBST_FAR)   avoiding=false;
    if(avoiding) fb=-dir;                 // lean back -> drive away from obstacle

    // ease the drive lean in/out so fwd/back accelerates gently and keeps balance
    static float driveBias=0;
    driveBias += (fb*DRIVE_TILT - driveBias) * DRIVE_RAMP;

    float target = offset + driveBias;   // gentle lean to drive fwd/back

    if(!run){
      armed=false; integral=0; motorsEnable(false); motorA(0); motorB(0);
    } else if(fabs(angle)>FALL_LIMIT){
      armed=false; integral=0; motorsEnable(false); motorA(0); motorB(0);
    } else if(!armed && fabs(angle-offset)<ARM_WINDOW){
      armed=true; integral=0; motorsEnable(true);
    }

    if(run && armed){
      float error=target-angle;
      integral+=error*CTRL_DT; integral=constrain(integral,-200,200);
      float output=Kp*error + Ki*integral - Kd*gyroRate;
      output=constrain(output,-255,255);
      int drive=(int)output;
      if(drive>1) drive+=minPwm; else if(drive<-1) drive-=minPwm;
      drive=constrain(drive,-255,255)*dir;
      int steer=lr*STEER_PWM;                 // differential for turning
      int left =constrain(drive+steer,-255,255);
      int right=constrain(drive-steer,-255,255);
      motorA(left); motorB(right);
    } else if(run){
      motorA(0); motorB(0);
    }

    // publish telemetry
    xSemaphoreTake(stMutex,portMAX_DELAY);
    st.angle=angle; st.armed=armed;
    xSemaphoreGive(stMutex);
#endif
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));   // exact 200 Hz
  }
}

// --- Comms task: telemetry to web, core 0, 25 Hz ---
void commsTask(void *pv){
  for(;;){
    float vbat = readBattery();
    int   r    = apRssi();
    float a,d; bool run,armed;
    xSemaphoreTake(stMutex,portMAX_DELAY);
    st.battV=vbat; st.rssi=r;
    a=st.angle; run=st.run; armed=st.armed; d=st.distanceCm;
    xSemaphoreGive(stMutex);
    ws.cleanupClients();
    String m="s;"+String(a,2)+";"+String(run?1:0)+";"+String(armed?1:0)+";"
             +String(d,0)+";"+String(vbat,2)+";"+String(r);
    ws.textAll(m);
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

// --- Distance task: blocking HC-SR04 isolated here, core 0, 10 Hz ---
void distanceTask(void *pv){
  for(;;){
    digitalWrite(TRIG,LOW);  delayMicroseconds(2);
    digitalWrite(TRIG,HIGH); delayMicroseconds(10); digitalWrite(TRIG,LOW);
    unsigned long us = pulseIn(ECHO, HIGH, 30000UL);   // blocks up to 30 ms
    float cm = us ? us/58.0f : -1.0f;
    xSemaphoreTake(stMutex,portMAX_DELAY);
    st.distanceCm=cm;
    xSemaphoreGive(stMutex);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ============================================================
//  Setup / loop
// ============================================================
void setup(){
  Serial.begin(115200);
  pinMode(STBY,OUTPUT); digitalWrite(STBY,LOW);
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT); pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT);
  pwmSetup(); motorA(0); motorB(0);
  pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);
  analogReadResolution(12);

  Wire.begin(); Wire.setClock(400000); mpuInit();

  stMutex = xSemaphoreCreateMutex();
  loadPrefs();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest*r){ r->send_P(200,"text/html",PAGE); });
  server.begin();

  // priority: control(3) > comms(1)=distance(1).  core 1 = control only.
  xTaskCreatePinnedToCore(controlTask,  "control",  4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(commsTask,    "comms",    4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(distanceTask, "distance", 2048, NULL, 1, NULL, 0);
}

void loop(){
  vTaskDelay(pdMS_TO_TICKS(1000));   // nothing here; work is in the tasks
}