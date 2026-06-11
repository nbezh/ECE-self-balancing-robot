/* ============================================================
   Self-Balancing Robot  —  ESP32 + MPU6050 + TB6612FNG
   FreeRTOS version: 3 tasks, mutex-protected shared state.
   + Drive control (WiFi), battery / RSSI telemetry, gyro calibration.
   ------------------------------------------------------------
   RTOS DESIGN
     controlTask  : core 1, prio 3, fixed 200 Hz (vTaskDelayUntil)
                    MPU -> filter -> PID -> motors (+ drive bias/steer).
     commsTask    : core 0, prio 1, 25 Hz. Telemetry + battery + RSSI.
     distanceTask : core 0, prio 1, 10 Hz. HC-SR04 (blocking pulseIn).
     Shared state guarded by a mutex (stMutex).

   BATTERY: divider on GPIO35 -> 100k (batt+ to pin) + 47k (pin to GND).
            If you use different resistors, set BATT_DIV = R2/(R1+R2).

   LIBRARIES: ESP32Async/ESPAsyncWebServer + ESP32Async/AsyncTCP
   ------------------------------------------------------------
   REV NOTES (fixes applied):
     [1] PID anti-windup: integral clamp now scales with Ki so
         Ki*integral can never exceed the +/-255 output range.
     [2] MPU reads: high/low byte order is now explicitly sequenced
         (was relying on unspecified evaluation order of |).
     [3] Steering now respects motorDir: dir is applied to the final
         per-wheel command, so flipping motorDir flips turning too.
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
const float DRIVE_TILT = 3.0;            // deg of setpoint lean per fwd/back press
const int   STEER_PWM  = 40;             // wheel differential per left/right press
const float BATT_DIV   = 0.319;          // R2/(R1+R2) = 47/(100+47); Vbat = Vpin/BATT_DIV

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
State st = { 20.0, 0.0, 0.8, 0.0, 40, 1, false, 0, 0, 0.0, false, -1.0, 0.0, 0 };
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
body{background:#11151a;color:#e6e6e6;font-family:system-ui,sans-serif;margin:0;padding:16px;max-width:520px;margin:auto}
h1{font-size:20px;margin:4px 0 10px}
#stat{font-size:14px;color:#9fd;margin-bottom:8px;min-height:34px}
canvas{width:100%;height:150px;background:#0b0e12;border-radius:10px;display:block}
.row{margin:14px 0}
label{display:flex;justify-content:space-between;font-size:14px;margin-bottom:4px}
input[type=range]{width:100%;height:30px}
button{width:100%;padding:14px;font-size:17px;border:0;border-radius:10px;margin:6px 0;color:#fff;background:#2a7}
button.on{background:#c33}
#save{background:#357}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-width:260px;margin:14px auto}
.pad button{width:100%;padding:18px 0;font-size:22px;background:#456;margin:0;touch-action:none;user-select:none}
.pad .sp{visibility:hidden}
</style></head><body>
<h1>BalanceBot</h1>
<div id=stat>connecting…</div>
<canvas id=g width=480 height=150></canvas>
<button id=go>GO</button>
<div class=pad>
 <div class=sp></div><button id=bf>&#9650;</button><div class=sp></div>
 <button id=bl>&#9664;</button><button id=bc>&#9632;</button><button id=br>&#9654;</button>
 <div class=sp></div><button id=bb>&#9660;</button><div class=sp></div>
</div>
<div class=row><label>Kp <span id=vkp></span></label><input type=range id=kp min=0 max=80 step=0.5></div>
<div class=row><label>Ki <span id=vki></span></label><input type=range id=ki min=0 max=400 step=1></div>
<div class=row><label>Kd <span id=vkd></span></label><input type=range id=kd min=0 max=6 step=0.05></div>
<div class=row><label>Offset (deg) <span id=voff></span></label><input type=range id=off min=-12 max=12 step=0.1></div>
<div class=row><label>Min PWM (deadband) <span id=vmp></span></label><input type=range id=mp min=0 max=120 step=1></div>
<button id=save>Save to flash</button>
<script>
let ws,buf=[],MAXN=240,curRun=false;
const cv=document.getElementById('g'),cx=cv.getContext('2d');
function setS(id,v){const e=document.getElementById(id);e.value=v;document.getElementById('v'+id).textContent=(+v).toFixed(2);}
function send(k,v){if(ws&&ws.readyState==1)ws.send(k+':'+v);}
function draw(){const w=cv.width,h=cv.height,S=30;cx.clearRect(0,0,w,h);
 cx.strokeStyle='#2a3340';cx.lineWidth=1;cx.beginPath();cx.moveTo(0,h/2);cx.lineTo(w,h/2);cx.stroke();
 cx.strokeStyle='#39d';cx.lineWidth=2;cx.beginPath();
 for(let i=0;i<buf.length;i++){const x=i/MAXN*w,y=h/2-(buf[i]/S)*(h/2);i?cx.lineTo(x,y):cx.moveTo(x,y);}cx.stroke();}
function connect(){ws=new WebSocket('ws://'+location.host+'/ws');
 ws.onmessage=e=>{const d=e.data;
  if(d[0]=='s'){const p=d.split(';'),a=parseFloat(p[1]);curRun=p[2]=='1';const armed=p[3]=='1';
   document.getElementById('stat').innerHTML='angle '+a.toFixed(1)+'\u00B0 &nbsp; '+(curRun?(armed?'BALANCING':'waiting upright'):'STOPPED')+'<br>dist '+p[4]+'cm &nbsp; batt '+p[5]+'V &nbsp; rssi '+p[6]+'dBm';
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
// [FIX 2] explicit byte order: read high, then low, then combine.
//         Avoids the unspecified evaluation order of (Wire.read()<<8)|Wire.read().
static inline int16_t mpuRead16(){
  uint8_t h = Wire.read();
  uint8_t l = Wire.read();
  return (int16_t)((h << 8) | l);
}
void mpuRead(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz){
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14,true);
  ax=mpuRead16(); ay=mpuRead16(); az=mpuRead16();
  Wire.read(); Wire.read();                 // skip temperature (2 bytes)
  gx=mpuRead16(); gy=mpuRead16(); gz=mpuRead16();
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
  float vpin = analogReadMilliVolts(BATT_PIN) / 1000.0f;
  return vpin / BATT_DIV;
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
    float Kp,Ki,Kd,offset; int minPwm,dir,fb,lr; bool run;
    xSemaphoreTake(stMutex,portMAX_DELAY);
    Kp=st.Kp; Ki=st.Ki; Kd=st.Kd; offset=st.offset;
    minPwm=st.minPwm; dir=st.motorDir; run=st.run; fb=st.fb; lr=st.lr;
    xSemaphoreGive(stMutex);

    float target = offset + fb*DRIVE_TILT;   // lean the setpoint to drive fwd/back

    if(!run){
      armed=false; integral=0; motorsEnable(false); motorA(0); motorB(0);
    } else if(fabs(angle)>FALL_LIMIT){
      armed=false; integral=0; motorsEnable(false); motorA(0); motorB(0);
    } else if(!armed && fabs(angle-offset)<ARM_WINDOW){
      armed=true; integral=0; motorsEnable(true);
    }

    if(run && armed){
      float error=target-angle;
      integral+=error*CTRL_DT;
      // [FIX 1] anti-windup: clamp so Ki*integral can't exceed the +/-255 output
      float iLimit = (Ki>0.01f) ? 255.0f/Ki : 200.0f;
      integral=constrain(integral,-iLimit,iLimit);
      float output=Kp*error + Ki*integral - Kd*gyroRate;
      output=constrain(output,-255,255);
      int drive=(int)output;
      if(drive>1) drive+=minPwm; else if(drive<-1) drive-=minPwm;
      drive=constrain(drive,-255,255);        // base balance command; dir applied below
      int steer=lr*STEER_PWM;                 // differential for turning
      // [FIX 3] apply motorDir to the final per-wheel command so turning flips with it
      int left =constrain(drive+steer,-255,255)*dir;
      int right=constrain(drive-steer,-255,255)*dir;
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
