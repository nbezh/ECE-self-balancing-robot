/* ============================================================
   Self-Balancing Robot  —  ESP32 + MPU6050 + TB6612FNG
   With WiFi live-tuning page (Access Point + live angle graph)
   ------------------------------------------------------------
   LIBRARIES:
     - "ESP Async WebServer"  (ESP32Async / mathieucarbou)
     - "Async TCP"            (ESP32Async / mathieucarbou)

   PIN MAP (locked):
     Motor A : PWMA=4   AIN2=16  AIN1=17
     Motor B : PWMB=23  BIN1=18  BIN2=19
     STBY    : 5   (10k pulldown to GND)
     MPU6050 : SDA=21  SCL=22   (I2C default, 0x68)
     HC-SR04 : TRIG=26 ECHO=25  (not used yet)

   Connect phone WiFi to "BalanceBot" (pass: balance123),
   open http://192.168.4.1 -> sliders + live graph.
   Starts STOPPED; press GO to arm. Gains save to flash.
   ============================================================ */

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Preferences.h>

// ---------------- Mode ----------------
#define DIAGNOSTIC 0     // 1 = axis-finding (motors OFF). 0 = balance + WiFi tuner.

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

#define MPU_ADDR 0x68
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
#define PWM_CH_A 0
#define PWM_CH_B 1

// ---------------- Tuning (live-editable, saved to flash) ----------------
volatile float Kp = 20.0;
volatile float Ki = 0.0;
volatile float Kd = 0.8;
volatile float ANGLE_OFFSET = 0.0;
volatile int   MOTOR_DIR = 1;          // set -1 if wheels drive the wrong way
volatile bool  runEnable = false;      // web GO/STOP master switch (starts STOPPED)
const float FALL_LIMIT = 45.0;
const float ARM_WINDOW = 3.0;

// ---------------- State ----------------
float angle = 0;
float integral = 0;
unsigned long lastMicros = 0;
bool armed = false;

// ---------------- WiFi AP ----------------
const char* AP_SSID = "BalanceBot";
const char* AP_PASS = "balance123";    // must be >= 8 chars

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences prefs;

// ---------------- Web page ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>BalanceBot Tuner</title><style>
body{background:#11151a;color:#e6e6e6;font-family:system-ui,sans-serif;margin:0;padding:16px;max-width:520px;margin:auto}
h1{font-size:20px;margin:4px 0 10px}
#stat{font-size:15px;color:#9fd;margin-bottom:8px;min-height:20px}
canvas{width:100%;height:160px;background:#0b0e12;border-radius:10px;display:block}
.row{margin:14px 0}
label{display:flex;justify-content:space-between;font-size:14px;margin-bottom:4px}
input[type=range]{width:100%;height:30px}
button{width:100%;padding:14px;font-size:17px;border:0;border-radius:10px;margin:6px 0;color:#fff;background:#2a7;}
button.on{background:#c33}
#save{background:#357}
</style></head><body>
<h1>BalanceBot Tuner</h1>
<div id=stat>connecting…</div>
<canvas id=g width=480 height=160></canvas>
<button id=go>GO</button>
<div class=row><label>Kp <span id=vkp></span></label><input type=range id=kp min=0 max=80 step=0.5></div>
<div class=row><label>Ki <span id=vki></span></label><input type=range id=ki min=0 max=400 step=1></div>
<div class=row><label>Kd <span id=vkd></span></label><input type=range id=kd min=0 max=6 step=0.05></div>
<div class=row><label>Offset (deg) <span id=voff></span></label><input type=range id=off min=-12 max=12 step=0.1></div>
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
   document.getElementById('stat').textContent='angle '+a.toFixed(1)+'\u00B0  —  '+(curRun?(armed?'BALANCING':'waiting upright'):'STOPPED');
   buf.push(a);if(buf.length>MAXN)buf.shift();draw();
   const go=document.getElementById('go');go.textContent=curRun?'STOP':'GO';go.className=curRun?'on':'';}
  else if(d[0]=='c'){const p=d.split(';');setS('kp',p[1]);setS('ki',p[2]);setS('kd',p[3]);setS('off',p[4]);}};
 ws.onclose=()=>setTimeout(connect,1000);}
['kp','ki','kd','off'].forEach(id=>{const e=document.getElementById(id);
 e.oninput=()=>{document.getElementById('v'+id).textContent=(+e.value).toFixed(2);send(id,e.value);};});
document.getElementById('go').onclick=()=>send('run',curRun?0:1);
document.getElementById('save').onclick=()=>{if(ws&&ws.readyState==1)ws.send('save');};
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
//  Preferences (save / load gains)
// ============================================================
void loadPrefs(){
  prefs.begin("balance", true);
  Kp=prefs.getFloat("kp",Kp); Ki=prefs.getFloat("ki",Ki);
  Kd=prefs.getFloat("kd",Kd); ANGLE_OFFSET=prefs.getFloat("off",ANGLE_OFFSET);
  prefs.end();
}
void savePrefs(){
  prefs.begin("balance", false);
  prefs.putFloat("kp",(float)Kp); prefs.putFloat("ki",(float)Ki);
  prefs.putFloat("kd",(float)Kd); prefs.putFloat("off",(float)ANGLE_OFFSET);
  prefs.end();
}

// ============================================================
//  WebSocket
// ============================================================
void handleMsg(const String &m){
  int c=m.indexOf(':');
  if(c<0){ if(m=="save") savePrefs(); return; }
  String k=m.substring(0,c); float v=m.substring(c+1).toFloat();
  if(k=="kp")Kp=v; else if(k=="ki")Ki=v; else if(k=="kd")Kd=v;
  else if(k=="off")ANGLE_OFFSET=v; else if(k=="run")runEnable=(v!=0);
}
void onWsEvent(AsyncWebSocket*s,AsyncWebSocketClient*client,AwsEventType type,void*arg,uint8_t*data,size_t len){
  if(type==WS_EVT_CONNECT){
    String cfg="c;"+String((float)Kp,2)+";"+String((float)Ki,2)+";"+String((float)Kd,2)+";"+String((float)ANGLE_OFFSET,2);
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
//  Setup
// ============================================================
void setup(){
  Serial.begin(115200);
  pinMode(STBY,OUTPUT); digitalWrite(STBY,LOW);
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT); pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT);
  pwmSetup();
  motorA(0); motorB(0);
  pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);

  Wire.begin(); Wire.setClock(400000); mpuInit();
  int16_t ax,ay,az,gx,gy,gz; mpuRead(ax,ay,az,gx,gy,gz);
  angle = atan2((float)az, -(float)ax)*57.29578;

  loadPrefs();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());  // 192.168.4.1

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest*r){ r->send_P(200,"text/html",PAGE); });
  server.begin();

  lastMicros=micros();
}

// ============================================================
//  Loop
// ============================================================
void loop(){
  unsigned long now=micros();
  float dt=(now-lastMicros)/1000000.0;
  if(dt<0.003) return;
  lastMicros=now;

  int16_t ax,ay,az,gx,gy,gz; mpuRead(ax,ay,az,gx,gy,gz);

#if DIAGNOSTIC
  motorsEnable(false); motorA(0); motorB(0);
  static unsigned long lastRaw=0;
  if(millis()-lastRaw>100){ lastRaw=millis();
    Serial.print("ax=");Serial.print(ax);Serial.print(" ay=");Serial.print(ay);Serial.print(" az=");Serial.print(az);
    Serial.print(" | gx=");Serial.print(gx);Serial.print(" gy=");Serial.print(gy);Serial.print(" gz=");Serial.println(gz);}
  return;
#else

  // [AXIS] set from diagnostics: gravity=X, tilt=Z, rotation=Y
  float accAngle = atan2((float)az, -(float)ax) * 57.29578;
  float gyroRate = -gy / 131.0;

  angle = 0.98*(angle + gyroRate*dt) + 0.02*accAngle;

  if(!runEnable){
    armed=false; integral=0; motorsEnable(false); motorA(0); motorB(0);
  } else if(fabs(angle)>FALL_LIMIT){
    armed=false; integral=0; motorsEnable(false); motorA(0); motorB(0);
  } else if(!armed && fabs(angle-ANGLE_OFFSET)<ARM_WINDOW){
    armed=true; integral=0; motorsEnable(true);
  }

  if(runEnable && armed){
    float error=ANGLE_OFFSET-angle;
    integral+=error*dt; integral=constrain(integral,-200,200);
    float output=Kp*error + Ki*integral - Kd*gyroRate;
    output=constrain(output,-255,255);
    int drive=(int)(MOTOR_DIR*output);
    motorA(drive); motorB(drive);
  } else if(runEnable){
    motorA(0); motorB(0);
  }

  static unsigned long lastWs=0;
  if(millis()-lastWs>40){ lastWs=millis();
    ws.cleanupClients();
    String m="s;"+String(angle,2)+";"+String(runEnable?1:0)+";"+String(armed?1:0);
    ws.textAll(m);
  }
#endif
}