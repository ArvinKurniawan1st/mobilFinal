#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

/* ================= PIN ================= */
#define ENA 25
#define ENB 33
#define IN1 26
#define IN2 27
#define IN3 12
#define IN4 13

#define ENC_L 32
#define ENC_R 35

#define TRIG_PIN 4
#define ECHO_PIN 5

#define BUZZER_PIN 14

/* ================= PARAM ================= */
#define PWM_FREQ 20000
#define PWM_RES  8

#define BASE_PWM 180
#define TURN_PWM 150
#define BACK_PWM 150

#define OBSTACLE_CM 25
#define CLEAR_CM    35

#define TURN_TOL 3.0
#define STOP_TOL_CM 3.0

#define PULSE_PER_REV 506
#define WHEEL_DIAMETER_CM 6.5
#define CM_PER_PULSE ((3.1416 * WHEEL_DIAMETER_CM) / PULSE_PER_REV)

/* ================= WIFI & MQTT ================= */
const char* WIFI_SSID = "XXX";
const char* WIFI_PASS = "Abcdefghijklmn";

const char* MQTT_HOST = "3d334b6b47764fffb7480823a8402c8c.s1.eu.hivemq.cloud";
const int MQTT_PORT = 8883;
const char* MQTT_USER = "mobil";
const char* MQTT_PASS = "Password07";

WiFiClientSecure net;
PubSubClient mqtt(net);

/* ================= SENSOR ================= */
QMC5883LCompass compass;

/* ================= STATE ================= */
enum Mode { MANUAL, AUTO };
enum AutoState { IDLE, TURN, DRIVE, AVOID };

Mode mode = MANUAL;
AutoState autoState = IDLE;

/* ================= ENCODER ================= */
volatile long encL = 0, encR = 0;

/* ================= POSITION ================= */
float posX = 0, posY = 0;
float lastDist = 0;

/* ================= TARGET ================= */
float targetX = 0, targetY = 0;
float targetHeading = 0;
float targetDistanceCM = 0;

/* ================= BUZZER ================= */
bool buzzerManual = false;
bool buzzerAuto   = false;

/* ================= TELEMETRY TIMER ================= */
unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_INTERVAL = 500; // Send every 500ms

/* ================= ISR ================= */
void IRAM_ATTR encL_ISR(){ encL++; }
void IRAM_ATTR encR_ISR(){ encR++; }

/* ================= MOTOR ================= */
void motorStop(){ 
  ledcWrite(ENA,0); 
  ledcWrite(ENB,0); 
}

void motorForward(int p){
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
  ledcWrite(ENA,p); ledcWrite(ENB,p);
}

void motorBackward(int p){
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);
  ledcWrite(ENA,p); ledcWrite(ENB,p);
}

void motorLeft(int p){
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
  ledcWrite(ENA,p); ledcWrite(ENB,p);
}

void motorRight(int p){
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  ledcWrite(ENA,p); ledcWrite(ENB,p);
}

/* ================= SENSOR ================= */
float heading(){
  compass.read();
  int h = compass.getAzimuth();
  if(h < 0) h += 360;
  return h;
}

float headingErr(float t, float c){
  float e = t - c;
  if(e > 180) e -= 360;
  if(e < -180) e += 360;
  return e;
}

float distanceCM(){
  return ((encL + encR)/2.0) * CM_PER_PULSE;
}

long ultrasonic(){
  digitalWrite(TRIG_PIN,LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN,HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN,LOW);
  long d = pulseIn(ECHO_PIN,HIGH,25000);
  if(d==0) return 999;
  return d*0.034/2;
}

/* ================= POSITION UPDATE ================= */
void updatePosition(){
  float d = distanceCM();
  float delta = d - lastDist;
  lastDist = d;

  float rad = heading() * DEG_TO_RAD;
  posX += delta * cos(rad);
  posY += delta * sin(rad);
}

/* ================= SEND TELEMETRY ================= */
void sendTelemetry(){
  unsigned long now = millis();
  if(now - lastTelemetry < TELEMETRY_INTERVAL) return;
  lastTelemetry = now;

  StaticJsonDocument<256> doc;
  doc["x"] = posX / 100.0; // Convert to meters
  doc["y"] = posY / 100.0; // Convert to meters
  doc["heading"] = heading();
  doc["speed"] = 0; // Calculate from encoder delta if needed
  doc["front_distance"] = ultrasonic();
  doc["mode"] = (mode == MANUAL) ? "manual" : "auto";
  
  char buf[256];
  serializeJson(doc, buf);
  mqtt.publish("mobil/telemetry", buf);
}

/* ================= CALCULATE TARGET ================= */
void calculateTarget(float tx, float ty){
  // Calculate angle and distance from current position to target
  float dx = tx * 100 - posX; // Convert target to cm
  float dy = ty * 100 - posY;
  
  targetDistanceCM = sqrt(dx*dx + dy*dy);
  targetHeading = atan2(dy, dx) * 180 / PI;
  
  if(targetHeading < 0) targetHeading += 360;
  
  targetX = tx;
  targetY = ty;
}

/* ================= MQTT CALLBACK ================= */
void callback(char* topic, byte* payload, unsigned int len){
  StaticJsonDocument<256> doc;
  if(deserializeJson(doc,payload,len)) return;

  String t = topic;

  if(t == "mobil/command"){
    mode = MANUAL;
    autoState = IDLE;

    String c = doc["cmd"];
    int p = doc["speed"] | BASE_PWM;

    if(c == "forward") motorForward(p);
    else if(c == "backward") motorBackward(p);
    else if(c == "left") motorLeft(p);
    else if(c == "right") motorRight(p);
    else if(c == "stop") motorStop();
    else motorStop();
  }

  if(t == "mobil/mode"){
    String m = doc["mode"];
    if(m == "manual"){
      mode = MANUAL;
      autoState = IDLE;
      motorStop();
    }
  }

  if(t == "mobil/waypoint"){
    mode = AUTO;
    autoState = TURN;

    float tx = doc["x"];
    float ty = doc["y"];
    
    calculateTarget(tx, ty);
    
    encL = encR = 0;
    lastDist = 0;
    
    Serial.print("New waypoint: (");
    Serial.print(tx);
    Serial.print(", ");
    Serial.print(ty);
    Serial.print(") - Distance: ");
    Serial.print(targetDistanceCM);
    Serial.print(" cm, Heading: ");
    Serial.println(targetHeading);
  }

  if(t == "mobil/buzzer"){
    buzzerManual = (doc["state"] == "on");
  }
}

/* ================= SETUP ================= */
void setup(){
  Serial.begin(115200);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);
  pinMode(BUZZER_PIN,OUTPUT);

  ledcAttach(ENA,PWM_FREQ,PWM_RES);
  ledcAttach(ENB,PWM_FREQ,PWM_RES);

  pinMode(ENC_L,INPUT);
  pinMode(ENC_R,INPUT);
  attachInterrupt(ENC_L,encL_ISR,RISING);
  attachInterrupt(ENC_R,encR_ISR,RISING);

  Wire.begin(21,19);
  compass.init();

  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID,WIFI_PASS);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  net.setInsecure();
  mqtt.setServer(MQTT_HOST,MQTT_PORT);
  mqtt.setCallback(callback);
  mqtt.setKeepAlive(60);
}

/* ================= RECONNECT MQTT ================= */
void reconnectMQTT(){
  while(!mqtt.connected()){
    Serial.print("Connecting to MQTT...");
    if(mqtt.connect("ESP32_FINAL", MQTT_USER, MQTT_PASS)){
      Serial.println("Connected!");
      mqtt.subscribe("mobil/command");
      mqtt.subscribe("mobil/mode");
      mqtt.subscribe("mobil/waypoint");
      mqtt.subscribe("mobil/buzzer");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" Retrying in 5s...");
      delay(5000);
    }
  }
}

/* ================= LOOP ================= */
void loop(){
  if(!mqtt.connected()){
    reconnectMQTT();
  }
  mqtt.loop();

  long obs = ultrasonic();

  buzzerAuto = (mode == AUTO && obs < OBSTACLE_CM);
  digitalWrite(BUZZER_PIN, buzzerAuto || buzzerManual);

  // Send telemetry periodically
  sendTelemetry();

  if(mode != AUTO) return;

  if(autoState == DRIVE){
    updatePosition();
  }

  if(autoState == TURN){
    float e = headingErr(targetHeading, heading());
    if(abs(e) <= TURN_TOL){
      motorStop(); 
      encL = encR = 0; 
      lastDist = 0;
      autoState = DRIVE; 
      Serial.println("Turn complete, starting drive");
      return;
    }
    if(e > 0) motorRight(TURN_PWM);
    else motorLeft(TURN_PWM);
  }

  if(autoState == DRIVE){
    if(obs < OBSTACLE_CM){ 
      motorStop(); 
      autoState = AVOID;
      Serial.println("Obstacle detected!");
      return; 
    }
    
    float traveled = distanceCM();
    if(traveled >= targetDistanceCM - STOP_TOL_CM){
      motorStop(); 
      autoState = IDLE; 
      mode = MANUAL;
      Serial.println("Destination reached!");
      return;
    }
    motorForward(BASE_PWM);
  }

  if(autoState == AVOID){
    motorBackward(BACK_PWM); 
    delay(300);
    motorRight(TURN_PWM); 
    delay(400);
    motorStop();
    
    if(ultrasonic() > CLEAR_CM){
      autoState = DRIVE;
      Serial.println("Path clear, resuming");
    }
  }
}