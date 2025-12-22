#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* ===================== WIFI ===================== */
const char* WIFI_SSID = "XXX";
const char* WIFI_PASS = "Abcdefghijklmn";

/* ===================== MQTT ===================== */
const char* MQTT_HOST = "3d334b6b47764fffb7480823a8402c8c.s1.eu.hivemq.cloud";
const int   MQTT_PORT = 8883;
const char* MQTT_USER = "mobil";
const char* MQTT_PASS = "Password07";

#define TOPIC_MODE     "mobil/mode"
#define TOPIC_COMMAND  "mobil/command"
#define TOPIC_WAYPOINT "mobil/waypoint"

/* ===================== MOTOR PIN (L298N) ===================== */
#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 14
#define IN3 12
#define IN4 13

/* ===================== ENCODER ===================== */
#define ENC_L_A 34
#define ENC_L_B 35
#define ENC_R_A 32
#define ENC_R_B 33

volatile long encLeft = 0;
volatile long encRight = 0;

/* ===================== ULTRASONIC ===================== */
#define TRIG_PIN 4
#define ECHO_PIN 5

/* ===================== PWM ===================== */
#define PWM_FREQ 20000
#define PWM_RES  8

/* ===================== NAV PARAM ===================== */
const float PULSE_PER_METER = 810.0;
const float BASE_SPEED = 0.18;
const float HEADING_KP = 0.015;
const float TARGET_RADIUS = 0.20;

const float OBSTACLE_DIST = 30.0;
const int BACK_TIME = 300;
const int TURN_TIME = 400;

/* ===================== MODE & STATE ===================== */
enum Mode { MANUAL, AUTO };
Mode currentMode = MANUAL;

enum State {
  IDLE,
  MANUAL_DRIVE,
  AUTO_NAV,
  OBSTACLE_REACT,
  TARGET_REACHED
};
State state = IDLE;

/* ===================== ROBOT STATE ===================== */
struct {
  float x = 0;
  float y = 0;
  float heading = 0;
  float targetX = 0;
  float targetY = 0;
} mobil;

/* ===================== PID ===================== */
struct PID {
  float kp, ki, kd;
  float prevError;
  float integral;
};

PID pidL = {35, 4, 0.8, 0, 0};
PID pidR = {35, 4, 0.8, 0, 0};

float targetSpeedL = 0;
float targetSpeedR = 0;

unsigned long lastPID = 0;
const int PID_INTERVAL = 100;

/* ===================== MQTT ===================== */
WiFiClientSecure espClient;
PubSubClient mqtt(espClient);

/* ===================== COMPASS ===================== */
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);
float OFFSET_X = -32.4;
float OFFSET_Y = 15.8;

/* ===================== INTERRUPT ===================== */
void IRAM_ATTR encL_ISR() {
  encLeft += digitalRead(ENC_L_B) ? 1 : -1;
}
void IRAM_ATTR encR_ISR() {
  encRight += digitalRead(ENC_R_B) ? 1 : -1;
}

/* ===================== MOTOR ===================== */
void setMotorDir(bool lf, bool lb, bool rf, bool rb) {
  digitalWrite(IN1, lf);
  digitalWrite(IN2, lb);
  digitalWrite(IN3, rf);
  digitalWrite(IN4, rb);
}

void stopMotor() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
  targetSpeedL = 0;
  targetSpeedR = 0;
}

/* ===================== ULTRASONIC ===================== */
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long d = pulseIn(ECHO_PIN, HIGH, 25000);
  if (d == 0) return 999;
  return d * 0.034 / 2;
}

/* ===================== COMPASS ===================== */
float normalizeAngle(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

float readCompass() {
  sensors_event_t e;
  compass.getEvent(&e);

  float x = e.magnetic.x - OFFSET_X;
  float y = e.magnetic.y - OFFSET_Y;

  float h = atan2(y, x) * 180 / PI;
  if (h < 0) h += 360;
  return h;
}

/* ===================== ODOMETRY ===================== */
long lastEncL = 0, lastEncR = 0;

void updateOdometry() {
  long dL = encLeft - lastEncL;
  long dR = encRight - lastEncR;
  lastEncL = encLeft;
  lastEncR = encRight;

  float dist = ((dL + dR) / 2.0) / PULSE_PER_METER;
  mobil.x += dist * cos(mobil.heading * DEG_TO_RAD);
  mobil.y += dist * sin(mobil.heading * DEG_TO_RAD);
}

/* ===================== PID ===================== */
int computePID(PID &pid, float target, float current) {
  float error = target - current;
  pid.integral += error * (PID_INTERVAL / 1000.0);
  pid.integral = constrain(pid.integral, -50, 50);

  float d = (error - pid.prevError) / (PID_INTERVAL / 1000.0);
  pid.prevError = error;

  float out = pid.kp * error + pid.ki * pid.integral + pid.kd * d;
  return constrain(out, 0, 255);
}

void updatePID() {
  if (currentMode != AUTO) return;
  if (millis() - lastPID < PID_INTERVAL) return;
  lastPID = millis();

  static long prevL = 0, prevR = 0;
  float speedL = (encLeft - prevL) / PULSE_PER_METER / (PID_INTERVAL / 1000.0);
  float speedR = (encRight - prevR) / PULSE_PER_METER / (PID_INTERVAL / 1000.0);
  prevL = encLeft;
  prevR = encRight;

  ledcWrite(ENA, computePID(pidL, targetSpeedL, speedL));
  ledcWrite(ENB, computePID(pidR, targetSpeedR, speedR));
}

/* ===================== AUTO NAV ===================== */
bool lastTurnLeft = true;

void autoNavigation() {
  if (readUltrasonic() < OBSTACLE_DIST) {
    stopMotor();
    state = OBSTACLE_REACT;
    return;
  }

  float dx = mobil.targetX - mobil.x;
  float dy = mobil.targetY - mobil.y;

  float dist = sqrt(dx*dx + dy*dy);
  if (dist < TARGET_RADIUS) {
    stopMotor();
    state = TARGET_REACHED;
    return;
  }

  float targetAngle = atan2(dy, dx) * 180 / PI;
  float err = normalizeAngle(targetAngle - mobil.heading);

  float diff = constrain(HEADING_KP * err, -0.1, 0.1);

  targetSpeedL = BASE_SPEED - diff;
  targetSpeedR = BASE_SPEED + diff;

  setMotorDir(HIGH, LOW, LOW, HIGH);
}

void obstacleReact() {
  setMotorDir(LOW, HIGH, LOW, HIGH);
  ledcWrite(ENA, 120);
  ledcWrite(ENB, 120);
  delay(BACK_TIME);
  stopMotor();

  bool turnLeft = lastTurnLeft;
  lastTurnLeft = !lastTurnLeft;

  if (turnLeft)
    setMotorDir(LOW, HIGH, LOW, HIGH);
  else
    setMotorDir(HIGH, LOW, HIGH, LOW);

  ledcWrite(ENA, 120);
  ledcWrite(ENB, 120);
  delay(TURN_TIME);
  stopMotor();

  state = AUTO_NAV;
}

/* ===================== MQTT CALLBACK ===================== */
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  StaticJsonDocument<256> doc;
  deserializeJson(doc, payload, len);

  if (strcmp(topic, TOPIC_MODE) == 0) {
    currentMode = doc["mode"] == "auto" ? AUTO : MANUAL;
    stopMotor();
    state = (currentMode == AUTO) ? AUTO_NAV : MANUAL_DRIVE;
  }

  if (strcmp(topic, TOPIC_WAYPOINT) == 0) {
    mobil.targetX = doc["x"];
    mobil.targetY = doc["y"];
    state = AUTO_NAV;
  }

  if (strcmp(topic, TOPIC_COMMAND) == 0 && currentMode == MANUAL) {
    String cmd = doc["cmd"];
    int pwm = constrain(doc["speed"], 0, 255);

    if (cmd == "forward") setMotorDir(HIGH, LOW, LOW, HIGH);
    else if (cmd == "backward") setMotorDir(LOW, HIGH, HIGH, LOW);
    else if (cmd == "left") setMotorDir(LOW, HIGH, LOW, HIGH);
    else if (cmd == "right") setMotorDir(HIGH, LOW, HIGH, LOW);
    else { stopMotor(); return; }

    ledcWrite(ENA, pwm);
    ledcWrite(ENB, pwm);
  }
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(ENC_L_A, encL_ISR, CHANGE);
  attachInterrupt(ENC_R_A, encR_ISR, CHANGE);

  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  compass.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  espClient.setInsecure();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
}

/* ===================== LOOP ===================== */
void loop() {
  if (!mqtt.connected()) {
    mqtt.connect("ESP32_mobil", MQTT_USER, MQTT_PASS);
    mqtt.subscribe(TOPIC_MODE);
    mqtt.subscribe(TOPIC_WAYPOINT);
    mqtt.subscribe(TOPIC_COMMAND);
  }

  mqtt.loop();

  mobil.heading = readCompass();
  updateOdometry();

  if (currentMode == AUTO) updatePID();

  if (state == AUTO_NAV) autoNavigation();
  else if (state == OBSTACLE_REACT) obstacleReact();
}
