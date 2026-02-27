#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <AccelStepper.h>

// .env 파일에서 빌드 시 자동 주입됩니다 (env_script.py)
#ifndef WIFI_SSID
  #error ".env file is missing or env_script.py failed to inject WIFI_SSID"
#endif
#ifndef WIFI_PASS
  #error ".env file is missing or env_script.py failed to inject WIFI_PASS"
#endif
#ifndef MQTT_BROKER
  #error ".env file is missing or env_script.py failed to inject MQTT_BROKER"
#endif

const int MQTT_PORT = 1883;
const char* MQTT_TOPIC_PUB_SENSOR = "device/sensor";
const char* MQTT_TOPIC_SUB_CONTROL = "device/control";

// 핀 설정
#define SENSOR_PIN 13
#define IN1 26
#define IN2 25
#define IN3 33
#define IN4 32

// DC모터 핀 설정 (L298N 등)
#define DC_EN_PIN 27 // PWM 제어 핀 (IN1 혹은 EN)
#define DC_DIR_PIN 12 // 방향 제어 핀 (IN2 혹은 DIR)

AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// 모터 상태 제어
enum MotorState { IDLE, MOVING_FORWARD, MOVING_REVERSE, WAITING, MOVING_BACKWARD, SORT_DONE_WAITING };
MotorState motorState = IDLE;
unsigned long waitStartTime = 0;
const int STEPS_60_DEG = 1200; // 4096 기준 약 100도 가량 열리도록 범위 확대

// 센서 처리용
unsigned long lastSensorDetectTime = 0;
const unsigned long SENSOR_TIMEOUT = 1000; // 1초 이내 중복 감지 방지 (시뮬레이션용)

// DC PWM 설정
const int pwmChannel = 0;
const int pwmFreq = 5000;
const int pwmResolution = 8;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.printf("[MQTT] %s : %s\n", topic, message.c_str());

  if (String(topic) == MQTT_TOPIC_SUB_CONTROL) {
    if (message == "MOVE_RIGHT" && motorState == IDLE) {
      Serial.println("[ACTION] Stepper Motor Forward (Right)");
      stepper.setCurrentPosition(0);
      stepper.moveTo(STEPS_60_DEG);
      motorState = MOVING_FORWARD;
    } else if (message == "MOVE_LEFT" && motorState == IDLE) {
      Serial.println("[ACTION] Stepper Motor Reverse (Left)");
      stepper.setCurrentPosition(0);
      stepper.moveTo(-STEPS_60_DEG);
      motorState = MOVING_REVERSE;
    } else if (message == "MOVE" && motorState == IDLE) { // 기본 호환성 (우측 이동 유지)
      stepper.setCurrentPosition(0);
      stepper.moveTo(STEPS_60_DEG);
      motorState = MOVING_FORWARD;
    } else if (message.startsWith("DC_START:")) {
      int speed = message.substring(9).toInt();
      if (speed < 0) speed = 0;
      if (speed > 255) speed = 255;
      
      digitalWrite(DC_DIR_PIN, LOW);
      ledcWrite(pwmChannel, speed);
      Serial.printf("[ACTION] DC Motor Started at speed %d\n", speed);
    } else if (message == "DC_STOP") {
      ledcWrite(pwmChannel, 0);
      Serial.println("[ACTION] DC Motor Stopped");
    } else if (message == "SENSOR_SIMULATE") {
      if (millis() - lastSensorDetectTime > SENSOR_TIMEOUT) {
        Serial.println("[EVENT] Object Detected by Sensor (Simulated)!");
        mqttClient.publish(MQTT_TOPIC_PUB_SENSOR, "DETECTED");
        lastSensorDetectTime = millis();
      }
    }
  }
}

void reconnectMQTT() {
  if (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32DevKit-";
    clientId += String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
      mqttClient.subscribe(MQTT_TOPIC_SUB_CONTROL);
    } else {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
    }
  }
}

void setup() {
  Serial.begin(115200);

  // DC 모터 설정
  pinMode(DC_DIR_PIN, OUTPUT);
  digitalWrite(DC_DIR_PIN, LOW);
  
  // 구버전 ESP32 Arduino Core 호환 PWM (현재 PIO 기본값)
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(DC_EN_PIN, pwmChannel);
  ledcWrite(pwmChannel, 0);

  stepper.setMaxSpeed(4000.0);
  stepper.setAcceleration(2500.0);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.printf("\n[OK] WiFi Connected! IP: %s\n", WiFi.localIP().toString().c_str());

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setSocketTimeout(15);
  mqttClient.setCallback(mqttCallback);
}

void loop() {
  if (!mqttClient.connected()) {
    static unsigned long lastReconnect = 0;
    if (millis() - lastReconnect > 5000) {
      reconnectMQTT();
      lastReconnect = millis();
    }
  } else {
    mqttClient.loop();
  }

  // ==== 1. 스텝모터 비동기 제어 및 DC 자동 정지 시나리오 ====
  stepper.run();
  
  if ((motorState == MOVING_FORWARD || motorState == MOVING_REVERSE) && stepper.distanceToGo() == 0) {
    motorState = WAITING;
    waitStartTime = millis();
    Serial.println("[ACTION] Waiting 2 seconds before returning...");
  } else if (motorState == WAITING && (millis() - waitStartTime >= 2000)) {
    stepper.moveTo(0); // 원위치 복귀
    motorState = MOVING_BACKWARD;
    Serial.println("[ACTION] Stepper Motor Backward origin");
  } else if (motorState == MOVING_BACKWARD && stepper.distanceToGo() == 0) {
    motorState = SORT_DONE_WAITING;
    waitStartTime = millis(); // 1초 대기 시작
    Serial.println("[ACTION] Stepper Motor Idle, waiting 1 sec to stop DC...");
  } else if (motorState == SORT_DONE_WAITING && (millis() - waitStartTime >= 1000)) {
    motorState = IDLE;
    // 1초 후 DC 모터 정지 및 GUI로 상태 전송
    ledcWrite(pwmChannel, 0);
    mqttClient.publish(MQTT_TOPIC_PUB_SENSOR, "DC_STOPPED");
    Serial.println("[ACTION] Scenario Complete: DC Motor Auto-Stopped");
  }
}
