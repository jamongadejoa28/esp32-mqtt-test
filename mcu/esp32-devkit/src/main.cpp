#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "driver/mcpwm.h" // ESP32 전용 MCPWM 모듈 사용

// 서보 모터 제어를 LEDC가 아닌 MCPWM 모듈로 완전히 분리하여
// DC 모터의 LEDC 타이머와 발생하는 하드웨어 충돌(글리치) 원천 차단

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

// ========================
// 핀 설정
// ========================
#define SENSOR_PIN 34    // (왼쪽 입력 전용 핀 구역으로 이동)
#define SERVO_1_PIN 32   // 서보 1 (보드 왼쪽 구역으로 이동)
#define SERVO_2_PIN 33   // 서보 2 (보드 왼쪽 구역으로 이동)

// DC모터 핀 설정 (A4950)
// Slow Decay 제어: IN1=HIGH 고정, IN2에 PWM
#define DC_IN1_PIN 27  // A4950 IN1 (HIGH 고정)
#define DC_IN2_PIN 12  // A4950 IN2 (PWM 속도 제어)

// ========================
// LEDC 채널 (DC 모터 전용)
// ========================
const int DC_CHANNEL      = 0;
const int DC_FREQ          = 5000;
const int DC_RESOLUTION    = 8;     // 0~255

// 서보 제어는 MCPWM을 사용하므로 채널/타이머 할당 불필요
const int SERVO_RESOLUTION = 16;    // 0~65535 (정밀 제어)
const int SERVO_MIN_US     = 544;   // 0도 펄스폭 (µs)
const int SERVO_MAX_US     = 2400;  // 180도 펄스폭 (µs)

// 서보 설정
const int SERVO_CENTER = 90;   // 중립 위치 (도)
const int SERVO_OFFSET = 60;   // 중심에서 좌우 이동 각도

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// 모터 상태 제어
enum MotorState { IDLE, WAITING_AT_POSITION, WAITING_AT_ORIGIN };
MotorState motorState = IDLE;
unsigned long waitStartTime = 0;

// 센서 처리용
unsigned long lastSensorDetectTime = 0;
const unsigned long SENSOR_TIMEOUT = 1000;

// ========================
// 서보 헬퍼 함수 (MCPWM 통신)
// ========================
void servo1Write(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  uint32_t pulseUs = SERVO_MIN_US + (uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle / 180;
  // MCPWM 모듈의 A채널(서보1)로 펄스폭 전송
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulseUs);
}

void servo2Write(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  uint32_t pulseUs = SERVO_MIN_US + (uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle / 180;
  // MCPWM 모듈의 B채널(서보2)로 펄스폭 전송
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, pulseUs);
}

void servoStopAll() {
  // 펄스폭을 0으로 만들어 두 서보 위치 유지(보정) 끄고 잔떨림 방지
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
}

// ========================
// MQTT 콜백
// ========================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.printf("[MQTT] %s : %s\n", topic, message.c_str());

  if (String(topic) == MQTT_TOPIC_SUB_CONTROL) {
    if (message == "MOVE_RIGHT" && motorState == IDLE) {
      Serial.println("[ACTION] Servo Motor Right");
      servo1Write(SERVO_CENTER + SERVO_OFFSET); // 서보 1, 2 모두 움직인다고 가정 (필요시 방향 조절)
      servo2Write(SERVO_CENTER + SERVO_OFFSET);
      motorState = WAITING_AT_POSITION;
      waitStartTime = millis();
    } else if (message == "MOVE_LEFT" && motorState == IDLE) {
      Serial.println("[ACTION] Servo Motor Left");
      servo1Write(SERVO_CENTER - SERVO_OFFSET);
      servo2Write(SERVO_CENTER - SERVO_OFFSET);
      motorState = WAITING_AT_POSITION;
      waitStartTime = millis();
    } else if (message == "MOVE" && motorState == IDLE) {
      servo1Write(SERVO_CENTER + SERVO_OFFSET);
      servo2Write(SERVO_CENTER + SERVO_OFFSET);
      motorState = WAITING_AT_POSITION;
      waitStartTime = millis();
    } else if (message.startsWith("DC_START:")) {
      int speed = message.substring(9).toInt();
      if (speed < 0) speed = 0;
      if (speed > 255) speed = 255;

      // A4950 Slow Decay: IN1=HIGH(고정), IN2=PWM
      // PWM LOW → Forward, PWM HIGH → Brake
      ledcWrite(DC_CHANNEL, 255 - speed);
      Serial.printf("[ACTION] DC Motor Started at speed %d\n", speed);
    } else if (message == "DC_STOP") {
      ledcWrite(DC_CHANNEL, 255);  // (1,1) Brake
      Serial.println("[ACTION] DC Motor Stopped");
    } else if (message == "SENSOR_SIMULATE") {
      if (motorState != IDLE) {
        Serial.println("[BLOCKED] Sensor ignored: servo in motion");
      } else if (millis() - lastSensorDetectTime > SENSOR_TIMEOUT) {
        Serial.println("[EVENT] Object Detected by Sensor (Simulated)!");
        mqttClient.publish(MQTT_TOPIC_PUB_SENSOR, "DETECTED");
        lastSensorDetectTime = millis();

        // 서보 모터 열림 동작 시작
        servo1Write(SERVO_CENTER + SERVO_OFFSET);
        servo2Write(SERVO_CENTER + SERVO_OFFSET);
        motorState = WAITING_AT_POSITION;
        waitStartTime = millis();
        Serial.println("[ACTION] Servo opening (sensor triggered)");
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

  // ★ 부팅 직후: IN1, IN2 모두 HIGH → (1,1) Brake 강제
  pinMode(DC_IN1_PIN, OUTPUT);
  pinMode(DC_IN2_PIN, OUTPUT);
  digitalWrite(DC_IN1_PIN, HIGH);
  digitalWrite(DC_IN2_PIN, HIGH);

  // DC 모터: LEDC 채널 0 (타이머 내부 자동할당), 5000Hz, 8bit
  ledcSetup(DC_CHANNEL, DC_FREQ, DC_RESOLUTION);
  ledcAttachPin(DC_IN2_PIN, DC_CHANNEL);
  ledcWrite(DC_CHANNEL, 255);  // Brake

  // ★ 서보 2개: 완전히 독립된 MCPWM 모듈 사용 (채널 A, B)
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_1_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, SERVO_2_PIN);
  
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 50;    // 서보 표준 50Hz
  pwm_config.cmpr_a = 0;        // 시작 시 duty 0 (정지)
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  // 두 모터 모두 중립 위치로 시작 초기화
  servo1Write(SERVO_CENTER);  
  servo2Write(SERVO_CENTER);

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

  // ==== 서보 모터 시나리오 ====
  if (motorState == WAITING_AT_POSITION && (millis() - waitStartTime >= 2000)) {
    // 2초 후 서보 중립 복귀
    servo1Write(SERVO_CENTER);
    servo2Write(SERVO_CENTER);
    motorState = WAITING_AT_ORIGIN;
    waitStartTime = millis();
    Serial.println("[ACTION] Servo returning to center");
  } else if (motorState == WAITING_AT_ORIGIN && (millis() - waitStartTime >= 1000)) {
    // 서보 PWM 신호 중단 → 잔떨림 방지
    servoStopAll();
    motorState = IDLE;
    Serial.println("[ACTION] Servo sequence complete");

    // [QR 자동정지 시나리오 - 임시 주석]
    // ledcWrite(DC_CHANNEL, 255);
    // mqttClient.publish(MQTT_TOPIC_PUB_SENSOR, "DC_STOPPED");
    // Serial.println("[ACTION] Scenario Complete: DC Motor Auto-Stopped");
  }
}
