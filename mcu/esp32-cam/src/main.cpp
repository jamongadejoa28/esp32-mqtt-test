#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"

// 브라운아웃(전압 강하에 의한 재부팅 루프) 방지용 헤더
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// .env 파일에서 빌드 시 자동 주입됩니다 (env_script.py)
#ifndef WIFI_SSID
  #error ".env file is missing or env_script.py failed to inject WIFI_SSID"
#endif
#ifndef WIFI_PASS
  #error ".env file is missing or env_script.py failed to inject WIFI_PASS"
#endif
#ifndef UDP_IP
  #error ".env file is missing or env_script.py failed to inject UDP_IP"
#endif

const int UDP_PORT = 8021;

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

WiFiUDP udp;

void setup() {
  // 카메라 등 전원 인가 시 전압 강하가 심해 발생하는 무한 재부팅 루프를 방지합니다.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(1000);

  camera_config_t camera_config = {};
  camera_config.ledc_channel = LEDC_CHANNEL_0;
  camera_config.ledc_timer   = LEDC_TIMER_0;
  camera_config.pin_d0       = Y2_GPIO_NUM;
  camera_config.pin_d1       = Y3_GPIO_NUM;
  camera_config.pin_d2       = Y4_GPIO_NUM;
  camera_config.pin_d3       = Y5_GPIO_NUM;
  camera_config.pin_d4       = Y6_GPIO_NUM;
  camera_config.pin_d5       = Y7_GPIO_NUM;
  camera_config.pin_d6       = Y8_GPIO_NUM;
  camera_config.pin_d7       = Y9_GPIO_NUM;
  camera_config.pin_xclk     = XCLK_GPIO_NUM;
  camera_config.pin_pclk     = PCLK_GPIO_NUM;
  camera_config.pin_vsync    = VSYNC_GPIO_NUM;
  camera_config.pin_href     = HREF_GPIO_NUM;
  camera_config.pin_sccb_sda = SIOD_GPIO_NUM;
  camera_config.pin_sccb_scl = SIOC_GPIO_NUM;
  camera_config.pin_pwdn     = PWDN_GPIO_NUM;
  camera_config.pin_reset    = RESET_GPIO_NUM;
  camera_config.xclk_freq_hz = 20000000;
  camera_config.pixel_format = PIXFORMAT_JPEG;
  camera_config.grab_mode    = CAMERA_GRAB_LATEST;
  camera_config.fb_location  = CAMERA_FB_IN_PSRAM;
  
  camera_config.frame_size   = FRAMESIZE_QVGA; 
  camera_config.jpeg_quality = 12;
  camera_config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("[FAIL] Camera init error 0x%x\n", err);
    while(true) delay(1000);
  }
  Serial.println("[OK] Camera Initialized");
  
  sensor_t *s = esp_camera_sensor_get();
  if (s){ s->set_vflip(s, 1); }

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.printf("\n[OK] WiFi Connected! IP: %s\n", WiFi.localIP().toString().c_str());

  udp.begin(UDP_PORT);
  Serial.println("[READY] Sending UDP Stream...");
}

void sendImageUDP() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  const int MAX_UDP_PAYLOAD = 1024;
  int total_chunks = (fb->len + MAX_UDP_PAYLOAD - 1) / MAX_UDP_PAYLOAD;
  uint16_t image_id = (uint16_t)(millis() & 0xFFFF);
  
  char frame_type = 'S'; 

  for (int i = 0; i < total_chunks; i++) {
    udp.beginPacket(UDP_IP, UDP_PORT);
    
    udp.write((const uint8_t*)"IMG", 3);
    udp.write((const uint8_t*)&frame_type, 1);
    udp.write((const uint8_t*)&image_id, 2);
    udp.write((const uint8_t*)&total_chunks, 2);
    udp.write((const uint8_t*)&i, 2);

    int offset = i * MAX_UDP_PAYLOAD;
    int current_payload_size = fb->len - offset;
    if (current_payload_size > MAX_UDP_PAYLOAD) {
      current_payload_size = MAX_UDP_PAYLOAD;
    }

    udp.write(fb->buf + offset, current_payload_size);
    
    if (udp.endPacket()) {
      delay(15);
    } else {
      delay(30);
    }
  }

  esp_camera_fb_return(fb);
}

void loop() {
  sendImageUDP();
  delay(200);
}