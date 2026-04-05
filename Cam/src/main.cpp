#include <Arduino.h>
#include "esp_camera.h"

#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5

#define Y2_GPIO_NUM    11
#define Y3_GPIO_NUM    9
#define Y4_GPIO_NUM    8
#define Y5_GPIO_NUM    10
#define Y6_GPIO_NUM    12
#define Y7_GPIO_NUM    18
#define Y8_GPIO_NUM    17
#define Y9_GPIO_NUM    16

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

struct DetectionResult {
  bool found;
  int cx;
  int cy;
  int count;
};

bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;

  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_96X96;
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 1);
    s->set_contrast(s, 2);
  }

  return true;
}

DetectionResult detectDarkObject(camera_fb_t *fb) {
  DetectionResult r = {false, -1, -1, 0};

  uint8_t *img = fb->buf;
  const int w = fb->width;
  const int h = fb->height;

  const int x0 = 10, x1 = 86;
  const int y0 = 10, y1 = 86;
  const int threshold = 85;

  long sumX = 0, sumY = 0;
  int count = 0;

  for (int y = y0; y < y1; y++) {
    for (int x = x0; x < x1; x++) {
      uint8_t p = img[y * w + x];
      if (p < threshold) {
        sumX += x;
        sumY += y;
        count++;
      }
    }
  }

  int roiArea = (x1 - x0) * (y1 - y0);
  if (count < 180) return r;
  if (count > roiArea * 0.55) return r;

  r.found = true;
  r.cx = sumX / count;
  r.cy = sumY / count;
  r.count = count;
  return r;
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  if (!initCamera()) {
    Serial.println("CAMERA FAIL");
    while (true) delay(1000);
  }

  Serial.println("CAMERA READY");
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("NONE");
    delay(120);
    return;
  }

  DetectionResult d = detectDarkObject(fb);

  if (d.found) {
    Serial.print("X:");
    Serial.print(d.cx);
    Serial.print(",Y:");
    Serial.print(d.cy);
    Serial.print(",S:");
    Serial.println(d.count);
  } else {
    Serial.println("NONE");
  }

  esp_camera_fb_return(fb);
  delay(100);
}