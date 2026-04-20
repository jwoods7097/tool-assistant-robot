#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

const char* ssid = "Lemonade";
const char* password = "DontWorryBoutIT";

// Hiwonder ESP32S3 EYE pins
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5
#define Y2_GPIO_NUM    11
#define Y3_GPIO_NUM     9
#define Y4_GPIO_NUM     8
#define Y5_GPIO_NUM    10
#define Y6_GPIO_NUM    12
#define Y7_GPIO_NUM    18
#define Y8_GPIO_NUM    17
#define Y9_GPIO_NUM    16
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM  13

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    
    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;
    bool converted = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
    esp_camera_fb_return(fb);
    
    if (!converted) { res = ESP_FAIL; break; }
    
    size_t hlen = snprintf(part_buf, 64,
      "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", jpg_len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)jpg_buf, jpg_len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n", 2);
    free(jpg_buf);
    if (res != ESP_OK) break;
  }
  return res;
}
  

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== BOOTING ===");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 2;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return;
  }
  Serial.println("Camera OK!");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi failed!");
    return;
  }
  Serial.println("\nWiFi connected!");
  Serial.print("Stream URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream");

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  httpd_uri_t uri = {
    .uri     = "/stream",
    .method  = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };
  if (httpd_start(&stream_httpd, &cfg) == ESP_OK)
    httpd_register_uri_handler(stream_httpd, &uri);

  Serial.println("Stream server started!");
}

void loop() { delay(1); }