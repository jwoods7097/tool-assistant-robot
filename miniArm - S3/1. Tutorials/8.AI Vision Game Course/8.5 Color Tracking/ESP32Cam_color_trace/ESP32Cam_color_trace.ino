#include "esp_camera.h"
// #include <WiFi.h>
#include <Wire.h>

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15 
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"
#include "find_color.h"

#define COLOR_NUM 2
int8_t color_threshold[COLOR_NUM][6] = {
  // {15,80,15,62,15,50} //, //red {15,80,15,62,15,35}
  // {14, 61, -39, -6, 0, 14},//Green
  // {21,50,-7,8,-35,-11} , //blue
  // {65, 78, -10, -5, 38, 50},//yellow
  // {20, 50, 17, 37, -34, -14}//purple

  // {21, 53, 7, 33, -10, 8} //EVA RED
  {43, 68, -26, -13, 4, 20}, //EVA GREEN
  // {49, 74, -7, 1, -20, -10} //EVA BLUE
  {43,91,-8,6,-27,-10} //EVA BLUE 2
};


struct COLOR_INFO_ST{
  uint8_t no_dect_count[COLOR_NUM];
  uint16_t dect_info[COLOR_NUM][4];
}color_info;

image_t img = {.bpp = 2};
list_t out = {NULL , NULL , 0 , 0};
rectangle_t roi = {0,0,320,240};
list_t thresholds;
unsigned long int time_clock = 0;

void py_helper_arg_to_thresholds(list_t* thresholds);

void receiveEvent(int howMany);
void requestEvent();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  // config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;
  config.frame_size = FRAMESIZE_QVGA; //FRAMESIZE_HQVGA; //FRAMESIZE_QQVGA; //FRAMESIZE_240X240;


  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);

  list_init(&thresholds, sizeof(color_thresholds_list_lnk_data_t));
  py_helper_arg_to_thresholds(&thresholds);

  ledcSetup(10, 1000, 8);  //设置LEDC通道8频率为1，分辨率为8位，即占空比可选0~255(Set the frequency of LEDC channel 8 to 1 and the resolution to 8 bits, allowing for a selectable duty cycle ranging from 0 to 255)
  ledcAttachPin(4, 10); //设置LEDC通道8在IO4上输出(configure LEDC channel 8 to output on IO4)
  ledcWrite(10, 0); //设置输出PWM(set output PWM)

  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Wire.setPins(14,13);
  Wire.begin((uint8_t)0x52);                // join I2C bus with address #0x52

  Serial.println("Color detection starts");
}

int lightness = 0;

void loop() {

  delay(10);
  time_clock = millis();

  if(lightness)
    ledcWrite(10, lightness); //设置输出PWM(set output PWM)
  else
    ledcWrite(10, 0);

  camera_fb_t* buf = esp_camera_fb_get();
  if(buf == NULL)
  {
    Serial.println("warn:image NULL!");
    return;
  }
  
  // Serial.print("a:");
  // Serial.println(millis() - time_clock);
  // time_clock = millis();

  img.data =  buf->buf;
  img.w = buf->width;
  img.h = buf->height;

  

  imlib_find_blobs(&out , &img , &roi , 2 , 1 , 
                  &thresholds , false , 50 , 80 ,
                  true , 10 , NULL , NULL , NULL , NULL );
  
  // Serial.print("b:");
  // Serial.println(millis() - time_clock);
  // time_clock = millis();

  if(buf != NULL)
  {
    esp_camera_fb_return(buf);
  }

  // Serial.print("c:");
  // Serial.println(millis() - time_clock);
  // time_clock = millis();

  while(list_size(&out))
  {
    find_blobs_list_lnk_data_t rec;
    list_pop_front(&out , &rec);
    int num = rec.code>>1;
    if(num < COLOR_NUM)
    {
      color_info.dect_info[num][0] = rec.rect.x;
      color_info.dect_info[num][1] = rec.rect.y;
      color_info.dect_info[num][2] = rec.rect.w;
      color_info.dect_info[num][3] = rec.rect.h;

      color_info.no_dect_count[num] = 0;

      Serial.print("color[");
      Serial.print(rec.code>>1);
      Serial.print("]");
      Serial.print(rec.rect.x);
      Serial.print(" ");
      // // Serial.print(rec.rect.y);
      // Serial.print(" ");
      Serial.println(rec.rect.w);
      // Serial.print(" ");
      // Serial.println(rec.rotation);
    }
  }

  // Serial.print("d:");
  // Serial.println(millis() - time_clock);
  // time_clock = millis();
  
  // for(int i = 0 ; i < COLOR_NUM ; i++)
  // {
  //   color_info.no_dect_count[i]++;
  //   if(color_info.no_dect_count[i] >= 4)
  //   {
  //     color_info.no_dect_count[i] = 0;
  //     color_info.dect_info[i][1] = 0;
  //     color_info.dect_info[i][2] = 0;
  //     color_info.dect_info[i][3] = 0;
  //     color_info.dect_info[i][4] = 0;
  //   }
  // }
  

  if(out.head_ptr != NULL)
  {
    Serial.print("color:");
    Serial.println(list_size(&out));
    //需要把结果链表删除(the result linked list needs to be deleted)
    list_free(&out);
  }
  // Serial.print("[cuzn]FPS:");
  // Serial.println(1000 / (millis() - time_clock));
  // Serial.println(millis() - time_clock);
  
}



void py_helper_arg_to_thresholds(list_t* thresholds)
{
  color_thresholds_list_lnk_data_t lnk_data;
  for(int i = 0 ; i < COLOR_NUM ; i++)
  {
    lnk_data.LMin = IM_MAX(IM_MIN(color_threshold[i][0],IM_MAX(COLOR_L_MAX, COLOR_GRAYSCALE_MAX)), IM_MIN(COLOR_L_MIN, COLOR_GRAYSCALE_MIN));

    lnk_data.LMax = IM_MAX(IM_MIN(color_threshold[i][1],IM_MAX(COLOR_L_MAX, COLOR_GRAYSCALE_MAX)), IM_MIN(COLOR_L_MIN, COLOR_GRAYSCALE_MIN));
    lnk_data.AMin = IM_MAX(IM_MIN(color_threshold[i][2], COLOR_A_MAX), COLOR_A_MIN);
    lnk_data.AMax = IM_MAX(IM_MIN(color_threshold[i][3], COLOR_A_MAX), COLOR_A_MIN);
    lnk_data.BMin = IM_MAX(IM_MIN(color_threshold[i][4], COLOR_B_MAX), COLOR_B_MIN);
    lnk_data.BMax = IM_MAX(IM_MIN(color_threshold[i][5], COLOR_B_MAX), COLOR_B_MIN);
    color_thresholds_list_lnk_data_t lnk_data_tmp;
    memcpy(&lnk_data_tmp, &lnk_data, sizeof(color_thresholds_list_lnk_data_t));
    lnk_data.LMin = IM_MIN(lnk_data_tmp.LMin, lnk_data_tmp.LMax);
    lnk_data.LMax = IM_MAX(lnk_data_tmp.LMin, lnk_data_tmp.LMax);
    lnk_data.AMin = IM_MIN(lnk_data_tmp.AMin, lnk_data_tmp.AMax);
    lnk_data.AMax = IM_MAX(lnk_data_tmp.AMin, lnk_data_tmp.AMax);
    lnk_data.BMin = IM_MIN(lnk_data_tmp.BMin, lnk_data_tmp.BMax);
    lnk_data.BMax = IM_MAX(lnk_data_tmp.BMin, lnk_data_tmp.BMax);
    list_push_back(thresholds, &lnk_data);
  }
}

uint8_t read_num = 0xFF;

void receiveEvent(int howMany) {
  uint8_t step = 0;
  while (0 < Wire.available()) { // loop through all but the last
    char rd = Wire.read(); // receive byte as a character
    switch(step)
    {
      case 0:
        if(rd < COLOR_NUM) //读取颜色识别数据，需装载数据(To retrieve color recognition data, the data loading process is required)
        {
          read_num = rd;
        }else if(rd == 0x11) //控制闪光灯亮灭，跳到第二步(Control the on and off of the LED and proceed to step 2)
        {
          step = 1;
          read_num = 0xFF;
        }
        break;
      case 1:
        if(rd > 0) //LED亮(LED on)
        {
          lightness = rd > 255 ? 255 : rd;
        }else//LED灭(LED off)
        {
          lightness = 0;
        }
        step = 0;
        break;
      default:
        step = 0;
        break;
    }
  }
}

void requestEvent() {
  
  if(read_num < COLOR_NUM)
  {
    Wire.slaveWrite((uint8_t*)color_info.dect_info[read_num] , 8);
    color_info.dect_info[read_num][0] = 0;
    color_info.dect_info[read_num][1] = 0;
    color_info.dect_info[read_num][2] = 0;
    color_info.dect_info[read_num][3] = 0;
    read_num = 0xFF;
  }
}
