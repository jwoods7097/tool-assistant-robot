/* 
 * 2024/02/21 hiwonder CuZn
 * 颜色夹取例程
 * 用IIC连接ESP32Cam，检测颜色并追踪
 * 注意：每个不同的功能，ESP32Cam也要下载对应功能的ESP32Cam程序
 */
#include <FastLED.h> //导入LED库
#include <Servo.h> //导入舵机库
#include "hw_esp32cam_ctl.h" //导入ESP32Cam通讯库
#include <EEPROM.h>

#define EEPROM_START_FLAG "HIWONDER"
#define EEPROM_SERVO_OFFSET_START_ADDR 0u
#define EEPROM_SERVO_OFFSET_DATA_ADDR 16u
#define EEPROM_SERVO_OFFSET_LEN 6u

/* 引脚定义 */
const static uint8_t servoPins[5] = { 7, 6, 5, 4, 3 };
const static uint8_t buzzerPin = 11;
const static uint8_t rgbPin = 13;

//RGB灯控制对象
static CRGB rgbs[1];
//ESP32Cam通讯对象
HW_ESP32Cam hw_cam;
//舵机控制对象
Servo servos[5];

static uint8_t extended_func_angles[5] = { 90,115,180,120,90 }; /* 二次开发例程使用的角度数值 */
const uint8_t limt_angles[5][2] = {{0,90},{0,180},{0,180},{25,180},{0,180} }; /* 各个关节角度的限制 */
static uint8_t servo_angles[5] = { 90,115,180,120,90 };  /* 舵机实际控制的角度数值 */
static int8_t servo_offset[5];
static uint8_t servo_expect[5];
static uint8_t eeprom_read_buf[16];

static void servo_control(void); /* 舵机控制 */
void espcam_task(void); /* esp32cam通讯任务 */
static void read_offset(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // 设置串行端口读取数据的超时时间
  Serial.setTimeout(500);

  // 绑定舵机IO口
  for (int i = 0; i < 5; ++i) {
    servos[i].attach(servoPins[i],500,2500);
  }

  hw_cam.begin(); //初始化与ESP32Cam通讯接口
  hw_cam.set_led(10); // 控制ESP32Cam补光灯亮度 [亮度值(0~255)]

  //RGB灯初始化并控制
  FastLED.addLeds<WS2812, rgbPin, GRB>(rgbs, 1);
  rgbs[0] = CRGB(0, 0, 100);
  FastLED.show();

  read_offset();

  //蜂鸣器初始化并鸣响一声
  pinMode(buzzerPin, OUTPUT);
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);

  delay(2000);
  Serial.println("start");
}

void loop() {
  // esp32cam通讯任务
  espcam_task();
  // 舵机控制
  servo_control();
}

// esp32cam通讯任务
void espcam_task(void)
{
  static uint32_t last_tick = 0;
  uint16_t color_info[4];

  // 时间间隔
  if (millis() - last_tick < 75) {
    return;
  }
  last_tick = millis();
  
  if(hw_cam.color_position(color_info)) //若识别到颜色
  {
    uint16_t num = color_info[0] + color_info[2]/2; //计算颜色块中心
    uint16_t angle = map(num , 0 , 320 , 60 , 120); //映射到对应的舵机角度
    extended_func_angles[4] = angle; //控制舵机运动追踪
  }
}


// 舵机控制任务（不需修改）
void servo_control(void) {
  static uint32_t last_tick = 0;
  if (millis() - last_tick < 20) {
    return;
  }
  last_tick = millis();
  for (int i = 0; i < 5; ++i) {
    servo_expect[i] = extended_func_angles[i] + servo_offset[i];
    if(servo_angles[i] > servo_expect[i])
    {
      servo_angles[i] = servo_angles[i] * 0.9 + servo_expect[i] * 0.1;
      if(servo_angles[i] < servo_expect[i])
        servo_angles[i] = servo_expect[i];
    }else if(servo_angles[i] < servo_expect[i])
    {
      servo_angles[i] = servo_angles[i] * 0.9 + (servo_expect[i] * 0.1 + 1);
      if(servo_angles[i] > servo_expect[i])
        servo_angles[i] = servo_expect[i];
    }

    servo_angles[i] = servo_angles[i] < limt_angles[i][0] ? limt_angles[i][0] : servo_angles[i];
    servo_angles[i] = servo_angles[i] > limt_angles[i][1] ? limt_angles[i][1] : servo_angles[i];
    servos[i].write(i == 0 || i == 5 ? 180 - servo_angles[i] : servo_angles[i]);
  }
}

void read_offset(void){
  // 读取偏差角度 
  for (int i = 0; i < 16; ++i) {
    eeprom_read_buf[i] = EEPROM.read(EEPROM_SERVO_OFFSET_START_ADDR + i);
  }
  if (strcmp(eeprom_read_buf, EEPROM_START_FLAG) == 0) {
    memset(eeprom_read_buf,  0 , sizeof(eeprom_read_buf));
    Serial.println("read offset");
    for (int i = 0; i < 5; ++i) {
      eeprom_read_buf[i] = EEPROM.read(EEPROM_SERVO_OFFSET_DATA_ADDR + i);
    }
    memcpy(servo_offset , eeprom_read_buf , 6);
    /*for (int i = 0; i < 5; ++i) {
      Serial.print(servo_offset[i]);
      Serial.print(" ");
    }
    Serial.println("");*/
  }
}


