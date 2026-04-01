/* 
 * 2024/02/21 hiwonder CuZn
 * 颜色夹取例程
 * 用IIC连接ESP32Cam，检测蓝色、绿色，分别夹取到两边
 * 注意：每个不同的功能，ESP32Cam也要下载对应功能的ESP32Cam程序
 */
#include <FastLED.h> //导入LED库
#include <Servo.h> //导入舵机库
#include "hw_esp32cam_ctl.h" //导入ESP32Cam通讯库
#include "tone.h" //音调库
#include "mini_servo.h"

#define COLOR_1   1
#define COLOR_2   2

const static uint16_t DOC5[] = { TONE_C5 };
const static uint16_t DOC6[] = { TONE_C6 };

/* 引脚定义 */
const static uint8_t servoPins[5] = { 7, 6, 5, 4, 3};//舵机引脚定义
const static uint8_t buzzerPin = 11;
const static uint8_t rgbPin = 13;

//RGB灯控制对象
static CRGB rgbs[1];
//ESP32Cam通讯对象
HW_ESP32Cam hw_cam;
//动作组控制对象
HW_ACTION_CTL action_ctl;
//舵机控制对象
Servo servos[5];

const uint8_t limt_angles[5][2] = {{0,82},{0,180},{0,180},{0,180},{0,180}}; /* 各个关节角度的限制 */
static uint8_t servo_angles[5] = { 40,21,159,115,90 };  /* 舵机实际控制的角度数值 */

/* 蜂鸣器控制相关变量 */
static uint16_t tune_num = 0;
static uint32_t tune_beat = 10;
static uint16_t *tune;


static void servo_control(void); /* 舵机控制 */
void play_tune(uint16_t *p, uint32_t beat, uint16_t len); /* 蜂鸣器控制接口 */
void tune_task(void); /* 蜂鸣器控制任务 */
void espcam_task(void); /* esp32cam通讯任务 */

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

  action_ctl.read_offset();

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
  // 蜂鸣器鸣响任务
  tune_task();
  // 舵机控制
  servo_control();
  // 动作组运动任务
  action_ctl.action_task();
}

// esp32cam通讯任务
void espcam_task(void)
{
  static uint32_t last_tick = 0;
  static uint8_t step = 0;
  static uint8_t act_num = 0;
  static uint32_t delay_count = 0;
  int color = 0;

  // 时间间隔100ms
  if (millis() - last_tick < 100) {
    return;
  }
  last_tick = millis();
  
  switch(step)
  {
    case 0:
      color = hw_cam.colorDetect(); //获取颜色
      if(color == COLOR_1) //若识别到颜色1
      {
        rgbs[0].r = 0;
        rgbs[0].g = 250;
        rgbs[0].b = 0;
        FastLED.show();
        play_tune(DOC6, 300u, 1u);
        // 需要运行动作组1进行分拣
        act_num = 1;
        step++;
      }else if(color == COLOR_2) //若识别到颜色2
      {
        rgbs[0].r = 0;    
        rgbs[0].g = 0;      
        rgbs[0].b = 250;    
        FastLED.show();
        play_tune(DOC6, 300u, 1u);
        // 需要运行动作组2进行分拣
        act_num = 2;
        step++;
      }else{ //若没识别到
        rgbs[0].r = 0;
        rgbs[0].g = 0;
        rgbs[0].b = 0;
        FastLED.show();
      }
      break;
    case 1: //等待1s，木块放正
      delay_count++;
      if(delay_count > 10)
      {
        delay_count = 0;
        // 运行动作组
        action_ctl.action_set(act_num);
        act_num = 0;
        step++;
      }
      break;
    case 2: //等待动作状态清零
      if(action_ctl.action_state_get() == 0)
      {
        step = 0;
      }
      break;
    default:
      step = 0;
      break;
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
    if(servo_angles[i] > action_ctl.extended_func_angles[i])
    {
      servo_angles[i] = servo_angles[i] * 0.9 + action_ctl.extended_func_angles[i] * 0.1;
      if(servo_angles[i] < action_ctl.extended_func_angles[i])
        servo_angles[i] = action_ctl.extended_func_angles[i];
    }else if(servo_angles[i] < action_ctl.extended_func_angles[i])
    {
      servo_angles[i] = servo_angles[i] * 0.9 + (action_ctl.extended_func_angles[i] * 0.1 + 1);
      if(servo_angles[i] > action_ctl.extended_func_angles[i])
        servo_angles[i] = action_ctl.extended_func_angles[i];
    }

    servo_angles[i] = servo_angles[i] < limt_angles[i][0] ? limt_angles[i][0] : servo_angles[i];
    servo_angles[i] = servo_angles[i] > limt_angles[i][1] ? limt_angles[i][1] : servo_angles[i];
    servos[i].write(i == 0 || i == 5 ? 180 - servo_angles[i] : servo_angles[i]);
  }
}

// 蜂鸣器鸣响任务
void tune_task(void) {
  static uint32_t l_tune_beat = 0;
  static uint32_t last_tick = 0;
  // 若未到定时时间 且 响的次数跟上一次的一样
  if (millis() - last_tick < l_tune_beat && tune_beat == l_tune_beat) {
    return;
  }
  l_tune_beat = tune_beat;
  last_tick = millis();
  if (tune_num > 0) {
    tune_num -= 1;
    tone(buzzerPin, *tune++);
  } else {
    noTone(buzzerPin);
    tune_beat = 10;
    l_tune_beat = 10;
  }
}

// 蜂鸣器控制接口
void play_tune(uint16_t *p, uint32_t beat, uint16_t len) {
  tune = p;
  tune_beat = beat;
  tune_num = len;
}
