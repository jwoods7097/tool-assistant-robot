/* 
 * 2024/02/21 hiwonder
 * 声波抓取例程
 * 超声波传感器检测到前方4cm左右的物体，会使用机械臂将物体夹取放置在左侧
 */
#include <FastLED.h> //导入LED库
#include <Servo.h> //导入舵机库
#include "tone.h" //音调库
#include "Ultrasound.h" //导入超声波库
#include "mini_servo.h"

const static uint16_t DOC5[] = { TONE_C5 };
const static uint16_t DOC6[] = { TONE_C6 };

/* 引脚定义 */
const static uint8_t servoPins[5] = { 7, 6, 5, 4, 3};//舵机引脚定义
const static uint8_t buzzerPin = 11;
const static uint8_t rgbPin = 13;

//RGB灯控制对象
static CRGB rgbs[1];
//动作组控制对象
HW_ACTION_CTL action_ctl;
//舵机控制对象
Servo servos[5];
// 创建超声波对象
Ultrasound ul;

const uint8_t limt_angles[5][2] = {{0,82},{0,180},{0,180},{0,180},{0,180}}; /* 各个关节角度的限制 */
static float servo_angles[5] = { 41 ,12 ,174 ,68 ,84 };  /* 舵机实际控制的角度数值 */

/* 蜂鸣器控制相关变量 */
static uint16_t tune_num = 0;
static uint32_t tune_beat = 10;
static uint16_t *tune;

static void servo_control(void);  //舵机控制
void play_tune(uint16_t *p, uint32_t beat, uint16_t len); /* 蜂鸣器控制接口 */
void tune_task(void); /* 蜂鸣器控制任务 */
static void ultrasound_task(void);  // 超声波任务

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // 设置串行端口读取数据的超时时间
  Serial.setTimeout(500);

  // 绑定舵机IO口
  for (int i = 0; i < 5; ++i) {
    servos[i].attach(servoPins[i],500,2500);
  }

  //RGB灯初始化并控制
  FastLED.addLeds<WS2812, rgbPin, GRB>(rgbs, 1);
  rgbs[0] = CRGB(0, 0, 100);
  FastLED.show();

  //读取偏差值
  action_ctl.read_offset();

  //蜂鸣器初始化并鸣响一声
  pinMode(buzzerPin, OUTPUT);
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);
  ul.Color(0,0,255,0,0,255);

  delay(1000);
  Serial.println("start");
}

void loop() {
  //超声波任务
  ultrasound_task();
  // 蜂鸣器鸣响任务
  tune_task();
  // 舵机控制
  servo_control();
  // 动作组运动任务
  action_ctl.action_task();

}

// 超声波任务
void ultrasound_task(void)
{
  static uint32_t last_ul_tick = 0;
  static uint8_t step = 0;
  static uint8_t act_num = 0;
  static uint32_t delay_count = 0;

  // 间隔100ms
  if (millis() - last_ul_tick < 100) {
    return;
  }
  last_ul_tick = millis();

  // 获取超声波距离
  int dis = ul.Filter();
  // Serial.println(dis); //打印距离

  switch(step)
  {
    case 0:
      if(dis > 35 && dis < 50) //若检测到物块
      {
        rgbs[0].r = 250;
        rgbs[0].g = 0;
        rgbs[0].b = 0;
        FastLED.show();
        play_tune(DOC6, 300u, 1u);
        act_num = 1;
        step++;
      }else //若未识别物块
      {
        rgbs[0].r = 0;
        rgbs[0].g = 0;
        rgbs[0].b = 0;
        FastLED.show();
      }
      break;
    case 1: //等待1s，物块放到指定位置
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

// 舵机控制任务
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
    // Serial.print(servo_angles[i]); 
    // Serial.print(" ");

  }
  // Serial.println();
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
