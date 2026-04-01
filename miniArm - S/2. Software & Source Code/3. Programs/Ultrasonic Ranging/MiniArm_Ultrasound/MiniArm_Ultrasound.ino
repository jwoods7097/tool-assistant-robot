/* 
 * 2024/02/21 hiwonder
 * 声波测距例程
 * 机械爪根据障碍物距离开合。距离越远，越张开；距离越近，越闭合。
 */
#include <FastLED.h> //导入LED库
#include <Servo.h> //导入舵机库
#include "tone.h" //音调库
#include "Ultrasound.h" //导入超声波库
#include <EEPROM.h>

#define EEPROM_START_FLAG "HIWONDER"
#define EEPROM_SERVO_OFFSET_START_ADDR 0u
#define EEPROM_SERVO_OFFSET_DATA_ADDR 16u
#define EEPROM_SERVO_OFFSET_LEN 6u

const static uint16_t DOC5[] = { TONE_C5 };
const static uint16_t DOC6[] = { TONE_C6 };

/* 引脚定义 */
const static uint8_t servoPins[6] = { 7, 6, 5, 4, 3, 2 };
const static uint8_t buzzerPin = 11;
const static uint8_t rgbPin = 13;

//RGB灯控制对象
static CRGB rgbs[1];
//舵机控制对象
Servo servos[6];

// 创建超声波对象
Ultrasound ul;

static uint8_t extended_func_angles[6] = { 73,10,161,57,90,90 }; /* 二次开发例程使用的角度数值 */
const uint8_t limt_angles[6][2] = {{0,82},{0,180},{0,180},{0,180},{0,180},{0,180}}; /* 各个关节角度的限制 */
static uint8_t servo_angles[6] = { 73,10,161,57,90,90 };  /* 舵机实际控制的角度数值 */
static int8_t servo_offset[5];
static uint8_t servo_expect[5];
static uint8_t eeprom_read_buf[16];

/* 蜂鸣器控制相关变量 */
static uint16_t tune_num = 0;
static uint32_t tune_beat = 10;
static uint16_t *tune;

//舵机控制
static void servo_control(void);
// 超声波任务
static void ultrasound_task(void);
static void read_offset(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // 设置串行端口读取数据的超时时间
  Serial.setTimeout(500);

  // 绑定舵机IO口
  for (int i = 0; i < 6; ++i) {
    servos[i].attach(servoPins[i],500,2500);
  }

  //RGB灯初始化并控制
  FastLED.addLeds<WS2812, rgbPin, GRB>(rgbs, 1);
  rgbs[0] = CRGB(0, 0, 100);
  FastLED.show();

  //蜂鸣器初始化并鸣响一声
  pinMode(buzzerPin, OUTPUT);
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);

  read_offset();

  delay(1000);
  Serial.println("start");
}

void loop() {
  // 舵机控制
  servo_control();

  //超声波任务
  ultrasound_task();
}

// 超声波任务
void ultrasound_task(void)
{
  static uint32_t last_ul_tick = 0;
  // 间隔100ms
  if (millis() - last_ul_tick < 100) {
    return;
  }
  last_ul_tick = millis();

  // 获取超声波距离
  int dis = ul.Filter();
  // Serial.println(dis); //打印距离

  // 若大于200mm
  if(dis >= 200)
  {
    //张开爪子
    extended_func_angles[0] = 0;
    // RGB灯蓝色
    rgbs[0].r = 0;
    rgbs[0].g = 0;
    rgbs[0].b = 255;
    FastLED.show();
    // 发光超声波蓝色
    ul.Color(0,0,255,0,0,255);
  }else if(dis <= 50) //若小于50mm
  {
    //闭合爪子
    extended_func_angles[0] = 82;
    // RGB红色
    rgbs[0].r = 255;
    rgbs[0].g = 0;
    rgbs[0].b = 0;
    FastLED.show();
    // 发光超声波红色
    ul.Color(255,0,0,255,0,0);
  }else{
    // 颜色根据距离变化而变化，越靠近R越大B越小，越远则反之
    int color[3] = {map(dis , 50 , 200 , 255 , 0),0,map(dis , 50 , 200 , 0 , 255)};

    uint8_t angles = map(dis , 50 , 200 , 82 , 0);
    extended_func_angles[0] = angles;
    rgbs[0].r = color[0];
    rgbs[0].g = color[1];
    rgbs[0].b = color[2];
    FastLED.show();
    ul.Color(color[0],color[1],color[2],color[0],color[1],color[2]);
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
    //打印偏差值
    // for (int i = 0; i < 5; ++i) {
    //   Serial.print(servo_offset[i]);
    //   Serial.print(" ");
    // }
    // Serial.println("");
  }
}

