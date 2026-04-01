/* 
 * 2024/02/21 hiwonder CuZn
 * 颜色夹取例程
 * 用IIC连接ESP32Cam，检测蓝色、绿色，分别夹取到两边
 * 注意：每个不同的功能，ESP32Cam也要下载对应功能的ESP32Cam程序
 */
#include <FastLED.h> //导入LED库
#include <Servo.h> //导入舵机库
#include "tone.h" //音调库
#include <MPU6050.h>
#include "mini_servo.h"

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float ax0, ay0, az0;
float gx0, gy0, gz0;
float ax1, ay1, az1;
float gx1, gy1, gz1;

float dx;
float dz;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
float radianX;
float radianY;
float radianZ;
float radianX_last; //最终获得的X轴倾角
float radianY_last; //最终获得的Y轴倾角


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

const uint8_t limt_angles[5][2] = {{0,90},{0,180},{0,180},{0,180},{0,180}}; /* 各个关节角度的限制 */
static float servo_angles[5] = { 80,15,175,65,90 };  /* 舵机实际控制的角度数值 */

/* 蜂鸣器控制相关变量 */
static uint16_t tune_num = 0;
static uint32_t tune_beat = 10;
static uint16_t *tune;


static void servo_control(void); /* 舵机控制 */
void play_tune(uint16_t *p, uint32_t beat, uint16_t len); /* 蜂鸣器控制接口 */
void tune_task(void); /* 蜂鸣器控制任务 */
void user_task(void); /* 用户任务 */

void update_mpu6050(void); /*更新倾角传感器数据*/

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

  //蜂鸣器初始化并鸣响一声
  pinMode(buzzerPin, OUTPUT);
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);

  //读取舵机偏差数据
  action_ctl.read_offset();

  //MPU6050 配置
  Wire.begin();
  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange(3); //设定角速度量程
  accelgyro.setFullScaleAccelRange(1); //设定加速度量程
  delay(200);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //获取当前各轴数据以校准
  ax_offset = ax;  //X轴加速度校准数据
  ay_offset = ay;  //Y轴加速度校准数据
  az_offset = az - 8192;  //Z轴加速度校准数据
  gx_offset = gx; //X轴角速度校准数据
  gy_offset = gy; //Y轴角速度校准数据
  gz_offset = gz; //Z轴教书的校准数据

  delay(1000);
  Serial.println("start");
}

void loop() {
  // 用户任务
  user_task();
  // 蜂鸣器鸣响任务
  tune_task();
  // 舵机控制
  servo_control();
  // 动作组运动任务
  action_ctl.action_task();

  update_mpu6050();
}

// 用户任务
void user_task(void)
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
      if(radianX_last > 40) //抓取
      {
        rgbs[0].r = 100; //红色
        rgbs[0].g = 0;
        rgbs[0].b = 0;
        FastLED.show();
        play_tune(DOC6, 300u, 1u);
        // 需要运行动作组1进行分拣
        act_num = 1;
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
      if(delay_count > 5)
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
        step++;
      }
      break;
    case 3:
      if(radianY_last > 50) //右边
      {
        rgbs[0].r = 0; 
        rgbs[0].g = 100;
        rgbs[0].b = 0;
        FastLED.show();
        play_tune(DOC6, 300u, 1u);
        // 需要运行动作组2进行分拣
        action_ctl.action_set(2);
        step++;
      }else if(radianY_last < -50) //左边
      {
        rgbs[0].r = 0; 
        rgbs[0].g = 0;
        rgbs[0].b = 100;//蓝色
        FastLED.show();
        play_tune(DOC6, 300u, 1u);
        // 需要运行动作组3进行分拣
        action_ctl.action_set(3);
        step++;
      }
      break;
    case 4: //等待动作状态清零
      if(action_ctl.action_state_get() == 0)
      {
        FastLED.clear();
        step = 0;
      }
      break;
    default:
      step = 0;
      break;
  }
}

//更新倾角传感器数据
void update_mpu6050(void)
{
  static uint32_t timer_u;
  if (timer_u < millis())
  {
    // put your main code here, to run repeatedly:
    timer_u = millis() + 20;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax0 = ((float)(ax)) * 0.3 + ax0 * 0.7;  //对读取到的值进行滤波
    ay0 = ((float)(ay)) * 0.3 + ay0 * 0.7;
    az0 = ((float)(az)) * 0.3 + az0 * 0.7;
    ax1 = (ax0 - ax_offset) /  8192.0;  // 校正，并转为重力加速度的倍数
    ay1 = (ay0 - ay_offset) /  8192.0;
    az1 = (az0 - az_offset) /  8192.0;

    gx0 = ((float)(gx)) * 0.3 + gx0 * 0.7;  //对读取到的角速度的值进行滤波
    gy0 = ((float)(gy)) * 0.3 + gy0 * 0.7;
    gz0 = ((float)(gz)) * 0.3 + gz0 * 0.7;
    gx1 = (gx0 - gx_offset);  //校正角速度
    gy1 = (gy0 - gy_offset);
    gz1 = (gz0 - gz_offset);


    //互补计算x轴倾角
    radianX = atan2(ay1, az1);
    radianX = radianX * 180.0 / 3.1415926;
    float radian_temp = (float)(gx1) / 16.4 * 0.02;
    radianX_last = 0.8 * (radianX_last + radian_temp) + (-radianX) * 0.2;

    //互补计算y轴倾角
    radianY = atan2(ax1, az1);
    radianY = radianY * 180.0 / 3.1415926;
    radian_temp = (float)(gy1) / 16.4 * 0.01;
    radianY_last = 0.8 * (radianY_last + radian_temp) + (-radianY) * 0.2;
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

