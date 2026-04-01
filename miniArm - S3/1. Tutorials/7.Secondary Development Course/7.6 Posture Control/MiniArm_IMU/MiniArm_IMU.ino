/* 
 * 2024/02/21 hiwonder CuZn
 * 颜色夹取例程(color gripping program)
 * 用IIC连接ESP32Cam，检测蓝色、绿色，分别夹取到两边(Connect ESP32S3-Cam with I2C to detect blue and green objects and the robotic arm grasps them to both sides respectively)
 * 注意：每个不同的功能，ESP32Cam也要下载对应功能的ESP32Cam程序(Note: for different functions, ESP32S3-Cam needs to be downloaded with corresponding ESP32S3-Cam program)
 */
#include <FastLED.h> //import LED library
#include <Servo.h> //import servo library
#include "tone.h" //tone library
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
float radianX_last; //final obtained X-axis inclination
float radianY_last; //final obtained Y-axis inclination


const static uint16_t DOC5[] = { TONE_C5 };
const static uint16_t DOC6[] = { TONE_C6 };

/* pin definition */
const static uint8_t servoPins[5] = { 7, 6, 5, 4, 3};//servo pin definition
const static uint8_t buzzerPin = 11;
const static uint8_t rgbPin = 13;

//RGB light control object
static CRGB rgbs[1];
//action group control object
HW_ACTION_CTL action_ctl;
//servo control object
Servo servos[5];

/* limit for each joint angle */
const uint8_t limt_angles[5][2] = {{0,90},{0,180},{0,180},{0,180},{0,180}}; 
/* the angle value actually controlled by servo */
static float servo_angles[5] = { 80,15,175,65,90 };  

/* 蜂鸣器控制相关变量(variable related to the buzzer control) */
static uint16_t tune_num = 0;
static uint32_t tune_beat = 10;
static uint16_t *tune;


static void servo_control(void); /* servo control */
void play_tune(uint16_t *p, uint32_t beat, uint16_t len); /* buzzer control interface */
void tune_task(void); /* buzzer control task */
void user_task(void); /* user task */

void update_mpu6050(void); /*update inclination sensor data*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // set the read data timeout of serial port
  Serial.setTimeout(500);

  // assign servo IO port
  for (int i = 0; i < 5; ++i) {
    servos[i].attach(servoPins[i],500,2500);
  }

  //initialize the RGB light and control it
  FastLED.addLeds<WS2812, rgbPin, GRB>(rgbs, 1);
  rgbs[0] = CRGB(0, 0, 100);
  FastLED.show();

  //initialize buzzer and beep one time
  pinMode(buzzerPin, OUTPUT);
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);

  //read deviation data of servo
  action_ctl.read_offset();

  //MPU6050 configuration
  Wire.begin();
  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange(3); //set the range of angular velocity
  accelgyro.setFullScaleAccelRange(1); //set the range of acceleration
  delay(200);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //get current data for each axis to calibrate
  ax_offset = ax;  //X-axis acceleration calibration data
  ay_offset = ay;  //Y-axis acceleration calibration data
  az_offset = az - 8192;  //Z-axis acceleration calibration data
  gx_offset = gx; //X-axis angular velocity calibration data
  gy_offset = gy; //Y-axis angular velocity calibration data
  gz_offset = gz; //Z-axis angular velocity calibration data

  delay(1000);
  Serial.println("start");
}

void loop() {
  // 用户任务(user task)
  user_task();
  // 蜂鸣器鸣响任务(buzzer sound task)
  tune_task();
  // 舵机控制(servo control)
  servo_control();
  // 动作组运动任务(action group running task)
  action_ctl.action_task();

  update_mpu6050();
}

// user task
void user_task(void)
{
  static uint32_t last_tick = 0;
  static uint8_t step = 0;
  static uint8_t act_num = 0;
  static uint32_t delay_count = 0;
  int color = 0;

  // time interval 100ms
  if (millis() - last_tick < 100) {
    return;
  }
  last_tick = millis();

  switch(step)
  {
    case 0:
      if(radianX_last > 40) //grasp
      {
        rgbs[0].r = 100; //red
        rgbs[0].g = 0;
        rgbs[0].b = 0;
        FastLED.show();
        play_tune(DOC6, 300u, 1u);
        // action group 1 needs to be executed for sorting
        act_num = 1;
        step++;
      }else{ //if it is not detected
        rgbs[0].r = 0;
        rgbs[0].g = 0;
        rgbs[0].b = 0;
        FastLED.show();
      }
      break;
    case 1: //wait for 1s and place the wood block to the correct position. 
      delay_count++;
      if(delay_count > 5)
      {
        delay_count = 0;
        // run action group
        action_ctl.action_set(act_num);
        act_num = 0;
        step++;
      }
      break;
    case 2: //wait for resetting the action state
      if(action_ctl.action_state_get() == 0)
      {
        step++;
      }
      break;
    case 3:
      if(radianY_last > 50) //right
      {
        rgbs[0].r = 0; 
        rgbs[0].g = 100;
        rgbs[0].b = 0;
        FastLED.show();
        play_tune(DOC6, 300u, 1u);
        // action group 2 needs to be executed for sorting
        action_ctl.action_set(2);
        step++;
      }else if(radianY_last < -50) //left
      {
        rgbs[0].r = 0; 
        rgbs[0].g = 0;
        rgbs[0].b = 100;//blue
        FastLED.show();
        play_tune(DOC6, 300u, 1u);
        // action group 3 needs to be executed for sorting
        action_ctl.action_set(3);
        step++;
      }
      break;
    case 4: //wait for resetting the action state
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

//更新倾角传感器数据(update inclination sensor data)
void update_mpu6050(void)
{
  static uint32_t timer_u;
  if (timer_u < millis())
  {
    // put your main code here, to run repeatedly:
    timer_u = millis() + 20;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax0 = ((float)(ax)) * 0.3 + ax0 * 0.7;  //对读取到的值进行滤波(filter the read value)
    ay0 = ((float)(ay)) * 0.3 + ay0 * 0.7;
    az0 = ((float)(az)) * 0.3 + az0 * 0.7;
    ax1 = (ax0 - ax_offset) /  8192.0;  // 校正，并转为重力加速度的倍数(calibrating and converting it to multiples of gravitational acceleration)
    ay1 = (ay0 - ay_offset) /  8192.0;
    az1 = (az0 - az_offset) /  8192.0;

    gx0 = ((float)(gx)) * 0.3 + gx0 * 0.7;  //对读取到的角速度的值进行滤波(filter the read value of the angular velocity)
    gy0 = ((float)(gy)) * 0.3 + gy0 * 0.7;
    gz0 = ((float)(gz)) * 0.3 + gz0 * 0.7;
    gx1 = (gx0 - gx_offset);  //校正角速度(calibarate the angular velocity)
    gy1 = (gy0 - gy_offset);
    gz1 = (gz0 - gz_offset);


    //互补计算x轴倾角(computing the complementary angle of the X-axis)
    radianX = atan2(ay1, az1);
    radianX = radianX * 180.0 / 3.1415926;
    float radian_temp = (float)(gx1) / 16.4 * 0.02;
    radianX_last = 0.8 * (radianX_last + radian_temp) + (-radianX) * 0.2;

    //互补计算y轴倾角(computing the complementary angle of the Y-axis)
    radianY = atan2(ax1, az1);
    radianY = radianY * 180.0 / 3.1415926;
    radian_temp = (float)(gy1) / 16.4 * 0.01;
    radianY_last = 0.8 * (radianY_last + radian_temp) + (-radianY) * 0.2;
  }
}

// 舵机控制任务（不需修改）(servo control task (no need to modify))
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

// 蜂鸣器鸣响任务(buzzer sound task)
void tune_task(void) {
  static uint32_t l_tune_beat = 0;
  static uint32_t last_tick = 0;
  // 若未到定时时间 且 响的次数跟上一次的一样(if it is not yet the scheduled time and the ringing duration is the same as the previous time)
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

// 蜂鸣器控制接口(buzzer control task)
void play_tune(uint16_t *p, uint32_t beat, uint16_t len) {
  tune = p;
  tune_beat = beat;
  tune_num = len;
}

