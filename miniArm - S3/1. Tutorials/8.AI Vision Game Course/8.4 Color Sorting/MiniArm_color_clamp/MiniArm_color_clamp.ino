/* 
 * 2024/02/21 hiwonder CuZn
 * 颜色夹取例程(color sorting program)
 * 用IIC连接ESP32Cam，检测蓝色、绿色，分别夹取到两边(Connect ESP32S3-Cam with I2C to detect blue and red objects, grasping them to their respective sides)
 * 注意：每个不同的功能，ESP32Cam也要下载对应功能的ESP32Cam程序(Note: For each different function, the ESP32S3-Cam also needs to be downloaded with corresponding ESP32S3-Cam program)
 */
#include <FastLED.h> //导入LED库(import LED library)
#include <Servo.h> //导入舵机库(import servo library)
#include "hw_esp32cam_ctl.h" //导入ESP32Cam通讯库(import ESP32S3-Cam communication library)
#include "tone.h" //音调库(tone library)
#include "mini_servo.h"

#define COLOR_1   1
#define COLOR_2   2

const static uint16_t DOC5[] = { TONE_C5 };
const static uint16_t DOC6[] = { TONE_C6 };

/* 引脚定义(define pins) */
const static uint8_t servoPins[5] = { 7, 6, 5, 4, 3};//舵机引脚定义(define servo pins) 
const static uint8_t buzzerPin = 11;
const static uint8_t rgbPin = 13;

//RGB light control object
static CRGB rgbs[1];
//ESP32Cam communication object
HW_ESP32Cam hw_cam;
//action group control object
HW_ACTION_CTL action_ctl;
//servo control object
Servo servos[5];

/* The limit of each joint’s angle */
const uint8_t limt_angles[5][2] = {{0,82},{0,180},{0,180},{0,180},{0,180}}; 
/* The actual angle value of the servo control */
static uint8_t servo_angles[5] = { 40,21,159,115,90 };  

/* 蜂鸣器控制相关变量(variable related to the buzzer control) */
static uint16_t tune_num = 0;
static uint32_t tune_beat = 10;
static uint16_t *tune;


static void servo_control(void); /* servo control */
void play_tune(uint16_t *p, uint32_t beat, uint16_t len); /* buzzer control connector */
void tune_task(void); /* buzzer control task */
void espcam_task(void); /* esp32cam communication task */

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // set the timeout for reading data from the serial port
  Serial.setTimeout(500);

  
  // Assign the servo IO port
  for (int i = 0; i < 5; ++i) {
    servos[i].attach(servoPins[i]);
  }

  hw_cam.begin(); //initialize the ESP32Cam communication connector
  hw_cam.set_led(10); // control ESP32Cam fill light brightness [brightness value (0~255)]

  //Initialize and control the RGB light
  FastLED.addLeds<WS2812, rgbPin, GRB>(rgbs, 1);
  rgbs[0] = CRGB(0, 0, 100);
  FastLED.show();

  action_ctl.read_offset();

  //蜂鸣器初始化并鸣响一声(initialize and control the buzzer to make a sound)
  pinMode(buzzerPin, OUTPUT);
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);

  delay(2000);
  Serial.println("start");
}

void loop() {
  // esp32cam communication task
  espcam_task();
  // buzzer sound task
  tune_task();
  // servo control
  servo_control();
  // action group running task
  action_ctl.action_task();
}

// esp32cam通讯任务(ESP32S3-Cam communication task)
void espcam_task(void)
{
  static uint32_t last_tick = 0;
  static uint8_t step = 0;
  static uint8_t act_num = 0;
  static uint32_t delay_count = 0;
  int color = 0;

  // time intervals 100ms
  if (millis() - last_tick < 100) {
    return;
  }
  last_tick = millis();
  
  switch(step)
  {
    case 0:
      color = hw_cam.colorDetect(); //obtain color
      if(color == COLOR_1) //if the color 1 is recognized
      {
        rgbs[0].r = 0;
        rgbs[0].g = 250;
        rgbs[0].b = 0;
        FastLED.show();
        play_tune(DOC6, 300u, 1u);
        // to run action group 1 for color sorting
        act_num = 1;
        step++;
      }else if(color == COLOR_2) //IF the color 2 is recognized
      {
        rgbs[0].r = 0;    
        rgbs[0].g = 0;      
        rgbs[0].b = 250;    
        FastLED.show();
        play_tune(DOC6, 300u, 1u);
        // Need to run the action group 2 and sort it.
        act_num = 2;
        step++;
      }else{ //If no target color is detected
        rgbs[0].r = 0;
        rgbs[0].g = 0;
        rgbs[0].b = 0;
        FastLED.show();
      }
      break;
    case 1: //wait for 1s to place the block well
      delay_count++;
      if(delay_count > 10)
      {
        delay_count = 0;
        // run action group
        action_ctl.action_set(act_num);
        act_num = 0;
        step++;
      }
      break;
    case 2: //wait for the action status to reset
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

// 舵机控制任务（不需修改）(servo contorl task (no need to modify))
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
  // if it is not timed and it rings the same number as last time
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

// 蜂鸣器控制接口(buzzer control interface)
void play_tune(uint16_t *p, uint32_t beat, uint16_t len) {
  tune = p;
  tune_beat = beat;
  tune_num = len;
}
