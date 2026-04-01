/* 
 * 2024/02/21 hiwonder
 * 触摸控制例程(touch control program)
 * 触摸传感器检测到触摸，机械臂夹取物块；当再次检测到触摸，将物块放置在左侧。(When the touch sensor detects the touch, the robotic arm will grasp the object; when it detects touch again, the robotic arm will place the object to the left side)
 */
#include <FastLED.h> //import LED library
#include <Servo.h> //import servo library
#include "tone.h" //tone library
#include "mini_servo.h"

const static uint16_t DOC5[] = { TONE_C5 };
const static uint16_t DOC6[] = { TONE_C6 };

/* pin definition */
const static uint8_t servoPins[5] = { 7, 6, 5, 4, 3};//servo pin definition
const static uint8_t touch = 12;
const static uint8_t buzzerPin = 11;
const static uint8_t rgbPin = 13;

//RGB light control object
static CRGB rgbs[1];
//action group control object
HW_ACTION_CTL action_ctl;
//servo control object
Servo servos[5];

/* limit for each joint angle */
const uint8_t limt_angles[5][2] = {{0,82},{0,180},{0,180},{0,180},{0,180}}; 
/* the angle value actually controlled by serv */
static float servo_angles[5] = { 1,40,15,175,65,90 };  

/* 蜂鸣器控制相关变量(variable related to the buzzer control) */
static uint16_t tune_num = 0;
static uint32_t tune_beat = 10;
static uint16_t *tune;

static void servo_control(void); /* servo control */
void play_tune(uint16_t *p, uint32_t beat, uint16_t len); /* buzzer control interface */
void tune_task(void); /* buzzer control task */
void touch_task(void); /* touch sensor detection task */

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // set the elapsed time for reading data from the serial port
  Serial.setTimeout(500);

  // assign servo IO port
  for (int i = 0; i < 5; ++i) {
    servos[i].attach(servoPins[i],500,2500);
  }

  //initialize and control RGB light
  FastLED.addLeds<WS2812, rgbPin, GRB>(rgbs, 1);
  rgbs[0] = CRGB(0, 0, 100);
  FastLED.show();

  //initialize touch sensor
  pinMode(touch, INPUT);
  //read deviation data of servo
  action_ctl.read_offset();

  //initialize buzzer and beep one time
  pinMode(buzzerPin, OUTPUT);
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);

  delay(2000);
  Serial.println("start");
}

void loop() {
    // 触摸传感器检测任务(touch sensor detection task)
    touch_task();
    // 蜂鸣器鸣响任务(buzzer sound task)
    tune_task();
    // 舵机控制(servo control)
    servo_control();
    // 动作组运动任务(action group running task)
    action_ctl.action_task();
}

// touch sensor detection task 
void touch_task(void)
{
  static uint32_t last_tick = 0;
  static uint8_t step = 0;
  static uint8_t act_num = 0;
  static uint32_t delay_count = 0;
  static uint8_t turn;
  //current stage. 0 represents grasping stage; 1 represents placing stage
  static uint8_t stage = 0; 
  

  // time interval is 100ms
  if (millis() - last_tick < 100) {
    return;
  }
  last_tick = millis();

  static uint8_t count = 0;
  turn = digitalRead(touch);
  
  switch(step)
  {
    case 0:
      if(turn == 0) //if a button press is detected
      {
        rgbs[0].r = 250;
        rgbs[0].g = 0;
        rgbs[0].b = 0;
        FastLED.show();
        play_tune(DOC6, 300u, 1u);
        if(stage)//placing stage
        { 
          act_num = 2;
          stage = 0;
        }
        else    //grasping stage
        { 
          act_num = 1;
          stage = 1;
        }
        count = 0;
        step++;
        }
      }else //if touch is not detected
      {
        rgbs[0].r = 0;
        rgbs[0].g = 0;
        rgbs[0].b = 0;
        FastLED.show();
      }
      break;
    case 1: //wait for 1s and place the wood block to the specified position
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
    case 2: //wait for resetting the action state
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

// servo control task
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
  // 若未到定时时间 且 响的次数跟上一次的一样(If it is not yet the scheduled time and the number of rings is the same as the previous time)
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

