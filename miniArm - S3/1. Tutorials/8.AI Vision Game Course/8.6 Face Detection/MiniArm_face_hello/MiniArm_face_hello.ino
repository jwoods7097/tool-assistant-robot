/* 
 * 2024/02/21 hiwonder CuZn
 * 人脸识别打招呼例程(face recognition program)
 * 人脸识别打招呼例程，用IIC连接ESP32Cam，检测到人脸，机械臂夹子开合打招呼(Face recognition program; connect to ESP32S3-Cam with I2C to detect face and control the robotic gripper to open and close for greeting)
 * 注意：每个不同的功能，ESP32Cam也要下载对应功能的ESP32Cam程序(Note: For each different function, the ESP32S3-Cam also needs to be downloaded with corresponding ESP32S3-Cam program)
 */
#include <FastLED.h> //导入LED库(import LED library)
#include <Servo.h> //导入舵机库(import servo library)
#include "hw_esp32cam_ctl.h" //导入ESP32Cam通讯库(import ESP32S3-Cam communication library)

/* 引脚定义(define pins) */
const static uint8_t servoPins[5] = { 7, 6, 5, 4, 3 };
const static uint8_t buzzerPin = 11;
const static uint8_t rgbPin = 13;

//RGB light control object
static CRGB rgbs[1];
//ESP32Cam communication object
HW_ESP32Cam hw_cam;
//servo control object
Servo servos[5];

/* the used angle values of secondary development example */
static uint8_t extended_func_angles[5] = { 90, 140, 140, 87, 88 }; 
/* limit of each joint’s angle */
const uint8_t limt_angles[5][2] = {{0,92},{0,180},{0,180},{25,180},{0,180} }; 
/* Actual angle values of servo control */
static uint8_t servo_angles[5] = { 90, 140, 140, 87, 88 };  

static void servo_control(void); /* 舵机控制(servo control) */
void espcam_task(void); /* esp32cam通讯任务(ESP32S3-Cam communication task) */

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // 设置串行端口读取数据的超时时间(set timeout for serial port reading data)
  Serial.setTimeout(500);

  // 绑定舵机IO口(bind servo IO port)
  for (int i = 0; i < 5; ++i) {
    servos[i].attach(servoPins[i]);
  }

  hw_cam.begin(); //初始化与ESP32Cam通讯接口(initialize ESP32S3-Cam communication interface)

  //RGB灯初始化并控制(initialize and control RGB LED)
  FastLED.addLeds<WS2812, rgbPin, GRB>(rgbs, 1);
  rgbs[0] = CRGB(0, 0, 100);
  FastLED.show();

  //蜂鸣器初始化并鸣响一声(initialize and control the buzzer to make a sound)
  pinMode(buzzerPin, OUTPUT);
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);

  delay(2000);
  Serial.println("start");
}


void loop() {
  // esp32cam通讯任务(ESP32S3-Cam communication task)
  espcam_task();
  // 舵机控制任务(servo control task)
  servo_control();
}


// esp32cam通讯任务(ESP32S3-Cam communication task)
void espcam_task(void)
{
  static uint32_t last_tick = 0;
  static uint8_t step = 0;
  static uint32_t delay_count = 0;

  // 时间间隔50ms(time interval 50ms)
  if (millis() - last_tick < 50) {
    return;
  }
  last_tick = millis();

  // 获取人脸数据(obtain face data)
  bool rt = hw_cam.faceDetect();
  
  switch(step)
  {
    case 0:
      if(rt) //If a face is detected
      {
        delay_count = 0;
        rgbs[0].r = 0;
        rgbs[0].g = 100;
        rgbs[0].b = 0;
        FastLED.show();
        // adjust to the next step
        step++;
      }
      break;
    case 1: //open the gripper
      extended_func_angles[0] = 40;
      // wait for action completion
      delay_count++;
      if(delay_count > 6)
      {
        delay_count = 0;
        step++;
      }
      break;
    case 2: //close the gripper
      extended_func_angles[0] = 82;
      // wait for action completion
      delay_count++;
      if(delay_count > 6)
      {
        delay_count = 0;
        step++;
      }
      break;
    case 3: //open the gripper
      extended_func_angles[0] = 40;
      // wait for action completion
      delay_count++;
      if(delay_count > 6)
      {
        delay_count = 0;
        step++;
      }
      break;
    case 4: //close the gripper
      extended_func_angles[0] = 82;
      // wait for action completion
      delay_count++;
      if(delay_count > 10)
      {
        rgbs[0].r = 0;
        rgbs[0].g = 0;
        rgbs[0].b = 100;
        FastLED.show();
        delay_count = 0;
        step++;
      }
      break;
    default:
      step = 0;
      break;
  }
}

// 舵机控制任务（不需修改）(servo control task (no need to modify)
void servo_control(void) {
  static uint32_t last_tick = 0;
  if (millis() - last_tick < 20) {
    return;
  }
  last_tick = millis();

  for (int i = 0; i < 5; ++i) {
    if(servo_angles[i] > extended_func_angles[i])
    {
      servo_angles[i] = servo_angles[i] * 0.8 + extended_func_angles[i] * 0.2;
      if(servo_angles[i] < extended_func_angles[i])
        servo_angles[i] = extended_func_angles[i];
    }else if(servo_angles[i] < extended_func_angles[i])
    {
      servo_angles[i] = servo_angles[i] * 0.8 + (extended_func_angles[i] * 0.2 + 1);
      if(servo_angles[i] > extended_func_angles[i])
        servo_angles[i] = extended_func_angles[i];
    }
    servo_angles[i] = servo_angles[i] < limt_angles[i][0] ? limt_angles[i][0] : servo_angles[i];
    servo_angles[i] = servo_angles[i] > limt_angles[i][1] ? limt_angles[i][1] : servo_angles[i];
    servos[i].write(i == 0 || i == 5 ? 180 - servo_angles[i] : servo_angles[i]);
  }
}
