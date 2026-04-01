/* 
 * 2024/02/21 hiwonder
 * 声波测距例程(ultrasonic ranging)
 * 机械爪根据障碍物距离开合。距离越远，越张开；距离越近，越闭合。(The robotic gripper adjusts its opening and closing based on the distance to the obstacle. The farther the distance, the wider it opens; the closer the distance, the narrower it closes)
 */
#include <FastLED.h> //import LED library
#include <Servo.h> //import servo library
#include "tone.h" //tone library
#include "Ultrasound.h" //import ultrasonic library
#include <EEPROM.h>

#define EEPROM_START_FLAG "HIWONDER"
#define EEPROM_SERVO_OFFSET_START_ADDR 0u
#define EEPROM_SERVO_OFFSET_DATA_ADDR 16u
#define EEPROM_SERVO_OFFSET_LEN 6u

const static uint16_t DOC5[] = { TONE_C5 };
const static uint16_t DOC6[] = { TONE_C6 };

/* pins definition */
const static uint8_t servoPins[6] = { 7, 6, 5, 4, 3, 2 };
const static uint8_t buzzerPin = 11;
const static uint8_t rgbPin = 13;

//RGB灯控制对象(RGB LED control object)
static CRGB rgbs[1];
//servo control object
Servo servos[6];

//create ultrasonic object
Ultrasound ul;
 /* Angle values used in secondary development routines */
static uint8_t extended_func_angles[6] = { 73,10,161,57,90,90 };
/* Limitations on joint angles */
const uint8_t limt_angles[6][2] = {{0,82},{0,180},{0,180},{0,180},{0,180},{0,180}}; 
/* The angle value of the actual control of the servo */
static uint8_t servo_angles[6] = { 73,10,161,57,90,90 };  
static int8_t servo_offset[5];
static uint8_t servo_expect[5];
static uint8_t eeprom_read_buf[16];

/* 蜂鸣器控制相关变量(relevant variables for buzzer control) */
static uint16_t tune_num = 0;
static uint32_t tune_beat = 10;
static uint16_t *tune;

//servo control
static void servo_control(void);
//ultrasonic task
static void ultrasound_task(void);
static void read_offset(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // set the read data timeout of serial port
  Serial.setTimeout(500);

  // assign servo IO port
  for (int i = 0; i < 6; ++i) {
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

  read_offset();
  //read deviation angle 

  delay(1000);
  Serial.println("start");
}

void loop() {
  // 舵机控制(servo control)
  servo_control();

  //超声波任务(ultrasonic task)
  ultrasound_task();
}

// Ultrasonic task
void ultrasound_task(void)
{
  static uint32_t last_ul_tick = 0;
  // delay 100ms
  if (millis() - last_ul_tick < 100) {
    return;
  }
  last_ul_tick = millis();

  // Obtain ultrasonic distance
  int dis = ul.Filter();
  // Serial.println(dis); 

  // If it is greater than 200mm
  if(dis >= 200)
  {
    //Open your claws
    extended_func_angles[0] = 0;
    // RGB light blue
    rgbs[0].r = 0;
    rgbs[0].g = 0;
    rgbs[0].b = 255;
    FastLED.show();
    // Luminescent ultrasonic blue
    ul.Color(0,0,255,0,0,255);
  }else if(dis <= 50) //if it is less than 50mm
  {
    //close the claw
    extended_func_angles[0] = 82;
    // RGB red
    rgbs[0].r = 255;
    rgbs[0].g = 0;
    rgbs[0].b = 0;
    FastLED.show();
    // glowing ultrasonic red
    ul.Color(255,0,0,255,0,0);
  }else{
    /* the color changes according to the variation. The closer it gets,
     the larger the R (red) and the smaller the B (blue). Conversely, the farther away, the opposite occurs */
    int color[3] = {map(dis , 50 , 200 , 255 , 0),map(dis , 50 , 200 , 0 , 255),0};
    // The mechanical claw closes as it gets closer and opens as it gets farther away
    uint8_t angles = map(dis , 50 , 200 , 82 , 0);
    extended_func_angles[0] = angles;
    rgbs[0].r = color[0];
    rgbs[0].g = color[1];
    rgbs[0].b = color[2];
    FastLED.show();
    ul.Color(color[0],color[1],color[2],color[0],color[1],color[2]);
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
  // 读取偏差角度(read deviation angle) 
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

