#include "Arduino.h"
#include "HardwareSerial.h"
#include "Servo.h"
#include "PC_rec.h"
#include <FastLED.h> //导入LED库

static SoftwareSerial SerialM(rxPin, txPin);  //实例化软串口
void SetServo(int i,int pul);
void servo_control(void);
void Buzzer_control(int buz,int btime);
/* 引脚定义 */
const static uint8_t servoPins[5] = { 7, 6, 5, 4, 3};//舵机引脚定义

const static uint8_t Buzzer_Pin = 11; // 蜂鸣器引脚定义
const static uint8_t rgbPin = 13; //RGB灯引脚定义

//RGB灯控制对象
static CRGB rgbs[1];

//舵机控制对象
Servo servos[5];

const uint8_t limt_angles[5][2] = {{0,82},{0,180},{0,180},{25,180},{0,180}}; /* 各个关节角度的限制 */
static uint8_t servo_angles[5] = { 90,90,90,90,90};  /* 舵机实际转动角度数值 */
static uint8_t extended_func_angles[5] = { 90,90,90,90,90}; //动作数据

/* CRC校验 */
static uint16_t checksum_crc8(const uint8_t *buf, uint16_t len)
{
    uint8_t check = 0;
    while (len--) {
        check = check + (*buf++);
    }
    check = ~check;
    return ((uint16_t) check) & 0x00FF;
}

PC_REC::PC_REC(void)
{
  
}

void PC_REC::begin(void)
{
  SerialM.begin(9600);
  //舵机初始化  
  for(int i= 0;i<5;i++){
    servos[i].attach(servoPins[i]);
  }
  //蜂鸣器PWM初始化
  pinMode(Buzzer_Pin,OUTPUT);

  //RGB灯初始化
  FastLED.addLeds<WS2812, rgbPin, GRB>(rgbs, 1);
  rgbs[0] = CRGB(255, 255, 255);
  FastLED.show();
}

void PC_REC::rec_data(void)
{
  //读取数据
  uint32_t len = SerialM.available();
  while(len--)
  {
    int rd = SerialM.read();
    pk_ctl.data[pk_ctl.index_tail] = (char)rd;
    pk_ctl.index_tail++;
    if(BUFFER_SIZE <= pk_ctl.index_tail)
    {
      pk_ctl.index_tail = 0;
    }
    if(pk_ctl.index_tail == pk_ctl.index_head)
    {
      pk_ctl.index_head++;
      if(BUFFER_SIZE <= pk_ctl.index_head)
      {
        pk_ctl.index_head = 0;
      }
    }else{
      pk_ctl.len++;
    }
  }

  uint8_t crc = 0;
  //解析数据
  while(pk_ctl.len > 0)
  {
    switch(pk_ctl.state)
    {
      case STATE_STARTBYTE1: /* 处理帧头标记1 */
        pk_ctl.state = CONST_STARTBYTE1 == pk_ctl.data[pk_ctl.index_head] ? STATE_STARTBYTE2 : STATE_STARTBYTE1;
        break;
      case STATE_STARTBYTE2: /* 处理帧头标记2 */
        pk_ctl.state = CONST_STARTBYTE2 == pk_ctl.data[pk_ctl.index_head] ? STATE_FUNCTION : STATE_STARTBYTE1;
        break;
      case STATE_FUNCTION: /* 处理帧功能号 */
        pk_ctl.state = STATE_LENGTH;
        if(FUNC_READ_ANGLE != pk_ctl.data[pk_ctl.index_head])
          if(FUNC_SET_BUZZER != pk_ctl.data[pk_ctl.index_head])
            if(FUNC_SET_RGB != pk_ctl.data[pk_ctl.index_head])
              if(FUNC_SET_SERVO != pk_ctl.data[pk_ctl.index_head])
              {
                pk_ctl.state = STATE_STARTBYTE1;
              }
        if(STATE_LENGTH == pk_ctl.state) {
            pk_ctl.frame.function = pk_ctl.data[pk_ctl.index_head];
        }
        break;
      case STATE_LENGTH: /* 处理帧数据长度 */
        if(pk_ctl.data[pk_ctl.index_head] >= DATA_SIZE) //若（包含具体信息）信息数据长度>DATA_SIZE,则有问题
        {
          pk_ctl.state = STATE_STARTBYTE1;
          continue;
        }else{
          pk_ctl.frame.data_length = pk_ctl.data[pk_ctl.index_head];
          pk_ctl.state = (0 == pk_ctl.frame.data_length) ? STATE_CHECKSUM : STATE_DATA;
          pk_ctl.data_index = 0;
          break;
        }
      case STATE_DATA: /* 处理帧数据 */
        pk_ctl.frame.data[pk_ctl.data_index] = pk_ctl.data[pk_ctl.index_head];
        ++pk_ctl.data_index;
        if(pk_ctl.data_index >= pk_ctl.frame.data_length) {
            pk_ctl.state = STATE_CHECKSUM;
            pk_ctl.frame.data[pk_ctl.data_index] = '\0';
        }
        break;
      case STATE_CHECKSUM: /* 处理校验值 */
        pk_ctl.frame.checksum = pk_ctl.data[pk_ctl.index_head];
        crc = checksum_crc8((uint8_t*)&pk_ctl.frame.function, pk_ctl.frame.data_length + 2);
        Serial.print("crc:");
        Serial.print(crc);
        if(crc == pk_ctl.frame.checksum) { /* 校验失败, 跳过执行 */
            Serial.println(" OK");
            deal_command(&pk_ctl.frame); //处理数据
        }else{
          Serial.print("not:");
          Serial.println(pk_ctl.frame.checksum);
        }
        memset(&pk_ctl.frame, 0, sizeof(struct PacketRawFrame)); //清除
        pk_ctl.state = STATE_STARTBYTE1;
        break;
      default:
        pk_ctl.state = STATE_STARTBYTE1;
        break;
    }
    if(pk_ctl.index_head != pk_ctl.index_tail)
        pk_ctl.index_head++;
    if(BUFFER_SIZE <= pk_ctl.index_head)
        pk_ctl.index_head = 0;
    pk_ctl.len--;
  }
}

void PC_REC::deal_command(struct PacketRawFrame* ctl_com)
{
  uint16_t len = ctl_com->data_length;
  Serial.println(len);
  switch(ctl_com->function)
  {
    case FUNC_SET_SERVO: //设置舵机 0x01
      {
        Serial.println("Set SERVO");
        Servo_Ctl_Data msg;
        if(len == 5)
        {
          memcpy(&msg , &ctl_com->data , sizeof(msg));
          Serial.print("Angle: ");
          for(int i = 0;i<5;i++){
            Serial.print(msg.pul[i]);
            Serial.print(" ");
            SetServo(i,msg.pul[i]);
          }
          Serial.println("");
          
        }else{
          Serial.print("error");
        }
        Serial.println("");
      }
      break;

    case FUNC_READ_ANGLE: //读取角度 0x11
      {
        Serial.println("Read Angle:");
        read_Angle_Data msg;
        uint8_t angles[5];
        for(int i=0;i<5;i++){
          angles[i] = (uint8_t)extended_func_angles[i];
        }
        uint8_t send_data[20] = {0xAA , 0x66 , 0x11 , 0x05};
        memcpy(&send_data[4] , angles , 5);
        send_data[9] = checksum_crc8(&send_data[2] , 8);
        SerialM.write(send_data , 10);
        Serial.print("Angle: ");
        for(int i=0;i<5;i++){
          Serial.print(angles[i]);
          Serial.print(" ");
        }
        Serial.println("");
      }
      break;

    case FUNC_SET_BUZZER://设置蜂鸣器 0x02
      {
        Serial.println("set BUZZER");
        Buzzer_Ctl_DATA msg;
        if(len == 4)
        {
          memcpy(&msg , &ctl_com->data , sizeof(msg));
          Serial.print("BUZZER ");
          Serial.print(msg.buz);
          Serial.print(" ");
          Serial.print(msg.btime);
          Serial.print("ms");
          
          Buzzer_control(msg.buz,msg.btime);

        }else{
          Serial.print("error");
        }
        Serial.println("");
      }
      break;
    
    case FUNC_SET_RGB://设置RGB灯 0x03
      {
        Serial.println("set RGB");
        RGB_Ctl_DATA msg;
        if(len == 3)
        {
          memcpy(&msg , &ctl_com->data , sizeof(msg));
          Serial.print("RGB ");
          for(int i=0;i<3;i++){
            Serial.print(msg.rgb[i]);
            Serial.print(" ");            
          }
          rgbs[0].r = msg.rgb[0];
          rgbs[0].g = msg.rgb[1];
          rgbs[0].b = msg.rgb[2];
          FastLED.show();
          Serial.println("");
          delay(5);
          tone(Buzzer_Pin,1047,100);

        }else{
          Serial.print("error");
        }
        Serial.println("");
      }
      break;
    default:
      Serial.println("no ctl");
      break;
  }
}
void SetServo(int i,int pul){
  extended_func_angles[i] = pul;
}
void Buzzer_control(int buz,int btime){
  delay(5);
  tone(Buzzer_Pin,buz,btime);
}

void PC_REC::servo_control(void) {
  static uint32_t last_tick = 0;
  if (millis() - last_tick < 20) {
    return;
  }
  last_tick = millis();

  for (int i = 0; i < 5; ++i) {
    if(servo_angles[i] > extended_func_angles[i])
    {
      servo_angles[i] = servo_angles[i] * 0.9 + extended_func_angles[i] * 0.1;
      if(servo_angles[i] < extended_func_angles[i])
        servo_angles[i] = extended_func_angles[i];
    }else if(servo_angles[i] < extended_func_angles[i])
    {
      servo_angles[i] = servo_angles[i] * 0.9 + (extended_func_angles[i] * 0.1 + 1);
      if(servo_angles[i] > extended_func_angles[i])
        servo_angles[i] = extended_func_angles[i];
    }
    
    servo_angles[i] = servo_angles[i] < limt_angles[i][0] ? limt_angles[i][0] : servo_angles[i];
    servo_angles[i] = servo_angles[i] > limt_angles[i][1] ? limt_angles[i][1] : servo_angles[i];
    servos[i].write(i == 0 ? 180 - servo_angles[i] : servo_angles[i]);
  }
}


