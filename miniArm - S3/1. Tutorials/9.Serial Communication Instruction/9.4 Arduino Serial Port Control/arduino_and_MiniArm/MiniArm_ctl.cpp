#include "HardwareSerial.h"
#include "MiniArm_ctl.h"

static SoftwareSerial SerialM(rxPin, txPin);  //实例化软串口(instantiate soft serial port)


/* CRC校验(CRC checksum) */
static uint16_t checksum_crc8(const uint8_t *buf, uint16_t len)
{
    uint8_t check = 0;
    while (len--) {
        check = check + (*buf++);
    }
    check = ~check;
    return ((uint16_t) check) & 0x00FF;
}

MiniArm_ctl::MiniArm_ctl(void)
{
  
}


void MiniArm_ctl::serial_begin(void)
{
  SerialM.begin(9600);
}




//设置角度 0~180 (set angle to 0 to 180)
void MiniArm_ctl::set_angles(uint8_t* angles)
{
  uint8_t pul[5];
  uint8_t msg[20] = {CONST_STARTBYTE1 , CONST_STARTBYTE2};
  msg[2] = FUNC_SET_SERVO;
  msg[3] = 5;
  pul[0] = angles[0] > 180 ? 180 : angles[0];
  pul[1] = angles[1] > 180 ? 180 : angles[1];
  pul[2] = angles[2] > 180 ? 180 : angles[2];
  pul[3] = angles[3] > 180 ? 180 : angles[3];
  pul[4] = angles[4] > 180 ? 180 : angles[4];
  
  memcpy(&msg[4] , pul , 5);

  msg[9] = checksum_crc8((uint8_t*)&msg[2] , 8);
  SerialM.write((char*)&msg , 10);

}

//设置蜂鸣器(set buzzer)
void MiniArm_ctl::set_buzzer(int buz , int time)
{
  uint8_t msg[20] = {CONST_STARTBYTE1 , CONST_STARTBYTE2};
  msg[2] = FUNC_SET_BUZZER;
  msg[3] = 4;
  msg[4] = buz & 0x00ff;
  msg[5] = (buz>>8)&0x00ff;

  msg[6] = time & 0x00ff;
  msg[7] = (time>>8)&0x00ff;
  msg[8] = checksum_crc8((uint8_t*)&msg[2] , 7);
  SerialM.write((char*)&msg , 9);
}

//设置RGB灯(set RGB LED)
void MiniArm_ctl::set_rgb(uint8_t* rgb){
  uint8_t msg[20] = {CONST_STARTBYTE1 , CONST_STARTBYTE2};
  msg[2] = FUNC_SET_RGB;
  msg[3] = 3;
  
  memcpy(&msg[4] , rgb , 3);

  msg[7] = checksum_crc8((uint8_t*)&msg[2] , 6);
  SerialM.write((char*)&msg , 8);
}

//读取舵机角度(read servo angle)


bool MiniArm_ctl::rec_handle(uint8_t* res , uint8_t func)
{
  int len = SerialM.available();
  // 限制读取的数据长度(limit the data length to be read)
  len = len > 30 ? 30 : len;
  uint8_t step = 0;
  uint8_t data[6] , index = 0;
  while(len--)
  {
    int rd = SerialM.read();
    switch(step)
    {
      case 0:
        if(rd == 0xAA)
        {
          step++;
        }
        break;
      case 1:
        if(rd == 0x66)
        {
          step++;
        }else{
          step = 0;
        }
        break;
      case 2:
        if(rd == func)
        {
          data[index++] = rd;
          step++;
        }else{
          step = 0;
        }
        break;
      case 3: //接收的数据长度必须为5个字节(the received data length must be five bytes)
        if(rd == 5)
        {
          data[index++] = rd;
          step++;
        }else{
          step = 0;
        }
        break;
      case 4:
        data[index++] = rd;
        if(index >= 7)
        {
          step++;
        }
        break;
      case 5:
        if(checksum_crc8(data , 7) == rd)
        {
          memcpy(res , &data[2] , 5);
          return true;
        }else{
          return false;
        }
        break;
      default:
        step = 0;
        break;
    }
  }
  return false;
}


//读取角度(read angle)
bool MiniArm_ctl::read_angles(uint8_t* angles)
{
  uint8_t msg[10] = {CONST_STARTBYTE1 , CONST_STARTBYTE2};
  msg[2] = FUNC_READ_ANGLE;
  msg[3] = 0;
  msg[4] = checksum_crc8((uint8_t*)&msg[2] , 2);
  while(SerialM.available())
  {
    SerialM.read();
  }
  SerialM.write((char*)&msg , 5);

  uint16_t count = 5;
  delay(300);
  uint8_t res[5];
  if(rec_handle((uint8_t*)res , 0x11))
  {
    angles[0] = res[0];
    angles[1] = res[1];
    angles[2] = res[2];
    angles[3] = res[3];
    angles[4] = res[4];
    
    return true;
  }
  return false;
}
