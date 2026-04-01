#ifndef MINIARM_CTL_H
#define MINIARM_CTL_H

#include <Arduino.h>
#include "SoftwareSerial.h" //软串口库(soft serial port library)

#define rxPin 6      //Arduino与MiniArm通讯串口(communication serial port of Arduino and miniArm)
#define txPin 7      

//************* 数据结构定义 begin(define data structure "begin") *************

/*
 * 0xAA 0x66 | func | data_len | data | check
 */

#define CONST_STARTBYTE1 0xAAu
#define CONST_STARTBYTE2 0x66u


//帧功能号枚举(frame function number enumeration)
enum PACKET_FUNCTION {
  FUNC_SET_SERVO = 0x01, //舵机控制(servo control)
  FUNC_SET_BUZZER = 0x02, //蜂鸣器控制(buzzer control)
  FUNC_SET_RGB = 0x03, //RGB控制(RGB control)
  FUNC_READ_ANGLE = 0x11 //Angle读取(read Angle)
};

//************* 数据结构定义 end(define data structure "end") ***************

class MiniArm_ctl {
  public:
    //构造函数(structure function)
    uhand_ctl(void);

    //串口开启函数(function for enabling the serial port)
    void serial_begin(void);

    //设置角度(set angle)
    void set_angles(uint8_t* angles);

    //设置蜂鸣器(set buzzer)
    void set_buzzer(int buz,int time);
    //设置RGB灯(set RGB LED)
    void set_rgb(uint8_t* rgb);
    //读取角度(read angle)
    bool read_angles(uint8_t* angles);

  private:
    bool rec_handle(uint8_t* res , uint8_t func);
};


#endif //MINIARM_CTL_H
