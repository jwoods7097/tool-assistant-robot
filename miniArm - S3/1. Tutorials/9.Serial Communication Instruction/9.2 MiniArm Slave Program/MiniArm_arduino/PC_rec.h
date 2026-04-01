#ifndef PC_REC_H
#define PC_REC_H

#include <Arduino.h>
#include "SoftwareSerial.h" //软串口库(soft serial port library)

#define rxPin 10      //Arduino与MaxArm通讯串口(communication serial port for Arduino and MaxArm)
#define txPin 12      

#define CONST_STARTBYTE1 0xAAu
#define CONST_STARTBYTE2 0x77u
const uint16_t BUFFER_SIZE = 90;
const uint16_t DATA_SIZE = 20; //有效数据不应该超过 DATA_SIZE(the valid data should not exceeds DATA_SIZE)

//帧功能号枚举(frame function number enumeration)
enum PACKET_FUNCTION {
  FUNC_SET_SERVO = 0x01, //舵机控制(servo control)
  FUNC_SET_BUZZER = 0x02, //蜂鸣器控制(buzzer control)
  FUNC_SET_RGB = 0x03, //RGB控制(RGB control)
  FUNC_READ_ANGLE = 0x11 //Angle读取(read Angle)
};

//解析器状态机状态枚举(state enumeration for parser state machine)
enum PacketControllerState {
    STATE_STARTBYTE1, /**< 正在寻找帧头标记1(searching for frame header mark 1) */
    STATE_STARTBYTE2, /**< 正在寻找帧头标记2(searching for frame header mark 2) */
    STATE_FUNCTION, /**< 正在处理帧功能号(processing frame function number) */
    STATE_LENGTH, /**< 正在处理帧长度(processing frame length) */
    STATE_DATA, /**< 正在处理帧数据(processing frame data) */
    STATE_CHECKSUM, /** 正在处理数据校验(processing data calibration) */
};

#pragma pack(1)

//命令内容参数(command content parameter)
typedef
struct PacketRawFrame {
    uint8_t start_byte1;
    uint8_t start_byte2;
    uint8_t function;   //功能号(function number)
    uint8_t data_length;
    uint8_t data[DATA_SIZE];  //数据(data)
    uint8_t checksum;
}packet_st;

//设置舵机(set servo)
typedef
struct Servo_Ctl_Data{
  uint8_t pul[6];
}Servo_Ctl_Data;


//读Angle(read Angle)
typedef
struct READ_ANGLE_DATA{
  uint8_t pos[6];
}read_Angle_Data;

//设置蜂鸣器(set buzzer)
typedef
struct Buzzer_Ctl_DATA{
  uint16_t buz;
  uint16_t btime;
}Buzzer_Ctl_DATA;

//设置RGB(set RGB)
typedef
struct RGB_Ctl_DATA{
  uint8_t rgb[3];
}RGB_Ctl_DATA;

#pragma pack(0)


/**
 * @brief 协议解析器(protocol parser)
 * @details 协议解析器, 存储了解析器的工作状态、状态机状态、缓冲存储区等(the protocol parser has stored the working state, state machine's state, buffer storage, and more of the parser)
 */
struct PacketController {
    //解析协议的相关变量(variables related to parse protocol)
    enum PacketControllerState state;        //解析器状态机当前状态(the current state of parser state machine)
    struct PacketRawFrame frame;             //解析器正在处理的帧(processing frame of the parser)
    int data_index;   //解析时需要用到的数据下标(the data index to be used during parsing)

    //缓冲区相关变量(buffer-related variables)
    uint8_t len;
    uint8_t index_head;
    uint8_t index_tail;
    uint8_t data[BUFFER_SIZE];
};



class PC_REC{
  public:
    PC_REC(void);

    void begin(void);
    void rec_data(void);

    void deal_command(struct PacketRawFrame* ctl_com);
    void servo_control(void);
  private:
    // 读取成功标志位(flag bit for read success)
    bool read_succeed;
    // 接收解析控制器(controller for receiving parse)
    struct PacketController pk_ctl;
    // 解析结果储存变量(parse result storage variable)
    // packet_st pk_result;
};

#endif //PC_REC_H
