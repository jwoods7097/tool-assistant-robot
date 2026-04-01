#ifndef _HW_ACTION_CTL_
#define _HW_ACTION_CTL_
#include "actions.h"
#include <EEPROM.h>

#define EEPROM_START_FLAG "HIWONDER"
#define EEPROM_SERVO_OFFSET_START_ADDR 0u
#define EEPROM_SERVO_OFFSET_DATA_ADDR 16u
#define EEPROM_SERVO_OFFSET_LEN 6u

class HW_ACTION_CTL{
  public:
    uint8_t extended_func_angles[5] = {40,15,175,65,90 }; /* 二次开发例程使用的角度数值 */
    //控制执行动作组
    void action_set(int num);
    int action_state_get(void);
    void action_task(void);
    void read_offset(void);
    int8_t* get_offset(void);
    
  private:
    //动作组控制变量
    int action_num = 0;
    int8_t servo_offset[5];
    uint8_t eeprom_read_buf[16];
};

#endif //_HW_ACTION_CTL_
