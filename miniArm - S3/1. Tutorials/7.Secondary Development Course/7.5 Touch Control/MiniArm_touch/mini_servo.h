#ifndef _HW_ACTION_CTL_
#define _HW_ACTION_CTL_
#include "actions.h"
#include <EEPROM.h>

#define EEPROM_START_FLAG "HIWONDER"
#define EEPROM_SERVO_OFFSET_START_ADDR 1024
#define EEPROM_SERVO_OFFSET_DATA_ADDR 1024+16
#define EEPROM_SERVO_OFFSET_LEN 6u

class HW_ACTION_CTL{
  public:
  /* the angle used in secondary development routine */
    uint8_t extended_func_angles[5] = { 1,40,15,175,65,90 }; 
    //control and perform action group
    void action_set(int num);
    int action_state_get(void);
    void action_task(void);
    void read_offset(void);
    int8_t* get_offset(void);
    
  private:
    //action group control variables
    int action_num = 0;
    int8_t servo_offset[5];
    uint8_t eeprom_read_buf[16];
};

#endif //_HW_ACTION_CTL_
