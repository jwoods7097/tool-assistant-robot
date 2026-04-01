//动作组文件(action group file)
#include <Arduino.h>
#define action_count 2 //动作组数量(number of action groups)

static uint8_t action[action_count][20][6] = 
    {
      //动作组1(action group 1)
      {{1 ,90 ,0 ,130 ,125 ,90},{1 ,93 ,0 ,141 ,135 ,0},
      {1 ,93 ,0 ,141 ,135 ,174},{1 ,90 ,90 ,90 ,90 ,90}, {0,0,0,0,0,0}}, 
      //动作组2(action group 2)
      {{1 ,74 ,34 ,169 ,135 ,89},{1 ,74 ,34 ,169 ,135 ,89},
      {1 ,90 ,90 ,90 ,90 ,90},{0,0,0,0,0,0}},
    };