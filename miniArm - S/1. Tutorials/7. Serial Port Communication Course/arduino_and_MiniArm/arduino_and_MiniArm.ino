#include "MiniArm_ctl.h"

MiniArm_ctl mi_ctl;

void setup() {
  Serial.begin(115200);
  mi_ctl.serial_begin();
}

void loop() {
  //set servos’ angles
  Serial.println("set angles");
  uint8_t angles[5] = {73,10,161,57,90};
  /* set respectively the bus servos’ angles to 73/10/0/57/90/180 digresses, 
    ranging from 0-180 */
  mi_ctl.set_angles(angles); 
  delay(1500);

  //set buzzer
  Serial.println("set buzzer");
  //set the buzzer frequency to 532 and the runtime to 1000 ms
  //the ranges of these two parameters are both 0-65535
  mi_ctl.set_buzzer(532,1000);
  delay(1500);

  //set RGB light
  Serial.println("set rgb");
  uint8_t rgb[3] = {0,255,0}; 
  //set RGB light to green (ranging from 0 to 255)
  mi_ctl.set_rgb(rgb);
  delay(1500);
  
  //read and print the servo angle of current status
  Serial.println("read angles");
  uint8_t read_a[5] = {0,0,0,0,0};
  if(mi_ctl.read_angles(read_a))
  {
    Serial.print("angles: ");
    for(int i=0;i<5;i++){
      Serial.print(read_a[i]);
      Serial.print(" ");
    }
  }
  delay(100);

}
