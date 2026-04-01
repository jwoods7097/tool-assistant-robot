// #include "ESPMax.h"
// #include "Buzzer.h"
// #include "ESP32PWMServo.h"
// #include "SuctionNozzle.h"
#include "PC_rec.h"


PC_REC pc_rec;

void setup() {
  Serial.begin(115200);
  pc_rec.begin();
  
  Serial.println("start...");
}

void loop() {
  pc_rec.rec_data();
  pc_rec.servo_control();
  delay(10);
}
