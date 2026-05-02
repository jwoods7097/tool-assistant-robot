#include <Servo.h>

#define NUM_SERVOS 5

#define GRIPPER  0
#define WRIST    1
#define ELBOW    2
#define SHOULDER 3
#define BASE     4

Servo servos[NUM_SERVOS];

// Your confirmed pins
int pins[NUM_SERVOS] = {7, 6, 5, 4, 3};

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(pins[i]);
  }

  Serial.println("Folding arm...");

  // Step 1: Open gripper
  servos[GRIPPER].write(20);
  delay(500);

  // Step 2: Lift wrist up
  servos[WRIST].write(150);
  delay(500);

  // Step 3: Fold elbow fully
  servos[ELBOW].write(150);   // adjust if needed
  delay(800);

  // Step 4: Bring shoulder down
  servos[SHOULDER].write(160);  // adjust if needed
  delay(800);

  // Step 5: Rotate base to side (optional)
  servos[BASE].write(90);
  delay(500);

  // Step 6: Final compact wrist
  servos[WRIST].write(-20);
  delay(500);

  Serial.println("Folded.");
}

void loop() {}