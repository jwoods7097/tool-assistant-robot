#include <Arduino.h>
#include <Servo.h>

// Servo objects
Servo gripperServo;   // D7
Servo shoulderServo;  // D4
Servo elbowServo;     // D5
Servo baseServo;      // D3
Servo wristServo;     // D6

// Current angles
int gripperAngle  = 90;
int shoulderAngle = 90;
int elbowAngle    = 90;
int baseAngle     = 90;
int wristAngle    = 90;

// Smooth movement function
void moveServoSmooth(Servo &s, int &current, int target, int stepDelay = 10) {
  if (current < target) {
    for (int a = current; a <= target; a++) {
      s.write(a);
      delay(stepDelay);
    }
  } else {
    for (int a = current; a >= target; a--) {
      s.write(a);
      delay(stepDelay);
    }
  }
  current = target;
}

// Basic poses
void homePose() {
  moveServoSmooth(gripperServo,  gripperAngle,  90);
  moveServoSmooth(shoulderServo, shoulderAngle, 90);
  moveServoSmooth(elbowServo,    elbowAngle,    90);
  moveServoSmooth(baseServo,     baseAngle,     90);
  moveServoSmooth(wristServo,    wristAngle,    90);
  Serial.println("HOME");
}

void openGripper() {
  moveServoSmooth(gripperServo, gripperAngle, 90);
  Serial.println("GRIP OPEN");
}

void closeGripper() {
  moveServoSmooth(gripperServo, gripperAngle, 20);
  Serial.println("GRIP CLOSE");
}

// Setup
void setup() {
  Serial.begin(115200);

  gripperServo.attach(7);
  shoulderServo.attach(4);
  elbowServo.attach(5);
  baseServo.attach(3);
  wristServo.attach(6);

  delay(1000);

  Serial.println("=== ARM TEST READY ===");
  homePose();
}

// Loop (manual testing via Serial)
void loop() {

  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {

      case 'h':
        homePose();
        break;

      case 'o':
        gripperAngle += 10;
        if (gripperAngle > 170) gripperAngle = 170;
        gripperServo.write(gripperAngle);
        Serial.println("GRIP OPEN");
        break;

      case 'c':
        gripperAngle -= 10;
        if (gripperAngle < 10) gripperAngle = 10;
        gripperServo.write(gripperAngle);
        Serial.println("GRIP CLOSE");
        break;

      case 'a':  // base left
        baseAngle += 10;
        if (baseAngle > 170) baseAngle = 170;
        baseServo.write(baseAngle);
        Serial.println("BASE LEFT");
        break;

      case 'd':  // base right
        baseAngle -= 10;
        if (baseAngle < 10) baseAngle = 10;
        baseServo.write(baseAngle);
        Serial.println("BASE RIGHT");
        break;

      case 'w':  // shoulder up
        shoulderAngle -= 10;
        if (shoulderAngle > 170) shoulderAngle = 170;
        shoulderServo.write(shoulderAngle);
        Serial.println("SHOULDER UP");
        break;

      case 's':  // shoulder down
        shoulderAngle += 10;
        if (shoulderAngle < 10) shoulderAngle = 10;
        shoulderServo.write(shoulderAngle);
        Serial.println("SHOULDER DOWN");
        break;

      case 'e':  // elbow up
        elbowAngle -= 10;
        if (elbowAngle > 170) elbowAngle = 170;
        elbowServo.write(elbowAngle);
        Serial.println("ELBOW UP");
        break;

      case 'q':  // elbow down
        elbowAngle += 10;
        if (elbowAngle < 10) elbowAngle = 10;
        elbowServo.write(elbowAngle);
        Serial.println("ELBOW DOWN");
        break;

      case 'r':  // wrist up
        wristAngle -= 10;
        if (wristAngle > 170) wristAngle = 170;
        wristServo.write(wristAngle);
        Serial.println("WRIST UP");
        break;

      case 'f':  // wrist down
        wristAngle += 10;
        if (wristAngle < 10) wristAngle = 10;
        wristServo.write(wristAngle);
        Serial.println("WRIST DOWN");
        break;
    }
  }
}